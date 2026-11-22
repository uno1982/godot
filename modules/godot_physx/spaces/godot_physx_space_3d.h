/**************************************************************************/
/*  godot_physx_space_3d.h                                                */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "core/math/math_funcs.h"
#include "core/math/vector3.h"
#include "core/templates/hash_set.h"
#include "core/templates/local_vector.h"
#include "core/templates/rid.h"
#include "core/variant/dictionary.h"
#include "servers/physics_3d/physics_server_3d.h"

namespace physx {
class PxScene;
class PxDefaultCpuDispatcher;
class PxPhysics;
class PxMaterial;
class PxRigidActor;
class PxCudaContextManager;
} //namespace physx

class GodotPhysXBody3D;
class GodotPhysXArea3D;
class GodotPhysXParticleFluid3D;
class GodotPhysXCloth3D;
class GodotPhysXSoftBody3D;
class GodotPhysXDirectSpaceState3D;

class GodotPhysXSpace3D {
	RID self;
	physx::PxPhysics *px_physics = nullptr;
	physx::PxScene *px_scene = nullptr;
	physx::PxCudaContextManager *px_cuda = nullptr; // null unless a GPU build with a usable CUDA device
	physx::PxMaterial *default_material = nullptr;
	GodotPhysXDirectSpaceState3D *direct_state = nullptr;

	Vector3 gravity = Vector3(0, -9.8, 0);
	real_t gravity_magnitude = 9.8;
	Vector3 gravity_direction = Vector3(0, -1, 0);
	real_t last_step = 0.0;

	// Godot expresses sleep as separate linear/angular velocity thresholds;
	// PhysX uses one mass-normalized kinetic-energy threshold. Defaults match the
	// other Godot physics backends.
	real_t sleep_threshold_linear = 0.1;
	real_t sleep_threshold_angular = Math::deg_to_rad(8.0);
	real_t time_before_sleep = 0.5;

	HashSet<GodotPhysXBody3D *> bodies;
	HashSet<GodotPhysXBody3D *> awake_bodies; // active as of the last step()
	HashSet<GodotPhysXBody3D *> contact_reporters; // bodies with contact monitoring on
	HashSet<GodotPhysXArea3D *> areas;
	// The areas each step has to look at -- usually few of many (most areas are passive: monitorable, monitoring
	// off): with a monitor callback (their events, area-area polls) and with a gravity / damping / wind override.
	HashSet<GodotPhysXArea3D *> monitoring_areas;
	HashSet<GodotPhysXArea3D *> override_areas;
	HashSet<GodotPhysXParticleFluid3D *> fluids;
	HashSet<GodotPhysXCloth3D *> cloths;
	HashSet<GodotPhysXSoftBody3D *> soft_bodies;
	LocalVector<GodotPhysXBody3D *> sync_bodies; // to notify in the next call_queries()
	HashSet<GodotPhysXBody3D *> force_integrators; // bodies driving their own _integrate_forces
	HashSet<GodotPhysXBody3D *> constant_force_bodies; // bodies with a nonzero constant force/torque
	HashSet<GodotPhysXBody3D *> separation_ray_bodies; // dynamic bodies with a SeparationRayShape3D
	HashSet<GodotPhysXBody3D *> gravity_scaled_bodies; // rigid bodies whose gravity_scale is neither 0 nor 1
	HashSet<GodotPhysXBody3D *> motor_driven_bodies; // turned by a running joint motor: kept awake
	// physics/3d/default_linear_damp / default_angular_damp, which the scene
	// tree sends to the space RID (the space's default area), and the bodies
	// an Area3D currently overrides that area-level damping for.
	real_t default_linear_damp = 0.0;
	real_t default_angular_damp = 0.0;
	HashSet<GodotPhysXBody3D *> area_damped_bodies;

public:
	// What the last step cost, by stage, for PhysX.space_get_step_stats(). Timed with a handful of clock reads per step;
	// the actor counts are bumped where a body or area actor enters or leaves the scene.
	struct StepStats {
		uint64_t step_usec = 0; // the whole of step(), the stages below together
		uint64_t pre_solve_usec = 0; // force integration callbacks, constant forces, separation rays, motor wakes
		uint64_t area_overrides_usec = 0; // area gravity / damping / wind onto bodies
		uint64_t area_overlaps_usec = 0; // area-vs-area overlap polls
		uint64_t simulate_usec = 0; // PxScene::simulate(): flushing scene changes, starting the step's tasks
		uint64_t fetch_results_usec = 0; // waiting for the solve, contact reports
		uint64_t sync_back_usec = 0; // moved bodies, fluids, cloth, soft bodies pulled back
		uint64_t callbacks_usec = 0; // call_queries() after the step: body state and area events (nodes and scripts)
		uint32_t actors_added = 0; // body / area actors that entered the scene for this step
		uint32_t actors_removed = 0; // ... and left it
		uint32_t synced_bodies = 0; // bodies whose new state went to their node
	};

private:
	StepStats step_stats;
	uint32_t pending_actors_added = 0;
	uint32_t pending_actors_removed = 0;

public:
	void count_actor_added() { pending_actors_added++; }
	void count_actor_removed() { pending_actors_removed++; }
	const StepStats &get_step_stats() const { return step_stats; }
	// The last step's stats plus PhysX's own simulation statistics; call on the physics thread, between steps.
	Dictionary get_step_stats_dictionary() const;

	// The bodies PhysX reported moving in the last step (eENABLE_ACTIVE_ACTORS): what the module's GI refreshes.
	const HashSet<GodotPhysXBody3D *> &get_awake_bodies() const { return awake_bodies; }

	// A body's PxMaterial is released here, not on the spot: the release waits for the start of the next step (just
	// before PxScene::simulate). PhysX hands a released material's handle to the very next createMaterial(), but a GPU
	// scene learns of the removal only at its next simulate -- a shape made in between maps the reused handle onto the
	// OLD material's GPU slot, which the sync then drops, so the new body collides with the dead material's friction /
	// bounce for good (a frictionless body freed-and-made in one frame slid at friction 1). Released right before the
	// simulate, nothing can be created between the removal and its sync.
	static void defer_material_release(physx::PxMaterial *p_material);
	static void flush_material_releases();
	// The same for an actor about to be released whose shapes hold the last references to their materials (a PhysX
	// vehicle's chassis / wheel materials): each is kept alive (one reference each) until the next step.
	static void hold_actor_materials(physx::PxRigidActor *p_actor);

	void set_self(const RID &p_self) { self = p_self; }
	RID get_self() const { return self; }

	physx::PxPhysics *get_px_physics() const { return px_physics; }
	physx::PxScene *get_px_scene() const { return px_scene; }
	physx::PxCudaContextManager *get_px_cuda() const { return px_cuda; }
	physx::PxMaterial *get_default_material() const { return default_material; }
	real_t get_last_step() const { return last_step; }
	Vector3 get_gravity() const { return gravity; }

	// Mass-normalized kinetic-energy threshold below which PhysX may sleep a body.
	real_t get_sleep_energy_threshold() const {
		return 0.5 * (sleep_threshold_linear * sleep_threshold_linear + sleep_threshold_angular * sleep_threshold_angular);
	}
	real_t get_time_before_sleep() const { return time_before_sleep; }

	void set_gravity_vector(const Vector3 &p_gravity);
	void set_gravity_magnitude(real_t p_magnitude);
	void set_gravity_direction(const Vector3 &p_direction);
	void set_param(PhysicsServer3D::SpaceParameter p_param, real_t p_value);
	real_t get_param(PhysicsServer3D::SpaceParameter p_param) const;

	void register_body(GodotPhysXBody3D *p_body) { bodies.insert(p_body); }
	void unregister_body(GodotPhysXBody3D *p_body) {
		bodies.erase(p_body);
		awake_bodies.erase(p_body);
		contact_reporters.erase(p_body);
		sync_bodies.erase(p_body);
		force_integrators.erase(p_body);
		constant_force_bodies.erase(p_body);
		separation_ray_bodies.erase(p_body);
		gravity_scaled_bodies.erase(p_body);
		motor_driven_bodies.erase(p_body);
		area_damped_bodies.erase(p_body);
	}

	real_t get_default_linear_damp() const { return default_linear_damp; }
	real_t get_default_angular_damp() const { return default_angular_damp; }
	void set_default_damping(real_t p_linear, real_t p_angular);

	void set_body_separation_rays(GodotPhysXBody3D *p_body, bool p_enabled) {
		if (p_enabled) {
			separation_ray_bodies.insert(p_body);
		} else {
			separation_ray_bodies.erase(p_body);
		}
	}

	void set_body_motor_driven(GodotPhysXBody3D *p_body, bool p_enabled) {
		if (p_enabled) {
			motor_driven_bodies.insert(p_body);
		} else {
			motor_driven_bodies.erase(p_body);
		}
	}

	void set_body_gravity_scaled(GodotPhysXBody3D *p_body, bool p_enabled) {
		if (p_enabled) {
			gravity_scaled_bodies.insert(p_body);
		} else {
			gravity_scaled_bodies.erase(p_body);
		}
	}

	void set_body_constant_forces(GodotPhysXBody3D *p_body, bool p_enabled) {
		if (p_enabled) {
			constant_force_bodies.insert(p_body);
		} else {
			constant_force_bodies.erase(p_body);
		}
	}

	void set_body_force_integrator(GodotPhysXBody3D *p_body, bool p_enabled) {
		if (p_enabled) {
			force_integrators.insert(p_body);
		} else {
			force_integrators.erase(p_body);
		}
	}

	void set_body_contact_reporting(GodotPhysXBody3D *p_body, bool p_enabled) {
		if (p_enabled) {
			contact_reporters.insert(p_body);
		} else {
			contact_reporters.erase(p_body);
		}
	}

	void register_area(GodotPhysXArea3D *p_area) {
		areas.insert(p_area);
		update_area_role(p_area);
	}
	// After an area's callbacks or overrides changed.
	void update_area_role(GodotPhysXArea3D *p_area);
	bool is_override_area(GodotPhysXArea3D *p_area) const { return override_areas.has(p_area); }
	void unregister_area(GodotPhysXArea3D *p_area);

	void register_fluid(GodotPhysXParticleFluid3D *p_fluid) { fluids.insert(p_fluid); }
	void unregister_fluid(GodotPhysXParticleFluid3D *p_fluid) { fluids.erase(p_fluid); }
	void register_cloth(GodotPhysXCloth3D *p_cloth) { cloths.insert(p_cloth); }
	void unregister_cloth(GodotPhysXCloth3D *p_cloth) { cloths.erase(p_cloth); }
	void register_soft_body(GodotPhysXSoftBody3D *p_sb) { soft_bodies.insert(p_sb); }
	void unregister_soft_body(GodotPhysXSoftBody3D *p_sb) { soft_bodies.erase(p_sb); }

	// Drop a body from every area's overlap set (body leaving the simulation).
	void body_removed_from_areas(GodotPhysXBody3D *p_body);
	// A body's actor is destroyed (rebuilt / removed): every area takes its overlaps back (see GodotPhysXArea3D).
	void body_actor_gone(GodotPhysXBody3D *p_body);
	// One of a body's shapes is detached from its live actor (a shape swapped / disabled in place): the same, for
	// the overlaps of that shape only.
	void body_shape_gone(GodotPhysXBody3D *p_body, int p_shape);

	void step(real_t p_step);
	void call_queries();

	bool test_body_motion(GodotPhysXBody3D *p_body, const PhysicsServer3D::MotionParameters &p_params, PhysicsServer3D::MotionResult *r_result);

	GodotPhysXDirectSpaceState3D *get_direct_state();

	bool is_gpu() const { return gpu_enabled; }

	GodotPhysXSpace3D(physx::PxPhysics *p_physics, physx::PxDefaultCpuDispatcher *p_dispatcher, physx::PxCudaContextManager *p_cuda);
	~GodotPhysXSpace3D();

private:
	bool gpu_enabled = false;

	// Apply each area's gravity/damp/wind overrides to the bodies it contains.
	void _apply_area_overrides();
	// Push dynamic bodies standing on separation rays out of what the rays touch.
	void _apply_separation_rays(real_t p_step);
	// Manual area-vs-area overlap poll (PhysX reports no trigger-trigger event
	// to drive this from instead -- see GodotPhysXArea3D's header comment).
	void _detect_area_overlaps();
};

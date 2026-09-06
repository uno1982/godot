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
#include "servers/physics_3d/physics_server_3d.h"

namespace physx {
class PxScene;
class PxDefaultCpuDispatcher;
class PxPhysics;
class PxMaterial;
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
	HashSet<GodotPhysXParticleFluid3D *> fluids;
	HashSet<GodotPhysXCloth3D *> cloths;
	HashSet<GodotPhysXSoftBody3D *> soft_bodies;
	LocalVector<GodotPhysXBody3D *> sync_bodies; // to notify in the next call_queries()

public:
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
	}

	void set_body_contact_reporting(GodotPhysXBody3D *p_body, bool p_enabled) {
		if (p_enabled) {
			contact_reporters.insert(p_body);
		} else {
			contact_reporters.erase(p_body);
		}
	}

	void register_area(GodotPhysXArea3D *p_area) { areas.insert(p_area); }
	void unregister_area(GodotPhysXArea3D *p_area) { areas.erase(p_area); }

	void register_fluid(GodotPhysXParticleFluid3D *p_fluid) { fluids.insert(p_fluid); }
	void unregister_fluid(GodotPhysXParticleFluid3D *p_fluid) { fluids.erase(p_fluid); }
	void register_cloth(GodotPhysXCloth3D *p_cloth) { cloths.insert(p_cloth); }
	void unregister_cloth(GodotPhysXCloth3D *p_cloth) { cloths.erase(p_cloth); }
	void register_soft_body(GodotPhysXSoftBody3D *p_sb) { soft_bodies.insert(p_sb); }
	void unregister_soft_body(GodotPhysXSoftBody3D *p_sb) { soft_bodies.erase(p_sb); }

	// Drop a body from every area's overlap set (body leaving the simulation).
	void body_removed_from_areas(GodotPhysXBody3D *p_body);

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
};

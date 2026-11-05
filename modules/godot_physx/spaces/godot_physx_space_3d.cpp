/**************************************************************************/
/*  godot_physx_space_3d.cpp                                              */
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

#include "godot_physx_space_3d.h"

#include "../godot_physx_conversions.h"
#include "../godot_physx_project_settings.h"
#include "../objects/godot_physx_area_3d.h"
#include "../objects/godot_physx_body_3d.h"
#include "../objects/godot_physx_cloth_3d.h"
#include "../objects/godot_physx_particle_fluid_3d.h"
#include "../objects/godot_physx_soft_body_3d.h"
#include "../shapes/godot_physx_shape_3d.h"
#include "godot_physx_direct_state_3d.h"

#include "core/error/error_macros.h"
#include "core/math/face3.h"
#include "core/math/geometry_3d.h"
#include "core/math/math_defs.h"
#include "core/object/object.h"
#include "core/os/mutex.h"

#include <PxPhysicsAPI.h>

using namespace physx;

namespace {

struct AreaPriorityCompare {
	_FORCE_INLINE_ bool operator()(const GodotPhysXArea3D *a, const GodotPhysXArea3D *b) const {
		return a->get_priority() < b->get_priority();
	}
};

_FORCE_INLINE_ GodotPhysXBody3D *actor_body(const PxActor *p_actor) {
	return p_actor ? static_cast<GodotPhysXBody3D *>(p_actor->userData) : nullptr;
}

_FORCE_INLINE_ int shape_index(const PxShape *p_shape) {
	return p_shape ? (int)reinterpret_cast<uintptr_t>(p_shape->userData) : 0;
}

// Godot collision layer/mask + opt-in contact notification.
PxFilterFlags godot_physx_filter_shader(
		PxFilterObjectAttributes attributes0, PxFilterData filter_data0,
		PxFilterObjectAttributes attributes1, PxFilterData filter_data1,
		PxPairFlags &pair_flags, const void *, PxU32) {
	// Godot collision layer/mask applies to triggers too.
	const bool collide =
			(filter_data0.word0 & filter_data1.word1) ||
			(filter_data1.word0 & filter_data0.word1);
	if (!collide) {
		return PxFilterFlag::eSUPPRESS;
	}

	if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1)) {
		pair_flags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}
	// The scene keeps kinematic-static and kinematic-kinematic pairs (for areas, which are kinematic); between
	// bodies they have nothing to solve, as PhysX's default would have it.
	const bool moves0 = PxGetFilterObjectType(attributes0) != PxFilterObjectType::eRIGID_STATIC && !PxFilterObjectIsKinematic(attributes0);
	const bool moves1 = PxGetFilterObjectType(attributes1) != PxFilterObjectType::eRIGID_STATIC && !PxFilterObjectIsKinematic(attributes1);
	if (!moves0 && !moves1) {
		return PxFilterFlag::eSUPPRESS;
	}

	pair_flags = PxPairFlag::eCONTACT_DEFAULT;
	if ((filter_data0.word2 | filter_data1.word2) & GodotPhysXBody3D::FILTER_REPORTS_CONTACTS) {
		pair_flags |= PxPairFlag::eNOTIFY_TOUCH_FOUND |
				PxPairFlag::eNOTIFY_TOUCH_PERSISTS |
				PxPairFlag::eNOTIFY_TOUCH_LOST |
				PxPairFlag::eNOTIFY_CONTACT_POINTS;
	}
	// A body with collision exceptions: the (stateless) shader can't look
	// them up, so its pairs go to g_filter_callback to decide.
	if ((filter_data0.word2 | filter_data1.word2) & GodotPhysXBody3D::FILTER_HAS_EXCEPTIONS) {
		return PxFilterFlag::eCALLBACK;
	}
	return PxFilterFlag::eDEFAULT;
}

// Collision exceptions (add_collision_exception_with()): a pair where either
// body excepts the other never collides. Only pairs the shader flags reach it.
class ExceptionFilterCallback : public PxSimulationFilterCallback {
	static const GodotPhysXBody3D *_body(const PxActor *p_actor, const PxFilterData &p_data) {
		return p_actor && p_data.word3 == GodotPhysXBody3D::FILTER_BODY_MARKER ? static_cast<const GodotPhysXBody3D *>(p_actor->userData) : nullptr;
	}

public:
	virtual PxFilterFlags pairFound(PxU64, PxFilterObjectAttributes, PxFilterData p_data0, const PxActor *p_a0, const PxShape *,
			PxFilterObjectAttributes, PxFilterData p_data1, const PxActor *p_a1, const PxShape *, PxPairFlags &) override {
		const GodotPhysXBody3D *b0 = _body(p_a0, p_data0);
		const GodotPhysXBody3D *b1 = _body(p_a1, p_data1);
		if (b0 && b1 && (b0->has_collision_exception(b1->get_self()) || b1->has_collision_exception(b0->get_self()))) {
			return PxFilterFlag::eSUPPRESS;
		}
		return PxFilterFlag::eDEFAULT;
	}
	virtual void pairLost(PxU64, PxFilterObjectAttributes, PxFilterData, PxFilterObjectAttributes, PxFilterData, bool) override {}
	virtual bool statusChange(PxU64 &, PxPairFlags &, PxFilterFlags &) override { return false; }
};

// Stateless: reaches bodies through PxActor::userData, so one instance is shared
// by every scene.
class ContactCallback : public PxSimulationEventCallback {
public:
	virtual void onContact(const PxContactPairHeader &p_header, const PxContactPair *p_pairs, PxU32 p_nb_pairs) override {
		// A body freed since the last step still gets the touch-lost report of its pairs here, flagged as a removed
		// actor: its actor may already be released and its GodotPhysXBody3D deleted, so neither is read. That side
		// is no body to report to or about (its pairs are flagged as removed shapes, skipped below).
		GodotPhysXBody3D *b0 = (p_header.flags & PxContactPairHeaderFlag::eREMOVED_ACTOR_0) ? nullptr : actor_body(p_header.actors[0]);
		GodotPhysXBody3D *b1 = (p_header.flags & PxContactPairHeaderFlag::eREMOVED_ACTOR_1) ? nullptr : actor_body(p_header.actors[1]);
		const bool r0 = b0 && b0->reports_contacts();
		const bool r1 = b1 && b1->reports_contacts();
		if (!r0 && !r1) {
			return;
		}

		PxContactPairPoint points[32];
		for (PxU32 i = 0; i < p_nb_pairs; i++) {
			const PxContactPair &cp = p_pairs[i];
			if (cp.flags & (PxContactPairFlag::eREMOVED_SHAPE_0 | PxContactPairFlag::eREMOVED_SHAPE_1)) {
				continue;
			}
			const int s0 = shape_index(cp.shapes[0]);
			const int s1 = shape_index(cp.shapes[1]);
			const PxU32 n = cp.extractContacts(points, 32);
			for (PxU32 j = 0; j < n; j++) {
				const PxContactPairPoint &p = points[j];
				const Vector3 pos = to_godot(p.position);
				const Vector3 nrm = to_godot(p.normal);
				const Vector3 imp = to_godot(p.impulse);
				if (r0) {
					GodotPhysXBody3D::Contact c;
					c.position = pos;
					c.normal = nrm;
					c.impulse = imp;
					c.local_shape = s0;
					c.collider = b1 ? b1->get_self() : RID();
					c.collider_id = b1 ? b1->get_instance_id() : ObjectID();
					c.collider_shape = s1;
					c.collider_velocity = b1 ? b1->get_linear_velocity() : Vector3();
					b0->add_contact(c);
				}
				if (r1) {
					GodotPhysXBody3D::Contact c;
					c.position = pos;
					c.normal = -nrm;
					c.impulse = -imp;
					c.local_shape = s1;
					c.collider = b0 ? b0->get_self() : RID();
					c.collider_id = b0 ? b0->get_instance_id() : ObjectID();
					c.collider_shape = s0;
					c.collider_velocity = b0 ? b0->get_linear_velocity() : Vector3();
					b1->add_contact(c);
				}
			}
		}
	}

	virtual void onTrigger(PxTriggerPair *p_pairs, PxU32 p_count) override {
		for (PxU32 i = 0; i < p_count; i++) {
			const PxTriggerPair &tp = p_pairs[i];
			if (tp.flags & (PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER | PxTriggerPairFlag::eREMOVED_SHAPE_OTHER)) {
				continue;
			}
			// triggerActor is always the area; otherActor is a rigid body
			// (PhysX does not report trigger-trigger pairs).
			GodotPhysXArea3D *area = static_cast<GodotPhysXArea3D *>(tp.triggerActor->userData);
			GodotPhysXBody3D *body = static_cast<GodotPhysXBody3D *>(tp.otherActor->userData);
			// Not vehicles yet: an area would also apply its gravity and damping to the stand-in body.
			if (!area || !body || body->is_vehicle_proxy()) {
				continue;
			}
			const bool entered = tp.status & PxPairFlag::eNOTIFY_TOUCH_FOUND;
			area->report_body_overlap(body, shape_index(tp.otherShape), shape_index(tp.triggerShape), entered);
		}
	}
	virtual void onConstraintBreak(PxConstraintInfo *, PxU32) override {}
	virtual void onWake(PxActor **, PxU32) override {}
	virtual void onSleep(PxActor **, PxU32) override {}
	virtual void onAdvance(const PxRigidBody *const *, const PxTransform *, PxU32) override {}
};

ContactCallback g_contact_callback;
ExceptionFilterCallback g_filter_callback;

} //namespace

GodotPhysXSpace3D::GodotPhysXSpace3D(PxPhysics *p_physics, PxDefaultCpuDispatcher *p_dispatcher, PxCudaContextManager *p_cuda) {
	px_physics = p_physics;
	ERR_FAIL_NULL(px_physics);

	PxSceneDesc scene_desc(px_physics->getTolerancesScale());
	scene_desc.gravity = to_px(gravity);
	scene_desc.cpuDispatcher = p_dispatcher;
	scene_desc.filterShader = godot_physx_filter_shader;
	scene_desc.filterCallback = &g_filter_callback;
	scene_desc.simulationEventCallback = &g_contact_callback;
	scene_desc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;
	// Areas are kinematic actors: these pairs reach the filter shader, so an area sees static and kinematic bodies.
	scene_desc.staticKineFilteringMode = PxPairFilteringMode::eKEEP;
	scene_desc.kineKineFilteringMode = PxPairFilteringMode::eKEEP;
	if (GodotPhysXProjectSettings::stabilization) {
		// Damps low-mass stacked/piled bodies toward rest so they settle and
		// cross the sleep threshold instead of jittering forever. The same
		// mechanism Unity and Unreal expose as "stabilization".
		scene_desc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
	}
	// Solver: PGS by default (matches other backends' feel in big rigid-body
	// scenes). TGS is opt-in via physics/physx_3d/simulation/solver_type -- it
	// holds joint chains steadier under sustained external forces like wind.
	scene_desc.solverType = GodotPhysXProjectSettings::solver_type == 1 ? PxSolverType::eTGS : PxSolverType::ePGS;

	if (GodotPhysXProjectSettings::enhanced_determinism) {
		// Same-binary/same-platform determinism, independent of worker count and
		// call order (not cross-platform). GPU dynamics stays off in this mode.
		scene_desc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
	}

	if (p_cuda) {
		px_cuda = p_cuda;
		scene_desc.cudaContextManager = p_cuda;
		scene_desc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
		scene_desc.broadPhaseType = PxBroadPhaseType::eGPU;
		// Headroom for large scenes; PhysX grows some of these on demand but
		// warns if the initial capacity is exceeded. Sized for tens of thousands
		// of colliding rigid bodies plus a modest particle-contact budget for
		// PhysXParticleFluid3D; deformable surface/volume buffers stay off.
		scene_desc.gpuMaxNumPartitions = 8;
		scene_desc.gpuDynamicsConfig.tempBufferCapacity = 64 * 1024 * 1024;
		scene_desc.gpuDynamicsConfig.maxRigidContactCount = 4 * 1024 * 1024;
		scene_desc.gpuDynamicsConfig.maxRigidPatchCount = 1024 * 1024;
		scene_desc.gpuDynamicsConfig.heapCapacity = 256 * 1024 * 1024;
		scene_desc.gpuDynamicsConfig.foundLostPairsCapacity = 4 * 1024 * 1024;
		scene_desc.gpuDynamicsConfig.collisionStackSize = 256 * 1024 * 1024;
		// Non-zero so PxDeformableSurface (cloth) / PxDeformableVolume (GPU soft
		// bodies) can generate contacts; either touching anything with its
		// budget at 0 overflows and kills GPU sim.
		scene_desc.gpuDynamicsConfig.maxDeformableSurfaceContacts = 512 * 1024;
		scene_desc.gpuDynamicsConfig.maxDeformableVolumeContacts = 1024 * 1024;
		scene_desc.gpuDynamicsConfig.maxParticleContacts = 1 * 1024 * 1024;
		gpu_enabled = true;
	}

	px_scene = px_physics->createScene(scene_desc);
	ERR_FAIL_NULL(px_scene);

	default_material = px_physics->createMaterial(0.5f, 0.5f, 0.0f);
}

static LocalVector<PxMaterial *> pending_material_releases;
static BinaryMutex pending_material_mutex;

void GodotPhysXSpace3D::defer_material_release(PxMaterial *p_material) {
	if (!p_material) {
		return;
	}
	MutexLock lock(pending_material_mutex);
	pending_material_releases.push_back(p_material);
}

void GodotPhysXSpace3D::flush_material_releases() {
	MutexLock lock(pending_material_mutex);
	for (PxMaterial *m : pending_material_releases) {
		m->release();
	}
	pending_material_releases.clear();
}

void GodotPhysXSpace3D::hold_actor_materials(PxRigidActor *p_actor) {
	if (!p_actor) {
		return;
	}
	const PxU32 n = p_actor->getNbShapes();
	LocalVector<PxShape *> shapes;
	shapes.resize(n);
	p_actor->getShapes(shapes.ptr(), n);
	LocalVector<PxMaterial *> held;
	for (PxShape *shape : shapes) {
		const PxU16 nm = shape->getNbMaterials();
		LocalVector<PxMaterial *> mats;
		mats.resize(nm);
		shape->getMaterials(mats.ptr(), nm);
		for (PxMaterial *m : mats) {
			if (m && !held.has(m)) {
				m->acquireReference();
				held.push_back(m);
				defer_material_release(m);
			}
		}
	}
}

GodotPhysXSpace3D::~GodotPhysXSpace3D() {
	if (direct_state) {
		memdelete(direct_state);
	}
	if (default_material) {
		default_material->release();
	}
	if (px_scene) {
		px_scene->release();
	}
}

void GodotPhysXSpace3D::set_gravity_vector(const Vector3 &p_gravity) {
	gravity = p_gravity;
	if (px_scene) {
		px_scene->setGravity(to_px(gravity));
	}
}

void GodotPhysXSpace3D::set_gravity_magnitude(real_t p_magnitude) {
	gravity_magnitude = p_magnitude;
	set_gravity_vector(gravity_direction * gravity_magnitude);
}

void GodotPhysXSpace3D::set_gravity_direction(const Vector3 &p_direction) {
	gravity_direction = p_direction;
	set_gravity_vector(gravity_direction * gravity_magnitude);
}

void GodotPhysXSpace3D::set_param(PhysicsServer3D::SpaceParameter p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
			sleep_threshold_linear = p_value;
			break;
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
			sleep_threshold_angular = p_value;
			break;
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP:
			time_before_sleep = p_value;
			break;
		default:
			// Contact bias / recycle radius / solver iteration count are not
			// mapped onto PxScene yet.
			break;
	}
}

real_t GodotPhysXSpace3D::get_param(PhysicsServer3D::SpaceParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
			return sleep_threshold_linear;
		case PhysicsServer3D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
			return sleep_threshold_angular;
		case PhysicsServer3D::SPACE_PARAM_BODY_TIME_TO_SLEEP:
			return time_before_sleep;
		default:
			return 0.0;
	}
}

void GodotPhysXSpace3D::step(real_t p_step) {
	ERR_FAIL_NULL(px_scene);
	if (p_step <= 0.0) {
		return;
	}
	last_step = p_step;

	for (GodotPhysXBody3D *body : contact_reporters) {
		body->clear_contacts();
	}

	// Bodies that integrate their own forces run their callback before the solve,
	// in place of the built-in gravity/damping/area integration.
	for (GodotPhysXBody3D *body : force_integrators) {
		body->call_force_integration();
	}
	for (GodotPhysXBody3D *body : constant_force_bodies) {
		body->apply_constant_forces();
	}
	_apply_separation_rays(p_step);
	for (GodotPhysXBody3D *body : motor_driven_bodies) {
		body->keep_awake();
	}

	_apply_area_overrides();
	_detect_area_overlaps();

	flush_material_releases(); // (see defer_material_release: removals reach this scene's sync below)
	px_scene->simulate((PxReal)p_step);
	px_scene->fetchResults(true); // fills body contact buffers via g_contact_callback

	// Pull back actors that moved this step (eENABLE_ACTIVE_ACTORS), plus one
	// final pull for bodies that just went to sleep so their resting pose and
	// sleep state reach the node. Bodies asleep across the whole step cost
	// nothing.
	sync_bodies.clear();
	HashSet<GodotPhysXBody3D *> now_awake;

	PxU32 nb_active = 0;
	PxActor **active = px_scene->getActiveActors(nb_active);
	for (PxU32 i = 0; i < nb_active; i++) {
		// The active set also contains particle systems and any future deformable
		// actors; only rigid dynamics carry a GodotPhysXBody3D in userData.
		if (!active[i]->is<PxRigidDynamic>()) {
			continue;
		}
		if (active[i]->getName() == GodotPhysXArea3D::ACTOR_NAME) {
			continue; // an area (kinematic): its userData is the area
		}
		GodotPhysXBody3D *body = static_cast<GodotPhysXBody3D *>(active[i]->userData);
		if (!body) {
			continue;
		}
		body->pull_transform_from_px();
		if (body->is_vehicle_proxy()) {
			continue; // a vehicle's identity: its pose and velocity kept current, nothing to call back
		}
		now_awake.insert(body);
		sync_bodies.push_back(body);
	}

	// finish_isosurface_extraction() first, for every fluid, before any of them
	// reads back: onPostSolve (already run for all fluids by the fetchResults()
	// above) kicks each fluid's smoothing kernel without syncing, so by this
	// point N concurrent fluids' GPU work is already in flight together --
	// finishing them here (sync + CPU clamp + extract) no longer serializes N
	// separate stalls the way syncing inline inside onPostSolve did.
	for (GodotPhysXParticleFluid3D *fluid : fluids) {
		fluid->finish_isosurface_extraction();
	}
	for (GodotPhysXParticleFluid3D *fluid : fluids) {
		fluid->read_back();
	}
	for (GodotPhysXCloth3D *cloth : cloths) {
		cloth->read_back();
	}
	// GPU soft bodies (PxDeformableVolume) simulated inside px_scene -- pull
	// their deformed state off the device. CPU soft bodies advance here, after
	// the rigid solve, so their per-vertex world query sees this step's final
	// rigid poses.
	for (GodotPhysXSoftBody3D *sb : soft_bodies) {
		sb->read_back();
		sb->step(p_step, gravity);
	}

	for (GodotPhysXBody3D *body : awake_bodies) {
		if (!now_awake.has(body)) { // active last step, asleep now
			body->pull_transform_from_px();
			sync_bodies.push_back(body);
		}
	}

	awake_bodies = now_awake;
}

void GodotPhysXSpace3D::body_actor_gone(GodotPhysXBody3D *p_body) {
	for (GodotPhysXArea3D *area : areas) {
		area->body_actor_gone(p_body);
	}
}

void GodotPhysXSpace3D::body_shape_gone(GodotPhysXBody3D *p_body, int p_shape) {
	for (GodotPhysXArea3D *area : areas) {
		area->body_shape_gone(p_body, p_shape);
	}
}

void GodotPhysXSpace3D::body_removed_from_areas(GodotPhysXBody3D *p_body) {
	for (GodotPhysXArea3D *area : areas) {
		area->body_removed(p_body);
	}
}

void GodotPhysXSpace3D::unregister_area(GodotPhysXArea3D *p_area) {
	areas.erase(p_area);
	for (GodotPhysXArea3D *area : areas) {
		area->area_removed(p_area);
	}
}

void GodotPhysXSpace3D::_detect_area_overlaps() {
	// O(n^2) over every (monitoring, monitorable) pair per step, but only pairs whose world bounds meet get an exact
	// shape test (it showed up on a profile: a playtest island with 28 monitoring rock-wake areas went from ~35 ms of
	// polls a step to a fraction of a millisecond). A real broad-phase (a bounds tree) would be the next lever.
	// Only the monitorable areas whose layer / mask can meet some monitoring area's are polled (poll_area_overlap
	// rejects every other pair by the same layer test before it touches any state, so the events are the same), in
	// the same order; and each one's world bounds are refreshed once per step (kept while it doesn't move).
	LocalVector<GodotPhysXArea3D *> monitoring;
	uint32_t monitoring_layers = 0;
	uint32_t monitoring_masks = 0;
	for (GodotPhysXArea3D *area : areas) {
		if (area->wants_area_monitoring()) {
			monitoring.push_back(area);
			monitoring_layers |= area->get_collision_layer();
			monitoring_masks |= area->get_collision_mask();
		}
	}
	if (monitoring.is_empty()) {
		return;
	}
	LocalVector<GodotPhysXArea3D *> polled;
	for (GodotPhysXArea3D *area : areas) {
		const bool reachable = area->is_monitorable() && ((area->get_collision_layer() & monitoring_masks) || (area->get_collision_mask() & monitoring_layers));
		if (reachable) {
			polled.push_back(area);
		}
		if (reachable || area->wants_area_monitoring()) {
			area->refresh_poll_bounds();
		}
	}
	for (GodotPhysXArea3D *area : monitoring) {
		for (GodotPhysXArea3D *other : polled) {
			if (other != area) {
				area->poll_area_overlap(other);
			}
		}
	}
}

void GodotPhysXSpace3D::set_default_damping(real_t p_linear, real_t p_angular) {
	default_linear_damp = p_linear;
	default_angular_damp = p_angular;
	for (GodotPhysXBody3D *body : bodies) {
		if (!area_damped_bodies.has(body)) {
			body->set_area_damping(default_linear_damp, default_angular_damp);
		}
	}
}

void GodotPhysXSpace3D::_apply_area_overrides() {
	// Gather, per body, the areas that impose a gravity/damp/wind override.
	HashMap<GodotPhysXBody3D *, LocalVector<GodotPhysXArea3D *>> affected;
	for (GodotPhysXArea3D *area : areas) {
		if (!area->has_force_override()) {
			continue;
		}
		for (const KeyValue<GodotPhysXBody3D *, uint32_t> &E : area->get_overlapping_bodies()) {
			GodotPhysXBody3D *body = E.key;
			if (body->get_mode() != PhysicsServer3D::BODY_MODE_RIGID && body->get_mode() != PhysicsServer3D::BODY_MODE_RIGID_LINEAR) {
				continue;
			}
			if (body->is_omitting_force_integration()) {
				continue; // the node's own callback is the only force source
			}
			affected[body].push_back(area);
		}
	}

	HashSet<GodotPhysXBody3D *> damped_now;
	for (KeyValue<GodotPhysXBody3D *, LocalVector<GodotPhysXArea3D *>> &E : affected) {
		GodotPhysXBody3D *body = E.key;
		LocalVector<GodotPhysXArea3D *> &list = E.value;
		list.sort_custom<AreaPriorityCompare>();

		const Vector3 pos = body->get_transform().origin;
		const real_t mass = MAX(body->get_mass(), (real_t)0.0001);
		const real_t gravity_scale = body->get_gravity_scale();

		// Areas build on the space's defaults: COMBINE adds, REPLACE replaces.
		Vector3 grav = gravity;
		real_t lin_damp = default_linear_damp;
		real_t ang_damp = default_angular_damp;
		Vector3 wind;

		for (GodotPhysXArea3D *area : list) {
			switch (area->get_gravity_mode()) {
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE:
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE:
					grav += area->gravity_at(pos);
					break;
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE:
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE:
					grav = area->gravity_at(pos);
					break;
				default:
					break;
			}
			switch (area->get_linear_damp_mode()) {
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE:
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE:
					lin_damp += area->get_linear_damp_value();
					break;
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE:
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE:
					lin_damp = area->get_linear_damp_value();
					break;
				default:
					break;
			}
			switch (area->get_angular_damp_mode()) {
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE:
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE:
					ang_damp += area->get_angular_damp_value();
					break;
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE:
				case PhysicsServer3D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE:
					ang_damp = area->get_angular_damp_value();
					break;
				default:
					break;
			}
			wind += area->wind_at(pos);
		}

		// Gravity delta relative to what the scene already gives the body (the
		// world default, or nothing at gravity_scale 0), times its gravity
		// scale, plus wind, as a force; the damping goes to the body's own
		// solver damping, combined with its own per its modes.
		const Vector3 builtin = gravity_scale == 0.0 ? Vector3() : gravity;
		const Vector3 force = (grav * gravity_scale - builtin) * mass + wind;
		if (!force.is_zero_approx()) {
			body->apply_central_force(force);
		}
		body->set_area_damping(lin_damp, ang_damp);
		damped_now.insert(body);
	}
	// gravity_scale other than 0 / 1 outside any overriding area: the rest of the scaled world gravity.
	for (GodotPhysXBody3D *body : gravity_scaled_bodies) {
		if (!affected.has(body)) {
			body->apply_gravity_delta(gravity * (body->get_gravity_scale() - 1.0) * MAX(body->get_mass(), (real_t)0.0001));
		}
	}
	// Bodies that left every overriding area go back to the defaults.
	for (GodotPhysXBody3D *body : area_damped_bodies) {
		if (!damped_now.has(body)) {
			body->set_area_damping(default_linear_damp, default_angular_damp);
		}
	}
	area_damped_bodies = damped_now;
}

void GodotPhysXSpace3D::call_queries() {
	// Notify bodies that moved or that just went to sleep this step.
	for (GodotPhysXBody3D *body : sync_bodies) {
		body->call_queries();
	}
	for (GodotPhysXArea3D *area : areas) {
		area->call_queries();
	}
}

GodotPhysXDirectSpaceState3D *GodotPhysXSpace3D::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(GodotPhysXDirectSpaceState3D);
		direct_state->space = this;
	}
	return direct_state;
}

namespace {

// Motion-test filter: skip the moving body's own actor and the caller's
// exclude sets, and skip triggers.
class MotionFilter : public PxQueryFilterCallback {
public:
	const PxRigidActor *self_actor = nullptr;
	const GodotPhysXBody3D *self_body = nullptr;
	const HashSet<RID> *exclude_bodies = nullptr;
	const HashSet<ObjectID> *exclude_objects = nullptr;
	uint32_t self_layer = 0;
	uint32_t self_mask = 0;

	virtual PxQueryHitType::Enum preFilter(const PxFilterData &, const PxShape *p_shape, const PxRigidActor *p_actor, PxHitFlags &) override {
		if (p_actor == self_actor) {
			return PxQueryHitType::eNONE;
		}
		if (p_shape && (p_shape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE)) {
			return PxQueryHitType::eNONE;
		}
		GodotPhysXBody3D *b = p_actor ? static_cast<GodotPhysXBody3D *>(p_actor->userData) : nullptr;
		if (!b) {
			return PxQueryHitType::eNONE;
		}
		if (exclude_bodies && exclude_bodies->has(b->get_self())) {
			return PxQueryHitType::eNONE;
		}
		// Collision exceptions, either way round (as Godot Physics' motion test).
		if (self_body && (self_body->has_collision_exception(b->get_self()) || b->has_collision_exception(self_body->get_self()))) {
			return PxQueryHitType::eNONE;
		}
		if (exclude_objects && exclude_objects->has(b->get_instance_id())) {
			return PxQueryHitType::eNONE;
		}
		// Same collision-layer/mask rule as regular contacts: hit only if
		// either side's mask matches the other's layer.
		const bool collide = (self_layer & b->get_collision_mask()) || (b->get_collision_layer() & self_mask);
		if (!collide) {
			return PxQueryHitType::eNONE;
		}
		return PxQueryHitType::eBLOCK;
	}
	virtual PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &, const PxShape *, const PxRigidActor *) override {
		return PxQueryHitType::eBLOCK;
	}
};

// One-sided trimeshes (backface_collision off) in the motion test. PhysX's
// overlap and penetration queries treat every mesh triangle as two-sided, so
// a body deep in a one-sided mesh from behind -- jumping up through a one-way
// platform -- got pushed out of its front and reported as standing on it.
// Jolt skips a triangle whose plane has the shape's center behind it; the same
// check runs here, but only for deep overlaps: a body resting on or walking
// over a mesh overlaps it by about the margin and never pays for it. (A
// sweep's initial-overlap MTD can report a deep overlap as ~0 deep, so that
// path always checks -- it only runs when a sweep starts inside a mesh.)
// And a body already moving out the way it would be pushed is left to its own
// motion -- pushing it there would land it on the surface mid-jump.
constexpr PxReal ONE_SIDED_DEEP_PENETRATION = 0.05f;
constexpr PxReal ONE_SIDED_MOVING_OUT = 0.5f; // cos of the angle between motion and push-out
constexpr PxU32 ONE_SIDED_MAX_TRIANGLES = 64;

bool is_one_sided_trimesh(const PxRigidActor *p_actor, const PxShape *p_shape) {
	if (!p_actor || !p_shape || p_shape->getGeometry().getType() != PxGeometryType::eTRIANGLEMESH) {
		return false;
	}
	const GodotPhysXBody3D *body = static_cast<const GodotPhysXBody3D *>(p_actor->userData);
	const GodotPhysXBody3D::ShapeRef *sr = body ? body->get_shape_ref((int)reinterpret_cast<uintptr_t>(p_shape->userData)) : nullptr;
	return sr && sr->shape && !sr->shape->has_backface_collision();
}

// True when the shape's center is behind every mesh triangle it overlaps.
bool behind_one_sided_mesh(const PxGeometry &p_geom, const PxTransform &p_pose, const PxShape *p_mesh_shape, const PxTransform &p_mesh_pose) {
	const PxTriangleMeshGeometry &mesh = static_cast<const PxTriangleMeshGeometry &>(p_mesh_shape->getGeometry());
	PxU32 tris[ONE_SIDED_MAX_TRIANGLES];
	bool overflow = false;
	const PxU32 count = PxMeshQuery::findOverlapTriangleMesh(p_geom, p_pose, mesh, p_mesh_pose, tris, ONE_SIDED_MAX_TRIANGLES, 0, overflow);
	if (count == 0 || overflow) {
		return false;
	}
	for (PxU32 i = 0; i < count; i++) {
		PxTriangle tri;
		PxMeshQuery::getTriangle(mesh, p_mesh_pose, tris[i], tri);
		PxVec3 normal;
		tri.normal(normal);
		if (normal.dot(p_pose.p - tri.verts[0]) >= 0.0f) {
			return false;
		}
	}
	return true;
}

// A SeparationRayShape3D on a body: it runs along the shape's +Z from the
// shape's origin, length scaled with the shape.
struct SeparationRay {
	PxVec3 origin;
	PxVec3 dir;
	PxReal length = 0.0f;
	bool slide_on_slope = false;
};

SeparationRay separation_ray(const GodotPhysXShape3D *p_shape, const PxTransform &p_body_pose, const Transform3D &p_shape_xform, const Vector3 &p_body_scale) {
	const PxTransform pose = p_body_pose * to_px(p_shape_xform);
	SeparationRay ray;
	ray.origin = pose.p;
	ray.dir = pose.q.rotate(PxVec3(0.0f, 0.0f, 1.0f));
	ray.length = (PxReal)(p_shape->get_ray_length() * (p_body_scale * p_shape_xform.basis.get_scale()).z);
	ray.slide_on_slope = p_shape->is_ray_sliding_on_slope();
	return ray;
}

// How far a separation ray's tip is past what it hit, and which way that
// pushes the body: back along the ray, or (slide_on_slope) out along the
// surface normal. False if the tip isn't in anything.
bool separation_ray_push(const SeparationRay &p_ray, const PxRaycastHit &p_hit, PxVec3 &r_dir, PxReal &r_amount) {
	const PxReal depth = p_ray.length - p_hit.distance;
	if (depth <= 0.0f) {
		return false;
	}
	if (p_ray.slide_on_slope) {
		r_dir = p_hit.normal;
		r_amount = depth * p_hit.normal.dot(-p_ray.dir);
	} else {
		r_dir = -p_ray.dir;
		r_amount = depth;
	}
	return r_amount > 0.0f;
}

// Whether to ignore an initial overlap of a motion test's shape with a mesh.
// p_depth < 0: depth unknown, always check.
bool skip_one_sided_overlap(const PxRigidActor *p_actor, const PxShape *p_shape, PxReal p_depth, const PxVec3 &p_push_dir, const PxVec3 &p_motion_dir,
		const PxGeometry &p_geom, const PxTransform &p_pose) {
	if (!is_one_sided_trimesh(p_actor, p_shape)) {
		return false;
	}
	if (p_motion_dir.dot(p_push_dir) > ONE_SIDED_MOVING_OUT) {
		return true;
	}
	return (p_depth < 0.0f || p_depth > ONE_SIDED_DEEP_PENETRATION) && behind_one_sided_mesh(p_geom, p_pose, p_shape, p_actor->getGlobalPose() * p_shape->getLocalPose());
}

constexpr PxU32 ROUND_CONTACT_MAX_TRIANGLES = 64;

enum RoundContact {
	ROUND_CONTACT_NONE, // overlaps no triangle
	ROUND_CONTACT_CORE_INSIDE, // the core crosses a triangle: no direction to push
	ROUND_CONTACT_FOUND,
};

// Where a capsule or sphere (core segment p_a..p_b, a point for a sphere) that
// overlaps a triangle mesh or height field really touches it: the deepest of
// the triangles it overlaps, as the direction from the touch point to the core
// and how far the round shape is in (<= 0: only touching). PhysX's own result
// for a sweep that starts overlapping a mesh isn't that contact -- its depth is
// the push along whichever axis it picked (32 cm back along the walk for a
// 3.5 cm lip between two floor pieces) and its position can be a vertex a metre
// away -- so a walking character was snapped sideways by tens of centimetres.
RoundContact round_mesh_contact(const PxGeometry &p_geom, const PxTransform &p_pose, const PxVec3 &p_a, const PxVec3 &p_b, PxReal p_radius,
		const PxShape *p_mesh_shape, const PxTransform &p_mesh_pose, PxVec3 &r_normal, PxReal &r_depth, PxVec3 &r_point) {
	const PxGeometry &mesh = p_mesh_shape->getGeometry();
	PxU32 tris[ROUND_CONTACT_MAX_TRIANGLES];
	bool overflow = false;
	PxU32 count = 0;
	if (mesh.getType() == PxGeometryType::eTRIANGLEMESH) {
		count = PxMeshQuery::findOverlapTriangleMesh(p_geom, p_pose, static_cast<const PxTriangleMeshGeometry &>(mesh), p_mesh_pose, tris, ROUND_CONTACT_MAX_TRIANGLES, 0, overflow);
	} else {
		count = PxMeshQuery::findOverlapHeightField(p_geom, p_pose, static_cast<const PxHeightFieldGeometry &>(mesh), p_mesh_pose, tris, ROUND_CONTACT_MAX_TRIANGLES, 0, overflow);
	}
	if (count == 0) {
		return ROUND_CONTACT_NONE;
	}
	const Vector3 a = to_godot(p_a);
	const Vector3 b = to_godot(p_b);
	real_t best = Math::INF;
	Vector3 best_core;
	Vector3 best_tri;
	for (PxU32 i = 0; i < count; i++) {
		PxTriangle tri;
		if (mesh.getType() == PxGeometryType::eTRIANGLEMESH) {
			PxMeshQuery::getTriangle(static_cast<const PxTriangleMeshGeometry &>(mesh), p_mesh_pose, tris[i], tri);
		} else {
			PxMeshQuery::getTriangle(static_cast<const PxHeightFieldGeometry &>(mesh), p_mesh_pose, tris[i], tri);
		}
		const Face3 face(to_godot(tri.verts[0]), to_godot(tri.verts[1]), to_godot(tri.verts[2]));
		if (a != b && Geometry3D::segment_intersects_triangle(a, b, face.vertex[0], face.vertex[1], face.vertex[2])) {
			return ROUND_CONTACT_CORE_INSIDE;
		}
		// Closest pair: an end of the core against the face, or the core
		// against one of the face's edges.
		const Vector3 ends[2] = { a, b };
		for (const Vector3 &end : ends) {
			const Vector3 on_tri = face.get_closest_point_to(end);
			const real_t d = end.distance_squared_to(on_tri);
			if (d < best) {
				best = d;
				best_core = end;
				best_tri = on_tri;
			}
		}
		for (int e = 0; e < 3; e++) {
			Vector3 on_core;
			Vector3 on_edge;
			Geometry3D::get_closest_points_between_segments(a, b, face.vertex[e], face.vertex[(e + 1) % 3], on_core, on_edge);
			const real_t d = on_core.distance_squared_to(on_edge);
			if (d < best) {
				best = d;
				best_core = on_core;
				best_tri = on_edge;
			}
		}
	}
	const real_t dist = Math::sqrt(best);
	if (dist < (real_t)1e-4) {
		return ROUND_CONTACT_CORE_INSIDE;
	}
	r_normal = to_px((best_core - best_tri) / dist);
	r_depth = p_radius - (PxReal)dist;
	r_point = to_px(best_tri);
	return ROUND_CONTACT_FOUND;
}

} //namespace

bool GodotPhysXSpace3D::test_body_motion(GodotPhysXBody3D *p_body, const PhysicsServer3D::MotionParameters &p_params, PhysicsServer3D::MotionResult *r_result) {
	ERR_FAIL_NULL_V(px_scene, false);
	ERR_FAIL_NULL_V(p_body, false);

	if (r_result) {
		*r_result = PhysicsServer3D::MotionResult();
		r_result->travel = p_params.motion;
		r_result->collision_safe_fraction = 1.0;
		r_result->collision_unsafe_fraction = 1.0;
	}

	const int shape_count = p_body->get_shape_count();
	if (shape_count == 0) {
		return false;
	}

	const PxReal margin = MAX((PxReal)p_params.margin, 0.0001f);
	// PhysX poses carry no scale, so the mover's node scale is baked into its
	// query geometry (combined with any per-shape transform scale).
	const Vector3 motion_scale = p_params.from.basis.get_scale();

	MotionFilter filter;
	filter.self_actor = p_body->get_px_actor();
	filter.self_body = p_body;
	filter.exclude_bodies = &p_params.exclude_bodies;
	filter.exclude_objects = &p_params.exclude_objects;
	filter.self_layer = p_body->get_collision_layer();
	filter.self_mask = p_body->get_collision_mask();
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER);
	const PxVec3 motion_dir = p_params.motion.length() > CMP_EPSILON ? to_px(p_params.motion.normalized()) : PxVec3(0.0f);

	// --- Depenetration recovery -------------------------------------------------
	PxVec3 recover(0.0f);
	// Deepest penetration seen, used as the collision report when there's no
	// sweep hit but the body was pushed out of something.
	PxF32 rec_depth = 0.0f;
	PxVec3 rec_normal(0.0f);
	PxVec3 rec_point(0.0f);
	const PxRigidActor *rec_actor = nullptr;
	const PxShape *rec_shape = nullptr;

	for (int iter = 0; iter < 4; iter++) {
		PxVec3 iter_recover(0.0f);
		bool any = false;
		for (int i = 0; i < shape_count; i++) {
			const GodotPhysXBody3D::ShapeRef *sr = p_body->get_shape_ref(i);
			if (sr && !sr->disabled && sr->shape && sr->shape->is_separation_ray()) {
				// Push the body back until the ray's tip sits on what it hit --
				// the character stands on the ray (and steps up onto ledges
				// lower than it reaches).
				const SeparationRay ray = separation_ray(sr->shape, PxTransform(recover) * to_px(p_params.from), sr->xform, motion_scale);
				PxRaycastBuffer rh;
				PxVec3 push_dir;
				PxReal amount;
				if (px_scene->raycast(ray.origin, ray.dir, ray.length, rh, PxHitFlag::ePOSITION | PxHitFlag::eNORMAL, fd, &filter) &&
						rh.hasBlock && separation_ray_push(ray, rh.block, push_dir, amount)) {
					iter_recover += push_dir * amount;
					any = true;
					if (iter == 0 && amount > rec_depth) {
						rec_depth = amount;
						rec_normal = push_dir;
						rec_point = rh.block.position;
						rec_actor = rh.block.actor;
						rec_shape = rh.block.shape;
					}
				}
				continue;
			}
			if (!sr || !sr->shape || !sr->shape->is_valid()) {
				continue;
			}
			const GodotPhysXShape3D::ScaledGeometry sg = sr->shape->scaled_geometry(motion_scale * sr->xform.basis.get_scale());
			const PxTransform pose = PxTransform(recover) * to_px(p_params.from) * to_px(sr->xform) * sg.local_pose;

			PxOverlapHit touches[16];
			PxOverlapBuffer buf(touches, 16);
			PxQueryFilterData ofd(fd.flags | PxQueryFlag::eNO_BLOCK);
			px_scene->overlap(sg.geom.any(), pose, buf, ofd, &filter);
			for (PxU32 t = 0; t < buf.getNbTouches(); t++) {
				const PxOverlapHit &h = buf.getTouch(t);
				PxVec3 dir;
				PxF32 depth;
				const PxTransform other_pose = h.actor->getGlobalPose() * h.shape->getLocalPose();
				if (PxGeometryQuery::computePenetration(dir, depth, sg.geom.any(), pose, h.shape->getGeometry(), other_pose)) {
					if (skip_one_sided_overlap(h.actor, h.shape, depth, dir, motion_dir, sg.geom.any(), pose)) {
						continue;
					}
					iter_recover += dir * (depth + margin);
					any = true;
					if (iter == 0 && depth > rec_depth) {
						rec_depth = depth;
						rec_normal = dir;
						rec_point = pose.p;
						rec_actor = h.actor;
						rec_shape = h.shape;
					}
				}
			}
		}
		if (!any) {
			break;
		}
		recover += iter_recover * 0.4f;
	}

	const PxVec3 recover_motion = recover;
	const PxTransform recovered_from = PxTransform(recover_motion) * to_px(p_params.from);

	// --- Sweep ---------------------------------------------------------------
	const real_t motion_len = p_params.motion.length();
	real_t safe_fraction = 1.0;
	PxSweepHit best_hit;
	bool has_hit = false;

	// Extra depenetration from geometry that computePenetration() can't handle
	// (triangle meshes, height fields): the swept MTD below fills this in.
	PxVec3 mtd_recover(0.0f);

	if (motion_len > CMP_EPSILON) {
		const PxVec3 unit_dir = to_px(p_params.motion / motion_len);
		// A capsule or sphere that starts the sweep overlapping a triangle mesh or
		// height field is pushed out first and swept again from there (as Godot
		// Physics and Jolt recover before they sweep): a sweep from inside reports
		// only the push-out, and following that sideways along a step's edge lifts
		// the body over steps far too tall to walk up.
		bool resweep = false;
		PxSweepHit resting_hit;
		bool resting = false;
		for (int pass = 0; pass < 2; pass++) {
			if (pass == 1) {
				if (!resweep) {
					break;
				}
				// The first sweep's floor, if it found nothing it was stopped by.
				resting = has_hit && safe_fraction == (real_t)1.0;
				resting_hit = best_hit;
				safe_fraction = 1.0;
				best_hit = PxSweepHit();
				has_hit = false;
			}
			const PxTransform sweep_from = PxTransform(mtd_recover) * recovered_from;
			for (int i = 0; i < shape_count; i++) {
				const GodotPhysXBody3D::ShapeRef *sr = p_body->get_shape_ref(i);
				if (sr && !sr->disabled && sr->shape && sr->shape->is_separation_ray()) {
					// Rays only take part in the motion when snapping to the floor
					// (collide_separation_ray), or acting as a regular shape with
					// slide_on_slope -- as on Godot Physics and Jolt. The tip moves
					// with the body; cast it along the motion.
					if (!p_params.collide_separation_ray && !sr->shape->is_ray_sliding_on_slope()) {
						continue;
					}
					const SeparationRay ray = separation_ray(sr->shape, sweep_from, sr->xform, motion_scale);
					PxRaycastBuffer rh;
					if (px_scene->raycast(ray.origin + ray.dir * ray.length, unit_dir, (PxReal)motion_len, rh, PxHitFlag::ePOSITION | PxHitFlag::eNORMAL, fd, &filter) &&
							rh.hasBlock && rh.block.normal.dot(-unit_dir) >= 0.001f) {
						const real_t frac = CLAMP((real_t)rh.block.distance / motion_len, (real_t)0.0, (real_t)1.0);
						if (frac < safe_fraction) {
							safe_fraction = frac;
							best_hit.actor = rh.block.actor;
							best_hit.shape = rh.block.shape;
							best_hit.position = rh.block.position;
							best_hit.normal = rh.block.normal;
							best_hit.distance = rh.block.distance;
							best_hit.faceIndex = rh.block.faceIndex;
							has_hit = true;
						}
					}
					continue;
				}
				if (!sr || !sr->shape || !sr->shape->is_valid()) {
					continue;
				}
				const GodotPhysXShape3D::ScaledGeometry sg = sr->shape->scaled_geometry(motion_scale * sr->xform.basis.get_scale());
				const PxTransform pose = sweep_from * to_px(sr->xform) * sg.local_pose;

				PxSweepBuffer hit;
				if (px_scene->sweep(sg.geom.any(), pose, unit_dir, (PxReal)motion_len, hit,
							PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eMTD,
							fd, &filter, nullptr, 0.0f) &&
						hit.hasBlock) {
					// Internal-edge fix: sweeping over a triangle-mesh surface catches
					// on the shared edges between facets and PhysX hands back the edge
					// normal (often axis-aligned, looks like a wall). A capsule or
					// sphere gets its true contact normal instead -- from the touch
					// point to the nearest point of its core -- which is straight up on
					// a flat floor's internal edges, and tilted on a real ledge's edge
					// as far as the ledge is low (so a low lip is walked up and a tall
					// step stops it, as on Godot Physics and Jolt). Other shapes get the
					// triangle's face normal, so a walking box slides on the surface
					// instead of hitting phantom steps.
					const PxGeometryType::Enum mover_type = sg.geom.any().getType();
					if (hit.block.actor && hit.block.shape && (mover_type == PxGeometryType::eCAPSULE || mover_type == PxGeometryType::eSPHERE) &&
							(hit.block.shape->getGeometry().getType() == PxGeometryType::eTRIANGLEMESH || hit.block.shape->getGeometry().getType() == PxGeometryType::eHEIGHTFIELD)) {
						const PxVec3 centre = pose.p + unit_dir * PxMax(hit.block.distance, 0.0f);
						PxVec3 a = centre;
						PxVec3 ab(0.0f);
						PxReal radius;
						if (mover_type == PxGeometryType::eCAPSULE) {
							// PhysX capsules lie along their local X axis.
							const PxCapsuleGeometry &capsule = static_cast<const PxCapsuleGeometry &>(sg.geom.any());
							const PxVec3 half = pose.q.getBasisVector0() * capsule.halfHeight;
							a = centre - half;
							ab = half * 2.0f;
							radius = capsule.radius;
						} else {
							radius = static_cast<const PxSphereGeometry &>(sg.geom.any()).radius;
						}
						if (hit.block.distance <= 0.0f) {
							// Starts overlapping: take the contact from the triangles
							// themselves (see round_mesh_contact).
							PxVec3 normal;
							PxReal depth;
							PxVec3 point;
							const RoundContact contact = round_mesh_contact(sg.geom.any(), pose, a, a + ab, radius, hit.block.shape,
									hit.block.actor->getGlobalPose() * hit.block.shape->getLocalPose(), normal, depth, point);
							if (contact == ROUND_CONTACT_NONE) {
								continue;
							}
							if (contact == ROUND_CONTACT_FOUND) {
								hit.block.normal = normal;
								hit.block.distance = -PxMax(depth, 0.0f);
								hit.block.position = point;
							}
						} else {
							const PxReal len2 = ab.magnitudeSquared();
							const PxReal s = len2 > 1e-8f ? PxClamp((hit.block.position - a).dot(ab) / len2, 0.0f, 1.0f) : 0.0f;
							const PxVec3 to_core = a + ab * s - hit.block.position;
							const PxReal dist = to_core.magnitude();
							if (dist > 1e-4f) {
								hit.block.normal = to_core / dist;
							}
						}
					} else if (hit.block.actor && hit.block.shape && hit.block.faceIndex != 0xffffffffu &&
							hit.block.shape->getGeometry().getType() == PxGeometryType::eTRIANGLEMESH) {
						const PxTransform hit_pose = hit.block.actor->getGlobalPose() *
								hit.block.shape->getLocalPose();
						PxTriangle tri;
						PxMeshQuery::getTriangle(
								static_cast<const PxTriangleMeshGeometry &>(hit.block.shape->getGeometry()),
								hit_pose, hit.block.faceIndex, tri);
						PxVec3 face_n;
						tri.normal(face_n);
						if (face_n.dot(hit.block.normal) < 0.0f) {
							face_n = -face_n;
						}
						hit.block.normal = face_n;
					}
					// eMTD reports an initial overlap as a negative distance with the
					// push-out normal. The body isn't blocked from moving -- it just
					// needs depenetrating -- so record the push and don't clamp the
					// travel. This is the only depenetration path for tri-mesh /
					// height-field terrain (computePenetration() rejects those).
					if (hit.block.distance <= 0.0f) {
						const PxF32 pen = -hit.block.distance;
						if (skip_one_sided_overlap(hit.block.actor, hit.block.shape, -1.0f, hit.block.normal, motion_dir, sg.geom.any(), pose)) {
							continue;
						}
						mtd_recover += hit.block.normal * (pen + margin);
						const bool round = mover_type == PxGeometryType::eCAPSULE || mover_type == PxGeometryType::eSPHERE;
						// A sweep that starts overlapping a mesh reports only that
						// overlap, never what else of the mesh is ahead -- standing on
						// a floor, the edge of a step walked into -- so a capsule or
						// sphere sweeps again from where it's pushed out to. A push-out
						// back against the way the body moves (an edge or a step it
						// walks into) is left to that second sweep.
						const PxReal push_back = -(hit.block.normal.x * unit_dir.x + hit.block.normal.z * unit_dir.z);
						if (pass == 0 && round && (hit.block.shape->getGeometry().getType() == PxGeometryType::eTRIANGLEMESH || hit.block.shape->getGeometry().getType() == PxGeometryType::eHEIGHTFIELD)) {
							resweep = true;
							if (push_back > 0.02f) {
								continue;
							}
						}
						// Only report a floor-like overlap so move_and_slide keeps a
						// character grounded. Near-horizontal push-outs here are
						// almost always a facet edge the capsule is grazing -- just
						// depenetrate, don't treat it as a wall that stops the travel
						// (that leaves a walking character frozen mid-slope).
						// A capsule's or sphere's push-out is its true contact normal
						// (above), never a facet edge, so one it's pressed into -- a
						// wall, a step's edge, a lip -- stops the travel there and
						// move_and_slide slides the rest along it: up a low lip's
						// edge, not up a tall step's. Otherwise the body would sink
						// into the step a little each frame and be pushed out over
						// its top.
						const bool pressed = round && (push_back > 0.02f || (hit.block.normal.y <= 0.3f && hit.block.normal.dot(-unit_dir) >= 0.001f));
						if (!pressed && hit.block.normal.y > 0.3f && (!has_hit || safe_fraction > (real_t)0.0)) {
							best_hit = hit.block;
							has_hit = true;
						} else if (pressed) {
							safe_fraction = 0.0;
							best_hit = hit.block;
							has_hit = true;
						}
						continue;
					}
					// Ignore contacts that don't actually oppose the motion: a hit
					// whose normal faces along (rather than against) the sweep is a
					// grazing/edge contact from sliding on a surface.
					const PxReal approach = hit.block.normal.dot(-unit_dir);
					if (approach < 0.001f) {
						continue;
					}
					// Stop the margin short of the surface, as Godot Physics and Jolt leave it. A character
					// resting in true contact rubs what it stands on through the kinematic capsule's friction,
					// and riding that body's motion feeds it back: a loose box spins up and throws it off.
					// (Limited for a glancing hit, where the same gap would cost a long way back along the motion.)
					const PxReal backoff = margin / PxMax(approach, 0.25f);
					const real_t frac = CLAMP((real_t)(hit.block.distance - backoff) / motion_len, (real_t)0.0, (real_t)1.0);
					if (frac < safe_fraction) {
						safe_fraction = frac;
						best_hit = hit.block;
						has_hit = true;
					}
				}
			}
		}
		if (!has_hit && resting) {
			best_hit = resting_hit;
			has_hit = true;
		}
	}

	const PxVec3 total_recover = recover_motion + mtd_recover;

	if (r_result) {
		r_result->travel = to_godot(total_recover) + p_params.motion * safe_fraction;
		r_result->remainder = p_params.motion - p_params.motion * safe_fraction;
		r_result->collision_safe_fraction = safe_fraction;
		r_result->collision_unsafe_fraction = safe_fraction;

		const bool recovery_hit = p_params.recovery_as_collision && rec_actor && rec_depth > (PxReal)CMP_EPSILON;

		if (has_hit) {
			GodotPhysXBody3D *other = static_cast<GodotPhysXBody3D *>(best_hit.actor->userData);
			PhysicsServer3D::MotionCollision &c = r_result->collisions[0];
			c.position = to_godot(best_hit.position);
			c.normal = to_godot(best_hit.normal);
			c.collider = other ? other->get_self() : RID();
			c.collider_id = other ? other->get_instance_id() : ObjectID();
			c.collider_velocity = other ? other->get_linear_velocity() : Vector3();
			c.local_shape = 0;
			c.collider_shape = best_hit.shape ? (int)reinterpret_cast<uintptr_t>(best_hit.shape->userData) : 0;
			c.depth = margin;
			r_result->collision_count = 1;
			r_result->collision_depth = margin;
		} else if (recovery_hit) {
			GodotPhysXBody3D *other = static_cast<GodotPhysXBody3D *>(rec_actor->userData);
			PhysicsServer3D::MotionCollision &c = r_result->collisions[0];
			c.position = to_godot(rec_point);
			c.normal = to_godot(rec_normal);
			c.collider = other ? other->get_self() : RID();
			c.collider_id = other ? other->get_instance_id() : ObjectID();
			c.collider_velocity = other ? other->get_linear_velocity() : Vector3();
			c.local_shape = 0;
			c.collider_shape = rec_shape ? (int)reinterpret_cast<uintptr_t>(rec_shape->userData) : 0;
			c.depth = rec_depth;
			r_result->collision_count = 1;
			r_result->collision_depth = rec_depth;
		}

		return has_hit || recovery_hit;
	}

	return has_hit || (p_params.recovery_as_collision && rec_depth > (PxReal)CMP_EPSILON);
}

void GodotPhysXSpace3D::_apply_separation_rays(real_t p_step) {
	if (separation_ray_bodies.is_empty() || p_step <= 0.0) {
		return;
	}
	// Each ray whose tip is in something gets an impulse at the contact, like
	// an inelastic solver contact along the ray (or the surface normal with
	// slide_on_slope), with friction: it stops the tip sinking further,
	// and lifts out only the depth past a small slop, a fraction per step
	// (Baumgarte). Lifting it out in one step kicks a body hard enough to flip
	// it when one ray meets a bump, and a bigger lift than that keeps pumping
	// energy in -- a sled on four rays rocked over a bump forever. Friction
	// (the two materials' coefficients combined as on Jolt, sqrt(a * b)) then
	// stops the contact sliding, up to mu times that push. Whatever it stands
	// on gets the opposite impulse if it's dynamic.
	constexpr PxReal SLOP = 0.02f; // m, as Jolt's penetration slop
	constexpr PxReal BAUMGARTE = 0.2f;
	constexpr PxReal MAX_SEPARATION_SPEED = 4.0f; // m/s
	for (GodotPhysXBody3D *body : separation_ray_bodies) {
		PxRigidDynamic *dyn = body->get_px_actor() ? body->get_px_actor()->is<PxRigidDynamic>() : nullptr;
		if (!dyn || !dyn->getScene() || dyn->isSleeping()) {
			continue;
		}
		MotionFilter filter;
		filter.self_actor = dyn;
		filter.self_body = body;
		filter.self_layer = body->get_collision_layer();
		filter.self_mask = body->get_collision_mask();
		const PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER);

		const PxTransform pose = dyn->getGlobalPose();
		const PxVec3 com = pose.transform(dyn->getCMassLocalPose().p);
		const Vector3 scale = body->get_transform().basis.get_scale();
		for (int i = 0; i < body->get_shape_count(); i++) {
			const GodotPhysXBody3D::ShapeRef *sr = body->get_shape_ref(i);
			if (!sr || sr->disabled || !sr->shape || !sr->shape->is_separation_ray()) {
				continue;
			}
			const SeparationRay ray = separation_ray(sr->shape, pose, sr->xform, scale);
			PxRaycastBuffer rh;
			PxVec3 n;
			PxReal amount;
			if (!px_scene->raycast(ray.origin, ray.dir, ray.length, rh, PxHitFlag::ePOSITION | PxHitFlag::eNORMAL, fd, &filter) ||
					!rh.hasBlock || !separation_ray_push(ray, rh.block, n, amount)) {
				continue;
			}
			const PxVec3 contact = rh.block.position;
			const PxVec3 r = contact - com;
			// Velocity change at the contact, along n, per unit impulse.
			PxVec3 dl, da;
			PxRigidBodyExt::computeVelocityDeltaFromImpulse(*dyn, n, r.cross(n), dl, da);
			const PxReal k = dl.dot(n) + da.cross(r).dot(n);
			if (k <= 0.0f) {
				continue;
			}
			const PxReal vn = (dyn->getLinearVelocity() + dyn->getAngularVelocity().cross(r)).dot(n);
			const PxReal target = MIN(MAX(amount - SLOP, 0.0f) * BAUMGARTE / (PxReal)p_step, MAX_SEPARATION_SPEED);
			if (vn >= target) {
				continue;
			}
			const PxReal jn = (target - vn) / k;
			PxVec3 total = n * jn;

			// Friction: cancel the contact's sliding velocity (as it will be
			// after the push), at most mu * jn.
			PxReal mu = (PxReal)(real_t)body->get_param(PhysicsServer3D::BODY_PARAM_FRICTION);
			PxMaterial *other_material = nullptr;
			if (rh.block.shape && rh.block.shape->getNbMaterials() > 0) {
				rh.block.shape->getMaterials(&other_material, 1);
			}
			mu = other_material ? PxSqrt(MAX(mu, 0.0f) * MAX(other_material->getDynamicFriction(), 0.0f)) : MAX(mu, 0.0f);
			const PxVec3 v_after = dyn->getLinearVelocity() + dl * jn + (dyn->getAngularVelocity() + da * jn).cross(r);
			PxVec3 vt = v_after - n * v_after.dot(n);
			const PxReal vt_len = vt.magnitude();
			if (mu > 0.0f && vt_len > 1e-4f) {
				const PxVec3 t = vt / vt_len;
				PxVec3 tl, ta;
				PxRigidBodyExt::computeVelocityDeltaFromImpulse(*dyn, t, r.cross(t), tl, ta);
				const PxReal kt = tl.dot(t) + ta.cross(r).dot(t);
				if (kt > 0.0f) {
					total -= t * MIN(vt_len / kt, mu * jn);
				}
			}

			PxRigidBodyExt::addForceAtPos(*dyn, total, contact, PxForceMode::eIMPULSE, false);
			if (PxRigidDynamic *other = rh.block.actor ? rh.block.actor->is<PxRigidDynamic>() : nullptr) {
				if (!(other->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
					PxRigidBodyExt::addForceAtPos(*other, -total, contact, PxForceMode::eIMPULSE, true);
				}
			}
		}
	}
}

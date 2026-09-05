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
#include "../shapes/godot_physx_shape_3d.h"
#include "godot_physx_direct_state_3d.h"

#include "core/error/error_macros.h"
#include "core/math/math_defs.h"
#include "core/object/object.h"

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

	pair_flags = PxPairFlag::eCONTACT_DEFAULT;
	if ((filter_data0.word2 | filter_data1.word2) & 1u) {
		pair_flags |= PxPairFlag::eNOTIFY_TOUCH_FOUND |
				PxPairFlag::eNOTIFY_TOUCH_PERSISTS |
				PxPairFlag::eNOTIFY_TOUCH_LOST |
				PxPairFlag::eNOTIFY_CONTACT_POINTS;
	}
	return PxFilterFlag::eDEFAULT;
}

// Stateless: reaches bodies through PxActor::userData, so one instance is shared
// by every scene.
class ContactCallback : public PxSimulationEventCallback {
public:
	virtual void onContact(const PxContactPairHeader &p_header, const PxContactPair *p_pairs, PxU32 p_nb_pairs) override {
		GodotPhysXBody3D *b0 = actor_body(p_header.actors[0]);
		GodotPhysXBody3D *b1 = actor_body(p_header.actors[1]);
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
			if (!area || !body) {
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

} //namespace

GodotPhysXSpace3D::GodotPhysXSpace3D(PxPhysics *p_physics, PxDefaultCpuDispatcher *p_dispatcher, PxCudaContextManager *p_cuda) {
	px_physics = p_physics;
	ERR_FAIL_NULL(px_physics);

	PxSceneDesc scene_desc(px_physics->getTolerancesScale());
	scene_desc.gravity = to_px(gravity);
	scene_desc.cpuDispatcher = p_dispatcher;
	scene_desc.filterShader = godot_physx_filter_shader;
	scene_desc.simulationEventCallback = &g_contact_callback;
	scene_desc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;
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
		// Non-zero so PxDeformableSurface (cloth) can generate contacts; a
		// surface touching anything with this at 0 overflows and kills GPU sim.
		scene_desc.gpuDynamicsConfig.maxDeformableSurfaceContacts = 512 * 1024;
		scene_desc.gpuDynamicsConfig.maxDeformableVolumeContacts = 0;
		scene_desc.gpuDynamicsConfig.maxParticleContacts = 1 * 1024 * 1024;
		gpu_enabled = true;
	}

	px_scene = px_physics->createScene(scene_desc);
	ERR_FAIL_NULL(px_scene);

	default_material = px_physics->createMaterial(0.5f, 0.5f, 0.0f);
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

	_apply_area_overrides();

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
		GodotPhysXBody3D *body = static_cast<GodotPhysXBody3D *>(active[i]->userData);
		if (!body) {
			continue;
		}
		body->pull_transform_from_px();
		now_awake.insert(body);
		sync_bodies.push_back(body);
	}

	for (GodotPhysXParticleFluid3D *fluid : fluids) {
		fluid->read_back();
	}
	for (GodotPhysXCloth3D *cloth : cloths) {
		cloth->read_back();
	}

	for (GodotPhysXBody3D *body : awake_bodies) {
		if (!now_awake.has(body)) { // active last step, asleep now
			body->pull_transform_from_px();
			sync_bodies.push_back(body);
		}
	}

	awake_bodies = now_awake;
}

void GodotPhysXSpace3D::body_removed_from_areas(GodotPhysXBody3D *p_body) {
	for (GodotPhysXArea3D *area : areas) {
		area->body_removed(p_body);
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
			affected[body].push_back(area);
		}
	}

	for (KeyValue<GodotPhysXBody3D *, LocalVector<GodotPhysXArea3D *>> &E : affected) {
		GodotPhysXBody3D *body = E.key;
		LocalVector<GodotPhysXArea3D *> &list = E.value;
		list.sort_custom<AreaPriorityCompare>();

		const Vector3 pos = body->get_transform().origin;
		const real_t mass = MAX(body->get_mass(), (real_t)0.0001);

		Vector3 grav = gravity;
		real_t lin_damp = 0.0;
		real_t ang_damp = 0.0;
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

		// Gravity delta relative to the world default (bodies already get world
		// gravity from the scene), plus wind, plus a velocity-proportional drag
		// standing in for the area's damping contribution.
		Vector3 force = (grav - gravity) * mass + wind;
		force += -lin_damp * mass * body->get_linear_velocity();
		Vector3 torque = -ang_damp * mass * body->get_angular_velocity();

		if (!force.is_zero_approx()) {
			body->apply_central_force(force);
		}
		if (!torque.is_zero_approx()) {
			body->apply_torque(torque);
		}
	}
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

	MotionFilter filter;
	filter.self_actor = p_body->get_px_actor();
	filter.exclude_bodies = &p_params.exclude_bodies;
	filter.exclude_objects = &p_params.exclude_objects;
	filter.self_layer = p_body->get_collision_layer();
	filter.self_mask = p_body->get_collision_mask();
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER);

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
			if (!sr || !sr->shape || !sr->shape->is_valid()) {
				continue;
			}
			const GodotPhysXShapeGeometry &g = sr->shape->get_geometry();
			const PxTransform pose = PxTransform(recover) * to_px(p_params.from) * to_px(sr->xform) * g.local_pose;

			PxOverlapHit touches[16];
			PxOverlapBuffer buf(touches, 16);
			PxQueryFilterData ofd(fd.flags | PxQueryFlag::eNO_BLOCK);
			px_scene->overlap(g.geometry(), pose, buf, ofd, &filter);
			for (PxU32 t = 0; t < buf.getNbTouches(); t++) {
				const PxOverlapHit &h = buf.getTouch(t);
				PxVec3 dir;
				PxF32 depth;
				const PxTransform other_pose = h.actor->getGlobalPose() * h.shape->getLocalPose();
				if (PxGeometryQuery::computePenetration(dir, depth, g.geometry(), pose, h.shape->getGeometry(), other_pose)) {
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

	if (motion_len > CMP_EPSILON) {
		const PxVec3 unit_dir = to_px(p_params.motion / motion_len);
		for (int i = 0; i < shape_count; i++) {
			const GodotPhysXBody3D::ShapeRef *sr = p_body->get_shape_ref(i);
			if (!sr || !sr->shape || !sr->shape->is_valid()) {
				continue;
			}
			const GodotPhysXShapeGeometry &g = sr->shape->get_geometry();
			const PxTransform pose = recovered_from * to_px(sr->xform) * g.local_pose;

			PxSweepBuffer hit;
			if (px_scene->sweep(g.geometry(), pose, unit_dir, (PxReal)motion_len, hit,
						PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::ePRECISE_SWEEP,
						fd, &filter, nullptr, 0.0f) &&
					hit.hasBlock) {
				// Ignore contacts that don't actually oppose the motion: a hit
				// whose normal faces along (rather than against) the sweep is a
				// grazing/edge contact from sliding on a surface.
				if (hit.block.normal.dot(-unit_dir) < 0.001f) {
					continue;
				}
				const real_t frac = CLAMP((real_t)hit.block.distance / motion_len, (real_t)0.0, (real_t)1.0);
				if (frac < safe_fraction) {
					safe_fraction = frac;
					best_hit = hit.block;
					has_hit = true;
				}
			}
		}
	}

	if (r_result) {
		r_result->travel = to_godot(recover_motion) + p_params.motion * safe_fraction;
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

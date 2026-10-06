/**************************************************************************/
/*  godot_physx_direct_state_3d.cpp                                       */
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

#include "godot_physx_direct_state_3d.h"

#include "../godot_physx_conversions.h"
#include "../godot_physx_server_3d.h"
#include "../objects/godot_physx_area_3d.h"
#include "../objects/godot_physx_body_3d.h"
#include "../shapes/godot_physx_shape_3d.h"
#include "godot_physx_space_3d.h"

#include "core/error/error_macros.h"
#include "core/math/math_defs.h"
#include "core/object/object.h"
#include "core/templates/hash_set.h"
#include "core/templates/local_vector.h"

#include <PxPhysicsAPI.h>

using namespace physx;

/* ------------------------------------------------------------------------ */
/*  Direct body state                                                       */
/* ------------------------------------------------------------------------ */

Vector3 GodotPhysXDirectBodyState3D::get_total_gravity() const {
	GodotPhysXSpace3D *sp = body->get_space();
	return sp ? sp->get_gravity() * body->get_gravity_scale() : Vector3();
}

real_t GodotPhysXDirectBodyState3D::get_total_linear_damp() const {
	return body->get_total_linear_damp();
}

real_t GodotPhysXDirectBodyState3D::get_total_angular_damp() const {
	return body->get_total_angular_damp();
}

Vector3 GodotPhysXDirectBodyState3D::get_center_of_mass() const {
	return body->get_center_of_mass_relative();
}

Vector3 GodotPhysXDirectBodyState3D::get_center_of_mass_local() const {
	return body->get_center_of_mass_local();
}

Basis GodotPhysXDirectBodyState3D::get_principal_inertia_axes() const {
	return body->get_principal_inertia_axes();
}

Vector3 GodotPhysXDirectBodyState3D::get_inverse_inertia() const {
	return body->get_inverse_inertia();
}

real_t GodotPhysXDirectBodyState3D::get_inverse_mass() const {
	const real_t m = body->get_mass();
	return m > 0.0 ? 1.0 / m : 0.0;
}

Basis GodotPhysXDirectBodyState3D::get_inverse_inertia_tensor() const {
	using namespace physx;
	if (PxRigidActor *actor = body->get_px_actor()) {
		if (PxRigidDynamic *dyn = actor->is<PxRigidDynamic>()) {
			// PhysX stores inertia diagonalized in the body's own principal-axis
			// frame (getCMassLocalPose(), not necessarily the actor's own local
			// axes); anything driving a body purely through
			// PhysicsDirectBodyState3D (e.g. VehicleBody3D's wheel friction)
			// needs the WORLD-space tensor this base class's contract promises
			// -- an identity stub here silently fed a wrong-but-plausible value
			// into any such calculation instead of the real one. A real repro
			// (VehicleBody3D on this backend) confirmed this specific stub was
			// NOT the cause of two separate observed bugs (a stalled car, a
			// runaway steering spin -- both traced to unrelated causes; see
			// their own commits/notes) -- swapping this function between the
			// identity stub and this real computation produced byte-identical
			// results in both repros. Kept anyway: it's still a real, confirmed
			// gap against the base class's own documented contract, and a
			// wrong value here would matter for any scenario with real
			// rotational coupling neither repro happened to exercise.
			const PxVec3 inv_inertia = dyn->getMassSpaceInvInertiaTensor();
			const PxQuat principal_rot = dyn->getGlobalPose().q * dyn->getCMassLocalPose().q;
			const Basis r(to_godot(principal_rot));
			Basis diag;
			diag[0] = Vector3(inv_inertia.x, 0, 0);
			diag[1] = Vector3(0, inv_inertia.y, 0);
			diag[2] = Vector3(0, 0, inv_inertia.z);
			return r * diag * r.transposed();
		}
	}
	return Basis();
}

void GodotPhysXDirectBodyState3D::set_linear_velocity(const Vector3 &p_velocity) {
	body->set_linear_velocity(p_velocity);
}

Vector3 GodotPhysXDirectBodyState3D::get_linear_velocity() const {
	return body->get_linear_velocity();
}

void GodotPhysXDirectBodyState3D::set_angular_velocity(const Vector3 &p_velocity) {
	body->set_angular_velocity(p_velocity);
}

Vector3 GodotPhysXDirectBodyState3D::get_angular_velocity() const {
	return body->get_angular_velocity();
}

void GodotPhysXDirectBodyState3D::set_transform(const Transform3D &p_transform) {
	body->set_state(PhysicsServer3D::BODY_STATE_TRANSFORM, p_transform);
}

Transform3D GodotPhysXDirectBodyState3D::get_transform() const {
	return body->get_transform();
}

Vector3 GodotPhysXDirectBodyState3D::get_velocity_at_local_position(const Vector3 &p_position) const {
	// p_position is an offset from the body origin in world axes
	// (CharacterBody3D passes the floor contact point minus the platform's
	// origin).
	return body->get_velocity_at_position(body->get_transform().origin + p_position);
}

void GodotPhysXDirectBodyState3D::apply_central_impulse(const Vector3 &p_impulse) {
	body->apply_central_impulse(p_impulse);
}

void GodotPhysXDirectBodyState3D::apply_impulse(const Vector3 &p_impulse, const Vector3 &p_position) {
	body->apply_impulse(p_impulse, p_position);
}

void GodotPhysXDirectBodyState3D::apply_torque_impulse(const Vector3 &p_impulse) {
	body->apply_torque_impulse(p_impulse);
}

void GodotPhysXDirectBodyState3D::apply_central_force(const Vector3 &p_force) {
	body->apply_central_force(p_force);
}

void GodotPhysXDirectBodyState3D::apply_force(const Vector3 &p_force, const Vector3 &p_position) {
	body->apply_force(p_force, p_position);
}

void GodotPhysXDirectBodyState3D::apply_torque(const Vector3 &p_torque) {
	body->apply_torque(p_torque);
}

void GodotPhysXDirectBodyState3D::add_constant_central_force(const Vector3 &p_force) {
	body->add_constant_central_force(p_force);
}

void GodotPhysXDirectBodyState3D::add_constant_force(const Vector3 &p_force, const Vector3 &p_position) {
	body->add_constant_force(p_force, p_position);
}

void GodotPhysXDirectBodyState3D::add_constant_torque(const Vector3 &p_torque) {
	body->add_constant_torque(p_torque);
}

void GodotPhysXDirectBodyState3D::set_constant_force(const Vector3 &p_force) {
	body->set_constant_force(p_force);
}

Vector3 GodotPhysXDirectBodyState3D::get_constant_force() const {
	return body->get_constant_force();
}

void GodotPhysXDirectBodyState3D::set_constant_torque(const Vector3 &p_torque) {
	body->set_constant_torque(p_torque);
}

Vector3 GodotPhysXDirectBodyState3D::get_constant_torque() const {
	return body->get_constant_torque();
}

void GodotPhysXDirectBodyState3D::set_collision_layer(uint32_t p_layer) {
	body->set_collision_layer(p_layer);
}

uint32_t GodotPhysXDirectBodyState3D::get_collision_layer() const {
	return body->get_collision_layer();
}

void GodotPhysXDirectBodyState3D::set_collision_mask(uint32_t p_mask) {
	body->set_collision_mask(p_mask);
}

uint32_t GodotPhysXDirectBodyState3D::get_collision_mask() const {
	return body->get_collision_mask();
}

void GodotPhysXDirectBodyState3D::set_sleep_state(bool p_sleep) {
	body->set_sleep_state(p_sleep);
}

bool GodotPhysXDirectBodyState3D::is_sleeping() const {
	return body->is_sleeping();
}

real_t GodotPhysXDirectBodyState3D::get_step() const {
	GodotPhysXSpace3D *sp = body->get_space();
	return sp ? sp->get_last_step() : 0.0;
}

RequiredResult<PhysicsDirectSpaceState3D> GodotPhysXDirectBodyState3D::get_space_state() {
	return body->get_space()->get_direct_state();
}

int GodotPhysXDirectBodyState3D::get_contact_count() const {
	return body->get_contact_count();
}

Vector3 GodotPhysXDirectBodyState3D::get_contact_local_position(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), Vector3());
	return body->get_contact(p_contact_idx).position;
}

Vector3 GodotPhysXDirectBodyState3D::get_contact_local_normal(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), Vector3());
	return body->get_contact(p_contact_idx).normal;
}

Vector3 GodotPhysXDirectBodyState3D::get_contact_impulse(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), Vector3());
	return body->get_contact(p_contact_idx).impulse;
}

int GodotPhysXDirectBodyState3D::get_contact_local_shape(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), 0);
	return body->get_contact(p_contact_idx).local_shape;
}

Vector3 GodotPhysXDirectBodyState3D::get_contact_local_velocity_at_position(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), Vector3());
	return body->get_linear_velocity();
}

RID GodotPhysXDirectBodyState3D::get_contact_collider(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), RID());
	return body->get_contact(p_contact_idx).collider;
}

Vector3 GodotPhysXDirectBodyState3D::get_contact_collider_position(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), Vector3());
	return body->get_contact(p_contact_idx).position;
}

ObjectID GodotPhysXDirectBodyState3D::get_contact_collider_id(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), ObjectID());
	return body->get_contact(p_contact_idx).collider_id;
}

int GodotPhysXDirectBodyState3D::get_contact_collider_shape(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), 0);
	return body->get_contact(p_contact_idx).collider_shape;
}

Vector3 GodotPhysXDirectBodyState3D::get_contact_collider_velocity_at_position(int p_contact_idx) const {
	ERR_FAIL_INDEX_V(p_contact_idx, body->get_contact_count(), Vector3());
	return body->get_contact(p_contact_idx).collider_velocity;
}

/* ------------------------------------------------------------------------ */
/*  Direct space state (scene queries)                                      */
/* ------------------------------------------------------------------------ */

namespace {

_FORCE_INLINE_ GodotPhysXBody3D *body_of(const PxActor *p_actor) {
	return p_actor ? static_cast<GodotPhysXBody3D *>(p_actor->userData) : nullptr;
}

_FORCE_INLINE_ int shape_index_of(const PxShape *p_shape) {
	return p_shape ? (int)reinterpret_cast<uintptr_t>(p_shape->userData) : 0;
}

// Area3D shapes are the only trigger shapes, and an area's actor carries the
// GodotPhysXArea3D in userData (a body's carries the GodotPhysXBody3D).
_FORCE_INLINE_ bool is_area_shape(const PxShape *p_shape) {
	return p_shape && (p_shape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE);
}

_FORCE_INLINE_ GodotPhysXArea3D *area_of(const PxActor *p_actor) {
	return p_actor ? static_cast<GodotPhysXArea3D *>(p_actor->userData) : nullptr;
}

// What a query hit: a body or an area.
struct QueryHitObject {
	RID rid;
	ObjectID instance_id;
	GodotPhysXBody3D *body = nullptr;
};

QueryHitObject hit_object(const PxRigidActor *p_actor, const PxShape *p_shape) {
	QueryHitObject o;
	if (is_area_shape(p_shape)) {
		if (GodotPhysXArea3D *area = area_of(p_actor)) {
			o.rid = area->get_self();
			o.instance_id = area->get_instance_id();
		}
	} else if (GodotPhysXBody3D *body = body_of(p_actor)) {
		o.rid = body->get_self();
		o.instance_id = body->get_instance_id();
		o.body = body;
	}
	return o;
}

// Applies Godot's collision mask, RID exclude list, collide_with_bodies /
// collide_with_areas, and -- for the viewport's picking ray -- the objects'
// input_ray_pickable.
class QueryFilter : public PxQueryFilterCallback {
public:
	const HashSet<RID> *exclude = nullptr;
	uint32_t collision_mask = UINT32_MAX;
	bool collide_with_bodies = true;
	bool collide_with_areas = false;
	bool picking = false;

	template <typename T>
	void set_from(const T &p_parameters) {
		exclude = &p_parameters.exclude;
		collision_mask = p_parameters.collision_mask;
		collide_with_bodies = p_parameters.collide_with_bodies;
		collide_with_areas = p_parameters.collide_with_areas;
	}

	virtual PxQueryHitType::Enum preFilter(const PxFilterData &, const PxShape *p_shape, const PxRigidActor *p_actor, PxHitFlags &) override {
		RID self;
		uint32_t layer = 0;
		bool pickable = true;
		if (is_area_shape(p_shape)) {
			const GodotPhysXArea3D *a = collide_with_areas ? area_of(p_actor) : nullptr;
			if (!a) {
				return PxQueryHitType::eNONE;
			}
			self = a->get_self();
			layer = a->get_collision_layer();
			pickable = a->is_ray_pickable();
		} else {
			const GodotPhysXBody3D *b = collide_with_bodies ? body_of(p_actor) : nullptr;
			if (!b) {
				return PxQueryHitType::eNONE;
			}
			self = b->get_self();
			layer = b->get_collision_layer();
			pickable = b->is_ray_pickable();
		}
		if ((layer & collision_mask) == 0) {
			return PxQueryHitType::eNONE;
		}
		if (picking && !pickable) {
			return PxQueryHitType::eNONE;
		}
		if (exclude && exclude->has(self)) {
			return PxQueryHitType::eNONE;
		}
		return PxQueryHitType::eBLOCK;
	}

	virtual PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &, const PxShape *, const PxRigidActor *) override {
		return PxQueryHitType::eBLOCK;
	}
};

const GodotPhysXShape3D *godot_shape_of(const PxRigidActor *p_actor, const PxShape *p_shape) {
	const GodotPhysXBody3D *body = is_area_shape(p_shape) ? nullptr : body_of(p_actor);
	if (!body || !p_shape) {
		return nullptr;
	}
	const GodotPhysXBody3D::ShapeRef *sr = body->get_shape_ref(shape_index_of(p_shape));
	return sr ? sr->shape : nullptr;
}

// Ray queries: a trimesh back face only counts when the shape has
// backface_collision on AND the query asks for back faces (Jolt's rule).
// eMESH_BOTH_SIDES (needed for hit_back_faces on height fields) would let a
// ray hit any mesh from behind, and a back-face mesh's flipped copies would
// let any ray hit it from behind -- this sorts those hits out.
class RayQueryFilter : public QueryFilter {
public:
	PxVec3 ray_dir;
	bool hit_back_faces = true;

	virtual PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &p_hit, const PxShape *p_shape, const PxRigidActor *p_actor) override {
		if (!p_shape || !p_actor || p_hit.faceIndex == 0xFFFFFFFF || p_shape->getGeometry().getType() != PxGeometryType::eTRIANGLEMESH) {
			return PxQueryHitType::eBLOCK;
		}
		const GodotPhysXShape3D *shape = godot_shape_of(p_actor, p_shape);
		if (!shape) {
			return PxQueryHitType::eBLOCK;
		}
		if (shape->has_backface_collision()) {
			if (hit_back_faces) {
				return PxQueryHitType::eBLOCK;
			}
			bool back = false;
			shape->source_face_index(p_hit.faceIndex, back);
			return back ? PxQueryHitType::eNONE : PxQueryHitType::eBLOCK;
		}
		if (!hit_back_faces) {
			return PxQueryHitType::eBLOCK; // one-sided query: PhysX only reported front hits
		}
		PxTriangle tri;
		PxMeshQuery::getTriangle(static_cast<const PxTriangleMeshGeometry &>(p_shape->getGeometry()),
				p_actor->getGlobalPose() * p_shape->getLocalPose(), p_hit.faceIndex, tri);
		PxVec3 face_n;
		tri.normal(face_n);
		return face_n.dot(ray_dir) > 0.0f ? PxQueryHitType::eNONE : PxQueryHitType::eBLOCK;
	}
};

GodotPhysXShape3D *query_shape(RID p_shape_rid) {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server) {
		return nullptr;
	}
	GodotPhysXShape3D *shape = server->get_shape(p_shape_rid);
	if (!shape || !shape->is_valid()) {
		return nullptr;
	}
	return shape;
}

} //namespace

bool GodotPhysXDirectSpaceState3D::intersect_ray(const RayParameters &p_parameters, RayResult &r_result) {
	PxScene *scene = space ? space->get_px_scene() : nullptr;
	ERR_FAIL_NULL_V(scene, false);

	const Vector3 delta = p_parameters.to - p_parameters.from;
	const real_t dist = delta.length();
	if (dist <= CMP_EPSILON) {
		return false;
	}

	RayQueryFilter filter;
	filter.set_from(p_parameters);
	filter.picking = p_parameters.pick_ray;
	filter.ray_dir = to_px(delta / dist);
	filter.hit_back_faces = p_parameters.hit_back_faces;

	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::ePOSTFILTER);
	PxHitFlags hit_flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
	if (p_parameters.hit_back_faces) {
		hit_flags |= PxHitFlag::eMESH_BOTH_SIDES;
	}

	PxRaycastBuffer hit;
	const bool has_hit = scene->raycast(to_px(p_parameters.from), to_px(delta / dist), (PxReal)dist, hit, hit_flags, fd, &filter);
	if (!has_hit || !hit.hasBlock) {
		return false;
	}

	const PxRaycastHit &b = hit.block;
	const QueryHitObject o = hit_object(b.actor, b.shape);
	r_result.position = to_godot(b.position);
	r_result.normal = to_godot(b.normal);
	r_result.rid = o.rid;
	r_result.collider_id = o.instance_id;
	r_result.collider = r_result.collider_id.is_valid() ? ObjectDB::get_instance(r_result.collider_id) : nullptr;
	r_result.shape = shape_index_of(b.shape);
	r_result.face_index = (b.faceIndex == 0xFFFFFFFF) ? -1 : (int)b.faceIndex;
	if (r_result.face_index >= 0 && b.shape && b.shape->getGeometry().getType() == PxGeometryType::eTRIANGLEMESH) {
		// Godot's face order, not the cooked one.
		if (const GodotPhysXShape3D *shape = godot_shape_of(b.actor, b.shape)) {
			bool back = false;
			r_result.face_index = shape->source_face_index(b.faceIndex, back);
		}
	}
	return true;
}

int GodotPhysXDirectSpaceState3D::intersect_point(const PointParameters &p_parameters, ShapeResult *r_results, int p_result_max) {
	PxScene *scene = space ? space->get_px_scene() : nullptr;
	ERR_FAIL_NULL_V(scene, 0);
	if (p_result_max <= 0) {
		return 0;
	}

	QueryFilter filter;
	filter.set_from(p_parameters);
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK);

	LocalVector<PxOverlapHit> touches;
	touches.resize(p_result_max);
	PxOverlapBuffer buf(touches.ptr(), (PxU32)p_result_max);

	// A near-zero sphere approximates a point overlap.
	const PxSphereGeometry probe(0.001f);
	scene->overlap(probe, PxTransform(to_px(p_parameters.position)), buf, fd, &filter);

	int count = 0;
	for (PxU32 i = 0; i < buf.getNbTouches() && count < p_result_max; i++) {
		const QueryHitObject o = hit_object(buf.getTouch(i).actor, buf.getTouch(i).shape);
		if (!o.rid.is_valid()) {
			continue;
		}
		r_results[count].rid = o.rid;
		r_results[count].collider_id = o.instance_id;
		r_results[count].collider = r_results[count].collider_id.is_valid() ? ObjectDB::get_instance(r_results[count].collider_id) : nullptr;
		r_results[count].shape = shape_index_of(buf.getTouch(i).shape);
		count++;
	}
	return count;
}

int GodotPhysXDirectSpaceState3D::intersect_shape(const ShapeParameters &p_parameters, ShapeResult *r_results, int p_result_max) {
	PxScene *scene = space ? space->get_px_scene() : nullptr;
	ERR_FAIL_NULL_V(scene, 0);
	GodotPhysXShape3D *shape = query_shape(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(shape, 0);
	const GodotPhysXShape3D::ScaledGeometry g = shape->scaled_geometry(p_parameters.transform.basis.get_scale());
	if (p_result_max <= 0) {
		return 0;
	}

	QueryFilter filter;
	filter.set_from(p_parameters);
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK);

	LocalVector<PxOverlapHit> touches;
	touches.resize(p_result_max);
	PxOverlapBuffer buf(touches.ptr(), (PxU32)p_result_max);

	const PxTransform pose = to_px(p_parameters.transform) * g.local_pose;
	scene->overlap(g.geom.any(), pose, buf, fd, &filter);

	int count = 0;
	for (PxU32 i = 0; i < buf.getNbTouches() && count < p_result_max; i++) {
		const QueryHitObject o = hit_object(buf.getTouch(i).actor, buf.getTouch(i).shape);
		if (!o.rid.is_valid()) {
			continue;
		}
		r_results[count].rid = o.rid;
		r_results[count].collider_id = o.instance_id;
		r_results[count].collider = r_results[count].collider_id.is_valid() ? ObjectDB::get_instance(r_results[count].collider_id) : nullptr;
		r_results[count].shape = shape_index_of(buf.getTouch(i).shape);
		count++;
	}
	return count;
}

bool GodotPhysXDirectSpaceState3D::cast_motion(const ShapeParameters &p_parameters, real_t &p_closest_safe, real_t &p_closest_unsafe, ShapeRestInfo *r_info) {
	p_closest_safe = 1.0;
	p_closest_unsafe = 1.0;

	PxScene *scene = space ? space->get_px_scene() : nullptr;
	ERR_FAIL_NULL_V(scene, false);
	GodotPhysXShape3D *shape = query_shape(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(shape, false);
	const GodotPhysXShape3D::ScaledGeometry g = shape->scaled_geometry(p_parameters.transform.basis.get_scale());

	const real_t motion_len = p_parameters.motion.length();
	if (motion_len <= CMP_EPSILON) {
		return true;
	}

	QueryFilter filter;
	filter.set_from(p_parameters);
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER);

	const PxTransform pose = to_px(p_parameters.transform) * g.local_pose;
	PxSweepBuffer hit;
	const bool has_hit = scene->sweep(g.geom.any(), pose, to_px(p_parameters.motion / motion_len),
			(PxReal)motion_len, hit, PxHitFlag::ePOSITION | PxHitFlag::eNORMAL, fd, &filter);

	if (!has_hit || !hit.hasBlock) {
		return true;
	}

	const real_t frac = CLAMP((real_t)hit.block.distance / motion_len, (real_t)0.0, (real_t)1.0);
	p_closest_safe = frac;
	p_closest_unsafe = frac;
	if (r_info) {
		const QueryHitObject o = hit_object(hit.block.actor, hit.block.shape);
		r_info->point = to_godot(hit.block.position);
		r_info->normal = to_godot(hit.block.normal);
		r_info->rid = o.rid;
		r_info->collider_id = o.instance_id;
		r_info->shape = shape_index_of(hit.block.shape);
		r_info->linear_velocity = o.body ? o.body->get_linear_velocity() : Vector3();
	}
	return true;
}

bool GodotPhysXDirectSpaceState3D::collide_shape(const ShapeParameters &p_parameters, Vector3 *r_results, int p_result_max, int &r_result_count) {
	r_result_count = 0;
	PxScene *scene = space ? space->get_px_scene() : nullptr;
	ERR_FAIL_NULL_V(scene, false);
	GodotPhysXShape3D *shape = query_shape(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(shape, false);
	const GodotPhysXShape3D::ScaledGeometry g = shape->scaled_geometry(p_parameters.transform.basis.get_scale());
	if (p_result_max <= 0) {
		return false;
	}

	QueryFilter filter;
	filter.set_from(p_parameters);
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK);

	const int max_hits = p_result_max / 2;
	LocalVector<PxOverlapHit> touches;
	touches.resize(MAX(max_hits, 1));
	PxOverlapBuffer buf(touches.ptr(), (PxU32)MAX(max_hits, 1));

	const PxTransform pose = to_px(p_parameters.transform) * g.local_pose;
	scene->overlap(g.geom.any(), pose, buf, fd, &filter);

	for (PxU32 i = 0; i < buf.getNbTouches() && r_result_count + 2 <= p_result_max; i++) {
		const PxOverlapHit &h = buf.getTouch(i);
		PxVec3 dir;
		PxF32 depth;
		if (PxGeometryQuery::computePenetration(dir, depth, g.geom.any(), pose,
					h.shape->getGeometry(), h.actor->getGlobalPose() * h.shape->getLocalPose())) {
			const Vector3 on_collider = to_godot(pose.p) - to_godot(dir) * depth;
			r_results[r_result_count++] = to_godot(pose.p); // point on the query shape
			r_results[r_result_count++] = on_collider; // point on the collider
		}
	}
	return r_result_count > 0;
}

bool GodotPhysXDirectSpaceState3D::rest_info(const ShapeParameters &p_parameters, ShapeRestInfo *r_info) {
	PxScene *scene = space ? space->get_px_scene() : nullptr;
	ERR_FAIL_NULL_V(scene, false);
	GodotPhysXShape3D *shape = query_shape(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(shape, false);
	const GodotPhysXShape3D::ScaledGeometry g = shape->scaled_geometry(p_parameters.transform.basis.get_scale());

	QueryFilter filter;
	filter.set_from(p_parameters);
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK);

	PxOverlapHit touch;
	PxOverlapBuffer buf(&touch, 1);
	const PxTransform pose = to_px(p_parameters.transform) * g.local_pose;
	scene->overlap(g.geom.any(), pose, buf, fd, &filter);
	if (buf.getNbTouches() == 0) {
		return false;
	}

	const PxOverlapHit &h = buf.getTouch(0);
	PxVec3 dir;
	PxF32 depth;
	const PxTransform collider_pose = h.actor->getGlobalPose() * h.shape->getLocalPose();
	if (!PxGeometryQuery::computePenetration(dir, depth, g.geom.any(), pose, h.shape->getGeometry(), collider_pose)) {
		return false;
	}

	const QueryHitObject o = hit_object(h.actor, h.shape);
	r_info->point = to_godot(pose.p) - to_godot(dir) * depth;
	r_info->normal = to_godot(dir);
	r_info->rid = o.rid;
	r_info->collider_id = o.instance_id;
	r_info->shape = shape_index_of(h.shape);
	r_info->linear_velocity = o.body ? o.body->get_linear_velocity() : Vector3();
	return true;
}

Vector3 GodotPhysXDirectSpaceState3D::get_closest_point_to_object_volume(RID p_object, const Vector3 p_point) const {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	ERR_FAIL_NULL_V(server, p_point);
	GodotPhysXBody3D *body = server->get_body(p_object);
	ERR_FAIL_NULL_V(body, p_point);
	PxRigidActor *actor = body->get_px_actor();
	ERR_FAIL_NULL_V(actor, p_point);

	real_t best_dist_sq = 1e30;
	Vector3 best = p_point;
	PxU32 nb = actor->getNbShapes();
	LocalVector<PxShape *> shapes;
	shapes.resize(nb);
	actor->getShapes(shapes.ptr(), nb);
	for (PxU32 i = 0; i < nb; i++) {
		const PxTransform pose = actor->getGlobalPose() * shapes[i]->getLocalPose();
		PxVec3 closest;
		// pointDistance() returns the SQUARE distance, 0.0 when the point is inside.
		const PxReal dist_sq = PxGeometryQuery::pointDistance(to_px(p_point), shapes[i]->getGeometry(), pose, &closest);
		if (dist_sq < 0.0f) {
			continue; // unsupported geometry
		}
		if ((real_t)dist_sq < best_dist_sq) {
			best_dist_sq = dist_sq;
			best = (dist_sq > 0.0f) ? to_godot(closest) : p_point;
		}
	}
	return best;
}

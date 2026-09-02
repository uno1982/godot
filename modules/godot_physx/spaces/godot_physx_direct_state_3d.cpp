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
	return body->get_linear_damp();
}

real_t GodotPhysXDirectBodyState3D::get_total_angular_damp() const {
	return body->get_angular_damp();
}

real_t GodotPhysXDirectBodyState3D::get_inverse_mass() const {
	const real_t m = body->get_mass();
	return m > 0.0 ? 1.0 / m : 0.0;
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
	// Approximated as a central force for now (ignores the torque arm).
	body->apply_central_force(p_force);
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

// Applies Godot's collision mask + RID exclude list. Bodies only for now
// (areas are not simulated).
class QueryFilter : public PxQueryFilterCallback {
public:
	const HashSet<RID> *exclude = nullptr;
	uint32_t collision_mask = UINT32_MAX;

	virtual PxQueryHitType::Enum preFilter(const PxFilterData &, const PxShape *p_shape, const PxRigidActor *p_actor, PxHitFlags &) override {
		// Skip area trigger shapes -- scene queries hit bodies only for now.
		if (p_shape && (p_shape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE)) {
			return PxQueryHitType::eNONE;
		}
		GodotPhysXBody3D *b = body_of(p_actor);
		if (!b) {
			return PxQueryHitType::eNONE;
		}
		if ((b->get_collision_layer() & collision_mask) == 0) {
			return PxQueryHitType::eNONE;
		}
		if (exclude && exclude->has(b->get_self())) {
			return PxQueryHitType::eNONE;
		}
		return PxQueryHitType::eBLOCK;
	}

	virtual PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &, const PxShape *, const PxRigidActor *) override {
		return PxQueryHitType::eBLOCK;
	}
};

const GodotPhysXShapeGeometry *query_geometry(RID p_shape_rid) {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server) {
		return nullptr;
	}
	GodotPhysXShape3D *shape = server->get_shape(p_shape_rid);
	if (!shape || !shape->is_valid()) {
		return nullptr;
	}
	return &shape->get_geometry();
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

	QueryFilter filter;
	filter.exclude = &p_parameters.exclude;
	filter.collision_mask = p_parameters.collision_mask;

	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER);
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
	GodotPhysXBody3D *body = body_of(b.actor);
	r_result.position = to_godot(b.position);
	r_result.normal = to_godot(b.normal);
	r_result.rid = body ? body->get_self() : RID();
	r_result.collider_id = body ? body->get_instance_id() : ObjectID();
	r_result.collider = r_result.collider_id.is_valid() ? ObjectDB::get_instance(r_result.collider_id) : nullptr;
	r_result.shape = shape_index_of(b.shape);
	r_result.face_index = (b.faceIndex == 0xFFFFFFFF) ? -1 : (int)b.faceIndex;
	return true;
}

int GodotPhysXDirectSpaceState3D::intersect_point(const PointParameters &p_parameters, ShapeResult *r_results, int p_result_max) {
	PxScene *scene = space ? space->get_px_scene() : nullptr;
	ERR_FAIL_NULL_V(scene, 0);
	if (p_result_max <= 0) {
		return 0;
	}

	QueryFilter filter;
	filter.exclude = &p_parameters.exclude;
	filter.collision_mask = p_parameters.collision_mask;
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK);

	LocalVector<PxOverlapHit> touches;
	touches.resize(p_result_max);
	PxOverlapBuffer buf(touches.ptr(), (PxU32)p_result_max);

	// A near-zero sphere approximates a point overlap.
	const PxSphereGeometry probe(0.001f);
	scene->overlap(probe, PxTransform(to_px(p_parameters.position)), buf, fd, &filter);

	int count = 0;
	for (PxU32 i = 0; i < buf.getNbTouches() && count < p_result_max; i++) {
		GodotPhysXBody3D *body = body_of(buf.getTouch(i).actor);
		if (!body) {
			continue;
		}
		r_results[count].rid = body->get_self();
		r_results[count].collider_id = body->get_instance_id();
		r_results[count].collider = r_results[count].collider_id.is_valid() ? ObjectDB::get_instance(r_results[count].collider_id) : nullptr;
		r_results[count].shape = shape_index_of(buf.getTouch(i).shape);
		count++;
	}
	return count;
}

int GodotPhysXDirectSpaceState3D::intersect_shape(const ShapeParameters &p_parameters, ShapeResult *r_results, int p_result_max) {
	PxScene *scene = space ? space->get_px_scene() : nullptr;
	ERR_FAIL_NULL_V(scene, 0);
	const GodotPhysXShapeGeometry *g = query_geometry(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(g, 0);
	if (p_result_max <= 0) {
		return 0;
	}

	QueryFilter filter;
	filter.exclude = &p_parameters.exclude;
	filter.collision_mask = p_parameters.collision_mask;
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK);

	LocalVector<PxOverlapHit> touches;
	touches.resize(p_result_max);
	PxOverlapBuffer buf(touches.ptr(), (PxU32)p_result_max);

	const PxTransform pose = to_px(p_parameters.transform) * g->local_pose;
	scene->overlap(g->geometry(), pose, buf, fd, &filter);

	int count = 0;
	for (PxU32 i = 0; i < buf.getNbTouches() && count < p_result_max; i++) {
		GodotPhysXBody3D *body = body_of(buf.getTouch(i).actor);
		if (!body) {
			continue;
		}
		r_results[count].rid = body->get_self();
		r_results[count].collider_id = body->get_instance_id();
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
	const GodotPhysXShapeGeometry *g = query_geometry(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(g, false);

	const real_t motion_len = p_parameters.motion.length();
	if (motion_len <= CMP_EPSILON) {
		return true;
	}

	QueryFilter filter;
	filter.exclude = &p_parameters.exclude;
	filter.collision_mask = p_parameters.collision_mask;
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER);

	const PxTransform pose = to_px(p_parameters.transform) * g->local_pose;
	PxSweepBuffer hit;
	const bool has_hit = scene->sweep(g->geometry(), pose, to_px(p_parameters.motion / motion_len),
			(PxReal)motion_len, hit, PxHitFlag::ePOSITION | PxHitFlag::eNORMAL, fd, &filter);

	if (!has_hit || !hit.hasBlock) {
		return true;
	}

	const real_t frac = CLAMP((real_t)hit.block.distance / motion_len, (real_t)0.0, (real_t)1.0);
	p_closest_safe = frac;
	p_closest_unsafe = frac;
	if (r_info) {
		GodotPhysXBody3D *body = body_of(hit.block.actor);
		r_info->point = to_godot(hit.block.position);
		r_info->normal = to_godot(hit.block.normal);
		r_info->rid = body ? body->get_self() : RID();
		r_info->collider_id = body ? body->get_instance_id() : ObjectID();
		r_info->shape = shape_index_of(hit.block.shape);
		r_info->linear_velocity = body ? body->get_linear_velocity() : Vector3();
	}
	return true;
}

bool GodotPhysXDirectSpaceState3D::collide_shape(const ShapeParameters &p_parameters, Vector3 *r_results, int p_result_max, int &r_result_count) {
	r_result_count = 0;
	PxScene *scene = space ? space->get_px_scene() : nullptr;
	ERR_FAIL_NULL_V(scene, false);
	const GodotPhysXShapeGeometry *g = query_geometry(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(g, false);
	if (p_result_max <= 0) {
		return false;
	}

	QueryFilter filter;
	filter.exclude = &p_parameters.exclude;
	filter.collision_mask = p_parameters.collision_mask;
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK);

	const int max_hits = p_result_max / 2;
	LocalVector<PxOverlapHit> touches;
	touches.resize(MAX(max_hits, 1));
	PxOverlapBuffer buf(touches.ptr(), (PxU32)MAX(max_hits, 1));

	const PxTransform pose = to_px(p_parameters.transform) * g->local_pose;
	scene->overlap(g->geometry(), pose, buf, fd, &filter);

	for (PxU32 i = 0; i < buf.getNbTouches() && r_result_count + 2 <= p_result_max; i++) {
		const PxOverlapHit &h = buf.getTouch(i);
		PxVec3 dir;
		PxF32 depth;
		if (PxGeometryQuery::computePenetration(dir, depth, g->geometry(), pose,
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
	const GodotPhysXShapeGeometry *g = query_geometry(p_parameters.shape_rid);
	ERR_FAIL_NULL_V(g, false);

	QueryFilter filter;
	filter.exclude = &p_parameters.exclude;
	filter.collision_mask = p_parameters.collision_mask;
	PxQueryFilterData fd(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER | PxQueryFlag::eNO_BLOCK);

	PxOverlapHit touch;
	PxOverlapBuffer buf(&touch, 1);
	const PxTransform pose = to_px(p_parameters.transform) * g->local_pose;
	scene->overlap(g->geometry(), pose, buf, fd, &filter);
	if (buf.getNbTouches() == 0) {
		return false;
	}

	const PxOverlapHit &h = buf.getTouch(0);
	PxVec3 dir;
	PxF32 depth;
	const PxTransform collider_pose = h.actor->getGlobalPose() * h.shape->getLocalPose();
	if (!PxGeometryQuery::computePenetration(dir, depth, g->geometry(), pose, h.shape->getGeometry(), collider_pose)) {
		return false;
	}

	GodotPhysXBody3D *body = body_of(h.actor);
	r_info->point = to_godot(pose.p) - to_godot(dir) * depth;
	r_info->normal = to_godot(dir);
	r_info->rid = body ? body->get_self() : RID();
	r_info->collider_id = body ? body->get_instance_id() : ObjectID();
	r_info->shape = shape_index_of(h.shape);
	r_info->linear_velocity = body ? body->get_linear_velocity() : Vector3();
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

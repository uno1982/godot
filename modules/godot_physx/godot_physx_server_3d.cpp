/**************************************************************************/
/*  godot_physx_server_3d.cpp                                             */
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

#include "godot_physx_server_3d.h"

#include "godot_physx_conversions.h"
#include "godot_physx_project_settings.h"
#include "joints/godot_physx_joint_3d.h"
#include "objects/godot_physx_area_3d.h"
#include "objects/godot_physx_body_3d.h"
#include "shapes/godot_physx_shape_3d.h"
#include "spaces/godot_physx_direct_state_3d.h"
#include "spaces/godot_physx_space_3d.h"

#include "core/error/error_macros.h"
#include "core/os/os.h"
#include "core/string/print_string.h"
#include "core/string/ustring.h"
#include "core/variant/variant.h"

#include <PxPhysicsAPI.h>

using namespace physx;

namespace {

PxDefaultAllocator px_allocator;

class GodotPxErrorCallback : public PxErrorCallback {
public:
	virtual void reportError(PxErrorCode::Enum p_code, const char *p_message, const char *p_file, int p_line) override {
		const String msg = vformat("PhysX: %s (%s:%d)", p_message, p_file, p_line);
		if (p_code & (PxErrorCode::eABORT | PxErrorCode::eINTERNAL_ERROR | PxErrorCode::eINVALID_OPERATION | PxErrorCode::eINVALID_PARAMETER | PxErrorCode::eOUT_OF_MEMORY)) {
			ERR_PRINT(msg);
		} else {
			WARN_PRINT(msg);
		}
	}
};

GodotPxErrorCallback px_error_callback;

} //namespace

GodotPhysXServer3D *GodotPhysXServer3D::singleton = nullptr;

GodotPhysXServer3D::GodotPhysXServer3D() {
	singleton = this;
}

GodotPhysXServer3D::~GodotPhysXServer3D() {
	if (singleton == this) {
		singleton = nullptr;
	}
}

/* SHAPE API */

RID GodotPhysXServer3D::_shape_create(PhysicsServer3D::ShapeType p_type) {
	GodotPhysXShape3D *shape = memnew(GodotPhysXShape3D);
	shape->set_type(p_type);
	RID rid = shape_owner.make_rid(shape);
	shape->set_self(rid);
	return rid;
}

RID GodotPhysXServer3D::world_boundary_shape_create() {
	return _shape_create(SHAPE_WORLD_BOUNDARY);
}

RID GodotPhysXServer3D::sphere_shape_create() {
	return _shape_create(SHAPE_SPHERE);
}

RID GodotPhysXServer3D::box_shape_create() {
	return _shape_create(SHAPE_BOX);
}

RID GodotPhysXServer3D::capsule_shape_create() {
	return _shape_create(SHAPE_CAPSULE);
}

RID GodotPhysXServer3D::convex_polygon_shape_create() {
	return _shape_create(SHAPE_CONVEX_POLYGON);
}

RID GodotPhysXServer3D::concave_polygon_shape_create() {
	return _shape_create(SHAPE_CONCAVE_POLYGON);
}

RID GodotPhysXServer3D::cylinder_shape_create() {
	return _shape_create(SHAPE_CYLINDER);
}

RID GodotPhysXServer3D::separation_ray_shape_create() {
	return _shape_create(SHAPE_SEPARATION_RAY);
}

RID GodotPhysXServer3D::heightmap_shape_create() {
	return _shape_create(SHAPE_HEIGHTMAP);
}

RID GodotPhysXServer3D::custom_shape_create() {
	return _shape_create(SHAPE_CUSTOM);
}

void GodotPhysXServer3D::shape_set_data(RID p_shape, const Variant &p_data) {
	GodotPhysXShape3D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);
	shape->set_data(p_data);

	// Rebuild any body currently using this shape.
	LocalVector<RID> body_rids = body_owner.get_owned_list();
	for (const RID &body_rid : body_rids) {
		GodotPhysXBody3D *body = body_owner.get_or_null(body_rid);
		if (body) {
			body->shape_changed(shape);
		}
	}
}

void GodotPhysXServer3D::shape_set_margin(RID p_shape, real_t p_margin) {
	GodotPhysXShape3D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);
	shape->set_margin(p_margin);
}

real_t GodotPhysXServer3D::shape_get_margin(RID p_shape) const {
	GodotPhysXShape3D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_V(shape, 0.0);
	return shape->get_margin();
}

PhysicsServer3D::ShapeType GodotPhysXServer3D::shape_get_type(RID p_shape) const {
	GodotPhysXShape3D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_V(shape, SHAPE_CUSTOM);
	return shape->get_type();
}

Variant GodotPhysXServer3D::shape_get_data(RID p_shape) const {
	GodotPhysXShape3D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL_V(shape, Variant());
	return shape->get_data();
}

/* SPACE API */

RID GodotPhysXServer3D::space_create() {
	ERR_FAIL_NULL_V(px_physics, RID());
	GodotPhysXSpace3D *space = memnew(GodotPhysXSpace3D(px_physics, px_dispatcher, px_cuda));
	RID rid = space_owner.make_rid(space);
	space->set_self(rid);
	return rid;
}

void GodotPhysXServer3D::space_set_active(RID p_space, bool p_active) {
	GodotPhysXSpace3D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);
	if (p_active) {
		active_spaces.insert(space);
	} else {
		active_spaces.erase(space);
	}
}

bool GodotPhysXServer3D::space_is_active(RID p_space) const {
	GodotPhysXSpace3D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V(space, false);
	return active_spaces.has(space);
}

void GodotPhysXServer3D::space_set_param(RID p_space, SpaceParameter p_param, real_t p_value) {
	GodotPhysXSpace3D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL(space);
	space->set_param(p_param, p_value);
}

real_t GodotPhysXServer3D::space_get_param(RID p_space, SpaceParameter p_param) const {
	GodotPhysXSpace3D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V(space, 0.0);
	return space->get_param(p_param);
}

PhysicsDirectSpaceState3D *GodotPhysXServer3D::space_get_direct_state(RID p_space) {
	GodotPhysXSpace3D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_NULL_V(space, nullptr);
	return space->get_direct_state();
}

/* AREA API */

RID GodotPhysXServer3D::area_create() {
	GodotPhysXArea3D *area = memnew(GodotPhysXArea3D);
	RID rid = area_owner.make_rid(area);
	area->set_self(rid);
	return rid;
}

void GodotPhysXServer3D::area_set_space(RID p_area, RID p_space) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_space(space_owner.get_or_null(p_space));
}

RID GodotPhysXServer3D::area_get_space(RID p_area) const {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, RID());
	GodotPhysXSpace3D *space = area->get_space();
	return space ? space->get_self() : RID();
}

void GodotPhysXServer3D::area_add_shape(RID p_area, RID p_shape, const Transform3D &p_transform, bool p_disabled) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	GodotPhysXShape3D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);
	area->add_shape(shape, p_transform, p_disabled);
}

void GodotPhysXServer3D::area_set_shape(RID p_area, int p_shape_idx, RID p_shape) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_shape(p_shape_idx, shape_owner.get_or_null(p_shape));
}

void GodotPhysXServer3D::area_set_shape_transform(RID p_area, int p_shape_idx, const Transform3D &p_transform) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_shape_transform(p_shape_idx, p_transform);
}

int GodotPhysXServer3D::area_get_shape_count(RID p_area) const {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, 0);
	return area->get_shape_count();
}

RID GodotPhysXServer3D::area_get_shape(RID p_area, int p_shape_idx) const {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, RID());
	const GodotPhysXArea3D::ShapeRef *sr = area->get_shape_ref(p_shape_idx);
	return (sr && sr->shape) ? sr->shape->get_self() : RID();
}

Transform3D GodotPhysXServer3D::area_get_shape_transform(RID p_area, int p_shape_idx) const {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, Transform3D());
	const GodotPhysXArea3D::ShapeRef *sr = area->get_shape_ref(p_shape_idx);
	return sr ? sr->xform : Transform3D();
}

void GodotPhysXServer3D::area_remove_shape(RID p_area, int p_shape_idx) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->remove_shape(p_shape_idx);
}

void GodotPhysXServer3D::area_clear_shapes(RID p_area) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->clear_shapes();
}

void GodotPhysXServer3D::area_set_shape_disabled(RID p_area, int p_shape_idx, bool p_disabled) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_shape_disabled(p_shape_idx, p_disabled);
}

void GodotPhysXServer3D::area_attach_object_instance_id(RID p_area, ObjectID p_id) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_instance_id(p_id);
}

ObjectID GodotPhysXServer3D::area_get_object_instance_id(RID p_area) const {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, ObjectID());
	return area->get_instance_id();
}

void GodotPhysXServer3D::area_set_transform(RID p_area, const Transform3D &p_transform) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_transform(p_transform);
}

Transform3D GodotPhysXServer3D::area_get_transform(RID p_area) const {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, Transform3D());
	return area->get_transform();
}

void GodotPhysXServer3D::area_set_collision_layer(RID p_area, uint32_t p_layer) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_collision_layer(p_layer);
}

uint32_t GodotPhysXServer3D::area_get_collision_layer(RID p_area) const {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, 0);
	return area->get_collision_layer();
}

void GodotPhysXServer3D::area_set_collision_mask(RID p_area, uint32_t p_mask) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_collision_mask(p_mask);
}

uint32_t GodotPhysXServer3D::area_get_collision_mask(RID p_area) const {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, 0);
	return area->get_collision_mask();
}

void GodotPhysXServer3D::area_set_monitorable(RID p_area, bool p_monitorable) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_monitorable(p_monitorable);
}

void GodotPhysXServer3D::area_set_monitor_callback(RID p_area, const Callable &p_callback) {
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_monitor_callback(p_callback);
}

void GodotPhysXServer3D::area_set_param(RID p_area, AreaParameter p_param, const Variant &p_value) {
	// The scene tree writes default gravity to the space RID, which doubles as
	// the space's default area.
	GodotPhysXSpace3D *space = space_owner.get_or_null(p_area);
	if (space) {
		if (p_param == AREA_PARAM_GRAVITY) {
			space->set_gravity_magnitude(p_value);
		} else if (p_param == AREA_PARAM_GRAVITY_VECTOR) {
			space->set_gravity_direction(p_value);
		}
		return;
	}
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL(area);
	area->set_param(p_param, p_value);
}

Variant GodotPhysXServer3D::area_get_param(RID p_area, AreaParameter p_param) const {
	GodotPhysXSpace3D *space = space_owner.get_or_null(p_area);
	if (space) {
		if (p_param == AREA_PARAM_GRAVITY) {
			return space->get_gravity().length();
		} else if (p_param == AREA_PARAM_GRAVITY_VECTOR) {
			return space->get_gravity().normalized();
		}
		return Variant();
	}
	GodotPhysXArea3D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_NULL_V(area, Variant());
	return area->get_param(p_param);
}

/* BODY API */

RID GodotPhysXServer3D::body_create() {
	GodotPhysXBody3D *body = memnew(GodotPhysXBody3D);
	RID rid = body_owner.make_rid(body);
	body->set_self(rid);
	return rid;
}

void GodotPhysXServer3D::body_set_space(RID p_body, RID p_space) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_space(p_space.is_valid() ? space_owner.get_or_null(p_space) : nullptr);
}

RID GodotPhysXServer3D::body_get_space(RID p_body) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, RID());
	GodotPhysXSpace3D *space = body->get_space();
	return space ? space->get_self() : RID();
}

void GodotPhysXServer3D::body_set_mode(RID p_body, BodyMode p_mode) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_mode(p_mode);
}

PhysicsServer3D::BodyMode GodotPhysXServer3D::body_get_mode(RID p_body) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, BODY_MODE_STATIC);
	return body->get_mode();
}

void GodotPhysXServer3D::body_add_shape(RID p_body, RID p_shape, const Transform3D &p_transform, bool p_disabled) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	GodotPhysXShape3D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_NULL(shape);
	body->add_shape(shape, p_transform, p_disabled);
}

void GodotPhysXServer3D::body_set_shape(RID p_body, int p_shape_idx, RID p_shape) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_shape(p_shape_idx, shape_owner.get_or_null(p_shape));
}

void GodotPhysXServer3D::body_set_shape_transform(RID p_body, int p_shape_idx, const Transform3D &p_transform) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_shape_transform(p_shape_idx, p_transform);
}

int GodotPhysXServer3D::body_get_shape_count(RID p_body) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);
	return body->get_shape_count();
}

RID GodotPhysXServer3D::body_get_shape(RID p_body, int p_shape_idx) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, RID());
	const GodotPhysXBody3D::ShapeRef *sr = body->get_shape_ref(p_shape_idx);
	return (sr && sr->shape) ? sr->shape->get_self() : RID();
}

Transform3D GodotPhysXServer3D::body_get_shape_transform(RID p_body, int p_shape_idx) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, Transform3D());
	const GodotPhysXBody3D::ShapeRef *sr = body->get_shape_ref(p_shape_idx);
	return sr ? sr->xform : Transform3D();
}

void GodotPhysXServer3D::body_remove_shape(RID p_body, int p_shape_idx) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->remove_shape(p_shape_idx);
}

void GodotPhysXServer3D::body_clear_shapes(RID p_body) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->clear_shapes();
}

void GodotPhysXServer3D::body_set_shape_disabled(RID p_body, int p_shape_idx, bool p_disabled) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_shape_disabled(p_shape_idx, p_disabled);
}

void GodotPhysXServer3D::body_attach_object_instance_id(RID p_body, ObjectID p_id) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_instance_id(p_id);
}

ObjectID GodotPhysXServer3D::body_get_object_instance_id(RID p_body) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, ObjectID());
	return body->get_instance_id();
}

void GodotPhysXServer3D::body_set_collision_layer(RID p_body, uint32_t p_layer) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_collision_layer(p_layer);
}

uint32_t GodotPhysXServer3D::body_get_collision_layer(RID p_body) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);
	return body->get_collision_layer();
}

void GodotPhysXServer3D::body_set_collision_mask(RID p_body, uint32_t p_mask) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_collision_mask(p_mask);
}

uint32_t GodotPhysXServer3D::body_get_collision_mask(RID p_body) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);
	return body->get_collision_mask();
}

void GodotPhysXServer3D::body_set_enable_continuous_collision_detection(RID p_body, bool p_enable) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_ccd(p_enable);
}

bool GodotPhysXServer3D::body_is_continuous_collision_detection_enabled(RID p_body) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, false);
	return body->is_ccd_enabled();
}

void GodotPhysXServer3D::body_set_param(RID p_body, BodyParameter p_param, const Variant &p_value) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_param(p_param, p_value);
}

Variant GodotPhysXServer3D::body_get_param(RID p_body, BodyParameter p_param) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, Variant());
	return body->get_param(p_param);
}

void GodotPhysXServer3D::body_set_state(RID p_body, BodyState p_state, const Variant &p_variant) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_state(p_state, p_variant);
}

Variant GodotPhysXServer3D::body_get_state(RID p_body, BodyState p_state) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, Variant());
	return body->get_state(p_state);
}

void GodotPhysXServer3D::body_apply_central_impulse(RID p_body, const Vector3 &p_impulse) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->apply_central_impulse(p_impulse);
}

void GodotPhysXServer3D::body_apply_impulse(RID p_body, const Vector3 &p_impulse, const Vector3 &p_position) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->apply_impulse(p_impulse, p_position);
}

void GodotPhysXServer3D::body_apply_torque_impulse(RID p_body, const Vector3 &p_impulse) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->apply_torque_impulse(p_impulse);
}

void GodotPhysXServer3D::body_apply_central_force(RID p_body, const Vector3 &p_force) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->apply_central_force(p_force);
}

void GodotPhysXServer3D::body_set_axis_lock(RID p_body, BodyAxis p_axis, bool p_lock) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_axis_lock(p_axis, p_lock);
}

bool GodotPhysXServer3D::body_is_axis_locked(RID p_body, BodyAxis p_axis) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, false);
	return body->is_axis_locked(p_axis);
}

void GodotPhysXServer3D::body_set_max_contacts_reported(RID p_body, int p_contacts) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_max_contacts_reported(p_contacts);
}

int GodotPhysXServer3D::body_get_max_contacts_reported(RID p_body) const {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, 0);
	return body->get_max_contacts_reported();
}

void GodotPhysXServer3D::body_set_state_sync_callback(RID p_body, const Callable &p_callable) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL(body);
	body->set_state_sync_callback(p_callable);
}

PhysicsDirectBodyState3D *GodotPhysXServer3D::body_get_direct_state(RID p_body) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, nullptr);
	return body->get_direct_state();
}

bool GodotPhysXServer3D::body_test_motion(RID p_body, const MotionParameters &p_parameters, MotionResult *r_result) {
	GodotPhysXBody3D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_NULL_V(body, false);
	GodotPhysXSpace3D *space = body->get_space();
	ERR_FAIL_NULL_V(space, false);
	return space->test_body_motion(body, p_parameters, r_result);
}

/* JOINT API */

#define GET_JOINT(m_var, m_rid, m_ret) \
	GodotPhysXJoint3D *m_var = joint_owner.get_or_null(m_rid); \
	ERR_FAIL_NULL_V(m_var, m_ret)
#define GET_JOINT_VOID(m_var, m_rid) \
	GodotPhysXJoint3D *m_var = joint_owner.get_or_null(m_rid); \
	ERR_FAIL_NULL(m_var)

RID GodotPhysXServer3D::joint_create() {
	GodotPhysXJoint3D *joint = memnew(GodotPhysXJoint3D);
	RID rid = joint_owner.make_rid(joint);
	joint->set_self(rid);
	return rid;
}

void GodotPhysXServer3D::joint_clear(RID p_joint) {
	GET_JOINT_VOID(joint, p_joint);
	joint->clear();
}

PhysicsServer3D::JointType GodotPhysXServer3D::joint_get_type(RID p_joint) const {
	GET_JOINT(joint, p_joint, JOINT_TYPE_MAX);
	return joint->get_type();
}

void GodotPhysXServer3D::joint_set_solver_priority(RID p_joint, int p_priority) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_solver_priority(p_priority);
}

int GodotPhysXServer3D::joint_get_solver_priority(RID p_joint) const {
	GET_JOINT(joint, p_joint, 0);
	return joint->get_solver_priority();
}

void GodotPhysXServer3D::joint_disable_collisions_between_bodies(RID p_joint, bool p_disable) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_collisions_disabled(p_disable);
}

bool GodotPhysXServer3D::joint_is_disabled_collisions_between_bodies(RID p_joint) const {
	GET_JOINT(joint, p_joint, true);
	return joint->are_collisions_disabled();
}

void GodotPhysXServer3D::joint_make_pin(RID p_joint, RID p_body_A, const Vector3 &p_local_A, RID p_body_B, const Vector3 &p_local_B) {
	GET_JOINT_VOID(joint, p_joint);
	joint->make_pin(body_owner.get_or_null(p_body_A), p_local_A, body_owner.get_or_null(p_body_B), p_local_B);
}

void GodotPhysXServer3D::pin_joint_set_param(RID p_joint, PinJointParam p_param, real_t p_value) {}
real_t GodotPhysXServer3D::pin_joint_get_param(RID p_joint, PinJointParam p_param) const {
	return 0.0;
}

void GodotPhysXServer3D::pin_joint_set_local_a(RID p_joint, const Vector3 &p_A) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_pin_local_a(p_A);
}
Vector3 GodotPhysXServer3D::pin_joint_get_local_a(RID p_joint) const {
	GET_JOINT(joint, p_joint, Vector3());
	return joint->get_pin_local_a();
}
void GodotPhysXServer3D::pin_joint_set_local_b(RID p_joint, const Vector3 &p_B) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_pin_local_b(p_B);
}
Vector3 GodotPhysXServer3D::pin_joint_get_local_b(RID p_joint) const {
	GET_JOINT(joint, p_joint, Vector3());
	return joint->get_pin_local_b();
}

void GodotPhysXServer3D::joint_make_hinge(RID p_joint, RID p_body_A, const Transform3D &p_hinge_A, RID p_body_B, const Transform3D &p_hinge_B) {
	GET_JOINT_VOID(joint, p_joint);
	joint->make_hinge(body_owner.get_or_null(p_body_A), p_hinge_A, body_owner.get_or_null(p_body_B), p_hinge_B);
}

void GodotPhysXServer3D::joint_make_hinge_simple(RID p_joint, RID p_body_A, const Vector3 &p_pivot_A, const Vector3 &p_axis_A, RID p_body_B, const Vector3 &p_pivot_B, const Vector3 &p_axis_B) {
	// Build hinge frames whose Z axis is the hinge axis.
	auto frame_from = [](const Vector3 &p_pivot, const Vector3 &p_axis) {
		Vector3 z = p_axis.normalized();
		Vector3 x = (Math::abs(z.x) < 0.9) ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
		x = (x - z * x.dot(z)).normalized();
		Vector3 y = z.cross(x);
		Basis b;
		b.set_column(0, x);
		b.set_column(1, y);
		b.set_column(2, z);
		return Transform3D(b, p_pivot);
	};
	joint_make_hinge(p_joint, p_body_A, frame_from(p_pivot_A, p_axis_A), p_body_B, frame_from(p_pivot_B, p_axis_B));
}

void GodotPhysXServer3D::hinge_joint_set_param(RID p_joint, HingeJointParam p_param, real_t p_value) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_hinge_param(p_param, p_value);
}
real_t GodotPhysXServer3D::hinge_joint_get_param(RID p_joint, HingeJointParam p_param) const {
	GET_JOINT(joint, p_joint, 0.0);
	return joint->get_hinge_param(p_param);
}
void GodotPhysXServer3D::hinge_joint_set_flag(RID p_joint, HingeJointFlag p_flag, bool p_enabled) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_hinge_flag(p_flag, p_enabled);
}
bool GodotPhysXServer3D::hinge_joint_get_flag(RID p_joint, HingeJointFlag p_flag) const {
	GET_JOINT(joint, p_joint, false);
	return joint->get_hinge_flag(p_flag);
}

void GodotPhysXServer3D::joint_make_slider(RID p_joint, RID p_body_A, const Transform3D &p_local_frame_A, RID p_body_B, const Transform3D &p_local_frame_B) {
	GET_JOINT_VOID(joint, p_joint);
	joint->make_slider(body_owner.get_or_null(p_body_A), p_local_frame_A, body_owner.get_or_null(p_body_B), p_local_frame_B);
}
void GodotPhysXServer3D::slider_joint_set_param(RID p_joint, SliderJointParam p_param, real_t p_value) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_slider_param(p_param, p_value);
}
real_t GodotPhysXServer3D::slider_joint_get_param(RID p_joint, SliderJointParam p_param) const {
	GET_JOINT(joint, p_joint, 0.0);
	return joint->get_slider_param(p_param);
}

void GodotPhysXServer3D::joint_make_cone_twist(RID p_joint, RID p_body_A, const Transform3D &p_local_frame_A, RID p_body_B, const Transform3D &p_local_frame_B) {
	GET_JOINT_VOID(joint, p_joint);
	joint->make_cone_twist(body_owner.get_or_null(p_body_A), p_local_frame_A, body_owner.get_or_null(p_body_B), p_local_frame_B);
}
void GodotPhysXServer3D::cone_twist_joint_set_param(RID p_joint, ConeTwistJointParam p_param, real_t p_value) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_cone_twist_param(p_param, p_value);
}
real_t GodotPhysXServer3D::cone_twist_joint_get_param(RID p_joint, ConeTwistJointParam p_param) const {
	GET_JOINT(joint, p_joint, 0.0);
	return joint->get_cone_twist_param(p_param);
}

void GodotPhysXServer3D::joint_make_generic_6dof(RID p_joint, RID p_body_A, const Transform3D &p_local_frame_A, RID p_body_B, const Transform3D &p_local_frame_B) {
	GET_JOINT_VOID(joint, p_joint);
	joint->make_6dof(body_owner.get_or_null(p_body_A), p_local_frame_A, body_owner.get_or_null(p_body_B), p_local_frame_B);
}
void GodotPhysXServer3D::generic_6dof_joint_set_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param, real_t p_value) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_6dof_param(p_axis, p_param, p_value);
}
real_t GodotPhysXServer3D::generic_6dof_joint_get_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param) const {
	GET_JOINT(joint, p_joint, 0.0);
	return joint->get_6dof_param(p_axis, p_param);
}
void GodotPhysXServer3D::generic_6dof_joint_set_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag, bool p_enable) {
	GET_JOINT_VOID(joint, p_joint);
	joint->set_6dof_flag(p_axis, p_flag, p_enable);
}
bool GodotPhysXServer3D::generic_6dof_joint_get_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag) const {
	GET_JOINT(joint, p_joint, false);
	return joint->get_6dof_flag(p_axis, p_flag);
}

/* MISC */

void GodotPhysXServer3D::free_rid(RID p_rid) {
	if (GodotPhysXShape3D *shape = shape_owner.get_or_null(p_rid)) {
		shape_owner.free(p_rid);
		memdelete(shape);
	} else if (GodotPhysXBody3D *body = body_owner.get_or_null(p_rid)) {
		body->set_space(nullptr);
		body_owner.free(p_rid);
		memdelete(body);
	} else if (GodotPhysXJoint3D *joint = joint_owner.get_or_null(p_rid)) {
		joint->clear();
		joint_owner.free(p_rid);
		memdelete(joint);
	} else if (GodotPhysXArea3D *area = area_owner.get_or_null(p_rid)) {
		area->set_space(nullptr);
		area_owner.free(p_rid);
		memdelete(area);
	} else if (GodotPhysXSpace3D *space = space_owner.get_or_null(p_rid)) {
		active_spaces.erase(space);
		space_owner.free(p_rid);
		memdelete(space);
	} else {
		ERR_FAIL_MSG("PhysX: attempted to free invalid RID.");
	}
}

void GodotPhysXServer3D::set_active(bool p_active) {
	active = p_active;
}

void GodotPhysXServer3D::init() {
	PhysicsServer3DDummy::init();

	GodotPhysXProjectSettings::read_settings();

	px_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, px_allocator, px_error_callback);
	ERR_FAIL_NULL_MSG(px_foundation, "PhysX: PxCreateFoundation failed.");

	px_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *px_foundation, PxTolerancesScale(), false, nullptr);
	ERR_FAIL_NULL_MSG(px_physics, "PhysX: PxCreatePhysics failed.");

#ifdef GODOT_PHYSX_GPU
	if (!GodotPhysXProjectSettings::enhanced_determinism) {
		PxCudaContextManagerDesc cuda_desc;
		px_cuda = PxCreateCudaContextManager(*px_foundation, cuda_desc, PxGetProfilerCallback());
		if (px_cuda && !px_cuda->contextIsValid()) {
			px_cuda->release();
			px_cuda = nullptr;
		}
		if (px_cuda) {
			print_line(vformat("PhysX: CUDA context ready on device '%s' -> GPU dynamics available.", px_cuda->getDeviceName()));
		} else {
			WARN_PRINT("PhysX: no usable CUDA device; falling back to CPU simulation.");
		}
	} else {
		print_verbose("PhysX: enhanced_determinism is set -> GPU dynamics disabled (the GPU solver is not deterministic).");
	}
#endif

	// Size the PhysX CPU task pool. The CPU solver scales with worker count, so
	// give it most of the machine (leaving a core for the main thread + render).
	// With GPU dynamics the CPU mostly waits on the GPU each step, so extra
	// workers only add coordination overhead below very large scene sizes -- keep
	// that pool small. Both cases are backed by benchmarks;
	// physics/physx_3d/simulation/cpu_worker_threads overrides (0 = auto).
	const int cpu_count = OS::get_singleton()->get_processor_count();
	uint32_t worker_threads;
	if (GodotPhysXProjectSettings::cpu_worker_threads > 0) {
		worker_threads = (uint32_t)GodotPhysXProjectSettings::cpu_worker_threads;
	} else {
		worker_threads = px_cuda
				? (uint32_t)CLAMP(cpu_count / 4, 2, 4)
				: (uint32_t)CLAMP(cpu_count - 1, 1, 16);
	}
	px_dispatcher = PxDefaultCpuDispatcherCreate(worker_threads);
	ERR_FAIL_NULL_MSG(px_dispatcher, "PhysX: PxDefaultCpuDispatcherCreate failed.");
	print_verbose(vformat("PhysX: CPU dispatcher using %d worker threads (of %d)%s.",
			worker_threads, cpu_count, px_cuda ? " [GPU path]" : ""));

	print_verbose(vformat("PhysX %d.%d.%d initialized%s.",
			PX_PHYSICS_VERSION_MAJOR, PX_PHYSICS_VERSION_MINOR, PX_PHYSICS_VERSION_BUGFIX,
			px_cuda ? " [GPU]" : ""));
}

void GodotPhysXServer3D::step(real_t p_step) {
	if (!active) {
		return;
	}
	active_objects = 0;
	for (GodotPhysXSpace3D *space : active_spaces) {
		space->step(p_step);
		if (PxScene *scene = space->get_px_scene()) {
			PxU32 nb_active = 0;
			scene->getActiveActors(nb_active);
			active_objects += (int)nb_active;
		}
	}
}

void GodotPhysXServer3D::flush_queries() {
	if (!active) {
		return;
	}
	flushing_queries = true;
	for (GodotPhysXSpace3D *space : active_spaces) {
		space->call_queries();
	}
	flushing_queries = false;
}

void GodotPhysXServer3D::finish() {
	if (px_cuda) {
		px_cuda->release();
		px_cuda = nullptr;
	}
	if (px_dispatcher) {
		px_dispatcher->release();
		px_dispatcher = nullptr;
	}
	if (px_physics) {
		px_physics->release();
		px_physics = nullptr;
	}
	if (px_foundation) {
		px_foundation->release();
		px_foundation = nullptr;
	}
	PhysicsServer3DDummy::finish();
}

int GodotPhysXServer3D::get_process_info(ProcessInfo p_info) {
	if (p_info == INFO_ACTIVE_OBJECTS) {
		return active_objects;
	}
	return 0;
}

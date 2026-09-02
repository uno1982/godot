/**************************************************************************/
/*  godot_physx_area_3d.cpp                                               */
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

#include "godot_physx_area_3d.h"

#include "../godot_physx_conversions.h"
#include "../shapes/godot_physx_shape_3d.h"
#include "../spaces/godot_physx_space_3d.h"
#include "godot_physx_body_3d.h"

#include "core/math/math_funcs.h"
#include "core/variant/variant.h"

#include <PxPhysicsAPI.h>

using namespace physx;

GodotPhysXArea3D::GodotPhysXArea3D() {}

GodotPhysXArea3D::~GodotPhysXArea3D() {
	_destroy_actor();
}

void GodotPhysXArea3D::_destroy_actor() {
	if (!px_actor) {
		return;
	}
	if (space && space->get_px_scene()) {
		space->get_px_scene()->removeActor(*px_actor);
	}
	px_actor->release();
	px_actor = nullptr;
}

void GodotPhysXArea3D::_build_actor() {
	_destroy_actor();
	if (!space) {
		return;
	}
	PxPhysics *physics = space->get_px_physics();
	PxScene *scene = space->get_px_scene();
	PxMaterial *material = space->get_default_material();
	ERR_FAIL_NULL(physics);
	ERR_FAIL_NULL(scene);

	px_actor = physics->createRigidStatic(to_px(area_transform));
	ERR_FAIL_NULL(px_actor);
	px_actor->userData = this;

	for (uint32_t i = 0; i < shapes.size(); i++) {
		const ShapeRef &sr = shapes[i];
		if (sr.disabled || !sr.shape || !sr.shape->is_valid()) {
			continue;
		}
		const GodotPhysXShapeGeometry &g = sr.shape->get_geometry();
		PxShape *px_shape = physics->createShape(g.geometry(), *material, true,
				PxShapeFlag::eTRIGGER_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE);
		if (!px_shape) {
			continue;
		}
		px_shape->setLocalPose(to_px(sr.xform) * g.local_pose);
		px_shape->userData = reinterpret_cast<void *>(static_cast<uintptr_t>(i));
		px_actor->attachShape(*px_shape);
		px_shape->release();
	}

	_apply_filter_data();
	scene->addActor(*px_actor);
}

void GodotPhysXArea3D::_apply_filter_data() {
	if (!px_actor) {
		return;
	}
	PxFilterData fd;
	fd.word0 = collision_layer;
	fd.word1 = collision_mask;
	fd.word2 = 0;
	fd.word3 = 0;

	const PxU32 nb = px_actor->getNbShapes();
	LocalVector<PxShape *> px_shapes;
	px_shapes.resize(nb);
	px_actor->getShapes(px_shapes.ptr(), nb);
	for (PxU32 i = 0; i < nb; i++) {
		px_shapes[i]->setSimulationFilterData(fd);
		px_shapes[i]->setQueryFilterData(fd);
	}
}

void GodotPhysXArea3D::set_space(GodotPhysXSpace3D *p_space) {
	if (space == p_space) {
		return;
	}
	_destroy_actor();
	if (space) {
		space->unregister_area(this);
	}
	space = p_space;
	if (space) {
		space->register_area(this);
		_build_actor();
	}
}

void GodotPhysXArea3D::add_shape(GodotPhysXShape3D *p_shape, const Transform3D &p_xform, bool p_disabled) {
	ShapeRef sr;
	sr.shape = p_shape;
	sr.xform = p_xform;
	sr.disabled = p_disabled;
	shapes.push_back(sr);
	if (space) {
		_build_actor();
	}
}

void GodotPhysXArea3D::set_shape(int p_idx, GodotPhysXShape3D *p_shape) {
	ERR_FAIL_INDEX(p_idx, (int)shapes.size());
	shapes[p_idx].shape = p_shape;
	if (space) {
		_build_actor();
	}
}

void GodotPhysXArea3D::set_shape_transform(int p_idx, const Transform3D &p_xform) {
	ERR_FAIL_INDEX(p_idx, (int)shapes.size());
	shapes[p_idx].xform = p_xform;
	if (space) {
		_build_actor();
	}
}

void GodotPhysXArea3D::set_shape_disabled(int p_idx, bool p_disabled) {
	ERR_FAIL_INDEX(p_idx, (int)shapes.size());
	shapes[p_idx].disabled = p_disabled;
	if (space) {
		_build_actor();
	}
}

void GodotPhysXArea3D::remove_shape(int p_idx) {
	ERR_FAIL_INDEX(p_idx, (int)shapes.size());
	shapes.remove_at(p_idx);
	if (space) {
		_build_actor();
	}
}

void GodotPhysXArea3D::clear_shapes() {
	shapes.clear();
	if (space) {
		_build_actor();
	}
}

const GodotPhysXArea3D::ShapeRef *GodotPhysXArea3D::get_shape_ref(int p_idx) const {
	ERR_FAIL_INDEX_V(p_idx, (int)shapes.size(), nullptr);
	return &shapes[p_idx];
}

void GodotPhysXArea3D::set_transform(const Transform3D &p_transform) {
	area_transform = p_transform;
	if (px_actor) {
		px_actor->setGlobalPose(to_px(area_transform));
	}
}

void GodotPhysXArea3D::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;
	_apply_filter_data();
}

void GodotPhysXArea3D::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
	_apply_filter_data();
}

void GodotPhysXArea3D::report_body_overlap(GodotPhysXBody3D *p_body, int p_body_shape, int p_area_shape, bool p_entered) {
	if (!p_body) {
		return;
	}

	// Track the set of overlapping bodies (refcounted by shape pair) so the space
	// can apply gravity/damp/wind overrides even when no monitor callback is set.
	if (p_entered) {
		overlapping_bodies[p_body]++;
	} else {
		HashMap<GodotPhysXBody3D *, uint32_t>::Iterator it = overlapping_bodies.find(p_body);
		if (it) {
			if (it->value <= 1) {
				overlapping_bodies.remove(it);
			} else {
				it->value--;
			}
		}
	}

	if (monitor_callback.is_null()) {
		return;
	}
	OverlapKey key;
	key.body_rid = p_body->get_self();
	key.shape_pair = ((uint32_t)p_body_shape << 16) | ((uint32_t)p_area_shape & 0xFFFF);
	OverlapState &st = pending[key];
	st.instance_id = p_body->get_instance_id();
	st.delta += p_entered ? 1 : -1;
}

void GodotPhysXArea3D::set_param(PhysicsServer3D::AreaParameter p_param, const Variant &p_value) {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY_OVERRIDE_MODE:
			gravity_override_mode = (PhysicsServer3D::AreaSpaceOverrideMode)(int)p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY:
			gravity = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR:
			gravity_vector = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT:
			gravity_is_point = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE:
			gravity_point_unit_distance = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE:
			linear_damp_override_mode = (PhysicsServer3D::AreaSpaceOverrideMode)(int)p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP:
			linear_damp = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE:
			angular_damp_override_mode = (PhysicsServer3D::AreaSpaceOverrideMode)(int)p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP:
			angular_damp = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_PRIORITY:
			priority = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_WIND_FORCE_MAGNITUDE:
			wind_force_magnitude = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_WIND_SOURCE:
			wind_source = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_WIND_DIRECTION:
			wind_direction = p_value;
			break;
		case PhysicsServer3D::AREA_PARAM_WIND_ATTENUATION_FACTOR:
			wind_attenuation_factor = p_value;
			break;
		default:
			break;
	}
}

Variant GodotPhysXArea3D::get_param(PhysicsServer3D::AreaParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::AREA_PARAM_GRAVITY_OVERRIDE_MODE:
			return gravity_override_mode;
		case PhysicsServer3D::AREA_PARAM_GRAVITY:
			return gravity;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_VECTOR:
			return gravity_vector;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_IS_POINT:
			return gravity_is_point;
		case PhysicsServer3D::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE:
			return gravity_point_unit_distance;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE:
			return linear_damp_override_mode;
		case PhysicsServer3D::AREA_PARAM_LINEAR_DAMP:
			return linear_damp;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE:
			return angular_damp_override_mode;
		case PhysicsServer3D::AREA_PARAM_ANGULAR_DAMP:
			return angular_damp;
		case PhysicsServer3D::AREA_PARAM_PRIORITY:
			return priority;
		case PhysicsServer3D::AREA_PARAM_WIND_FORCE_MAGNITUDE:
			return wind_force_magnitude;
		case PhysicsServer3D::AREA_PARAM_WIND_SOURCE:
			return wind_source;
		case PhysicsServer3D::AREA_PARAM_WIND_DIRECTION:
			return wind_direction;
		case PhysicsServer3D::AREA_PARAM_WIND_ATTENUATION_FACTOR:
			return wind_attenuation_factor;
		default:
			return Variant();
	}
}

Vector3 GodotPhysXArea3D::gravity_at(const Vector3 &p_position) const {
	if (!gravity_is_point) {
		return gravity_vector * gravity;
	}
	const Vector3 point = area_transform.xform(gravity_vector);
	const Vector3 to_point = point - p_position;
	const real_t dist_sq = MAX(to_point.length_squared(), (real_t)CMP_EPSILON);
	const Vector3 dir = to_point / Math::sqrt(dist_sq);
	if (gravity_point_unit_distance <= 0.0) {
		return dir * gravity;
	}
	const real_t unit_sq = gravity_point_unit_distance * gravity_point_unit_distance;
	return dir * (gravity * unit_sq / dist_sq);
}

Vector3 GodotPhysXArea3D::wind_at(const Vector3 &p_position) const {
	if (wind_force_magnitude == 0.0 || wind_direction.length_squared() < CMP_EPSILON) {
		return Vector3();
	}
	const Vector3 dir = wind_direction.normalized();
	real_t attenuation = 1.0;
	if (wind_attenuation_factor != 0.0) {
		const real_t along = MAX((p_position - wind_source).dot(dir), (real_t)1.0);
		attenuation = Math::pow(along, -wind_attenuation_factor);
	}
	return dir * (wind_force_magnitude * attenuation);
}

void GodotPhysXArea3D::call_queries() {
	if (monitor_callback.is_null() || pending.is_empty()) {
		return;
	}
	Variant args[5];
	const Variant *argp[5] = { &args[0], &args[1], &args[2], &args[3], &args[4] };

	for (const KeyValue<OverlapKey, OverlapState> &E : pending) {
		if (E.value.delta == 0) {
			continue;
		}
		args[0] = E.value.delta > 0 ? PhysicsServer3D::AREA_BODY_ADDED : PhysicsServer3D::AREA_BODY_REMOVED;
		args[1] = E.key.body_rid;
		args[2] = E.value.instance_id;
		args[3] = (int)(E.key.shape_pair >> 16); // body shape
		args[4] = (int)(E.key.shape_pair & 0xFFFF); // area shape

		Callable::CallError ce;
		Variant ret;
		monitor_callback.callp(argp, 5, ret, ce);
	}
	pending.clear();
}

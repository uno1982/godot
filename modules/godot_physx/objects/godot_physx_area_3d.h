/**************************************************************************/
/*  godot_physx_area_3d.h                                                 */
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

#include "core/math/transform_3d.h"
#include "core/object/object_id.h"
#include "core/templates/hash_map.h"
#include "core/templates/hashfuncs.h"
#include "core/templates/local_vector.h"
#include "core/templates/rid.h"
#include "core/variant/callable.h"
#include "servers/physics_3d/physics_server_3d.h"

namespace physx {
class PxRigidActor;
} //namespace physx

class GodotPhysXSpace3D;
class GodotPhysXShape3D;
class GodotPhysXBody3D;

// Trigger volume. Overlap with rigid bodies is detected via PhysX trigger shapes
// and reported to Godot through the monitor callback. Gravity, damping and wind
// overrides are applied to overlapping bodies by the space each step. Area-area
// detection is not implemented (PhysX does not report trigger-trigger pairs).
class GodotPhysXArea3D {
public:
	struct ShapeRef {
		GodotPhysXShape3D *shape = nullptr;
		Transform3D xform;
		bool disabled = false;
	};

private:
	RID self;
	ObjectID instance_id;
	GodotPhysXSpace3D *space = nullptr;
	physx::PxRigidActor *px_actor = nullptr;

	LocalVector<ShapeRef> shapes;
	Transform3D area_transform;
	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;
	bool monitorable = false;

	// Space overrides applied to overlapping bodies (see apply_overrides()).
	PhysicsServer3D::AreaSpaceOverrideMode gravity_override_mode = PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;
	real_t gravity = 9.8;
	Vector3 gravity_vector = Vector3(0, -1, 0);
	bool gravity_is_point = false;
	real_t gravity_point_unit_distance = 0.0;

	PhysicsServer3D::AreaSpaceOverrideMode linear_damp_override_mode = PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;
	real_t linear_damp = 0.1;
	PhysicsServer3D::AreaSpaceOverrideMode angular_damp_override_mode = PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED;
	real_t angular_damp = 0.1;

	real_t wind_force_magnitude = 0.0;
	real_t wind_attenuation_factor = 0.0;
	Vector3 wind_source;
	Vector3 wind_direction;

	int priority = 0;

	// Rigid bodies currently overlapping this area, with a shape-pair refcount.
	HashMap<GodotPhysXBody3D *, uint32_t> overlapping_bodies;

	Callable monitor_callback;

	// (body_rid, body_shape, area_shape) -> net enter/exit delta this frame.
	struct OverlapKey {
		RID body_rid;
		uint32_t shape_pair = 0; // body_shape << 16 | area_shape
		bool operator==(const OverlapKey &p_o) const { return body_rid == p_o.body_rid && shape_pair == p_o.shape_pair; }
	};
	struct OverlapKeyHasher {
		static uint32_t hash(const OverlapKey &p_k) { return hash_murmur3_one_32(p_k.shape_pair, p_k.body_rid.get_id()); }
	};
	struct OverlapState {
		ObjectID instance_id;
		int delta = 0;
	};
	HashMap<OverlapKey, OverlapState, OverlapKeyHasher> pending;

	void _destroy_actor();
	void _build_actor();
	void _apply_filter_data();

public:
	void set_self(const RID &p_self) { self = p_self; }
	RID get_self() const { return self; }
	void set_instance_id(ObjectID p_id) { instance_id = p_id; }
	ObjectID get_instance_id() const { return instance_id; }

	physx::PxRigidActor *get_px_actor() const { return px_actor; }
	GodotPhysXSpace3D *get_space() const { return space; }
	void set_space(GodotPhysXSpace3D *p_space);

	void add_shape(GodotPhysXShape3D *p_shape, const Transform3D &p_xform, bool p_disabled);
	void set_shape(int p_idx, GodotPhysXShape3D *p_shape);
	void set_shape_transform(int p_idx, const Transform3D &p_xform);
	void set_shape_disabled(int p_idx, bool p_disabled);
	void remove_shape(int p_idx);
	void clear_shapes();
	int get_shape_count() const { return shapes.size(); }
	const ShapeRef *get_shape_ref(int p_idx) const;

	void set_transform(const Transform3D &p_transform);
	Transform3D get_transform() const { return area_transform; }

	void set_collision_layer(uint32_t p_layer);
	uint32_t get_collision_layer() const { return collision_layer; }
	void set_collision_mask(uint32_t p_mask);
	uint32_t get_collision_mask() const { return collision_mask; }

	void set_monitorable(bool p_monitorable) { monitorable = p_monitorable; }
	bool is_monitorable() const { return monitorable; }

	void set_monitor_callback(const Callable &p_callback) { monitor_callback = p_callback; }

	void set_param(PhysicsServer3D::AreaParameter p_param, const Variant &p_value);
	Variant get_param(PhysicsServer3D::AreaParameter p_param) const;

	int get_priority() const { return priority; }

	// True if this area changes gravity, damping or wind for overlapping bodies.
	bool has_force_override() const {
		return gravity_override_mode != PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED ||
				linear_damp_override_mode != PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED ||
				angular_damp_override_mode != PhysicsServer3D::AREA_SPACE_OVERRIDE_DISABLED ||
				wind_force_magnitude != 0.0;
	}

	// Gravity acceleration this area imposes at a world-space point.
	Vector3 gravity_at(const Vector3 &p_position) const;
	Vector3 wind_at(const Vector3 &p_position) const;

	const HashMap<GodotPhysXBody3D *, uint32_t> &get_overlapping_bodies() const { return overlapping_bodies; }
	PhysicsServer3D::AreaSpaceOverrideMode get_gravity_mode() const { return gravity_override_mode; }
	PhysicsServer3D::AreaSpaceOverrideMode get_linear_damp_mode() const { return linear_damp_override_mode; }
	PhysicsServer3D::AreaSpaceOverrideMode get_angular_damp_mode() const { return angular_damp_override_mode; }
	real_t get_linear_damp_value() const { return linear_damp; }
	real_t get_angular_damp_value() const { return angular_damp; }

	// Called from the space's onTrigger handler.
	void report_body_overlap(GodotPhysXBody3D *p_body, int p_body_shape, int p_area_shape, bool p_entered);
	// Called when a body leaves the simulation while still overlapping.
	void body_removed(GodotPhysXBody3D *p_body) { overlapping_bodies.erase(p_body); }
	// Called from flush_queries.
	void call_queries();

	GodotPhysXArea3D();
	~GodotPhysXArea3D();
};

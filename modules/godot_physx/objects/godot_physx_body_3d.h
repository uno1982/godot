/**************************************************************************/
/*  godot_physx_body_3d.h                                                 */
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
#include "core/templates/hash_set.h"
#include "core/templates/local_vector.h"
#include "core/templates/rid.h"
#include "core/variant/callable.h"
#include "servers/physics_3d/physics_server_3d.h"

namespace physx {
class PxRigidActor;
class PxMaterial;
} //namespace physx

class GodotPhysXSpace3D;
class GodotPhysXShape3D;
class GodotPhysXDirectBodyState3D;
class GodotPhysXJoint3D;

class GodotPhysXBody3D {
public:
	struct ShapeRef {
		GodotPhysXShape3D *shape = nullptr;
		Transform3D xform;
		bool disabled = false;
	};

	struct Contact {
		Vector3 position; // world-space contact point
		Vector3 normal;
		Vector3 impulse;
		int local_shape = 0;
		RID collider;
		ObjectID collider_id;
		int collider_shape = 0;
		Vector3 collider_velocity;
	};

private:
	RID self;
	ObjectID instance_id;

	PhysicsServer3D::BodyMode mode = PhysicsServer3D::BODY_MODE_RIGID;
	GodotPhysXSpace3D *space = nullptr;
	physx::PxRigidActor *px_actor = nullptr;

	LocalVector<ShapeRef> shapes;

	Transform3D body_transform;
	Vector3 linear_velocity;
	Vector3 angular_velocity;
	real_t mass = 1.0;
	real_t gravity_scale = 1.0;
	real_t bounce = 0.0;
	real_t friction = 1.0;
	real_t linear_damp = 0.0;
	real_t angular_damp = 0.0;
	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;
	uint32_t axis_lock = 0; // PhysicsServer3D::BodyAxis bitmask
	bool ccd = false;
	bool can_sleep = true;
	bool sleeping = false;

	physx::PxMaterial *px_material = nullptr;

	Callable body_state_callback;
	GodotPhysXDirectBodyState3D *direct_state = nullptr;

	int max_contacts_reported = 0;
	LocalVector<Contact> contacts;

	HashSet<GodotPhysXJoint3D *> joints;

	// PhysX's default 4 position / 1 velocity iterations are marginal for joint
	// chains; bodies that participate in a joint get a modest bump so pendulums,
	// ragdolls and cloth strips stay taut. TGS (set on the scene) does most of
	// the work, this trims the residual stretch.
	static constexpr uint32_t SOLVER_ITERS_DEFAULT_POS = 4;
	static constexpr uint32_t SOLVER_ITERS_DEFAULT_VEL = 1;
	static constexpr uint32_t SOLVER_ITERS_JOINTED_POS = 8;
	static constexpr uint32_t SOLVER_ITERS_JOINTED_VEL = 2;

	bool _is_dynamic() const;
	void _destroy_actor();
	void _apply_solver_iterations();
	void _build_actor();
	void _apply_filter_data();
	void _apply_damping();
	void _apply_axis_lock();
	physx::PxMaterial *_get_material();

public:
	void set_self(const RID &p_self) { self = p_self; }
	RID get_self() const { return self; }

	void set_instance_id(ObjectID p_id) { instance_id = p_id; }
	ObjectID get_instance_id() const { return instance_id; }

	physx::PxRigidActor *get_px_actor() const { return px_actor; }
	GodotPhysXSpace3D *get_space() const { return space; }

	void set_space(GodotPhysXSpace3D *p_space);

	void set_mode(PhysicsServer3D::BodyMode p_mode);
	PhysicsServer3D::BodyMode get_mode() const { return mode; }

	void add_shape(GodotPhysXShape3D *p_shape, const Transform3D &p_xform, bool p_disabled);
	void set_shape(int p_idx, GodotPhysXShape3D *p_shape);
	void set_shape_transform(int p_idx, const Transform3D &p_xform);
	void set_shape_disabled(int p_idx, bool p_disabled);
	void remove_shape(int p_idx);
	void clear_shapes();
	int get_shape_count() const { return shapes.size(); }
	const ShapeRef *get_shape_ref(int p_idx) const;
	void shape_changed(GodotPhysXShape3D *p_shape);

	void set_param(PhysicsServer3D::BodyParameter p_param, const Variant &p_value);
	Variant get_param(PhysicsServer3D::BodyParameter p_param) const;

	void set_state(PhysicsServer3D::BodyState p_state, const Variant &p_value);
	Variant get_state(PhysicsServer3D::BodyState p_state) const;

	void set_collision_layer(uint32_t p_layer);
	uint32_t get_collision_layer() const { return collision_layer; }
	void set_collision_mask(uint32_t p_mask);
	uint32_t get_collision_mask() const { return collision_mask; }

	void set_ccd(bool p_enable);
	bool is_ccd_enabled() const { return ccd; }

	void set_axis_lock(PhysicsServer3D::BodyAxis p_axis, bool p_lock);
	bool is_axis_locked(PhysicsServer3D::BodyAxis p_axis) const { return axis_lock & p_axis; }

	void set_max_contacts_reported(int p_amount);
	int get_max_contacts_reported() const { return max_contacts_reported; }
	bool reports_contacts() const { return max_contacts_reported > 0; }

	// Called by the space's simulation event callback while contacts are being
	// processed (inside fetchResults).
	void clear_contacts() { contacts.clear(); }
	void add_contact(const Contact &p_contact);
	int get_contact_count() const { return contacts.size(); }
	const Contact &get_contact(int p_idx) const { return contacts[p_idx]; }

	void set_state_sync_callback(const Callable &p_callable) { body_state_callback = p_callable; }

	// Called by the space after fetchResults(): pull the simulated pose/velocity
	// back onto this wrapper.
	void pull_transform_from_px();
	// Called by the space during flush_queries().
	void call_queries();

	Transform3D get_transform() const { return body_transform; }
	Vector3 get_linear_velocity() const;
	Vector3 get_angular_velocity() const;
	void set_linear_velocity(const Vector3 &p_v);
	void set_angular_velocity(const Vector3 &p_v);
	real_t get_mass() const { return mass; }
	real_t get_gravity_scale() const { return gravity_scale; }
	real_t get_linear_damp() const { return linear_damp; }
	real_t get_angular_damp() const { return angular_damp; }
	bool is_sleeping() const;
	void set_sleep_state(bool p_sleep);
	void apply_impulse(const Vector3 &p_impulse, const Vector3 &p_position);
	void apply_central_impulse(const Vector3 &p_impulse);
	void apply_torque_impulse(const Vector3 &p_impulse);
	void apply_central_force(const Vector3 &p_force);
	void apply_torque(const Vector3 &p_torque);

	GodotPhysXDirectBodyState3D *get_direct_state();

	void add_joint(GodotPhysXJoint3D *p_joint) {
		joints.insert(p_joint);
		_apply_solver_iterations();
	}
	void remove_joint(GodotPhysXJoint3D *p_joint) {
		joints.erase(p_joint);
		_apply_solver_iterations();
	}

	GodotPhysXBody3D();
	~GodotPhysXBody3D();
};

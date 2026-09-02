/**************************************************************************/
/*  godot_physx_joint_3d.h                                                */
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
#include "core/templates/rid.h"
#include "servers/physics_3d/physics_server_3d.h"

namespace physx {
class PxJoint;
} //namespace physx

class GodotPhysXBody3D;

class GodotPhysXJoint3D {
	using JointType = PhysicsServer3D::JointType;

	RID self;
	JointType type = JointType::JOINT_TYPE_MAX;

	GodotPhysXBody3D *body_a = nullptr;
	GodotPhysXBody3D *body_b = nullptr;
	Transform3D frame_a;
	Transform3D frame_b;

	physx::PxJoint *px_joint = nullptr;
	bool collisions_disabled = true;
	int solver_priority = 1;

	// Per-type parameters (defaults chosen to be inert).
	real_t hinge_lower = 0.0, hinge_upper = 0.0;
	bool hinge_use_limit = false, hinge_motor = false;
	real_t hinge_motor_velocity = 0.0, hinge_motor_max_impulse = 0.0;

	real_t slider_lower = 0.0, slider_upper = 0.0;

	real_t cone_swing = Math::PI, cone_twist = Math::PI;

	struct Axis6DOF {
		real_t lin_lower = 0.0, lin_upper = 0.0;
		real_t ang_lower = 0.0, ang_upper = 0.0;
		bool lin_limit = false, ang_limit = false;
		bool lin_motor = false, ang_motor = false;
		real_t lin_motor_target = 0.0, lin_motor_force = 0.0;
		real_t ang_motor_target = 0.0, ang_motor_force = 0.0;
	} axis6[3];

	void _destroy();
	physx::PxJoint *_create() const;
	void _apply_params();

public:
	void set_self(const RID &p_self) { self = p_self; }
	RID get_self() const { return self; }
	JointType get_type() const { return type; }

	void set_solver_priority(int p_priority) { solver_priority = p_priority; }
	int get_solver_priority() const { return solver_priority; }

	void set_collisions_disabled(bool p_disabled);
	bool are_collisions_disabled() const { return collisions_disabled; }

	void clear();
	void rebuild(); // called when a jointed body's actor changes
	void body_removed(GodotPhysXBody3D *p_body);

	void make_pin(GodotPhysXBody3D *p_a, const Vector3 &p_local_a, GodotPhysXBody3D *p_b, const Vector3 &p_local_b);
	void make_hinge(GodotPhysXBody3D *p_a, const Transform3D &p_frame_a, GodotPhysXBody3D *p_b, const Transform3D &p_frame_b);
	void make_slider(GodotPhysXBody3D *p_a, const Transform3D &p_frame_a, GodotPhysXBody3D *p_b, const Transform3D &p_frame_b);
	void make_cone_twist(GodotPhysXBody3D *p_a, const Transform3D &p_frame_a, GodotPhysXBody3D *p_b, const Transform3D &p_frame_b);
	void make_6dof(GodotPhysXBody3D *p_a, const Transform3D &p_frame_a, GodotPhysXBody3D *p_b, const Transform3D &p_frame_b);

	void set_pin_local_a(const Vector3 &p_a);
	void set_pin_local_b(const Vector3 &p_b);
	Vector3 get_pin_local_a() const { return frame_a.origin; }
	Vector3 get_pin_local_b() const { return frame_b.origin; }

	void set_hinge_param(PhysicsServer3D::HingeJointParam p_param, real_t p_value);
	real_t get_hinge_param(PhysicsServer3D::HingeJointParam p_param) const;
	void set_hinge_flag(PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled);
	bool get_hinge_flag(PhysicsServer3D::HingeJointFlag p_flag) const;

	void set_slider_param(PhysicsServer3D::SliderJointParam p_param, real_t p_value);
	real_t get_slider_param(PhysicsServer3D::SliderJointParam p_param) const;

	void set_cone_twist_param(PhysicsServer3D::ConeTwistJointParam p_param, real_t p_value);
	real_t get_cone_twist_param(PhysicsServer3D::ConeTwistJointParam p_param) const;

	void set_6dof_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param, real_t p_value);
	real_t get_6dof_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const;
	void set_6dof_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag, bool p_enable);
	bool get_6dof_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const;

	~GodotPhysXJoint3D();
};

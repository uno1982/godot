/**************************************************************************/
/*  physx_info.cpp                                                        */
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

#include "physx_info.h"

#include "godot_physx_server_3d.h"
#include "joints/godot_physx_joint_3d.h"
#include "objects/godot_physx_body_3d.h"

#include "core/object/class_db.h"
#include "scene/3d/physics/physical_bone_3d.h"

bool PhysX::is_gpu_available() {
	const GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	return server != nullptr && server->has_gpu();
}

String PhysX::get_gpu_device_name() {
	const GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	return server != nullptr ? server->get_gpu_device_name() : String();
}

// The joint a drive call acts on, looked up where the server may touch it (the calls run through
// GodotPhysXServer3D::run_on_physics_thread(), in order behind the commands sent before them).
static GodotPhysXJoint3D *_drive_joint(const RID &p_joint) {
	GodotPhysXJoint3D *joint = GodotPhysXServer3D::get_singleton()->get_joint(p_joint);
	ERR_FAIL_NULL_V_MSG(joint, nullptr, "PhysX: not a joint RID.");
	// A joint not made yet keeps the settings for when it is (a Joint3D node makes it once its bodies are in the tree).
	ERR_FAIL_COND_V_MSG(joint->get_type() == PhysicsServer3D::JOINT_TYPE_SLIDER, nullptr, "PhysX: a slider joint has no drives (pin, hinge, cone twist and Generic6DOF joints do).");
	return joint;
}

template <typename F>
static void _set_on_joint(const RID &p_joint, const F &p_func) {
	ERR_FAIL_NULL_MSG(GodotPhysXServer3D::get_singleton(), "PhysX: PhysX is not the active physics engine.");
	GodotPhysXServer3D::run_on_physics_thread([=]() {
		if (GodotPhysXJoint3D *joint = _drive_joint(p_joint)) {
			p_func(joint);
		}
	});
}

template <typename T, typename F>
static T _get_from_joint(const RID &p_joint, const T &p_default, const F &p_func) {
	ERR_FAIL_NULL_V_MSG(GodotPhysXServer3D::get_singleton(), p_default, "PhysX: PhysX is not the active physics engine.");
	T value = p_default;
	GodotPhysXServer3D::run_on_physics_thread([&]() {
		if (const GodotPhysXJoint3D *joint = _drive_joint(p_joint)) {
			value = p_func(joint);
		}
	},
			true);
	return value;
}

void PhysX::joint_set_slerp_drive(const RID &p_joint, bool p_enabled, real_t p_stiffness, real_t p_damping, real_t p_torque_limit) {
	_set_on_joint(p_joint, [=](GodotPhysXJoint3D *p_j) { p_j->set_slerp_drive(p_enabled, p_stiffness, p_damping, p_torque_limit); });
}

bool PhysX::joint_is_slerp_drive_enabled(const RID &p_joint) {
	return _get_from_joint(p_joint, false, [](const GodotPhysXJoint3D *p_j) { return p_j->is_slerp_drive_enabled(); });
}

void PhysX::joint_set_drive_acceleration(const RID &p_joint, bool p_enabled) {
	_set_on_joint(p_joint, [=](GodotPhysXJoint3D *p_j) { p_j->set_drive_acceleration(p_enabled); });
}

bool PhysX::joint_is_drive_acceleration(const RID &p_joint) {
	return _get_from_joint(p_joint, false, [](const GodotPhysXJoint3D *p_j) { return p_j->is_drive_acceleration(); });
}

void PhysX::joint_set_linear_spring_force_limit(const RID &p_joint, Vector3::Axis p_axis, real_t p_limit) {
	ERR_FAIL_INDEX(p_axis, 3);
	_set_on_joint(p_joint, [=](GodotPhysXJoint3D *p_j) { p_j->set_spring_force_limit(p_axis, false, p_limit); });
}

real_t PhysX::joint_get_linear_spring_force_limit(const RID &p_joint, Vector3::Axis p_axis) {
	ERR_FAIL_INDEX_V(p_axis, 3, Math::INF);
	return _get_from_joint(p_joint, (real_t)Math::INF, [=](const GodotPhysXJoint3D *p_j) { return p_j->get_spring_force_limit(p_axis, false); });
}

void PhysX::joint_set_angular_spring_torque_limit(const RID &p_joint, Vector3::Axis p_axis, real_t p_limit) {
	ERR_FAIL_INDEX(p_axis, 3);
	_set_on_joint(p_joint, [=](GodotPhysXJoint3D *p_j) { p_j->set_spring_force_limit(p_axis, true, p_limit); });
}

real_t PhysX::joint_get_angular_spring_torque_limit(const RID &p_joint, Vector3::Axis p_axis) {
	ERR_FAIL_INDEX_V(p_axis, 3, Math::INF);
	return _get_from_joint(p_joint, (real_t)Math::INF, [=](const GodotPhysXJoint3D *p_j) { return p_j->get_spring_force_limit(p_axis, true); });
}

void PhysX::joint_set_drive_target(const RID &p_joint, const Transform3D &p_target, real_t p_delta) {
	_set_on_joint(p_joint, [=](GodotPhysXJoint3D *p_j) { p_j->set_drive_target(p_target, p_delta); });
}

void PhysX::joint_set_drive_target_from_poses(const RID &p_joint, const Transform3D &p_pose_a, const Transform3D &p_pose_b, real_t p_delta) {
	_set_on_joint(p_joint, [=](GodotPhysXJoint3D *p_j) { p_j->set_drive_target_from_poses(p_pose_a, p_pose_b, p_delta); });
}

Transform3D PhysX::joint_get_drive_target(const RID &p_joint) {
	return _get_from_joint(p_joint, Transform3D(), [](const GodotPhysXJoint3D *p_j) { return p_j->get_drive_target(); });
}

RID PhysX::physical_bone_get_joint(PhysicalBone3D *p_bone) {
	ERR_FAIL_NULL_V(p_bone, RID());
	ERR_FAIL_NULL_V_MSG(GodotPhysXServer3D::get_singleton(), RID(), "PhysX: PhysX is not the active physics engine.");
	// The bone's joint ties its parent bone (body A) to it (body B).
	const RID body_rid = p_bone->get_rid();
	RID joint_rid;
	GodotPhysXServer3D::run_on_physics_thread([&]() {
		const GodotPhysXBody3D *body = GodotPhysXServer3D::get_singleton()->get_body(body_rid);
		if (!body) {
			return;
		}
		for (const GodotPhysXJoint3D *joint : body->get_joints()) {
			if (joint->get_body_b() == body) {
				joint_rid = joint->get_self();
				return;
			}
		}
	},
			true);
	return joint_rid;
}

void PhysX::_bind_methods() {
	ClassDB::bind_static_method("PhysX", D_METHOD("is_gpu_available"), &PhysX::is_gpu_available);
	ClassDB::bind_static_method("PhysX", D_METHOD("get_gpu_device_name"), &PhysX::get_gpu_device_name);

	ClassDB::bind_static_method("PhysX", D_METHOD("joint_set_slerp_drive", "joint", "enabled", "stiffness", "damping", "torque_limit"), &PhysX::joint_set_slerp_drive, DEFVAL(0.0), DEFVAL(0.0), DEFVAL(Math::INF));
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_is_slerp_drive_enabled", "joint"), &PhysX::joint_is_slerp_drive_enabled);
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_set_drive_acceleration", "joint", "enabled"), &PhysX::joint_set_drive_acceleration);
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_is_drive_acceleration", "joint"), &PhysX::joint_is_drive_acceleration);
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_set_linear_spring_force_limit", "joint", "axis", "limit"), &PhysX::joint_set_linear_spring_force_limit);
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_get_linear_spring_force_limit", "joint", "axis"), &PhysX::joint_get_linear_spring_force_limit);
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_set_angular_spring_torque_limit", "joint", "axis", "limit"), &PhysX::joint_set_angular_spring_torque_limit);
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_get_angular_spring_torque_limit", "joint", "axis"), &PhysX::joint_get_angular_spring_torque_limit);
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_set_drive_target", "joint", "target", "delta"), &PhysX::joint_set_drive_target, DEFVAL(0.0));
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_set_drive_target_from_poses", "joint", "pose_a", "pose_b", "delta"), &PhysX::joint_set_drive_target_from_poses, DEFVAL(0.0));
	ClassDB::bind_static_method("PhysX", D_METHOD("joint_get_drive_target", "joint"), &PhysX::joint_get_drive_target);
	ClassDB::bind_static_method("PhysX", D_METHOD("physical_bone_get_joint", "bone"), &PhysX::physical_bone_get_joint);
}

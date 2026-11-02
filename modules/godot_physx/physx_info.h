/**************************************************************************/
/*  physx_info.h                                                          */
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
#include "core/object/object.h"
#include "core/templates/rid.h"

class PhysicalBone3D;

// Script-side access to the PhysX backend itself (not tied to any node): PhysX.is_gpu_available(),
// PhysX.get_gpu_device_name(), and the joint drives Godot's API has no room for (active ragdolls).
class PhysX : public Object {
	GDCLASS(PhysX, Object);

protected:
	static void _bind_methods();

public:
	static bool is_gpu_available();
	static String get_gpu_device_name();

	static void joint_set_slerp_drive(const RID &p_joint, bool p_enabled, real_t p_stiffness, real_t p_damping, real_t p_torque_limit);
	static bool joint_is_slerp_drive_enabled(const RID &p_joint);
	static void joint_set_drive_acceleration(const RID &p_joint, bool p_enabled);
	static bool joint_is_drive_acceleration(const RID &p_joint);
	static void joint_set_linear_spring_force_limit(const RID &p_joint, Vector3::Axis p_axis, real_t p_limit);
	static real_t joint_get_linear_spring_force_limit(const RID &p_joint, Vector3::Axis p_axis);
	static void joint_set_angular_spring_torque_limit(const RID &p_joint, Vector3::Axis p_axis, real_t p_limit);
	static real_t joint_get_angular_spring_torque_limit(const RID &p_joint, Vector3::Axis p_axis);
	static void joint_set_drive_target(const RID &p_joint, const Transform3D &p_target, real_t p_delta);
	static void joint_set_drive_target_from_poses(const RID &p_joint, const Transform3D &p_pose_a, const Transform3D &p_pose_b, real_t p_delta);
	static Transform3D joint_get_drive_target(const RID &p_joint);
	static RID physical_bone_get_joint(PhysicalBone3D *p_bone);
};

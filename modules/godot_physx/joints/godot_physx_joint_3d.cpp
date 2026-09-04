/**************************************************************************/
/*  godot_physx_joint_3d.cpp                                              */
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

#include "godot_physx_joint_3d.h"

#include "../godot_physx_conversions.h"
#include "../objects/godot_physx_body_3d.h"
#include "../spaces/godot_physx_space_3d.h"

#include "core/error/error_macros.h"

#include <PxPhysicsAPI.h>
#include <extensions/PxD6Joint.h>
#include <extensions/PxPrismaticJoint.h>
#include <extensions/PxRevoluteJoint.h>
#include <extensions/PxSphericalJoint.h>

using namespace physx;

// Rotation mapping the frame's +Z (Godot hinge axis) onto PhysX's +X.
static const PxQuat HINGE_Z_TO_X(-PxHalfPi, PxVec3(0, 1, 0));

GodotPhysXJoint3D::~GodotPhysXJoint3D() {
	clear();
}

void GodotPhysXJoint3D::_destroy() {
	if (px_joint) {
		px_joint->release();
		px_joint = nullptr;
	}
}

void GodotPhysXJoint3D::clear() {
	_destroy();
	if (body_a) {
		body_a->remove_joint(this);
	}
	if (body_b) {
		body_b->remove_joint(this);
	}
	body_a = nullptr;
	body_b = nullptr;
	type = JointType::JOINT_TYPE_MAX;
}

void GodotPhysXJoint3D::body_removed(GodotPhysXBody3D *p_body) {
	if (body_a == p_body || body_b == p_body) {
		_destroy();
		if (body_a == p_body) {
			body_a = nullptr;
		}
		if (body_b == p_body) {
			body_b = nullptr;
		}
	}
}

PxJoint *GodotPhysXJoint3D::_create() const {
	GodotPhysXSpace3D *space = body_a ? body_a->get_space() : (body_b ? body_b->get_space() : nullptr);
	if (!space || !space->get_px_physics()) {
		return nullptr;
	}
	// Both bodies must live in the same space.
	if (body_a && body_b && body_a->get_space() != body_b->get_space()) {
		return nullptr;
	}
	PxPhysics *physics = space->get_px_physics();
	PxRigidActor *actor_a = body_a ? body_a->get_px_actor() : nullptr;
	PxRigidActor *actor_b = body_b ? body_b->get_px_actor() : nullptr;
	if (!actor_a && !actor_b) {
		return nullptr;
	}

	switch (type) {
		case JointType::JOINT_TYPE_PIN: {
			// A D6 with the three linear axes locked and all rotation free is
			// kinematically a ball joint, but PhysX's D6 solver path holds a
			// chain of them together far better than PxSphericalJoint under
			// sustained external load (wind).
			return PxD6JointCreate(*physics,
					actor_a, PxTransform(to_px(frame_a.origin)),
					actor_b, PxTransform(to_px(frame_b.origin)));
		}
		case JointType::JOINT_TYPE_HINGE: {
			return PxRevoluteJointCreate(*physics,
					actor_a, PxTransform(to_px(frame_a)) * PxTransform(HINGE_Z_TO_X),
					actor_b, PxTransform(to_px(frame_b)) * PxTransform(HINGE_Z_TO_X));
		}
		case JointType::JOINT_TYPE_SLIDER: {
			return PxPrismaticJointCreate(*physics,
					actor_a, PxTransform(to_px(frame_a)),
					actor_b, PxTransform(to_px(frame_b)));
		}
		case JointType::JOINT_TYPE_CONE_TWIST:
		case JointType::JOINT_TYPE_6DOF: {
			return PxD6JointCreate(*physics,
					actor_a, PxTransform(to_px(frame_a)),
					actor_b, PxTransform(to_px(frame_b)));
		}
		default:
			return nullptr;
	}
}

void GodotPhysXJoint3D::_apply_params() {
	if (!px_joint) {
		return;
	}
	px_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, !collisions_disabled);
	const PxTolerancesScale scale = px_joint->getScene() ? px_joint->getScene()->getPhysics().getTolerancesScale() : PxTolerancesScale();

	switch (type) {
		case JointType::JOINT_TYPE_PIN: {
			PxD6Joint *j = static_cast<PxD6Joint *>(px_joint);
			j->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
			j->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
			j->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
			j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
			j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
			j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
		} break;

		case JointType::JOINT_TYPE_HINGE: {
			PxRevoluteJoint *j = static_cast<PxRevoluteJoint *>(px_joint);
			j->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, hinge_use_limit);
			if (hinge_use_limit) {
				j->setLimit(PxJointAngularLimitPair((PxReal)hinge_lower, (PxReal)hinge_upper));
			}
			j->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, hinge_motor);
			if (hinge_motor) {
				j->setDriveVelocity((PxReal)hinge_motor_velocity);
				j->setDriveForceLimit((PxReal)(hinge_motor_max_impulse > 0.0 ? hinge_motor_max_impulse : PX_MAX_F32));
			}
		} break;

		case JointType::JOINT_TYPE_SLIDER: {
			PxPrismaticJoint *j = static_cast<PxPrismaticJoint *>(px_joint);
			const bool limited = slider_upper > slider_lower;
			j->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, limited);
			if (limited) {
				j->setLimit(PxJointLinearLimitPair(scale, (PxReal)slider_lower, (PxReal)slider_upper));
			}
		} break;

		case JointType::JOINT_TYPE_CONE_TWIST: {
			PxD6Joint *j = static_cast<PxD6Joint *>(px_joint);
			j->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
			j->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
			j->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
			j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
			j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
			j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
			j->setTwistLimit(PxJointAngularLimitPair((PxReal)-cone_twist, (PxReal)cone_twist));
			j->setSwingLimit(PxJointLimitCone((PxReal)cone_swing, (PxReal)cone_swing));
		} break;

		case JointType::JOINT_TYPE_6DOF: {
			PxD6Joint *j = static_cast<PxD6Joint *>(px_joint);
			const PxD6Axis::Enum lin_axes[3] = { PxD6Axis::eX, PxD6Axis::eY, PxD6Axis::eZ };
			const PxD6Axis::Enum ang_axes[3] = { PxD6Axis::eTWIST, PxD6Axis::eSWING1, PxD6Axis::eSWING2 };
			for (int a = 0; a < 3; a++) {
				const Axis6DOF &ax = axis6[a];
				if (ax.lin_limit) {
					j->setMotion(lin_axes[a], (ax.lin_upper > ax.lin_lower) ? PxD6Motion::eLIMITED : PxD6Motion::eLOCKED);
					if (ax.lin_upper > ax.lin_lower) {
						j->setLinearLimit(lin_axes[a], PxJointLinearLimitPair(scale, (PxReal)ax.lin_lower, (PxReal)ax.lin_upper));
					}
				} else {
					j->setMotion(lin_axes[a], PxD6Motion::eFREE);
				}
				if (ax.ang_limit) {
					j->setMotion(ang_axes[a], (ax.ang_upper > ax.ang_lower) ? PxD6Motion::eLIMITED : PxD6Motion::eLOCKED);
				} else {
					j->setMotion(ang_axes[a], PxD6Motion::eFREE);
				}
			}
			j->setTwistLimit(PxJointAngularLimitPair((PxReal)axis6[0].ang_lower, (PxReal)axis6[0].ang_upper));
			j->setPyramidSwingLimit(PxJointLimitPyramid(
					(PxReal)axis6[1].ang_lower, (PxReal)axis6[1].ang_upper,
					(PxReal)axis6[2].ang_lower, (PxReal)axis6[2].ang_upper));

			// Linear drives: a spring (stiffness toward an equilibrium point) and a
			// velocity motor share the per-axis drive slot.
			PxVec3 drive_pos(0.0f);
			PxVec3 lin_vel_target(0.0f);
			bool any_lin_drive = false;
			for (int a = 0; a < 3; a++) {
				const Axis6DOF &ax = axis6[a];
				if (!ax.lin_spring && !ax.lin_motor) {
					continue;
				}
				any_lin_drive = true;
				const PxReal k = ax.lin_spring ? (PxReal)ax.lin_spring_stiffness : 0.0f;
				const PxReal d = ax.lin_spring ? (PxReal)ax.lin_spring_damping : 1.0e6f;
				const PxReal f = (ax.lin_motor && ax.lin_motor_force > 0.0) ? (PxReal)ax.lin_motor_force : PX_MAX_F32;
				j->setDrive((PxD6Drive::Enum)(PxD6Drive::eX + a), PxD6JointDrive(k, d, f));
				if (ax.lin_spring) {
					(&drive_pos.x)[a] = (PxReal)ax.lin_spring_eq;
				}
				if (ax.lin_motor) {
					(&lin_vel_target.x)[a] = (PxReal)ax.lin_motor_target;
				}
				// A drive only acts on an axis that is not locked. Free any axis
				// that has a spring but no valid limit range.
				if (ax.lin_spring && !(ax.lin_limit && ax.lin_upper > ax.lin_lower)) {
					j->setMotion(lin_axes[a], PxD6Motion::eFREE);
				}
			}

			// Angular springs: axis X -> twist drive, axis Y -> swing1, axis Z ->
			// swing2. The drive target rotation is built from the equilibrium angles.
			const PxD6Drive::Enum ang_drives[3] = { PxD6Drive::eTWIST, PxD6Drive::eSWING1, PxD6Drive::eSWING2 };
			const PxVec3 ang_drive_axis[3] = { PxVec3(1, 0, 0), PxVec3(0, 1, 0), PxVec3(0, 0, 1) };
			PxQuat drive_rot(PxIdentity);
			for (int a = 0; a < 3; a++) {
				const Axis6DOF &ax = axis6[a];
				if (!ax.ang_spring) {
					continue;
				}
				j->setDrive(ang_drives[a], PxD6JointDrive((PxReal)ax.ang_spring_stiffness, (PxReal)ax.ang_spring_damping, PX_MAX_F32));
				drive_rot = drive_rot * PxQuat((PxReal)ax.ang_spring_eq, ang_drive_axis[a]);
				any_lin_drive = true;
				if (!(ax.ang_limit && ax.ang_upper > ax.ang_lower)) {
					j->setMotion(ang_axes[a], PxD6Motion::eFREE);
				}
			}

			if (any_lin_drive) {
				j->setDrivePosition(PxTransform(drive_pos, drive_rot), true);
				j->setDriveVelocity(lin_vel_target, PxVec3(0.0f));
			}
		} break;

		default:
			break;
	}
}

void GodotPhysXJoint3D::rebuild() {
	if (type == JointType::JOINT_TYPE_MAX) {
		return;
	}
	_destroy();
	px_joint = _create();
	if (px_joint) {
		_apply_params();
	}
}

void GodotPhysXJoint3D::set_collisions_disabled(bool p_disabled) {
	collisions_disabled = p_disabled;
	if (px_joint) {
		px_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, !collisions_disabled);
	}
}

/* --- make_* -------------------------------------------------------------- */

void GodotPhysXJoint3D::make_pin(GodotPhysXBody3D *p_a, const Vector3 &p_local_a, GodotPhysXBody3D *p_b, const Vector3 &p_local_b) {
	clear();
	type = JointType::JOINT_TYPE_PIN;
	body_a = p_a;
	body_b = p_b;
	frame_a = Transform3D(Basis(), p_local_a);
	frame_b = Transform3D(Basis(), p_local_b);
	if (body_a) {
		body_a->add_joint(this);
	}
	if (body_b) {
		body_b->add_joint(this);
	}
	rebuild();
}

void GodotPhysXJoint3D::make_hinge(GodotPhysXBody3D *p_a, const Transform3D &p_frame_a, GodotPhysXBody3D *p_b, const Transform3D &p_frame_b) {
	clear();
	type = JointType::JOINT_TYPE_HINGE;
	body_a = p_a;
	body_b = p_b;
	frame_a = p_frame_a;
	frame_b = p_frame_b;
	if (body_a) {
		body_a->add_joint(this);
	}
	if (body_b) {
		body_b->add_joint(this);
	}
	rebuild();
}

void GodotPhysXJoint3D::make_slider(GodotPhysXBody3D *p_a, const Transform3D &p_frame_a, GodotPhysXBody3D *p_b, const Transform3D &p_frame_b) {
	clear();
	type = JointType::JOINT_TYPE_SLIDER;
	body_a = p_a;
	body_b = p_b;
	frame_a = p_frame_a;
	frame_b = p_frame_b;
	if (body_a) {
		body_a->add_joint(this);
	}
	if (body_b) {
		body_b->add_joint(this);
	}
	rebuild();
}

void GodotPhysXJoint3D::make_cone_twist(GodotPhysXBody3D *p_a, const Transform3D &p_frame_a, GodotPhysXBody3D *p_b, const Transform3D &p_frame_b) {
	clear();
	type = JointType::JOINT_TYPE_CONE_TWIST;
	body_a = p_a;
	body_b = p_b;
	frame_a = p_frame_a;
	frame_b = p_frame_b;
	if (body_a) {
		body_a->add_joint(this);
	}
	if (body_b) {
		body_b->add_joint(this);
	}
	rebuild();
}

void GodotPhysXJoint3D::make_6dof(GodotPhysXBody3D *p_a, const Transform3D &p_frame_a, GodotPhysXBody3D *p_b, const Transform3D &p_frame_b) {
	clear();
	type = JointType::JOINT_TYPE_6DOF;
	body_a = p_a;
	body_b = p_b;
	frame_a = p_frame_a;
	frame_b = p_frame_b;
	if (body_a) {
		body_a->add_joint(this);
	}
	if (body_b) {
		body_b->add_joint(this);
	}
	rebuild();
}

/* --- pin --------------------------------------------------------------- */

void GodotPhysXJoint3D::set_pin_local_a(const Vector3 &p_a) {
	frame_a.origin = p_a;
	rebuild();
}

void GodotPhysXJoint3D::set_pin_local_b(const Vector3 &p_b) {
	frame_b.origin = p_b;
	rebuild();
}

/* --- hinge ----------------------------------------------------------------- */

void GodotPhysXJoint3D::set_hinge_param(PhysicsServer3D::HingeJointParam p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER:
			hinge_lower = p_value;
			break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER:
			hinge_upper = p_value;
			break;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY:
			hinge_motor_velocity = p_value;
			break;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE:
			hinge_motor_max_impulse = p_value;
			break;
		default:
			break; // bias / softness / relaxation not mapped
	}
	_apply_params();
}

real_t GodotPhysXJoint3D::get_hinge_param(PhysicsServer3D::HingeJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER:
			return hinge_lower;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER:
			return hinge_upper;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY:
			return hinge_motor_velocity;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE:
			return hinge_motor_max_impulse;
		default:
			return 0.0;
	}
}

void GodotPhysXJoint3D::set_hinge_flag(PhysicsServer3D::HingeJointFlag p_flag, bool p_enabled) {
	if (p_flag == PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT) {
		hinge_use_limit = p_enabled;
	} else if (p_flag == PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR) {
		hinge_motor = p_enabled;
	}
	_apply_params();
}

bool GodotPhysXJoint3D::get_hinge_flag(PhysicsServer3D::HingeJointFlag p_flag) const {
	if (p_flag == PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT) {
		return hinge_use_limit;
	}
	if (p_flag == PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR) {
		return hinge_motor;
	}
	return false;
}

/* --- slider -------------------------------------------------------------- */

void GodotPhysXJoint3D::set_slider_param(PhysicsServer3D::SliderJointParam p_param, real_t p_value) {
	if (p_param == PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_LOWER) {
		slider_lower = p_value;
	} else if (p_param == PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_UPPER) {
		slider_upper = p_value;
	}
	_apply_params();
}

real_t GodotPhysXJoint3D::get_slider_param(PhysicsServer3D::SliderJointParam p_param) const {
	if (p_param == PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_LOWER) {
		return slider_lower;
	}
	if (p_param == PhysicsServer3D::SLIDER_JOINT_LINEAR_LIMIT_UPPER) {
		return slider_upper;
	}
	return 0.0;
}

/* --- cone twist ---------------------------------------------------------- */

void GodotPhysXJoint3D::set_cone_twist_param(PhysicsServer3D::ConeTwistJointParam p_param, real_t p_value) {
	if (p_param == PhysicsServer3D::CONE_TWIST_JOINT_SWING_SPAN) {
		cone_swing = CLAMP(p_value, (real_t)0.01, (real_t)Math::PI);
	} else if (p_param == PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN) {
		cone_twist = CLAMP(p_value, (real_t)0.01, (real_t)Math::PI);
	}
	_apply_params();
}

real_t GodotPhysXJoint3D::get_cone_twist_param(PhysicsServer3D::ConeTwistJointParam p_param) const {
	if (p_param == PhysicsServer3D::CONE_TWIST_JOINT_SWING_SPAN) {
		return cone_swing;
	}
	if (p_param == PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN) {
		return cone_twist;
	}
	return 0.0;
}

/* --- generic 6dof ------------------------------------------------------------ */

void GodotPhysXJoint3D::set_6dof_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param, real_t p_value) {
	Axis6DOF &ax = axis6[p_axis];
	switch (p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LOWER_LIMIT:
			ax.lin_lower = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT:
			ax.lin_upper = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT:
			ax.ang_lower = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT:
			ax.ang_upper = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_TARGET_VELOCITY:
			ax.lin_motor_target = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_MOTOR_FORCE_LIMIT:
			ax.lin_motor_force = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_TARGET_VELOCITY:
			ax.ang_motor_target = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_MOTOR_FORCE_LIMIT:
			ax.ang_motor_force = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS:
			ax.lin_spring_stiffness = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
			ax.lin_spring_damping = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
			ax.lin_spring_eq = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS:
			ax.ang_spring_stiffness = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_DAMPING:
			ax.ang_spring_damping = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT:
			ax.ang_spring_eq = p_value;
			break;
		default:
			break; // softness / restitution / ERP not mapped
	}
	_apply_params();
}

real_t GodotPhysXJoint3D::get_6dof_param(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisParam p_param) const {
	const Axis6DOF &ax = axis6[p_axis];
	switch (p_param) {
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_LOWER_LIMIT:
			return ax.lin_lower;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_UPPER_LIMIT:
			return ax.lin_upper;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LOWER_LIMIT:
			return ax.ang_lower;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_UPPER_LIMIT:
			return ax.ang_upper;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_STIFFNESS:
			return ax.lin_spring_stiffness;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_DAMPING:
			return ax.lin_spring_damping;
		case PhysicsServer3D::G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT:
			return ax.lin_spring_eq;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS:
			return ax.ang_spring_stiffness;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_DAMPING:
			return ax.ang_spring_damping;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT:
			return ax.ang_spring_eq;
		default:
			return 0.0;
	}
}

void GodotPhysXJoint3D::set_6dof_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag, bool p_enable) {
	Axis6DOF &ax = axis6[p_axis];
	switch (p_flag) {
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT:
			ax.lin_limit = p_enable;
			break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT:
			ax.ang_limit = p_enable;
			break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR:
			ax.lin_motor = p_enable;
			break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR:
			ax.ang_motor = p_enable;
			break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING:
			ax.lin_spring = p_enable;
			break;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING:
			ax.ang_spring = p_enable;
			break;
		default:
			break;
	}
	_apply_params();
}

bool GodotPhysXJoint3D::get_6dof_flag(Vector3::Axis p_axis, PhysicsServer3D::G6DOFJointAxisFlag p_flag) const {
	const Axis6DOF &ax = axis6[p_axis];
	switch (p_flag) {
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT:
			return ax.lin_limit;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT:
			return ax.ang_limit;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_MOTOR:
			return ax.lin_motor;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_MOTOR:
			return ax.ang_motor;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING:
			return ax.lin_spring;
		case PhysicsServer3D::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING:
			return ax.ang_spring;
		default:
			return false;
	}
}

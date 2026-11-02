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

#include "core/config/engine.h"
#include "core/error/error_macros.h"

#include <PxPhysicsAPI.h>
#include <extensions/PxD6Joint.h>
#include <extensions/PxPrismaticJoint.h>
#include <extensions/PxRevoluteJoint.h>
#include <extensions/PxSphericalJoint.h>

using namespace physx;

// Rotation mapping the frame's +Z (Godot hinge axis) onto PhysX's +X.
static const PxQuat HINGE_Z_TO_X(-PxHalfPi, PxVec3(0, 1, 0));

// A drive's force / torque limit; infinite (or a negative value) for none.
static PxReal _drive_limit(real_t p_limit) {
	return (Math::is_finite(p_limit) && p_limit >= 0.0) ? (PxReal)MIN(p_limit, (real_t)PX_MAX_F32) : PX_MAX_F32;
}

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

bool GodotPhysXJoint3D::has_running_motor() const {
	if (type == JointType::JOINT_TYPE_HINGE) {
		return hinge_motor && hinge_motor_velocity != 0.0;
	}
	if (type == JointType::JOINT_TYPE_6DOF) {
		for (const Axis6DOF &ax : axis6) {
			if ((ax.lin_motor && ax.lin_motor_target != 0.0) || (ax.ang_motor && ax.ang_motor_target != 0.0)) {
				return true;
			}
		}
	}
	return false;
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
			if (hinge_d6) {
				// A D6 with only the twist free can drive the hinge angle toward a target (an active ragdoll's knees
				// and elbows); the revolute joint's drive is a velocity motor only.
				return PxD6JointCreate(*physics,
						actor_a, PxTransform(to_px(frame_a)) * PxTransform(HINGE_Z_TO_X),
						actor_b, PxTransform(to_px(frame_b)) * PxTransform(HINGE_Z_TO_X));
			}
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
	if (body_a) {
		body_a->update_motor_driven();
	}
	if (body_b) {
		body_b->update_motor_driven();
	}
	if (!px_joint) {
		return;
	}
	px_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, !collisions_disabled);
	const PxTolerancesScale scale = px_joint->getScene() ? px_joint->getScene()->getPhysics().getTolerancesScale() : PxTolerancesScale();

	PxD6Joint *d6 = _is_px_d6() ? static_cast<PxD6Joint *>(px_joint) : nullptr;
	if (d6) {
		// Start from no drives: a spring or motor switched off at runtime must stop acting (a left-over velocity
		// drive braked the axis instead of letting it turn freely). The per-axis drives only exist outside SLERP.
		d6->setAngularDriveConfig(PxD6AngularDriveConfig::eSWING_TWIST);
		for (int d = PxD6Drive::eX; d <= PxD6Drive::eSWING2; d++) {
			d6->setDrive((PxD6Drive::Enum)d, PxD6JointDrive());
		}
	}
	// Motor velocities, in PhysX's drive convention; a motor's axis isn't driven at the target's velocity.
	motor_lin_vel = Vector3();
	motor_ang_vel = Vector3();
	motor_lin_axes = 0;
	motor_ang_axes = 0;

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
			if (d6) {
				d6->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
				d6->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
				d6->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
				d6->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
				d6->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
				d6->setMotion(PxD6Axis::eTWIST, hinge_use_limit ? (hinge_upper > hinge_lower ? PxD6Motion::eLIMITED : PxD6Motion::eLOCKED) : PxD6Motion::eFREE);
				if (hinge_use_limit && hinge_upper > hinge_lower) {
					const double twist_max = Math::TAU - 0.001;
					d6->setTwistLimit(PxJointAngularLimitPair((PxReal)CLAMP(-hinge_upper, -twist_max, twist_max), (PxReal)CLAMP(-hinge_lower, -twist_max, twist_max)));
				}
				if (hinge_motor) {
					// Godot's limit is an impulse per step; the D6's drive limits are forces (see below).
					const GodotPhysXSpace3D *space = body_a ? body_a->get_space() : (body_b ? body_b->get_space() : nullptr);
					const real_t step = (space && space->get_last_step() > 0.0) ? space->get_last_step() : 1.0 / Engine::get_singleton()->get_physics_ticks_per_second();
					d6->setDrive(PxD6Drive::eTWIST, PxD6JointDrive(0.0f, 1.0e6f, hinge_motor_max_impulse > 0.0 ? _drive_limit(hinge_motor_max_impulse / step) : PX_MAX_F32));
					// Not negated as for the revolute drive: the D6's drive velocity runs the other way (see the 6DOF motors).
					motor_ang_vel.x = hinge_motor_velocity;
					motor_ang_axes = 1;
				} else if (slerp_drive) {
					// The hinge's one angle is its SLERP: the twist drive turns it toward the target's.
					d6->setDrive(PxD6Drive::eTWIST, PxD6JointDrive((PxReal)slerp_stiffness, (PxReal)slerp_damping, _drive_limit(slerp_torque_limit), drive_acceleration));
				}
				break;
			}
			PxRevoluteJoint *j = static_cast<PxRevoluteJoint *>(px_joint);
			j->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, hinge_use_limit);
			if (hinge_use_limit) {
				// Godot's hinge angle runs the other way round to PhysX's
				// (body B's counter-clockwise turn about the axis relative to
				// A): [lower, upper] is [-upper, -lower] here, as Jolt maps it.
				// Passed straight through, a ragdoll's knees bent backwards.
				j->setLimit(PxJointAngularLimitPair((PxReal)-hinge_upper, (PxReal)-hinge_lower));
			}
			j->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, hinge_motor);
			if (hinge_motor) {
				j->setDriveVelocity((PxReal)-hinge_motor_velocity); // the other way round too (see the limits)
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
			// Godot's 6DOF angles run the other way round to PhysX's D6 (as for the hinge above, and as Jolt maps
			// them): [lower, upper] is [-upper, -lower]. Passed straight through, a ragdoll's asymmetric limits
			// (knees, elbows) bent the wrong way. The pyramid swing must sit inside (-pi, pi), the twist inside
			// (-2pi, 2pi): a limit at +-180 deg was silently dropped.
			const double swing_max = Math::PI - 0.001;
			const double twist_max = Math::TAU - 0.001;
			j->setTwistLimit(PxJointAngularLimitPair((PxReal)CLAMP(-axis6[0].ang_upper, -twist_max, twist_max),
					(PxReal)CLAMP(-axis6[0].ang_lower, -twist_max, twist_max)));
			j->setPyramidSwingLimit(PxJointLimitPyramid(
					(PxReal)CLAMP(-axis6[1].ang_upper, -swing_max, swing_max), (PxReal)CLAMP(-axis6[1].ang_lower, -swing_max, swing_max),
					(PxReal)CLAMP(-axis6[2].ang_upper, -swing_max, swing_max), (PxReal)CLAMP(-axis6[2].ang_lower, -swing_max, swing_max)));

			// Linear drives: a spring (stiffness toward an equilibrium point) and a
			// velocity motor share the per-axis drive slot.
			for (int a = 0; a < 3; a++) {
				const Axis6DOF &ax = axis6[a];
				if (!ax.lin_spring && !ax.lin_motor) {
					continue;
				}
				const PxReal k = ax.lin_spring ? (PxReal)ax.lin_spring_stiffness : 0.0f;
				const PxReal d = ax.lin_spring ? (PxReal)ax.lin_spring_damping : 1.0e6f;
				const PxReal f = (ax.lin_motor && ax.lin_motor_force > 0.0) ? (PxReal)ax.lin_motor_force : _drive_limit(ax.lin_spring_force_limit);
				// A motor keeps its force units: its limit is Godot's force limit.
				j->setDrive((PxD6Drive::Enum)(PxD6Drive::eX + a), PxD6JointDrive(k, d, f, drive_acceleration && !ax.lin_motor));
				if (ax.lin_motor) {
					motor_lin_vel[a] = ax.lin_motor_target;
					motor_lin_axes |= 1 << a;
				}
				// A drive only acts on an axis that is not locked. Free any axis
				// that has a spring but no valid limit range.
				if (ax.lin_spring && !(ax.lin_limit && ax.lin_upper > ax.lin_lower)) {
					j->setMotion(lin_axes[a], PxD6Motion::eFREE);
				}
			}

			// Angular springs: axis X -> twist drive, axis Y -> swing1, axis Z ->
			// swing2, toward the equilibrium angles (see _equilibrium_target()). A SLERP drive replaces them and the
			// motors, and needs all three angles unlocked.
			const PxD6Drive::Enum ang_drives[3] = { PxD6Drive::eTWIST, PxD6Drive::eSWING1, PxD6Drive::eSWING2 };
			for (int a = 0; a < 3; a++) {
				const Axis6DOF &ax = axis6[a];
				if (!ax.ang_spring && !slerp_drive) {
					continue;
				}
				if (!slerp_drive) {
					j->setDrive(ang_drives[a], PxD6JointDrive((PxReal)ax.ang_spring_stiffness, (PxReal)ax.ang_spring_damping, _drive_limit(ax.ang_spring_torque_limit), drive_acceleration));
				}
				if (!(ax.ang_limit && ax.ang_upper > ax.ang_lower)) {
					j->setMotion(ang_axes[a], PxD6Motion::eFREE);
				}
			}

			// Angular motors: a velocity drive on the axis (no stiffness; the damping is the velocity gain, the force
			// limit the motor's torque). Like Jolt, a motor takes the axis over from a spring.
			for (int a = 0; a < 3; a++) {
				const Axis6DOF &ax = axis6[a];
				if (!ax.ang_motor || slerp_drive) {
					continue;
				}
				const PxReal f = ax.ang_motor_force > 0.0 ? (PxReal)ax.ang_motor_force : PX_MAX_F32;
				j->setDrive(ang_drives[a], PxD6JointDrive(0.0f, 1.0e6f, f));
				// Not negated like the limits: PhysX's drive velocity is actor0 relative to actor1, so this already
				// turns body B the way Jolt does (a positive target spins it clockwise about the axis).
				motor_ang_vel[a] = ax.ang_motor_target;
				motor_ang_axes |= 1 << a;
				if (!(ax.ang_limit && ax.ang_upper > ax.ang_lower)) {
					j->setMotion(ang_axes[a], PxD6Motion::eFREE);
				}
			}
		} break;

		default:
			break;
	}

	if (d6) {
		if (slerp_drive && type != JointType::JOINT_TYPE_HINGE) {
			d6->setAngularDriveConfig(PxD6AngularDriveConfig::eSLERP);
			d6->setDrive(PxD6Drive::eSLERP, PxD6JointDrive((PxReal)slerp_stiffness, (PxReal)slerp_damping, _drive_limit(slerp_torque_limit), drive_acceleration));
		}
		// Motor limits are a force / torque, as Godot Physics and Jolt read them (PhysX's default is an impulse
		// per step, which made a linear motor ~60x stronger at 60 Hz).
		d6->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
		// Always set (zero without drives): it also wakes the bodies, so a change made while they sleep acts.
		d6->setDrivePosition(to_px(_px_drive_target()), true);
		_send_drive_velocity();
	}
}

void GodotPhysXJoint3D::_send_drive_velocity() {
	// The target's velocity (body B's relative to body A, in A's joint frame) in PhysX's drive convention: the linear
	// and SLERP rows take B's velocity relative to A, the per-axis twist / swing rows A's relative to B.
	Vector3 lin = drive_lin_vel;
	Vector3 ang = (slerp_drive && type != JointType::JOINT_TYPE_HINGE) ? drive_ang_vel : -drive_ang_vel;
	if (type == JointType::JOINT_TYPE_HINGE) {
		const Basis z_to_x(to_godot(HINGE_Z_TO_X)); // the hinge's PhysX frames (see _create())
		lin = z_to_x.xform_inv(lin);
		ang = z_to_x.xform_inv(ang);
	}
	for (int a = 0; a < 3; a++) {
		if (motor_lin_axes & (1 << a)) {
			lin[a] = motor_lin_vel[a];
		}
		if (motor_ang_axes & (1 << a)) {
			ang[a] = motor_ang_vel[a];
		}
	}
	static_cast<PxD6Joint *>(px_joint)->setDriveVelocity(to_px(lin), to_px(ang), true);
}

bool GodotPhysXJoint3D::has_drives() const {
	return type == JointType::JOINT_TYPE_PIN || type == JointType::JOINT_TYPE_HINGE || type == JointType::JOINT_TYPE_CONE_TWIST || type == JointType::JOINT_TYPE_6DOF;
}

bool GodotPhysXJoint3D::_is_px_d6() const {
	return has_drives() && (type != JointType::JOINT_TYPE_HINGE || hinge_d6);
}

Transform3D GodotPhysXJoint3D::_px_drive_target() const {
	const Transform3D target = get_drive_target();
	if (type == JointType::JOINT_TYPE_HINGE) {
		// The hinge's PhysX frames are turned so its axis is X (see _create()).
		const Transform3D z_to_x(Basis(to_godot(HINGE_Z_TO_X)));
		return z_to_x.affine_inverse() * target * z_to_x;
	}
	return target;
}

Transform3D GodotPhysXJoint3D::_equilibrium_target() const {
	Transform3D target;
	if (type != JointType::JOINT_TYPE_6DOF) {
		return target;
	}
	const Vector3 axes[3] = { Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1) };
	Quaternion rot;
	for (int a = 0; a < 3; a++) {
		const Axis6DOF &ax = axis6[a];
		if (ax.lin_spring) {
			target.origin[a] = ax.lin_spring_eq;
		}
		if (ax.ang_spring) {
			rot = rot * Quaternion(axes[a], -ax.ang_spring_eq); // (the other way round, as the limits)
		}
	}
	target.basis = Basis(rot);
	return target;
}

void GodotPhysXJoint3D::rebuild() {
	if (type == JointType::JOINT_TYPE_MAX) {
		return;
	}
	_destroy();
	if (type == JointType::JOINT_TYPE_HINGE && slerp_drive) {
		hinge_d6 = true; // for good: a hinge that was driven once stays a D6
	}
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
		default: {
			// Bullet-era bias / softness / relaxation -- unsupported here, as in
			// Jolt. Only worth a warning when set away from its default: joint
			// nodes and PhysicalBone3D send the defaults every time.
			const real_t def = p_param == PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS ? 0.9 : (p_param == PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION ? 1.0 : 0.3);
			if (!Math::is_equal_approx(p_value, def)) {
				WARN_PRINT_ONCE("PhysX: this hinge joint parameter is not supported and will be ignored.");
			}
		} break;
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
	} else {
		// Bullet-era softness / restitution / damping -- unsupported here, as in Jolt.
		WARN_PRINT_ONCE("PhysX: this slider joint parameter is not supported and will be ignored.");
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
	} else {
		// Bullet-era bias / softness / relaxation -- unsupported here, as in
		// Jolt; a warning only when set away from the default (see the hinge).
		const real_t def = p_param == PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS ? 0.8 : (p_param == PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION ? 1.0 : 0.3);
		if (!Math::is_equal_approx(p_value, def)) {
			WARN_PRINT_ONCE("PhysX: this cone-twist joint parameter is not supported and will be ignored.");
		}
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
			// A new equilibrium point takes over from a drive target (the joint nodes resend the same value).
			has_drive_target = has_drive_target && ax.lin_spring_eq == p_value;
			ax.lin_spring_eq = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS:
			ax.ang_spring_stiffness = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_DAMPING:
			ax.ang_spring_damping = p_value;
			break;
		case PhysicsServer3D::G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT:
			has_drive_target = has_drive_target && ax.ang_spring_eq == p_value;
			ax.ang_spring_eq = p_value;
			break;
		default: {
			// Bullet-era softness / restitution / ERP / damping -- unsupported here, as in Jolt; a warning only when
			// set away from Godot's default (the node sends every parameter when it builds the joint).
			real_t def = 0.0;
			switch (p_param) {
				case PhysicsServer3D::G6DOF_JOINT_LINEAR_LIMIT_SOFTNESS:
					def = 0.7;
					break;
				case PhysicsServer3D::G6DOF_JOINT_LINEAR_RESTITUTION:
				case PhysicsServer3D::G6DOF_JOINT_ANGULAR_LIMIT_SOFTNESS:
				case PhysicsServer3D::G6DOF_JOINT_ANGULAR_ERP:
					def = 0.5;
					break;
				case PhysicsServer3D::G6DOF_JOINT_LINEAR_DAMPING:
				case PhysicsServer3D::G6DOF_JOINT_ANGULAR_DAMPING:
					def = 1.0;
					break;
				default:
					break;
			}
			if (!Math::is_equal_approx(p_value, def)) {
				WARN_PRINT_ONCE("PhysX: this Generic6DOF joint parameter is not supported and will be ignored.");
			}
		} break;
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

/* --- active-ragdoll drives (PhysX.joint_*) --------------------------------- */

void GodotPhysXJoint3D::set_spring_force_limit(Vector3::Axis p_axis, bool p_angular, real_t p_limit) {
	Axis6DOF &ax = axis6[p_axis];
	(p_angular ? ax.ang_spring_torque_limit : ax.lin_spring_force_limit) = p_limit;
	_apply_params();
}

real_t GodotPhysXJoint3D::get_spring_force_limit(Vector3::Axis p_axis, bool p_angular) const {
	const Axis6DOF &ax = axis6[p_axis];
	return p_angular ? ax.ang_spring_torque_limit : ax.lin_spring_force_limit;
}

void GodotPhysXJoint3D::set_slerp_drive(bool p_enabled, real_t p_stiffness, real_t p_damping, real_t p_torque_limit) {
	slerp_drive = p_enabled;
	slerp_stiffness = MAX(p_stiffness, (real_t)0.0);
	slerp_damping = MAX(p_damping, (real_t)0.0);
	slerp_torque_limit = p_torque_limit;
	if (slerp_drive && type == JointType::JOINT_TYPE_HINGE && !hinge_d6) {
		rebuild(); // as a D6
		return;
	}
	_apply_params();
}

void GodotPhysXJoint3D::set_drive_acceleration(bool p_enabled) {
	drive_acceleration = p_enabled;
	_apply_params();
}

void GodotPhysXJoint3D::set_drive_target(const Transform3D &p_target, real_t p_delta) {
	const Transform3D target = p_target.orthonormalized();
	// The target's own velocity, from the last one p_delta ago: the drives then move with it instead of trailing a
	// moving target (they damp toward this velocity).
	drive_lin_vel = Vector3();
	drive_ang_vel = Vector3();
	if (p_delta > 0.0 && has_drive_target) {
		drive_lin_vel = (target.origin - drive_target.origin) / p_delta;
		Quaternion turn = (target.basis * drive_target.basis.inverse()).get_rotation_quaternion(); // in A's frame
		if (turn.w < 0.0) {
			turn = -turn;
		}
		const real_t angle = turn.get_angle();
		if (angle > CMP_EPSILON) {
			drive_ang_vel = turn.get_axis() * (angle / p_delta);
		}
	}
	drive_target = target;
	has_drive_target = true;
	if (px_joint && _is_px_d6()) {
		static_cast<PxD6Joint *>(px_joint)->setDrivePosition(to_px(_px_drive_target()), true);
		_send_drive_velocity();
	}
}

void GodotPhysXJoint3D::set_drive_target_from_poses(const Transform3D &p_pose_a, const Transform3D &p_pose_b, real_t p_delta) {
	// A missing body's frame is already in world space.
	const Transform3D joint_a = body_a ? p_pose_a.orthonormalized() * frame_a : frame_a;
	const Transform3D joint_b = body_b ? p_pose_b.orthonormalized() * frame_b : frame_b;
	set_drive_target(joint_a.affine_inverse() * joint_b, p_delta);
}

Transform3D GodotPhysXJoint3D::get_drive_target() const {
	return has_drive_target ? drive_target : _equilibrium_target();
}

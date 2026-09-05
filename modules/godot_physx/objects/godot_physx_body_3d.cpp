/**************************************************************************/
/*  godot_physx_body_3d.cpp                                               */
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

#include "godot_physx_body_3d.h"

#include "../godot_physx_conversions.h"
#include "../godot_physx_project_settings.h"
#include "../joints/godot_physx_joint_3d.h"
#include "../shapes/godot_physx_shape_3d.h"
#include "../spaces/godot_physx_direct_state_3d.h"
#include "../spaces/godot_physx_space_3d.h"

#include <PxPhysicsAPI.h>

using namespace physx;

GodotPhysXBody3D::GodotPhysXBody3D() {}

GodotPhysXBody3D::~GodotPhysXBody3D() {
	// Copy: joint->body_removed() mutates our `joints` set.
	LocalVector<GodotPhysXJoint3D *> js;
	for (GodotPhysXJoint3D *j : joints) {
		js.push_back(j);
	}
	for (GodotPhysXJoint3D *j : js) {
		j->body_removed(this);
	}
	joints.clear();

	_destroy_actor();
	if (px_material) {
		px_material->release();
	}
	if (direct_state) {
		memdelete(direct_state);
	}
}

PxMaterial *GodotPhysXBody3D::_get_material() {
	if (!px_material && space && space->get_px_physics()) {
		px_material = space->get_px_physics()->createMaterial(
				(PxReal)friction, (PxReal)friction, (PxReal)bounce);
	}
	return px_material ? px_material : (space ? space->get_default_material() : nullptr);
}

bool GodotPhysXBody3D::_is_dynamic() const {
	return mode == PhysicsServer3D::BODY_MODE_RIGID || mode == PhysicsServer3D::BODY_MODE_RIGID_LINEAR;
}

void GodotPhysXBody3D::_destroy_actor() {
	if (!px_actor) {
		return;
	}
	if (space && space->get_px_scene()) {
		space->get_px_scene()->removeActor(*px_actor);
	}
	px_actor->release();
	px_actor = nullptr;
}

void GodotPhysXBody3D::_build_actor() {
	_destroy_actor();

	if (!space) {
		return;
	}
	PxPhysics *physics = space->get_px_physics();
	PxScene *scene = space->get_px_scene();
	PxMaterial *material = _get_material();
	ERR_FAIL_NULL(physics);
	ERR_FAIL_NULL(scene);
	ERR_FAIL_NULL(material);

	const PxTransform pose = to_px(body_transform);

	if (_is_dynamic()) {
		PxRigidDynamic *dyn = physics->createRigidDynamic(pose);
		ERR_FAIL_NULL(dyn);
		px_actor = dyn;
		if (mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
			dyn->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		}
		dyn->setLinearVelocity(to_px(linear_velocity));
		dyn->setAngularVelocity(to_px(angular_velocity));
	} else if (mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
		PxRigidDynamic *dyn = physics->createRigidDynamic(pose);
		ERR_FAIL_NULL(dyn);
		dyn->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		px_actor = dyn;
	} else { // BODY_MODE_STATIC
		px_actor = physics->createRigidStatic(pose);
		ERR_FAIL_NULL(px_actor);
	}

	px_actor->userData = this;

	const bool non_kinematic_dynamic = _is_dynamic() && mode != PhysicsServer3D::BODY_MODE_KINEMATIC;

	for (uint32_t shape_idx = 0; shape_idx < shapes.size(); shape_idx++) {
		const ShapeRef &sr = shapes[shape_idx];
		if (sr.disabled || !sr.shape || !sr.shape->is_valid()) {
			continue;
		}
		if (sr.shape->is_trimesh() && non_kinematic_dynamic) {
			ERR_PRINT_ONCE("PhysX: concave (trimesh) shapes are only supported on static and kinematic bodies; shape skipped.");
			continue;
		}
		const GodotPhysXShapeGeometry &g = sr.shape->get_geometry();
		PxShape *px_shape = physics->createShape(g.geometry(), *material, true);
		if (!px_shape) {
			ERR_PRINT_ONCE(vformat("PhysX: createShape failed for geometry type %d.", (int)g.geometry().getType()));
			continue;
		}
		px_shape->setLocalPose(to_px(sr.xform) * g.local_pose);
		// Store the Godot shape index so queries can report body_shape.
		px_shape->userData = reinterpret_cast<void *>(static_cast<uintptr_t>(shape_idx));
		px_actor->attachShape(*px_shape);
		px_shape->release();
	}

	_apply_filter_data();

	if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
		if (mode != PhysicsServer3D::BODY_MODE_KINEMATIC) {
			// setMassAndUpdateInertia takes an absolute mass, matching Godot's
			// RigidBody3D.mass semantics -- updateMassAndInertia's argument is a
			// *density*, which silently gave the wrong mass for any shape whose
			// volume isn't ~1 m^3 (it only looked right for unit-sized shapes,
			// where mass and density are numerically the same).
			PxRigidBodyExt::setMassAndUpdateInertia(*dyn, mass > 0.0 ? (PxReal)mass : 1.0f);
			dyn->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, gravity_scale == 0.0);
			dyn->setSleepThreshold((PxReal)space->get_sleep_energy_threshold());
			dyn->setWakeCounter((PxReal)space->get_time_before_sleep());
			if (!can_sleep || !GodotPhysXProjectSettings::allow_sleep) {
				// Keep it awake by making the energy threshold unreachable.
				dyn->setSleepThreshold(0.0f);
			}
		}
		if (ccd) {
			dyn->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
		}
	}

	_apply_damping();
	_apply_axis_lock();
	_apply_solver_iterations();

	scene->addActor(*px_actor);

	// The actor pointer changed; any joints referencing this body must be
	// recreated against the new actor.
	for (GodotPhysXJoint3D *j : joints) {
		j->rebuild();
	}
}

void GodotPhysXBody3D::set_space(GodotPhysXSpace3D *p_space) {
	if (space == p_space) {
		return;
	}
	_destroy_actor();
	if (space) {
		space->body_removed_from_areas(this);
		space->unregister_body(this);
	}
	space = p_space;
	if (space) {
		space->register_body(this);
		if (reports_contacts()) {
			space->set_body_contact_reporting(this, true);
		}
		_build_actor();
	}
}

void GodotPhysXBody3D::set_mode(PhysicsServer3D::BodyMode p_mode) {
	if (mode == p_mode) {
		return;
	}
	mode = p_mode;
	if (space) {
		_build_actor();
	}
}

void GodotPhysXBody3D::add_shape(GodotPhysXShape3D *p_shape, const Transform3D &p_xform, bool p_disabled) {
	ShapeRef sr;
	sr.shape = p_shape;
	sr.xform = p_xform;
	sr.disabled = p_disabled;
	shapes.push_back(sr);
	if (space) {
		_build_actor();
	}
}

void GodotPhysXBody3D::set_shape(int p_idx, GodotPhysXShape3D *p_shape) {
	ERR_FAIL_INDEX(p_idx, (int)shapes.size());
	shapes[p_idx].shape = p_shape;
	if (space) {
		_build_actor();
	}
}

void GodotPhysXBody3D::set_shape_transform(int p_idx, const Transform3D &p_xform) {
	ERR_FAIL_INDEX(p_idx, (int)shapes.size());
	shapes[p_idx].xform = p_xform;
	if (space) {
		_build_actor();
	}
}

void GodotPhysXBody3D::set_shape_disabled(int p_idx, bool p_disabled) {
	ERR_FAIL_INDEX(p_idx, (int)shapes.size());
	shapes[p_idx].disabled = p_disabled;
	if (space) {
		_build_actor();
	}
}

void GodotPhysXBody3D::remove_shape(int p_idx) {
	ERR_FAIL_INDEX(p_idx, (int)shapes.size());
	shapes.remove_at(p_idx);
	if (space) {
		_build_actor();
	}
}

void GodotPhysXBody3D::clear_shapes() {
	shapes.clear();
	if (space) {
		_build_actor();
	}
}

const GodotPhysXBody3D::ShapeRef *GodotPhysXBody3D::get_shape_ref(int p_idx) const {
	ERR_FAIL_INDEX_V(p_idx, (int)shapes.size(), nullptr);
	return &shapes[p_idx];
}

void GodotPhysXBody3D::shape_changed(GodotPhysXShape3D *p_shape) {
	if (!space) {
		return;
	}
	for (const ShapeRef &sr : shapes) {
		if (sr.shape == p_shape) {
			_build_actor();
			return;
		}
	}
}

void GodotPhysXBody3D::set_param(PhysicsServer3D::BodyParameter p_param, const Variant &p_value) {
	switch (p_param) {
		case PhysicsServer3D::BODY_PARAM_MASS:
			mass = p_value;
			break;
		case PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE:
			gravity_scale = p_value;
			break;
		case PhysicsServer3D::BODY_PARAM_BOUNCE:
			bounce = p_value;
			break;
		case PhysicsServer3D::BODY_PARAM_FRICTION:
			friction = p_value;
			break;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP:
			linear_damp = p_value;
			break;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP:
			angular_damp = p_value;
			break;
		default:
			// Damp modes, inertia and center-of-mass overrides not handled yet.
			break;
	}

	if (px_material && (p_param == PhysicsServer3D::BODY_PARAM_FRICTION || p_param == PhysicsServer3D::BODY_PARAM_BOUNCE)) {
		px_material->setStaticFriction((PxReal)friction);
		px_material->setDynamicFriction((PxReal)friction);
		px_material->setRestitution((PxReal)CLAMP(bounce, 0.0, 1.0));
	}

	if (space && px_actor) {
		if (p_param == PhysicsServer3D::BODY_PARAM_LINEAR_DAMP || p_param == PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP) {
			_apply_damping();
		}
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			if (p_param == PhysicsServer3D::BODY_PARAM_MASS && mode != PhysicsServer3D::BODY_MODE_KINEMATIC) {
				PxRigidBodyExt::setMassAndUpdateInertia(*dyn, mass > 0.0 ? (PxReal)mass : 1.0f);
			}
			if (p_param == PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE) {
				dyn->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, gravity_scale == 0.0);
			}
		}
	}
}

Variant GodotPhysXBody3D::get_param(PhysicsServer3D::BodyParameter p_param) const {
	switch (p_param) {
		case PhysicsServer3D::BODY_PARAM_MASS:
			return mass;
		case PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE:
			return gravity_scale;
		case PhysicsServer3D::BODY_PARAM_BOUNCE:
			return bounce;
		case PhysicsServer3D::BODY_PARAM_FRICTION:
			return friction;
		case PhysicsServer3D::BODY_PARAM_LINEAR_DAMP:
			return linear_damp;
		case PhysicsServer3D::BODY_PARAM_ANGULAR_DAMP:
			return angular_damp;
		default:
			return 0.0;
	}
}

void GodotPhysXBody3D::set_state(PhysicsServer3D::BodyState p_state, const Variant &p_value) {
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM: {
			body_transform = p_value;
			if (px_actor) {
				const PxTransform pose = to_px(body_transform);
				if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
					if (mode == PhysicsServer3D::BODY_MODE_KINEMATIC) {
						dyn->setKinematicTarget(pose);
					} else {
						dyn->setGlobalPose(pose);
					}
				} else {
					px_actor->setGlobalPose(pose);
				}
			}
		} break;
		case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY: {
			set_linear_velocity(p_value);
		} break;
		case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY: {
			set_angular_velocity(p_value);
		} break;
		case PhysicsServer3D::BODY_STATE_SLEEPING: {
			set_sleep_state(p_value);
		} break;
		case PhysicsServer3D::BODY_STATE_CAN_SLEEP: {
			can_sleep = p_value;
			if (space && px_actor) {
				if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
					dyn->setSleepThreshold(can_sleep ? (PxReal)space->get_sleep_energy_threshold() : 0.0f);
					if (!can_sleep) {
						dyn->wakeUp();
					}
				}
			}
		} break;
	}
}

Variant GodotPhysXBody3D::get_state(PhysicsServer3D::BodyState p_state) const {
	switch (p_state) {
		case PhysicsServer3D::BODY_STATE_TRANSFORM:
			return body_transform;
		case PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY:
			return get_linear_velocity();
		case PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY:
			return get_angular_velocity();
		case PhysicsServer3D::BODY_STATE_SLEEPING:
			return is_sleeping();
		case PhysicsServer3D::BODY_STATE_CAN_SLEEP:
			return can_sleep;
	}
	return Variant();
}

void GodotPhysXBody3D::set_ccd(bool p_enable) {
	ccd = p_enable;
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			dyn->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, p_enable);
		}
	}
}

void GodotPhysXBody3D::_apply_filter_data() {
	if (!px_actor) {
		return;
	}
	// word0/word1: Godot collision layer/mask. word2 bit 0: this body wants
	// contact reports, read by the scene filter shader.
	PxFilterData fd;
	fd.word0 = collision_layer;
	fd.word1 = collision_mask;
	fd.word2 = reports_contacts() ? 1u : 0u;
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

void GodotPhysXBody3D::_apply_damping() {
	if (!px_actor) {
		return;
	}
	if (PxRigidBody *rb = px_actor->is<PxRigidBody>()) {
		rb->setLinearDamping((PxReal)MAX(linear_damp, 0.0));
		rb->setAngularDamping((PxReal)MAX(angular_damp, 0.0));
	}
}

void GodotPhysXBody3D::_apply_axis_lock() {
	if (!px_actor) {
		return;
	}
	PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>();
	if (!dyn) {
		return;
	}
	// PxRigidDynamicLockFlag bits match PhysicsServer3D::BodyAxis 1:1.
	uint32_t lock = axis_lock;
	if (mode == PhysicsServer3D::BODY_MODE_RIGID_LINEAR) {
		lock |= PhysicsServer3D::BODY_AXIS_ANGULAR_X |
				PhysicsServer3D::BODY_AXIS_ANGULAR_Y |
				PhysicsServer3D::BODY_AXIS_ANGULAR_Z;
	}
	dyn->setRigidDynamicLockFlags(PxRigidDynamicLockFlags((PxU8)(lock & 0x3Fu)));
}

void GodotPhysXBody3D::set_axis_lock(PhysicsServer3D::BodyAxis p_axis, bool p_lock) {
	if (p_lock) {
		axis_lock |= p_axis;
	} else {
		axis_lock &= ~(uint32_t)p_axis;
	}
	_apply_axis_lock();
}

void GodotPhysXBody3D::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;
	_apply_filter_data();
}

void GodotPhysXBody3D::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
	_apply_filter_data();
}

void GodotPhysXBody3D::set_max_contacts_reported(int p_amount) {
	const bool was_reporting = reports_contacts();
	max_contacts_reported = MAX(p_amount, 0);
	contacts.reserve(max_contacts_reported);
	if (was_reporting != reports_contacts() && space) {
		space->set_body_contact_reporting(this, reports_contacts());
		// The filter shader keys contact notifications off word2, and existing
		// contact pairs are only re-filtered on touch changes -- rebuild so the
		// new setting takes effect immediately.
		_build_actor();
	}
}

void GodotPhysXBody3D::add_contact(const Contact &p_contact) {
	if ((int)contacts.size() >= max_contacts_reported) {
		return;
	}
	contacts.push_back(p_contact);
}

Vector3 GodotPhysXBody3D::get_linear_velocity() const {
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			return to_godot(dyn->getLinearVelocity());
		}
	}
	return linear_velocity;
}

Vector3 GodotPhysXBody3D::get_angular_velocity() const {
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			return to_godot(dyn->getAngularVelocity());
		}
	}
	return angular_velocity;
}

void GodotPhysXBody3D::set_linear_velocity(const Vector3 &p_v) {
	linear_velocity = p_v;
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			if (mode != PhysicsServer3D::BODY_MODE_KINEMATIC) {
				dyn->setLinearVelocity(to_px(p_v));
			}
		}
	}
}

void GodotPhysXBody3D::set_angular_velocity(const Vector3 &p_v) {
	angular_velocity = p_v;
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			if (mode != PhysicsServer3D::BODY_MODE_KINEMATIC) {
				dyn->setAngularVelocity(to_px(p_v));
			}
		}
	}
}

bool GodotPhysXBody3D::is_sleeping() const {
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			return dyn->isSleeping();
		}
	}
	return sleeping;
}

void GodotPhysXBody3D::set_sleep_state(bool p_sleep) {
	sleeping = p_sleep;
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			if (mode != PhysicsServer3D::BODY_MODE_KINEMATIC && dyn->getScene()) {
				if (p_sleep) {
					dyn->putToSleep();
				} else {
					dyn->wakeUp();
				}
			}
		}
	}
}

void GodotPhysXBody3D::apply_impulse(const Vector3 &p_impulse, const Vector3 &p_position) {
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			PxRigidBodyExt::addForceAtLocalPos(*dyn, to_px(p_impulse), to_px(p_position), PxForceMode::eIMPULSE);
		}
	}
}

void GodotPhysXBody3D::apply_central_impulse(const Vector3 &p_impulse) {
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			dyn->addForce(to_px(p_impulse), PxForceMode::eIMPULSE);
		}
	}
}

void GodotPhysXBody3D::apply_torque_impulse(const Vector3 &p_impulse) {
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			dyn->addTorque(to_px(p_impulse), PxForceMode::eIMPULSE);
		}
	}
}

void GodotPhysXBody3D::apply_central_force(const Vector3 &p_force) {
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			dyn->addForce(to_px(p_force), PxForceMode::eFORCE);
		}
	}
}

void GodotPhysXBody3D::_apply_solver_iterations() {
	if (!px_actor) {
		return;
	}
	PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>();
	if (!dyn) {
		return;
	}
	if (joints.is_empty()) {
		dyn->setSolverIterationCounts(SOLVER_ITERS_DEFAULT_POS, SOLVER_ITERS_DEFAULT_VEL);
	} else {
		dyn->setSolverIterationCounts(SOLVER_ITERS_JOINTED_POS, SOLVER_ITERS_JOINTED_VEL);
	}
}

void GodotPhysXBody3D::apply_torque(const Vector3 &p_torque) {
	if (px_actor) {
		if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
			dyn->addTorque(to_px(p_torque), PxForceMode::eFORCE);
		}
	}
}

void GodotPhysXBody3D::pull_transform_from_px() {
	if (!px_actor) {
		return;
	}
	body_transform = to_godot(px_actor->getGlobalPose());
	if (PxRigidDynamic *dyn = px_actor->is<PxRigidDynamic>()) {
		linear_velocity = to_godot(dyn->getLinearVelocity());
		angular_velocity = to_godot(dyn->getAngularVelocity());
		sleeping = dyn->isSleeping();
	}
}

void GodotPhysXBody3D::call_queries() {
	if (body_state_callback.is_valid()) {
		GodotPhysXDirectBodyState3D *state = get_direct_state();
		Variant v = state;
		body_state_callback.call(v);
	}
}

GodotPhysXDirectBodyState3D *GodotPhysXBody3D::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(GodotPhysXDirectBodyState3D);
		direct_state->body = this;
	}
	return direct_state;
}

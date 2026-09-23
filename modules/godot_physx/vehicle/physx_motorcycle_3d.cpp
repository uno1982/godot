/**************************************************************************/
/*  physx_motorcycle_3d.cpp                                               */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
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
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "physx_motorcycle_3d.h"

#include "../godot_physx_conversions.h"
#include "../godot_physx_project_settings.h"
#include "../godot_physx_server_3d.h"
#include "../spaces/godot_physx_space_3d.h"
#include "godot_physx_vehicle2w.h"
#include "physx_vehicle_wheel_3d.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/resources/3d/box_shape_3d.h"
#include "scene/resources/3d/world_3d.h"

struct PhysXMotorcycle3D::Impl {
	Vehicle2W vehicle;
	PxVehiclePhysXSimulationContext simulationContext;
	PxScene *scene = nullptr;
	PxReal sleep_threshold = 0.0f;
	PxReal time_before_sleep = 0.0f;
	bool built = false;
	// wheel_order[Vehicle2W::WHEEL_FRONT/REAR] = index into the parent's own
	// `wheels` vector (child-registration order) for that canonical slot.
	uint32_t wheel_order[2] = {};
};

PhysXMotorcycle3D::PhysXMotorcycle3D() {
	impl = memnew(Impl);
}

PhysXMotorcycle3D::~PhysXMotorcycle3D() {
	_destroy();
	memdelete(impl);
}

bool PhysXMotorcycle3D::_build() {
	if (impl->built) {
		return true;
	}
	// Same editor-vs-play guard as PhysXVehicle3D -- see that class's own
	// comment for why.
	if (Engine::get_singleton()->is_editor_hint()) {
		return false;
	}
	if (!is_inside_world() || get_world_3d().is_null()) {
		return false;
	}
	if (wheels.size() != 2) {
		return false;
	}

	int front_idx = -1, rear_idx = -1;
	for (uint32_t i = 0; i < wheels.size(); i++) {
		if (wheels[i]->is_used_as_steering()) {
			if (front_idx != -1) {
				return false; // more than one steering wheel -- get_configuration_warnings() surfaces this
			}
			front_idx = (int)i;
		} else {
			if (rear_idx != -1) {
				return false;
			}
			rear_idx = (int)i;
		}
	}
	if (front_idx == -1 || rear_idx == -1) {
		return false;
	}
	impl->wheel_order[Vehicle2W::WHEEL_FRONT] = (uint32_t)front_idx;
	impl->wheel_order[Vehicle2W::WHEEL_REAR] = (uint32_t)rear_idx;

	CollisionShape3D *chassis_shape_node = nullptr;
	for (int i = 0; i < get_child_count(); i++) {
		chassis_shape_node = Object::cast_to<CollisionShape3D>(get_child(i));
		if (chassis_shape_node) {
			break;
		}
	}
	if (!chassis_shape_node || chassis_shape_node->get_shape().is_null()) {
		return false;
	}
	Ref<BoxShape3D> box_shape = chassis_shape_node->get_shape();
	if (box_shape.is_null()) {
		ERR_PRINT("PhysXMotorcycle3D: chassis CollisionShape3D must use a BoxShape3D.");
		return false;
	}

	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server) {
		return false;
	}
	GodotPhysXSpace3D *space = server->get_space(get_world_3d()->get_space());
	if (!space) {
		return false;
	}
	PxPhysics *physics = space->get_px_physics();
	PxScene *scene = space->get_px_scene();
	if (!physics || !scene) {
		return false;
	}

	Vehicle2WConfig cfg;
	cfg.mass = mass;
	cfg.moment_of_inertia = moment_of_inertia;
	cfg.chassis_half_extents = box_shape->get_size() * 0.5;
	cfg.chassis_box_center_local = chassis_shape_node->get_position();
	cfg.chassis_com_local = center_of_mass_mode == CENTER_OF_MASS_MODE_CUSTOM ? center_of_mass : chassis_shape_node->get_position();
	cfg.max_engine_torque = max_engine_torque;
	cfg.max_brake_torque = max_brake_torque;
	cfg.max_steer_angle = max_steer_angle;
	cfg.collision_layer = collision_layer;
	cfg.collision_mask = collision_mask;

	PhysXVehicleWheel3D *front_wheel = wheels[front_idx];
	PhysXVehicleWheel3D *rear_wheel = wheels[rear_idx];
	Vehicle2WWheelConfig *wheel_cfgs[2] = { &cfg.front_wheel, &cfg.rear_wheel };
	PhysXVehicleWheel3D *wheel_nodes[2] = { front_wheel, rear_wheel };
	for (int i = 0; i < 2; i++) {
		Vehicle2WWheelConfig &wc = *wheel_cfgs[i];
		PhysXVehicleWheel3D *w = wheel_nodes[i];
		wc.position = w->get_position();
		wc.basis = w->get_transform().basis;
		wc.radius = w->get_radius();
		wc.half_width = w->get_half_width();
		wc.wheel_mass = w->get_wheel_mass();
		wc.wheel_moment_of_inertia = w->get_wheel_moment_of_inertia();
		wc.damping_rate = w->get_damping_rate();
		wc.suspension_travel = w->get_suspension_travel();
		wc.suspension_stiffness = w->get_suspension_stiffness();
		wc.suspension_damping = w->get_suspension_damping();
		wc.tire_lateral_stiffness = w->get_tire_lateral_stiffness();
		wc.tire_longitudinal_stiffness = w->get_tire_longitudinal_stiffness();
		wc.tire_camber_stiffness = w->get_tire_camber_stiffness();
		wc.tire_friction = w->get_tire_friction();
		wc.tire_rest_grip = w->get_tire_rest_grip();
		wc.tire_slide_grip = w->get_tire_slide_grip();
	}

	if (!configure_vehicle2w(impl->vehicle, cfg, *physics, *scene, impl->simulationContext)) {
		return false;
	}

	Vehicle2W &v = impl->vehicle;
	v.physxActor.rigidBody->setGlobalPose(to_px(get_global_transform()));
	impl->sleep_threshold = (PxReal)space->get_sleep_energy_threshold();
	impl->time_before_sleep = (PxReal)space->get_time_before_sleep();
	PxRigidDynamic *dynamic_body = v.physxActor.rigidBody->is<PxRigidDynamic>();
	ERR_FAIL_NULL_V(dynamic_body, false);
	dynamic_body->setSleepThreshold(can_sleep && GodotPhysXProjectSettings::allow_sleep ? impl->sleep_threshold : 0.0f);
	dynamic_body->setWakeCounter(impl->time_before_sleep);
	scene->addActor(*v.physxActor.rigidBody);
	v.physxActor.rigidBody->setName("PhysXMotorcycle3D");

	impl->scene = scene;
	impl->built = true;
	return true;
}

void PhysXMotorcycle3D::_destroy() {
	if (impl->built) {
		if (impl->scene) {
			impl->scene->removeActor(*impl->vehicle.physxActor.rigidBody);
		}
		impl->vehicle.destroy();
		impl->built = false;
		impl->scene = nullptr;
	}
}

void PhysXMotorcycle3D::_rebuild_if_live() {
	if (!is_inside_world()) {
		return;
	}
	_destroy();
	set_physics_process_internal(_build());
}

void PhysXMotorcycle3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			set_physics_process_internal(_build());
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			set_physics_process_internal(false);
			_destroy();
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (!impl->built) {
				break;
			}
			Vehicle2W &v = impl->vehicle;
			v.commandState.throttle = (PxReal)throttle;
			v.commandState.brakes[0] = (PxReal)brake;
			v.commandState.nbBrakes = 1;
			v.commandState.steer = (PxReal)steer;
			v.transmissionCommandState.gear = reverse ? PxVehicleDirectDriveTransmissionCommandState::eREVERSE : PxVehicleDirectDriveTransmissionCommandState::eFORWARD;
			v.step((PxReal)get_physics_process_delta_time(), impl->simulationContext);
			// Same CoM-vs-actor-origin frame notes as PhysXVehicle3D's own
			// identical code -- see that class's comment for the SDK source
			// citations that confirmed this.
			set_global_transform(to_godot(v.physxActor.rigidBody->getGlobalPose()));
			const PxTransform cmass_local_pose = v.physxActor.rigidBody->getCMassLocalPose();
			for (uint32_t i = 0; i < 2; i++) {
				wheels[impl->wheel_order[i]]->set_transform(to_godot(cmass_local_pose * v.wheelLocalPoses[i].localPose));
			}
		} break;
	}
}

void PhysXMotorcycle3D::apply_torque_impulse(const Vector3 &p_impulse) {
	if (!impl->built) {
		return;
	}
	PxRigidDynamic *dynamic_body = impl->vehicle.physxActor.rigidBody->is<PxRigidDynamic>();
	ERR_FAIL_NULL(dynamic_body);
	dynamic_body->addTorque(to_px(p_impulse), PxForceMode::eIMPULSE);
}

void PhysXMotorcycle3D::apply_force_at_local_position(const Vector3 &p_force, const Vector3 &p_local_position) {
	if (!impl->built) {
		return;
	}
	PxRigidDynamic *dynamic_body = impl->vehicle.physxActor.rigidBody->is<PxRigidDynamic>();
	ERR_FAIL_NULL(dynamic_body);
	// addForceAtLocalPos: force is WORLD-space, position is LOCAL-space (the
	// body's own frame) -- see GodotPhysXBody3D::apply_impulse()'s own doc
	// comment for the real bug history of getting a force/position frame
	// mismatch wrong with the sibling PxRigidBodyExt overloads. eFORCE (not
	// eIMPULSE) since this is meant to be applied continuously across ticks
	// (e.g. a small ongoing rear-wheel slide force), not a one-shot kick.
	PxRigidBodyExt::addForceAtLocalPos(*dynamic_body, to_px(p_force), to_px(p_local_position), PxForceMode::eFORCE);
}

void PhysXMotorcycle3D::set_angular_velocity(const Vector3 &p_angular_velocity) {
	if (!impl->built) {
		return;
	}
	PxRigidDynamic *dynamic_body = impl->vehicle.physxActor.rigidBody->is<PxRigidDynamic>();
	ERR_FAIL_NULL(dynamic_body);
	dynamic_body->setAngularVelocity(to_px(p_angular_velocity));
}

Vector3 PhysXMotorcycle3D::get_angular_velocity() const {
	if (!impl->built) {
		return Vector3();
	}
	PxRigidDynamic *dynamic_body = impl->vehicle.physxActor.rigidBody->is<PxRigidDynamic>();
	ERR_FAIL_NULL_V(dynamic_body, Vector3());
	return to_godot(dynamic_body->getAngularVelocity());
}

Vector3 PhysXMotorcycle3D::get_up() const {
	if (!impl->built) {
		return Vector3(0, 1, 0);
	}
	const PxTransform actor_pose = impl->vehicle.physxActor.rigidBody->getGlobalPose();
	return to_godot(actor_pose.q.getBasisVector1());
}

Vector3 PhysXMotorcycle3D::get_forward() const {
	if (!impl->built) {
		return -get_global_transform().basis.get_column(2);
	}
	const PxTransform actor_pose = impl->vehicle.physxActor.rigidBody->getGlobalPose();
	return -to_godot(actor_pose.q.getBasisVector2());
}

real_t PhysXMotorcycle3D::get_roll_angle() const {
	if (!impl->built) {
		return 0.0;
	}
	const PxTransform actor_pose = impl->vehicle.physxActor.rigidBody->getGlobalPose();
	const PxVec3 right = actor_pose.q.getBasisVector0();
	const PxVec3 up = actor_pose.q.getBasisVector1();
	return (real_t)Math::atan2((double)right.y, (double)up.y);
}

Vector3 PhysXMotorcycle3D::get_linear_velocity() const {
	if (!impl->built) {
		return Vector3();
	}
	return to_godot(impl->vehicle.rigidBodyState.linearVelocity);
}

real_t PhysXMotorcycle3D::get_forward_speed() const {
	if (!impl->built) {
		return 0.0;
	}
	// frame.getLngAxis() is a fixed LOCAL-frame constant (e.g. (0,0,-1)),
	// not a world-space direction -- rotate it into world space by the
	// actor's current orientation before dotting against the world-space
	// velocity, or this is only correct when yaw matches spawn orientation
	// (found via a real bug report: correct at first, sign-flips as the
	// vehicle yaws further away from its start heading -- the actual cause
	// of "leans in for the first half of a held turn, leans out for the
	// second half," since this feeds target_lean's banking-angle formula).
	const PxTransform actor_pose = impl->vehicle.physxActor.rigidBody->getGlobalPose();
	const PxVec3 fwd = actor_pose.q.rotate(impl->vehicle.frame.getLngAxis());
	return (real_t)impl->vehicle.rigidBodyState.linearVelocity.dot(fwd);
}

real_t PhysXMotorcycle3D::get_wheel_jounce(int p_wheel) const {
	if (!impl->built) {
		return 0.0;
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), 0.0);
	for (uint32_t i = 0; i < 2; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			return (real_t)impl->vehicle.suspensionStates[i].jounce;
		}
	}
	return 0.0;
}

real_t PhysXMotorcycle3D::get_wheel_separation(int p_wheel) const {
	if (!impl->built) {
		return 0.0;
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), 0.0);
	for (uint32_t i = 0; i < 2; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			return (real_t)impl->vehicle.suspensionStates[i].separation;
		}
	}
	return 0.0;
}

real_t PhysXMotorcycle3D::get_wheel_lateral_force(int p_wheel) const {
	if (!impl->built) {
		return 0.0;
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), 0.0);
	for (uint32_t i = 0; i < 2; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			const Vector3 lateral_force_world = to_godot(impl->vehicle.tireForces[i].forces[PxVehicleTireDirectionModes::eLATERAL]);
			const PxTransform actor_pose = impl->vehicle.physxActor.rigidBody->getGlobalPose();
			const Vector3 right_now = to_godot(actor_pose.q.getBasisVector0());
			return lateral_force_world.dot(right_now);
		}
	}
	return 0.0;
}

real_t PhysXMotorcycle3D::get_wheel_lateral_speed(int p_wheel) const {
	if (!impl->built) {
		return 0.0;
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), 0.0);
	for (uint32_t i = 0; i < 2; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			return (real_t)impl->vehicle.tireSpeedStates[i].speedStates[PxVehicleTireDirectionModes::eLATERAL];
		}
	}
	return 0.0;
}

Vector3 PhysXMotorcycle3D::get_wheel_lateral_direction(int p_wheel) const {
	if (!impl->built) {
		return Vector3();
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), Vector3());
	for (uint32_t i = 0; i < 2; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			return to_godot(impl->vehicle.tireDirectionStates[i].directions[PxVehicleTireDirectionModes::eLATERAL]);
		}
	}
	return Vector3();
}

Vector3 PhysXMotorcycle3D::get_wheel_lateral_force_vector(int p_wheel) const {
	if (!impl->built) {
		return Vector3();
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), Vector3());
	for (uint32_t i = 0; i < 2; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			return to_godot(impl->vehicle.tireForces[i].forces[PxVehicleTireDirectionModes::eLATERAL]);
		}
	}
	return Vector3();
}

Vector3 PhysXMotorcycle3D::get_wheel_lateral_velocity(int p_wheel) const {
	if (!impl->built) {
		return Vector3();
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), Vector3());
	for (uint32_t i = 0; i < 2; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			const PxVec3 &direction = impl->vehicle.tireDirectionStates[i].directions[PxVehicleTireDirectionModes::eLATERAL];
			return to_godot(direction * impl->vehicle.tireSpeedStates[i].speedStates[PxVehicleTireDirectionModes::eLATERAL]);
		}
	}
	return Vector3();
}

real_t PhysXMotorcycle3D::get_wheel_camber_angle(int p_wheel) const {
	if (!impl->built) {
		return 0.0;
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), 0.0);
	for (uint32_t i = 0; i < 2; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			return (real_t)impl->vehicle.tireCamberAngleStates[i].camberAngle;
		}
	}
	return 0.0;
}

Vector3 PhysXMotorcycle3D::get_actor_position() const {
	if (!impl->built) {
		return Vector3();
	}
	return to_godot(impl->vehicle.physxActor.rigidBody->getGlobalPose().p);
}

PackedStringArray PhysXMotorcycle3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();
	bool has_box_shape = false;
	for (int i = 0; i < get_child_count(); i++) {
		CollisionShape3D *cs = Object::cast_to<CollisionShape3D>(get_child(i));
		if (cs && cs->get_shape().is_valid() && Object::cast_to<BoxShape3D>(cs->get_shape().ptr())) {
			has_box_shape = true;
			break;
		}
	}
	if (!has_box_shape) {
		warnings.push_back(RTR("PhysXMotorcycle3D needs a CollisionShape3D child with a BoxShape3D for its chassis."));
	}
	if (wheels.size() != 2) {
		warnings.push_back(vformat(RTR("PhysXMotorcycle3D needs exactly 2 PhysXVehicleWheel3D children (has %d)."), (int)wheels.size()));
	} else {
		int nb_steering = 0;
		for (uint32_t i = 0; i < wheels.size(); i++) {
			if (wheels[i]->is_used_as_steering()) {
				nb_steering++;
			}
		}
		if (nb_steering != 1) {
			warnings.push_back(vformat(RTR("PhysXMotorcycle3D needs exactly 1 wheel with use_as_steering enabled (has %d)."), nb_steering));
		}
	}
	return warnings;
}

void PhysXMotorcycle3D::_validate_property(PropertyInfo &p_property) const {
	if (center_of_mass_mode != CENTER_OF_MASS_MODE_CUSTOM && p_property.name == "center_of_mass") {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

#define PHYSX_MOTORCYCLE_SETTER(m_name, m_field)      \
	void PhysXMotorcycle3D::set_##m_name(real_t p_v) { \
		m_field = p_v;                                  \
		_rebuild_if_live();                             \
	}

void PhysXMotorcycle3D::set_mass(real_t p_mass) {
	mass = p_mass;
	_rebuild_if_live();
}
void PhysXMotorcycle3D::set_moment_of_inertia(const Vector3 &p_moi) {
	moment_of_inertia = p_moi;
	_rebuild_if_live();
}
void PhysXMotorcycle3D::set_center_of_mass_mode(CenterOfMassMode p_mode) {
	if (center_of_mass_mode == p_mode) {
		return;
	}
	center_of_mass_mode = p_mode;
	notify_property_list_changed();
	_rebuild_if_live();
}
void PhysXMotorcycle3D::set_center_of_mass(const Vector3 &p_center_of_mass) {
	if (center_of_mass == p_center_of_mass) {
		return;
	}
	ERR_FAIL_COND(center_of_mass_mode != CENTER_OF_MASS_MODE_CUSTOM);
	center_of_mass = p_center_of_mass;
	_rebuild_if_live();
}
void PhysXMotorcycle3D::set_can_sleep(bool p_can_sleep) {
	if (can_sleep == p_can_sleep) {
		return;
	}
	can_sleep = p_can_sleep;
	if (impl->built) {
		PxRigidDynamic *body = impl->vehicle.physxActor.rigidBody->is<PxRigidDynamic>();
		ERR_FAIL_NULL(body);
		body->setSleepThreshold(can_sleep && GodotPhysXProjectSettings::allow_sleep ? impl->sleep_threshold : 0.0f);
		body->setWakeCounter(impl->time_before_sleep);
		if (!can_sleep) {
			body->wakeUp();
		}
	}
}

bool PhysXMotorcycle3D::is_sleeping() const {
	PxRigidDynamic *body = impl->built ? impl->vehicle.physxActor.rigidBody->is<PxRigidDynamic>() : nullptr;
	return body && body->isSleeping();
}
PHYSX_MOTORCYCLE_SETTER(max_engine_torque, max_engine_torque)
PHYSX_MOTORCYCLE_SETTER(max_brake_torque, max_brake_torque)
PHYSX_MOTORCYCLE_SETTER(max_steer_angle, max_steer_angle)

#undef PHYSX_MOTORCYCLE_SETTER

void PhysXMotorcycle3D::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;
	_rebuild_if_live();
}
void PhysXMotorcycle3D::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
	_rebuild_if_live();
}

void PhysXMotorcycle3D::_bind_methods() {
	BIND_ENUM_CONSTANT(CENTER_OF_MASS_MODE_AUTO);
	BIND_ENUM_CONSTANT(CENTER_OF_MASS_MODE_CUSTOM);

	ClassDB::bind_method(D_METHOD("set_mass", "mass"), &PhysXMotorcycle3D::set_mass);
	ClassDB::bind_method(D_METHOD("get_mass"), &PhysXMotorcycle3D::get_mass);
	ClassDB::bind_method(D_METHOD("set_moment_of_inertia", "moi"), &PhysXMotorcycle3D::set_moment_of_inertia);
	ClassDB::bind_method(D_METHOD("get_moment_of_inertia"), &PhysXMotorcycle3D::get_moment_of_inertia);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "1,2000,1,or_greater"), "set_mass", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "moment_of_inertia"), "set_moment_of_inertia", "get_moment_of_inertia");
	ClassDB::bind_method(D_METHOD("set_center_of_mass_mode", "mode"), &PhysXMotorcycle3D::set_center_of_mass_mode);
	ClassDB::bind_method(D_METHOD("get_center_of_mass_mode"), &PhysXMotorcycle3D::get_center_of_mass_mode);
	ClassDB::bind_method(D_METHOD("set_center_of_mass", "center_of_mass"), &PhysXMotorcycle3D::set_center_of_mass);
	ClassDB::bind_method(D_METHOD("get_center_of_mass"), &PhysXMotorcycle3D::get_center_of_mass);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "center_of_mass_mode", PROPERTY_HINT_ENUM, "Auto,Custom"), "set_center_of_mass_mode", "get_center_of_mass_mode");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "center_of_mass", PROPERTY_HINT_RANGE, "-10,10,0.01,or_less,or_greater,suffix:m"), "set_center_of_mass", "get_center_of_mass");
	ClassDB::bind_method(D_METHOD("set_can_sleep", "able_to_sleep"), &PhysXMotorcycle3D::set_can_sleep);
	ClassDB::bind_method(D_METHOD("is_able_to_sleep"), &PhysXMotorcycle3D::is_able_to_sleep);
	ClassDB::bind_method(D_METHOD("is_sleeping"), &PhysXMotorcycle3D::is_sleeping);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "can_sleep"), "set_can_sleep", "is_able_to_sleep");

	ClassDB::bind_method(D_METHOD("set_max_engine_torque", "value"), &PhysXMotorcycle3D::set_max_engine_torque);
	ClassDB::bind_method(D_METHOD("get_max_engine_torque"), &PhysXMotorcycle3D::get_max_engine_torque);
	ClassDB::bind_method(D_METHOD("set_max_brake_torque", "value"), &PhysXMotorcycle3D::set_max_brake_torque);
	ClassDB::bind_method(D_METHOD("get_max_brake_torque"), &PhysXMotorcycle3D::get_max_brake_torque);
	ClassDB::bind_method(D_METHOD("set_max_steer_angle", "value"), &PhysXMotorcycle3D::set_max_steer_angle);
	ClassDB::bind_method(D_METHOD("get_max_steer_angle"), &PhysXMotorcycle3D::get_max_steer_angle);
	ADD_GROUP("Drivetrain", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_engine_torque", PROPERTY_HINT_RANGE, "0,2000,10,or_greater"), "set_max_engine_torque", "get_max_engine_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_brake_torque", PROPERTY_HINT_RANGE, "0,5000,10,or_greater"), "set_max_brake_torque", "get_max_brake_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_steer_angle", PROPERTY_HINT_RANGE, "0,1.0472,0.01"), "set_max_steer_angle", "get_max_steer_angle");

	ClassDB::bind_method(D_METHOD("set_throttle", "value"), &PhysXMotorcycle3D::set_throttle);
	ClassDB::bind_method(D_METHOD("get_throttle"), &PhysXMotorcycle3D::get_throttle);
	ClassDB::bind_method(D_METHOD("set_brake", "value"), &PhysXMotorcycle3D::set_brake);
	ClassDB::bind_method(D_METHOD("get_brake"), &PhysXMotorcycle3D::get_brake);
	ClassDB::bind_method(D_METHOD("set_steer", "value"), &PhysXMotorcycle3D::set_steer);
	ClassDB::bind_method(D_METHOD("get_steer"), &PhysXMotorcycle3D::get_steer);
	ADD_GROUP("Controls", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "throttle", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_throttle", "get_throttle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_brake", "get_brake");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steer", PROPERTY_HINT_RANGE, "-1,1,0.01"), "set_steer", "get_steer");
	ClassDB::bind_method(D_METHOD("set_reverse", "value"), &PhysXMotorcycle3D::set_reverse);
	ClassDB::bind_method(D_METHOD("is_reverse"), &PhysXMotorcycle3D::is_reverse);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "reverse"), "set_reverse", "is_reverse");

	ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"), &PhysXMotorcycle3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &PhysXMotorcycle3D::get_collision_layer);
	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &PhysXMotorcycle3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &PhysXMotorcycle3D::get_collision_mask);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	ClassDB::bind_method(D_METHOD("get_linear_velocity"), &PhysXMotorcycle3D::get_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_forward_speed"), &PhysXMotorcycle3D::get_forward_speed);
	ClassDB::bind_method(D_METHOD("get_wheel_jounce", "wheel"), &PhysXMotorcycle3D::get_wheel_jounce);
	ClassDB::bind_method(D_METHOD("get_wheel_separation", "wheel"), &PhysXMotorcycle3D::get_wheel_separation);
	ClassDB::bind_method(D_METHOD("get_wheel_lateral_force", "wheel"), &PhysXMotorcycle3D::get_wheel_lateral_force);
	ClassDB::bind_method(D_METHOD("get_wheel_lateral_speed", "wheel"), &PhysXMotorcycle3D::get_wheel_lateral_speed);
	ClassDB::bind_method(D_METHOD("get_wheel_lateral_direction", "wheel"), &PhysXMotorcycle3D::get_wheel_lateral_direction);
	ClassDB::bind_method(D_METHOD("get_wheel_lateral_force_vector", "wheel"), &PhysXMotorcycle3D::get_wheel_lateral_force_vector);
	ClassDB::bind_method(D_METHOD("get_wheel_lateral_velocity", "wheel"), &PhysXMotorcycle3D::get_wheel_lateral_velocity);
	ClassDB::bind_method(D_METHOD("get_wheel_camber_angle", "wheel"), &PhysXMotorcycle3D::get_wheel_camber_angle);
	ClassDB::bind_method(D_METHOD("get_actor_position"), &PhysXMotorcycle3D::get_actor_position);

	ClassDB::bind_method(D_METHOD("apply_torque_impulse", "impulse"), &PhysXMotorcycle3D::apply_torque_impulse);
	ClassDB::bind_method(D_METHOD("apply_force_at_local_position", "force", "local_position"), &PhysXMotorcycle3D::apply_force_at_local_position);
	ClassDB::bind_method(D_METHOD("set_angular_velocity", "angular_velocity"), &PhysXMotorcycle3D::set_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_angular_velocity"), &PhysXMotorcycle3D::get_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_up"), &PhysXMotorcycle3D::get_up);
	ClassDB::bind_method(D_METHOD("get_forward"), &PhysXMotorcycle3D::get_forward);
	ClassDB::bind_method(D_METHOD("get_roll_angle"), &PhysXMotorcycle3D::get_roll_angle);
}

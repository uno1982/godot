/**************************************************************************/
/*  physx_vehicle_3d.cpp                                                  */
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
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "physx_vehicle_3d.h"

#include "../godot_physx_conversions.h"
#include "../godot_physx_server_3d.h"
#include "../spaces/godot_physx_space_3d.h"
#include "godot_physx_vehicle4w.h"

#include "core/object/class_db.h"
#include "scene/resources/3d/world_3d.h"

struct PhysXVehicle3D::Impl {
	Vehicle4W vehicle;
	PxVehiclePhysXSimulationContext simulationContext;
	PxScene *scene = nullptr;
	bool built = false;
};

PhysXVehicle3D::PhysXVehicle3D() {
	impl = memnew(Impl);
}

PhysXVehicle3D::~PhysXVehicle3D() {
	_destroy();
	memdelete(impl);
}

bool PhysXVehicle3D::_build() {
	if (impl->built) {
		return true;
	}
	if (!is_inside_world() || get_world_3d().is_null()) {
		return false;
	}
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server) {
		return false;
	}
	GodotPhysXSpace3D *space = server->get_space(get_world_3d()->get_space());
	if (!space) {
		// Not running on the PhysX backend -- this node offers PxVehicle2-
		// specific capability and has no fallback for another backend.
		return false;
	}
	PxPhysics *physics = space->get_px_physics();
	PxScene *scene = space->get_px_scene();
	if (!physics || !scene) {
		return false;
	}

	Vehicle4WConfig cfg;
	cfg.mass = mass;
	cfg.moment_of_inertia = moment_of_inertia;
	cfg.half_track = half_track;
	cfg.front_axle_z = front_axle_z;
	cfg.rear_axle_z = rear_axle_z;
	cfg.wheel_radius = wheel_radius;
	cfg.wheel_half_width = wheel_half_width;
	cfg.wheel_mass = wheel_mass;
	cfg.wheel_moment_of_inertia = wheel_moment_of_inertia;
	cfg.wheel_damping_rate = wheel_damping_rate;
	cfg.suspension_travel = suspension_travel;
	cfg.suspension_stiffness = suspension_stiffness;
	cfg.suspension_damping = suspension_damping;
	cfg.tire_lateral_stiffness = tire_lateral_stiffness;
	cfg.tire_longitudinal_stiffness = tire_longitudinal_stiffness;
	cfg.tire_friction = tire_friction;
	cfg.max_engine_torque = max_engine_torque;
	cfg.max_brake_torque = max_brake_torque;
	cfg.max_steer_angle = max_steer_angle;
	cfg.ackermann_strength = ackermann_strength;

	if (!configure_vehicle4w(impl->vehicle, cfg, *physics, *scene, impl->simulationContext)) {
		return false;
	}

	Vehicle4W &v = impl->vehicle;
	v.physxActor.rigidBody->setGlobalPose(to_px(get_global_transform()));
	scene->addActor(*v.physxActor.rigidBody);
	v.physxActor.rigidBody->setName("PhysXVehicle3D");

	impl->scene = scene;
	impl->built = true;
	return true;
}

void PhysXVehicle3D::_destroy() {
	if (impl->built) {
		if (impl->scene) {
			impl->scene->removeActor(*impl->vehicle.physxActor.rigidBody);
		}
		impl->vehicle.destroy();
		impl->built = false;
		impl->scene = nullptr;
	}
}

void PhysXVehicle3D::_rebuild_if_live() {
	if (impl->built) {
		_destroy();
		_build();
	}
}

void PhysXVehicle3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			if (_build()) {
				set_physics_process_internal(true);
			}
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			set_physics_process_internal(false);
			_destroy();
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (!impl->built) {
				break;
			}
			Vehicle4W &v = impl->vehicle;
			v.commandState.throttle = (PxReal)throttle;
			v.commandState.brakes[0] = (PxReal)brake;
			v.commandState.nbBrakes = 1;
			v.commandState.steer = (PxReal)steer;
			v.step((PxReal)get_physics_process_delta_time(), impl->simulationContext);
			set_global_transform(to_godot(v.rigidBodyState.pose));
		} break;
	}
}

Vector3 PhysXVehicle3D::get_linear_velocity() const {
	if (!impl->built) {
		return Vector3();
	}
	return to_godot(impl->vehicle.rigidBodyState.linearVelocity);
}

real_t PhysXVehicle3D::get_forward_speed() const {
	if (!impl->built) {
		return 0.0;
	}
	const PxVec3 fwd = impl->vehicle.frame.getLngAxis();
	return (real_t)impl->vehicle.rigidBodyState.linearVelocity.dot(fwd);
}

#define PHYSX_VEHICLE_SETTER(m_name, m_field)      \
	void PhysXVehicle3D::set_##m_name(real_t p_v) { \
		m_field = p_v;                              \
		_rebuild_if_live();                         \
	}

void PhysXVehicle3D::set_mass(real_t p_mass) {
	mass = p_mass;
	_rebuild_if_live();
}
void PhysXVehicle3D::set_moment_of_inertia(const Vector3 &p_moi) {
	moment_of_inertia = p_moi;
	_rebuild_if_live();
}
PHYSX_VEHICLE_SETTER(half_track, half_track)
PHYSX_VEHICLE_SETTER(front_axle_z, front_axle_z)
PHYSX_VEHICLE_SETTER(rear_axle_z, rear_axle_z)
PHYSX_VEHICLE_SETTER(wheel_radius, wheel_radius)
PHYSX_VEHICLE_SETTER(wheel_half_width, wheel_half_width)
PHYSX_VEHICLE_SETTER(wheel_mass, wheel_mass)
PHYSX_VEHICLE_SETTER(wheel_moment_of_inertia, wheel_moment_of_inertia)
PHYSX_VEHICLE_SETTER(wheel_damping_rate, wheel_damping_rate)
PHYSX_VEHICLE_SETTER(suspension_travel, suspension_travel)
PHYSX_VEHICLE_SETTER(suspension_stiffness, suspension_stiffness)
PHYSX_VEHICLE_SETTER(suspension_damping, suspension_damping)
PHYSX_VEHICLE_SETTER(tire_lateral_stiffness, tire_lateral_stiffness)
PHYSX_VEHICLE_SETTER(tire_longitudinal_stiffness, tire_longitudinal_stiffness)
PHYSX_VEHICLE_SETTER(tire_friction, tire_friction)
PHYSX_VEHICLE_SETTER(max_engine_torque, max_engine_torque)
PHYSX_VEHICLE_SETTER(max_brake_torque, max_brake_torque)
PHYSX_VEHICLE_SETTER(max_steer_angle, max_steer_angle)
PHYSX_VEHICLE_SETTER(ackermann_strength, ackermann_strength)

#undef PHYSX_VEHICLE_SETTER

void PhysXVehicle3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_mass", "mass"), &PhysXVehicle3D::set_mass);
	ClassDB::bind_method(D_METHOD("get_mass"), &PhysXVehicle3D::get_mass);
	ClassDB::bind_method(D_METHOD("set_moment_of_inertia", "moi"), &PhysXVehicle3D::set_moment_of_inertia);
	ClassDB::bind_method(D_METHOD("get_moment_of_inertia"), &PhysXVehicle3D::get_moment_of_inertia);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "1,10000,1,or_greater"), "set_mass", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "moment_of_inertia"), "set_moment_of_inertia", "get_moment_of_inertia");

	ClassDB::bind_method(D_METHOD("set_half_track", "value"), &PhysXVehicle3D::set_half_track);
	ClassDB::bind_method(D_METHOD("get_half_track"), &PhysXVehicle3D::get_half_track);
	ClassDB::bind_method(D_METHOD("set_front_axle_z", "value"), &PhysXVehicle3D::set_front_axle_z);
	ClassDB::bind_method(D_METHOD("get_front_axle_z"), &PhysXVehicle3D::get_front_axle_z);
	ClassDB::bind_method(D_METHOD("set_rear_axle_z", "value"), &PhysXVehicle3D::set_rear_axle_z);
	ClassDB::bind_method(D_METHOD("get_rear_axle_z"), &PhysXVehicle3D::get_rear_axle_z);
	ADD_GROUP("Chassis", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "half_track", PROPERTY_HINT_RANGE, "0.1,3,0.01,or_greater"), "set_half_track", "get_half_track");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "front_axle_z", PROPERTY_HINT_RANGE, "-5,5,0.01,or_lesser,or_greater"), "set_front_axle_z", "get_front_axle_z");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rear_axle_z", PROPERTY_HINT_RANGE, "-5,5,0.01,or_lesser,or_greater"), "set_rear_axle_z", "get_rear_axle_z");

	ClassDB::bind_method(D_METHOD("set_wheel_radius", "value"), &PhysXVehicle3D::set_wheel_radius);
	ClassDB::bind_method(D_METHOD("get_wheel_radius"), &PhysXVehicle3D::get_wheel_radius);
	ClassDB::bind_method(D_METHOD("set_wheel_half_width", "value"), &PhysXVehicle3D::set_wheel_half_width);
	ClassDB::bind_method(D_METHOD("get_wheel_half_width"), &PhysXVehicle3D::get_wheel_half_width);
	ClassDB::bind_method(D_METHOD("set_wheel_mass", "value"), &PhysXVehicle3D::set_wheel_mass);
	ClassDB::bind_method(D_METHOD("get_wheel_mass"), &PhysXVehicle3D::get_wheel_mass);
	ClassDB::bind_method(D_METHOD("set_wheel_moment_of_inertia", "value"), &PhysXVehicle3D::set_wheel_moment_of_inertia);
	ClassDB::bind_method(D_METHOD("get_wheel_moment_of_inertia"), &PhysXVehicle3D::get_wheel_moment_of_inertia);
	ClassDB::bind_method(D_METHOD("set_wheel_damping_rate", "value"), &PhysXVehicle3D::set_wheel_damping_rate);
	ClassDB::bind_method(D_METHOD("get_wheel_damping_rate"), &PhysXVehicle3D::get_wheel_damping_rate);
	ADD_GROUP("Wheels", "wheel_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_radius", PROPERTY_HINT_RANGE, "0.05,2,0.01,or_greater"), "set_wheel_radius", "get_wheel_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_half_width", PROPERTY_HINT_RANGE, "0.01,1,0.01,or_greater"), "set_wheel_half_width", "get_wheel_half_width");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_mass", PROPERTY_HINT_RANGE, "0.1,200,0.1,or_greater"), "set_wheel_mass", "get_wheel_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_moment_of_inertia", PROPERTY_HINT_RANGE, "0.01,50,0.01,or_greater"), "set_wheel_moment_of_inertia", "get_wheel_moment_of_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_damping_rate", PROPERTY_HINT_RANGE, "0,5,0.01,or_greater"), "set_wheel_damping_rate", "get_wheel_damping_rate");

	ClassDB::bind_method(D_METHOD("set_suspension_travel", "value"), &PhysXVehicle3D::set_suspension_travel);
	ClassDB::bind_method(D_METHOD("get_suspension_travel"), &PhysXVehicle3D::get_suspension_travel);
	ClassDB::bind_method(D_METHOD("set_suspension_stiffness", "value"), &PhysXVehicle3D::set_suspension_stiffness);
	ClassDB::bind_method(D_METHOD("get_suspension_stiffness"), &PhysXVehicle3D::get_suspension_stiffness);
	ClassDB::bind_method(D_METHOD("set_suspension_damping", "value"), &PhysXVehicle3D::set_suspension_damping);
	ClassDB::bind_method(D_METHOD("get_suspension_damping"), &PhysXVehicle3D::get_suspension_damping);
	ADD_GROUP("Suspension", "suspension_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_travel", PROPERTY_HINT_RANGE, "0.01,1,0.01,or_greater"), "set_suspension_travel", "get_suspension_travel");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_stiffness", PROPERTY_HINT_RANGE, "1000,100000,100,or_greater"), "set_suspension_stiffness", "get_suspension_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_damping", PROPERTY_HINT_RANGE, "0,20000,10,or_greater"), "set_suspension_damping", "get_suspension_damping");

	ClassDB::bind_method(D_METHOD("set_tire_lateral_stiffness", "value"), &PhysXVehicle3D::set_tire_lateral_stiffness);
	ClassDB::bind_method(D_METHOD("get_tire_lateral_stiffness"), &PhysXVehicle3D::get_tire_lateral_stiffness);
	ClassDB::bind_method(D_METHOD("set_tire_longitudinal_stiffness", "value"), &PhysXVehicle3D::set_tire_longitudinal_stiffness);
	ClassDB::bind_method(D_METHOD("get_tire_longitudinal_stiffness"), &PhysXVehicle3D::get_tire_longitudinal_stiffness);
	ClassDB::bind_method(D_METHOD("set_tire_friction", "value"), &PhysXVehicle3D::set_tire_friction);
	ClassDB::bind_method(D_METHOD("get_tire_friction"), &PhysXVehicle3D::get_tire_friction);
	ADD_GROUP("Tires", "tire_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_lateral_stiffness", PROPERTY_HINT_RANGE, "1000,100000,100,or_greater"), "set_tire_lateral_stiffness", "get_tire_lateral_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_longitudinal_stiffness", PROPERTY_HINT_RANGE, "1000,100000,100,or_greater"), "set_tire_longitudinal_stiffness", "get_tire_longitudinal_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_friction", PROPERTY_HINT_RANGE, "0.1,3,0.01,or_greater"), "set_tire_friction", "get_tire_friction");

	ClassDB::bind_method(D_METHOD("set_max_engine_torque", "value"), &PhysXVehicle3D::set_max_engine_torque);
	ClassDB::bind_method(D_METHOD("get_max_engine_torque"), &PhysXVehicle3D::get_max_engine_torque);
	ClassDB::bind_method(D_METHOD("set_max_brake_torque", "value"), &PhysXVehicle3D::set_max_brake_torque);
	ClassDB::bind_method(D_METHOD("get_max_brake_torque"), &PhysXVehicle3D::get_max_brake_torque);
	ClassDB::bind_method(D_METHOD("set_max_steer_angle", "value"), &PhysXVehicle3D::set_max_steer_angle);
	ClassDB::bind_method(D_METHOD("get_max_steer_angle"), &PhysXVehicle3D::get_max_steer_angle);
	ClassDB::bind_method(D_METHOD("set_ackermann_strength", "value"), &PhysXVehicle3D::set_ackermann_strength);
	ClassDB::bind_method(D_METHOD("get_ackermann_strength"), &PhysXVehicle3D::get_ackermann_strength);
	ADD_GROUP("Drivetrain", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_engine_torque", PROPERTY_HINT_RANGE, "0,5000,10,or_greater"), "set_max_engine_torque", "get_max_engine_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_brake_torque", PROPERTY_HINT_RANGE, "0,20000,10,or_greater"), "set_max_brake_torque", "get_max_brake_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_steer_angle", PROPERTY_HINT_RANGE, "0,1.5708,0.01"), "set_max_steer_angle", "get_max_steer_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ackermann_strength", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_ackermann_strength", "get_ackermann_strength");

	ClassDB::bind_method(D_METHOD("set_throttle", "value"), &PhysXVehicle3D::set_throttle);
	ClassDB::bind_method(D_METHOD("get_throttle"), &PhysXVehicle3D::get_throttle);
	ClassDB::bind_method(D_METHOD("set_brake", "value"), &PhysXVehicle3D::set_brake);
	ClassDB::bind_method(D_METHOD("get_brake"), &PhysXVehicle3D::get_brake);
	ClassDB::bind_method(D_METHOD("set_steer", "value"), &PhysXVehicle3D::set_steer);
	ClassDB::bind_method(D_METHOD("get_steer"), &PhysXVehicle3D::get_steer);
	ADD_GROUP("Controls", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "throttle", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_throttle", "get_throttle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_brake", "get_brake");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steer", PROPERTY_HINT_RANGE, "-1,1,0.01"), "set_steer", "get_steer");

	ClassDB::bind_method(D_METHOD("get_linear_velocity"), &PhysXVehicle3D::get_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_forward_speed"), &PhysXVehicle3D::get_forward_speed);
}

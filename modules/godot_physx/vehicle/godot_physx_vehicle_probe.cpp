/**************************************************************************/
/*  godot_physx_vehicle_probe.cpp                                         */
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

#include "godot_physx_vehicle_probe.h"

#include "../godot_physx_conversions.h"
#include "../godot_physx_server_3d.h"
#include "../spaces/godot_physx_space_3d.h"
#include "godot_physx_vehicle4w.h"

#include "core/object/class_db.h"

struct GodotPhysXVehicleProbe::Impl {
	Vehicle4W vehicle;
	PxVehiclePhysXSimulationContext simulationContext;
	PxScene *scene = nullptr;
	bool initialized = false;
};

GodotPhysXVehicleProbe::GodotPhysXVehicleProbe() {
	impl = memnew(Impl);
}

GodotPhysXVehicleProbe::~GodotPhysXVehicleProbe() {
	if (impl) {
		if (impl->initialized && impl->scene) {
			impl->scene->removeActor(*impl->vehicle.physxActor.rigidBody);
			impl->vehicle.destroy();
		}
		memdelete(impl);
	}
}

bool GodotPhysXVehicleProbe::initialize(RID p_space, const Vector3 &p_position, real_t p_wheel_radius) {
	ERR_FAIL_COND_V(impl->initialized, false);

	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	ERR_FAIL_NULL_V(server, false);
	GodotPhysXSpace3D *space = server->get_space(p_space);
	ERR_FAIL_NULL_V(space, false);
	PxPhysics *physics = space->get_px_physics();
	PxScene *scene = space->get_px_scene();
	ERR_FAIL_NULL_V(physics, false);
	ERR_FAIL_NULL_V(scene, false);

	// A ~1500kg sedan: half-track 0.75m, wheelbase 2.7m (front axle +1.35,
	// rear -1.35) -- explicit here (not just Vehicle4WWheelConfig's own
	// defaults) since wheel *position* has no sensible default of its own,
	// unlike every other per-wheel field.
	Vehicle4WConfig cfg;
	cfg.wheels[0].position = Vector3(-0.75f, 0.05f, 1.35f); // FL
	cfg.wheels[0].use_as_steering = true;
	cfg.wheels[1].position = Vector3(0.75f, 0.05f, 1.35f); // FR
	cfg.wheels[1].use_as_steering = true;
	cfg.wheels[2].position = Vector3(-0.75f, 0.05f, -1.35f); // RL
	cfg.wheels[2].use_as_steering = false;
	cfg.wheels[3].position = Vector3(0.75f, 0.05f, -1.35f); // RR
	cfg.wheels[3].use_as_steering = false;

	if (p_wheel_radius > 0.0) {
		for (int i = 0; i < 4; i++) {
			cfg.wheels[i].radius = p_wheel_radius;
		}
	}

	PxU32 wheel_order[4];
	if (!configure_vehicle4w(impl->vehicle, cfg, *physics, *scene, impl->simulationContext, wheel_order)) {
		return false;
	}

	Vehicle4W &v = impl->vehicle;
	const PxTransform startPose(to_px(p_position), PxQuat(PxIdentity));
	v.physxActor.rigidBody->setGlobalPose(startPose);
	scene->addActor(*v.physxActor.rigidBody);
	v.physxActor.rigidBody->setName("GodotPhysXVehicleProbe");

	impl->scene = scene;
	impl->initialized = true;
	return true;
}

void GodotPhysXVehicleProbe::step(real_t p_dt, real_t p_throttle, real_t p_brake, real_t p_steer) {
	ERR_FAIL_COND(!impl->initialized);
	Vehicle4W &v = impl->vehicle;
	v.commandState.throttle = (PxReal)p_throttle;
	v.commandState.brakes[0] = (PxReal)p_brake;
	v.commandState.nbBrakes = 1;
	v.commandState.steer = (PxReal)p_steer;
	v.step((PxReal)p_dt, impl->simulationContext);
}

Vector3 GodotPhysXVehicleProbe::get_position() const {
	ERR_FAIL_COND_V(!impl->initialized, Vector3());
	return to_godot(impl->vehicle.rigidBodyState.pose.p);
}

Vector3 GodotPhysXVehicleProbe::get_linear_velocity() const {
	ERR_FAIL_COND_V(!impl->initialized, Vector3());
	return to_godot(impl->vehicle.rigidBodyState.linearVelocity);
}

real_t GodotPhysXVehicleProbe::get_forward_speed() const {
	ERR_FAIL_COND_V(!impl->initialized, 0.0);
	const PxVec3 fwd = impl->vehicle.frame.getLngAxis();
	return (real_t)impl->vehicle.rigidBodyState.linearVelocity.dot(fwd);
}

Vector3 GodotPhysXVehicleProbe::get_actor_position() const {
	ERR_FAIL_COND_V(!impl->initialized, Vector3());
	return to_godot(impl->vehicle.physxActor.rigidBody->getGlobalPose().p);
}

Vector3 GodotPhysXVehicleProbe::get_wheel_position(int p_wheel) const {
	ERR_FAIL_COND_V(!impl->initialized, Vector3());
	ERR_FAIL_INDEX_V(p_wheel, 4, Vector3());
	// wheelLocalPoses[i].localPose is in the CoM frame, not the actor's own
	// origin frame -- see PhysXVehicle3D's identical note for the SDK source
	// citation that confirmed this.
	const PxTransform actor_pose = impl->vehicle.physxActor.rigidBody->getGlobalPose();
	const PxTransform cmass_local_pose = impl->vehicle.physxActor.rigidBody->getCMassLocalPose();
	const PxTransform wheel_world = actor_pose * cmass_local_pose * impl->vehicle.wheelLocalPoses[p_wheel].localPose;
	return to_godot(wheel_world.p);
}

real_t GodotPhysXVehicleProbe::get_wheel_jounce(int p_wheel) const {
	ERR_FAIL_COND_V(!impl->initialized, 0.0);
	ERR_FAIL_INDEX_V(p_wheel, 4, 0.0);
	return (real_t)impl->vehicle.suspensionStates[p_wheel].jounce;
}

real_t GodotPhysXVehicleProbe::get_wheel_separation(int p_wheel) const {
	ERR_FAIL_COND_V(!impl->initialized, 0.0);
	ERR_FAIL_INDEX_V(p_wheel, 4, 0.0);
	return (real_t)impl->vehicle.suspensionStates[p_wheel].separation;
}

void GodotPhysXVehicleProbe::_bind_methods() {
	ClassDB::bind_method(D_METHOD("initialize", "space", "position", "wheel_radius"), &GodotPhysXVehicleProbe::initialize, DEFVAL(-1.0));
	ClassDB::bind_method(D_METHOD("step", "dt", "throttle", "brake", "steer"), &GodotPhysXVehicleProbe::step);
	ClassDB::bind_method(D_METHOD("get_position"), &GodotPhysXVehicleProbe::get_position);
	ClassDB::bind_method(D_METHOD("get_linear_velocity"), &GodotPhysXVehicleProbe::get_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_forward_speed"), &GodotPhysXVehicleProbe::get_forward_speed);
	ClassDB::bind_method(D_METHOD("get_wheel_jounce", "wheel"), &GodotPhysXVehicleProbe::get_wheel_jounce);
	ClassDB::bind_method(D_METHOD("get_wheel_separation", "wheel"), &GodotPhysXVehicleProbe::get_wheel_separation);
	ClassDB::bind_method(D_METHOD("get_actor_position"), &GodotPhysXVehicleProbe::get_actor_position);
	ClassDB::bind_method(D_METHOD("get_wheel_position", "wheel"), &GodotPhysXVehicleProbe::get_wheel_position);
}

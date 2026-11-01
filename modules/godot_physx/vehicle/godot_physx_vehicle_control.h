/**************************************************************************/
/*  godot_physx_vehicle_control.h                                         */
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

// RigidBody3D-style control of a PhysX vehicle's chassis, shared by
// PhysXVehicle3D, PhysXTank3D and PhysXMotorcycle3D: forces, torques and
// impulses (world space; a force's position is an offset from the chassis
// origin in world axes, as RigidBody3D's), velocities, a teleport that keeps
// the vehicle (no rebuild), and a live tire grip multiplier. And its identity in
// the physics server: get_rid() names a body standing for the chassis (see
// GodotPhysXBody3D::set_vehicle_actor), which raycasts and CharacterBody3D
// motion report as the collider, exclude lists take, and collision exceptions
// pair with -- add_collision_exception_with() here, or from a body's side
// PhysicsServer3D.body_add_collision_exception(body_rid, vehicle.get_rid()).
//
// A class declares the methods with PHYSX_VEHICLE_CONTROL_DECLARE() and
// defines / binds them with PHYSX_VEHICLE_CONTROL_DEFINE / _BIND. Its Impl
// needs `built` and `vehicle` (with physxActor, physxMaterialFrictionParams,
// wheelRigidBody1dStates, tireStickyStates); m_wheel_count is an expression for
// how many wheels the vehicle has. Call _store_tire_grip_base() once the vehicle
// is configured, so tire_grip scales the tires' own friction -- through
// _control_built(), which also binds the identity; _control_unbind() before the
// chassis actor goes, _control_free() in the destructor.
//
// With physics/3d/run_on_separate_thread the PhysX scene steps on its own thread,
// so the chassis actor is touched only where it may be (see
// GodotPhysXServer3D::run_on_physics_thread()): writes are queued in order for
// that thread, building and freeing wait for it, and the getters read the actor
// live in physics process and otherwise the state cached at the end of it
// (_cache_chassis()), as RigidBody3D's do.

#include "../godot_physx_conversions.h"
#include "../godot_physx_server_3d.h"
#include "../objects/godot_physx_body_3d.h"

#include "core/math/transform_3d.h"
#include "core/object/class_db.h"
#include "core/variant/typed_array.h"
#include "scene/3d/physics/physics_body_3d.h"

// The RID a collision exception takes: a PhysicsBody3D's, or another PhysX vehicle's identity.
inline RID godot_physx_vehicle_exception_rid(Node *p_node) {
	if (PhysicsBody3D *body = Object::cast_to<PhysicsBody3D>(p_node)) {
		return body->get_rid();
	}
	if (p_node && (p_node->is_class("PhysXVehicle3D") || p_node->is_class("PhysXTank3D") || p_node->is_class("PhysXMotorcycle3D"))) {
		return p_node->call("get_rid");
	}
	return RID();
}

#include <PxPhysicsAPI.h>

#define PHYSX_VEHICLE_CONTROL_DECLARE() \
public: \
	void apply_central_force(const Vector3 &p_force); \
	void apply_force(const Vector3 &p_force, const Vector3 &p_position = Vector3()); \
	void apply_torque(const Vector3 &p_torque); \
	void apply_central_impulse(const Vector3 &p_impulse); \
	void apply_impulse(const Vector3 &p_impulse, const Vector3 &p_position = Vector3()); \
	void apply_torque_impulse(const Vector3 &p_impulse); \
	void set_linear_velocity(const Vector3 &p_velocity); \
	void set_angular_velocity(const Vector3 &p_velocity); \
	Vector3 get_angular_velocity() const; \
	void teleport(const Transform3D &p_transform, bool p_keep_velocity = false); \
	void set_tire_grip(real_t p_grip); \
	real_t get_tire_grip() const { \
		return tire_grip; \
	} \
	RID get_rid() const; \
	void add_collision_exception_with(Node *p_node); \
	void remove_collision_exception_with(Node *p_node); \
	TypedArray<Node> get_collision_exceptions() const; \
\
private: \
	mutable RID proxy_rid; /* the identity body, created on first use */ \
	GodotPhysXBody3D *_proxy_body() const; \
	void _control_built(); \
	void _control_unbind(); \
	void _control_free(); \
	/* Live multiplier on the tires' friction (terrain grip); the base values */ \
	/* are the tires' own, stored when the vehicle is built. */ \
	real_t tire_grip = 1.0; \
	float tire_base_friction[32] = {}; \
	physx::PxRigidDynamic *_control_body() const; \
	void _store_tire_grip_base(); \
	void _apply_tire_grip(); \
	/* The chassis state as of the end of the last physics process, for the getters outside it. */ \
	physx::PxTransform chassis_pose = physx::PxTransform(physx::PxIdentity); \
	physx::PxVec3 chassis_linear_velocity = physx::PxVec3(0.0f); \
	physx::PxVec3 chassis_angular_velocity = physx::PxVec3(0.0f); \
	bool chassis_sleeping = false; \
	void _cache_chassis(); \
	physx::PxTransform _chassis_pose() const; \
	physx::PxVec3 _chassis_linear_velocity() const; \
	physx::PxVec3 _chassis_angular_velocity() const; \
	bool _chassis_sleeping() const; \
\
public:

#define PHYSX_VEHICLE_CONTROL_DEFINE(m_class, m_wheel_count) \
	physx::PxRigidDynamic *m_class::_control_body() const { \
		return impl->built ? impl->vehicle.physxActor.rigidBody->is<physx::PxRigidDynamic>() : nullptr; \
	} \
	void m_class::_cache_chassis() { \
		if (physx::PxRigidDynamic *b = _control_body()) { \
			chassis_pose = b->getGlobalPose(); \
			chassis_linear_velocity = b->getLinearVelocity(); \
			chassis_angular_velocity = b->getAngularVelocity(); \
			chassis_sleeping = b->isSleeping(); \
		} \
	} \
	physx::PxTransform m_class::_chassis_pose() const { \
		physx::PxRigidDynamic *b = _control_body(); \
		return b != nullptr && GodotPhysXServer3D::can_touch_scene() ? b->getGlobalPose() : chassis_pose; \
	} \
	physx::PxVec3 m_class::_chassis_linear_velocity() const { \
		physx::PxRigidDynamic *b = _control_body(); \
		return b != nullptr && GodotPhysXServer3D::can_touch_scene() ? b->getLinearVelocity() : chassis_linear_velocity; \
	} \
	physx::PxVec3 m_class::_chassis_angular_velocity() const { \
		physx::PxRigidDynamic *b = _control_body(); \
		return b != nullptr && GodotPhysXServer3D::can_touch_scene() ? b->getAngularVelocity() : chassis_angular_velocity; \
	} \
	bool m_class::_chassis_sleeping() const { \
		physx::PxRigidDynamic *b = _control_body(); \
		return b != nullptr && GodotPhysXServer3D::can_touch_scene() ? b->isSleeping() : chassis_sleeping; \
	} \
	void m_class::apply_central_force(const Vector3 &p_force) { \
		if (physx::PxRigidDynamic *b = _control_body()) { \
			GodotPhysXServer3D::run_on_physics_thread([=]() { b->addForce(to_px(p_force), physx::PxForceMode::eFORCE); }); \
		} \
	} \
	void m_class::apply_force(const Vector3 &p_force, const Vector3 &p_position) { \
		if (physx::PxRigidDynamic *b = _control_body()) { \
			GodotPhysXServer3D::run_on_physics_thread([=]() { physx::PxRigidBodyExt::addForceAtPos(*b, to_px(p_force), b->getGlobalPose().p + to_px(p_position), physx::PxForceMode::eFORCE); }); \
		} \
	} \
	void m_class::apply_torque(const Vector3 &p_torque) { \
		if (physx::PxRigidDynamic *b = _control_body()) { \
			GodotPhysXServer3D::run_on_physics_thread([=]() { b->addTorque(to_px(p_torque), physx::PxForceMode::eFORCE); }); \
		} \
	} \
	void m_class::apply_central_impulse(const Vector3 &p_impulse) { \
		if (physx::PxRigidDynamic *b = _control_body()) { \
			GodotPhysXServer3D::run_on_physics_thread([=]() { b->addForce(to_px(p_impulse), physx::PxForceMode::eIMPULSE); }); \
		} \
	} \
	void m_class::apply_impulse(const Vector3 &p_impulse, const Vector3 &p_position) { \
		if (physx::PxRigidDynamic *b = _control_body()) { \
			GodotPhysXServer3D::run_on_physics_thread([=]() { physx::PxRigidBodyExt::addForceAtPos(*b, to_px(p_impulse), b->getGlobalPose().p + to_px(p_position), physx::PxForceMode::eIMPULSE); }); \
		} \
	} \
	void m_class::apply_torque_impulse(const Vector3 &p_impulse) { \
		if (physx::PxRigidDynamic *b = _control_body()) { \
			GodotPhysXServer3D::run_on_physics_thread([=]() { b->addTorque(to_px(p_impulse), physx::PxForceMode::eIMPULSE); }); \
		} \
	} \
	void m_class::set_linear_velocity(const Vector3 &p_velocity) { \
		if (physx::PxRigidDynamic *b = _control_body()) { \
			chassis_linear_velocity = to_px(p_velocity); \
			GodotPhysXServer3D::run_on_physics_thread([=]() { b->setLinearVelocity(to_px(p_velocity)); }); \
		} \
	} \
	void m_class::set_angular_velocity(const Vector3 &p_velocity) { \
		if (physx::PxRigidDynamic *b = _control_body()) { \
			chassis_angular_velocity = to_px(p_velocity); \
			GodotPhysXServer3D::run_on_physics_thread([=]() { b->setAngularVelocity(to_px(p_velocity)); }); \
		} \
	} \
	Vector3 m_class::get_angular_velocity() const { \
		return _control_body() != nullptr ? to_godot(_chassis_angular_velocity()) : Vector3(); \
	} \
	void m_class::teleport(const Transform3D &p_transform, bool p_keep_velocity) { \
		physx::PxRigidDynamic *b = _control_body(); \
		if (b == nullptr) { \
			set_global_transform(p_transform); \
			return; \
		} \
		/* The chassis moves in place: same actor, wheels and settings, nothing rebuilt. */ \
		const physx::PxTransform pose = to_px(p_transform); \
		GodotPhysXServer3D::run_on_physics_thread([=]() { \
			b->setGlobalPose(pose); \
			if (!p_keep_velocity) { \
				b->setLinearVelocity(physx::PxVec3(0.0f)); \
				b->setAngularVelocity(physx::PxVec3(0.0f)); \
			} \
			b->wakeUp(); \
		}); \
		chassis_pose = pose; \
		if (!p_keep_velocity) { \
			chassis_linear_velocity = physx::PxVec3(0.0f); \
			chassis_angular_velocity = physx::PxVec3(0.0f); \
			/* And the wheels stop spinning; no tire stays "stuck" from before. */ \
			for (uint32_t i = 0; i < (uint32_t)(m_wheel_count); i++) { \
				impl->vehicle.wheelRigidBody1dStates[i].setToDefault(); \
				impl->vehicle.tireStickyStates[i].setToDefault(); \
			} \
			impl->vehicle.stop_drivetrain(); /* an engine drops to idle, or the wheels spin back up */ \
		} \
		set_global_transform(to_godot(pose)); \
	} \
	void m_class::set_tire_grip(real_t p_grip) { \
		tire_grip = MAX(p_grip, (real_t)0.0); \
		_apply_tire_grip(); \
	} \
	void m_class::_store_tire_grip_base() { \
		for (uint32_t i = 0; i < (uint32_t)(m_wheel_count) && i < 32; i++) { \
			tire_base_friction[i] = impl->vehicle.physxMaterialFrictionParams[i].defaultFriction; \
		} \
		_apply_tire_grip(); \
	} \
	void m_class::_apply_tire_grip() { \
		if (!impl->built) { \
			return; \
		} \
		/* The road friction the tire model scales each tire's grip by -- read every step, so */ \
		/* this takes effect at once (the wheels' tire_* properties rebuild the vehicle). */ \
		for (uint32_t i = 0; i < (uint32_t)(m_wheel_count) && i < 32; i++) { \
			impl->vehicle.physxMaterialFrictionParams[i].defaultFriction = tire_base_friction[i] * (float)tire_grip; \
		} \
	} \
	GodotPhysXBody3D *m_class::_proxy_body() const { \
		GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton(); \
		if (server == nullptr) { \
			return nullptr; \
		} \
		if (!proxy_rid.is_valid()) { \
			proxy_rid = server->body_create(); \
			server->body_attach_object_instance_id(proxy_rid, get_instance_id()); \
		} \
		return server->get_body(proxy_rid); \
	} \
	void m_class::_control_built() { \
		_store_tire_grip_base(); \
		if (GodotPhysXBody3D *body = _proxy_body()) { \
			body->set_collision_layer(collision_layer); \
			body->set_collision_mask(collision_mask); \
			body->set_vehicle_actor(_control_body()); \
			impl->vehicle.wheelQueryFilter.self_body = body; \
		} \
	} \
	void m_class::_control_unbind() { \
		if (GodotPhysXBody3D *body = proxy_rid.is_valid() ? _proxy_body() : nullptr) { \
			body->set_vehicle_actor(nullptr); \
		} \
		impl->vehicle.wheelQueryFilter.self_body = nullptr; \
	} \
	void m_class::_control_free() { \
		GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton(); \
		if (server != nullptr && proxy_rid.is_valid()) { \
			server->free_rid(proxy_rid); \
		} \
		proxy_rid = RID(); \
	} \
	RID m_class::get_rid() const { \
		_proxy_body(); \
		return proxy_rid; \
	} \
	void m_class::add_collision_exception_with(Node *p_node) { \
		const RID other = godot_physx_vehicle_exception_rid(p_node); \
		ERR_FAIL_COND_MSG(!other.is_valid(), "Collision exception only works with a PhysicsBody3D or a PhysX vehicle."); \
		if (GodotPhysXBody3D *body = _proxy_body()) { \
			GodotPhysXServer3D::run_on_physics_thread([=]() { body->add_collision_exception(other); }); \
		} \
	} \
	void m_class::remove_collision_exception_with(Node *p_node) { \
		const RID other = godot_physx_vehicle_exception_rid(p_node); \
		ERR_FAIL_COND_MSG(!other.is_valid(), "Collision exception only works with a PhysicsBody3D or a PhysX vehicle."); \
		if (GodotPhysXBody3D *body = _proxy_body()) { \
			GodotPhysXServer3D::run_on_physics_thread([=]() { body->remove_collision_exception(other); }); \
		} \
	} \
	TypedArray<Node> m_class::get_collision_exceptions() const { \
		TypedArray<Node> out; \
		GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton(); \
		const GodotPhysXBody3D *body = _proxy_body(); \
		if (body == nullptr) { \
			return out; \
		} \
		LocalVector<RID> exceptions; \
		GodotPhysXServer3D::run_on_physics_thread([&]() { \
			for (const RID &rid : body->get_collision_exceptions()) { \
				exceptions.push_back(rid); \
			} \
		}, \
				true); \
		for (const RID &rid : exceptions) { \
			const GodotPhysXBody3D *other = server->get_body(rid); \
			if (Node *node = other ? Object::cast_to<Node>(ObjectDB::get_instance(other->get_instance_id())) : nullptr) { \
				out.push_back(node); \
			} \
		} \
		return out; \
	}

#define PHYSX_VEHICLE_CONTROL_BIND(m_class) \
	ClassDB::bind_method(D_METHOD("apply_central_force", "force"), &m_class::apply_central_force); \
	ClassDB::bind_method(D_METHOD("apply_force", "force", "position"), &m_class::apply_force, DEFVAL(Vector3())); \
	ClassDB::bind_method(D_METHOD("apply_torque", "torque"), &m_class::apply_torque); \
	ClassDB::bind_method(D_METHOD("apply_central_impulse", "impulse"), &m_class::apply_central_impulse); \
	ClassDB::bind_method(D_METHOD("apply_impulse", "impulse", "position"), &m_class::apply_impulse, DEFVAL(Vector3())); \
	ClassDB::bind_method(D_METHOD("apply_torque_impulse", "impulse"), &m_class::apply_torque_impulse); \
	ClassDB::bind_method(D_METHOD("set_linear_velocity", "velocity"), &m_class::set_linear_velocity); \
	ClassDB::bind_method(D_METHOD("set_angular_velocity", "velocity"), &m_class::set_angular_velocity); \
	ClassDB::bind_method(D_METHOD("get_angular_velocity"), &m_class::get_angular_velocity); \
	ClassDB::bind_method(D_METHOD("teleport", "transform", "keep_velocity"), &m_class::teleport, DEFVAL(false)); \
	ClassDB::bind_method(D_METHOD("set_tire_grip", "grip"), &m_class::set_tire_grip); \
	ClassDB::bind_method(D_METHOD("get_tire_grip"), &m_class::get_tire_grip); \
	ClassDB::bind_method(D_METHOD("get_rid"), &m_class::get_rid); \
	ClassDB::bind_method(D_METHOD("add_collision_exception_with", "body"), &m_class::add_collision_exception_with); \
	ClassDB::bind_method(D_METHOD("remove_collision_exception_with", "body"), &m_class::remove_collision_exception_with); \
	ClassDB::bind_method(D_METHOD("get_collision_exceptions"), &m_class::get_collision_exceptions); \
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_grip", PROPERTY_HINT_RANGE, "0,4,0.01,or_greater"), "set_tire_grip", "get_tire_grip");

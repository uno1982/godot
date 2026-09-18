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
#include "physx_vehicle_wheel_3d.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/resources/3d/box_shape_3d.h"
#include "scene/resources/3d/world_3d.h"

struct PhysXVehicle3D::Impl {
	Vehicle4W vehicle;
	PxVehiclePhysXSimulationContext simulationContext;
	PxScene *scene = nullptr;
	bool built = false;
	// wheel_order[Vehicle4W::WHEEL_FL/FR/RL/RR] = index into the parent's own
	// `wheels` vector (child-registration order) for that canonical slot --
	// see configure_vehicle4w()'s own doc comment.
	PxU32 wheel_order[4] = {};
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
	// PxVehicleRigidBodyComponent integrates gravity/velocity into the
	// chassis pose itself, inside v.step() -- independent of whether
	// PxScene::simulate() is ever called. In the editor (not Play), nothing
	// steps the scene, but NOTIFICATION_INTERNAL_PHYSICS_PROCESS still fires
	// on this node, so without this guard the car just falls forever right
	// in the viewport the moment it's placed. Same precedent as
	// PhysXDestructible3D: no physics runs in the editor at all, only real
	// gameplay (Engine::is_editor_hint() == false).
	if (Engine::get_singleton()->is_editor_hint()) {
		return false;
	}
	if (!is_inside_world() || get_world_3d().is_null()) {
		return false;
	}
	if (wheels.size() != 4) {
		// Not a real error -- this fires transiently while wheel children are
		// still entering the tree one at a time (see PhysXVehicleWheel3D's
		// own NOTIFICATION_ENTER_TREE), and permanently if the scene author
		// hasn't added exactly 4 yet. get_configuration_warnings() surfaces
		// the permanent case in the Inspector.
		return false;
	}

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
		ERR_PRINT("PhysXVehicle3D: chassis CollisionShape3D must use a BoxShape3D.");
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
	cfg.chassis_half_extents = box_shape->get_size() * 0.5;
	cfg.chassis_box_center_local = chassis_shape_node->get_position();
	cfg.chassis_com_local = center_of_mass_mode == CENTER_OF_MASS_MODE_CUSTOM ? center_of_mass : chassis_shape_node->get_position();
	cfg.max_engine_torque = max_engine_torque;
	cfg.max_brake_torque = max_brake_torque;
	cfg.max_steer_angle = max_steer_angle;
	cfg.ackermann_strength = ackermann_strength;
	cfg.collision_layer = collision_layer;
	cfg.collision_mask = collision_mask;

	for (int i = 0; i < 4; i++) {
		PhysXVehicleWheel3D *w = wheels[i];
		Vehicle4WWheelConfig &wc = cfg.wheels[i];
		wc.position = w->get_position();
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
		wc.tire_friction = w->get_tire_friction();
		wc.tire_rest_grip = w->get_tire_rest_grip();
		wc.tire_slide_grip = w->get_tire_slide_grip();
		wc.use_as_steering = w->is_used_as_steering();
		wc.use_as_traction = w->is_used_as_traction();
	}

	if (!configure_vehicle4w(impl->vehicle, cfg, *physics, *scene, impl->simulationContext, impl->wheel_order)) {
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
	if (!is_inside_world()) {
		// Not in the tree/world yet -- NOTIFICATION_ENTER_WORLD will do the
		// real first build once it is.
		return;
	}
	// Unconditional, not just "if already built" -- a wheel child can finish
	// registering (e.g. the 4th of 4, added after this node's own
	// NOTIFICATION_ENTER_WORLD already ran and failed with too few wheels)
	// and that later success needs a real attempt here, not just a rebuild
	// of something that was never built. Also toggles physics processing
	// itself -- NOTIFICATION_ENTER_WORLD isn't the only path that can
	// transition built false->true (a wheel registering after is exactly
	// that), so it can't be the only place this gets turned on.
	_destroy();
	set_physics_process_internal(_build());
}

void PhysXVehicle3D::_notification(int p_what) {
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
			Vehicle4W &v = impl->vehicle;
			v.commandState.throttle = (PxReal)throttle;
			v.commandState.brakes[0] = (PxReal)brake;
			v.commandState.nbBrakes = 1;
			v.commandState.steer = (PxReal)steer;
			v.transmissionCommandState.gear = reverse ? PxVehicleDirectDriveTransmissionCommandState::eREVERSE : PxVehicleDirectDriveTransmissionCommandState::eFORWARD;
			v.step((PxReal)get_physics_process_delta_time(), impl->simulationContext);
			// v.rigidBodyState.pose is CoM-relative, not the actor's real
			// origin (confirmed directly by comparing it against a raw
			// getGlobalPose() read while root-causing the chassis-collision
			// regression) -- the CollisionShape3D/wheel/mesh children are all
			// positioned relative to the actor's own origin (that's the frame
			// PxVehiclePhysXActorCreate cooked the real PxShape geometry
			// into), so this node's own transform has to track that same
			// origin, not the CoM. Using rigidBodyState.pose here left the
			// visual body a constant offset away from where the real
			// collision geometry actually sits at rest, and made that offset
			// visibly swing whenever the body rotated (braking, cornering, a
			// bump) since it rotates with the body.
			set_global_transform(to_godot(v.physxActor.rigidBody->getGlobalPose()));
			// Push each wheel's live pose (suspension jounce + steer angle +
			// roll spin, all baked into wheelLocalPoses by PxVehicleWheelComponent)
			// onto that wheel's own node -- same idea as VehicleBody3D's own
			// _update_wheel_transform(): the wheel NODE moves, and any
			// MeshInstance3D the scene author parented under it inherits that
			// motion automatically, no separate mesh-transform wiring needed.
			//
			// wheelLocalPoses[i].localPose is in the CoM frame, NOT the actor's
			// own origin frame (confirmed directly in PhysX's own write-back
			// code, VhPhysXActorFunctions.cpp: "Local pose in actor frame" is
			// computed as cmassLocalPose * wheelLocalPose, not wheelLocalPose
			// alone) -- this node's own transform above is the actor's real
			// origin, so composing wheelLocalPose directly onto it left every
			// wheel a constant offset away (equal to the CoM offset, ~0.35m by
			// default) from where it should be, visible as wheels sunk deep
			// into the ground regardless of wheel radius (a real regression
			// caught by an A/B on radius that showed the SAME absolute
			// penetration depth for two different radii -- ruling out anything
			// radius-proportional and pointing straight at a fixed frame
			// offset instead).
			const PxTransform cmass_local_pose = v.physxActor.rigidBody->getCMassLocalPose();
			for (uint32_t i = 0; i < 4; i++) {
				wheels[impl->wheel_order[i]]->set_transform(to_godot(cmass_local_pose * v.wheelLocalPoses[i].localPose));
			}
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

real_t PhysXVehicle3D::get_wheel_jounce(int p_wheel) const {
	if (!impl->built) {
		return 0.0;
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), 0.0);
	for (uint32_t i = 0; i < 4; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			return (real_t)impl->vehicle.suspensionStates[i].jounce;
		}
	}
	return 0.0;
}

real_t PhysXVehicle3D::get_wheel_separation(int p_wheel) const {
	if (!impl->built) {
		return 0.0;
	}
	ERR_FAIL_INDEX_V((uint32_t)p_wheel, wheels.size(), 0.0);
	for (uint32_t i = 0; i < 4; i++) {
		if (impl->wheel_order[i] == (uint32_t)p_wheel) {
			return (real_t)impl->vehicle.suspensionStates[i].separation;
		}
	}
	return 0.0;
}

Vector3 PhysXVehicle3D::get_actor_position() const {
	if (!impl->built) {
		return Vector3();
	}
	return to_godot(impl->vehicle.physxActor.rigidBody->getGlobalPose().p);
}

PackedStringArray PhysXVehicle3D::get_configuration_warnings() const {
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
		warnings.push_back(RTR("PhysXVehicle3D needs a CollisionShape3D child with a BoxShape3D for its chassis."));
	}
	int nb_steering = 0;
	for (uint32_t i = 0; i < wheels.size(); i++) {
		if (wheels[i]->is_used_as_steering()) {
			nb_steering++;
		}
	}
	if (wheels.size() != 4) {
		warnings.push_back(vformat(RTR("PhysXVehicle3D needs exactly 4 PhysXVehicleWheel3D children (has %d)."), (int)wheels.size()));
	} else if (nb_steering != 2) {
		warnings.push_back(vformat(RTR("PhysXVehicle3D needs exactly 2 wheels with use_as_steering enabled (has %d)."), nb_steering));
	}
	return warnings;
}

#define PHYSX_VEHICLE_SETTER(m_name, m_field)      \
	void PhysXVehicle3D::set_##m_name(real_t p_v) { \
		m_field = p_v;                              \
		_rebuild_if_live();                         \
	}

void PhysXVehicle3D::set_mass(real_t p_mass) {
	mass = p_mass;
	// Wheel gizmos estimate rest ride height from this vehicle's own mass
	// (sprung_mass = mass * 0.25, matching configure_vehicle4w()) -- refresh
	// them so the editor preview stays accurate as mass is tuned.
	for (PhysXVehicleWheel3D *w : wheels) {
		w->update_gizmos();
	}
	_rebuild_if_live();
}
void PhysXVehicle3D::set_moment_of_inertia(const Vector3 &p_moi) {
	moment_of_inertia = p_moi;
	_rebuild_if_live();
}
void PhysXVehicle3D::set_center_of_mass_mode(CenterOfMassMode p_mode) {
	if (center_of_mass_mode == p_mode) {
		return;
	}
	center_of_mass_mode = p_mode;
	notify_property_list_changed();
	_rebuild_if_live();
}
void PhysXVehicle3D::set_center_of_mass(const Vector3 &p_center_of_mass) {
	if (center_of_mass == p_center_of_mass) {
		return;
	}
	ERR_FAIL_COND(center_of_mass_mode != CENTER_OF_MASS_MODE_CUSTOM);
	center_of_mass = p_center_of_mass;
	_rebuild_if_live();
}
PHYSX_VEHICLE_SETTER(max_engine_torque, max_engine_torque)
PHYSX_VEHICLE_SETTER(max_brake_torque, max_brake_torque)
PHYSX_VEHICLE_SETTER(max_steer_angle, max_steer_angle)
PHYSX_VEHICLE_SETTER(ackermann_strength, ackermann_strength)

#undef PHYSX_VEHICLE_SETTER

void PhysXVehicle3D::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;
	_rebuild_if_live();
}
void PhysXVehicle3D::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
	_rebuild_if_live();
}

void PhysXVehicle3D::_validate_property(PropertyInfo &p_property) const {
	if (center_of_mass_mode != CENTER_OF_MASS_MODE_CUSTOM && p_property.name == "center_of_mass") {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void PhysXVehicle3D::_bind_methods() {
	BIND_ENUM_CONSTANT(CENTER_OF_MASS_MODE_AUTO);
	BIND_ENUM_CONSTANT(CENTER_OF_MASS_MODE_CUSTOM);

	ClassDB::bind_method(D_METHOD("set_mass", "mass"), &PhysXVehicle3D::set_mass);
	ClassDB::bind_method(D_METHOD("get_mass"), &PhysXVehicle3D::get_mass);
	ClassDB::bind_method(D_METHOD("set_moment_of_inertia", "moi"), &PhysXVehicle3D::set_moment_of_inertia);
	ClassDB::bind_method(D_METHOD("get_moment_of_inertia"), &PhysXVehicle3D::get_moment_of_inertia);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "1,10000,1,or_greater"), "set_mass", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "moment_of_inertia"), "set_moment_of_inertia", "get_moment_of_inertia");
	ClassDB::bind_method(D_METHOD("set_center_of_mass_mode", "mode"), &PhysXVehicle3D::set_center_of_mass_mode);
	ClassDB::bind_method(D_METHOD("get_center_of_mass_mode"), &PhysXVehicle3D::get_center_of_mass_mode);
	ClassDB::bind_method(D_METHOD("set_center_of_mass", "center_of_mass"), &PhysXVehicle3D::set_center_of_mass);
	ClassDB::bind_method(D_METHOD("get_center_of_mass"), &PhysXVehicle3D::get_center_of_mass);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "center_of_mass_mode", PROPERTY_HINT_ENUM, "Auto,Custom"), "set_center_of_mass_mode", "get_center_of_mass_mode");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "center_of_mass", PROPERTY_HINT_RANGE, "-10,10,0.01,or_less,or_greater,suffix:m"), "set_center_of_mass", "get_center_of_mass");

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
	ClassDB::bind_method(D_METHOD("set_reverse", "value"), &PhysXVehicle3D::set_reverse);
	ClassDB::bind_method(D_METHOD("is_reverse"), &PhysXVehicle3D::is_reverse);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "reverse"), "set_reverse", "is_reverse");

	ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"), &PhysXVehicle3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &PhysXVehicle3D::get_collision_layer);
	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &PhysXVehicle3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &PhysXVehicle3D::get_collision_mask);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	ClassDB::bind_method(D_METHOD("get_linear_velocity"), &PhysXVehicle3D::get_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_forward_speed"), &PhysXVehicle3D::get_forward_speed);
	ClassDB::bind_method(D_METHOD("get_wheel_jounce", "wheel"), &PhysXVehicle3D::get_wheel_jounce);
	ClassDB::bind_method(D_METHOD("get_wheel_separation", "wheel"), &PhysXVehicle3D::get_wheel_separation);
	ClassDB::bind_method(D_METHOD("get_actor_position"), &PhysXVehicle3D::get_actor_position);
}

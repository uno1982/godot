/**************************************************************************/
/*  physx_vehicle_3d.h                                                    */
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

#include "godot_physx_vehicle_control.h"

#include "core/math/vector3.h"
#include "core/templates/local_vector.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/curve.h"
#include "scene/resources/physics_material.h"

class PhysXVehicleWheel3D;

// A real, PhysX-specific 4-wheel vehicle -- offers PxVehicle2 capability
// stock VehicleBody3D/VehicleWheel3D structurally can't (real engine torque
// response, Ackermann steering correction, a real slip-based tire friction
// curve), alongside (not replacing) this module's VehicleBody3D support.
// Wraps the same Vehicle4W/configure_vehicle4w() composition
// GodotPhysXVehicleProbe proved out headlessly (vehicle/godot_physx_vehicle4w.h)
// -- shared directly, not duplicated, since that layer is pure PxVehicle2
// composition with no project-specific behavior in it (unlike
// PhysXDestructible3D's probe, whose damage/fracture logic is genuinely
// node-specific and was deliberately duplicated instead).
//
// Node structure mirrors VehicleBody3D/VehicleWheel3D exactly, for a direct
// 1:1 comparison: a body node with a real CollisionShape3D (BoxShape3D only)
// child for the chassis, and exactly 4 PhysXVehicleWheel3D children for the
// wheels -- each wheel's own `position` is the suspension attachment
// hardpoint, so a MeshInstance3D child under it lines up with the real
// physics automatically instead of needing hand-guessed offsets (an earlier,
// flat-scalar-properties version of this node needed exactly that, and the
// offsets were wrong -- see the demo's own commit history).
//
// Owns its own PxRigidDynamic directly (via configure_vehicle4w), not a
// GodotPhysXBody3D -- PxVehicle2's own PxVehiclePhysXActorEndComponent writes
// wheel-shape local poses and rigid-body momentum straight onto the actor
// every tick, which needs raw actor/shape pointers, not the generic
// PhysicsServer3D RID abstraction.
class PhysXVehicle3D : public Node3D {
	GDCLASS(PhysXVehicle3D, Node3D);

	friend class PhysXVehicleWheel3D;
	LocalVector<PhysXVehicleWheel3D *> wheels;

public:
	enum CenterOfMassMode {
		CENTER_OF_MASS_MODE_AUTO,
		CENTER_OF_MASS_MODE_CUSTOM,
	};
	// How throttle reaches the wheels. DIRECT: straight to wheel torque (max_engine_torque), forward or reverse.
	// ENGINE: PhysX's engine drive -- an engine with a torque curve, clutch, gearbox (automatic or manual) and
	// differential, tuned with the engine_*, gear_*, autobox_*, clutch_* and differential properties.
	enum Drivetrain {
		DRIVETRAIN_DIRECT,
		DRIVETRAIN_ENGINE,
	};
	// Engine drive: how the engine's torque is shared between the traction wheels. OPEN: evenly, whatever each wheel
	// does. LIMITED_SLIP: torque moves away from a wheel spinning faster than its partner (limited_slip_*).
	enum Differential {
		DIFFERENTIAL_OPEN,
		DIFFERENTIAL_LIMITED_SLIP,
	};

protected:
	static void _bind_methods();
	void _notification(int p_what);
	void _validate_property(PropertyInfo &p_property) const;

public:
	void set_mass(real_t p_mass);
	real_t get_mass() const { return mass; }
	void set_moment_of_inertia(const Vector3 &p_moi);
	Vector3 get_moment_of_inertia() const { return moment_of_inertia; }
	void set_center_of_mass_mode(CenterOfMassMode p_mode);
	CenterOfMassMode get_center_of_mass_mode() const { return center_of_mass_mode; }
	void set_center_of_mass(const Vector3 &p_center_of_mass);
	const Vector3 &get_center_of_mass() const { return center_of_mass; }
	void set_can_sleep(bool p_can_sleep);
	bool is_able_to_sleep() const { return can_sleep; }
	bool is_sleeping() const;

	void set_drivetrain(Drivetrain p_drivetrain);
	Drivetrain get_drivetrain() const { return drivetrain; }
	void set_max_engine_torque(real_t p_v);
	real_t get_max_engine_torque() const { return max_engine_torque; }

	// Engine drive (DRIVETRAIN_ENGINE). Engine speeds are RPM; gear ratios are positive, first gear to top.
	void set_engine_peak_torque(real_t p_v);
	real_t get_engine_peak_torque() const { return engine_peak_torque; }
	void set_engine_max_rpm(real_t p_v);
	real_t get_engine_max_rpm() const { return engine_max_rpm; }
	void set_engine_idle_rpm(real_t p_v);
	real_t get_engine_idle_rpm() const { return engine_idle_rpm; }
	// Torque (of engine_peak_torque) against engine speed (of engine_max_rpm), both 0..1. Null = flat.
	void set_engine_torque_curve(const Ref<Curve> &p_curve);
	Ref<Curve> get_engine_torque_curve() const { return engine_torque_curve; }
	void set_engine_moment_of_inertia(real_t p_v);
	real_t get_engine_moment_of_inertia() const { return engine_moment_of_inertia; }
	void set_engine_damping_full_throttle(real_t p_v);
	real_t get_engine_damping_full_throttle() const { return engine_damping_full_throttle; }
	void set_engine_damping_zero_throttle_clutch_engaged(real_t p_v);
	real_t get_engine_damping_zero_throttle_clutch_engaged() const { return engine_damping_zero_throttle_clutch_engaged; }
	void set_engine_damping_zero_throttle_clutch_disengaged(real_t p_v);
	real_t get_engine_damping_zero_throttle_clutch_disengaged() const { return engine_damping_zero_throttle_clutch_disengaged; }
	void set_gear_ratios(const PackedFloat32Array &p_ratios);
	PackedFloat32Array get_gear_ratios() const { return gear_ratios; }
	void set_reverse_gear_ratio(real_t p_v);
	real_t get_reverse_gear_ratio() const { return reverse_gear_ratio; }
	void set_final_drive_ratio(real_t p_v);
	real_t get_final_drive_ratio() const { return final_drive_ratio; }
	void set_gear_switch_time(real_t p_v);
	real_t get_gear_switch_time() const { return gear_switch_time; }
	void set_autobox_up_ratio(real_t p_v);
	real_t get_autobox_up_ratio() const { return autobox_up_ratio; }
	void set_autobox_down_ratio(real_t p_v);
	real_t get_autobox_down_ratio() const { return autobox_down_ratio; }
	void set_autobox_latency(real_t p_v);
	real_t get_autobox_latency() const { return autobox_latency; }
	void set_clutch_strength(real_t p_v);
	real_t get_clutch_strength() const { return clutch_strength; }
	void set_differential(Differential p_differential);
	Differential get_differential() const { return differential; }
	void set_limited_slip_front_bias(real_t p_v);
	real_t get_limited_slip_front_bias() const { return limited_slip_front_bias; }
	void set_limited_slip_front_target(real_t p_v);
	real_t get_limited_slip_front_target() const { return limited_slip_front_target; }
	void set_limited_slip_rear_bias(real_t p_v);
	real_t get_limited_slip_rear_bias() const { return limited_slip_rear_bias; }
	void set_limited_slip_rear_target(real_t p_v);
	real_t get_limited_slip_rear_target() const { return limited_slip_rear_target; }
	void set_limited_slip_center_bias(real_t p_v);
	real_t get_limited_slip_center_bias() const { return limited_slip_center_bias; }
	void set_limited_slip_center_target(real_t p_v);
	real_t get_limited_slip_center_target() const { return limited_slip_center_target; }
	void set_limited_slip_rate(real_t p_v);
	real_t get_limited_slip_rate() const { return limited_slip_rate; }
	void set_max_brake_torque(real_t p_v);
	real_t get_max_brake_torque() const { return max_brake_torque; }
	void set_max_steer_angle(real_t p_v);
	real_t get_max_steer_angle() const { return max_steer_angle; }
	void set_ackermann_strength(real_t p_v);
	real_t get_ackermann_strength() const { return ackermann_strength; }
	void set_front_anti_roll_stiffness(real_t p_v);
	real_t get_front_anti_roll_stiffness() const { return front_anti_roll_stiffness; }
	void set_rear_anti_roll_stiffness(real_t p_v);
	real_t get_rear_anti_roll_stiffness() const { return rear_anti_roll_stiffness; }

	// Runtime control inputs, same convention as PxVehicleCommandState: throttle/
	// brake in [0,1], steer in [-1,1]. Set every tick from script, same pattern
	// as VehicleBody3D.engine_force/brake/steering -- different units (PxVehicle2's
	// own normalized commands, not a raw force/raw angle), since that's what the
	// underlying SDK actually takes.
	void set_throttle(real_t p_v) { throttle = p_v; }
	real_t get_throttle() const { return throttle; }
	void set_brake(real_t p_v) { brake = p_v; }
	real_t get_brake() const { return brake; }
	// The rear wheel(s) only, on top of brake: a parking brake, or a slide.
	void set_handbrake(real_t p_v) { handbrake = p_v; }
	real_t get_handbrake() const { return handbrake; }
	void set_steer(real_t p_v) { steer = p_v; }
	real_t get_steer() const { return steer; }
	// Direct-drive has no gearbox, just a fixed forward/neutral/reverse
	// multiplier on throttle response (PxVehicleDirectDriveTransmissionCommandState) --
	// this is that switch, not a raw property on Vehicle4WConfig, since it's
	// a per-tick control input like throttle/brake/steer, not a build-time
	// tuning value.
	void set_reverse(bool p_v) { reverse = p_v; }
	bool is_reverse() const { return reverse; }
	// Engine drive controls, read every tick. automatic_transmission: the autobox picks forward gears (reverse
	// still selects the reverse gear); off, the gearbox goes to `gear` (-1 reverse, 0 neutral, 1.. forward), and
	// reverse = true still means the reverse gear. clutch: 0 engaged .. 1 pressed. clutch_assist off: a manual
	// gearbox changes gear (reverse too) only while the clutch is pressed past halfway, and otherwise holds its gear.
	void set_automatic_transmission(bool p_v) { automatic_transmission = p_v; }
	bool is_automatic_transmission() const { return automatic_transmission; }
	void set_gear(int p_gear) { gear = p_gear; }
	int get_gear() const { return gear; }
	void set_clutch(real_t p_v) { clutch = p_v; }
	real_t get_clutch() const { return clutch; }
	void set_clutch_assist(bool p_v) { clutch_assist = p_v; }
	bool is_clutch_assist() const { return clutch_assist; }
	// The gear engaged now, numbered as `gear` (0 while a shift is under way). Direct drive: 1 or -1.
	int get_current_gear() const;
	// The gear the gearbox is shifting to (the current one when not shifting), numbered as `gear`.
	int get_target_gear() const;
	// Engine speed in RPM; 0 with direct drive.
	real_t get_engine_rpm() const;

	void set_collision_layer(uint32_t p_layer);
	uint32_t get_collision_layer() const { return collision_layer; }
	void set_collision_mask(uint32_t p_mask);
	uint32_t get_collision_mask() const { return collision_mask; }

	// The chassis box's contact material (friction / rough / bounce / absorbent); null = friction 0.5, bounce 0.1.
	// Edited live in place, no rebuild. The wheels' grip is the tire model's, not this.
	void set_physics_material_override(const Ref<PhysicsMaterial> &p_material);
	Ref<PhysicsMaterial> get_physics_material_override() const { return physics_material_override; }

	Vector3 get_linear_velocity() const;
	PHYSX_VEHICLE_CONTROL_DECLARE()
	real_t get_forward_speed() const;
	// World-space forward: the chassis's local +Z, as VehicleBody3D (Vector3.MODEL_FRONT).
	Vector3 get_forward() const;

	// Diagnostics -- see GodotPhysXVehicleProbe's own identical methods for
	// what these mean. wheel index here matches this node's own `wheels`
	// child-registration order (NOT the FL/FR/RL/RR canonical order the
	// underlying Vehicle4W uses internally).
	real_t get_wheel_jounce(int p_wheel) const;
	real_t get_wheel_separation(int p_wheel) const;
	Vector3 get_actor_position() const;

	PackedStringArray get_configuration_warnings() const override;

	PhysXVehicle3D();
	~PhysXVehicle3D();

private:
	real_t mass = 1500.0f;
	Vector3 moment_of_inertia = Vector3(2000.0f, 2200.0f, 1000.0f);
	CenterOfMassMode center_of_mass_mode = CENTER_OF_MASS_MODE_AUTO;
	Vector3 center_of_mass;
	bool can_sleep = true;
	Drivetrain drivetrain = DRIVETRAIN_DIRECT;
	real_t max_engine_torque = 700.0f;
	// Engine drive defaults: see Vehicle4WEngineConfig.
	real_t engine_peak_torque = 500.0f;
	real_t engine_max_rpm = 6000.0f;
	real_t engine_idle_rpm = 0.0f;
	Ref<Curve> engine_torque_curve;
	real_t engine_moment_of_inertia = 1.0f;
	real_t engine_damping_full_throttle = 0.15f;
	real_t engine_damping_zero_throttle_clutch_engaged = 2.0f;
	real_t engine_damping_zero_throttle_clutch_disengaged = 0.35f;
	PackedFloat32Array gear_ratios = { 4.0f, 2.0f, 1.5f, 1.1f, 1.0f };
	real_t reverse_gear_ratio = 4.0f;
	real_t final_drive_ratio = 4.0f;
	real_t gear_switch_time = 0.5f;
	real_t autobox_up_ratio = 0.65f;
	real_t autobox_down_ratio = 0.5f;
	real_t autobox_latency = 2.0f;
	real_t clutch_strength = 10.0f;
	Differential differential = DIFFERENTIAL_OPEN;
	real_t limited_slip_front_bias = 1.3f;
	real_t limited_slip_front_target = 1.29f;
	real_t limited_slip_rear_bias = 1.3f;
	real_t limited_slip_rear_target = 1.29f;
	real_t limited_slip_center_bias = 1.3f;
	real_t limited_slip_center_target = 1.29f;
	real_t limited_slip_rate = 10.0f;
	real_t max_brake_torque = 6000.0f;
	real_t max_steer_angle = 0.6f;
	real_t ackermann_strength = 1.0f;
	real_t front_anti_roll_stiffness = 0.0f;
	real_t rear_anti_roll_stiffness = 0.0f;

	real_t throttle = 0.0f;
	real_t brake = 0.0f;
	real_t handbrake = 0.0f;
	real_t steer = 0.0f;
	bool reverse = false;
	bool automatic_transmission = true;
	int gear = 1;
	real_t clutch = 0.0f;
	bool clutch_assist = true;
	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;
	Ref<PhysicsMaterial> physics_material_override;

	// Opaque pointer to the real PxVehicle2 composition (kept out of this
	// header so nothing outside physx_vehicle_3d.cpp needs vehicle/PxVehicleAPI.h).
	struct Impl;
	Impl *impl = nullptr;

	// Any exported-property setter, or a child PhysXVehicleWheel3D's own
	// property setter, calls this if the vehicle is already built (editing in
	// the Inspector while Playing) -- full rebuild, same "simple; optimize
	// later" convention GodotPhysXBody3D's own shape/mode-change path already
	// uses. No-op if not yet built.
	void _rebuild_if_live();
	void _chassis_material(real_t &r_friction, real_t &r_bounce) const;
	void _chassis_material_changed();
	bool _build();
	void _destroy();
};

VARIANT_ENUM_CAST(PhysXVehicle3D::CenterOfMassMode);
VARIANT_ENUM_CAST(PhysXVehicle3D::Drivetrain);
VARIANT_ENUM_CAST(PhysXVehicle3D::Differential);

/**************************************************************************/
/*  physx_vehicle_3d.h                                                    */
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

#pragma once

#include "core/math/vector3.h"
#include "core/templates/local_vector.h"
#include "scene/3d/node_3d.h"

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

	void set_max_engine_torque(real_t p_v);
	real_t get_max_engine_torque() const { return max_engine_torque; }
	void set_max_brake_torque(real_t p_v);
	real_t get_max_brake_torque() const { return max_brake_torque; }
	void set_max_steer_angle(real_t p_v);
	real_t get_max_steer_angle() const { return max_steer_angle; }
	void set_ackermann_strength(real_t p_v);
	real_t get_ackermann_strength() const { return ackermann_strength; }

	// Runtime control inputs, same convention as PxVehicleCommandState: throttle/
	// brake in [0,1], steer in [-1,1]. Set every tick from script, same pattern
	// as VehicleBody3D.engine_force/brake/steering -- different units (PxVehicle2's
	// own normalized commands, not a raw force/raw angle), since that's what the
	// underlying SDK actually takes.
	void set_throttle(real_t p_v) { throttle = p_v; }
	real_t get_throttle() const { return throttle; }
	void set_brake(real_t p_v) { brake = p_v; }
	real_t get_brake() const { return brake; }
	void set_steer(real_t p_v) { steer = p_v; }
	real_t get_steer() const { return steer; }
	// Direct-drive has no gearbox, just a fixed forward/neutral/reverse
	// multiplier on throttle response (PxVehicleDirectDriveTransmissionCommandState) --
	// this is that switch, not a raw property on Vehicle4WConfig, since it's
	// a per-tick control input like throttle/brake/steer, not a build-time
	// tuning value.
	void set_reverse(bool p_v) { reverse = p_v; }
	bool is_reverse() const { return reverse; }

	void set_collision_layer(uint32_t p_layer);
	uint32_t get_collision_layer() const { return collision_layer; }
	void set_collision_mask(uint32_t p_mask);
	uint32_t get_collision_mask() const { return collision_mask; }

	Vector3 get_linear_velocity() const;
	real_t get_forward_speed() const;

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
	real_t max_engine_torque = 700.0f;
	real_t max_brake_torque = 6000.0f;
	real_t max_steer_angle = 0.6f;
	real_t ackermann_strength = 1.0f;

	real_t throttle = 0.0f;
	real_t brake = 0.0f;
	real_t steer = 0.0f;
	bool reverse = false;
	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;

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
	bool _build();
	void _destroy();
};

VARIANT_ENUM_CAST(PhysXVehicle3D::CenterOfMassMode);

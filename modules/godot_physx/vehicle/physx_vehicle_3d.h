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
#include "scene/3d/node_3d.h"

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
// Owns its own PxRigidDynamic directly (via configure_vehicle4w), not a
// GodotPhysXBody3D -- PxVehicle2's own PxVehiclePhysXActorEndComponent writes
// wheel-shape local poses and rigid-body momentum straight onto the actor
// every tick, which needs raw actor/shape pointers, not the generic
// PhysicsServer3D RID abstraction.
class PhysXVehicle3D : public Node3D {
	GDCLASS(PhysXVehicle3D, Node3D);

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	void set_mass(real_t p_mass);
	real_t get_mass() const { return mass; }
	void set_moment_of_inertia(const Vector3 &p_moi);
	Vector3 get_moment_of_inertia() const { return moment_of_inertia; }

	void set_half_track(real_t p_v);
	real_t get_half_track() const { return half_track; }
	void set_front_axle_z(real_t p_v);
	real_t get_front_axle_z() const { return front_axle_z; }
	void set_rear_axle_z(real_t p_v);
	real_t get_rear_axle_z() const { return rear_axle_z; }

	void set_wheel_radius(real_t p_v);
	real_t get_wheel_radius() const { return wheel_radius; }
	void set_wheel_half_width(real_t p_v);
	real_t get_wheel_half_width() const { return wheel_half_width; }
	void set_wheel_mass(real_t p_v);
	real_t get_wheel_mass() const { return wheel_mass; }
	void set_wheel_moment_of_inertia(real_t p_v);
	real_t get_wheel_moment_of_inertia() const { return wheel_moment_of_inertia; }
	void set_wheel_damping_rate(real_t p_v);
	real_t get_wheel_damping_rate() const { return wheel_damping_rate; }

	void set_suspension_travel(real_t p_v);
	real_t get_suspension_travel() const { return suspension_travel; }
	void set_suspension_stiffness(real_t p_v);
	real_t get_suspension_stiffness() const { return suspension_stiffness; }
	void set_suspension_damping(real_t p_v);
	real_t get_suspension_damping() const { return suspension_damping; }

	void set_tire_lateral_stiffness(real_t p_v);
	real_t get_tire_lateral_stiffness() const { return tire_lateral_stiffness; }
	void set_tire_longitudinal_stiffness(real_t p_v);
	real_t get_tire_longitudinal_stiffness() const { return tire_longitudinal_stiffness; }
	void set_tire_friction(real_t p_v);
	real_t get_tire_friction() const { return tire_friction; }

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

	Vector3 get_linear_velocity() const;
	real_t get_forward_speed() const;

	PhysXVehicle3D();
	~PhysXVehicle3D();

private:
	real_t mass = 1500.0f;
	Vector3 moment_of_inertia = Vector3(2000.0f, 2200.0f, 1000.0f);
	real_t half_track = 0.75f;
	real_t front_axle_z = 1.35f;
	real_t rear_axle_z = -1.35f;
	real_t wheel_radius = 0.35f;
	real_t wheel_half_width = 0.15f;
	real_t wheel_mass = 20.0f;
	real_t wheel_moment_of_inertia = 1.2f;
	real_t wheel_damping_rate = 0.25f;
	real_t suspension_travel = 0.15f;
	real_t suspension_stiffness = 35000.0f;
	real_t suspension_damping = 4500.0f;
	real_t tire_lateral_stiffness = 20000.0f;
	real_t tire_longitudinal_stiffness = 20000.0f;
	real_t tire_friction = 1.0f;
	real_t max_engine_torque = 700.0f;
	real_t max_brake_torque = 6000.0f;
	real_t max_steer_angle = 0.6f;
	real_t ackermann_strength = 1.0f;

	real_t throttle = 0.0f;
	real_t brake = 0.0f;
	real_t steer = 0.0f;

	// Opaque pointer to the real PxVehicle2 composition (kept out of this
	// header so nothing outside physx_vehicle_3d.cpp needs vehicle/PxVehicleAPI.h).
	struct Impl;
	Impl *impl = nullptr;

	// Any exported-property setter calls this if the vehicle is already built
	// (editing in the Inspector while Playing) -- full rebuild, same
	// "simple; optimize later" convention GodotPhysXBody3D's own
	// shape/mode-change path already uses. No-op if not yet built.
	void _rebuild_if_live();
	bool _build();
	void _destroy();
};

/**************************************************************************/
/*  physx_water_spray_3d.h                                                */
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

#include "scene/3d/node_3d.h"

class GPUParticles3D;
class ParticleProcessMaterial;
class PhysXWaterSurface3D;
class RigidBody3D;

// Spray thrown up by a hull moving through a PhysXWaterSurface3D, drawn by
// ordinary GPUParticles3D nodes authored in the editor -- this node only
// drives them from the body's motion. Make it a child of the RigidBody3D
// (a boat) and point its emitter lists at GPUParticles3D nodes:
// - bow_emitters spray while the hull moves through the water: rate and
//   launch speed rise with forward speed, none where the hull is out of the
//   water.
// - stern_emitters spray with a sibling PhysXBoat3D's propeller thrust.
// - slam_emitters fire once (restart) when the hull drops back onto the
//   water fast enough, scaled with the impact.
// Every tick the emitters are moved to the water surface under where they
// were placed on the hull, level and turned with the hull's heading, so the
// spray leaves the water there whatever the hull's pitch and roll.
class PhysXWaterSpray3D : public Node3D {
	GDCLASS(PhysXWaterSpray3D, Node3D);

public:
	enum Role {
		ROLE_BOW,
		ROLE_STERN,
		ROLE_SLAM,
	};

private:
	struct Emitter {
		ObjectID id;
		Role role = ROLE_BOW;
		Transform3D rest; // as placed, relative to the body
		Ref<ParticleProcessMaterial> material; // this emitter's own copy
		float base_velocity_min = 0.0f;
		float base_velocity_max = 0.0f;
		float base_ratio = 1.0f;
		float prev_gap = 0.0f; // hull point height above the water, last tick
		float prev_water = 0.0f;
		bool have_prev = false;
		float cooldown = 0.0f;
	};

	NodePath water_surface_path; // empty = the sibling PhysXBuoyancy3D's
	TypedArray<NodePath> bow_emitters;
	TypedArray<NodePath> stern_emitters;
	TypedArray<NodePath> slam_emitters;
	float start_speed = 1.5f;
	float reference_speed = 6.0f;
	float slow_velocity_scale = 0.4f;
	float submerge_depth = 0.25f;
	float slam_speed = 1.5f;
	float slam_full_speed = 4.0f;
	float slam_cooldown = 0.3f;

	Vector<Emitter> emitters;
	ObjectID water_id;
	ObjectID wake_id; // the sibling PhysXWaterWake3D: the hull's own wake isn't water it clears
	bool built = false;

	void _build();
	void _clear();
	void _physics_step(double p_delta);
	void _add_emitters(const TypedArray<NodePath> &p_paths, Role p_role);

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	RigidBody3D *get_body() const;
	PhysXWaterSurface3D *get_water_surface() const;

	void set_water_surface_path(const NodePath &p_path);
	NodePath get_water_surface_path() const { return water_surface_path; }
	void set_bow_emitters(const TypedArray<NodePath> &p_paths);
	TypedArray<NodePath> get_bow_emitters() const { return bow_emitters; }
	void set_stern_emitters(const TypedArray<NodePath> &p_paths);
	TypedArray<NodePath> get_stern_emitters() const { return stern_emitters; }
	void set_slam_emitters(const TypedArray<NodePath> &p_paths);
	TypedArray<NodePath> get_slam_emitters() const { return slam_emitters; }
	void set_start_speed(float p_speed) { start_speed = MAX(p_speed, 0.0f); }
	float get_start_speed() const { return start_speed; }
	void set_reference_speed(float p_speed) { reference_speed = MAX(p_speed, 0.01f); }
	float get_reference_speed() const { return reference_speed; }
	void set_slow_velocity_scale(float p_scale) { slow_velocity_scale = CLAMP(p_scale, 0.0f, 1.0f); }
	float get_slow_velocity_scale() const { return slow_velocity_scale; }
	void set_submerge_depth(float p_depth) { submerge_depth = MAX(p_depth, 0.01f); }
	float get_submerge_depth() const { return submerge_depth; }
	void set_slam_speed(float p_speed) { slam_speed = MAX(p_speed, 0.0f); }
	float get_slam_speed() const { return slam_speed; }
	void set_slam_full_speed(float p_speed) { slam_full_speed = MAX(p_speed, 0.01f); }
	float get_slam_full_speed() const { return slam_full_speed; }
	void set_slam_cooldown(float p_seconds) { slam_cooldown = MAX(p_seconds, 0.0f); }
	float get_slam_cooldown() const { return slam_cooldown; }

	PackedStringArray get_configuration_warnings() const override;
};

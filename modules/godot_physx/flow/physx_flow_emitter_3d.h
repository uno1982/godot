/**************************************************************************/
/*  physx_flow_emitter_3d.h                                               */
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

class PhysXFlow3D;

// A source of smoke / heat / fuel for a PhysXFlow3D, or -- with collision on --
// a solid the gas flows around. It feeds the PhysXFlow3D its `flow` path
// points at, or with none set, the scene's only one -- so it can live
// anywhere, e.g. inside a vehicle scene by its wheels. Its own motion is what
// a collider pushes into the gas.
class PhysXFlowEmitter3D : public Node3D {
	GDCLASS(PhysXFlowEmitter3D, Node3D);

public:
	enum Shape {
		SHAPE_SPHERE,
		SHAPE_BOX,
	};

	static constexpr const char *GROUP = "physx_flow_emitters";

private:
	bool enabled = true;
	Shape shape = SHAPE_SPHERE;
	float radius = 0.5f;
	Vector3 size = Vector3(1, 1, 1);
	Vector3 velocity = Vector3(0, 2, 0);
	float temperature = 1.0f;
	float fuel = 0.0f;
	float burn = 0.0f;
	float smoke = 1.0f;
	float divergence = 0.0f;
	float couple_rate = 2.0f;
	bool collision = false;
	NodePath flow_path;

	// For collision: the body's velocity, from this node's own motion.
	Vector3 last_position;
	Vector3 motion_velocity;
	bool has_last_position = false;

protected:
	void _notification(int p_what);
	void _validate_property(PropertyInfo &p_property) const;
	static void _bind_methods();

public:
	void set_enabled(bool p_enabled) { enabled = p_enabled; }
	bool is_enabled() const { return enabled; }
	void set_shape(Shape p_shape);
	Shape get_shape() const { return shape; }
	void set_radius(float p_radius);
	float get_radius() const { return radius; }
	void set_size(const Vector3 &p_size);
	Vector3 get_size() const { return size; }
	void set_velocity(const Vector3 &p_velocity) { velocity = p_velocity; }
	Vector3 get_velocity() const { return velocity; }
	void set_temperature(float p_v) { temperature = p_v; }
	float get_temperature() const { return temperature; }
	void set_fuel(float p_v) { fuel = MAX(p_v, 0.0f); }
	float get_fuel() const { return fuel; }
	void set_burn(float p_v) { burn = MAX(p_v, 0.0f); }
	float get_burn() const { return burn; }
	void set_smoke(float p_v) { smoke = MAX(p_v, 0.0f); }
	float get_smoke() const { return smoke; }
	void set_divergence(float p_v) { divergence = p_v; }
	float get_divergence() const { return divergence; }
	void set_couple_rate(float p_v) { couple_rate = MAX(p_v, 0.0f); }
	float get_couple_rate() const { return couple_rate; }
	void set_collision(bool p_collision);
	bool is_collision() const { return collision; }

	// World-space velocity of this node over the last physics step.
	Vector3 get_motion_velocity() const { return motion_velocity; }

	void set_flow_path(const NodePath &p_path);
	NodePath get_flow_path() const { return flow_path; }

	// The PhysXFlow3D this feeds: flow_path's, or with it empty, the scene's
	// only one; null if neither applies.
	PhysXFlow3D *get_flow() const;

	PackedStringArray get_configuration_warnings() const override;
};

VARIANT_ENUM_CAST(PhysXFlowEmitter3D::Shape);

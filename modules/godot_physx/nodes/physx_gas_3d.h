/**************************************************************************/
/*  physx_gas_3d.h                                                        */
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

#include "core/variant/typed_array.h"
#include "scene/3d/node_3d.h"

class GasSolver;
class Mesh;

// A sparse-grid Eulerian smoke/gas volume (velocity + density field on a
// block-hash sparse grid -- see particles/gas_block_inc.glsl for the full
// design writeup and particles/gas_solver.h for the solver). A validated
// proof of concept (see the PhysX module backlog memory), ported into the
// module as a real node for the first time -- still v1 scope: the domain's
// lateral (X/Z) footprint is fixed at first configure (grows automatically
// upward as a plume rises, see GasSolver::_grow_if_needed, but not
// sideways), no pressure projection, and up to 4 sphere colliders.
//
// Renders as a debug point cloud (small cubes, one per grid cell above a
// density threshold, colour ramped by density) via a manually-managed
// MultiMesh RS instance in world space -- not a proper volumetric render.
// That is the next obvious follow-up once the physics milestones settle.
class PhysXGas3D : public Node3D {
	GDCLASS(PhysXGas3D, Node3D);

	GasSolver *solver = nullptr;

	Vector3 domain_size = Vector3(1.92f, 3.2f, 1.92f); // world size of the fixed box (rounded to whole 4-cell blocks)
	float cell_size = 0.08f;
	Vector3 source_position;
	float source_radius = 0.22f;
	Vector3 source_velocity = Vector3(0, 1.4f, 0);
	float source_density = 1.0f;
	float buoyancy = 9.0f;
	float vorticity_strength = 8.0f;
	float dissipation = 0.996f;

	// Up to GasSolver::MAX_COLLIDERS Node3D paths; each one's global position
	// drives a sphere obstacle. A CollisionShape3D (or CollisionObject3D)
	// carrying a SphereShape3D gets its own world-scaled radius (see
	// _resolve_collider_radius); anything else falls back to collider_radius.
	TypedArray<NodePath> colliders;
	float collider_radius = 0.0f;
	float _resolve_collider_radius(Node3D *p_node) const;

	float render_threshold = 0.01f;
	Ref<Mesh> cell_mesh;
	RID multimesh;
	RID mm_instance;
	bool configured = false;

	void _ensure_configured();
	void _step(double p_delta);
	void _update_render();

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void set_domain_size(const Vector3 &p_size);
	Vector3 get_domain_size() const { return domain_size; }
	void set_cell_size(float p_size);
	float get_cell_size() const { return cell_size; }

	void set_source_position(const Vector3 &p_pos) { source_position = p_pos; }
	Vector3 get_source_position() const { return source_position; }
	void set_source_radius(float p_r) { source_radius = MAX(p_r, 0.001f); }
	float get_source_radius() const { return source_radius; }
	void set_source_velocity(const Vector3 &p_v) { source_velocity = p_v; }
	Vector3 get_source_velocity() const { return source_velocity; }
	void set_source_density(float p_d) { source_density = MAX(p_d, 0.0f); }
	float get_source_density() const { return source_density; }

	void set_buoyancy(float p_b) { buoyancy = p_b; }
	float get_buoyancy() const { return buoyancy; }
	void set_vorticity_strength(float p_v) { vorticity_strength = MAX(p_v, 0.0f); }
	float get_vorticity_strength() const { return vorticity_strength; }
	void set_dissipation(float p_d) { dissipation = CLAMP(p_d, 0.0f, 1.0f); }
	float get_dissipation() const { return dissipation; }

	void set_colliders(const TypedArray<NodePath> &p_colliders) { colliders = p_colliders; }
	TypedArray<NodePath> get_colliders() const { return colliders; }
	void set_collider_radius(float p_r) { collider_radius = MAX(p_r, 0.0f); }
	float get_collider_radius() const { return collider_radius; }

	PhysXGas3D() {}
	~PhysXGas3D();
};

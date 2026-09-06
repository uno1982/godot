/**************************************************************************/
/*  godot_physx_soft_body_3d.h                                            */
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

#include "../cloth/xpbd_cloth_solver.h"

#include "core/math/aabb.h"
#include "core/math/transform_3d.h"
#include "core/templates/hash_set.h"
#include "core/templates/local_vector.h"
#include "core/templates/rid.h"

class GodotPhysXSpace3D;
class PhysicsServer3DRenderingServerHandler;

// CPU soft body backing the stock SoftBody3D node on the PhysX backend. It runs
// the module's XPBD solver over the render mesh (welded to unique positions):
// edge constraints hold the shape, an optional volume constraint (pressure)
// keeps a closed mesh from collapsing, and a per-vertex query against the PhysX
// scene pushes vertices out of rigid bodies each step. No PhysX deformable
// actor is created -- PxDeformableVolume (GPU FEM) is a separate, future path.
class GodotPhysXSoftBody3D {
	RID self;
	ObjectID instance_id;
	GodotPhysXSpace3D *space = nullptr;

	XPBDClothSolver solver;

	RID mesh;
	LocalVector<uint32_t> map_visual_to_physics; // render vertex -> solver vertex
	uint32_t visual_vertex_count = 0;
	HashSet<int> pinned_render_points; // survives set_mesh, like the other backends
	LocalVector<Vector3> normals; // per solver vertex, world space
	AABB bounds;
	bool mesh_ready = false;

	Transform3D transform; // initial placement; sim then runs in world space
	bool placed = false; // set once the node hands us its world transform

	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;
	HashSet<RID> collision_exceptions;
	bool ray_pickable = true;

	// Stock SoftBody3D properties, kept as given and mapped onto the solver.
	int simulation_precision = 5;
	real_t total_mass = 1.0;
	real_t linear_stiffness = 0.5;
	real_t shrinking_factor = 0.0;
	real_t pressure_coefficient = 0.0;
	real_t damping_coefficient = 0.01;
	real_t drag_coefficient = 0.0;

	RID collision_sphere; // lazily created shape for the per-vertex query
	// Per-vertex world-contact cache, refreshed once per frame and re-projected
	// against every substep (anti-tunnelling).
	LocalVector<Vector3> contact_n;
	LocalVector<Vector3> contact_p;
	LocalVector<uint8_t> contact_hit;
	int contact_count = 0; // penetrating vertices at the last refresh

	void _apply_solver_settings();
	void _rebuild_from_mesh();
	void _refresh_contacts();
	void _resolve_contacts();
	void _damp_rigid_drift();
	void _update_normals_and_bounds();

public:
	GodotPhysXSoftBody3D() {}
	~GodotPhysXSoftBody3D();

	void set_self(const RID &p_self) { self = p_self; }
	RID get_self() const { return self; }

	void set_instance_id(ObjectID p_id) { instance_id = p_id; }
	ObjectID get_instance_id() const { return instance_id; }

	void set_space(GodotPhysXSpace3D *p_space);
	GodotPhysXSpace3D *get_space() const { return space; }

	void set_mesh(RID p_mesh);
	RID get_mesh() const { return mesh; }
	bool is_ready() const { return mesh_ready; }
	AABB get_bounds() const { return bounds; }

	void set_transform(const Transform3D &p_transform);

	void set_collision_layer(uint32_t p_layer) { collision_layer = p_layer; }
	uint32_t get_collision_layer() const { return collision_layer; }
	void set_collision_mask(uint32_t p_mask) { collision_mask = p_mask; }
	uint32_t get_collision_mask() const { return collision_mask; }
	void add_collision_exception(const RID &p_body) { collision_exceptions.insert(p_body); }
	void remove_collision_exception(const RID &p_body) { collision_exceptions.erase(p_body); }
	const HashSet<RID> &get_collision_exceptions() const { return collision_exceptions; }

	void set_ray_pickable(bool p_enable) { ray_pickable = p_enable; }

	void set_simulation_precision(int p_precision);
	int get_simulation_precision() const { return simulation_precision; }
	void set_total_mass(real_t p_mass);
	real_t get_total_mass() const { return total_mass; }
	void set_linear_stiffness(real_t p_stiffness);
	real_t get_linear_stiffness() const { return linear_stiffness; }
	void set_shrinking_factor(real_t p_factor);
	real_t get_shrinking_factor() const { return shrinking_factor; }
	void set_pressure_coefficient(real_t p_pressure);
	real_t get_pressure_coefficient() const { return pressure_coefficient; }
	void set_damping_coefficient(real_t p_damping);
	real_t get_damping_coefficient() const { return damping_coefficient; }
	void set_drag_coefficient(real_t p_drag);
	real_t get_drag_coefficient() const { return drag_coefficient; }

	// Point / pin operations, indexed by *render* vertex.
	void move_point(int p_point_index, const Vector3 &p_global_position);
	Vector3 get_point_global_position(int p_point_index) const;
	void pin_point(int p_point_index, bool p_pin);
	bool is_point_pinned(int p_point_index) const;
	void unpin_all();
	void apply_point_impulse(int p_point_index, const Vector3 &p_impulse);
	void apply_point_force(int p_point_index, const Vector3 &p_force, double p_delta);
	void apply_central_impulse(const Vector3 &p_impulse);
	void apply_central_force(const Vector3 &p_force, double p_delta);

	// Driven by the space each step.
	void step(double p_delta, const Vector3 &p_gravity);
	// Feed deformed positions/normals to the render mesh.
	void update_rendering_server(PhysicsServer3DRenderingServerHandler *p_handler);
};

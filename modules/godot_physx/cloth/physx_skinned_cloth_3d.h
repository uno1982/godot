/**************************************************************************/
/*  physx_skinned_cloth_3d.h                                              */
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

#include "skinned_cloth_solver.h"

#include "scene/3d/node_3d.h"
#include "scene/resources/material.h"

class MeshInstance3D;
class Skeleton3D;
class ShaderMaterial;
class Texture2DRD;

// Cloth on a region of a character's skinned mesh -- a robe, cape or skirt
// that's part of the character, not a separate pinned sheet. Each vertex has
// a max distance it may stray from where the animation puts it: 0 follows the
// animation exactly (shoulders, waistband), larger values swing freely (hem,
// cape tail). The mesh is simulated on the GPU with compute shaders (any GPU)
// and collides with capsules on the character's bones. The source mesh is
// hidden and drawn instead from the simulated positions.
class PhysXSkinnedCloth3D : public Node3D {
	GDCLASS(PhysXSkinnedCloth3D, Node3D);

	NodePath mesh_instance_path;
	NodePath body_mesh_path; // optional: fit the body capsules to this skinned mesh
	int surface = 0;
	PackedFloat32Array max_distances; // per vertex of the surface; empty = auto from height
	float pin_height = 1.3f; // auto paint: at/above this bind-space height the cloth follows the animation
	float max_distance = 0.4f; // auto paint: max distance at the lowest point, ramping up from pin_height
	bool use_vertex_color = false; // unpainted: the mesh's vertex colour red channel x max_distance
	int substeps = 8;
	float stiffness = 0.9f;
	float bend_stiffness = 0.3f;
	float damping = 0.05f;
	float thickness = 0.012f;
	float backstop = 0.02f;
	float animation_drive = 0.0f;
	// Gravity: the world's (this node's physics space default gravity — the project setting, or a space override)
	// unless use_world_gravity is off (then fixed_gravity), plus extra_acceleration (the game's force fields, wind …).
	bool use_world_gravity = true;
	Vector3 fixed_gravity = Vector3(0, -9.8f, 0);
	Vector3 extra_acceleration;
	bool self_collision = false;
	float self_collision_thickness = 0.02f;
	float collision_radius_scale = 1.0f;
	bool collide_with_body = true;
	bool simulating = true;

	SkinnedClothSolver *solver = nullptr;
	MeshInstance3D *render_instance = nullptr;
	ObjectID render_instance_id;
	Ref<ShaderMaterial> render_material;
	Ref<Texture2DRD> pos_texture;
	Ref<Texture2DRD> nrm_texture;
	ObjectID source_id;
	// How the source's original surface was hidden: the whole node (a
	// one-surface mesh), or just that surface, with an override material that
	// draws nothing (the user's own override kept to put back).
	bool hid_source_node = false;
	int hidden_surface = -1;
	Ref<Material> saved_surface_override;
	static Ref<ShaderMaterial> hide_material;
	// The render copy's shader, shared by every cloth: a new one each build was compiled again (~17 ms a build).
	static Ref<Shader> render_shader;
	PackedInt32Array vertex_particles; // render copy vertex -> particle (its CUSTOM0.x), as built
	void _hide_source(MeshInstance3D *p_source);
	void _unhide_source(MeshInstance3D *p_source);

public:
	static void free_shared();

private:
	ObjectID skeleton_id;
	Vector<int> bind_bones; // skeleton bone index per skin bind
	Vector<Transform3D> bind_poses;
	struct Capsule {
		// Automatic: between two bones' heads. From a physical bone: a
		// capsule (or sphere) shape fixed to one bone.
		int bone_a = -1;
		int bone_b = -1;
		float radius_a = 0.05f;
		float radius_b = 0.05f;
		bool from_shape = false;
		Transform3D shape_local; // bone space: body_offset * shape transform
		float half_length = 0.0f; // along the shape's local Y
		// From a physical bone: the nodes, re-read every frame (sizes, offsets
		// and disabling apply live), and whether it's a box.
		ObjectID body_id;
		ObjectID shape_id;
		bool is_box = false;
		Vector3 half_extents; // box, in its own (unscaled) space
		bool active = true;
		Transform3D bone_pose; // bone_a's pose (skeleton space) at the last capture
		Vector3 end_pose; // automatic capsules: bone_b's head then
	};
	bool using_physical_bones = false;
	Vector<Capsule> capsules;
	PackedFloat32Array prev_capsules; // last frame's capsule ends, for blending across substeps
	PackedFloat32Array prev_boxes; // last frame's boxes, likewise
	int particle_count = 0;
	bool built = false;
	bool build_pending = false;

	void _try_build();
	void _clear();
	void _find_capsules(Skeleton3D *p_skeleton);
	bool _find_physical_bone_shapes(Skeleton3D *p_skeleton);
	static bool _read_shape(Capsule &r_c);
	void _pack_colliders();
	void _fit_capsules(Skeleton3D *p_skeleton, MeshInstance3D *p_body);
	void _step();
	// The skeleton's FINAL pose — after its modifiers (IK, spring bones, ...) — exists only while the skeleton
	// updates: it emits skeleton_updated, skins, then restores the unmodified poses. So the pose is captured in
	// skeleton_updated (and the cloth steps right there); a frame with no skeleton update reuses the last capture,
	// placed where the skeleton is then (the pose holds still, the character may not).
	void _capture_pose(Skeleton3D *p_skeleton);
	void _place_capture(Skeleton3D *p_skeleton);
	void _late_step();
	void _advance(double p_delta);
	void _on_skeleton_updated();
	void _connect_skeleton(Skeleton3D *p_skeleton);
	void _disconnect_skeleton();
	ObjectID connected_skeleton_id;
	Vector<Transform3D> captured_poses; // per skin bind: global pose * bind pose (skeleton space)
	Transform3D skeleton_xform; // where the skeleton is drawn, as last placed
	PackedFloat32Array captured_bones; // per skin bind: skeleton_xform * global pose * bind pose (mat4)
	PackedFloat32Array captured_capsules; // per capsule: a.xyz ra, b.xyz rb (world)
	PackedFloat32Array captured_boxes; // per box: centre.xyz active, half-axes x, y, z (world)
	bool has_capture = false;
	uint64_t last_step_frame = 0;
	// Game time (the process delta, so Engine.time_scale and pausing apply) not yet simulated; a step consumes it.
	double pending_delta = 0.0;

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void set_mesh_instance_path(const NodePath &p_path);
	NodePath get_mesh_instance_path() const { return mesh_instance_path; }
	void set_body_mesh_path(const NodePath &p_path);
	NodePath get_body_mesh_path() const { return body_mesh_path; }
	void set_surface(int p_surface);
	int get_surface() const { return surface; }
	void set_max_distances(const PackedFloat32Array &p_distances);
	PackedFloat32Array get_max_distances() const { return max_distances; }
	void set_pin_height(float p_height);
	float get_pin_height() const { return pin_height; }
	void set_max_distance(float p_distance);
	float get_max_distance() const { return max_distance; }
	void set_use_vertex_color(bool p_enabled);
	bool get_use_vertex_color() const { return use_vertex_color; }

	// The source MeshInstance3D / surface arrays, and the max distance each of
	// its vertices gets: painted (max_distances), else the vertex colour red
	// channel (use_vertex_color), else the height ramp. Also used by the
	// editor's paint tool.
	MeshInstance3D *get_source_mesh_instance() const;
	PackedFloat32Array get_effective_max_distances() const;
	PackedFloat32Array compute_height_ramp() const;
	void set_substeps(int p_substeps);
	int get_substeps() const { return substeps; }
	void set_stiffness(float p_stiffness);
	float get_stiffness() const { return stiffness; }
	void set_bend_stiffness(float p_stiffness);
	float get_bend_stiffness() const { return bend_stiffness; }
	void set_damping(float p_damping);
	float get_damping() const { return damping; }
	void set_thickness(float p_thickness);
	float get_thickness() const { return thickness; }
	void set_backstop(float p_distance);
	float get_backstop() const { return backstop; }
	void set_self_collision(bool p_enabled);
	bool get_self_collision() const { return self_collision; }
	void set_self_collision_thickness(float p_thickness);
	float get_self_collision_thickness() const { return self_collision_thickness; }
	void set_animation_drive(float p_drive);
	float get_animation_drive() const { return animation_drive; }
	void set_use_world_gravity(bool p_on) { use_world_gravity = p_on; }
	bool get_use_world_gravity() const { return use_world_gravity; }
	void set_fixed_gravity(const Vector3 &p_g) { fixed_gravity = p_g; }
	Vector3 get_fixed_gravity() const { return fixed_gravity; }
	void set_extra_acceleration(const Vector3 &p_a) { extra_acceleration = p_a; }
	Vector3 get_extra_acceleration() const { return extra_acceleration; }
	// The gravity the cloth steps with right now (world or fixed, + extra_acceleration), m/s^2.
	Vector3 get_effective_gravity() const;
	void set_collide_with_body(bool p_enabled);
	bool get_collide_with_body() const { return collide_with_body; }
	void set_collision_radius_scale(float p_scale);
	float get_collision_radius_scale() const { return collision_radius_scale; }
	void set_simulating(bool p_simulating);
	bool is_simulating() const { return simulating; }

	void rebuild();
	void reset();
	int get_particle_count() const { return particle_count; }
	MeshInstance3D *get_render_instance() const;
	PackedInt32Array get_vertex_particles() const { return vertex_particles; }
	Array get_body_capsules() const;

	PhysXSkinnedCloth3D();
	~PhysXSkinnedCloth3D();
};

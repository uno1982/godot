/**************************************************************************/
/*  pxtgi_3d.h                                                            */
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

#include "physx_gi_effect.h"

#include "core/templates/hash_map.h"
#include "core/templates/hash_set.h"
#include "core/templates/local_vector.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/3d/shape_3d.h"
#include "scene/resources/environment.h"

// PXTGI (experimental): the module's own per-pixel global illumination, traced on screen and then against the physics
// shapes, delivered through Forward+'s stock GI path. With SDFGI enabled, every surface shader reads Godot's per-pixel
// GI buffer and multiplies it by its albedo. PhysXGIEffect writes that buffer before the opaque pass; while a visible
// PXTGI3D is in the tree, Godot's own GI step (which would overwrite it) runs a do-nothing shader with the same binding
// layout instead, and SDFGI is kept to its cheapest settings (it is only the switch that makes surfaces read the
// buffer). Hidden or gone, Godot's pipelines and the environment's own SDFGI settings are restored.
class PXTGI3D : public Node3D {
	GDCLASS(PXTGI3D, Node3D);

	static int users;

	float intensity = 1.0f;
	float max_distance = 8.0f;
	float world_ray_distance = 30.0f;
	int debug = 0;
	bool multi_bounce = true;
	int rays_per_texel = 4;
	int reflection_mode = 2;
	float reflection_cutoff = 0.5f;
	ObjectID attached_world;

	// The collision shapes found in the scene (rescanned now and then), each with its body's albedo and emission;
	// and emissive meshes outside any body, which stand in as their bounding box (mesh_proxy).
	struct ShapeEntry {
		ObjectID shape_node;
		Color albedo;
		Color emission;
		bool mesh_proxy = false;
		bool fixed = false; // a static body's shape (not animatable): a reflection probe's capture of it stays true
		ObjectID shape_resource; // a collision shape's Shape3D when found: another one means finding the shapes again
		// Hulls and meshes: their type (4, 5), where their geometry starts in the geometry buffer, their local bounds.
		int geometry_type = 0;
		int geometry_a = 0;
		int geometry_b = 0;
		AABB local_bounds;
		// Where the shape sits on its body and what it is (type, extents), read from its node when last checked: the
		// shapes of a body PhysX moved are placed from the body's own pose, without the scene tree.
		bool posed = false;
		Transform3D in_body;
		float type = 0.0f;
		Vector3 extents;
	};
	// Hull planes, or a mesh's tree and triangles, built once per shape resource.
	struct GeometryBlock {
		int type = 0;
		Vector<float> planes; // hull: normal, offset
		Vector<float> nodes; // mesh: its tree (8 floats a node; indices relative to the mesh)
		Vector<float> triangles; // mesh: 12 floats a triangle (three corners, padded)
		AABB bounds;
	};
	HashMap<ObjectID, GeometryBlock> geometry_blocks;
	Vector<float> geometry;
	uint64_t geometry_version = 0;
	uint32_t geometry_signature = 0;

	const GeometryBlock *_geometry_for(const Ref<Shape3D> &p_shape);
	LocalVector<ShapeEntry> shapes;
	LocalVector<ObjectID> suns; // every DirectionalLight3D: the brightest one drawn on surfaces is the sun, chosen each frame
	Ref<Environment> environment; // the one whose SDFGI is the switch, given its own settings back on leaving
	uint32_t last_hash = 0; // what was published last: a change means the GI has to converge again
	// What gives off light (which lights and emitters, how bright; not where, as moving lamps change it every frame):
	// a change lets the radiance cache follow quickly for a while.
	uint32_t last_sources_hash = 0;
	int fresh_frames = 0;
	LocalVector<ObjectID> omnis;
	LocalVector<ObjectID> spots;
	LocalVector<ObjectID> probes; // ReflectionProbes: where they override the GI, it isn't traced
	uint32_t shape_signature = 0; // the shapes found by the last rescan: a change means rebuilding the tree

	// The nodes the GI keeps track of (lights, reflection probes, the collision shapes of bodies, meshes outside any
	// body): found by one walk of the scene when PXTGI starts, then kept up to date from the scene tree's node_added and
	// node_removed, so finding the shapes again never walks the whole scene.
	HashSet<ObjectID> tracked;
	ObjectID tracked_root; // the node they were found under (the edited scene in the editor); none: walk again
	bool tracked_changed = true; // a tracked node came or went (or a look or shape changed): find the shapes again
	int frames_since_rescan = 0;
	// Each tracked shape's and mesh's look (albedo, emission), worked out once and then a slice of them each frame
	// (all in about a second), so a changed material reaches the GI without a pause.
	HashMap<ObjectID, Pair<Color, Color>> looks;
	LocalVector<ObjectID> look_order;
	uint32_t look_cursor = 0;
	HashMap<ObjectID, int> entry_of; // a shape node's index in shapes
	uint32_t verify_cursor = 0;

	void _track_all(Node *p_root);
	void _node_added(Node *p_node);
	void _node_removed(Node *p_node);
	Node *_scan_root() const;
	Pair<Color, Color> _look_of(Node *p_node) const;
	void _restyle_slice(LocalVector<int> &r_changed);

	// Each shape's GPU record and world bounds (in shape order), kept between frames: only what moved is rewritten,
	// and the tree is refit rather than rebuilt.
	LocalVector<float> entry_floats;
	LocalVector<AABB> entry_bounds;
	LocalVector<uint8_t> entry_valid;
	HashMap<ObjectID, LocalVector<int>> body_entries; // a physics body's shapes, by its node
	// The poses of the PhysX bodies awake in the physics steps since the last frame, by their nodes: read during
	// physics process, where the space may be read even with physics on its own thread.
	HashMap<ObjectID, Transform3D> moved_poses;
	LocalVector<int> proxy_entries; // emissive meshes outside any body: checked every frame
	// Whether the shapes include triangle meshes (found at the last rescan), and whether the tree was last built for
	// them: leaves of one shape, the meshes walked in the same loop as the shapes (Snapshot::mesh_walk).
	bool mesh_shapes = false;
	bool tree_mesh_walk = false;
	LocalVector<int> emissive_entries; // shapes that give off light: sampled directly
	// A mesh resource's surfaces' area-weighted mean normals (their length: how much of the surface faces that way),
	// for which way a flat glow faces.
	HashMap<ObjectID, Vector<Vector3>> surface_normals;
	HashSet<int> chosen_emissives; // the emissives sampled directly last frame (shape indices): a head start this one
	Vector3 _emissive_facing(int p_index);
	LocalVector<float> ordered_floats; // the records in the tree's leaf order
	LocalVector<float> tree_nodes;
	LocalVector<int> leaf_order; // leaf slot -> shape
	LocalVector<int> entry_slot; // shape -> leaf slot (-1: not in the tree)
	Vector<float> published_primitives;
	Vector<float> published_nodes;
	int published_count = 0;
	uint64_t primitives_version = 0;
	int moves_since_build = 0;

	void _rescan();
	bool _shape_frame(int p_index, Transform3D &r_to_world, float &r_type, Vector3 &r_extents) const;
	void _pack_entry(int p_index, const Transform3D &p_to_world, float p_type, const Vector3 &p_extents, float *r_out, AABB &r_bounds) const;
	bool _write_entry(int p_index, float *r_out, AABB &r_bounds) const;
	bool _refresh_entry(int p_index, float *r_out, AABB &r_bounds);
	bool _entry_frame(int p_index, Transform3D &r_to_world, AABB &r_local) const;
	void _update_primitives(PhysXGIEffect::Snapshot &r_snapshot);

	static void _install();
	static void _uninstall();

public:
	static void free_shared();

private:
	bool active = false; // in the world and visible: delivering the GI
	void _activate();
	void _deactivate();
	void _attach();
	void _detach();
	void _publish();

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	PackedStringArray get_configuration_warnings() const override;

	void set_intensity(float p_value) { intensity = MAX(p_value, 0.0f); }
	float get_intensity() const { return intensity; }
	void set_max_distance(float p_value) { max_distance = MAX(p_value, 0.1f); }
	float get_max_distance() const { return max_distance; }
	void set_world_ray_distance(float p_value) { world_ray_distance = MAX(p_value, 0.1f); }
	float get_world_ray_distance() const { return world_ray_distance; }
	void set_rays_per_texel(int p_value) { rays_per_texel = CLAMP(p_value, 1, 4); }
	int get_rays_per_texel() const { return rays_per_texel; }
	void set_multi_bounce(bool p_value) { multi_bounce = p_value; }
	bool get_multi_bounce() const { return multi_bounce; }
	void set_reflection_mode(int p_value) {
		reflection_mode = CLAMP(p_value, 0, 3);
		update_configuration_warnings();
	}
	int get_reflection_mode() const { return reflection_mode; }
	void set_reflection_roughness_cutoff(float p_value) { reflection_cutoff = CLAMP(p_value, 0.0f, 1.0f); }
	float get_reflection_roughness_cutoff() const { return reflection_cutoff; }
	void set_debug(int p_value) { debug = p_value; }
	int get_debug() const { return debug; }
};

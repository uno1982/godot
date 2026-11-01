/**************************************************************************/
/*  physx_gi_effect.h                                                     */
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

#include "core/math/projection.h"
#include "core/math/transform_3d.h"
#include "core/os/mutex.h"
#include "core/templates/hash_map.h"
#include "scene/resources/compositor.h"
#include "servers/rendering/renderer_rd/storage_rd/render_buffer_custom_data_rd.h"
#include "servers/rendering/rendering_device.h"

#include <atomic>

class Camera3D;
class World3D;
class RenderData;

// One viewport's GI targets and history, kept on its render buffers (and freed with them): each viewport of the world
// (a minimap, a camera feed, split screen) keeps its own, rather than all of them sharing one and wiping it.
class PhysXGIView : public RenderBufferCustomDataRD {
	GDCLASS(PhysXGIView, RenderBufferCustomDataRD);

public:
	RID trace; // trace size: this frame's rays (premultiplied light, hit fraction)
	RID accum[2]; // trace size: accumulated over frames, ping-ponged
	RID accum_depth[2]; // trace size: the linear depth each accumulated texel was for
	RID moments[2]; // trace size: brightness mean, its square's mean, and how many frames the history holds
	RID variance; // trace size: how noisy each accumulated texel still is
	RID filtered[2]; // trace size: the edge-aware filter's passes, ping-ponged
	RID filtered_variance[2];
	RID reflect_trace; // half size: this frame's reflection rays (light, found)
	RID reflect_accum[2]; // half size: accumulated, ping-ponged
	RID reflect_depth[2];
	RID reflect_filtered[2]; // half size: the accumulated reflections after each filter pass (for the resolve only)
	Size2i reflect_size;
	Size2i trace_size;
	Size2i targets_for;
	int current = 0;
	bool have_history = false;
	Transform3D prev_cam_transform;
	Projection prev_projection;

	void ensure(RenderingDevice *p_rd, const Size2i &p_frame);
	void configure(RenderSceneBuffersRD *p_render_buffers) override {}
	void free_data() override;
	~PhysXGIView() { free_data(); }
};

// PXTGI's GPU side, computed before the opaque pass (PRE_OPAQUE) and written into Godot's per-pixel GI buffer, which
// every Forward+ surface shader multiplies by its albedo (PXTGI3D stops Godot's own GI step from overwriting it). Rays
// from the visible surfaces are marched against the depth buffer, a hit taking the light last frame's image showed
// there; rays that leave the screen continue against the collision shapes, lit from a radiance cache. Accumulated over
// frames, filtered along depth and normals, upsampled; plus traced reflections and directly sampled emissives.
class PhysXGIEffect : public CompositorEffect {
	GDCLASS(PhysXGIEffect, CompositorEffect);

public:
	// One collision shape the off-screen rays are traced against: world-to-local rows, the shape (x: 0 box, 1 sphere,
	// 2 capsule, 3 cylinder along local y, 4 convex hull, 5 triangle mesh; yzw: half extents; the radius; the radius
	// and half the height; for a hull its first plane and plane count, for a mesh its first tree node and first
	// triangle, in the geometry buffer, as int bits), its body's albedo, the light it emits.
	static constexpr int PRIMITIVE_FLOATS = 24;

	// A bounding volume tree over the primitives, two vec4 per node: min (w: the right child for an inner node, the
	// first primitive for a leaf), max (w: 0 for an inner node, the leaf's primitive count); ints stored as float bits.
	// An inner node's left child follows it.
	static constexpr int NODE_FLOATS = 8;

	struct Snapshot {
		Vector<float> primitives;
		int primitive_count = 0;
		uint64_t primitives_version = 0; // primitives and nodes: uploaded only when this changes
		Vector<float> nodes;
		int node_count = 0;
		// Hull planes and mesh trees and triangles (vec4s), shared by the primitives that point into it; uploaded when
		// the version changes.
		Vector<float> geometry;
		uint64_t geometry_version = 0;
		Vector3 sun_direction; // toward the sun, world space
		Color sun_color; // linear, times energy; black for none
		// Omni lights: position and 1 / range; linear colour times energy, and the attenuation exponent.
		static constexpr int MAX_OMNIS = 8;
		int omni_count = 0;
		Vector3 omni_position[MAX_OMNIS];
		float omni_inv_range[MAX_OMNIS] = {};
		Color omni_color[MAX_OMNIS];
		float omni_attenuation[MAX_OMNIS] = {};
		// Spot lights: as the omni lights, plus the direction they point, the cosine of their cone's half angle and the
		// cone's edge falloff (1 / spot attenuation, as Godot's shader takes it).
		static constexpr int MAX_SPOTS = 8;
		int spot_count = 0;
		Vector3 spot_position[MAX_SPOTS];
		float spot_inv_range[MAX_SPOTS] = {};
		Vector3 spot_direction[MAX_SPOTS];
		float spot_cos_angle[MAX_SPOTS] = {};
		Color spot_color[MAX_SPOTS];
		float spot_attenuation[MAX_SPOTS] = {};
		float spot_cone_attenuation[MAX_SPOTS] = {};
		float intensity = 1.0f;
		float max_distance = 8.0f;
		float world_distance = 30.0f; // how far a GI ray goes against the collision shapes
		int rays_per_texel = 4;
		// Emissive shapes sampled directly by every texel (the nearest to the camera): an oriented box each (centre and
		// three half axes, world space) and the light its surface gives off.
		static constexpr int MAX_EMISSIVES = 64;
		int emissive_count = 0;
		Vector3 emissive_center[MAX_EMISSIVES];
		Vector3 emissive_axis[MAX_EMISSIVES][3];
		Color emissive_color[MAX_EMISSIVES];
		Vector3 emissive_facing[MAX_EMISSIVES]; // the way a one-sided glow faces; zero: it glows all round
		float emissive_pick[MAX_EMISSIVES]; // how often each is picked (the shares add up to 1)
		int emissive_slot[MAX_EMISSIVES]; // its shape's slot in the tree's records (-1: none)
		bool multi_bounce = true; // off-screen hits light from the radiance cache
		bool light_changed = false; // the light sources just changed: the radiance cache follows quickly
		int reflection_mode = 2; // 0: Godot's own, 1: screen space, 2: screen space then the collision shapes, 3: and the probes
		float reflection_cutoff = 0.5f; // rougher surfaces keep Godot's reflection (probes, sky)
		// Reflection probes that override the GI buffers inside their box (Godot's scene shader replaces them there):
		// world-to-local, the inner half extents where the override is whole, what they override (1 ambient, 2
		// reflections).
		static constexpr int MAX_PROBES = 8;
		int probe_count = 0;
		Transform3D probe_to_local[MAX_PROBES];
		Vector3 probe_core[MAX_PROBES];
		int probe_overrides[MAX_PROBES] = {};
		int debug = 0; // 0: GI, 1: last frame's image at the reprojected pixel, 2: reconstruct / project round trip error
		// The shapes include triangle meshes: the tree has leaves of one shape, and the tracing passes walk it and the
		// meshes' trees in one loop.
		bool mesh_walk = false;
	};

private:
	static Mutex mutex;
	static std::atomic<int> settle; // frames still to draw before the GI has converged (the editor draws lazily)
	static HashMap<ObjectID, Snapshot> settings; // per node
	static HashMap<ObjectID, int> worlds; // nodes per world

	enum Pass {
		PASS_TRACE,
		PASS_TEMPORAL,
		PASS_ATROUS,
		PASS_REFLECT,
		PASS_REFLECT_TEMPORAL,
		PASS_CACHE,
		PASS_RESOLVE,
		PASS_REFLECT_FILTER,
		PASS_MAX,
	};
	RID shaders[PASS_MAX];
	RID pipelines[PASS_MAX];
	// The tracing passes again, built to walk the shapes and the meshes in one loop (Snapshot::mesh_walk).
	RID mesh_walk_shaders[PASS_MAX];
	RID mesh_walk_pipelines[PASS_MAX];
	bool mesh_walk = false; // this frame's
	RID linear_sampler;
	RID mip_sampler;
	RID nearest_sampler;
	RID data_buffer; // the frame's matrices
	RID primitive_buffer;
	int primitive_capacity = 0;
	uint64_t primitives_uploaded = UINT64_MAX;
	RID node_buffer;
	int node_capacity = 0;
	RID geometry_buffer;
	uint64_t geometry_uploaded = UINT64_MAX;
	bool failed = false;

	// The radiance cache: surface patches the off-screen rays hit, hashed by position (cells growing with distance from
	// the camera) and facing; each holds the light leaving it, bounce light included, refined a ray per frame.
	static constexpr int CACHE_SLOTS = 1 << 17;
	static constexpr int CACHE_ROW = 512; // the update pass walks the slots as a CACHE_ROW wide grid
	RID cache_keys;
	RID cache_cells;
	uint32_t frame = 0;

	bool _ensure(RenderingDevice *p_rd);
	void _dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, const Size2i &p_size, const void *p_push, uint32_t p_push_size);
	void _render(int p_callback_type, const RenderData *p_render_data);

protected:
	static void _bind_methods() {}

public:
	static void set_settings(const ObjectID &p_node, const Snapshot &p_snapshot);

	// The editor only draws when something changes, but the GI converges over frames: after the camera moves or the
	// scene changes it asks for this many more frames (unsettle), which the node draws while settling() is true.
	static void unsettle(int p_frames = 90) { settle.store(p_frames); }
	static bool settling() { return settle.fetch_sub(1) > 0; }
	static void remove_settings(const ObjectID &p_node);

	static void attach_world(World3D *p_world);
	static void detach_world(const ObjectID &p_world);
	static void install(World3D *p_world, Camera3D *p_camera);

	static Ref<PhysXGIEffect> get_shared();
	static void free_shared();

	PhysXGIEffect();
	~PhysXGIEffect();
};

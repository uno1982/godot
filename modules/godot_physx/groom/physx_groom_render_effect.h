/**************************************************************************/
/*  physx_groom_render_effect.h                                           */
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

#include "core/math/transform_3d.h"
#include "core/os/mutex.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "scene/resources/compositor.h"
#include "servers/rendering/rendering_device.h"

class Camera3D;
class QuadMesh;
class RenderData;
class Shader;
class ShaderMaterial;
class Texture2DRD;
class World3D;

// PhysXGroom3D's screen-space render mode. Compute passes before the transparent pass draw every groom's strands, each
// segment a band of its own width with its exact share of each pixel, and add up how much of each pixel the hair covers
// (Indiana Jones and the Great Circle's coverage, SIGGRAPH 2025); the two nearest strands at each pixel leave their
// direction, place along the strand and self-shadowing. Two quads over the screen in Godot's transparent pass (the
// screen-space fluid's proxy, PhysXFluidRenderEffect) shade those with the hair lighting (groom_shading.h), lit by the
// scene's lights and shadows: the second strand over what the hair covers beyond the first, then the first over its own
// share of the pixel, so a strand's highlight covers what the strand does. Smooth, thin hair with or without MSAA, not
// dithered.
class PhysXGroomRenderEffect : public CompositorEffect {
	GDCLASS(PhysXGroomRenderEffect, CompositorEffect);

public:
	struct Look {
		Color root_color;
		Color tip_color;
		float self_shadow = 0.6f;
		float roughness = 0.35f;
		int shading_model = 0;
	};

	// The strands, in the groom's space, 12 floats a point: position and radius; place along the strand (0 root, 1 tip),
	// place in the groom's random strand order (0-1), 1 on a strand's last point; the baked self-shadowing (zeros
	// without). Sent when they change (p_version), not every frame.
	static void set_points(const ObjectID &p_groom, uint64_t p_version, const PackedFloat32Array &p_points);
	// Every frame: where the groom is, its LOD and its look, and its simulated points (GroomSim's texture, world space,
	// p_sim_width texels a row; an invalid RID when it isn't simulated).
	static void set_state(const ObjectID &p_groom, const Transform3D &p_transform, float p_strand_fraction, float p_width_scale, const Look &p_look, const RID &p_sim_points = RID(), int p_sim_width = 0);
	static void remove_groom(const ObjectID &p_groom);

	static void attach_world(World3D *p_world);
	static void detach_world(const ObjectID &p_world);
	static void install(World3D *p_world, Camera3D *p_camera);
	static Ref<PhysXGroomRenderEffect> get_shared();
	static void free_shared();

	PhysXGroomRenderEffect();
	~PhysXGroomRenderEffect();

private:
	struct Groom {
		uint64_t version = 0;
		PackedFloat32Array points;
		Transform3D transform;
		float strand_fraction = 1.0f;
		float width_scale = 1.0f;
		Look look;
		RID sim_points;
		int sim_width = 0;
	};
	static Mutex mutex;
	static HashMap<ObjectID, Groom> grooms;

	static HashMap<ObjectID, int> worlds; // screen-space grooms, per world
	// The two proxy quads of each world: the strand behind, then the strand in front.
	static constexpr int LAYERS = 2;
	static constexpr int TEXTURES = 7;
	static HashMap<ObjectID, RID> proxies[LAYERS];
	static Ref<QuadMesh> proxy_meshes[LAYERS];
	static Ref<ShaderMaterial> proxy_materials[LAYERS];
	static Ref<Shader> proxy_shader;
	static Ref<Texture2DRD> hair_textures[TEXTURES];
	static RID published[TEXTURES];
	static bool textures_hooked, proxies_shown;
	static uint64_t ready_since;
	static void _update_proxy(Camera3D *p_camera);

	// GPU state (render thread only).
	enum Pass {
		PASS_CLEAR,
		PASS_RASTER,
		PASS_COMPOSITE,
		PASS_MAX,
	};
	struct GpuGroom {
		uint64_t version = 0;
		RID buffer;
		uint32_t point_count = 0;
	};
	bool failed = false;
	RID shaders[PASS_MAX];
	RID pipelines[PASS_MAX];
	RID point_sampler;
	RID params_buffer;
	// Bound for the simulated points when a groom isn't simulated.
	RID no_sim_points;
	HashMap<ObjectID, GpuGroom> gpu_grooms;
	Size2i frame_size;
	RID coverage_u, depth_u, depth2_u, front_cover;
	// a: view-space strand direction, place along it; b: eye depth, coverage, the front strand's share, the second
	// strand's eye depth; c: self-shadowing, view space, times the groom's strength; d: the groom's color at that place
	// along the strand, roughness + 2 * shading model; a2, c2, d2: a, c and d of the second strand.
	RID hair[TEXTURES];
	struct Retired {
		RID rid;
		uint64_t frame = 0;
	};
	LocalVector<Retired> retired;

	bool _ensure(RenderingDevice *p_rd);
	void _ensure_targets(RenderingDevice *p_rd, const Size2i &p_frame);
	void _free_targets(RenderingDevice *p_rd, bool p_now);
	void _dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, uint32_t p_x, uint32_t p_y, const void *p_push, uint32_t p_push_size);
	void _render(int p_callback_type, const RenderData *p_render_data);

protected:
	static void _bind_methods() {}
};

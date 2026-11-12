/**************************************************************************/
/*  physx_light_shafts_effect.h                                           */
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

#include "core/math/color.h"
#include "core/math/projection.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/os/mutex.h"
#include "core/templates/hash_map.h"
#include "scene/resources/compositor.h"
#include "servers/rendering/rendering_device.h"

class Camera3D;
class World3D;
class RenderData;

// Draws PhysXLightShafts3D's sun shafts into the frame, after transparent
// geometry and after Flow's gas (the liquid is in the depth by then; the gas
// is measured against the frame from before it, see gas_backdrop()): where
// the sky shows, its
// light is smeared outward from the sun's place on screen (bloom), and where
// it's blocked the frame is darkened along the same rays (occlusion).
// Installed like the module's other effects (see PhysXCompositorEffects).
class PhysXLightShaftsEffect : public CompositorEffect {
	GDCLASS(PhysXLightShaftsEffect, CompositorEffect);

public:
	// What a PhysXLightShafts3D draws with, copied for the render thread.
	struct Snapshot {
		Vector3 light_direction; // the way the light travels (world)
		Color bloom_color; // tint * scale; black: no bloom
		float bloom_threshold = 0.0f;
		float bloom_max_brightness = 0.0f;
		float occlusion_strength = 0.0f; // 0: no occlusion
		float occlusion_depth_range = 0.0f;
		float ray_length = 0.0f;
		float edge_fade = 0.0f;
	};

private:
	static Mutex mutex;
	static HashMap<ObjectID, Snapshot> shafts; // per node
	static HashMap<ObjectID, int> worlds; // nodes per world

	enum Pass {
		PASS_PREPARE,
		PASS_BLUR,
		PASS_COMPOSITE,
		PASS_MAX,
	};
	RID shaders[PASS_MAX];
	RID pipelines[PASS_MAX]; // the compute passes'
	HashMap<int64_t, RID> composite_pipelines; // per framebuffer format
	RID linear_sampler;
	bool failed = false;
	// Half resolution, ping-ponged by the blur passes: rgb the bloom, a how
	// unblocked the light is.
	RID half_a;
	RID half_b;
	RID gas_cover; // half size: how much of each pixel is gas (written by the prepare pass)
	Size2i half_size;
	// The frame as it was before PhysXFlow3D drew its gas (see gas_backdrop()),
	// one per view (an XR camera's eyes).
	static constexpr uint32_t MAX_VIEWS = 2;
	RID backdrop[MAX_VIEWS];
	Size2i backdrop_size;
	static uint32_t backdrop_ready; // a bit per view copied this frame

	bool _ensure(RenderingDevice *p_rd);
	void _ensure_targets(RenderingDevice *p_rd, const Size2i &p_size);
	void _free_targets(RenderingDevice *p_rd);
	void _dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, const Size2i &p_size, const void *p_push, uint32_t p_push_size);
	void _composite(RenderingDevice *p_rd, const RID &p_color, const RID &p_shafts, const RID &p_depth, const void *p_push, uint32_t p_push_size);
	void _render_view(RenderingDevice *p_rd, const Snapshot &p_shafts, const Size2i &p_frame, const RID &p_color, const RID &p_depth, const RID &p_backdrop, const Transform3D &p_camera, const Projection &p_projection, const Vector3 &p_eye_offset, float p_luminance);
	void _render(int p_callback_type, const RenderData *p_render_data);

protected:
	static void _bind_methods() {}

public:
	// For PhysXFlowRenderEffect, before it draws its gas in view p_view: a
	// texture of p_size to copy that view's frame into (the shafts take their
	// light from the frame without the gas, dimmed by how much the gas covers),
	// or an invalid RID when there are no shafts to draw.
	static RID gas_backdrop(RenderingDevice *p_rd, const Size2i &p_size, uint32_t p_view);

	static void set_shafts(const ObjectID &p_node, const Snapshot &p_snapshot);
	static void remove_shafts(const ObjectID &p_node);

	// A world gained / lost a node (counted); install keeps the effect running
	// there (call every frame: the user may swap compositors).
	static void attach_world(World3D *p_world);
	static void detach_world(const ObjectID &p_world);
	static void install(World3D *p_world, Camera3D *p_camera);

	static Ref<PhysXLightShaftsEffect> get_shared();
	static void free_shared();

	PhysXLightShaftsEffect();
	~PhysXLightShaftsEffect();
};

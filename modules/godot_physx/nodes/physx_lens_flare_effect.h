/**************************************************************************/
/*  physx_lens_flare_effect.h                                             */
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
#include "core/os/mutex.h"
#include "core/templates/hash_map.h"
#include "scene/resources/compositor.h"
#include "servers/rendering/rendering_device.h"

class Camera3D;
class World3D;
class RenderData;

// Draws PhysXLensFlare3D's flare into the frame: an image-based camera flare, after the light shafts and before the
// lens dirt (which it lights too). The frame's bright light, softened, is copied back over the screen as ghosts --
// scaled about the screen's centre, mirrored through it for negative scales, tinted, colour-split -- plus a halo
// ring, and added on top. Every bright light flares, wherever it is on screen. Installed like the module's other
// effects (see PhysXCompositorEffects).
class PhysXLensFlareEffect : public CompositorEffect {
	GDCLASS(PhysXLensFlareEffect, CompositorEffect);

public:
	static constexpr int MAX_GHOSTS = 8;

	// What a PhysXLensFlare3D draws with, copied for the render thread.
	struct Snapshot {
		Color color; // tint * intensity
		float threshold = 0.0f;
		float threshold_range = 1.0f;
		float ghost_intensity = 0.0f;
		float ghost_chroma_shift = 0.0f;
		int ghost_count = 0;
		Color ghost_colors[MAX_GHOSTS];
		float ghost_scales[MAX_GHOSTS] = {};
		float halo_intensity = 0.0f;
		float halo_width = 0.0f;
		float halo_mask = 0.0f;
		float halo_compression = 0.0f;
		float halo_chroma_shift = 0.0f;
	};

private:
	static Mutex mutex;
	static HashMap<ObjectID, Snapshot> flares; // per node
	static HashMap<ObjectID, int> worlds; // nodes per world

	enum Pass {
		PASS_PREPARE,
		PASS_DOWN,
		PASS_UP,
		PASS_GHOSTS,
		PASS_COMPOSITE,
		PASS_MAX,
	};
	RID shaders[PASS_MAX];
	RID pipelines[PASS_MAX]; // the compute passes'
	HashMap<int64_t, RID> composite_pipelines; // per framebuffer format
	RID linear_sampler;
	RID border_sampler; // black outside the image: a ghost reaching past the frame adds nothing
	RID ghost_buffer; // the ghosts' colours and scales
	bool failed = false;
	RID bright; // quarter size: the bright light
	// Blurred well past the light's own shape on the way down and back up, the way a real lens's reflections are
	// out of focus: a ghost is then a soft round blob, not a copy of whatever shape the light has.
	static constexpr int BLUR_LEVELS = 2;
	RID down[BLUR_LEVELS]; // an eighth, a sixteenth
	RID up[BLUR_LEVELS - 1]; // an eighth
	Size2i down_size[BLUR_LEVELS];
	RID soft; // quarter size again: the softened bright light the ghosts copy
	RID flare; // quarter size: ghosts and halo
	Size2i quarter;
	Size2i targets_for;

	bool _ensure(RenderingDevice *p_rd);
	void _ensure_targets(RenderingDevice *p_rd, const Size2i &p_frame);
	void _free_targets(RenderingDevice *p_rd);
	void _dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, const Size2i &p_size, const void *p_push, uint32_t p_push_size);
	void _composite(RenderingDevice *p_rd, const RID &p_color, const void *p_push, uint32_t p_push_size);
	void _render(int p_callback_type, const RenderData *p_render_data);
	void _render_view(RenderingDevice *p_rd, const Size2i &p_frame, const RID &p_color, const void *p_push, uint32_t p_push_size);

protected:
	static void _bind_methods() {}

public:
	static void set_flare(const ObjectID &p_node, const Snapshot &p_snapshot);
	static void remove_flare(const ObjectID &p_node);

	// A world gained / lost a node (counted); install keeps the effect running there (call every frame: the user may
	// swap compositors).
	static void attach_world(World3D *p_world);
	static void detach_world(const ObjectID &p_world);
	static void install(World3D *p_world, Camera3D *p_camera);

	static Ref<PhysXLensFlareEffect> get_shared();
	static void free_shared();

	PhysXLensFlareEffect();
	~PhysXLensFlareEffect();
};

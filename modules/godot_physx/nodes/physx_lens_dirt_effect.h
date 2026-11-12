/**************************************************************************/
/*  physx_lens_dirt_effect.h                                              */
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

// Draws PhysXLensDirt3D's lens dirt into the frame, after transparent geometry, Flow and the light shafts (so all
// of them light it): the frame's bright light, spread wide the way a lens scatters it, times a dirt texture, added
// on top (Unreal's bloom dirt mask: frame += bloom * dirt * tint). Installed like the module's other effects (see
// PhysXCompositorEffects).
class PhysXLensDirtEffect : public CompositorEffect {
	GDCLASS(PhysXLensDirtEffect, CompositorEffect);

public:
	// What a PhysXLensDirt3D draws with, copied for the render thread.
	struct Snapshot {
		RID dirt_texture; // a RenderingServer texture
		Color color; // tint * intensity
		float threshold = 0.0f;
	};

private:
	static Mutex mutex;
	static HashMap<ObjectID, Snapshot> dirts; // per node
	static HashMap<ObjectID, int> worlds; // nodes per world

	enum Pass {
		PASS_PREPARE,
		PASS_DOWN,
		PASS_UP,
		PASS_COMPOSITE,
		PASS_MAX,
	};
	// The bloom pyramid: LEVELS levels from a quarter of the frame down, halving each time.
	static constexpr int LEVELS = 4;
	RID shaders[PASS_MAX];
	RID pipelines[PASS_MAX]; // the compute passes'
	HashMap<int64_t, RID> composite_pipelines; // per framebuffer format
	RID linear_sampler;
	bool failed = false;
	RID down[LEVELS]; // bright light, ever smaller
	RID up[LEVELS - 1]; // summed back up: up[0] is the wide bloom the dirt shows
	Size2i level_size[LEVELS];
	Size2i targets_for;

	bool _ensure(RenderingDevice *p_rd);
	void _ensure_targets(RenderingDevice *p_rd, const Size2i &p_frame);
	void _free_targets(RenderingDevice *p_rd);
	void _dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, const Size2i &p_size, const void *p_push, uint32_t p_push_size);
	void _composite(RenderingDevice *p_rd, const RID &p_color, const RID &p_bloom, const RID &p_dirt, const void *p_push, uint32_t p_push_size);
	void _render(int p_callback_type, const RenderData *p_render_data);
	void _render_view(RenderingDevice *p_rd, const Snapshot &p_dirt, float p_luminance, const Size2i &p_frame, const RID &p_color, const RID &p_dirt_texture);

protected:
	static void _bind_methods() {}

public:
	static void set_dirt(const ObjectID &p_node, const Snapshot &p_snapshot);
	static void remove_dirt(const ObjectID &p_node);

	// A world gained / lost a node (counted); install keeps the effect running there (call every frame: the user may
	// swap compositors).
	static void attach_world(World3D *p_world);
	static void detach_world(const ObjectID &p_world);
	static void install(World3D *p_world, Camera3D *p_camera);

	static Ref<PhysXLensDirtEffect> get_shared();
	static void free_shared();

	PhysXLensDirtEffect();
	~PhysXLensDirtEffect();
};

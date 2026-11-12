/**************************************************************************/
/*  physx_lens_dirt_effect.cpp                                            */
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

#include "physx_lens_dirt_effect.h"

#include "../godot_physx_render_thread.h"
#include "physx_compositor_effects.h"

#include "core/object/callable_mp.h"
#include "scene/3d/camera_3d.h"
#include "scene/resources/3d/world_3d.h"
#include "servers/rendering/renderer_rd/framebuffer_cache_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_buffers_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/texture_storage.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"
#include "servers/rendering/storage/render_data.h"

Mutex PhysXLensDirtEffect::mutex;
HashMap<ObjectID, PhysXLensDirtEffect::Snapshot> PhysXLensDirtEffect::dirts;
HashMap<ObjectID, int> PhysXLensDirtEffect::worlds;

namespace {

Ref<PhysXLensDirtEffect> shared_effect;

// Shared by every pass, spliced in at @PARAMS@ (a push constant).
const char *PARAMS = R"(
layout(push_constant, std430) uniform Params {
	ivec4 size; // xy: the pass's output, zw: its (larger or smaller) input
	vec4 misc; // x: threshold, y: the frame's luminance multiplier (the Mobile renderer stores color halved)
	vec4 color; // rgb: tint * intensity (the composite)
} p;
)";

const char *SHADERS[4] = {
	// PASS_PREPARE: the frame's bright light, at a quarter of its size (four bilinear taps cover the 4x4 pixels).
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 0) uniform sampler2D color_tex;
layout(set = 0, binding = 1, rgba16f) uniform restrict writeonly image2D dst;
void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, p.size.xy))) {
		return;
	}
	vec2 uv = (vec2(px) + 0.5) / vec2(p.size.xy);
	vec2 texel = 1.0 / vec2(p.size.zw);
	vec3 c = (texture(color_tex, uv + texel * vec2(-1.0, -1.0)).rgb + texture(color_tex, uv + texel * vec2(1.0, -1.0)).rgb +
					 texture(color_tex, uv + texel * vec2(-1.0, 1.0)).rgb + texture(color_tex, uv + texel * vec2(1.0, 1.0)).rgb) *
			(0.25 * p.misc.y);
	float lum = max(dot(c, vec3(0.2126, 0.7152, 0.0722)), 1e-5);
	// What's above the threshold, capped so one blazing pixel doesn't flood the dirt.
	vec3 bright = c * (clamp(lum - p.misc.x, 0.0, 64.0) / lum);
	imageStore(dst, px, vec4(bright, 1.0));
}
)",
	// PASS_DOWN: half the size, a blur on the way (dual filtering: the center and four diagonal taps).
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 0) uniform sampler2D src;
layout(set = 0, binding = 1, rgba16f) uniform restrict writeonly image2D dst;
void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, p.size.xy))) {
		return;
	}
	vec2 uv = (vec2(px) + 0.5) / vec2(p.size.xy);
	vec2 h = 1.0 / vec2(p.size.zw);
	vec3 sum = texture(src, uv).rgb * 4.0;
	sum += texture(src, uv + vec2(-h.x, -h.y)).rgb;
	sum += texture(src, uv + vec2(h.x, -h.y)).rgb;
	sum += texture(src, uv + vec2(-h.x, h.y)).rgb;
	sum += texture(src, uv + vec2(h.x, h.y)).rgb;
	imageStore(dst, px, vec4(sum / 8.0, 1.0));
}
)",
	// PASS_UP: the level below, twice the size (a tent of eight taps), plus this level's own bright light -- every
	// level summed, the light spreads wide with a bright core, as a lens scatters it.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 0) uniform sampler2D lower;
layout(set = 0, binding = 1) uniform sampler2D same;
layout(set = 0, binding = 2, rgba16f) uniform restrict writeonly image2D dst;
void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, p.size.xy))) {
		return;
	}
	vec2 uv = (vec2(px) + 0.5) / vec2(p.size.xy);
	vec2 h = 1.0 / vec2(p.size.zw);
	vec3 sum = texture(lower, uv + vec2(-2.0 * h.x, 0.0)).rgb + texture(lower, uv + vec2(2.0 * h.x, 0.0)).rgb +
			texture(lower, uv + vec2(0.0, -2.0 * h.y)).rgb + texture(lower, uv + vec2(0.0, 2.0 * h.y)).rgb;
	sum += (texture(lower, uv + vec2(-h.x, -h.y)).rgb + texture(lower, uv + vec2(h.x, -h.y)).rgb +
				   texture(lower, uv + vec2(-h.x, h.y)).rgb + texture(lower, uv + vec2(h.x, h.y)).rgb) *
			2.0;
	imageStore(dst, px, vec4(sum / 12.0 + texture(same, uv).rgb, 1.0));
}
)",
	// PASS_COMPOSITE: frame += bloom * dirt * tint, drawn (blended) so the Mobile renderer's color buffer, which
	// can't be a storage image, takes it too.
	R"(
#[vertex]
#version 450
void main() {
	vec2 p = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
	gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
#[fragment]
#version 450
@PARAMS@
layout(set = 0, binding = 0) uniform sampler2D bloom;
layout(set = 0, binding = 1) uniform sampler2D dirt;
layout(location = 0) out vec4 frag;
void main() {
	vec2 uv = gl_FragCoord.xy / vec2(p.size.xy);
	// The levels were summed: averaged back to the light's own brightness, then stored the way the frame is.
	vec3 light = texture(bloom, uv).rgb * (1.0 / float(@LEVELS@));
	frag = vec4(light * texture(dirt, uv).rgb * p.color.rgb / p.misc.y, 0.0);
}
)",
};

const char *PASS_NAMES[4] = { "prepare", "down", "up", "composite" };

struct Push {
	int32_t size[4];
	float misc[4];
	float color[4];
};

Size2i halved(const Size2i &p_size) {
	return Size2i(MAX(p_size.x / 2, 1), MAX(p_size.y / 2, 1));
}

} // namespace

void PhysXLensDirtEffect::set_dirt(const ObjectID &p_node, const Snapshot &p_snapshot) {
	MutexLock lock(mutex);
	dirts[p_node] = p_snapshot;
}

void PhysXLensDirtEffect::remove_dirt(const ObjectID &p_node) {
	MutexLock lock(mutex);
	dirts.erase(p_node);
}

void PhysXLensDirtEffect::attach_world(World3D *p_world) {
	ERR_FAIL_NULL(p_world);
	worlds[p_world->get_instance_id()]++;
}

void PhysXLensDirtEffect::detach_world(const ObjectID &p_world) {
	int *count = worlds.getptr(p_world);
	if (count == nullptr || --(*count) > 0) {
		return;
	}
	worlds.erase(p_world);
	PhysXCompositorEffects::uninstall(p_world, get_shared());
}

void PhysXLensDirtEffect::install(World3D *p_world, Camera3D *p_camera) {
	ERR_FAIL_NULL(p_world);
	ERR_FAIL_COND(!worlds.has(p_world->get_instance_id()));
	PhysXCompositorEffects::install(p_world, p_camera, get_shared(), PhysXCompositorEffects::ORDER_LENS_DIRT);
}

Ref<PhysXLensDirtEffect> PhysXLensDirtEffect::get_shared() {
	if (shared_effect.is_null()) {
		shared_effect.instantiate();
	}
	return shared_effect;
}

void PhysXLensDirtEffect::free_shared() {
	shared_effect.unref();
}

PhysXLensDirtEffect::PhysXLensDirtEffect() {
	set_effect_callback_type(EFFECT_CALLBACK_TYPE_POST_TRANSPARENT);
	set_access_resolved_color(true);
	RenderingServer::get_singleton()->compositor_effect_set_callback(get_rid(), RSE::COMPOSITOR_EFFECT_CALLBACK_TYPE_POST_TRANSPARENT, callable_mp(this, &PhysXLensDirtEffect::_render));
}

PhysXLensDirtEffect::~PhysXLensDirtEffect() {
	// Freed on the render thread, wherever the last reference goes.
	godot_physx_run_on_render_thread([this]() {
		RenderingDevice *rd = RenderingServer::get_singleton() ? RenderingServer::get_singleton()->get_rendering_device() : nullptr;
		if (rd == nullptr) {
			return;
		}
		_free_targets(rd);
		for (const KeyValue<int64_t, RID> &kv : composite_pipelines) {
			rd->free_rid(kv.value);
		}
		for (int i = 0; i < PASS_MAX; i++) {
			if (pipelines[i].is_valid()) {
				rd->free_rid(pipelines[i]);
			}
			if (shaders[i].is_valid()) {
				rd->free_rid(shaders[i]);
			}
		}
		if (linear_sampler.is_valid()) {
			rd->free_rid(linear_sampler);
		}
	});
}

bool PhysXLensDirtEffect::_ensure(RenderingDevice *p_rd) {
	if (shaders[PASS_COMPOSITE].is_valid()) {
		return true;
	}
	if (failed) {
		return false;
	}
	failed = true;
	for (int i = 0; i < PASS_MAX; i++) {
		Ref<RDShaderFile> file;
		file.instantiate();
		const String source = String(SHADERS[i]).replace("@PARAMS@", PARAMS).replace("@LEVELS@", itos(LEVELS));
		ERR_FAIL_COND_V_MSG(file->parse_versions_from_text(source) != OK, false, vformat("PhysXLensDirt3D: %s shader failed to compile.", PASS_NAMES[i]));
		Ref<RDShaderSPIRV> spirv = file->get_spirv();
		ERR_FAIL_COND_V(spirv.is_null(), false);
		const bool raster = i == PASS_COMPOSITE;
		for (RD::ShaderStage stage : raster ? Vector<RD::ShaderStage>({ RD::SHADER_STAGE_VERTEX, RD::SHADER_STAGE_FRAGMENT }) : Vector<RD::ShaderStage>({ RD::SHADER_STAGE_COMPUTE })) {
			const String err = spirv->get_stage_compile_error(stage);
			ERR_FAIL_COND_V_MSG(!err.is_empty(), false, vformat("PhysXLensDirt3D: %s shader: %s", PASS_NAMES[i], err));
		}
		shaders[i] = p_rd->shader_create_from_spirv(file->get_spirv_stages());
		ERR_FAIL_COND_V(!shaders[i].is_valid(), false);
		if (!raster) {
			pipelines[i] = p_rd->compute_pipeline_create(shaders[i]);
			ERR_FAIL_COND_V(!pipelines[i].is_valid(), false);
		}
	}
	RD::SamplerState linear;
	linear.mag_filter = RD::SAMPLER_FILTER_LINEAR;
	linear.min_filter = RD::SAMPLER_FILTER_LINEAR;
	linear.repeat_u = RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE;
	linear.repeat_v = RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE;
	linear_sampler = p_rd->sampler_create(linear);
	failed = false;
	return true;
}

void PhysXLensDirtEffect::_free_targets(RenderingDevice *p_rd) {
	for (RID &r : down) {
		if (r.is_valid()) {
			p_rd->free_rid(r);
			r = RID();
		}
	}
	for (RID &r : up) {
		if (r.is_valid()) {
			p_rd->free_rid(r);
			r = RID();
		}
	}
	targets_for = Size2i();
}

void PhysXLensDirtEffect::_ensure_targets(RenderingDevice *p_rd, const Size2i &p_frame) {
	if (p_frame == targets_for && down[0].is_valid()) {
		return;
	}
	_free_targets(p_rd);
	targets_for = p_frame;
	RD::TextureFormat tf;
	tf.format = RD::DATA_FORMAT_R16G16B16A16_SFLOAT;
	tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT;
	Size2i size = halved(halved(p_frame));
	for (int i = 0; i < LEVELS; i++) {
		level_size[i] = size;
		tf.width = size.x;
		tf.height = size.y;
		down[i] = p_rd->texture_create(tf, RD::TextureView());
		if (i < LEVELS - 1) {
			up[i] = p_rd->texture_create(tf, RD::TextureView());
		}
		size = halved(size);
	}
}

void PhysXLensDirtEffect::_dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, const Size2i &p_size, const void *p_push, uint32_t p_push_size) {
	const RID set = p_rd->uniform_set_create(p_uniforms, shaders[p_pass], 0);
	ERR_FAIL_COND(!set.is_valid());
	// Its own compute list per pass: the next pass then sees this one's writes.
	const RD::ComputeListID list = p_rd->compute_list_begin();
	p_rd->compute_list_bind_compute_pipeline(list, pipelines[p_pass]);
	p_rd->compute_list_bind_uniform_set(list, set, 0);
	p_rd->compute_list_set_push_constant(list, p_push, p_push_size);
	p_rd->compute_list_dispatch(list, (p_size.x + 7) / 8, (p_size.y + 7) / 8, 1);
	p_rd->compute_list_end();
	p_rd->free_rid(set);
}

void PhysXLensDirtEffect::_composite(RenderingDevice *p_rd, const RID &p_color, const RID &p_bloom, const RID &p_dirt, const void *p_push, uint32_t p_push_size) {
	const RID framebuffer = FramebufferCacheRD::get_singleton()->get_cache(p_color);
	ERR_FAIL_COND(!framebuffer.is_valid());
	const RD::FramebufferFormatID format = p_rd->framebuffer_get_format(framebuffer);
	RID *pipeline = composite_pipelines.getptr(format);
	if (pipeline == nullptr) {
		// frame + light on the dirt; the frame's own alpha kept.
		RD::PipelineColorBlendState blend;
		RD::PipelineColorBlendState::Attachment attachment;
		attachment.enable_blend = true;
		attachment.src_color_blend_factor = RD::BLEND_FACTOR_ONE;
		attachment.dst_color_blend_factor = RD::BLEND_FACTOR_ONE;
		attachment.color_blend_op = RD::BLEND_OP_ADD;
		attachment.src_alpha_blend_factor = RD::BLEND_FACTOR_ZERO;
		attachment.dst_alpha_blend_factor = RD::BLEND_FACTOR_ONE;
		attachment.alpha_blend_op = RD::BLEND_OP_ADD;
		blend.attachments.push_back(attachment);
		RD::PipelineMultisampleState multisample;
		multisample.sample_count = p_rd->framebuffer_format_get_texture_samples(format);
		const RID created = p_rd->render_pipeline_create(shaders[PASS_COMPOSITE], format, RD::INVALID_ID, RD::RENDER_PRIMITIVE_TRIANGLES, RD::PipelineRasterizationState(), multisample, RD::PipelineDepthStencilState(), blend);
		ERR_FAIL_COND(!created.is_valid());
		pipeline = &composite_pipelines.insert(format, created)->value;
	}
	const RID set = p_rd->uniform_set_create(Vector<RD::Uniform>({ RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ linear_sampler, p_bloom })), RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 1, Vector<RID>({ linear_sampler, p_dirt })) }), shaders[PASS_COMPOSITE], 0);
	ERR_FAIL_COND(!set.is_valid());
	const RD::DrawListID list = p_rd->draw_list_begin(framebuffer, RD::DRAW_DEFAULT_ALL);
	p_rd->draw_list_bind_render_pipeline(list, *pipeline);
	p_rd->draw_list_bind_uniform_set(list, set, 0);
	p_rd->draw_list_set_push_constant(list, p_push, p_push_size);
	p_rd->draw_list_draw(list, false, 1, 3);
	p_rd->draw_list_end();
	p_rd->free_rid(set);
}

void PhysXLensDirtEffect::_render(int p_callback_type, const RenderData *p_render_data) {
	Snapshot s;
	{
		MutexLock lock(mutex);
		if (dirts.is_empty()) {
			return;
		}
		s = dirts.begin()->value;
	}
	if (p_render_data == nullptr || !s.dirt_texture.is_valid()) {
		return;
	}
	RenderSceneBuffersRD *buffers = Object::cast_to<RenderSceneBuffersRD>(p_render_data->get_render_scene_buffers().ptr());
	RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
	if (buffers == nullptr || rd == nullptr || !_ensure(rd)) {
		return;
	}
	const Size2i frame = buffers->get_internal_size();
	// The dirt as the GPU has it (its sRGB view, so it reads linear like the frame).
	const RID dirt = RendererRD::TextureStorage::get_singleton()->texture_get_rd_texture(s.dirt_texture, true);
	if (frame.x < 8 || frame.y < 8 || !buffers->get_internal_texture().is_valid() || !dirt.is_valid()) {
		return;
	}
	_ensure_targets(rd, frame);
	// An XR camera renders its eyes as the layers of one frame: each eye's lens
	// gets its own pass, on its own slice.
	const uint32_t view_count = MAX(buffers->get_view_count(), 1u);
	for (uint32_t v = 0; v < view_count; v++) {
		const RID color = view_count > 1 ? buffers->get_internal_texture(v) : buffers->get_internal_texture();
		if (color.is_valid()) {
			_render_view(rd, s, buffers->get_luminance_multiplier(), frame, color, dirt);
		}
	}
}

void PhysXLensDirtEffect::_render_view(RenderingDevice *p_rd, const Snapshot &p_dirt, float p_luminance, const Size2i &p_frame, const RID &p_color, const RID &p_dirt_texture) {
	RenderingDevice *rd = p_rd;
	const Snapshot &s = p_dirt;
	const Size2i frame = p_frame;
	const RID color = p_color;
	const RID dirt = p_dirt_texture;

	Push push = {};
	// The Mobile renderer (without HDR 2D) stores color halved, so 0..2 fits its buffer: read it back up, and keep the
	// threshold under what it can hold.
	const float luminance = p_luminance;
	push.misc[0] = luminance > 1.0f ? MIN(s.threshold, 0.75f * luminance) : s.threshold;
	push.misc[1] = luminance;
	push.color[0] = s.color.r;
	push.color[1] = s.color.g;
	push.color[2] = s.color.b;

	push.size[0] = level_size[0].x;
	push.size[1] = level_size[0].y;
	push.size[2] = frame.x;
	push.size[3] = frame.y;
	_dispatch(rd, PASS_PREPARE, { RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ linear_sampler, color })), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, down[0]) }, level_size[0], &push, sizeof(push));
	for (int i = 1; i < LEVELS; i++) {
		push.size[0] = level_size[i].x;
		push.size[1] = level_size[i].y;
		push.size[2] = level_size[i - 1].x;
		push.size[3] = level_size[i - 1].y;
		_dispatch(rd, PASS_DOWN, { RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ linear_sampler, down[i - 1] })), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, down[i]) }, level_size[i], &push, sizeof(push));
	}
	for (int i = LEVELS - 2; i >= 0; i--) {
		const RID lower = i == LEVELS - 2 ? down[LEVELS - 1] : up[i + 1];
		push.size[0] = level_size[i].x;
		push.size[1] = level_size[i].y;
		push.size[2] = level_size[i + 1].x;
		push.size[3] = level_size[i + 1].y;
		_dispatch(rd, PASS_UP, { RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ linear_sampler, lower })), RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 1, Vector<RID>({ linear_sampler, down[i] })), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, up[i]) }, level_size[i], &push, sizeof(push));
	}
	push.size[0] = frame.x;
	push.size[1] = frame.y;
	_composite(rd, color, up[0], dirt, &push, sizeof(push));
}

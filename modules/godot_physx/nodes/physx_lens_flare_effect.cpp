/**************************************************************************/
/*  physx_lens_flare_effect.cpp                                           */
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

#include "physx_lens_flare_effect.h"

#include "../godot_physx_render_thread.h"
#include "physx_compositor_effects.h"

#include "core/object/callable_mp.h"
#include "scene/3d/camera_3d.h"
#include "scene/resources/3d/world_3d.h"
#include "servers/rendering/renderer_rd/framebuffer_cache_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_buffers_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_data_rd.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"
#include "servers/rendering/storage/render_data.h"

Mutex PhysXLensFlareEffect::mutex;
HashMap<ObjectID, PhysXLensFlareEffect::Snapshot> PhysXLensFlareEffect::flares;
HashMap<ObjectID, int> PhysXLensFlareEffect::worlds;

namespace {

Ref<PhysXLensFlareEffect> shared_effect;

// Shared by every pass, spliced in at @PARAMS@ (a push constant).
const char *PARAMS = R"(
layout(push_constant, std430) uniform Params {
	ivec4 size; // xy: the pass's output, zw: its input
	vec4 misc; // x: threshold, y: threshold range, z: the frame's luminance multiplier, w: width / height
	vec4 ghost; // x: intensity, y: chroma shift, z: count
	vec4 halo; // x: intensity, y: width, z: mask, w: compression
	vec4 color; // rgb: tint * intensity; a: the halo's chroma shift
	vec4 centre; // xy: the point the flare mirrors through (uv): the screen's centre, or for an XR eye where straight ahead is
} p;
)";

const char *SHADERS[5] = {
	// PASS_PREPARE: the frame's bright light, a quarter of its size, eased in over the threshold range.
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
	vec2 t = 1.0 / vec2(p.size.zw);
	vec3 c = (texture(color_tex, uv + t * vec2(-1.0, -1.0)).rgb + texture(color_tex, uv + t * vec2(1.0, -1.0)).rgb +
					 texture(color_tex, uv + t * vec2(-1.0, 1.0)).rgb + texture(color_tex, uv + t * vec2(1.0, 1.0)).rgb) *
			(0.25 * p.misc.z);
	float lum = dot(c, vec3(0.2126, 0.7152, 0.0722));
	float amount = clamp((lum - p.misc.x) / p.misc.y, 0.0, 1.0);
	// Capped, so one blazing pixel doesn't flood the ghosts.
	imageStore(dst, px, vec4(min(c * amount, vec3(32.0)), 1.0));
}
)",
	// PASS_DOWN: half the size, blurred on the way (the centre and four diagonal taps).
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
	sum += texture(src, uv + vec2(-h.x, -h.y)).rgb + texture(src, uv + vec2(h.x, -h.y)).rgb;
	sum += texture(src, uv + vec2(-h.x, h.y)).rgb + texture(src, uv + vec2(h.x, h.y)).rgb;
	imageStore(dst, px, vec4(sum / 8.0, 1.0));
}
)",
	// PASS_UP: back to a quarter of the frame, a tent of eight taps: the softened bright light.
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
	vec3 sum = texture(src, uv + vec2(-2.0 * h.x, 0.0)).rgb + texture(src, uv + vec2(2.0 * h.x, 0.0)).rgb +
			texture(src, uv + vec2(0.0, -2.0 * h.y)).rgb + texture(src, uv + vec2(0.0, 2.0 * h.y)).rgb;
	sum += (texture(src, uv + vec2(-h.x, -h.y)).rgb + texture(src, uv + vec2(h.x, -h.y)).rgb +
				   texture(src, uv + vec2(-h.x, h.y)).rgb + texture(src, uv + vec2(h.x, h.y)).rgb) *
			2.0;
	imageStore(dst, px, vec4(sum / 12.0, 1.0));
}
)",
	// PASS_GHOSTS: the ghosts and the halo, a quarter of the frame's size.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 0) uniform sampler2D src; // black outside
layout(set = 0, binding = 1, rgba16f) uniform restrict writeonly image2D dst;
layout(set = 0, binding = 2, std140) uniform Ghosts {
	vec4 colors[8]; // rgb * a
	vec4 scales[2];
} g;

// The bright light at uv, faded out toward the frame's edges: what's past the edge isn't in the frame, so without
// the fade a light cut by the edge copies over as a shape with a hard straight side.
vec3 edged(vec2 uv) {
	vec2 e = smoothstep(vec2(0.0), vec2(0.2), uv) * smoothstep(vec2(0.0), vec2(0.2), 1.0 - uv);
	return texture(src, uv).rgb * (e.x * e.y);
}

// The bright light at uv, red and blue from slightly larger / smaller copies about the centre: the colour fringing
// a real lens gives its reflections. Each channel fades by where it samples, so a shifted channel doesn't run into
// the frame's edge.
vec3 split(vec2 uv, float shift) {
	vec2 o = uv - p.centre.xy;
	return vec3(edged(o * (1.0 + shift) + p.centre.xy).r, edged(uv).g, edged(o * (1.0 - shift) + p.centre.xy).b);
}

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, p.size.xy))) {
		return;
	}
	vec2 uv = (vec2(px) + 0.5) / vec2(p.size.xy);
	vec2 aspect = vec2(p.misc.w, 1.0);
	vec2 from_centre = uv - p.centre.xy;

	// Each ghost: this pixel shows the bright light at its own offset from the centre times the ghost's scale (a
	// negative scale mirrors it through the centre). A ghost drawn larger than the light spreads the same light over
	// more of the screen, so it's dimmer by its area.
	vec3 ghosts = vec3(0.0);
	int count = int(p.ghost.z);
	for (int i = 0; i < count; i++) {
		float scale = g.scales[i / 4][i % 4];
		ghosts += split(from_centre * scale + p.centre.xy, p.ghost.y) * g.colors[i].rgb * min(scale * scale, 1.0);
	}

	// The halo: a ring around the centre, the bright light a fixed step from each pixel toward the centre, so a light
	// draws an arc on the far side of the ring. Compression bends the screen like a fisheye first (the ring rounds out
	// near the edges); the mask keeps it to the outer part of the screen. A light's halo spreads it around a whole
	// ring, much thinner than a ghost: a tenth of the light at intensity 1.
	float r = length(from_centre * aspect);
	vec2 bent = from_centre / (1.0 + p.halo.w * r);
	vec2 inward = r > 1e-4 ? -normalize(from_centre * aspect) / aspect : vec2(0.0);
	vec2 at = bent + inward * p.halo.y * 0.5 + p.centre.xy;
	float ring = smoothstep(p.halo.z, 1.0, clamp(r / (0.5 * p.misc.w), 0.0, 1.0) * 0.5 + 0.5);
	vec3 halo = split(at, p.color.a) * ring * (0.1 * p.halo.x);

	imageStore(dst, px, vec4(ghosts * p.ghost.x + halo, 1.0));
}
)",
	// PASS_COMPOSITE: frame += flare * tint, drawn (blended) so the Mobile renderer's color buffer, which can't be a
	// storage image, takes it too; faded toward the frame's border.
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
layout(set = 0, binding = 0) uniform sampler2D flare;
layout(location = 0) out vec4 frag;
void main() {
	vec2 uv = gl_FragCoord.xy / vec2(p.size.xy);
	float border = 1.0 - smoothstep(0.8, 1.0, length((uv - 0.5) * 2.0) * 0.75);
	frag = vec4(texture(flare, uv).rgb * p.color.rgb * border / p.misc.z, 0.0);
}
)",
};

const char *PASS_NAMES[5] = { "prepare", "down", "up", "ghosts", "composite" };

struct Push {
	int32_t size[4];
	float misc[4];
	float ghost[4];
	float halo[4];
	float color[4];
	float centre[4];
};
static_assert(sizeof(Push) <= 128);

struct GhostData {
	float colors[PhysXLensFlareEffect::MAX_GHOSTS][4];
	float scales[PhysXLensFlareEffect::MAX_GHOSTS];
};

Size2i halved(const Size2i &p_size) {
	return Size2i(MAX(p_size.x / 2, 1), MAX(p_size.y / 2, 1));
}

} // namespace

void PhysXLensFlareEffect::set_flare(const ObjectID &p_node, const Snapshot &p_snapshot) {
	MutexLock lock(mutex);
	flares[p_node] = p_snapshot;
}

void PhysXLensFlareEffect::remove_flare(const ObjectID &p_node) {
	MutexLock lock(mutex);
	flares.erase(p_node);
}

void PhysXLensFlareEffect::attach_world(World3D *p_world) {
	ERR_FAIL_NULL(p_world);
	worlds[p_world->get_instance_id()]++;
}

void PhysXLensFlareEffect::detach_world(const ObjectID &p_world) {
	int *count = worlds.getptr(p_world);
	if (count == nullptr || --(*count) > 0) {
		return;
	}
	worlds.erase(p_world);
	PhysXCompositorEffects::uninstall(p_world, get_shared());
}

void PhysXLensFlareEffect::install(World3D *p_world, Camera3D *p_camera) {
	ERR_FAIL_NULL(p_world);
	ERR_FAIL_COND(!worlds.has(p_world->get_instance_id()));
	PhysXCompositorEffects::install(p_world, p_camera, get_shared(), PhysXCompositorEffects::ORDER_LENS_FLARE);
}

Ref<PhysXLensFlareEffect> PhysXLensFlareEffect::get_shared() {
	if (shared_effect.is_null()) {
		shared_effect.instantiate();
	}
	return shared_effect;
}

void PhysXLensFlareEffect::free_shared() {
	shared_effect.unref();
}

PhysXLensFlareEffect::PhysXLensFlareEffect() {
	set_effect_callback_type(EFFECT_CALLBACK_TYPE_POST_TRANSPARENT);
	set_access_resolved_color(true);
	RenderingServer::get_singleton()->compositor_effect_set_callback(get_rid(), RSE::COMPOSITOR_EFFECT_CALLBACK_TYPE_POST_TRANSPARENT, callable_mp(this, &PhysXLensFlareEffect::_render));
}

PhysXLensFlareEffect::~PhysXLensFlareEffect() {
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
		for (RID r : { linear_sampler, border_sampler, ghost_buffer }) {
			if (r.is_valid()) {
				rd->free_rid(r);
			}
		}
	});
}

bool PhysXLensFlareEffect::_ensure(RenderingDevice *p_rd) {
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
		const String source = String(SHADERS[i]).replace("@PARAMS@", PARAMS);
		ERR_FAIL_COND_V_MSG(file->parse_versions_from_text(source) != OK, false, vformat("PhysXLensFlare3D: %s shader failed to compile.", PASS_NAMES[i]));
		Ref<RDShaderSPIRV> spirv = file->get_spirv();
		ERR_FAIL_COND_V(spirv.is_null(), false);
		const bool raster = i == PASS_COMPOSITE;
		for (RD::ShaderStage stage : raster ? Vector<RD::ShaderStage>({ RD::SHADER_STAGE_VERTEX, RD::SHADER_STAGE_FRAGMENT }) : Vector<RD::ShaderStage>({ RD::SHADER_STAGE_COMPUTE })) {
			const String err = spirv->get_stage_compile_error(stage);
			ERR_FAIL_COND_V_MSG(!err.is_empty(), false, vformat("PhysXLensFlare3D: %s shader: %s", PASS_NAMES[i], err));
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
	RD::SamplerState border = linear;
	border.repeat_u = RD::SAMPLER_REPEAT_MODE_CLAMP_TO_BORDER;
	border.repeat_v = RD::SAMPLER_REPEAT_MODE_CLAMP_TO_BORDER;
	border.border_color = RD::SAMPLER_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
	border_sampler = p_rd->sampler_create(border);
	ghost_buffer = p_rd->uniform_buffer_create(MAX_GHOSTS * 16 + 32);
	failed = false;
	return true;
}

void PhysXLensFlareEffect::_free_targets(RenderingDevice *p_rd) {
	for (RID *r : { &bright, &soft, &flare, &down[0], &down[1], &up[0] }) {
		if (r->is_valid()) {
			p_rd->free_rid(*r);
			*r = RID();
		}
	}
	targets_for = Size2i();
}

void PhysXLensFlareEffect::_ensure_targets(RenderingDevice *p_rd, const Size2i &p_frame) {
	if (p_frame == targets_for && bright.is_valid()) {
		return;
	}
	_free_targets(p_rd);
	targets_for = p_frame;
	quarter = halved(halved(p_frame));
	RD::TextureFormat tf;
	tf.format = RD::DATA_FORMAT_R16G16B16A16_SFLOAT;
	tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT;
	tf.width = quarter.x;
	tf.height = quarter.y;
	bright = p_rd->texture_create(tf, RD::TextureView());
	soft = p_rd->texture_create(tf, RD::TextureView());
	flare = p_rd->texture_create(tf, RD::TextureView());
	Size2i size = quarter;
	for (int i = 0; i < BLUR_LEVELS; i++) {
		size = halved(size);
		down_size[i] = size;
		tf.width = size.x;
		tf.height = size.y;
		down[i] = p_rd->texture_create(tf, RD::TextureView());
		if (i < BLUR_LEVELS - 1) {
			up[i] = p_rd->texture_create(tf, RD::TextureView());
		}
	}
}

void PhysXLensFlareEffect::_dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, const Size2i &p_size, const void *p_push, uint32_t p_push_size) {
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

void PhysXLensFlareEffect::_composite(RenderingDevice *p_rd, const RID &p_color, const void *p_push, uint32_t p_push_size) {
	const RID framebuffer = FramebufferCacheRD::get_singleton()->get_cache(p_color);
	ERR_FAIL_COND(!framebuffer.is_valid());
	const RD::FramebufferFormatID format = p_rd->framebuffer_get_format(framebuffer);
	RID *pipeline = composite_pipelines.getptr(format);
	if (pipeline == nullptr) {
		// frame + flare; the frame's own alpha kept.
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
	const RID set = p_rd->uniform_set_create(Vector<RD::Uniform>({ RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ linear_sampler, flare })) }), shaders[PASS_COMPOSITE], 0);
	ERR_FAIL_COND(!set.is_valid());
	const RD::DrawListID list = p_rd->draw_list_begin(framebuffer, RD::DRAW_DEFAULT_ALL);
	p_rd->draw_list_bind_render_pipeline(list, *pipeline);
	p_rd->draw_list_bind_uniform_set(list, set, 0);
	p_rd->draw_list_set_push_constant(list, p_push, p_push_size);
	p_rd->draw_list_draw(list, false, 1, 3);
	p_rd->draw_list_end();
	p_rd->free_rid(set);
}

void PhysXLensFlareEffect::_render(int p_callback_type, const RenderData *p_render_data) {
	Snapshot s;
	{
		MutexLock lock(mutex);
		if (flares.is_empty()) {
			return;
		}
		s = flares.begin()->value;
	}
	if (p_render_data == nullptr) {
		return;
	}
	RenderSceneBuffersRD *buffers = Object::cast_to<RenderSceneBuffersRD>(p_render_data->get_render_scene_buffers().ptr());
	RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
	if (buffers == nullptr || rd == nullptr || !_ensure(rd)) {
		return;
	}
	const Size2i frame = buffers->get_internal_size();
	if (frame.x < 8 || frame.y < 8 || !buffers->get_internal_texture().is_valid()) {
		return;
	}
	_ensure_targets(rd, frame);

	GhostData ghosts = {};
	for (int i = 0; i < s.ghost_count; i++) {
		ghosts.colors[i][0] = s.ghost_colors[i].r * s.ghost_colors[i].a;
		ghosts.colors[i][1] = s.ghost_colors[i].g * s.ghost_colors[i].a;
		ghosts.colors[i][2] = s.ghost_colors[i].b * s.ghost_colors[i].a;
		ghosts.scales[i] = s.ghost_scales[i];
	}
	rd->buffer_update(ghost_buffer, 0, sizeof(ghosts), &ghosts);

	Push push = {};
	// The Mobile renderer (without HDR 2D) stores color halved, so 0..2 fits its buffer: read it back up, and keep the
	// threshold under what it can hold.
	const float luminance = buffers->get_luminance_multiplier();
	push.misc[0] = luminance > 1.0f ? MIN(s.threshold, 0.75f * luminance) : s.threshold;
	push.misc[1] = MAX(s.threshold_range, 0.01f);
	push.misc[2] = luminance;
	push.misc[3] = (float)frame.x / (float)frame.y;
	push.ghost[0] = s.ghost_intensity;
	push.ghost[1] = s.ghost_chroma_shift;
	push.ghost[2] = (float)s.ghost_count;
	push.halo[0] = s.halo_intensity;
	push.halo[1] = s.halo_width;
	push.halo[2] = s.halo_mask;
	push.halo[3] = s.halo_compression;
	push.color[0] = s.color.r;
	push.color[1] = s.color.g;
	push.color[2] = s.color.b;
	push.color[3] = s.halo_chroma_shift;

	// An XR camera renders its eyes as the layers of one frame: each eye flares
	// on its own slice. The flare mirrors through the screen's centre; an eye's
	// centre points a little away from the head's straight ahead, so each eye
	// mirrors through where straight ahead is in it instead -- the same
	// direction for both, so their ghosts agree (through their own centres
	// they'd land on opposite sides, at a depth that doesn't fuse).
	const uint32_t view_count = MAX(buffers->get_view_count(), 1u);
	RenderSceneDataRD *scene = Object::cast_to<RenderSceneDataRD>(p_render_data->get_render_scene_data());
	for (uint32_t v = 0; v < view_count; v++) {
		const RID color = view_count > 1 ? buffers->get_internal_texture(v) : buffers->get_internal_texture();
		if (!color.is_valid()) {
			continue;
		}
		push.centre[0] = 0.5f;
		push.centre[1] = 0.5f;
		if (view_count > 1 && scene != nullptr) {
			const Vector4 clip = scene->get_view_projection(v).xform(Vector4(0.0, 0.0, -1.0, 0.0));
			if (clip.w > 1e-4) {
				push.centre[0] = (float)(clip.x / clip.w * 0.5 + 0.5);
				push.centre[1] = (float)(clip.y / clip.w * 0.5 + 0.5);
			}
		}
		_render_view(rd, frame, color, &push, sizeof(push));
	}
}

void PhysXLensFlareEffect::_render_view(RenderingDevice *p_rd, const Size2i &p_frame, const RID &p_color, const void *p_push, uint32_t p_push_size) {
	RenderingDevice *rd = p_rd;
	const Size2i frame = p_frame;
	const RID color = p_color;
	ERR_FAIL_COND(p_push_size != sizeof(Push));
	Push push = *(const Push *)p_push;
	auto sized = [&](const Size2i &p_out, const Size2i &p_in) {
		push.size[0] = p_out.x;
		push.size[1] = p_out.y;
		push.size[2] = p_in.x;
		push.size[3] = p_in.y;
	};
	sized(quarter, frame);
	_dispatch(rd, PASS_PREPARE, { RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ linear_sampler, color })), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, bright) }, quarter, &push, sizeof(push));
	// Down to a sixteenth of the frame, then back up to a quarter.
	auto sampled = [&](const RID &p_texture) {
		return RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ linear_sampler, p_texture }));
	};
	RID from = bright;
	Size2i from_size = quarter;
	for (int i = 0; i < BLUR_LEVELS; i++) {
		sized(down_size[i], from_size);
		_dispatch(rd, PASS_DOWN, { sampled(from), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, down[i]) }, down_size[i], &push, sizeof(push));
		from = down[i];
		from_size = down_size[i];
	}
	for (int i = BLUR_LEVELS - 2; i >= -1; i--) {
		const RID to = i >= 0 ? up[i] : soft;
		const Size2i to_size = i >= 0 ? down_size[i] : quarter;
		sized(to_size, from_size);
		_dispatch(rd, PASS_UP, { sampled(from), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, to) }, to_size, &push, sizeof(push));
		from = to;
		from_size = to_size;
	}
	sized(quarter, quarter);
	_dispatch(rd, PASS_GHOSTS, { RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ border_sampler, soft })), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, flare), RD::Uniform(RD::UNIFORM_TYPE_UNIFORM_BUFFER, 2, ghost_buffer) }, quarter, &push, sizeof(push));
	sized(frame, quarter);
	_composite(rd, color, &push, sizeof(push));
}

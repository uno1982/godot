/**************************************************************************/
/*  physx_flow_render_effect.cpp                                          */
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

#include "physx_flow_render_effect.h"

#include "../godot_physx_render_thread.h"
#include "../nodes/physx_compositor_effects.h"
#include "../nodes/physx_light_shafts_effect.h"
#include "flow_backend.h"
#include "physx_flow_3d.h"

#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "scene/3d/camera_3d.h"
#include "scene/resources/3d/world_3d.h"
#include "servers/rendering/renderer_rd/framebuffer_cache_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_data_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_buffers_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_data_rd.h"
#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"
#include "servers/rendering/storage/render_data.h"

LocalVector<PhysXFlow3D *> PhysXFlowRenderEffect::flows;
Mutex PhysXFlowRenderEffect::flows_mutex;
HashMap<ObjectID, int> PhysXFlowRenderEffect::worlds;

namespace {

Ref<PhysXFlowRenderEffect> shared_effect;

const char *COPY_SHADER = R"(
#[compute]
#version 450

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

// The source is sampled, not loaded as an image: the Mobile renderer's color
// buffer can't be a storage image.
layout(set = 0, binding = 0) uniform sampler2D src;
layout(set = 0, binding = 1, rgba16f) uniform restrict writeonly image2D dst;

layout(push_constant, std430) uniform Params {
	ivec2 size;
	float scale; // on rgb
	float pad;
} params;

void main() {
	ivec2 p = ivec2(gl_GlobalInvocationID.xy);
	if (p.x >= params.size.x || p.y >= params.size.y) {
		return;
	}
	vec4 c = texelFetch(src, p, 0);
	imageStore(dst, p, vec4(c.rgb * params.scale, c.a));
}
)";

// Writes the gas, drawn on a copy of the frame, back into a frame that can't be
// written from a compute pass (the Mobile renderer's): a fullscreen draw.
const char *WRITE_BACK_SHADER = R"(
#[vertex]
#version 450
void main() {
	vec2 p = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
	gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
#[fragment]
#version 450
layout(set = 0, binding = 0) uniform sampler2D src;
layout(push_constant, std430) uniform Params {
	float scale; // on rgb
	float pad[3];
} params;
layout(location = 0) out vec4 frag;
void main() {
	vec4 c = texelFetch(src, ivec2(gl_FragCoord.xy), 0);
	frag = vec4(c.rgb * params.scale, c.a);
}
)";

// Godot's column-vector matrix as Flow's row-vector one: Flow's row i is
// Godot's column i, which is exactly Projection's memory layout.
void to_flow(const Projection &p_m, float r_out[16]) {
	for (int c = 0; c < 4; c++) {
		for (int r = 0; r < 4; r++) {
			r_out[c * 4 + r] = (float)p_m.columns[c][r];
		}
	}
}

} // namespace

void PhysXFlowRenderEffect::register_flow(PhysXFlow3D *p_flow) {
	MutexLock lock(flows_mutex);
	if (flows.find(p_flow) < 0) {
		flows.push_back(p_flow);
	}
}

void PhysXFlowRenderEffect::unregister_flow(PhysXFlow3D *p_flow) {
	// Waits for a frame drawing it: after this the render thread never touches it.
	MutexLock lock(flows_mutex);
	flows.erase(p_flow);
}

namespace {

} // namespace

void PhysXFlowRenderEffect::attach_world(World3D *p_world) {
	ERR_FAIL_NULL(p_world);
	worlds[p_world->get_instance_id()]++;
}

void PhysXFlowRenderEffect::detach_world(const ObjectID &p_world) {
	int *count = worlds.getptr(p_world);
	if (count == nullptr || --(*count) > 0) {
		return;
	}
	worlds.erase(p_world);
	PhysXCompositorEffects::uninstall(p_world, get_shared());
}

void PhysXFlowRenderEffect::install(World3D *p_world, Camera3D *p_camera) {
	ERR_FAIL_NULL(p_world);
	ERR_FAIL_COND(!worlds.has(p_world->get_instance_id()));
	PhysXCompositorEffects::install(p_world, p_camera, get_shared(), PhysXCompositorEffects::ORDER_FLOW);
}

Ref<Compositor> PhysXFlowRenderEffect::get_world_compositor(const Ref<World3D> &p_world) {
	return p_world.is_valid() ? p_world->get_compositor() : Ref<Compositor>();
}

void PhysXFlowRenderEffect::_bind_methods() {
	ClassDB::bind_static_method("PhysXFlowRenderEffect", D_METHOD("get_world_compositor", "world"), &PhysXFlowRenderEffect::get_world_compositor);
}

Ref<PhysXFlowRenderEffect> PhysXFlowRenderEffect::get_shared() {
	if (shared_effect.is_null()) {
		shared_effect.instantiate();
	}
	return shared_effect;
}

void PhysXFlowRenderEffect::free_shared() {
	shared_effect.unref();
}

PhysXFlowRenderEffect::PhysXFlowRenderEffect() {
	set_effect_callback_type(EFFECT_CALLBACK_TYPE_POST_TRANSPARENT);
	set_access_resolved_color(true);
	set_access_resolved_depth(true);
	// Straight to C++, instead of the script-facing _render_callback.
	RenderingServer::get_singleton()->compositor_effect_set_callback(get_rid(), RSE::COMPOSITOR_EFFECT_CALLBACK_TYPE_POST_TRANSPARENT, callable_mp(this, &PhysXFlowRenderEffect::_render));
}

PhysXFlowRenderEffect::~PhysXFlowRenderEffect() {
	// Freed on the render thread, wherever the last reference goes.
	godot_physx_run_on_render_thread([this]() {
		RenderingDevice *rd = RenderingServer::get_singleton() ? RenderingServer::get_singleton()->get_rendering_device() : nullptr;
		if (rd != nullptr) {
			if (copy_pipeline.is_valid()) {
				rd->free_rid(copy_pipeline);
			}
			for (const KeyValue<int64_t, RID> &kv : write_back_pipelines) {
				rd->free_rid(kv.value);
			}
			for (RID r : { copy_shader, copy_sampler, write_back_shader, work }) {
				if (r.is_valid()) {
					rd->free_rid(r);
				}
			}
			scatter.free(rd);
		}
	});
}

bool PhysXFlowRenderEffect::_ensure_copy_pipeline(RenderingDevice *p_rd) {
	if (copy_pipeline.is_valid()) {
		return true;
	}
	if (copy_failed) {
		return false;
	}
	Ref<RDShaderFile> file;
	file.instantiate();
	if (file->parse_versions_from_text(COPY_SHADER) != OK) {
		copy_failed = true;
		ERR_FAIL_V_MSG(false, "PhysXFlowRenderEffect: copy shader failed to compile.");
	}
	copy_shader = p_rd->shader_create_from_spirv(file->get_spirv_stages());
	if (copy_shader.is_valid()) {
		copy_pipeline = p_rd->compute_pipeline_create(copy_shader);
	}
	Ref<RDShaderFile> write_file;
	write_file.instantiate();
	if (write_file->parse_versions_from_text(WRITE_BACK_SHADER) == OK) {
		write_back_shader = p_rd->shader_create_from_spirv(write_file->get_spirv_stages());
	}
	copy_sampler = p_rd->sampler_create(RD::SamplerState());
	copy_failed = !copy_pipeline.is_valid() || !write_back_shader.is_valid();
	ERR_FAIL_COND_V_MSG(copy_failed, false, "PhysXFlowRenderEffect: copy shaders failed to compile.");
	return true;
}

void PhysXFlowRenderEffect::copy_texture(RenderingDevice *p_rd, const RID &p_src, const RID &p_dst, const Size2i &p_size, float p_scale) {
	if (!_ensure_copy_pipeline(p_rd)) {
		return;
	}
	RD::Uniform src(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ copy_sampler, p_src }));
	RD::Uniform dst(RD::UNIFORM_TYPE_IMAGE, 1, p_dst);
	LocalVector<RD::Uniform> uniforms;
	uniforms.push_back(src);
	uniforms.push_back(dst);
	const RID set = p_rd->uniform_set_create(uniforms, copy_shader, 0);
	ERR_FAIL_COND(!set.is_valid());
	struct {
		int32_t size[2];
		float scale;
		float pad;
	} push = { { p_size.x, p_size.y }, p_scale, 0.0f };
	const RD::ComputeListID list = p_rd->compute_list_begin();
	p_rd->compute_list_bind_compute_pipeline(list, copy_pipeline);
	p_rd->compute_list_bind_uniform_set(list, set, 0);
	p_rd->compute_list_set_push_constant(list, &push, sizeof(push));
	p_rd->compute_list_dispatch(list, (p_size.x + 7) / 8, (p_size.y + 7) / 8, 1);
	p_rd->compute_list_end();
	p_rd->free_rid(set);
}

void PhysXFlowRenderEffect::_write_back(RenderingDevice *p_rd, const RID &p_src, const RID &p_color, float p_scale) {
	const RID framebuffer = FramebufferCacheRD::get_singleton()->get_cache(p_color);
	ERR_FAIL_COND(!framebuffer.is_valid());
	const RD::FramebufferFormatID format = p_rd->framebuffer_get_format(framebuffer);
	RID *pipeline = write_back_pipelines.getptr(format);
	if (pipeline == nullptr) {
		const RID created = p_rd->render_pipeline_create(write_back_shader, format, RD::INVALID_ID, RD::RENDER_PRIMITIVE_TRIANGLES, RD::PipelineRasterizationState(), RD::PipelineMultisampleState(), RD::PipelineDepthStencilState(), RD::PipelineColorBlendState::create_disabled());
		ERR_FAIL_COND(!created.is_valid());
		pipeline = &write_back_pipelines.insert(format, created)->value;
	}
	const RID set = p_rd->uniform_set_create(Vector<RD::Uniform>({ RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ copy_sampler, p_src })) }), write_back_shader, 0);
	ERR_FAIL_COND(!set.is_valid());
	const float push[4] = { p_scale, 0.0f, 0.0f, 0.0f };
	const RD::DrawListID list = p_rd->draw_list_begin(framebuffer, RD::DRAW_DEFAULT_ALL);
	p_rd->draw_list_bind_render_pipeline(list, *pipeline);
	p_rd->draw_list_bind_uniform_set(list, set, 0);
	p_rd->draw_list_set_push_constant(list, push, sizeof(push));
	p_rd->draw_list_draw(list, false, 1, 3);
	p_rd->draw_list_end();
	p_rd->free_rid(set);
}

void PhysXFlowRenderEffect::_render(int p_callback_type, const RenderData *p_render_data) {
	MutexLock lock(flows_mutex);
	if (flows.is_empty() || p_render_data == nullptr) {
		return;
	}
	RenderSceneBuffersRD *buffers = Object::cast_to<RenderSceneBuffersRD>(p_render_data->get_render_scene_buffers().ptr());
	RenderSceneDataRD *scene = Object::cast_to<RenderSceneDataRD>(p_render_data->get_render_scene_data());
	RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
	if (buffers == nullptr || scene == nullptr || rd == nullptr) {
		return;
	}
	const Size2i size = buffers->get_internal_size();
	// An XR camera renders its eyes as the layers of one frame, each with its
	// own projection (on top of the shared camera transform): every eye gets
	// its own pass, on its own slice of the color and depth.
	const uint32_t view_count = MAX(buffers->get_view_count(), 1u);
	if (size.x <= 0 || size.y <= 0 || !buffers->get_internal_texture().is_valid() || !buffers->get_depth_texture().is_valid()) {
		return;
	}
	if (!_ensure_copy_pipeline(rd)) {
		return;
	}

	// Each flow composites over what's drawn so far, so far ones go first.
	const Vector3 eye = scene->get_cam_transform().origin;
	LocalVector<PhysXFlow3D *> sorted;
	LocalVector<real_t> distance;
	for (PhysXFlow3D *flow : flows) {
		sorted.push_back(flow);
		distance.push_back(eye.distance_squared_to(flow->get_draw_center()));
	}
	for (uint32_t i = 1; i < sorted.size(); i++) {
		for (uint32_t j = i; j > 0 && distance[j - 1] < distance[j]; j--) {
			SWAP(distance[j - 1], distance[j]);
			SWAP(sorted[j - 1], sorted[j]);
		}
	}
	// Flow's world is in its own units: scale down to meters first.
	Projection to_meters;
	const real_t inv = 1.0 / FlowBackend::UNITS_PER_METER;
	to_meters.columns[0][0] = inv;
	to_meters.columns[1][1] = inv;
	to_meters.columns[2][2] = inv;
	const Projection world_to_camera = Projection(scene->get_cam_transform().affine_inverse());
	const Projection view = world_to_camera * to_meters;
	float view_f[16];
	to_flow(view, view_f);
	// Flow reads its targets with row 0 at clip-space +Y; Godot's are the
	// other way up (checked on screen: without this the plume is upside down).
	Projection y_flip;
	y_flip.columns[1][1] = -1.0;
	// The frame's lights and shadow atlas, for the scattering pass's shadows.
	const RenderDataRD *render_data_rd = dynamic_cast<const RenderDataRD *>(p_render_data);
	const float luminance = buffers->get_luminance_multiplier();

	for (uint32_t v = 0; v < view_count; v++) {
		const RID color = view_count > 1 ? buffers->get_internal_texture(v) : buffers->get_internal_texture();
		// (A screen-space fluid has written its surface into this depth: the gas
		// is marched up to the liquid.)
		const RID depth = view_count > 1 ? buffers->get_depth_texture(v) : buffers->get_depth_texture();
		if (!color.is_valid() || !depth.is_valid()) {
			continue;
		}
		// The projection the depth buffer was rendered with: Godot's camera
		// projection already carries its depth correction (reverse Z in 0..1, Y
		// as this target needs) and TAA jitter, so Flow reconstructs scene
		// positions from depth exactly. An eye's has the eye's offset in it.
		const Projection camera_projection = view_count > 1 ? scene->get_view_projection(v) : scene->get_cam_projection();
		float projection_f[16];
		to_flow(y_flip * camera_projection, projection_f);
		// For the light scattering pass: clip space (as the depth buffer has it)
		// straight back to world meters.
		const Projection inv_view_projection = (camera_projection * world_to_camera).inverse();
		const Vector3 view_eye = view_count > 1 ? scene->get_cam_transform().xform(scene->get_view_eye_offset(v)) : eye;

		// The gas is drawn straight into the frame when it can be written from a
		// compute pass (Forward+). The Mobile renderer's can't, and holds color
		// halved: the gas is drawn on a full-brightness RGBA16F copy then, written
		// back after.
		const bool direct = rd->texture_get_format(color).usage_bits & RD::TEXTURE_USAGE_STORAGE_BIT;
		RID target = color;
		if (!direct) {
			if (!work.is_valid() || work_size != size) {
				if (work.is_valid()) {
					rd->free_rid(work);
				}
				RD::TextureFormat tf;
				tf.width = size.x;
				tf.height = size.y;
				tf.format = RD::DATA_FORMAT_R16G16B16A16_SFLOAT;
				tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT;
				work = rd->texture_create(tf, RD::TextureView());
				work_size = size;
			}
			copy_texture(rd, color, work, size, luminance);
			target = work;
		}
		// The light shafts (drawn next) take their light from the frame without the
		// gas: glowing gas isn't the sun. (As the frame holds it: halved on Mobile.)
		// Each view has its own.
		const RID backdrop = PhysXLightShaftsEffect::gas_backdrop(rd, size, v);
		if (backdrop.is_valid()) {
			copy_texture(rd, target, backdrop, size, direct ? 1.0f : 1.0f / luminance);
		}
		for (PhysXFlow3D *flow : sorted) {
			flow->render_rd(this, rd, view_f, projection_f, size, depth, target, inv_view_projection, view_eye, render_data_rd != nullptr ? render_data_rd->lights : nullptr, render_data_rd != nullptr ? render_data_rd->shadow_atlas : RID(), v == 0, v + 1 == view_count);
		}
		if (!direct) {
			_write_back(rd, work, color, 1.0f / luminance);
		}
	}
}

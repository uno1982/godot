/**************************************************************************/
/*  physx_groom_render_effect.cpp                                         */
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

#include "physx_groom_render_effect.h"

#include "../godot_physx_render_thread.h"
#include "../nodes/physx_compositor_effects.h"
#include "groom_shading.h"

#include "core/config/engine.h"
#include "core/object/callable_mp.h"
#include "scene/3d/camera_3d.h"
#include "scene/resources/3d/primitive_meshes.h"
#include "scene/resources/3d/world_3d.h"
#include "scene/resources/material.h"
#include "scene/resources/shader.h"
#include "scene/resources/texture_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_buffers_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_data_rd.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"
#include "servers/rendering/storage/render_data.h"

Mutex PhysXGroomRenderEffect::mutex;
HashMap<ObjectID, PhysXGroomRenderEffect::Groom> PhysXGroomRenderEffect::grooms;
HashMap<ObjectID, int> PhysXGroomRenderEffect::worlds;
HashMap<ObjectID, RID> PhysXGroomRenderEffect::proxies[LAYERS];
Ref<QuadMesh> PhysXGroomRenderEffect::proxy_meshes[LAYERS];
Ref<ShaderMaterial> PhysXGroomRenderEffect::proxy_materials[LAYERS];
Ref<Shader> PhysXGroomRenderEffect::proxy_shader;
Ref<Texture2DRD> PhysXGroomRenderEffect::hair_textures[TEXTURES];
RID PhysXGroomRenderEffect::published[TEXTURES];
bool PhysXGroomRenderEffect::textures_hooked = false;
bool PhysXGroomRenderEffect::proxies_shown = false;
uint64_t PhysXGroomRenderEffect::ready_since = UINT64_MAX;

namespace {

Ref<PhysXGroomRenderEffect> shared_effect;

constexpr uint32_t PARAMS_FLOATS = 52;

const char *PARAMS = R"(
layout(set = 0, binding = 0, std140) uniform Params {
	mat4 view; // world -> view
	mat4 proj; // view -> clip, as the scene's depth was drawn
	mat4 inv_proj;
	ivec4 frame; // xy: the frame
} p;
)";

const char *PASS_NAMES[] = { "clear", "raster", "composite" };

const char *SHADERS[] = {
	// PASS_CLEAR: no hair anywhere.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 1, r32ui) uniform restrict writeonly uimage2D coverage_img;
layout(set = 0, binding = 2, r32ui) uniform restrict writeonly uimage2D depth_img;
layout(set = 0, binding = 3, r32ui) uniform restrict writeonly uimage2D depth2_img;

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, p.frame.xy))) {
		return;
	}
	imageStore(coverage_img, px, uvec4(0));
	imageStore(depth_img, px, uvec4(floatBitsToUint(1e30)));
	imageStore(depth2_img, px, uvec4(floatBitsToUint(1e30)));
}
)",
	// PASS_RASTER: one strand segment per thread, as a band of its own width (at least a pixel's), each pixel taking the
	// share of it the strand covers. Run three times per groom: adding up the coverage (in 1/256ths of a pixel) and
	// keeping the nearest depth (atomic min of the float's bits: positive floats order like their bits); keeping the
	// second nearest; then the two nearest strands writing what the proxies shade them with.
	R"(
#[compute]
#version 450
layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 1, r32ui) uniform restrict uimage2D coverage_img;
layout(set = 0, binding = 2, r32ui) uniform restrict uimage2D depth_img;
layout(set = 0, binding = 3, std430) restrict readonly buffer Points {
	vec4 data[]; // 3 per point: position, radius | along, rank, last, - | self-shadowing
};
layout(set = 0, binding = 4) uniform sampler2D scene_depth;
layout(set = 0, binding = 5, rgba16f) uniform restrict writeonly image2D hair_a;
layout(set = 0, binding = 6, rgba16f) uniform restrict writeonly image2D hair_c;
layout(set = 0, binding = 7, r32ui) uniform restrict uimage2D depth2_img;
layout(set = 0, binding = 8, rgba16f) uniform restrict writeonly image2D hair_a2;
layout(set = 0, binding = 9, rgba16f) uniform restrict writeonly image2D hair_c2;
layout(set = 0, binding = 10, r32f) uniform restrict writeonly image2D front_cover_img;

layout(push_constant, std430) uniform Groom {
	mat4 model;
	vec4 lod; // x: strands drawn (0-1), y: width scale (the LOD's widening in), z: the model's scale
	ivec4 info; // x: points, y: 0 coverage and the nearest depth, 1 the second nearest, 2 the two strands' attributes
} g;

float scene_eye_depth(ivec2 px) {
	float d = texelFetch(scene_depth, px, 0).r;
	vec2 ndc = (vec2(px) + 0.5) / vec2(p.frame.xy) * 2.0 - 1.0;
	vec4 v = p.inv_proj * vec4(ndc, d, 1.0);
	return -v.z / v.w;
}

vec4 turn_strands(vec4 sh, mat3 rotation) {
	vec3 v = rotation * vec3(sh.w, sh.y, sh.z);
	return vec4(sh.x, v.y, v.z, v.x);
}

vec2 to_pixels(vec3 v) {
	vec4 c = p.proj * vec4(v, 1.0);
	return (c.xy / c.w * 0.5 + 0.5) * vec2(p.frame.xy);
}

// The segment, in view space and on screen, and what interpolates along it.
vec3 va, vb;
vec2 pa, pb;
float wa, wb; // width, px
float s0, s1; // the visible part's ends, as a share of the whole segment
bool last_segment;
uint segment;

void shade_pixel(ivec2 px) {
	if (any(lessThan(px, ivec2(0))) || any(greaterThanEqual(px, p.frame.xy))) {
		return;
	}
	vec2 c = vec2(px) + 0.5;
	vec2 d = pb - pa;
	float s = dot(c - pa, d) / max(dot(d, d), 1e-6);
	// Each pixel once per strand: a segment owns its start, the next one its end.
	if (s < 0.0 || s > 1.0 || (s == 1.0 && !last_segment)) {
		return;
	}
	float w = mix(wa, wb, s);
	float dist = length(c - mix(pa, pb, s));
	float cover = clamp(max(w, 1.0) * 0.5 + 0.5 - dist, 0.0, 1.0) * min(w, 1.0);
	if (cover <= 0.0) {
		return;
	}
	float eye = -mix(va.z, vb.z, s);
	if (eye > scene_eye_depth(px)) {
		return; // behind the scene's own geometry
	}
	if (g.info.y == 0) {
		imageAtomicAdd(coverage_img, px, uint(cover * 256.0 + 0.5));
		imageAtomicMin(depth_img, px, floatBitsToUint(eye));
		return;
	}
	uint nearest = imageLoad(depth_img, px).r;
	if (g.info.y == 1) {
		// Behind the nearest strand by more than float error: the next strand back.
		float front = uintBitsToFloat(nearest);
		if (eye > front + max(front * 1e-4, 1e-5)) {
			imageAtomicMin(depth2_img, px, floatBitsToUint(eye));
		}
		return;
	}
	bool first = nearest == floatBitsToUint(eye);
	if (!first && imageLoad(depth2_img, px).r != floatBitsToUint(eye)) {
		return;
	}
	float so = mix(s0, s1, s);
	vec4 a1 = data[segment * 3u + 1u];
	vec4 b1 = data[segment * 3u + 4u];
	mat3 turn = mat3(p.view * g.model);
	turn = mat3(normalize(turn[0]), normalize(turn[1]), normalize(turn[2]));
	vec4 sh = turn_strands(mix(data[segment * 3u + 2u], data[segment * 3u + 5u], so), turn);
	vec4 a = vec4(normalize(vb - va), mix(a1.x, b1.x, so));
	if (first) {
		imageStore(hair_a, px, a);
		imageStore(hair_c, px, sh);
		imageStore(front_cover_img, px, vec4(cover));
	} else {
		imageStore(hair_a2, px, a);
		imageStore(hair_c2, px, sh);
	}
}

void main() {
	segment = gl_GlobalInvocationID.x;
	if (segment + 1u >= uint(g.info.x)) {
		return;
	}
	vec4 a0 = data[segment * 3u];
	vec4 a1 = data[segment * 3u + 1u];
	if (a1.z > 0.5 || a1.y >= g.lod.x) {
		return; // a strand's last point, or a strand not drawn at this distance
	}
	vec4 b0 = data[segment * 3u + 3u];
	last_segment = data[segment * 3u + 4u].z > 0.5;
	va = (p.view * (g.model * vec4(a0.xyz, 1.0))).xyz;
	vb = (p.view * (g.model * vec4(b0.xyz, 1.0))).xyz;
	// The part in front of the camera.
	const float near = 0.01;
	s0 = 0.0;
	s1 = 1.0;
	if (va.z > -near && vb.z > -near) {
		return;
	}
	if (va.z > -near) {
		s0 = (-near - va.z) / (vb.z - va.z);
	} else if (vb.z > -near) {
		s1 = (-near - va.z) / (vb.z - va.z);
	}
	vec3 v0 = mix(va, vb, s0);
	vec3 v1 = mix(va, vb, s1);
	float ra = mix(a0.w, b0.w, s0) * g.lod.y * g.lod.z;
	float rb = mix(a0.w, b0.w, s1) * g.lod.y * g.lod.z;
	va = v0;
	vb = v1;
	pa = to_pixels(va);
	pb = to_pixels(vb);
	float to_px = abs(p.proj[1][1]) * float(p.frame.y);
	wa = 2.0 * ra * to_px * 0.5 / -va.z;
	wb = 2.0 * rb * to_px * 0.5 / -vb.z;

	// Scan along the segment's longer axis, each column (row) through the band across it.
	vec2 d = pb - pa;
	float ext = max(max(wa, wb), 1.0) * 0.5 + 1.0;
	bool along_x = abs(d.x) >= abs(d.y);
	float major_a = along_x ? pa.x : pa.y;
	float major_b = along_x ? pb.x : pb.y;
	float minor_a = along_x ? pa.y : pa.x;
	float slope = (along_x ? d.y : d.x) / max(abs(along_x ? d.x : d.y), 1e-6) * sign(along_x ? d.x : d.y);
	if (abs(along_x ? d.x : d.y) < 1e-6) {
		slope = 0.0;
	}
	int m0 = int(floor(min(major_a, major_b) - ext));
	int m1 = int(ceil(max(major_a, major_b) + ext));
	int frame_major = along_x ? p.frame.x : p.frame.y;
	m0 = max(m0, 0);
	m1 = min(m1, frame_major - 1);
	if (m1 - m0 > 4096) {
		return;
	}
	float span = ext * sqrt(1.0 + slope * slope);
	for (int m = m0; m <= m1; m++) {
		float center = minor_a + slope * (float(m) + 0.5 - major_a);
		int n0 = int(floor(center - span));
		int n1 = int(ceil(center + span));
		for (int n = n0; n <= n1; n++) {
			shade_pixel(along_x ? ivec2(m, n) : ivec2(n, m));
		}
	}
}
)",
	// PASS_COMPOSITE: the added-up coverage as the share of the pixel the hair hides. Strands overlap at random, so a
	// pixel two strands each half cover is three quarters covered, not all of it: Indiana Jones and the Great
	// Circle's fit of that, x - x^2/4 up to x = 2, then full.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 1, r32ui) uniform restrict readonly uimage2D coverage_img;
layout(set = 0, binding = 2, r32ui) uniform restrict readonly uimage2D depth_img;
layout(set = 0, binding = 3, rgba16f) uniform restrict writeonly image2D hair_b;
layout(set = 0, binding = 4, r32ui) uniform restrict readonly uimage2D depth2_img;
layout(set = 0, binding = 5, r32f) uniform restrict readonly image2D front_cover_img;

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, p.frame.xy))) {
		return;
	}
	float x = float(imageLoad(coverage_img, px).r) / 256.0;
	float cover = clamp(x - 0.5 * x * clamp(0.5 * x, 0.0, 1.0), 0.0, 1.0);
	float eye = uintBitsToFloat(imageLoad(depth_img, px).r);
	// The front strand's own share of the pixel, no more than the hair's: the rest is the strands behind it.
	float front = cover > 0.0 ? min(imageLoad(front_cover_img, px).r, cover) : 0.0;
	float eye2 = uintBitsToFloat(imageLoad(depth2_img, px).r);
	imageStore(hair_b, px, vec4(eye, cover, front, eye2));
}
)",
};

// The hair the compute passes left, shaded in Godot's transparent pass (first there) on two quads over the screen, lit by
// the scene's lights and shadows with the hair lighting: layer 1, the strand behind, over what the hair covers beyond the
// front strand (the front strand again where there's none behind it); then layer 0, the front strand, over its own
// share of the pixel. Together they cover what the hair does.
const char *PROXY_SHADER_HEAD = R"(
shader_type spatial;
render_mode blend_mix, depth_draw_never, depth_test_disabled, cull_disabled, skip_vertex_transform;
)";
const char *PROXY_SHADER_BODY = R"(
uniform sampler2D hair_a : filter_nearest, repeat_disable;
uniform sampler2D hair_b : filter_nearest, repeat_disable;
uniform sampler2D hair_c : filter_nearest, repeat_disable;
uniform sampler2D hair_a2 : filter_nearest, repeat_disable;
uniform sampler2D hair_c2 : filter_nearest, repeat_disable;
uniform int layer = 0;

void vertex() {
	// The quad's corners on the camera's frustum: each pixel's VERTEX (and so VIEW) lies on its own ray.
	vec4 v = INV_PROJECTION_MATRIX * vec4(VERTEX.xy, 0.5, 1.0);
	VERTEX = v.xyz / v.w;
	POSITION = PROJECTION_MATRIX * vec4(VERTEX, 1.0);
}

void fragment() {
	vec4 b = texture(hair_b, SCREEN_UV);
	float cover = b.y;
	float front = b.z;
	float alpha;
	float eye;
	vec4 a;
	vec4 sh;
	if (layer == 0) {
		alpha = front;
		eye = b.x;
		a = texture(hair_a, SCREEN_UV);
		sh = texture(hair_c, SCREEN_UV);
	} else {
		// Under the front strand's share, blended over: what covers the rest of the hair's.
		alpha = front < 0.999 ? max(cover - front, 0.0) / (1.0 - front) : 0.0;
		bool behind = b.w < 1e29;
		eye = behind ? b.w : b.x;
		a = behind ? texture(hair_a2, SCREEN_UV) : texture(hair_a, SCREEN_UV);
		sh = behind ? texture(hair_c2, SCREEN_UV) : texture(hair_c, SCREEN_UV);
	}
	if (alpha <= 0.002) {
		discard;
	}
	vec4 ray = INV_PROJECTION_MATRIX * vec4(SCREEN_UV * 2.0 - 1.0, 0.5, 1.0);
	vec3 dir = ray.xyz / ray.w;
	vec3 at = dir * (eye / -dir.z);
	LIGHT_VERTEX = at;
	vec3 t = normalize(a.xyz);
	vec3 v = normalize(-at);
	vec3 facing = v - t * dot(v, t);
	NORMAL = dot(facing, facing) > 1e-6 ? normalize(facing) : v;
	ALBEDO = strand_albedo(a.w);
	if (shading_model == 1) {
		ROUGHNESS = hair_roughness;
		SPECULAR = 0.0;
	} else {
		ROUGHNESS = 0.6;
	}
	light_tangent = t;
	light_shadow = sh;
	AO = exp(-self_shadow * max(0.0, light_shadow.x * 0.282095));
	ALPHA = alpha;
}
)";

void put_matrix(float *r_dst, const Projection &p_m) {
	for (int c = 0; c < 4; c++) {
		for (int r = 0; r < 4; r++) {
			r_dst[c * 4 + r] = (float)p_m.columns[c][r];
		}
	}
}

} // namespace

void PhysXGroomRenderEffect::set_points(const ObjectID &p_groom, uint64_t p_version, const PackedFloat32Array &p_points) {
	MutexLock lock(mutex);
	Groom &groom = grooms[p_groom];
	groom.version = p_version;
	groom.points = p_points;
}

void PhysXGroomRenderEffect::set_state(const ObjectID &p_groom, const Transform3D &p_transform, float p_strand_fraction, float p_width_scale, const Look &p_look) {
	MutexLock lock(mutex);
	Groom &groom = grooms[p_groom];
	groom.transform = p_transform;
	groom.strand_fraction = p_strand_fraction;
	groom.width_scale = p_width_scale;
	groom.look = p_look;
}

void PhysXGroomRenderEffect::remove_groom(const ObjectID &p_groom) {
	MutexLock lock(mutex);
	grooms.erase(p_groom);
}

void PhysXGroomRenderEffect::attach_world(World3D *p_world) {
	ERR_FAIL_NULL(p_world);
	if (worlds[p_world->get_instance_id()]++ > 0) {
		return;
	}
	if (proxy_shader.is_null()) {
		proxy_shader.instantiate();
		proxy_shader->set_code(String(PROXY_SHADER_HEAD) + GROOM_SHADING_COMMON + PROXY_SHADER_BODY + GROOM_SHADING_LIGHT);
		const char *names[TEXTURES] = { "hair_a", "hair_b", "hair_c", "hair_a2", "hair_c2" };
		for (int i = 0; i < TEXTURES; i++) {
			hair_textures[i].instantiate();
		}
		for (int l = 0; l < LAYERS; l++) {
			proxy_materials[l].instantiate();
			proxy_materials[l]->set_shader(proxy_shader);
			for (int i = 0; i < TEXTURES; i++) {
				proxy_materials[l]->set_shader_parameter(names[i], hair_textures[i]);
			}
			proxy_materials[l]->set_shader_parameter("layer", l);
			// First among transparent geometry, the strand behind before the one in front.
			proxy_materials[l]->set_render_priority(l == 1 ? Material::RENDER_PRIORITY_MIN : Material::RENDER_PRIORITY_MIN + 1);
			proxy_meshes[l].instantiate();
			proxy_meshes[l]->set_size(Size2(2, 2));
			proxy_meshes[l]->set_material(proxy_materials[l]);
		}
	}
	RenderingServer *rs = RenderingServer::get_singleton();
	for (int l = 0; l < LAYERS; l++) {
		const RID instance = rs->instance_create2(proxy_meshes[l]->get_rid(), p_world->get_scenario());
		rs->instance_set_custom_aabb(instance, AABB(Vector3(-1e6, -1e6, -1e6), Vector3(2e6, 2e6, 2e6)));
		rs->instance_geometry_set_cast_shadows_setting(instance, RSE::SHADOW_CASTING_SETTING_OFF);
		rs->instance_set_ignore_culling(instance, true);
		// Hidden until its textures are hooked up (see _update_proxy). A world joining after that (the hair hidden and
		// shown again) shows it as the others are: _update_proxy only shows or hides them when that changes.
		rs->instance_set_visible(instance, proxies_shown);
		proxies[l][p_world->get_instance_id()] = instance;
	}
}

void PhysXGroomRenderEffect::detach_world(const ObjectID &p_world) {
	int *count = worlds.getptr(p_world);
	if (count == nullptr || --(*count) > 0) {
		return;
	}
	worlds.erase(p_world);
	PhysXCompositorEffects::uninstall(p_world, get_shared());
	for (int l = 0; l < LAYERS; l++) {
		if (proxies[l].has(p_world)) {
			RenderingServer::get_singleton()->free_rid(proxies[l][p_world]);
			proxies[l].erase(p_world);
		}
	}
}

void PhysXGroomRenderEffect::install(World3D *p_world, Camera3D *p_camera) {
	ERR_FAIL_NULL(p_world);
	ERR_FAIL_COND(!worlds.has(p_world->get_instance_id()));
	PhysXCompositorEffects::install(p_world, p_camera, get_shared(), PhysXCompositorEffects::ORDER_GROOM);
	_update_proxy(p_camera);
}

void PhysXGroomRenderEffect::_update_proxy(Camera3D *p_camera) {
	if (proxy_shader.is_null()) {
		return;
	}
	RID textures[TEXTURES];
	Look look;
	bool found = false;
	{
		MutexLock lock(mutex);
		for (int i = 0; i < TEXTURES; i++) {
			textures[i] = published[i];
		}
		// The look of the groom nearest the camera.
		const Vector3 eye = p_camera != nullptr ? p_camera->get_global_position() : Vector3();
		real_t best = 1e30;
		for (const KeyValue<ObjectID, Groom> &kv : grooms) {
			const real_t d = eye.distance_squared_to(kv.value.transform.origin);
			if (d < best) {
				best = d;
				look = kv.value.look;
				found = true;
			}
		}
	}
	bool hooked = true;
	for (int i = 0; i < TEXTURES; i++) {
		if (textures[i].is_valid() && hair_textures[i]->get_texture_rd_rid() != textures[i]) {
			hair_textures[i]->set_texture_rd_rid(textures[i]);
		}
		hooked = hooked && hair_textures[i]->get_texture_rd_rid().is_valid();
	}
	textures_hooked = textures_hooked || hooked;
	// Shown two frames after the textures are hooked up (the hookup lands on the render thread a frame later).
	const uint64_t now = Engine::get_singleton()->get_frames_drawn();
	if (!(textures_hooked && found)) {
		ready_since = UINT64_MAX;
	} else if (ready_since == UINT64_MAX) {
		ready_since = now;
	}
	const bool show = ready_since != UINT64_MAX && now >= ready_since + 2;
	if (show != proxies_shown) {
		proxies_shown = show;
		for (int l = 0; l < LAYERS; l++) {
			for (const KeyValue<ObjectID, RID> &kv : proxies[l]) {
				RenderingServer::get_singleton()->instance_set_visible(kv.value, show);
			}
		}
	}
	if (!found) {
		return;
	}
	for (int l = 0; l < LAYERS; l++) {
		proxy_materials[l]->set_shader_parameter("root_color", look.root_color);
		proxy_materials[l]->set_shader_parameter("tip_color", look.tip_color);
		proxy_materials[l]->set_shader_parameter("self_shadow", look.self_shadow);
		proxy_materials[l]->set_shader_parameter("hair_roughness", look.roughness);
		proxy_materials[l]->set_shader_parameter("shading_model", look.shading_model);
	}
}

Ref<PhysXGroomRenderEffect> PhysXGroomRenderEffect::get_shared() {
	if (shared_effect.is_null()) {
		shared_effect.instantiate();
	}
	return shared_effect;
}

void PhysXGroomRenderEffect::free_shared() {
	for (int l = 0; l < LAYERS; l++) {
		for (const KeyValue<ObjectID, RID> &kv : proxies[l]) {
			RenderingServer::get_singleton()->free_rid(kv.value);
		}
		proxies[l].clear();
		proxy_meshes[l].unref();
		proxy_materials[l].unref();
	}
	proxy_shader.unref();
	for (int i = 0; i < TEXTURES; i++) {
		hair_textures[i].unref();
	}
	textures_hooked = false;
	proxies_shown = false;
	ready_since = UINT64_MAX;
	shared_effect.unref();
}

PhysXGroomRenderEffect::PhysXGroomRenderEffect() {
	set_effect_callback_type(EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT);
	set_access_resolved_depth(true);
	RenderingServer::get_singleton()->compositor_effect_set_callback(get_rid(), RSE::COMPOSITOR_EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT, callable_mp(this, &PhysXGroomRenderEffect::_render));
}

PhysXGroomRenderEffect::~PhysXGroomRenderEffect() {
	// Freed on the render thread, wherever the last reference goes.
	godot_physx_run_on_render_thread([this]() {
		RenderingDevice *rd = RenderingServer::get_singleton() ? RenderingServer::get_singleton()->get_rendering_device() : nullptr;
		if (rd == nullptr) {
			return;
		}
		_free_targets(rd, true);
		for (const Retired &r : retired) {
			rd->free_rid(r.rid);
		}
		for (const KeyValue<ObjectID, GpuGroom> &kv : gpu_grooms) {
			rd->free_rid(kv.value.buffer);
		}
		for (int i = 0; i < PASS_MAX; i++) {
			if (pipelines[i].is_valid()) {
				rd->free_rid(pipelines[i]);
			}
			if (shaders[i].is_valid()) {
				rd->free_rid(shaders[i]);
			}
		}
		for (RID r : { point_sampler, params_buffer }) {
			if (r.is_valid()) {
				rd->free_rid(r);
			}
		}
	});
}

bool PhysXGroomRenderEffect::_ensure(RenderingDevice *p_rd) {
	if (pipelines[PASS_COMPOSITE].is_valid()) {
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
		ERR_FAIL_COND_V_MSG(file->parse_versions_from_text(source) != OK, false, vformat("PhysXGroom3D: screen-space %s shader failed to compile.", PASS_NAMES[i]));
		Ref<RDShaderSPIRV> spirv = file->get_spirv();
		ERR_FAIL_COND_V(spirv.is_null(), false);
		const String err = spirv->get_stage_compile_error(RD::SHADER_STAGE_COMPUTE);
		ERR_FAIL_COND_V_MSG(!err.is_empty(), false, vformat("PhysXGroom3D: screen-space %s shader: %s", PASS_NAMES[i], err));
		shaders[i] = p_rd->shader_create_from_spirv(file->get_spirv_stages());
		ERR_FAIL_COND_V(!shaders[i].is_valid(), false);
		pipelines[i] = p_rd->compute_pipeline_create(shaders[i]);
		ERR_FAIL_COND_V(!pipelines[i].is_valid(), false);
	}
	RD::SamplerState point;
	point_sampler = p_rd->sampler_create(point);
	params_buffer = p_rd->uniform_buffer_create(PARAMS_FLOATS * 4);
	failed = false;
	return true;
}

void PhysXGroomRenderEffect::_free_targets(RenderingDevice *p_rd, bool p_now) {
	for (RID *r : { &coverage_u, &depth_u, &depth2_u, &front_cover, &hair[0], &hair[1], &hair[2], &hair[3], &hair[4] }) {
		if (r->is_valid()) {
			if (p_now) {
				p_rd->free_rid(*r);
			} else {
				// The proxy material may still point at it until the main thread catches up.
				retired.push_back(Retired{ *r, p_rd->get_frames_drawn() });
			}
			*r = RID();
		}
	}
	frame_size = Size2i();
}

void PhysXGroomRenderEffect::_ensure_targets(RenderingDevice *p_rd, const Size2i &p_frame) {
	if (p_frame == frame_size && coverage_u.is_valid()) {
		return;
	}
	_free_targets(p_rd, false);
	frame_size = p_frame;
	RD::TextureFormat tf;
	tf.width = p_frame.x;
	tf.height = p_frame.y;
	tf.format = RD::DATA_FORMAT_R32_UINT;
	tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT | RD::TEXTURE_USAGE_STORAGE_ATOMIC_BIT;
	coverage_u = p_rd->texture_create(tf, RD::TextureView());
	depth_u = p_rd->texture_create(tf, RD::TextureView());
	depth2_u = p_rd->texture_create(tf, RD::TextureView());
	tf.format = RD::DATA_FORMAT_R32_SFLOAT;
	tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT;
	front_cover = p_rd->texture_create(tf, RD::TextureView());
	tf.format = RD::DATA_FORMAT_R16G16B16A16_SFLOAT;
	tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT | RD::TEXTURE_USAGE_CAN_COPY_TO_BIT;
	for (int i = 0; i < TEXTURES; i++) {
		hair[i] = p_rd->texture_create(tf, RD::TextureView());
		// No hair anywhere until the passes have run.
		p_rd->texture_clear(hair[i], Color(0, 0, 0, 0), 0, 1, 0, 1);
	}
	MutexLock lock(mutex);
	for (int i = 0; i < TEXTURES; i++) {
		published[i] = hair[i];
	}
}

void PhysXGroomRenderEffect::_dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, uint32_t p_x, uint32_t p_y, const void *p_push, uint32_t p_push_size) {
	const RID set = p_rd->uniform_set_create(p_uniforms, shaders[p_pass], 0);
	ERR_FAIL_COND(!set.is_valid());
	// Its own compute list per pass: the next pass then sees this one's writes.
	const RD::ComputeListID list = p_rd->compute_list_begin();
	p_rd->compute_list_bind_compute_pipeline(list, pipelines[p_pass]);
	p_rd->compute_list_bind_uniform_set(list, set, 0);
	if (p_push != nullptr) {
		p_rd->compute_list_set_push_constant(list, p_push, p_push_size);
	}
	p_rd->compute_list_dispatch(list, p_x, p_y, 1);
	p_rd->compute_list_end();
	p_rd->free_rid(set);
}

void PhysXGroomRenderEffect::_render(int p_callback_type, const RenderData *p_render_data) {
	struct Draw {
		ObjectID id;
		uint64_t version = 0;
		PackedFloat32Array points; // only when the GPU copy is out of date
		Transform3D transform;
		float strand_fraction = 1.0f;
		float width_scale = 1.0f;
	};
	LocalVector<Draw> draws;
	{
		MutexLock lock(mutex);
		for (const KeyValue<ObjectID, Groom> &kv : grooms) {
			if (kv.value.points.is_empty()) {
				continue;
			}
			Draw draw;
			draw.id = kv.key;
			draw.version = kv.value.version;
			const GpuGroom *gpu = gpu_grooms.getptr(kv.key);
			if (gpu == nullptr || gpu->version != kv.value.version) {
				draw.points = kv.value.points;
			}
			draw.transform = kv.value.transform;
			draw.strand_fraction = kv.value.strand_fraction;
			draw.width_scale = kv.value.width_scale;
			draws.push_back(draw);
		}
	}
	RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
	// GPU copies of grooms that are gone.
	if (rd != nullptr) {
		LocalVector<ObjectID> gone;
		for (const KeyValue<ObjectID, GpuGroom> &kv : gpu_grooms) {
			bool kept = false;
			for (const Draw &draw : draws) {
				kept = kept || draw.id == kv.key;
			}
			if (!kept) {
				gone.push_back(kv.key);
			}
		}
		for (const ObjectID &id : gone) {
			rd->free_rid(gpu_grooms[id].buffer);
			gpu_grooms.erase(id);
		}
	}
	if (draws.is_empty() || p_render_data == nullptr) {
		return;
	}
	RenderSceneBuffersRD *buffers = Object::cast_to<RenderSceneBuffersRD>(p_render_data->get_render_scene_buffers().ptr());
	RenderSceneDataRD *scene = Object::cast_to<RenderSceneDataRD>(p_render_data->get_render_scene_data());
	if (buffers == nullptr || scene == nullptr || rd == nullptr || !_ensure(rd)) {
		return;
	}
	const Size2i frame = buffers->get_internal_size();
	const RID depth = buffers->get_depth_texture();
	if (frame.x <= 0 || frame.y <= 0 || !depth.is_valid()) {
		return;
	}
	_ensure_targets(rd, frame);

	for (const Draw &draw : draws) {
		if (draw.points.is_empty()) {
			continue;
		}
		GpuGroom &gpu = gpu_grooms[draw.id];
		if (gpu.buffer.is_valid()) {
			rd->free_rid(gpu.buffer);
		}
		gpu.point_count = draw.points.size() / 12;
		gpu.buffer = rd->storage_buffer_create(draw.points.size() * sizeof(float));
		rd->buffer_update(gpu.buffer, 0, draw.points.size() * sizeof(float), draw.points.ptr());
		gpu.version = draw.version;
	}

	const Transform3D cam = scene->get_cam_transform();
	const Projection proj = scene->get_cam_projection();
	float f[PARAMS_FLOATS] = {};
	put_matrix(f + 0, Projection(cam.affine_inverse()));
	put_matrix(f + 16, proj);
	put_matrix(f + 32, proj.inverse());
	const int32_t ints[4] = { frame.x, frame.y, 0, 0 };
	memcpy(f + 48, ints, sizeof(ints));
	rd->buffer_update(params_buffer, 0, sizeof(f), f);

	const uint32_t fx = (frame.x + 7) / 8;
	const uint32_t fy = (frame.y + 7) / 8;
	const RD::Uniform params(RD::UNIFORM_TYPE_UNIFORM_BUFFER, 0, params_buffer);
	_dispatch(rd, PASS_CLEAR, { params, RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, coverage_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, depth_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 3, depth2_u) }, fx, fy, nullptr, 0);
	for (int mode = 0; mode < 3; mode++) {
		for (const Draw &draw : draws) {
			const GpuGroom *gpu = gpu_grooms.getptr(draw.id);
			if (gpu == nullptr || gpu->point_count < 2) {
				continue;
			}
			struct {
				float model[16];
				float lod[4];
				int32_t info[4];
			} push;
			put_matrix(push.model, Projection(draw.transform));
			const float fraction = MAX(draw.strand_fraction, 0.0001f);
			push.lod[0] = draw.strand_fraction;
			push.lod[1] = draw.width_scale / fraction;
			push.lod[2] = (float)draw.transform.basis.get_scale().x;
			push.lod[3] = 0.0f;
			push.info[0] = (int32_t)gpu->point_count;
			push.info[1] = mode;
			push.info[2] = 0;
			push.info[3] = 0;
			_dispatch(rd, PASS_RASTER, { params, RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, coverage_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, depth_u), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 3, gpu->buffer), RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 4, Vector<RID>({ point_sampler, depth })), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 5, hair[0]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 6, hair[2]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 7, depth2_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 8, hair[3]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 9, hair[4]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 10, front_cover) }, (gpu->point_count + 63) / 64, 1, &push, sizeof(push));
		}
	}
	_dispatch(rd, PASS_COMPOSITE, { params, RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, coverage_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, depth_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 3, hair[1]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 4, depth2_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 5, front_cover) }, fx, fy, nullptr, 0);

	// Retired targets the proxy can no longer be reading.
	for (uint32_t i = 0; i < retired.size();) {
		if (rd->get_frames_drawn() > retired[i].frame + 4) {
			rd->free_rid(retired[i].rid);
			retired.remove_at_unordered(i);
		} else {
			i++;
		}
	}
}

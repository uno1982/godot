/**************************************************************************/
/*  physx_fluid_render_effect.cpp                                         */
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

#include "physx_fluid_render_effect.h"

#include "../godot_physx_render_thread.h"
#include "physx_compositor_effects.h"

#include "core/config/engine.h"
#include "core/object/callable_mp.h"
#include "scene/3d/camera_3d.h"
#include "scene/resources/3d/primitive_meshes.h"
#include "scene/resources/3d/world_3d.h"
#include "scene/resources/material.h"
#include "scene/resources/shader.h"
#include "scene/resources/texture_rd.h"
#include "servers/rendering/renderer_rd/framebuffer_cache_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_buffers_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_data_rd.h"
#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"
#include "servers/rendering/storage/render_data.h"

Mutex PhysXFluidRenderEffect::mutex;
HashMap<ObjectID, PhysXFluidRenderEffect::Snapshot> PhysXFluidRenderEffect::fluids;
HashMap<ObjectID, int> PhysXFluidRenderEffect::worlds;
HashMap<ObjectID, RID> PhysXFluidRenderEffect::proxies;
Ref<QuadMesh> PhysXFluidRenderEffect::proxy_mesh;
Ref<ShaderMaterial> PhysXFluidRenderEffect::proxy_material;
Ref<Shader> PhysXFluidRenderEffect::proxy_shader;
Ref<Texture2DRD> PhysXFluidRenderEffect::surf_a_texture;
Ref<Texture2DRD> PhysXFluidRenderEffect::surf_b_texture;
RID PhysXFluidRenderEffect::published_a;
RID PhysXFluidRenderEffect::published_b;
bool PhysXFluidRenderEffect::textures_hooked = false;
bool PhysXFluidRenderEffect::proxies_shown = false;
uint64_t PhysXFluidRenderEffect::ready_since = UINT64_MAX;

namespace {

Ref<PhysXFluidRenderEffect> shared_effect;

enum Pass {
	PASS_CLEAR,
	PASS_SPLAT,
	PASS_FOAM,
	PASS_RESOLVE,
	PASS_FILTER,
	PASS_COMPOSITE,
	PASS_MAX,
};

// Shared by every pass (binding 0), spliced in at @PARAMS@.
const char *PARAMS = R"(
layout(set = 0, binding = 0, std140) uniform Params {
	mat4 view; // world -> view
	mat4 proj; // view -> clip, as the scene's depth was drawn
	mat4 inv_proj;
	vec4 absorption; // rgb: per meter of fluid; a: the depth jump the smoothing keeps (m)
	vec4 unused_a;
	vec4 unused_b;
	vec4 light_color; // a: the (splatted) thickness under which the fluid fades out
	vec4 misc; // x: refraction, y: smoothing radius (m), z: max splat / filter radius (px), w: thickness units per meter
	ivec4 size; // xy: the surface buffers (the frame / frame.z), z: particles
	ivec4 frame; // xy: the frame, z: frame pixels per surface texel, each way
	vec4 surface; // rgb: the body colour (linear), a: opacity per meter of fluid (0: clear)
	vec4 shading; // z: splatted thickness -> the fluid's own
	vec4 emission; // rgb, added
	vec4 foam; // rgb: foam colour (linear), a: the most a foam patch covers
	ivec4 foam_n; // x: foam particles
} p;

// Eye-space depth (meters in front of the camera) of a point on the pixel's ray.
vec3 view_pos(vec2 px, float z) {
	vec2 ndc = (px + 0.5) / vec2(p.size.xy) * 2.0 - 1.0;
	vec4 v = p.inv_proj * vec4(ndc, 0.5, 1.0);
	vec3 dir = v.xyz / v.w;
	return dir * (z / -dir.z);
}
)";

const char *SHADERS[PASS_MAX] = {
	// PASS_CLEAR
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 1, r32ui) uniform restrict writeonly uimage2D depth_img;
layout(set = 0, binding = 2, r32ui) uniform restrict writeonly uimage2D thick_img;
layout(set = 0, binding = 3, r32ui) uniform restrict writeonly uimage2D foam_depth_img;
layout(set = 0, binding = 4, r32ui) uniform restrict writeonly uimage2D foam_img;
void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, p.size.xy))) {
		return;
	}
	imageStore(depth_img, px, uvec4(0x7f800000u)); // +inf: no fluid
	imageStore(thick_img, px, uvec4(0u));
	imageStore(foam_depth_img, px, uvec4(0x7f800000u));
	imageStore(foam_img, px, uvec4(0u));
}
)",
	// PASS_SPLAT: each particle draws a sphere's front depth (atomic min of the
	// float bits -- positive floats order like their bits) and its thickness.
	R"(
#[compute]
#version 450
layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 1, r32ui) uniform restrict uimage2D depth_img;
layout(set = 0, binding = 2, r32ui) uniform restrict uimage2D thick_img;
layout(set = 0, binding = 3, std430) restrict readonly buffer Points {
	vec4 pts[];
};
layout(set = 0, binding = 4) uniform sampler2D scene_depth;

float scene_eye_depth(ivec2 px) {
	ivec2 fpx = min(px * p.frame.z + p.frame.z / 2, p.frame.xy - 1);
	float d = texelFetch(scene_depth, fpx, 0).r;
	vec2 ndc = (vec2(fpx) + 0.5) / vec2(p.frame.xy) * 2.0 - 1.0;
	vec4 v = p.inv_proj * vec4(ndc, d, 1.0);
	return -v.z / v.w;
}

void main() {
	uint i = gl_GlobalInvocationID.x;
	if (i >= uint(p.size.z)) {
		return;
	}
	vec4 pt = pts[i];
	float r = pt.w;
	vec4 vp = p.view * vec4(pt.xyz, 1.0);
	float z = -vp.z;
	if (z < r + 0.01) {
		return;
	}
	vec4 c = p.proj * vp;
	vec2 cpx = (c.xy / c.w * 0.5 + 0.5) * vec2(p.size.xy);
	float rpx = max(r * abs(p.proj[1][1]) * 0.5 * float(p.size.y) / z, 0.5);
	int R = int(ceil(min(rpx, p.misc.z)));
	ivec2 cc = ivec2(floor(cpx));
	for (int y = -R; y <= R; y++) {
		for (int x = -R; x <= R; x++) {
			ivec2 px = cc + ivec2(x, y);
			if (any(lessThan(px, ivec2(0))) || any(greaterThanEqual(px, p.size.xy))) {
				continue;
			}
			vec2 d = (vec2(px) + 0.5 - cpx) / rpx;
			float d2 = dot(d, d);
			if (d2 > 1.0) {
				continue;
			}
			float h = sqrt(1.0 - d2);
			float zs = z - r * h;
			if (zs > scene_eye_depth(px)) {
				continue; // behind the scene's own geometry
			}
			imageAtomicMin(depth_img, px, floatBitsToUint(zs));
			imageAtomicAdd(thick_img, px, uint(2.0 * r * h * p.misc.w));
		}
	}
}
)",
	// PASS_FOAM: each foam particle a soft disc -- its front depth (atomic min)
	// and a density that falls off to its edge (thousandths, summed).
	R"(
#[compute]
#version 450
layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 1, r32ui) uniform restrict uimage2D foam_depth_img;
layout(set = 0, binding = 2, r32ui) uniform restrict uimage2D foam_img;
layout(set = 0, binding = 3, std430) restrict readonly buffer Points {
	vec4 pts[];
};
layout(set = 0, binding = 4) uniform sampler2D scene_depth;

float scene_eye_depth(ivec2 px) {
	ivec2 fpx = min(px * p.frame.z + p.frame.z / 2, p.frame.xy - 1);
	float d = texelFetch(scene_depth, fpx, 0).r;
	vec2 ndc = (vec2(fpx) + 0.5) / vec2(p.frame.xy) * 2.0 - 1.0;
	vec4 v = p.inv_proj * vec4(ndc, d, 1.0);
	return -v.z / v.w;
}

void main() {
	uint i = gl_GlobalInvocationID.x;
	if (i >= uint(p.foam_n.x)) {
		return;
	}
	vec4 pt = pts[i];
	float r = pt.w;
	vec4 vp = p.view * vec4(pt.xyz, 1.0);
	float z = -vp.z;
	if (z < r + 0.01) {
		return;
	}
	vec4 c = p.proj * vp;
	vec2 cpx = (c.xy / c.w * 0.5 + 0.5) * vec2(p.size.xy);
	float rpx = max(r * abs(p.proj[1][1]) * 0.5 * float(p.size.y) / z, 0.75);
	int R = int(ceil(min(rpx, p.misc.z)));
	ivec2 cc = ivec2(floor(cpx));
	for (int y = -R; y <= R; y++) {
		for (int x = -R; x <= R; x++) {
			ivec2 px = cc + ivec2(x, y);
			if (any(lessThan(px, ivec2(0))) || any(greaterThanEqual(px, p.size.xy))) {
				continue;
			}
			vec2 d = (vec2(px) + 0.5 - cpx) / rpx;
			float d2 = dot(d, d);
			if (d2 > 1.0 || z - r > scene_eye_depth(px)) {
				continue;
			}
			imageAtomicMin(foam_depth_img, px, floatBitsToUint(z - r * sqrt(1.0 - d2)));
			imageAtomicAdd(foam_img, px, uint((1.0 - d2) * 1000.0));
		}
	}
}
)",
	// PASS_RESOLVE: the splatted depth bits as floats (+inf where empty).
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 1, r32ui) uniform restrict readonly uimage2D depth_img;
layout(set = 0, binding = 2, r32f) uniform restrict writeonly image2D dst;
void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, p.size.xy))) {
		return;
	}
	imageStore(dst, px, vec4(uintBitsToFloat(imageLoad(depth_img, px).r)));
}
)",
	// PASS_FILTER: a 2D bilateral blur of the depth over up to 8 px each way, run
	// a few times (small blurs add up to a wide one without a sparse sampling
	// grid's screen-door pattern) -- the spheres melt into one surface, while a
	// jump in depth (the stream in front of the pool, the fluid's outline) is
	// kept. (A separable blur leaves axis-aligned blocks.)
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(push_constant, std430) uniform Push {
	ivec2 dir;
	ivec2 pad;
} pc;
layout(set = 0, binding = 1, r32f) uniform restrict readonly image2D src;
layout(set = 0, binding = 2, r32f) uniform restrict writeonly image2D dst;
void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, p.size.xy))) {
		return;
	}
	float z = imageLoad(src, px).r;
	if (z > 1e30) {
		imageStore(dst, px, vec4(z));
		return;
	}
	float rpx = p.misc.y * abs(p.proj[1][1]) * 0.5 * float(p.size.y) / z;
	int steps = int(clamp(ceil(rpx), 1.0, 8.0));
	float stride = 1.0;
	float sigma_s = float(steps) * 0.5;
	float sigma_z = p.absorption.a; // depth jump kept, meters
	float sum = 0.0;
	float wsum = 0.0;
	for (int j = -steps; j <= steps; j++) {
		for (int i = -steps; i <= steps; i++) {
			vec2 o = vec2(i, j) * stride;
			ivec2 q = clamp(px + ivec2(round(o)), ivec2(0), p.size.xy - 1);
			float zq = imageLoad(src, q).r;
			if (zq > 1e30) {
				continue;
			}
			float dz = zq - z;
			float w = exp(-dot(o, o) / (2.0 * sigma_s * sigma_s)) * exp(-dz * dz / (2.0 * sigma_z * sigma_z));
			sum += zq * w;
			wsum += w;
		}
	}
	imageStore(dst, px, vec4(sum / max(wsum, 1e-6)));
}
)",
	// PASS_COMPOSITE (full resolution): the smoothed surface, for the proxy
	// material to shade (see PROXY_SHADER). At a reduced resolution each frame
	// pixel blends the 2x2 surface texels around it that hold fluid -- the share
	// that do is its coverage, a soft edge against the background -- and is
	// tested against the scene's own full-resolution depth, so where the fluid
	// meets geometry the edge stays sharp.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@PARAMS@
layout(set = 0, binding = 1, r32f) uniform restrict readonly image2D zimg;
layout(set = 0, binding = 2, r32ui) uniform restrict readonly uimage2D thick_img;
layout(set = 0, binding = 3, rgba16f) uniform restrict writeonly image2D surf_a; // view-space normal, thickness (m)
layout(set = 0, binding = 4, rgba16f) uniform restrict writeonly image2D surf_b; // eye depth (m), cover, foam, -
layout(set = 0, binding = 5) uniform sampler2D scene_depth;
layout(set = 0, binding = 6, r32ui) uniform restrict readonly uimage2D foam_depth_img;
layout(set = 0, binding = 7, r32ui) uniform restrict readonly uimage2D foam_img;
layout(set = 0, binding = 8, r32f) uniform restrict writeonly image2D liquid_depth; // the depth buffer's value here
layout(set = 0, binding = 9, r32ui) uniform restrict readonly uimage2D raw_depth; // the splatted spheres, before smoothing

// The unsmoothed depth: each particle's own sphere (+inf where empty).
float rzat(ivec2 q) {
	q = clamp(q, ivec2(0), p.size.xy - 1);
	return uintBitsToFloat(imageLoad(raw_depth, q).r);
}

float zat(ivec2 q, float fallback) {
	q = clamp(q, ivec2(0), p.size.xy - 1);
	float z = imageLoad(zimg, q).r;
	return z > 1e30 ? fallback : z;
}

void main() {
	ivec2 fpx = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(fpx, p.frame.xy))) {
		return;
	}
	vec2 uv = (vec2(fpx) + 0.5) / vec2(p.frame.xy);
	float sd = texelFetch(scene_depth, fpx, 0).r;
	vec4 sv = p.inv_proj * vec4(uv * 2.0 - 1.0, sd, 1.0);
	float scene_z = -sv.z / sv.w;
	float out_depth = sd; // goes into the depth buffer: what later passes test against
	float surface_z = 1e30; // the fluid's surface, where it draws solid enough
	vec3 n = vec3(0.0, 0.0, 1.0);
	float z_out = 0.0;
	float T = 0.0;
	float a = 0.0;
	float fa = 0.0;

	vec2 l = (vec2(fpx) + 0.5) / float(p.frame.z) - 0.5;
	ivec2 b = ivec2(floor(l));
	vec2 t = l - vec2(b);
	ivec2 qs[4] = ivec2[](b, b + ivec2(1, 0), b + ivec2(0, 1), b + ivec2(1, 1));
	float ws[4] = float[]((1.0 - t.x) * (1.0 - t.y), t.x * (1.0 - t.y), (1.0 - t.x) * t.y, t.x * t.y);
	float coverage = 0.0;
	float z = 0.0;
	float best = -1.0;
	ivec2 px = b;
	float foam = 0.0;
	float foam_z = 1e30;
	for (int k = 0; k < 4; k++) {
		ivec2 q = clamp(qs[k], ivec2(0), p.size.xy - 1);
		float fk = float(imageLoad(foam_img, q).r) / 1000.0;
		if (fk > 0.0) {
			foam += fk * ws[k];
			foam_z = min(foam_z, uintBitsToFloat(imageLoad(foam_depth_img, q).r));
		}
		float zk = imageLoad(zimg, q).r;
		if (zk > 1e30) {
			continue;
		}
		coverage += ws[k];
		z += zk * ws[k];
		T += float(imageLoad(thick_img, q).r) / p.misc.w * ws[k];
		if (ws[k] > best) {
			best = ws[k];
			px = q;
		}
	}
	// In front of the scene's geometry at this exact pixel?
	if (coverage > 0.0 && z / coverage <= scene_z) {
		z /= coverage;
		T /= coverage;
		// Normal from the depth around the nearest surface texel: per axis, the
		// neighbor on the flatter side, so silhouettes don't smear -- but where
		// one side is empty, the side with fluid. (Treating the empty side as
		// level tilted a sheet's rim toward the camera, and rims that tilt onto
		// the sun's mirror angle flashed white.) A texel with neither neighbor
		// on an axis keeps a level slope there.
		float zc = imageLoad(zimg, px).r;
		vec3 P = view_pos(vec2(px), zc);
		float zr = zat(px + ivec2(1, 0), 1e30);
		float zl = zat(px - ivec2(1, 0), 1e30);
		float zu = zat(px + ivec2(0, 1), 1e30);
		float zd = zat(px - ivec2(0, 1), 1e30);
		bool use_r = zr < 1e29 && (zl > 1e29 || abs(zr - zc) < abs(zc - zl));
		bool use_u = zu < 1e29 && (zd > 1e29 || abs(zu - zc) < abs(zc - zd));
		vec3 ddx = use_r ? view_pos(vec2(px + ivec2(1, 0)), zr) - P : (zl < 1e29 ? P - view_pos(vec2(px - ivec2(1, 0)), zl) : view_pos(vec2(px + ivec2(1, 0)), zc) - P);
		vec3 ddy = use_u ? view_pos(vec2(px + ivec2(0, 1)), zu) - P : (zd < 1e29 ? P - view_pos(vec2(px - ivec2(0, 1)), zd) : view_pos(vec2(px + ivec2(0, 1)), zc) - P);
		n = normalize(cross(ddx, ddy));
		// Fluid that isn't part of a larger surface (droplets, a scattered
		// smear): the smoothing is wider than it, flattening it to one depth --
		// a flat disc facing the camera, whose rim rings and whose face mirrors
		// the sun whole. Its normal comes from the unsmoothed depth instead, the
		// particles' own spheres. Pools and streams keep the smoothed normal.
		// Probed a few particle radii out, measured on screen: a close-up droplet
		// spans many texels, and a fixed distance would land inside it (counting
		// it as pooled, so flat and facing the camera, mirroring the sun whole).
		float radius_px = p.absorption.a * 0.5 * abs(p.proj[1][1]) * 0.5 * float(p.size.y) / zc;
		float probe = clamp(radius_px * 2.5, 3.0, 24.0);
		float pooled = 0.0;
		for (int k = 0; k < 8; k++) {
			vec2 dir = vec2(cos(float(k) * 0.785398), sin(float(k) * 0.785398));
			pooled += float(zat(px + ivec2(round(dir * probe)), 1e30) < 1e29);
		}
		pooled /= 8.0;
		float rc = rzat(px);
		if (pooled < 1.0 && rc < 1e30) {
			vec3 RP = view_pos(vec2(px), rc);
			float rr = rzat(px + ivec2(1, 0));
			float rl = rzat(px - ivec2(1, 0));
			float ru = rzat(px + ivec2(0, 1));
			float rd = rzat(px - ivec2(0, 1));
			// Per axis the nearer-in-depth neighbor, skipping empty ones (a
			// sphere's rim): a jump to another particle isn't this one's slope.
			bool rur = rr < 1e30 && (rl >= 1e30 || abs(rr - rc) < abs(rc - rl));
			bool ruu = ru < 1e30 && (rd >= 1e30 || abs(ru - rc) < abs(rc - rd));
			vec3 rdx = rur ? view_pos(vec2(px + ivec2(1, 0)), rr) - RP : (rl < 1e30 ? RP - view_pos(vec2(px - ivec2(1, 0)), rl) : vec3(1.0, 0.0, 0.0));
			vec3 rdy = ruu ? view_pos(vec2(px + ivec2(0, 1)), ru) - RP : (rd < 1e30 ? RP - view_pos(vec2(px - ivec2(0, 1)), rd) : vec3(0.0, 1.0, 0.0));
			vec3 rn = normalize(cross(rdx, rdy));
			if (dot(rn, normalize(-RP)) < 0.0) {
				rn = -rn;
			}
			// A body's edge (a pool's rim, a stream's sides) still has fluid in about
			// half the directions; only loose fluid has next to none. (Blending up
			// to 1.0 gave every silhouette the raw spheres' edge-on rim normals,
			// outlining the pool and stream in dark.)
			n = normalize(mix(rn, n, smoothstep(0.1, 0.35, pooled)));
		}
		if (dot(n, normalize(-P)) < 0.0) {
			n = -n;
		}
		// Thin fluid (a lone droplet, a sheet's edge) fades into the scene rather
		// than drawing as a hard bubble; so does the edge of the coverage.
		a = smoothstep(0.0, p.light_color.a, T) * coverage;
		z_out = z;
		if (a > 0.5) {
			// The surface, on this pixel's ray, as the depth buffer stores it.
			vec4 v = p.inv_proj * vec4(uv * 2.0 - 1.0, 0.5, 1.0);
			vec3 dir = v.xyz / v.w;
			vec4 c = p.proj * vec4(dir * (z / -dir.z), 1.0);
			out_depth = c.z / c.w;
			surface_z = z;
		}
	}
	// Foam over the surface; foam under it fades with how deep it is.
	if (foam > 0.0 && foam_z <= scene_z) {
		float under = max(foam_z - surface_z, 0.0);
		fa = (1.0 - exp(-foam * 1.5)) * exp(-under * 12.0);
		if (a <= 0.0) {
			z_out = foam_z; // spray in the open: lit where the foam is
		}
	}
	imageStore(surf_a, fpx, vec4(n, T * p.shading.z));
	imageStore(surf_b, fpx, vec4(z_out, a, fa, 0.0));
	imageStore(liquid_depth, fpx, vec4(out_depth));
}
)",
};

const char *PASS_NAMES[PASS_MAX] = { "clear", "splat", "foam", "resolve", "filter", "composite" };

// Writes the liquid's depth into the frame's own depth target (the
// multisampled one with MSAA) before the transparent pass, where it's solid:
// transparent geometry then sorts against it. A compute pass can't write a
// depth attachment; a fullscreen triangle with gl_FragDepth can.
const char *WRITE_SHADER = R"(
#[vertex]
#version 450
void main() {
	vec2 p = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
	gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
#[fragment]
#version 450
layout(set = 0, binding = 0) uniform sampler2D surf_b;
layout(set = 0, binding = 1) uniform sampler2D liquid_depth;
void main() {
	ivec2 px = ivec2(gl_FragCoord.xy);
	if (texelFetch(surf_b, px, 0).g <= 0.5) {
		discard;
	}
	gl_FragDepth = texelFetch(liquid_depth, px, 0).r;
}
)";

// The material that shades the surface in Godot's own transparent pass (drawn
// first there), on a quad covering the screen: it puts the surface where the
// compute passes found it (LIGHT_VERTEX, NORMAL), so the scene's lights,
// shadows, ambient light, reflections and sky light it like any other
// surface. What shows through it is the screen behind (opaque geometry and
// sky) refracted and absorbed by the fluid's thickness; the fluid's own colour
// (opaque-ish materials) and its foam are lit.
const char *PROXY_SHADER = R"(
shader_type spatial;
render_mode blend_mix, depth_draw_never, depth_test_disabled, cull_disabled, skip_vertex_transform;

uniform sampler2D surf_a : filter_nearest, repeat_disable;
uniform sampler2D surf_b : filter_nearest, repeat_disable;
uniform sampler2D screen : hint_screen_texture, filter_linear, repeat_disable;
uniform vec3 absorption = vec3(0.45, 0.12, 0.06);
uniform vec4 body = vec4(0.0); // rgb: colour, a: opacity per meter of fluid
uniform float refraction = 0.04;
uniform float surface_roughness = 0.05;
uniform float surface_metallic = 0.0;
uniform vec3 emission_color = vec3(0.0);
uniform vec4 foam_color = vec4(1.0, 1.0, 1.0, 0.85);

void vertex() {
	// The quad's corners on the camera's frustum: each pixel's VERTEX (and so
	// VIEW, which reflections and highlights use) then lies on its own ray.
	vec4 v = INV_PROJECTION_MATRIX * vec4(VERTEX.xy, 0.5, 1.0);
	VERTEX = v.xyz / v.w;
	POSITION = PROJECTION_MATRIX * vec4(VERTEX, 1.0);
}

void fragment() {
	vec4 b = texture(surf_b, SCREEN_UV);
	float cover = b.y;
	float foam = b.z * foam_color.a;
	if (cover <= 0.001 && foam <= 0.001) {
		discard;
	}
	vec4 a = texture(surf_a, SCREEN_UV);
	vec4 ray = INV_PROJECTION_MATRIX * vec4(SCREEN_UV * 2.0 - 1.0, 0.5, 1.0);
	vec3 dir = ray.xyz / ray.w;
	vec3 P = dir * (b.x / -dir.z);
	LIGHT_VERTEX = P;
	vec3 n = normalize(a.xyz);
	NORMAL = n;
	float T = a.w;
	// At least the material's alpha wherever there's fluid, as on a mesh (5 cm
	// of fluid reaches it), denser where it's thicker: a thin stream keeps its
	// colour rather than going clear.
	float own = 1.0 - exp(-body.a * max(T, 0.05));
	vec2 ruv = clamp(SCREEN_UV + n.xy * refraction * min(T / 0.07, 1.0), vec2(0.0), vec2(1.0));
	vec3 through = textureLod(screen, ruv, 0.0).rgb * exp(-absorption * T);
	ALBEDO = mix(body.rgb * own, foam_color.rgb, foam);
	// (Godot's own specular adds the reflection on top; it doesn't take the
	// grazing-angle share out of what shows through, so neither does this.)
	EMISSION = through * (1.0 - own) * (1.0 - foam) + emission_color;
	ROUGHNESS = mix(surface_roughness, 1.0, foam);
	SPECULAR = 0.25 * (1.0 - foam); // F0 ~ 0.02: water
	METALLIC = surface_metallic * (1.0 - foam);
	ALPHA = clamp(max(cover, foam), 0.0, 1.0);
}

// A liquid's own colour is light scattered inside it, so it hardly depends on
// which way its surface faces: a wrapped diffuse lets the side away from a
// light still glow (a solid's Lambert would go black there). Shadows still
// apply (ATTENUATION). The highlight is a normalized Blinn-Phong with water's
// fresnel, since writing light() replaces Godot's own.
void light() {
	float r = max(surface_roughness, 0.02);
	float power = clamp(2.0 / (r * r * r * r) - 2.0, 4.0, 4096.0);
	vec3 H = normalize(VIEW + LIGHT);
	float F = 0.02 + 0.98 * pow(1.0 - clamp(dot(H, VIEW), 0.0, 1.0), 5.0);
	float spec = pow(max(dot(NORMAL, H), 0.0), power) * (power + 8.0) / (8.0 * PI);
	float wrapped = max((dot(NORMAL, LIGHT) + 0.6) / 1.6, 0.0);
	DIFFUSE_LIGHT += LIGHT_COLOR * ATTENUATION * wrapped / PI;
	SPECULAR_LIGHT += LIGHT_COLOR * ATTENUATION * spec * F * max(dot(NORMAL, LIGHT), 0.0);
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

void PhysXFluidRenderEffect::set_fluid(const ObjectID &p_fluid, const Snapshot &p_snapshot) {
	MutexLock lock(mutex);
	fluids[p_fluid] = p_snapshot;
}

void PhysXFluidRenderEffect::set_foam(const ObjectID &p_fluid, const PackedFloat32Array &p_foam) {
	MutexLock lock(mutex);
	Snapshot *s = fluids.getptr(p_fluid);
	if (s != nullptr) {
		s->foam = p_foam;
	}
}

void PhysXFluidRenderEffect::remove_fluid(const ObjectID &p_fluid) {
	MutexLock lock(mutex);
	fluids.erase(p_fluid);
}

void PhysXFluidRenderEffect::attach_world(World3D *p_world) {
	ERR_FAIL_NULL(p_world);
	if (worlds[p_world->get_instance_id()]++ > 0) {
		return;
	}
	if (proxy_material.is_null()) {
		proxy_shader.instantiate();
		proxy_shader->set_code(PROXY_SHADER);
		surf_a_texture.instantiate();
		surf_b_texture.instantiate();
		proxy_material.instantiate();
		proxy_material->set_shader(proxy_shader);
		proxy_material->set_shader_parameter("surf_a", surf_a_texture);
		proxy_material->set_shader_parameter("surf_b", surf_b_texture);
		proxy_material->set_render_priority(Material::RENDER_PRIORITY_MIN); // first among transparent geometry
		proxy_mesh.instantiate();
		proxy_mesh->set_size(Size2(2, 2));
		proxy_mesh->set_material(proxy_material);
	}
	RenderingServer *rs = RenderingServer::get_singleton();
	const RID instance = rs->instance_create2(proxy_mesh->get_rid(), p_world->get_scenario());
	rs->instance_set_custom_aabb(instance, AABB(Vector3(-1e6, -1e6, -1e6), Vector3(2e6, 2e6, 2e6)));
	rs->instance_geometry_set_cast_shadows_setting(instance, RSE::SHADOW_CASTING_SETTING_OFF);
	rs->instance_set_ignore_culling(instance, true);
	// Hidden until its textures are hooked up (see _update_proxy): until then
	// they read as Godot's placeholder, a checkerboard over the whole screen.
	rs->instance_set_visible(instance, false);
	proxies[p_world->get_instance_id()] = instance;
}

void PhysXFluidRenderEffect::detach_world(const ObjectID &p_world) {
	int *count = worlds.getptr(p_world);
	if (count == nullptr || --(*count) > 0) {
		return;
	}
	worlds.erase(p_world);
	PhysXCompositorEffects::uninstall(p_world, get_shared());
	if (proxies.has(p_world)) {
		RenderingServer::get_singleton()->free_rid(proxies[p_world]);
		proxies.erase(p_world);
	}
}

void PhysXFluidRenderEffect::install(World3D *p_world, Camera3D *p_camera) {
	ERR_FAIL_NULL(p_world);
	ERR_FAIL_COND(!worlds.has(p_world->get_instance_id()));
	PhysXCompositorEffects::install(p_world, p_camera, get_shared(), PhysXCompositorEffects::ORDER_FLUID);
	_update_proxy(p_camera);
}

void PhysXFluidRenderEffect::_update_proxy(Camera3D *p_camera) {
	if (proxy_material.is_null()) {
		return;
	}
	RID a, b;
	Snapshot look;
	bool found = false;
	bool anything = false; // a fluid with particles or foam to draw
	{
		MutexLock lock(mutex);
		a = published_a;
		b = published_b;
		// The look of the fluid nearest the camera.
		const Vector3 eye = p_camera != nullptr ? p_camera->get_global_position() : Vector3();
		real_t best = 1e30;
		for (const KeyValue<ObjectID, Snapshot> &kv : fluids) {
			anything = anything || !kv.value.points.is_empty() || !kv.value.foam.is_empty();
			const real_t d = eye.distance_squared_to(kv.value.center);
			if (d < best) {
				best = d;
				look = kv.value;
				found = true;
			}
		}
	}
	if (a.is_valid() && surf_a_texture->get_texture_rd_rid() != a) {
		surf_a_texture->set_texture_rd_rid(a);
	}
	if (b.is_valid() && surf_b_texture->get_texture_rd_rid() != b) {
		surf_b_texture->set_texture_rd_rid(b);
	}
	// Shown once both textures point at the effect's own (the hookup lands on
	// the render thread, a frame later) -- and they stay hooked up across a
	// resize, the old textures outliving the switch -- while there's anything
	// to draw (no stale surface after the fluids clear).
	if (surf_a_texture->get_texture_rd_rid().is_valid() && surf_b_texture->get_texture_rd_rid().is_valid()) {
		textures_hooked = true;
	}
	// ...and two frames on from that, by which time everything the material
	// reads (the screen copy among them) has been drawn once.
	const uint64_t now = Engine::get_singleton()->get_frames_drawn();
	if (!(textures_hooked && anything)) {
		ready_since = UINT64_MAX;
	} else if (ready_since == UINT64_MAX) {
		ready_since = now;
	}
	const bool show = ready_since != UINT64_MAX && now >= ready_since + 2;
	if (show != proxies_shown) {
		proxies_shown = show;
		for (const KeyValue<ObjectID, RID> &kv : proxies) {
			RenderingServer::get_singleton()->instance_set_visible(kv.value, show);
		}
	}
	if (!found) {
		return;
	}
	proxy_material->set_shader_parameter("absorption", Vector3(look.absorption.r, look.absorption.g, look.absorption.b));
	// As Vector4, not Color: a Color would be converted to linear again.
	proxy_material->set_shader_parameter("body", Vector4(look.surface.r, look.surface.g, look.surface.b, look.surface.a));
	proxy_material->set_shader_parameter("refraction", look.refraction);
	proxy_material->set_shader_parameter("surface_roughness", look.roughness);
	proxy_material->set_shader_parameter("surface_metallic", look.metallic);
	proxy_material->set_shader_parameter("emission_color", Vector3(look.emission.r, look.emission.g, look.emission.b));
	proxy_material->set_shader_parameter("foam_color", Vector4(look.foam_color.r, look.foam_color.g, look.foam_color.b, look.foam_color.a));
}

Ref<PhysXFluidRenderEffect> PhysXFluidRenderEffect::get_shared() {
	if (shared_effect.is_null()) {
		shared_effect.instantiate();
	}
	return shared_effect;
}

void PhysXFluidRenderEffect::free_shared() {
	for (const KeyValue<ObjectID, RID> &kv : proxies) {
		RenderingServer::get_singleton()->free_rid(kv.value);
	}
	proxies.clear();
	proxy_mesh.unref();
	proxy_material.unref();
	proxy_shader.unref();
	surf_a_texture.unref();
	surf_b_texture.unref();
	textures_hooked = false;
	proxies_shown = false;
	ready_since = UINT64_MAX;
	shared_effect.unref();
}

PhysXFluidRenderEffect::PhysXFluidRenderEffect() {
	// Before the transparent pass, so transparent geometry sorts against the
	// liquid (see WRITE_SHADER).
	set_effect_callback_type(EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT);
	set_access_resolved_color(true);
	set_access_resolved_depth(true);
	RenderingServer::get_singleton()->compositor_effect_set_callback(get_rid(), RSE::COMPOSITOR_EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT, callable_mp(this, &PhysXFluidRenderEffect::_render));
}

PhysXFluidRenderEffect::~PhysXFluidRenderEffect() {
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
		for (int i = 0; i < PASS_MAX; i++) {
			if (pipelines[i].is_valid()) {
				rd->free_rid(pipelines[i]);
			}
			if (shaders[i].is_valid()) {
				rd->free_rid(shaders[i]);
			}
		}
		for (const KeyValue<int64_t, RID> &kv : write_pipelines) {
			rd->free_rid(kv.value);
		}
		for (RID r : { write_shader, linear_sampler, point_sampler, params_buffer, points_buffer, foam_buffer }) {
			if (r.is_valid()) {
				rd->free_rid(r);
			}
		}
	});
}

bool PhysXFluidRenderEffect::_ensure(RenderingDevice *p_rd) {
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
		ERR_FAIL_COND_V_MSG(file->parse_versions_from_text(source) != OK, false, vformat("PhysXParticleFluid3D: screen-space %s shader failed to compile.", PASS_NAMES[i]));
		Ref<RDShaderSPIRV> spirv = file->get_spirv();
		ERR_FAIL_COND_V(spirv.is_null(), false);
		const String err = spirv->get_stage_compile_error(RD::SHADER_STAGE_COMPUTE);
		ERR_FAIL_COND_V_MSG(!err.is_empty(), false, vformat("PhysXParticleFluid3D: screen-space %s shader: %s", PASS_NAMES[i], err));
		shaders[i] = p_rd->shader_create_from_spirv(file->get_spirv_stages());
		ERR_FAIL_COND_V(!shaders[i].is_valid(), false);
		pipelines[i] = p_rd->compute_pipeline_create(shaders[i]);
		ERR_FAIL_COND_V(!pipelines[i].is_valid(), false);
	}
	{
		Ref<RDShaderFile> file;
		file.instantiate();
		ERR_FAIL_COND_V_MSG(file->parse_versions_from_text(WRITE_SHADER) != OK, false, "PhysXParticleFluid3D: screen-space write shader failed to compile.");
		Ref<RDShaderSPIRV> spirv = file->get_spirv();
		ERR_FAIL_COND_V(spirv.is_null(), false);
		for (RD::ShaderStage stage : { RD::SHADER_STAGE_VERTEX, RD::SHADER_STAGE_FRAGMENT }) {
			const String err = spirv->get_stage_compile_error(stage);
			ERR_FAIL_COND_V_MSG(!err.is_empty(), false, vformat("PhysXParticleFluid3D: screen-space write shader: %s", err));
		}
		write_shader = p_rd->shader_create_from_spirv(file->get_spirv_stages());
		ERR_FAIL_COND_V(!write_shader.is_valid(), false);
	}
	RD::SamplerState linear;
	linear.mag_filter = RD::SAMPLER_FILTER_LINEAR;
	linear.min_filter = RD::SAMPLER_FILTER_LINEAR;
	linear.repeat_u = RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE;
	linear.repeat_v = RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE;
	linear_sampler = p_rd->sampler_create(linear);
	RD::SamplerState point;
	point_sampler = p_rd->sampler_create(point);
	params_buffer = p_rd->uniform_buffer_create(PARAMS_FLOATS * 4);
	failed = false;
	return true;
}

void PhysXFluidRenderEffect::_free_targets(RenderingDevice *p_rd, bool p_now) {
	for (RID *r : { &depth_u, &thick_u, &foam_depth_u, &foam_u, &smooth_a, &smooth_b, &surf_a, &surf_b, &liquid_depth }) {
		if (r->is_valid()) {
			if (p_now) {
				p_rd->free_rid(*r);
			} else {
				// The proxy material may still point at it until the main thread
				// catches up: free it a few frames later.
				retired.push_back(Retired{ *r, p_rd->get_frames_drawn() });
			}
			*r = RID();
		}
	}
	size = Size2i();
	frame_size = Size2i();
}

void PhysXFluidRenderEffect::_ensure_targets(RenderingDevice *p_rd, const Size2i &p_size, const Size2i &p_frame) {
	if (p_size == size && p_frame == frame_size && depth_u.is_valid()) {
		return;
	}
	_free_targets(p_rd, false);
	size = p_size;
	frame_size = p_frame;
	RD::TextureFormat tf;
	tf.width = p_size.x;
	tf.height = p_size.y;
	tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT;
	tf.format = RD::DATA_FORMAT_R32_UINT;
	tf.usage_bits |= RD::TEXTURE_USAGE_STORAGE_ATOMIC_BIT;
	depth_u = p_rd->texture_create(tf, RD::TextureView());
	thick_u = p_rd->texture_create(tf, RD::TextureView());
	foam_depth_u = p_rd->texture_create(tf, RD::TextureView());
	foam_u = p_rd->texture_create(tf, RD::TextureView());
	tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT;
	tf.format = RD::DATA_FORMAT_R32_SFLOAT;
	smooth_a = p_rd->texture_create(tf, RD::TextureView());
	smooth_b = p_rd->texture_create(tf, RD::TextureView());
	tf.format = RD::DATA_FORMAT_R16G16B16A16_SFLOAT;
	tf.width = p_frame.x;
	tf.height = p_frame.y;
	tf.usage_bits |= RD::TEXTURE_USAGE_CAN_COPY_TO_BIT; // cleared on creation
	surf_a = p_rd->texture_create(tf, RD::TextureView());
	surf_b = p_rd->texture_create(tf, RD::TextureView());
	// No surface anywhere until the composite has run.
	p_rd->texture_clear(surf_a, Color(0, 0, 0, 0), 0, 1, 0, 1);
	p_rd->texture_clear(surf_b, Color(0, 0, 0, 0), 0, 1, 0, 1);
	tf.format = RD::DATA_FORMAT_R32_SFLOAT;
	liquid_depth = p_rd->texture_create(tf, RD::TextureView());
	MutexLock lock(mutex);
	published_a = surf_a;
	published_b = surf_b;
}

void PhysXFluidRenderEffect::_dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, uint32_t p_x, uint32_t p_y, const void *p_push, uint32_t p_push_size) {
	const RID set = p_rd->uniform_set_create(p_uniforms, shaders[p_pass], 0);
	ERR_FAIL_COND(!set.is_valid());
	// Its own compute list per pass: the next pass then sees this one's writes
	// (as the Flow light scattering's two passes do).
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

void PhysXFluidRenderEffect::_render(int p_callback_type, const RenderData *p_render_data) {
	// Every fluid's particles become one surface (streams that meet merge);
	// the shading comes from the fluid nearest the camera.
	PackedFloat32Array points;
	PackedFloat32Array foam;
	Snapshot look;
	{
		MutexLock lock(mutex);
		if (fluids.is_empty()) {
			return;
		}
		int total = 0;
		int foam_total = 0;
		for (const KeyValue<ObjectID, Snapshot> &kv : fluids) {
			total += kv.value.points.size();
			foam_total += kv.value.foam.size();
		}
		if (total == 0 && foam_total == 0) {
			return;
		}
		points.resize(MAX(total, 4)); // a placeholder point keeps the buffer valid
		foam.resize(MAX(foam_total, 4));
		float *w = points.ptrw();
		float *fw = foam.ptrw();
		for (const KeyValue<ObjectID, Snapshot> &kv : fluids) {
			memcpy(w, kv.value.points.ptr(), kv.value.points.size() * sizeof(float));
			w += kv.value.points.size();
			memcpy(fw, kv.value.foam.ptr(), kv.value.foam.size() * sizeof(float));
			fw += kv.value.foam.size();
		}
		if (total == 0) {
			points.resize(0);
		}
		if (foam_total == 0) {
			foam.resize(0);
		}
		look = fluids.begin()->value;
	}
	if (p_render_data == nullptr) {
		return;
	}
	RenderSceneBuffersRD *buffers = Object::cast_to<RenderSceneBuffersRD>(p_render_data->get_render_scene_buffers().ptr());
	RenderSceneDataRD *scene = Object::cast_to<RenderSceneDataRD>(p_render_data->get_render_scene_data());
	RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
	if (buffers == nullptr || scene == nullptr || rd == nullptr || !_ensure(rd)) {
		return;
	}
	const Size2i frame = buffers->get_internal_size();
	const RID color = buffers->get_internal_texture();
	const RID depth = buffers->get_depth_texture();
	if (frame.x <= 0 || frame.y <= 0 || !color.is_valid() || !depth.is_valid()) {
		return;
	}
	// Nearest fluid's look.
	const Transform3D cam = scene->get_cam_transform();
	{
		MutexLock lock(mutex);
		real_t best = 1e30;
		for (const KeyValue<ObjectID, Snapshot> &kv : fluids) {
			const real_t d = cam.origin.distance_squared_to(kv.value.center);
			if (d < best) {
				best = d;
				look = kv.value;
			}
		}
	}

	const int div = look.resolution_divisor;
	const Size2i low((frame.x + div - 1) / div, (frame.y + div - 1) / div);
	_ensure_targets(rd, low, frame);

	const uint32_t count = points.size() / 4;
	const uint32_t foam_count = foam.size() / 4;
	auto upload = [&](RID &r_buffer, uint32_t &r_capacity, const PackedFloat32Array &p_data, uint32_t p_count) {
		if (p_count > r_capacity || !r_buffer.is_valid()) {
			if (r_buffer.is_valid()) {
				rd->free_rid(r_buffer);
			}
			r_capacity = MAX(MAX(p_count, 1u), r_capacity * 2);
			r_buffer = rd->storage_buffer_create(r_capacity * 16);
		}
		if (p_count > 0) {
			rd->buffer_update(r_buffer, 0, p_count * 16, p_data.ptr());
		}
	};
	upload(points_buffer, points_capacity, points, count);
	upload(foam_buffer, foam_capacity, foam, foam_count);

	const Projection proj = scene->get_cam_projection();
	const Projection view = Projection(cam.affine_inverse());
	float f[PARAMS_FLOATS] = {};
	put_matrix(f + 0, view);
	put_matrix(f + 16, proj);
	put_matrix(f + 32, proj.inverse());
	f[48] = look.absorption.r;
	f[49] = look.absorption.g;
	f[50] = look.absorption.b;
	f[51] = look.depth_threshold;
	f[63] = look.fade_thickness;
	f[64] = look.refraction;
	f[65] = look.smoothing;
	f[66] = 24.0f; // max splat / filter radius, px
	f[67] = 1000.0f; // thickness: millimeters in the uint buffer
	const int32_t ints[8] = { low.x, low.y, (int32_t)count, 0, frame.x, frame.y, div, 0 };
	memcpy(f + 68, ints, sizeof(ints));
	f[76] = look.surface.r;
	f[77] = look.surface.g;
	f[78] = look.surface.b;
	f[79] = look.surface.a;
	f[82] = look.thickness_scale;
	f[84] = look.emission.r;
	f[85] = look.emission.g;
	f[86] = look.emission.b;
	f[88] = look.foam_color.r;
	f[89] = look.foam_color.g;
	f[90] = look.foam_color.b;
	f[91] = look.foam_color.a;
	const int32_t foam_ints[4] = { (int32_t)foam_count, 0, 0, 0 };
	memcpy(f + 92, foam_ints, sizeof(foam_ints));
	rd->buffer_update(params_buffer, 0, sizeof(f), f);

	const uint32_t gx = (low.x + 7) / 8;
	const uint32_t gy = (low.y + 7) / 8;
	const uint32_t fx = (frame.x + 7) / 8;
	const uint32_t fy = (frame.y + 7) / 8;
	const RD::Uniform params(RD::UNIFORM_TYPE_UNIFORM_BUFFER, 0, params_buffer);
	_dispatch(rd, PASS_CLEAR, { params, RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, depth_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, thick_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 3, foam_depth_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 4, foam_u) }, gx, gy, nullptr, 0);
	if (foam_count > 0) {
		_dispatch(rd, PASS_FOAM, { params, RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, foam_depth_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, foam_u), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 3, foam_buffer), RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 4, Vector<RID>({ point_sampler, depth })) }, (foam_count + 63) / 64, 1, nullptr, 0);
	}
	if (count > 0) {
		_dispatch(rd, PASS_SPLAT, { params, RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, depth_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, thick_u), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 3, points_buffer), RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 4, Vector<RID>({ point_sampler, depth })) }, (count + 63) / 64, 1, nullptr, 0);
	}
	_dispatch(rd, PASS_RESOLVE, { params, RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, depth_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, smooth_b) }, gx, gy, nullptr, 0);
	const int32_t no_dir[4] = { 0, 0, 0, 0 };
	for (int i = 0; i < 3; i++) { // b -> a -> b -> a
		const RID from = (i % 2 == 0) ? smooth_b : smooth_a;
		const RID to = (i % 2 == 0) ? smooth_a : smooth_b;
		_dispatch(rd, PASS_FILTER, { params, RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, from), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, to) }, gx, gy, no_dir, sizeof(no_dir));
	}
	_dispatch(rd, PASS_COMPOSITE, { params, RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 1, smooth_a), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 2, thick_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 3, surf_a), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 4, surf_b), RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 5, Vector<RID>({ point_sampler, depth })), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 6, foam_depth_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 7, foam_u), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 8, liquid_depth), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 9, depth_u) }, fx, fy, nullptr, 0);
	_write(rd, buffers, depth);
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

void PhysXFluidRenderEffect::_write(RenderingDevice *p_rd, RenderSceneBuffersRD *p_buffers, const RID &p_depth) {
	// The depth target the transparent pass tests against: with MSAA the
	// multisampled one (the resolved copy we read gets overwritten from it
	// later). Depth only -- no colour attachment, so nothing but depth can be
	// written (a colour write mask alone didn't hold: the pass painted black
	// where the liquid was solid).
	const bool msaa = p_buffers->get_msaa_3d() != RSE::VIEWPORT_MSAA_DISABLED;
	const RID depth_target = msaa ? p_buffers->get_depth_msaa() : p_depth;
	ERR_FAIL_COND(!depth_target.is_valid());
	const RID framebuffer = FramebufferCacheRD::get_singleton()->get_cache(depth_target);
	ERR_FAIL_COND(!framebuffer.is_valid());
	const RD::FramebufferFormatID format = p_rd->framebuffer_get_format(framebuffer);
	RID *pipeline = write_pipelines.getptr(format);
	if (pipeline == nullptr) {
		RD::PipelineMultisampleState multisample;
		multisample.sample_count = p_rd->framebuffer_format_get_texture_samples(format);
		RD::PipelineDepthStencilState depth_state;
		depth_state.enable_depth_test = true;
		depth_state.enable_depth_write = true;
		depth_state.depth_compare_operator = RD::COMPARE_OP_ALWAYS; // the composite already kept the nearer
		const RID created = p_rd->render_pipeline_create(write_shader, format, RD::INVALID_ID, RD::RENDER_PRIMITIVE_TRIANGLES, RD::PipelineRasterizationState(), multisample, depth_state, RD::PipelineColorBlendState());
		ERR_FAIL_COND(!created.is_valid());
		pipeline = &write_pipelines.insert(format, created)->value;
	}
	Vector<RD::Uniform> uniforms;
	uniforms.push_back(RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 0, Vector<RID>({ point_sampler, surf_b })));
	uniforms.push_back(RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, 1, Vector<RID>({ point_sampler, liquid_depth })));
	const RID set = p_rd->uniform_set_create(uniforms, write_shader, 0);
	ERR_FAIL_COND(!set.is_valid());
	const RD::DrawListID list = p_rd->draw_list_begin(framebuffer, RD::DRAW_DEFAULT_ALL);
	p_rd->draw_list_bind_render_pipeline(list, *pipeline);
	p_rd->draw_list_bind_uniform_set(list, set, 0);
	p_rd->draw_list_draw(list, false, 1, 3);
	p_rd->draw_list_end();
	p_rd->free_rid(set);
}

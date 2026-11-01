/**************************************************************************/
/*  physx_gi_effect.cpp                                                   */
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

#include "physx_gi_effect.h"

#include "../godot_physx_render_thread.h"
#include "physx_compositor_effects.h"

#include "core/object/callable_mp.h"
#include "scene/3d/camera_3d.h"
#include "scene/resources/3d/world_3d.h"
#include "servers/rendering/renderer_rd/environment/sky.h"
#include "servers/rendering/renderer_rd/renderer_scene_render_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/light_storage.h"
#include "servers/rendering/renderer_rd/storage_rd/material_storage.h"
#include "servers/rendering/renderer_rd/storage_rd/render_data_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_buffers_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/render_scene_data_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/texture_storage.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"
#include "servers/rendering/rendering_server_globals.h"
#include "servers/rendering/storage/render_data.h"
#include "servers/rendering/storage/render_scene_data.h"

Mutex PhysXGIEffect::mutex;
std::atomic<int> PhysXGIEffect::settle{ 0 };
HashMap<ObjectID, PhysXGIEffect::Snapshot> PhysXGIEffect::settings;
HashMap<ObjectID, int> PhysXGIEffect::worlds;

namespace {

Ref<PhysXGIEffect> shared_effect;

// The frame's camera, shared by every pass at binding 9, and the view-space helpers. Positions are rebuilt from depth
// exactly as Godot's gi.glsl does (reverse-Z depth, linearised with the near / far planes); pixel_from_view is its
// inverse.
const char *COMMON = R"(
layout(set = 0, binding = 9, std140) uniform Frame {
	mat4 view_to_world;
	mat4 world_to_view;
	mat4 world_to_prev_view;
	vec4 proj_info;
	vec4 prev_proj_info;
	vec4 clip; // near, far, previous near, previous far
	ivec4 sizes; // xy: the frame, zw: the trace
	vec4 misc; // frame index, max ray distance, intensity, debug mode
	vec4 sun_direction; // toward the sun, world space
	vec4 sun_color; // w: the camera's exposure, which the shapes' glow is scaled by
	vec4 ambient; // w: how far a GI ray goes against the collision shapes
	ivec4 counts; // x: primitives, y: tree nodes, z: omni lights, w: spot lights
	vec4 omni_position[8]; // w: 1 / range
	vec4 omni_color[8]; // w: attenuation exponent
	vec4 spot_position[8]; // w: 1 / range
	vec4 spot_direction[8]; // w: cosine of the cone's half angle
	vec4 spot_color[8]; // w: attenuation exponent
	vec4 spot_cone[8]; // x: the cone edge's falloff exponent
	vec4 reflect; // x: mode (0 Godot's own, 1 screen space, 2 and the shapes, 3 and the probes), y: roughness cutoff, z: emissives, w: the light sources just changed
	vec4 emit_center[64]; // w: its shape's slot in the records (-1: none)
	vec4 emit_x[64]; // half axes; the three w: which way a one-sided glow faces (zero: all round)
	vec4 emit_y[64];
	vec4 emit_z[64];
	vec4 emit_color[64];
	vec4 emit_cdf[16]; // how often each is picked, as running totals (four to a vec4)
	uvec4 cache; // x: frame, y: slot mask, z: on, w: rays a texel
	// Reflection probes whose light replaces this pass's inside their box (Godot's scene shader overrides the GI
	// buffers there): world-to-local rows, and the box's inner half extents where the override is whole (w: 1 the
	// bounce light, 2 the reflections).
	vec4 probe_x[8];
	vec4 probe_y[8];
	vec4 probe_z[8];
	vec4 probe_core[8];
	ivec4 probes; // x: count
	// What a ray that leaves the scene sees: the sky's radiance map (at its roughest level: x the level, a layer or a
	// mip; y and z the map's border offset and scale; w its strength, 0: no sky map), turned by sky_x/y/z (the
	// world-to-sky rotation's rows), or else a background colour (sky_flat).
	vec4 sky;
	vec4 sky_x;
	vec4 sky_y;
	vec4 sky_z;
	vec4 sky_flat;
	// Traced + probes: the reflection probes whose capture colours a reflection's off-screen hits (probe_source) and
	// fills in the rest inside their box (probe_reflection): world-to-box rows, the box's half extents (w: the probe's
	// layer in Godot's reflection atlas times two, plus one when it's box projected), the capture point in the box (w:
	// the brightness the capture is scaled by) and in the world (w: the box's blend distance).
	vec4 src_x[8];
	vec4 src_y[8];
	vec4 src_z[8];
	vec4 src_box[8];
	vec4 src_offset[8];
	vec4 src_capture[8];
	ivec4 srcs; // x: count
	vec4 src_border; // x: the atlas maps' border, y: the scale inside it, z: the roughest level
}
f;

// Whether a reflection probe replaces this kind of light (1 bounce, 2 reflections) at world point w anyway: then
// there's no point tracing it.
bool probe_overrides(vec3 w, int what) {
	for (int i = 0; i < f.probes.x; i++) {
		if ((int(f.probe_core[i].w) & what) == 0) {
			continue;
		}
		vec3 l = vec3(dot(f.probe_x[i].xyz, w) + f.probe_x[i].w, dot(f.probe_y[i].xyz, w) + f.probe_y[i].w, dot(f.probe_z[i].xyz, w) + f.probe_z[i].w);
		if (all(lessThanEqual(abs(l), f.probe_core[i].xyz))) {
			return true;
		}
	}
	return false;
}

// Godot packs roughness with a dynamic-object flag (above one half: mirrored) and a 127 / 255 scale.
float decode_roughness(float w) {
	if (w > 0.5) {
		w = 1.0 - w;
	}
	return w / (127.0 / 255.0);
}

float linear_depth(float depth, float near, float far) {
	float z = depth * 2.0 - 1.0;
	return 2.0 * near * far / (far + near + z * (far - near));
}

vec3 view_from_pixel(vec2 px, float depth, vec4 pi, float near, float far) {
	vec3 p;
	p.z = -linear_depth(depth, near, far);
	p.xy = (px * pi.xy + pi.zw) * p.z;
	p.y = -p.y;
	return p;
}

vec2 pixel_from_view(vec3 v, vec4 pi) {
	vec2 ab = vec2(v.x, -v.y) / v.z;
	return (ab - pi.zw) / pi.xy;
}

vec3 view_from_linear(vec2 px, float linear, vec4 pi) {
	vec3 p;
	p.z = -linear;
	p.xy = (px * pi.xy + pi.zw) * p.z;
	p.y = -p.y;
	return p;
}

bool on_screen(vec2 px) {
	return all(greaterThanEqual(px, vec2(0.0))) && all(lessThan(px, vec2(f.sizes.xy)));
}

// A march step found the ray behind the depth buffer: the crossing between the step before (in front) and this one,
// by bisection. Taking the step itself puts the hit at a slightly different spot on each texel and frame, which
// shows as glitter where a surface is seen in a mirror.
float refine_hit(sampler2D depth_tex, vec3 origin, vec3 dir, float in_front, float behind) {
	for (int i = 0; i < 6; i++) {
		float mid = 0.5 * (in_front + behind);
		vec3 q = origin + dir * mid;
		vec2 qp = clamp(pixel_from_view(q, f.proj_info), vec2(0.0), vec2(f.sizes.xy) - 1.0);
		float sd = texelFetch(depth_tex, ivec2(qp), 0).r;
		float surface = sd > 0.0 ? linear_depth(sd, f.clip.x, f.clip.y) : 1e9;
		if (-q.z > surface) {
			behind = mid;
		} else {
			in_front = mid;
		}
	}
	return behind;
}

// A diffuse bounce gathers a whole hemisphere: one very bright hit (a sun glint in a mirror, on last frame's image)
// would otherwise survive the filter as a flickering blob. Its brightness is capped, far above any lit surface's.
vec3 clamp_firefly(vec3 c) {
	float l = dot(c, vec3(0.2126, 0.7152, 0.0722));
	return c * min(1.0, 4.0 / max(l, 1e-4));
}

// Where a current view-space point was on last frame's screen.
vec2 prev_pixel(vec3 v, out float prev_depth) {
	vec3 w = (f.view_to_world * vec4(v, 1.0)).xyz;
	vec3 pv = (f.world_to_prev_view * vec4(w, 1.0)).xyz;
	prev_depth = -pv.z;
	return pixel_from_view(pv, f.prev_proj_info);
}

ivec2 frame_pixel(ivec2 px, ivec2 size) {
	return ivec2((vec2(px) + 0.5) * vec2(f.sizes.xy) / vec2(size));
}
)";

// The collision shapes the rays fall back on (bindings 4 and 5), and the light leaving them; spliced in at @WORLD@ by
// the shaders that trace (which bind the depth and colour at 0 and 2).
const char *WORLD = R"(
struct Primitive {
	vec4 r0;
	vec4 r1;
	vec4 r2;
	vec4 shape;
	vec4 albedo;
	vec4 emission;
};
layout(std430, set = 0, binding = 4) restrict readonly buffer Primitives {
	Primitive p[];
}
prims;

struct Node {
	vec4 lo;
	vec4 hi;
};

// Hull planes (normal, offset), mesh tree nodes (two each, as the scene tree's) and mesh triangles (three corners).
layout(std430, set = 0, binding = 10) restrict readonly buffer Geometry {
	vec4 v[];
}
geo;

// Where a local ray enters a mesh tree node's box (1e30: it misses it, or only beyond best).
float mesh_box_entry(vec3 o, vec3 inv, int at, float best) {
	vec3 t0 = (geo.v[at].xyz - o) * inv;
	vec3 t1 = (geo.v[at + 1].xyz - o) * inv;
	vec3 near_t = min(t0, t1);
	vec3 far_t = max(t0, t1);
	float enter = max(max(near_t.x, near_t.y), max(near_t.z, 0.0));
	float leave = min(min(far_t.x, far_t.y), min(far_t.z, best));
	return enter <= leave ? enter : 1e30;
}

// The nearest triangle of a mesh along a local ray (either side), walking its own tree near child first (the far
// one waits, and is skipped once a triangle nearer than its box has been found).
bool hit_mesh(int nodes, int tris, vec3 o, vec3 d, inout float t, out vec3 n) {
	bool found = false;
	vec3 inv = 1.0 / d;
	int stack[24];
	int top = 0;
	int node = 0;
	if (mesh_box_entry(o, inv, nodes, t) >= 1e30) {
		return false;
	}
	while (true) {
		vec4 lo = geo.v[nodes + node * 2];
		vec4 hi = geo.v[nodes + node * 2 + 1];
		{
			int count = floatBitsToInt(hi.w);
			if (count == 0) {
				int left = node + 1;
				int right = floatBitsToInt(lo.w);
				float tl = mesh_box_entry(o, inv, nodes + left * 2, t);
				float tr = mesh_box_entry(o, inv, nodes + right * 2, t);
				if (tl < 1e30 && tr < 1e30) {
					bool left_first = tl <= tr;
					if (top < 24) {
						stack[top++] = left_first ? right : left;
					}
					node = left_first ? left : right;
					continue;
				} else if (tl < 1e30) {
					node = left;
					continue;
				} else if (tr < 1e30) {
					node = right;
					continue;
				}
			} else {
			int first = floatBitsToInt(lo.w);
			for (int k = first; k < first + count; k++) {
				vec3 a = geo.v[tris + k * 3].xyz;
				vec3 e1 = geo.v[tris + k * 3 + 1].xyz - a;
				vec3 e2 = geo.v[tris + k * 3 + 2].xyz - a;
				vec3 pv = cross(d, e2);
				float det = dot(e1, pv);
				if (abs(det) < 1e-12) {
					continue;
				}
				float inv_det = 1.0 / det;
				vec3 tv = o - a;
				float u = dot(tv, pv) * inv_det;
				if (u < 0.0 || u > 1.0) {
					continue;
				}
				vec3 qv = cross(tv, e1);
				float w = dot(d, qv) * inv_det;
				if (w < 0.0 || u + w > 1.0) {
					continue;
				}
				float th = dot(e2, qv) * inv_det;
				if (th > 0.0 && th < t) {
					t = th;
					n = cross(e1, e2);
					found = true;
				}
			}
			}
		}
		// The next waiting node that could still hold a nearer triangle (its box again: cheaper than keeping each
		// one's distance in registers).
		bool next = false;
		while (top > 0) {
			top--;
			if (mesh_box_entry(o, inv, nodes + stack[top] * 2, t) < 1e30) {
				node = stack[top];
				next = true;
				break;
			}
		}
		if (!next) {
			break;
		}
	}
	// Facing the ray: a mesh is hit from either side.
	if (found && dot(n, d) > 0.0) {
		n = -n;
	}
	return found;
}
layout(std430, set = 0, binding = 5) restrict readonly buffer Nodes {
	Node n[];
}
bvh;

bool hit_box(vec3 o, vec3 inv, vec3 lo, vec3 hi, float best) {
	vec3 t0 = (lo - o) * inv;
	vec3 t1 = (hi - o) * inv;
	vec3 near_t = min(t0, t1);
	vec3 far_t = max(t0, t1);
	float enter = max(max(near_t.x, near_t.y), max(near_t.z, 0.0));
	float leave = min(min(far_t.x, far_t.y), min(far_t.z, best));
	return enter <= leave;
}

// The nearest of the collision shapes along a world-space ray, closer than best; its world normal.
bool hit_primitive(Primitive q, vec3 o, vec3 d, inout float best, out vec3 normal) {
	vec3 lo = vec3(dot(q.r0.xyz, o) + q.r0.w, dot(q.r1.xyz, o) + q.r1.w, dot(q.r2.xyz, o) + q.r2.w);
	vec3 ld = vec3(dot(q.r0.xyz, d), dot(q.r1.xyz, d), dot(q.r2.xyz, d));
	vec3 n;
	float t;
	int type = int(q.shape.x + 0.5);
	if (type == 4) {
		// Convex hull: the ray's span inside every plane's half space.
		int first = floatBitsToInt(q.shape.y);
		int count = floatBitsToInt(q.shape.z);
		float t_in = -1e30;
		float t_out = best;
		for (int i = first; i < first + count; i++) {
			vec4 plane = geo.v[i];
			float denom = dot(plane.xyz, ld);
			float dist = dot(plane.xyz, lo) - plane.w;
			if (abs(denom) < 1e-12) {
				if (dist > 0.0) {
					return false;
				}
				continue;
			}
			float th = -dist / denom;
			if (denom < 0.0) {
				if (th > t_in) {
					t_in = th;
					n = plane.xyz;
				}
			} else {
				t_out = min(t_out, th);
			}
		}
		if (t_in > t_out || t_in <= 0.0 || t_in >= best) {
			return false;
		}
		t = t_in;
	} else if (type == 5) {
		t = best;
		if (!hit_mesh(floatBitsToInt(q.shape.y), floatBitsToInt(q.shape.z), lo, ld, t, n)) {
			return false;
		}
	} else if (type == 2 || type == 3) {
		// Capsule / cylinder along local y: the infinite cylinder first, then the end caps (spheres or discs).
		float r = q.shape.y;
		float half_len = type == 2 ? max(q.shape.z - r, 0.0) : q.shape.z;
		vec3 pa = vec3(0.0, -half_len, 0.0);
		vec3 ba = vec3(0.0, 2.0 * half_len, 0.0);
		vec3 oa = lo - pa;
		float baba = dot(ba, ba);
		float bard = dot(ba, ld);
		float baoa = dot(ba, oa);
		float a = baba * dot(ld, ld) - bard * bard;
		float b = baba * dot(oa, ld) - baoa * bard;
		float c = baba * dot(oa, oa) - baoa * baoa - r * r * baba;
		float h = b * b - a * c;
		if (h < 0.0) {
			return false;
		}
		h = sqrt(h);
		t = (-b - h) / a;
		float y = baoa + t * bard;
		if (y > 0.0 && y < baba) {
			n = (oa + ld * t - ba * (y / baba)) / r;
		} else if (type == 2) {
			// The sphere at the end the ray reached.
			vec3 oc = y <= 0.0 ? oa : lo - (pa + ba);
			float qa = dot(ld, ld);
			float qb = dot(oc, ld);
			float qc = dot(oc, oc) - r * r;
			float qh = qb * qb - qa * qc;
			if (qh < 0.0) {
				return false;
			}
			t = (-qb - sqrt(qh)) / qa;
			n = oc + ld * t;
		} else {
			// The disc at the end the ray reached.
			t = ((y < 0.0 ? 0.0 : baba) - baoa) / bard;
			if (abs(b + a * t) >= h) {
				return false;
			}
			n = vec3(0.0, y < 0.0 ? -1.0 : 1.0, 0.0);
		}
		if (t <= 0.0 || t >= best) {
			return false;
		}
	} else if (type == 0) {
		vec3 inv = 1.0 / ld;
		vec3 t0 = (-q.shape.yzw - lo) * inv;
		vec3 t1 = (q.shape.yzw - lo) * inv;
		vec3 lo_t = min(t0, t1);
		vec3 hi_t = max(t0, t1);
		t = max(max(lo_t.x, lo_t.y), lo_t.z);
		float out_t = min(min(hi_t.x, hi_t.y), hi_t.z);
		if (t > out_t || t <= 0.0 || t >= best) {
			return false;
		}
		n = t == lo_t.x ? vec3(-sign(ld.x), 0.0, 0.0) : (t == lo_t.y ? vec3(0.0, -sign(ld.y), 0.0) : vec3(0.0, 0.0, -sign(ld.z)));
	} else {
		float a = dot(ld, ld);
		float b = dot(lo, ld);
		float c = dot(lo, lo) - q.shape.y * q.shape.y;
		float h = b * b - a * c;
		if (h < 0.0) {
			return false;
		}
		t = (-b - sqrt(h)) / a;
		if (t <= 0.0 || t >= best) {
			return false;
		}
		n = lo + ld * t;
	}
	best = t;
	// Local to world for a normal: the transpose of the world-to-local rows.
	normal = normalize(n.x * q.r0.xyz + n.y * q.r1.xyz + n.z * q.r2.xyz);
	return true;
}

// Where the ray enters a node's box (1e30: it misses it, or only beyond best).
float box_entry(vec3 o, vec3 inv, vec3 lo, vec3 hi, float best) {
	vec3 t0 = (lo - o) * inv;
	vec3 t1 = (hi - o) * inv;
	vec3 near_t = min(t0, t1);
	vec3 far_t = max(t0, t1);
	float enter = max(max(near_t.x, near_t.y), max(near_t.z, 0.0));
	float leave = min(min(far_t.x, far_t.y), min(far_t.z, best));
	return enter <= leave ? enter : 1e30;
}

)"
					R"(
#ifdef MESH_WALK
// With triangle meshes among the shapes: the shape tree and the meshes' trees walked in one loop, a node a step (a
// shape tree node, or ~node: a node of the mesh being walked, in its own space). Rays in different meshes keep
// stepping together, where walking each mesh in a loop of its own leaves the rest of their group idle until it's done.
// The shape tree then has leaves of one shape; a mesh's entries sit above the shape tree's on the shared stack, so
// popping one of the shape tree's means the mesh is done.
int walk_world(vec3 o, vec3 d, inout float t, out vec3 normal, bool any_hit) {
	int hit = -1;
	if (f.counts.y == 0) {
		return hit;
	}
	vec3 inv = 1.0 / d;
	if (box_entry(o, inv, bvh.n[0].lo.xyz, bvh.n[0].hi.xyz, t) >= 1e30) {
		return hit;
	}
	int stack[40];
	int top = 0;
	int node = 0;
	int mesh = -1; // the record whose mesh is being walked
	int mesh_nodes = 0;
	int mesh_tris = 0;
	vec3 lo = o;
	vec3 ld = d;
	vec3 linv = inv;
	while (true) {
		bool in_mesh = node < 0;
		int at = in_mesh ? ~node : node;
		vec4 nlo;
		vec4 nhi;
		if (in_mesh) {
			nlo = geo.v[mesh_nodes + at * 2];
			nhi = geo.v[mesh_nodes + at * 2 + 1];
		} else {
			nlo = bvh.n[at].lo;
			nhi = bvh.n[at].hi;
		}
		int count = floatBitsToInt(nhi.w);
		bool entered = false;
		if (count == 0) {
			int left = at + 1;
			int right = floatBitsToInt(nlo.w);
			vec3 ro = in_mesh ? lo : o;
			vec3 rinv = in_mesh ? linv : inv;
			vec4 al;
			vec4 ah;
			vec4 bl;
			vec4 bh;
			if (in_mesh) {
				al = geo.v[mesh_nodes + left * 2];
				ah = geo.v[mesh_nodes + left * 2 + 1];
				bl = geo.v[mesh_nodes + right * 2];
				bh = geo.v[mesh_nodes + right * 2 + 1];
			} else {
				al = bvh.n[left].lo;
				ah = bvh.n[left].hi;
				bl = bvh.n[right].lo;
				bh = bvh.n[right].hi;
			}
			float tl = box_entry(ro, rinv, al.xyz, ah.xyz, t);
			float tr = box_entry(ro, rinv, bl.xyz, bh.xyz, t);
			int el = in_mesh ? ~left : left;
			int er = in_mesh ? ~right : right;
			if (tl < 1e30 && tr < 1e30) {
				bool left_first = tl <= tr;
				if (top < 40) {
					stack[top++] = left_first ? er : el;
				}
				node = left_first ? el : er;
				continue;
			} else if (tl < 1e30) {
				node = el;
				continue;
			} else if (tr < 1e30) {
				node = er;
				continue;
			}
		} else if (in_mesh) {
			int first = floatBitsToInt(nlo.w);
			for (int k = first; k < first + count; k++) {
				vec3 a = geo.v[mesh_tris + k * 3].xyz;
				vec3 e1 = geo.v[mesh_tris + k * 3 + 1].xyz - a;
				vec3 e2 = geo.v[mesh_tris + k * 3 + 2].xyz - a;
				vec3 pv = cross(ld, e2);
				float det = dot(e1, pv);
				if (abs(det) < 1e-12) {
					continue;
				}
				float inv_det = 1.0 / det;
				vec3 tv = lo - a;
				float u = dot(tv, pv) * inv_det;
				if (u < 0.0 || u > 1.0) {
					continue;
				}
				vec3 qv = cross(tv, e1);
				float w = dot(ld, qv) * inv_det;
				if (w < 0.0 || u + w > 1.0) {
					continue;
				}
				float th = dot(e2, qv) * inv_det;
				if (th > 0.0 && th < t) {
					t = th;
					vec3 n = cross(e1, e2);
					if (dot(n, ld) > 0.0) {
						n = -n;
					}
					Primitive q = prims.p[mesh];
					normal = normalize(n.x * q.r0.xyz + n.y * q.r1.xyz + n.z * q.r2.xyz);
					hit = mesh;
					if (any_hit) {
						return hit;
					}
				}
			}
		} else {
			int first = floatBitsToInt(nlo.w);
			for (int i = first; i < first + count; i++) {
				vec4 shape = prims.p[i].shape;
				if (int(shape.x + 0.5) == 5) {
					// Into a mesh: the ray in its space, its own tree from its root (a leaf holds one shape).
					Primitive q = prims.p[i];
					vec3 mo = vec3(dot(q.r0.xyz, o) + q.r0.w, dot(q.r1.xyz, o) + q.r1.w, dot(q.r2.xyz, o) + q.r2.w);
					vec3 md = vec3(dot(q.r0.xyz, d), dot(q.r1.xyz, d), dot(q.r2.xyz, d));
					vec3 minv = 1.0 / md;
					int mn = floatBitsToInt(shape.y);
					if (mesh_box_entry(mo, minv, mn, t) < 1e30) {
						mesh = i;
						mesh_nodes = mn;
						mesh_tris = floatBitsToInt(shape.z);
						lo = mo;
						ld = md;
						linv = minv;
						entered = true;
						break;
					}
				} else {
					vec3 n;
					if (hit_primitive(prims.p[i], o, d, t, n)) {
						hit = i;
						normal = n;
						if (any_hit) {
							return hit;
						}
					}
				}
			}
		}
		if (entered) {
			node = ~0;
			continue;
		}
		// The next waiting node that could still hold something nearer.
		bool found = false;
		while (top > 0) {
			top--;
			int e = stack[top];
			if (e < 0) {
				vec4 bl = geo.v[mesh_nodes + (~e) * 2];
				vec4 bh = geo.v[mesh_nodes + (~e) * 2 + 1];
				if (box_entry(lo, linv, bl.xyz, bh.xyz, t) < 1e30) {
					node = e;
					found = true;
					break;
				}
			} else {
				mesh = -1;
				if (box_entry(o, inv, bvh.n[e].lo.xyz, bvh.n[e].hi.xyz, t) < 1e30) {
					node = e;
					found = true;
					break;
				}
			}
		}
		if (!found) {
			break;
		}
	}
	return hit;
}
#else
// The nearest shape along the ray, walking the tree near child first (the far one waits on the stack, and is skipped
// when a hit closer than its box has been found by then). With any_hit, the first shape in the way will do (shadow
// rays): returns at once.
int walk_world(vec3 o, vec3 d, inout float t, out vec3 normal, bool any_hit) {
	int hit = -1;
	if (f.counts.y == 0) {
		return hit;
	}
	vec3 inv = 1.0 / d;
	int stack[16];
	int top = 0;
	int node = 0;
	if (box_entry(o, inv, bvh.n[0].lo.xyz, bvh.n[0].hi.xyz, t) >= 1e30) {
		return hit;
	}
	while (true) {
		Node nd = bvh.n[node];
		int count = floatBitsToInt(nd.hi.w);
		if (count == 0) {
			int left = node + 1;
			int right = floatBitsToInt(nd.lo.w);
			float tl = box_entry(o, inv, bvh.n[left].lo.xyz, bvh.n[left].hi.xyz, t);
			float tr = box_entry(o, inv, bvh.n[right].lo.xyz, bvh.n[right].hi.xyz, t);
			if (tl < 1e30 && tr < 1e30) {
				bool left_first = tl <= tr;
				if (top < 16) {
					stack[top] = left_first ? right : left;
					top++;
				}
				node = left_first ? left : right;
				continue;
			} else if (tl < 1e30) {
				node = left;
				continue;
			} else if (tr < 1e30) {
				node = right;
				continue;
			}
		} else {
			int first = floatBitsToInt(nd.lo.w);
			for (int i = first; i < first + count; i++) {
				vec3 n;
				if (hit_primitive(prims.p[i], o, d, t, n)) {
					hit = i;
					normal = n;
					if (any_hit) {
						return hit;
					}
				}
			}
		}
		// The next waiting node that could still hold something nearer than what's been hit.
		bool found = false;
		while (top > 0) {
			top--;
			// Its box again (cheaper than keeping each waiting node's distance in registers).
			if (box_entry(o, inv, bvh.n[stack[top]].lo.xyz, bvh.n[stack[top]].hi.xyz, t) < 1e30) {
				node = stack[top];
				found = true;
				break;
			}
		}
		if (!found) {
			break;
		}
	}
	return hit;
}

#endif

int trace_world(vec3 o, vec3 d, inout float t, out vec3 normal) {
	return walk_world(o, d, t, normal, false);
}

// Whether anything lies along the ray before reach (shadow rays).
bool blocked(vec3 o, vec3 d, float reach) {
	vec3 n;
	return walk_world(o, d, reach, n, true) >= 0;
}

// The radiance cache. A slot's key is a fingerprint of its cell (0: empty); its cell holds where it was first hit
// (w: the frame a pixel's ray last used it), its normal (w: refinements so far), its albedo, the bounce light arriving
// (a running mean, as the GI buffer holds it) and the light leaving it.
layout(std430, set = 0, binding = 6) restrict buffer CacheKeys {
	uint k[];
}
cache_keys;
struct Cell {
	vec4 pos;
	vec4 normal;
	vec4 albedo;
	vec4 incoming;
	vec4 outgoing;
};
layout(std430, set = 0, binding = 7) restrict buffer CacheCells {
	Cell c[];
}
cells;

uint hash_u(uint x) {
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x;
}

// The slot of the cell holding x (facing n), claiming an empty one when asked; -1 when the table is full there.
int cache_slot(vec3 x, vec3 n, bool claim, out bool claimed) {
	claimed = false;
	// Cells a quarter metre across near the camera, doubling with each doubling of distance past 8 m.
	float dist = length(x - f.view_to_world[3].xyz);
	uint level = uint(clamp(floor(log2(max(dist, 1.0) / 8.0)) + 1.0, 0.0, 5.0));
	ivec3 cell = ivec3(floor(x / (0.25 * float(1u << level))));
	vec3 a = abs(n);
	uint facing = a.x > a.y && a.x > a.z ? (n.x > 0.0 ? 0u : 1u) : (a.y > a.z ? (n.y > 0.0 ? 2u : 3u) : (n.z > 0.0 ? 4u : 5u));
	uint h = hash_u(uint(cell.x) + hash_u(uint(cell.y) + hash_u(uint(cell.z) + hash_u(facing * 8u + level))));
	uint key = hash_u(h ^ 0x9e3779b9u) | 1u;
	for (uint i = 0u; i < 8u; i++) {
		uint slot = (h + i) & f.cache.y;
		uint was = cache_keys.k[slot];
		if (was == key) {
			return int(slot);
		}
		if (was == 0u) {
			if (!claim) {
				return -1;
			}
			was = atomicCompSwap(cache_keys.k[slot], 0u, key);
			if (was == 0u) {
				claimed = true;
				return int(slot);
			}
			if (was == key) {
				return int(slot);
			}
		}
	}
	return -1;
}

// The bounce light arriving at x (facing n), or with leaving the light leaving it, blended from the eight refined cells
// around it as a texture is filtered: one cell's own value shows as a square patch where a reflection looks at it.
// Black when none is refined yet.
vec3 smooth_cache(vec3 x, vec3 n, bool leaving) {
	float dist = length(x - f.view_to_world[3].xyz);
	uint level = uint(clamp(floor(log2(max(dist, 1.0) / 8.0)) + 1.0, 0.0, 5.0));
	vec3 c = x / (0.25 * float(1u << level)) - 0.5;
	ivec3 base = ivec3(floor(c));
	vec3 t = c - vec3(base);
	vec3 a = abs(n);
	uint facing = a.x > a.y && a.x > a.z ? (n.x > 0.0 ? 0u : 1u) : (a.y > a.z ? (n.y > 0.0 ? 2u : 3u) : (n.z > 0.0 ? 4u : 5u));
	vec4 sum = vec4(0.0);
	for (int i = 0; i < 8; i++) {
		ivec3 o = ivec3(i & 1, (i >> 1) & 1, i >> 2);
		ivec3 cell = base + o;
		uint h = hash_u(uint(cell.x) + hash_u(uint(cell.y) + hash_u(uint(cell.z) + hash_u(facing * 8u + level))));
		uint key = hash_u(h ^ 0x9e3779b9u) | 1u;
		for (uint j = 0u; j < 8u; j++) {
			uint slot = (h + j) & f.cache.y;
			uint was = cache_keys.k[slot];
			if (was == key) {
				if (cells.c[slot].normal.w > 0.5) {
					vec3 w3 = mix(1.0 - t, t, vec3(o));
					float w = w3.x * w3.y * w3.z;
					sum += vec4((leaving ? cells.c[slot].outgoing.rgb : cells.c[slot].incoming.rgb) * w, w);
				}
				break;
			}
			if (was == 0u) {
				break;
			}
		}
	}
	return sum.w > 1e-4 ? sum.rgb / sum.w : vec3(0.0);
}

#ifdef SKY_OCTMAP_ARRAY
layout(set = 0, binding = 11) uniform sampler2DArray sky_tex;
#else
layout(set = 0, binding = 11) uniform sampler2D sky_tex;
#endif

// Godot's octahedral mapping of a direction into its sky radiance map (oct_inc.glsl).
vec2 sky_oct_wrap(vec2 v) {
	return (1.0 - abs(v.yx)) * vec2(v.x >= 0.0 ? 1.0 : -1.0, v.y >= 0.0 ? 1.0 : -1.0);
}

vec2 sky_oct(vec3 n) {
	n /= abs(n.x) + abs(n.y) + abs(n.z);
	n.xy = n.z >= 0.0 ? n.xy : sky_oct_wrap(n.xy);
	return (n.xy * 0.5 + 0.5) * f.sky.z + f.sky.y;
}

)"
					R"(
// The light from the background a ray sees when it leaves the scene going along d: the sky (blurred, as many rays
// average it), or the background colour.
vec3 background_light(vec3 d) {
	if (f.sky.w <= 0.0) {
		return f.sky_flat.rgb;
	}
	vec3 s = vec3(dot(f.sky_x.xyz, d), dot(f.sky_y.xyz, d), dot(f.sky_z.xyz, d));
#ifdef SKY_OCTMAP_ARRAY
	return textureLod(sky_tex, vec3(sky_oct(s), f.sky.x), 0.0).rgb * f.sky.w;
#else
	return textureLod(sky_tex, sky_oct(s), f.sky.x).rgb * f.sky.w;
#endif
}

// Whether a ray that hit nothing within reach has left the scene (the box around every shape): only then does it see
// the background. One that ran out of reach inside a large closed space (a long tunnel) doesn't.
bool escapes(vec3 o, vec3 d, float reach) {
	if (f.counts.y == 0) {
		return true;
	}
	vec3 inv = 1.0 / d;
	vec3 t0 = (bvh.n[0].lo.xyz - o) * inv;
	vec3 t1 = (bvh.n[0].hi.xyz - o) * inv;
	vec3 far_t = max(t0, t1);
	return min(min(far_t.x, far_t.y), far_t.z) <= reach;
}

// The light a surface patch receives straight from the sun and the omni lights (shadowed by the shapes), and the
// ambient light: what it shows times its albedo, before any bounce light.
vec3 direct_light(vec3 x, vec3 n) {
	vec3 light = f.ambient.rgb;
	float lit = dot(n, f.sun_direction.xyz);
	if (lit > 0.0 && f.sun_direction.w > 0.5) {
		if (!blocked(x + n * 0.02, f.sun_direction.xyz, 1e6)) {
			light += f.sun_color.rgb * lit;
		}
	}
	// Omni lights, falling off as Godot's do, shadowed by the shapes.
	for (int i = 0; i < f.counts.z; i++) {
		vec3 to = f.omni_position[i].xyz - x;
		float dist = length(to);
		vec3 l = to / max(dist, 1e-4);
		float nl = dot(n, l);
		if (nl <= 0.0) {
			continue;
		}
		float nd = dist * f.omni_position[i].w;
		nd *= nd;
		nd *= nd;
		nd = max(1.0 - nd, 0.0);
		float falloff = nd * nd * pow(max(dist, 1e-4), -f.omni_color[i].w);
		if (falloff <= 0.0) {
			continue;
		}
		if (blocked(x + n * 0.02, l, dist - 0.05)) {
			continue;
		}
		light += f.omni_color[i].rgb * nl * falloff;
	}
	// Spot lights: the omni falloff inside a cone, its edge softened as Godot's are.
	for (int i = 0; i < f.counts.w; i++) {
		vec3 to = f.spot_position[i].xyz - x;
		float dist = length(to);
		vec3 l = to / max(dist, 1e-4);
		float nl = dot(n, l);
		if (nl <= 0.0) {
			continue;
		}
		float cos_cone = f.spot_direction[i].w;
		float scos = max(dot(-l, f.spot_direction[i].xyz), cos_cone);
		float rim = max(1e-4, (1.0 - scos) / (1.0 - cos_cone));
		float cone = 1.0 - pow(rim, f.spot_cone[i].x);
		float nd = dist * f.spot_position[i].w;
		nd *= nd;
		nd *= nd;
		nd = max(1.0 - nd, 0.0);
		float falloff = nd * nd * pow(max(dist, 1e-4), -f.spot_color[i].w) * cone;
		if (falloff <= 0.0) {
			continue;
		}
		if (blocked(x + n * 0.02, l, dist - 0.05)) {
			continue;
		}
		light += f.spot_color[i].rgb * nl * falloff;
	}
	return light;
}

// The light leaving a shape's surface toward the ray: last frame's image where that point was on it; otherwise what it
// emits, plus its radiance cache cell (bounce light included) once that has been refined, or else the direct light on
// its albedo. A pixel's ray (from_pixels) claims the cell for the cache and keeps it alive. A sharp one (a reflection,
// which shows the point itself) takes the cells around it blended, as a cell is a quarter metre or more across, too
// coarse to show as it is; a mirror's (mirror_hit) lights the point directly and takes only their bounce light (a
// glossy surface's rays land somewhere new each frame, and lit directly they'd flicker between light and shadow).
// Whether the shape in record slot p_hit is one of the emissives sampled directly.
bool sampled_emissive(int p_hit) {
	for (int i = 0; i < int(f.reflect.z); i++) {
		if (int(f.emit_center[i].w) == p_hit) {
			return true;
		}
	}
	return false;
}

vec3 world_light(vec3 x, vec3 n, int hit, sampler2D depth_tex, sampler2D color_tex, bool from_pixels, bool with_glow, bool sharp) {
	// Without with_glow only the emissives sampled directly go without their glow here (it would count them twice);
	// every other one glows as it is hit.
	bool glows = with_glow || !sampled_emissive(hit);
	vec3 v = (f.world_to_view * vec4(x, 1.0)).xyz;
	if (v.z < -f.clip.x) {
		vec2 sp = pixel_from_view(v, f.proj_info);
		if (on_screen(sp)) {
			float sd = texelFetch(depth_tex, ivec2(sp), 0).r;
			float surface = sd > 0.0 ? linear_depth(sd, f.clip.x, f.clip.y) : 1e9;
			if (abs(surface + v.z) < 0.1 + 0.02 * surface) {
				float pd;
				vec2 pp = prev_pixel(v, pd);
				if (on_screen(pp)) {
					vec3 seen = texture(color_tex, (pp + 0.5) / vec2(f.sizes.xy)).rgb;
					return glows ? seen : max(seen - prims.p[hit].emission.rgb * f.sun_color.w, vec3(0.0));
				}
			}
		}
	}
#ifdef PROBE_SOURCES
	// Off screen, a reflection probe's capture may have seen the point itself (textures and all).
	if (sharp && probe_texture) {
		vec3 seen;
		if (probe_source(x, n, hit, seen)) {
			return seen;
		}
	}
#endif
	vec3 albedo = prims.p[hit].albedo.rgb;
	vec3 glow = glows ? prims.p[hit].emission.rgb * f.sun_color.w : vec3(0.0);
	if (f.cache.z != 0u) {
		bool claimed;
		int slot = cache_slot(x, n, from_pixels, claimed);
		if (slot >= 0) {
			if (claimed) {
				cells.c[slot].pos = vec4(x, uintBitsToFloat(f.cache.x));
				cells.c[slot].normal = vec4(n, 0.0);
				cells.c[slot].albedo = vec4(albedo, 0.0);
				cells.c[slot].incoming = vec4(0.0);
			} else {
				if (from_pixels) {
					cells.c[slot].pos.w = uintBitsToFloat(f.cache.x);
				}
				if (cells.c[slot].normal.w > 0.5) {
#ifdef PROBE_SOURCES
					bool lit_here = sharp && mirror_hit;
#else
					bool lit_here = sharp;
#endif
					if (!sharp) {
						return glow + cells.c[slot].outgoing.rgb;
					}
					return glow + (lit_here ? albedo * (direct_light(x, n) + smooth_cache(x, n, false)) : smooth_cache(x, n, true));
				}
			}
		}
	}
	return glow + albedo * direct_light(x, n);
}

// The emissive whose box holds world point w (a little grown), or -1.
int emissive_at(vec3 w) {
	for (int i = 0; i < int(f.reflect.z); i++) {
		vec3 d = w - f.emit_center[i].xyz;
		vec3 ax = f.emit_x[i].xyz;
		vec3 ay = f.emit_y[i].xyz;
		vec3 az = f.emit_z[i].xyz;
		if (abs(dot(d, ax)) <= dot(ax, ax) + 0.03 * length(ax) && abs(dot(d, ay)) <= dot(ay, ay) + 0.03 * length(ay) && abs(dot(d, az)) <= dot(az, az) + 0.03 * length(az)) {
			return i;
		}
	}
	return -1;
}

// How far along l from x a ray enters the box (centre c, half axes ax, ay, az) grown by p_grow; 0 from inside it. A box
// flat along one axis (a panel) takes that axis from the other two.
float emissive_entry(vec3 x, vec3 l, vec3 c, vec3 ax, vec3 ay, vec3 az, float p_grow) {
	vec3 lens = vec3(length(ax), length(ay), length(az));
	vec3 ux = lens.x > 1e-5 ? ax / lens.x : normalize(cross(ay, az));
	vec3 uy = lens.y > 1e-5 ? ay / lens.y : normalize(cross(az, ax));
	vec3 uz = lens.z > 1e-5 ? az / lens.z : normalize(cross(ax, ay));
	vec3 h = lens + p_grow;
	vec3 o = vec3(dot(x - c, ux), dot(x - c, uy), dot(x - c, uz));
	vec3 d = vec3(dot(l, ux), dot(l, uy), dot(l, uz));
	d = mix(d, vec3(1e-6), lessThan(abs(d), vec3(1e-6)));
	vec3 t1 = (-h - o) / d;
	vec3 t2 = (h - o) / d;
	vec3 lo = min(t1, t2);
	return max(max(lo.x, max(lo.y, lo.z)), 0.0);
}

// One emissive (picked uniformly), one point on the faces of its box that look toward x (a face pair chosen by its
// area), a shadow ray: the light it gives x, as the GI buffer holds light (irradiance / pi).
vec3 sample_emissive(vec3 x, vec3 n, vec2 xi, float pick, int count) {
	// The emissive whose share of the running totals holds pick (a binary search), and its share.
	int lo = 0;
	int hi = count - 1;
	while (lo < hi) {
		int mid = (lo + hi) >> 1;
		if (pick < f.emit_cdf[mid >> 2][mid & 3]) {
			hi = mid;
		} else {
			lo = mid + 1;
		}
	}
	int i = lo;
	float share = f.emit_cdf[i >> 2][i & 3] - (i > 0 ? f.emit_cdf[(i - 1) >> 2][(i - 1) & 3] : 0.0);
	if (share <= 0.0) {
		return vec3(0.0);
	}
	vec3 c = f.emit_center[i].xyz;
	vec3 ax = f.emit_x[i].xyz;
	vec3 ay = f.emit_y[i].xyz;
	vec3 az = f.emit_z[i].xyz;
	float lx = length(ax);
	float ly = length(ay);
	float lz = length(az);
	float a0 = ly * lz;
	float a1 = lx * lz;
	float a2 = lx * ly;
	float total = a0 + a1 + a2;
	if (total <= 0.0) {
		return vec3(0.0);
	}
	vec3 face;
	vec3 u;
	vec3 v;
	vec3 face_n;
	// The area the point was picked over (a quarter of it: the axes are half axes).
	float area;
	vec3 facing = vec3(f.emit_x[i].w, f.emit_y[i].w, f.emit_z[i].w);
	if (dot(facing, facing) > 0.5) {
		// One-sided (a panel, a strip): only its front face glows, and only toward what is in front of it. The front
		// face: the box face whose axis lines up best with the way it faces (for a flat box, its flat axis).
		if (dot(x - c, facing) <= 0.0) {
			return vec3(0.0);
		}
		vec3 al = abs(vec3(dot(facing, ax) / max(lx, 1e-6), dot(facing, ay) / max(ly, 1e-6), dot(facing, az) / max(lz, 1e-6)));
		if (lx < 1e-5) {
			al.x = 2.0;
		} else if (ly < 1e-5) {
			al.y = 2.0;
		} else if (lz < 1e-5) {
			al.z = 2.0;
		}
		if (al.x >= al.y && al.x >= al.z) {
			face_n = lx > 1e-5 ? ax / lx : normalize(cross(ay, az));
			face = face_n * lx;
			u = ay;
			v = az;
			area = a0;
		} else if (al.y >= al.z) {
			face_n = ly > 1e-5 ? ay / ly : normalize(cross(az, ax));
			face = face_n * ly;
			u = ax;
			v = az;
			area = a1;
		} else {
			face_n = lz > 1e-5 ? az / lz : normalize(cross(ax, ay));
			face = face_n * lz;
			u = ax;
			v = ay;
			area = a2;
		}
		if (dot(face_n, facing) < 0.0) {
			face = -face;
			face_n = -face_n;
		}
		if (area <= 0.0) {
			return vec3(0.0);
		}
	} else {
		float r = xi.x * total;
		if (r < a0) {
			face = ax;
			u = ay;
			v = az;
			xi.x = r / a0;
		} else if (r < a0 + a1) {
			face = ay;
			u = ax;
			v = az;
			xi.x = (r - a0) / a1;
		} else {
			face = az;
			u = ax;
			v = ay;
			xi.x = (r - a0 - a1) / a2;
		}
		face_n = normalize(face);
		if (dot(x - c, face_n) < 0.0) {
			face = -face;
			face_n = -face_n;
		}
		area = total;
	}
	vec3 point = c + face + u * (2.0 * xi.x - 1.0) + v * (2.0 * xi.y - 1.0);
	vec3 to = point - x;
	float d2 = max(dot(to, to), 1e-2);
	float dist = sqrt(d2);
	vec3 l = to / dist;
	float cos_x = dot(n, l);
	float cos_e = dot(face_n, -l);
	// The shadow ray stops where it enters the emissive's box (a little grown): collision is often coarser than what
	// is drawn, and a lamp mounted on a wall or ceiling sits partly inside it. That collision would hide part of the
	// lamp from its own sample points, a different part each frame (a flicker); a real occluder lies outside the box.
	float reach = min(dist - 0.05, emissive_entry(x, l, c, ax, ay, az, 0.05) - 0.01);
	if (cos_x <= 0.0 || cos_e <= 0.0 || (reach > 0.0 && blocked(x + n * 0.03, l, reach))) {
		return vec3(0.0);
	}
	// pdf over the area picked from: 1 / (4 * area); this emissive picked with its share.
	vec3 e = f.emit_color[i].rgb * cos_x * cos_e * (4.0 * area) / d2 / share / 3.14159265;
	float lum = dot(e, vec3(0.2126, 0.7152, 0.0722));
	return e * min(1.0, 8.0 / max(lum, 1e-4));
}
)";

const char *SHADERS[8] = {
	// PASS_TRACE: a few rays per texel from the surface, cosine-weighted about its normal, marched against the depth
	// buffer; a hit takes the light that point had on last frame's image. Stores the hits' light / rays (so,
	// premultiplied) and the fraction of rays that hit: misses are left to Godot's own ambient light.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@COMMON@
layout(set = 0, binding = 0) uniform sampler2D depth_tex;
layout(set = 0, binding = 1) uniform sampler2D normal_tex;
layout(set = 0, binding = 2) uniform sampler2D color_tex;
layout(rgba16f, set = 0, binding = 3) uniform restrict writeonly image2D dst;

@WORLD@

// Interleaved gradient noise: neighbouring texels get evenly spread values (blue-ish noise), which a small filter and
// a few frames average out far faster than white noise. Each dimension of a ray's sample (its direction's two angles,
// the march's jitter) takes its own, differently offset noise, stepped along an R2 sequence by frame and ray, so a
// texel's directions cover the hemisphere evenly over time.
float ign(vec2 p) {
	return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715))));
}

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, f.sizes.zw))) {
		return;
	}
	ivec2 fp = frame_pixel(px, f.sizes.zw);
	float depth = texelFetch(depth_tex, fp, 0).r;
	if (depth <= 0.0) {
		imageStore(dst, px, vec4(0.0));
		return;
	}
	vec3 p = view_from_pixel(vec2(fp), depth, f.proj_info, f.clip.x, f.clip.y);
	if (probe_overrides((f.view_to_world * vec4(p, 1.0)).xyz, 1)) {
		imageStore(dst, px, vec4(0.0));
		return;
	}
	vec3 n = normalize(texelFetch(normal_tex, fp, 0).xyz * 2.0 - 1.0);
	vec3 t = normalize(cross(n, abs(n.y) < 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0)));
	vec3 b = cross(n, t);
	vec3 noise = vec3(ign(vec2(px)), ign(vec2(px) + vec2(47.0, 17.0)), ign(vec2(px) + vec2(13.0, 101.0)));
	int emissives = int(f.reflect.z);

	const int RAYS = clamp(int(f.cache.w), 1, 4);
	const int STEPS = 24;
	vec3 light = vec3(0.0);
	float hits = 0.0;
	vec3 origin = p + n * (0.02 + 0.002 * -p.z);
	vec3 world_origin = (f.view_to_world * vec4(origin, 1.0)).xyz;
	for (int r = 0; r < RAYS; r++) {
		float index = f.misc.x * float(RAYS) + float(r);
		float u = fract(noise.x + index * 0.7548776662);
		float phi = 6.2831853 * fract(noise.y + index * 0.5698402910);
		vec3 dir = normalize(t * (sqrt(u) * cos(phi)) + b * (sqrt(u) * sin(phi)) + n * sqrt(max(0.0, 1.0 - u)));
		float jitter = fract(noise.z + index * 0.6180339887);
		bool found = false;
		float last = 0.0;
		// Debug mode 3: only the collision shapes' light, to see what the world trace adds.
		for (int s = 0; s < (int(f.misc.w) >= 3 ? 0 : STEPS); s++) {
			float along = f.misc.y * pow((float(s) + jitter) / float(STEPS), 2.0);
			float in_front = last;
			last = along;
			// How far this step went: a surface thinner than that could be stepped over.
			float stride = f.misc.y * 2.0 * (float(s) + jitter + 1.0) / float(STEPS * STEPS);
			vec3 q = origin + dir * along;
			if (q.z > -f.clip.x) {
				break;
			}
			vec2 qp = pixel_from_view(q, f.proj_info);
			if (!on_screen(qp)) {
				break;
			}
			float sd = texelFetch(depth_tex, ivec2(qp), 0).r;
			if (sd <= 0.0) {
				continue;
			}
			float surface = linear_depth(sd, f.clip.x, f.clip.y);
			float behind = -q.z - surface;
			if (behind > 0.0 && behind < max(0.1 + 0.02 * surface, 1.5 * stride)) {
				// The surface the ray reached, and its light on last frame's image.
				vec2 hp = clamp(pixel_from_view(origin + dir * refine_hit(depth_tex, origin, dir, in_front, along), f.proj_info), vec2(0.0), vec2(f.sizes.xy) - 1.0);
				float hd = texelFetch(depth_tex, ivec2(hp), 0).r;
				float pd;
				vec2 pp = prev_pixel(view_from_pixel(floor(hp), hd > 0.0 ? hd : sd, f.proj_info, f.clip.x, f.clip.y), pd);
				if (on_screen(pp)) {
					vec3 seen = texture(color_tex, (pp + 0.5) / vec2(f.sizes.xy)).rgb;
					if (emissives > 0) {
						int e = emissive_at((f.view_to_world * vec4(view_from_pixel(floor(hp), hd > 0.0 ? hd : sd, f.proj_info, f.clip.x, f.clip.y), 1.0)).xyz);
						if (e >= 0) {
							seen = max(seen - f.emit_color[e].rgb, vec3(0.0));
						}
					}
					light += clamp_firefly(seen);
					hits += 1.0;
					found = true;
				}
				break;
			}
		}
		if (!found && f.counts.x > 0) {
			// Off the screen, or past what it shows: the collision shapes.
			vec3 world_dir = mat3(f.view_to_world) * dir;
			float t = f.ambient.w;
			vec3 wn;
			int hit = trace_world(world_origin, world_dir, t, wn);
			if (hit >= 0) {
				light += clamp_firefly(world_light(world_origin + world_dir * t, wn, hit, depth_tex, color_tex, true, emissives == 0, false));
				hits += 1.0;
			}
		}
	}
	vec4 result = vec4(light, hits) / float(RAYS);
	// The emissives, sampled directly: one, a point on its surface, a shadow ray. Its light is what the texel's rays
	// would have gathered from it by chance (they no longer count its glow), in the GI buffer's units (the mean of the
	// light arriving, cosine weighted: irradiance / pi).
	if (emissives > 0) {
		vec3 xw = (f.view_to_world * vec4(p, 1.0)).xyz;
		vec3 nw = normalize(mat3(f.view_to_world) * n);
		float index = f.misc.x;
		vec2 xi = vec2(fract(noise.x + 0.37 + index * 0.7548776662), fract(noise.y + 0.71 + index * 0.5698402910));
		float pick = fract(noise.z + 0.13 + index * 0.6180339887);
		result.rgb += sample_emissive(xw, nw, xi, pick, emissives);
	}
	// One bad sample would otherwise live on in the accumulation forever.
	imageStore(dst, px, any(isnan(result)) || any(isinf(result)) ? vec4(0.0) : result);
}
)",
	// PASS_TEMPORAL: this frame's rays blended into what the same surface accumulated before, found by reprojecting
	// it onto last frame; a different depth there (something moved in front, or it was off screen) starts over.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@COMMON@
layout(push_constant, std430) uniform Params {
	ivec4 history; // x: there is history
}
pc;
layout(set = 0, binding = 0) uniform sampler2D trace_tex;
layout(set = 0, binding = 1) uniform sampler2D prev_accum;
layout(set = 0, binding = 2) uniform sampler2D prev_depth_tex;
layout(set = 0, binding = 3) uniform sampler2D depth_tex;
layout(rgba16f, set = 0, binding = 4) uniform restrict writeonly image2D accum;
layout(r32f, set = 0, binding = 5) uniform restrict writeonly image2D accum_depth;
layout(set = 0, binding = 6) uniform sampler2D prev_moments;
layout(rgba16f, set = 0, binding = 7) uniform restrict writeonly image2D moments;
layout(r32f, set = 0, binding = 8) uniform restrict writeonly image2D variance;

float luminance(vec3 c) {
	return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, f.sizes.zw))) {
		return;
	}
	ivec2 fp = frame_pixel(px, f.sizes.zw);
	float depth = texelFetch(depth_tex, fp, 0).r;
	if (depth <= 0.0) {
		imageStore(accum, px, vec4(0.0));
		imageStore(accum_depth, px, vec4(0.0));
		imageStore(moments, px, vec4(0.0));
		imageStore(variance, px, vec4(0.0));
		return;
	}
	vec3 p = view_from_pixel(vec2(fp), depth, f.proj_info, f.clip.x, f.clip.y);
	vec4 now = texelFetch(trace_tex, px, 0);
	float l = luminance(now.rgb);
	vec4 result = now;
	vec3 m = vec3(l, l * l, 1.0); // mean, mean of the square, frames of history
	if (pc.history.x != 0) {
		float expected;
		vec2 pp = prev_pixel(p, expected);
		if (on_screen(pp)) {
			vec2 uv = (pp + 0.5) / vec2(f.sizes.xy);
			float was = texture(prev_depth_tex, uv).r;
			if (abs(was - expected) < 0.05 * expected) {
				vec4 was_light = texture(prev_accum, uv);
				vec3 was_m = texture(prev_moments, uv).xyz;
				if (!any(isnan(was_light)) && !any(isnan(was_m))) {
					// A surface just uncovered takes its first frames at full weight, then settles to a long average.
					float frames = min(was_m.z + 1.0, 32.0);
					float alpha = max(1.0 / frames, 0.05);
					result = mix(was_light, now, alpha);
					m = vec3(mix(was_m.xy, vec2(l, l * l), alpha), frames);
				}
			}
		}
	}
	// Variance of the accumulated brightness; a young history hasn't seen enough frames to tell, so it's taken as noisy.
	float var = max(m.y - m.x * m.x, 0.0);
	if (m.z < 4.0) {
		var = max(var, 0.25 * m.x * m.x + 0.01);
	}
	imageStore(accum, px, result);
	imageStore(accum_depth, px, vec4(-p.z));
	imageStore(moments, px, vec4(m, 0.0));
	imageStore(variance, px, vec4(var));
}
)",
	// PASS_ATROUS: one pass of an edge-aware a-trous filter (5x5 taps, spaced by the pass's step): neighbours on the same
	// surface (the same plane, facing the same way) are averaged, more where the texel is still noisy, and the variance
	// is carried to the next pass.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@COMMON@
layout(push_constant, std430) uniform Params {
	ivec4 step; // x: the tap spacing
}
pc;
layout(set = 0, binding = 0) uniform sampler2D light_in;
layout(set = 0, binding = 1) uniform sampler2D variance_in;
layout(set = 0, binding = 2) uniform sampler2D accum_depth_tex;
layout(set = 0, binding = 3) uniform sampler2D normal_tex;
layout(rgba16f, set = 0, binding = 4) uniform restrict writeonly image2D light_out;
layout(r32f, set = 0, binding = 5) uniform restrict writeonly image2D variance_out;

float luminance(vec3 c) {
	return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, f.sizes.zw))) {
		return;
	}
	vec4 c = texelFetch(light_in, px, 0);
	float v = texelFetch(variance_in, px, 0).r;
	float z = texelFetch(accum_depth_tex, px, 0).r;
	if (z <= 0.0) {
		imageStore(light_out, px, c);
		imageStore(variance_out, px, vec4(v));
		return;
	}
	vec2 fpc = vec2(frame_pixel(px, f.sizes.zw));
	vec3 pos = view_from_linear(fpc, z, f.proj_info);
	vec3 n = normalize(texelFetch(normal_tex, ivec2(fpc), 0).xyz * 2.0 - 1.0);
	float lc = luminance(c.rgb);
	float sigma = 4.0 * sqrt(v) + 1e-3;
	const float k[3] = float[](0.375, 0.25, 0.0625);
	vec4 sum = vec4(0.0);
	float weights = 0.0;
	float var_sum = 0.0;
	for (int y = -2; y <= 2; y++) {
		for (int x = -2; x <= 2; x++) {
			ivec2 q = px + ivec2(x, y) * pc.step.x;
			if (any(lessThan(q, ivec2(0))) || any(greaterThanEqual(q, f.sizes.zw))) {
				continue;
			}
			float zq = texelFetch(accum_depth_tex, q, 0).r;
			if (zq <= 0.0) {
				continue;
			}
			vec2 fq = vec2(frame_pixel(q, f.sizes.zw));
			vec3 nq = normalize(texelFetch(normal_tex, ivec2(fq), 0).xyz * 2.0 - 1.0);
			vec4 cq = texelFetch(light_in, q, 0);
			// Same plane: the neighbour's distance from this texel's tangent plane, against a tolerance growing with depth.
			float plane = abs(dot(n, view_from_linear(fq, zq, f.proj_info) - pos));
			float w = k[abs(x)] * k[abs(y)];
			w *= exp(-plane / (0.02 + 0.01 * z));
			w *= pow(max(dot(n, nq), 0.0), 32.0);
			w *= exp(-abs(luminance(cq.rgb) - lc) / sigma);
			sum += cq * w;
			weights += w;
			var_sum += texelFetch(variance_in, q, 0).r * w * w;
		}
	}
	vec4 result = weights > 1e-6 ? sum / weights : c;
	float var = weights > 1e-6 ? var_sum / (weights * weights) : v;
	imageStore(light_out, px, result);
	imageStore(variance_out, px, vec4(var));
}
)",
	// PASS_REFLECT: one reflection ray per texel at half size, the view mirrored about a normal drawn from the surface's
	// roughness (GGX), so a mirror's rays stay together and a rough surface's spread. Marched against the depth buffer,
	// then (traced mode) the collision shapes; stores the light found and whether anything was. Surfaces rougher than
	// the cutoff, and rays that find nothing, are left to Godot's own reflection (probes, sky).
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@COMMON@
layout(push_constant, std430) uniform Params {
	ivec4 size;
}
pc;
layout(set = 0, binding = 0) uniform sampler2D depth_tex;
layout(set = 0, binding = 1) uniform sampler2D normal_tex;
layout(set = 0, binding = 2) uniform sampler2D color_tex;
layout(rgba16f, set = 0, binding = 3) uniform restrict writeonly image2D dst;
layout(set = 0, binding = 12) uniform sampler2DArray probe_atlas;

#define PROBE_SOURCES
bool probe_source(vec3 x, vec3 n, int hit, out vec3 seen);
// Whether this texel's ray may take its colour from a probe's capture: always on mirror-like surfaces, never past
// a roughness of 0.25 (a blurred reflection doesn't show the texture, and the line-of-sight ray costs), picked at
// random in between so the change is a blend rather than an edge.
bool probe_texture = false;
// Whether this texel's off-screen hits are lit at the point (a mirror's), or take the radiance cache's light (glossy).
bool mirror_hit = false;

@WORLD@

// The light a reflection probe's capture shows at world point x (facing n) on shape hit: from the first probe whose
// box holds x, when the shape doesn't move (the capture may be older than the frame), x faces the capture point and
// no collision shape stands between them (the capture holds only the nearest thing in each direction).
bool probe_source(vec3 x, vec3 n, int hit, out vec3 seen) {
	seen = vec3(0.0);
	if (prims.p[hit].emission.w < 0.5) {
		return false;
	}
	for (int i = 0; i < f.srcs.x; i++) {
		vec3 l = vec3(dot(f.src_x[i].xyz, x) + f.src_x[i].w, dot(f.src_y[i].xyz, x) + f.src_y[i].w, dot(f.src_z[i].xyz, x) + f.src_z[i].w);
		if (any(greaterThan(abs(l), f.src_box[i].xyz))) {
			continue;
		}
		vec3 to = f.src_capture[i].xyz - x;
		float dist = length(to);
		if (dist < 0.05 || dot(n, to) <= 0.0 || blocked(x + n * 0.02, to / dist, dist - 0.05)) {
			continue;
		}
		// Godot's octahedral map of the capture, in the probe's own axes, inside the atlas border.
		vec3 d = l - f.src_offset[i].xyz;
		d /= abs(d.x) + abs(d.y) + abs(d.z);
		vec2 uv = d.z >= 0.0 ? d.xy : sky_oct_wrap(d.xy);
		uv = (uv * 0.5 + 0.5) * f.src_border.y + f.src_border.x;
		seen = textureLod(probe_atlas, vec3(uv, floor(f.src_box[i].w * 0.5)), 0.0).rgb * f.src_offset[i].w;
		return true;
	}
	return false;
}

float ign(vec2 p) {
	return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715))));
}

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, pc.size.xy))) {
		return;
	}
	ivec2 fp = frame_pixel(px, pc.size.xy);
	float depth = texelFetch(depth_tex, fp, 0).r;
	vec4 nr = texelFetch(normal_tex, fp, 0);
	float rough = decode_roughness(nr.w);
	if (depth <= 0.0 || int(f.reflect.x) == 0 || rough > f.reflect.y) {
		imageStore(dst, px, vec4(0.0));
		return;
	}
	vec3 n = normalize(nr.xyz * 2.0 - 1.0);
	vec3 p = view_from_pixel(vec2(fp), depth, f.proj_info, f.clip.x, f.clip.y);
	if (probe_overrides((f.view_to_world * vec4(p, 1.0)).xyz, 2)) {
		imageStore(dst, px, vec4(0.0));
		return;
	}
	vec3 v = normalize(p);

	// A microfacet normal from the GGX distribution, low-discrepancy over frames.
	vec2 xi = vec2(fract(ign(vec2(px)) + f.misc.x * 0.7548776662), fract(ign(vec2(px) + vec2(47.0, 17.0)) + f.misc.x * 0.5698402910));
	float a2 = rough * rough * rough * rough;
	float cos_t = sqrt((1.0 - xi.x) / (1.0 + (a2 - 1.0) * xi.x));
	float sin_t = sqrt(max(0.0, 1.0 - cos_t * cos_t));
	float phi = 6.2831853 * xi.y;
	vec3 t = normalize(cross(n, abs(n.y) < 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0)));
	vec3 b = cross(n, t);
	vec3 h = normalize(t * (sin_t * cos(phi)) + b * (sin_t * sin(phi)) + n * cos_t);
	vec3 dir = reflect(v, h);
	if (dot(dir, n) <= 0.0) {
		dir = reflect(v, n);
	}

	vec3 origin = p + n * (0.02 + 0.002 * -p.z);
	const int STEPS = 32;
	float reach = 3.0 * f.misc.y;
	float jitter = fract(ign(vec2(px) + vec2(13.0, 101.0)) + f.misc.x * 0.6180339887);
	probe_texture = fract(ign(vec2(px) + vec2(71.0, 29.0)) + f.misc.x * 0.3819660113) < 1.0 - smoothstep(0.15, 0.25, rough);
	mirror_hit = rough < 0.06;
	vec3 light = vec3(0.0);
	bool found = false;
	float last = 0.0;
	for (int s = 0; s < STEPS; s++) {
		float along = reach * pow((float(s) + jitter) / float(STEPS), 2.0);
		float in_front = last;
		last = along;
		float stride = reach * 2.0 * (float(s) + jitter + 1.0) / float(STEPS * STEPS);
		vec3 q = origin + dir * along;
		if (q.z > -f.clip.x) {
			break;
		}
		vec2 qp = pixel_from_view(q, f.proj_info);
		if (!on_screen(qp)) {
			break;
		}
		float sd = texelFetch(depth_tex, ivec2(qp), 0).r;
		if (sd <= 0.0) {
			continue;
		}
		float surface = linear_depth(sd, f.clip.x, f.clip.y);
		float behind = -q.z - surface;
		if (behind > 0.0 && behind < max(0.1 + 0.02 * surface, 1.5 * stride)) {
			vec2 hp = clamp(pixel_from_view(origin + dir * refine_hit(depth_tex, origin, dir, in_front, along), f.proj_info), vec2(0.0), vec2(f.sizes.xy) - 1.0);
			float hd = texelFetch(depth_tex, ivec2(hp), 0).r;
			float pd;
			vec2 pp = prev_pixel(view_from_pixel(floor(hp), hd > 0.0 ? hd : sd, f.proj_info, f.clip.x, f.clip.y), pd);
			if (on_screen(pp)) {
				light = texture(color_tex, (pp + 0.5) / vec2(f.sizes.xy)).rgb;
				found = true;
			}
			break;
		}
	}
	if (!found && int(f.reflect.x) >= 2 && f.counts.x > 0) {
		vec3 world_origin = (f.view_to_world * vec4(origin, 1.0)).xyz;
		vec3 world_dir = mat3(f.view_to_world) * dir;
		float far = 60.0;
		vec3 wn;
		int hit = trace_world(world_origin, world_dir, far, wn);
		if (hit >= 0) {
			light = world_light(world_origin + world_dir * far, wn, hit, depth_tex, color_tex, true, true, true);
			found = true;
		}
	}
	vec4 result = found ? vec4(light, 1.0) : vec4(0.0);
	imageStore(dst, px, any(isnan(result)) || any(isinf(result)) ? vec4(0.0) : result);
}
)",
	// PASS_REFLECT_TEMPORAL: the reflection rays blended into what the same surface accumulated before (reprojected by
	// the surface, as the diffuse light is): a mirror keeps only a short history, so its reflection follows the camera,
	// a rough surface a longer one.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@COMMON@
layout(push_constant, std430) uniform Params {
	ivec4 size; // xy: the reflection texture, z: there is history
}
pc;
layout(set = 0, binding = 0) uniform sampler2D trace_tex;
layout(set = 0, binding = 1) uniform sampler2D prev_accum;
layout(set = 0, binding = 2) uniform sampler2D prev_depth_tex;
layout(set = 0, binding = 3) uniform sampler2D depth_tex;
layout(set = 0, binding = 4) uniform sampler2D normal_tex;
layout(rgba16f, set = 0, binding = 5) uniform restrict writeonly image2D accum;
layout(r32f, set = 0, binding = 6) uniform restrict writeonly image2D accum_depth;

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, pc.size.xy))) {
		return;
	}
	ivec2 fp = frame_pixel(px, pc.size.xy);
	float depth = texelFetch(depth_tex, fp, 0).r;
	if (depth <= 0.0) {
		imageStore(accum, px, vec4(0.0));
		imageStore(accum_depth, px, vec4(0.0));
		return;
	}
	vec3 p = view_from_pixel(vec2(fp), depth, f.proj_info, f.clip.x, f.clip.y);
	float rough = clamp(decode_roughness(texelFetch(normal_tex, fp, 0).w), 0.0, 1.0);
	vec4 now = texelFetch(trace_tex, px, 0);
	vec4 result = now;
	if (pc.size.z != 0) {
		float expected;
		vec2 pp = prev_pixel(p, expected);
		if (on_screen(pp)) {
			vec2 uv = (pp + 0.5) / vec2(f.sizes.xy);
			float was = texture(prev_depth_tex, uv).r;
			vec4 was_light = texture(prev_accum, uv);
			if (abs(was - expected) < 0.05 * expected && !any(isnan(was_light))) {
				// Blended with each side weighted down by its brightness (as tone mapped), so a lone bright ray (a
				// light, a glint) doesn't flare the accumulated reflection for frames.
				float take = mix(0.45, 0.06, min(sqrt(rough * 2.5), 1.0));
				float w_was = (1.0 - take) / (1.0 + dot(was_light.rgb, vec3(0.2126, 0.7152, 0.0722)));
				float w_now = take / (1.0 + dot(now.rgb, vec3(0.2126, 0.7152, 0.0722)));
				result = (was_light * w_was + now * w_now) / (w_was + w_now);
			}
		}
	}
	imageStore(accum, px, result);
	imageStore(accum_depth, px, vec4(-p.z));
}
)",
	// PASS_CACHE: every cell in use takes one more ray, cosine-weighted about its normal: the light arriving from what
	// it hits (that point on last frame's image, or its own cell: so bounce light builds up over frames) joins the
	// cell's running mean, and the light leaving the cell is its albedo times that plus its direct light. A cell no
	// pixel's ray has used for a while is freed.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@COMMON@
layout(set = 0, binding = 0) uniform sampler2D depth_tex;
layout(set = 0, binding = 2) uniform sampler2D color_tex;

@WORLD@

void main() {
	// A quarter of the slots a frame (each cell takes a ray every fourth frame: it feeds a running mean anyway).
	uint slot = (gl_GlobalInvocationID.y * 512u + gl_GlobalInvocationID.x) * 4u + (f.cache.x & 3u);
	if (slot > f.cache.y || cache_keys.k[slot] == 0u) {
		return;
	}
	Cell cell = cells.c[slot];
	if (f.cache.x - floatBitsToUint(cell.pos.w) > 600u) {
		cache_keys.k[slot] = 0u;
		return;
	}
	vec3 n = normalize(cell.normal.xyz);
	vec3 t = normalize(cross(n, abs(n.y) < 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0)));
	vec3 b = cross(n, t);
	float jitter = float(hash_u(slot) & 0xffffu) / 65536.0;
	float u = fract(jitter + float(f.cache.x) * 0.7548776662);
	float phi = 6.2831853 * fract(float(hash_u(slot ^ 0x5bd1e995u) & 0xffffu) / 65536.0 + float(f.cache.x) * 0.5698402910);
	vec3 dir = normalize(t * (sqrt(u) * cos(phi)) + b * (sqrt(u) * sin(phi)) + n * sqrt(max(0.0, 1.0 - u)));

	vec3 origin = cell.pos.xyz + n * 0.03;
	vec3 arriving = vec3(0.0);
	float far = f.ambient.w;
	vec3 wn;
	int emissives = int(f.reflect.z);
	int hit = trace_world(origin, dir, far, wn);
	if (hit >= 0) {
		arriving = clamp_firefly(world_light(origin + dir * far, wn, hit, depth_tex, color_tex, false, emissives == 0, false));
	} else if (escapes(origin, dir, f.ambient.w)) {
		arriving = background_light(dir);
	}
	// The emissives, sampled directly as the texels' rays sample them: the cell's own ray would find a small bright lamp
	// only now and then, and each find would light the whole cell for a while (a flicker).
	if (emissives > 0) {
		uint h = hash_u(slot ^ 0x27d4eb2du);
		vec2 xi = vec2(fract(float(h & 0xffffu) / 65536.0 + float(f.cache.x) * 0.7548776662), fract(float(h >> 16u) / 65536.0 + float(f.cache.x) * 0.5698402910));
		float pick = fract(float(hash_u(slot ^ 0x165667b1u) & 0xffffu) / 65536.0 + float(f.cache.x) * 0.6180339887);
		arriving += sample_emissive(cell.pos.xyz, n, xi, pick, emissives);
	}
	// A cell averages its last 32 samples; while the light sources have just changed (a lamp switched off, an emitter
	// gone) only its last 4, so the old light doesn't linger for seconds.
	float samples = min(cell.normal.w + 1.0, f.reflect.w > 0.0 ? 4.0 : 32.0);
	vec3 incoming = mix(cell.incoming.rgb, arriving, 1.0 / samples);
	if (any(isnan(incoming))) {
		incoming = vec3(0.0);
	}
	cells.c[slot].incoming = vec4(incoming, 0.0);
	cells.c[slot].normal.w = samples;
	cells.c[slot].outgoing = vec4(cell.albedo.rgb * (direct_light(cell.pos.xyz, n) + incoming), 0.0);
}
)",
	// PASS_RESOLVE: up to Godot's GI buffer size, each pixel taking the accumulated texels around it that lie on the
	// same surface (close in depth, facing the same way). Written as Godot's ambient buffer: rgb the light the hits
	// saw, alpha the fraction that hit, which the surface shader mixes over its own ambient light, then multiplies by
	// the albedo. The reflection buffer is left to Godot (alpha 0).
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@COMMON@
layout(push_constant, std430) uniform Params {
	ivec4 out_size;
}
pc;
layout(set = 0, binding = 0) uniform sampler2D accum_tex;
layout(set = 0, binding = 1) uniform sampler2D accum_depth_tex;
layout(set = 0, binding = 2) uniform sampler2D depth_tex;
layout(set = 0, binding = 3) uniform sampler2D normal_tex;
layout(set = 0, binding = 6) uniform sampler2D color_tex;
layout(rgba16f, set = 0, binding = 4) uniform restrict writeonly image2D ambient_buffer;
layout(rgba16f, set = 0, binding = 5) uniform restrict writeonly image2D reflection_buffer;
layout(set = 0, binding = 7) uniform sampler2D reflect_tex;
layout(set = 0, binding = 8) uniform sampler2D reflect_depth_tex;
layout(set = 0, binding = 12) uniform sampler2DArray probe_atlas;

// Traced + probes: the reflection a probe gives world point w along r at this roughness, as Godot's own probe
// reflection would (the smallest box holding w, box projected if the probe is, the capture's level for the roughness),
// and how much of it applies there (r_weight: 0 outside every box, fading to the edge over its blend distance).
vec3 probe_reflection(vec3 w, vec3 r, float rough, out float r_weight) {
	r_weight = 0.0;
	int best = -1;
	float best_volume = 1e30;
	vec3 l = vec3(0.0);
	for (int i = 0; i < f.srcs.x; i++) {
		vec3 li = vec3(dot(f.src_x[i].xyz, w) + f.src_x[i].w, dot(f.src_y[i].xyz, w) + f.src_y[i].w, dot(f.src_z[i].xyz, w) + f.src_z[i].w);
		vec3 e = f.src_box[i].xyz;
		float volume = e.x * e.y * e.z;
		if (all(lessThanEqual(abs(li), e)) && volume < best_volume) {
			best = i;
			best_volume = volume;
			l = li;
		}
	}
	if (best < 0) {
		return vec3(0.0);
	}
	vec3 e = f.src_box[best].xyz;
	r_weight = 1.0;
	float blend = f.src_capture[best].w;
	if (blend > 0.0) {
		vec3 axis = min(vec3(blend), e);
		vec3 b = clamp(1.0 - (abs(l) - e + axis) / axis, 0.0, 1.0);
		r_weight = pow(b.x * b.y * b.z, 2.0);
	}
	vec3 d = normalize(vec3(dot(f.src_x[best].xyz, r), dot(f.src_y[best].xyz, r), dot(f.src_z[best].xyz, r)));
	if (mod(f.src_box[best].w, 2.0) > 0.5) {
		vec3 to_max = (e - l) / d;
		vec3 to_min = (-e - l) / d;
		vec3 far_side = mix(to_min, to_max, greaterThan(d, vec3(0.0)));
		d = l + d * min(min(far_side.x, far_side.y), far_side.z) - f.src_offset[best].xyz;
	}
	d /= abs(d.x) + abs(d.y) + abs(d.z);
	vec2 uv = d.z >= 0.0 ? d.xy : (1.0 - abs(d.yx)) * vec2(d.x >= 0.0 ? 1.0 : -1.0, d.y >= 0.0 ? 1.0 : -1.0);
	uv = (uv * 0.5 + 0.5) * f.src_border.y + f.src_border.x;
	return textureLod(probe_atlas, vec3(uv, floor(f.src_box[best].w * 0.5)), sqrt(rough) * f.src_border.z).rgb * f.src_offset[best].w;
}

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, pc.out_size.xy))) {
		return;
	}
	ivec2 fp = frame_pixel(px, pc.out_size.xy);
	float depth = texelFetch(depth_tex, fp, 0).r;
	if (depth <= 0.0) {
		imageStore(ambient_buffer, px, vec4(0.0));
		imageStore(reflection_buffer, px, vec4(0.0));
		return;
	}
	vec3 p = view_from_pixel(vec2(fp), depth, f.proj_info, f.clip.x, f.clip.y);
	vec4 nr = texelFetch(normal_tex, fp, 0);
	vec3 n = normalize(nr.xyz * 2.0 - 1.0);
	float d = -p.z;

	// Reflections: the half-size reflection texels on the same surface around this pixel, blurred more the rougher the
	// surface; alpha is how many of their rays found something, the rest left to Godot's reflection (probes, sky).
	vec4 reflection = vec4(0.0);
	float rough = clamp(decode_roughness(nr.w), 0.0, 1.0);
	// Only surfaces smooth enough to have taken reflection rays gather them.
	if (int(f.reflect.x) != 0 && rough <= f.reflect.y) {
		vec2 rc = (vec2(fp) + 0.5) * vec2(pc.out_size.zw) / vec2(f.sizes.xy) - 0.5;
		ivec2 rb = ivec2(floor(rc));
		float spread = 0.35 + 4.0 * sqrt(rough);
		vec4 rsum = vec4(0.0);
		float rweights = 0.0;
		for (int y = -1; y <= 2; y++) {
			for (int x = -1; x <= 2; x++) {
				ivec2 t = clamp(rb + ivec2(x, y), ivec2(0), pc.out_size.zw - 1);
				float tz = texelFetch(reflect_depth_tex, t, 0).r;
				if (tz <= 0.0) {
					continue;
				}
				vec3 tn = normalize(texelFetch(normal_tex, frame_pixel(t, pc.out_size.zw), 0).xyz * 2.0 - 1.0);
				vec2 o = vec2(t) - rc;
				float w = exp(-dot(o, o) / (2.0 * spread * spread)) * exp(-abs(tz - d) / (0.03 * d + 0.01)) * pow(max(dot(tn, n), 0.0), 32.0);
				// Weighted down by brightness too: a lone bright texel among dim ones is a glint, not the surface.
				vec4 rt = texelFetch(reflect_tex, t, 0);
				w /= 1.0 + dot(rt.rgb, vec3(0.2126, 0.7152, 0.0722));
				rsum += rt * w;
				rweights += w;
			}
		}
		if (rweights > 1e-4) {
			vec4 r = rsum / rweights;
			// Faded out over the last quarter of the cutoff, so neighbouring roughnesses on either side of it don't
			// switch between two different reflections.
			float fade = smoothstep(0.0, 1.0, (f.reflect.y - rough) / max(0.25 * f.reflect.y, 1e-3));
			reflection = vec4(r.a > 1e-3 ? r.rgb / r.a : vec3(0.0), r.a * fade);
		}
	}
	// Traced + probes: inside a probe's box, what the rays leave (misses, the cutoff's fade, rougher surfaces) is the
	// probe's reflection rather than Godot's sky, under the traced light. Not where a probe Godot draws replaces it all.
	if (f.srcs.x > 0) {
		vec3 w = (f.view_to_world * vec4(p, 1.0)).xyz;
		if (!probe_overrides(w, 2)) {
			vec3 nw = normalize(mat3(f.view_to_world) * n);
			vec3 r = normalize(mix(reflect(normalize(w - f.view_to_world[3].xyz), nw), nw, rough * rough));
			float weight;
			vec3 from_probe = probe_reflection(w, r, rough, weight);
			if (weight > 0.0) {
				float a = reflection.a;
				float alpha = a + (1.0 - a) * weight;
				reflection = vec4((reflection.rgb * a + from_probe * (1.0 - a) * weight) / alpha, alpha);
			}
		}
	}
	imageStore(reflection_buffer, px, reflection);

	int debug = int(f.misc.w);
	if (debug == 1) {
		float pd;
		vec2 pp = prev_pixel(p, pd);
		imageStore(ambient_buffer, px, vec4(texture(color_tex, (pp + 0.5) / vec2(f.sizes.xy)).rgb, 1.0));
		return;
	}
	if (debug == 5) {
		// World position, one-metre bands: the floor at y = 0 should show a clean band edge at its surface.
		vec3 w = (f.view_to_world * vec4(p, 1.0)).xyz;
		imageStore(ambient_buffer, px, vec4(fract(w), 1.0));
		return;
	}
	if (debug == 2) {
		float err = length(pixel_from_view(p, f.proj_info) - vec2(fp));
		imageStore(ambient_buffer, px, vec4(clamp(err, 0.0, 1.0), clamp(1.0 - err, 0.0, 1.0), 0.0, 1.0));
		return;
	}

	vec2 tc = (vec2(fp) + 0.5) * vec2(f.sizes.zw) / vec2(f.sizes.xy) - 0.5;
	ivec2 base = ivec2(floor(tc));
	vec4 sum = vec4(0.0);
	float weights = 0.0;
	for (int y = -1; y <= 2; y++) {
		for (int x = -1; x <= 2; x++) {
			ivec2 t = clamp(base + ivec2(x, y), ivec2(0), f.sizes.zw - 1);
			float tz = texelFetch(accum_depth_tex, t, 0).r;
			if (tz <= 0.0) {
				continue;
			}
			vec3 tn = normalize(texelFetch(normal_tex, frame_pixel(t, f.sizes.zw), 0).xyz * 2.0 - 1.0);
			vec2 o = vec2(t) - tc;
			float w = exp(-dot(o, o) * 0.5) * exp(-abs(tz - d) / (0.03 * d + 0.01)) * pow(max(dot(tn, n), 0.0), 8.0);
			sum += texelFetch(accum_tex, t, 0) * w;
			weights += w;
		}
	}
	if (weights <= 1e-4) {
		// A sliver of a surface (a face seen edge on) that no texel shares: the texels around at a similar depth,
		// whichever way they face, rather than none (which left the sliver black).
		for (int y = -1; y <= 2; y++) {
			for (int x = -1; x <= 2; x++) {
				ivec2 t = clamp(base + ivec2(x, y), ivec2(0), f.sizes.zw - 1);
				float tz = texelFetch(accum_depth_tex, t, 0).r;
				if (tz <= 0.0) {
					continue;
				}
				vec2 o = vec2(t) - tc;
				float w = exp(-dot(o, o) * 0.5) * exp(-abs(tz - d) / (0.1 * d + 0.05));
				sum += texelFetch(accum_tex, t, 0) * w;
				weights += w;
			}
		}
	}
	vec4 gi = weights > 1e-4 ? sum / weights : vec4(0.0);
	if (debug == 4) {
		imageStore(ambient_buffer, px, vec4(vec3(gi.a), 1.0));
		return;
	}
	// rgb is premultiplied by the fraction of rays that hit (alpha), so the surface shader's mix with its own ambient
	// light gives ambient * (1 - alpha) + rgb; light from the emissive samples alone (no ray hit) keeps a tiny alpha.
	float alpha = gi.a;
	if (alpha < 1e-3 && max(gi.r, max(gi.g, gi.b)) > 0.0) {
		alpha = 1e-3;
	}
	vec3 light = alpha > 0.0 ? gi.rgb / alpha : vec3(0.0);
	imageStore(ambient_buffer, px, vec4(light * f.misc.z, alpha));
}
)",
	// PASS_REFLECT_FILTER: one pass of an edge-avoiding a-trous filter over the accumulated reflections (5x5 taps, spaced
	// by the pass's step): a glossy surface's one-ray-a-texel reflection averaged with its neighbours on the same surface
	// (the same plane, facing the same way, about as rough), over a reach growing with its roughness; a mirror's left as
	// it is. The history itself stays unfiltered: this only feeds the resolve.
	R"(
#[compute]
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
@COMMON@
layout(push_constant, std430) uniform Params {
	ivec4 step; // x: the tap spacing, zw: the reflection texture's size
}
pc;
layout(set = 0, binding = 0) uniform sampler2D light_in;
layout(set = 0, binding = 1) uniform sampler2D reflect_depth_tex;
layout(set = 0, binding = 2) uniform sampler2D normal_tex;
layout(rgba16f, set = 0, binding = 3) uniform restrict writeonly image2D light_out;

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	ivec2 size = pc.step.zw;
	if (any(greaterThanEqual(px, size))) {
		return;
	}
	vec4 c = texelFetch(light_in, px, 0);
	float z = texelFetch(reflect_depth_tex, px, 0).r;
	vec2 fpc = vec2(frame_pixel(px, size));
	vec4 nr = texelFetch(normal_tex, ivec2(fpc), 0);
	float rough = clamp(decode_roughness(nr.w), 0.0, 1.0);
	// The reach in texels a roughness spreads a reflection over: none for a mirror.
	float reach = 16.0 * rough;
	// Nothing to do for a mirror, nor past the roughness cutoff (no traced reflection there).
	if (z <= 0.0 || reach < 0.3 || rough > f.reflect.y) {
		imageStore(light_out, px, c);
		return;
	}
	vec3 pos = view_from_linear(fpc, z, f.proj_info);
	vec3 n = normalize(nr.xyz * 2.0 - 1.0);
	const float k[3] = float[](0.375, 0.25, 0.0625);
	vec4 sum = vec4(0.0);
	float weights = 0.0;
	for (int y = -2; y <= 2; y++) {
		for (int x = -2; x <= 2; x++) {
			ivec2 q = px + ivec2(x, y) * pc.step.x;
			if (any(lessThan(q, ivec2(0))) || any(greaterThanEqual(q, size))) {
				continue;
			}
			float zq = texelFetch(reflect_depth_tex, q, 0).r;
			if (zq <= 0.0) {
				continue;
			}
			vec2 fq = vec2(frame_pixel(q, size));
			vec4 nrq = texelFetch(normal_tex, ivec2(fq), 0);
			vec3 nq = normalize(nrq.xyz * 2.0 - 1.0);
			float rq = clamp(decode_roughness(nrq.w), 0.0, 1.0);
			float d2 = float(x * x + y * y) * float(pc.step.x * pc.step.x);
			float plane = abs(dot(n, view_from_linear(fq, zq, f.proj_info) - pos));
			float w = k[abs(x)] * k[abs(y)];
			w *= exp(-d2 / (2.0 * reach * reach));
			w *= exp(-plane / (0.02 + 0.01 * z));
			// Facing about the same way: a normal map's bumps are one surface (their reflections vary, and that's
			// what averages out), a corner isn't.
			w *= pow(max(dot(n, nq), 0.0), 8.0);
			w *= exp(-abs(rq - rough) / 0.05);
			sum += texelFetch(light_in, q, 0) * w;
			weights += w;
		}
	}
	imageStore(light_out, px, weights > 1e-6 ? sum / weights : c);
}
)",
};

const char *PASS_NAMES[8] = { "trace", "temporal", "atrous", "reflect", "reflect temporal", "cache", "resolve", "reflect filter" };

struct FrameData {
	float view_to_world[16];
	float world_to_view[16];
	float world_to_prev_view[16];
	float proj_info[4];
	float prev_proj_info[4];
	float clip[4];
	int32_t sizes[4];
	float misc[4];
	float sun_direction[4];
	float sun_color[4];
	float ambient[4];
	int32_t counts[4];
	float omni_position[8][4];
	float omni_color[8][4];
	float spot_position[8][4];
	float spot_direction[8][4];
	float spot_color[8][4];
	float spot_cone[8][4];
	float reflect[4];
	float emit_center[64][4];
	float emit_x[64][4];
	float emit_y[64][4];
	float emit_z[64][4];
	float emit_color[64][4];
	float emit_cdf[16][4];
	uint32_t cache[4];
	float probe_x[8][4];
	float probe_y[8][4];
	float probe_z[8][4];
	float probe_core[8][4];
	int32_t probes[4];
	float sky[4];
	float sky_x[4];
	float sky_y[4];
	float sky_z[4];
	float sky_flat[4];
	float src_x[8][4];
	float src_y[8][4];
	float src_z[8][4];
	float src_box[8][4];
	float src_offset[8][4];
	float src_capture[8][4];
	int32_t srcs[4];
	float src_border[4];
};

void projection_info(const Projection &p_projection, const Size2i &p_size, float *r_info) {
	r_info[0] = -2.0f / (p_size.x * p_projection.columns[0][0]);
	r_info[1] = -2.0f / (p_size.y * p_projection.columns[1][1]);
	r_info[2] = (1.0f - p_projection.columns[2][0]) / p_projection.columns[0][0];
	r_info[3] = (1.0f + p_projection.columns[2][1]) / p_projection.columns[1][1];
}

} // namespace

void PhysXGIEffect::set_settings(const ObjectID &p_node, const Snapshot &p_snapshot) {
	MutexLock lock(mutex);
	settings[p_node] = p_snapshot;
}

void PhysXGIEffect::remove_settings(const ObjectID &p_node) {
	MutexLock lock(mutex);
	settings.erase(p_node);
}

void PhysXGIEffect::attach_world(World3D *p_world) {
	ERR_FAIL_NULL(p_world);
	worlds[p_world->get_instance_id()]++;
}

void PhysXGIEffect::detach_world(const ObjectID &p_world) {
	int *count = worlds.getptr(p_world);
	if (count == nullptr || --(*count) > 0) {
		return;
	}
	worlds.erase(p_world);
	PhysXCompositorEffects::uninstall(p_world, get_shared());
}

void PhysXGIEffect::install(World3D *p_world, Camera3D *p_camera) {
	ERR_FAIL_NULL(p_world);
	ERR_FAIL_COND(!worlds.has(p_world->get_instance_id()));
	PhysXCompositorEffects::install(p_world, p_camera, get_shared(), PhysXCompositorEffects::ORDER_GI);
}

Ref<PhysXGIEffect> PhysXGIEffect::get_shared() {
	if (shared_effect.is_null()) {
		shared_effect.instantiate();
	}
	return shared_effect;
}

void PhysXGIEffect::free_shared() {
	shared_effect.unref();
}

PhysXGIEffect::PhysXGIEffect() {
	set_effect_callback_type(EFFECT_CALLBACK_TYPE_PRE_OPAQUE);
	set_needs_normal_roughness(true);
	RenderingServer::get_singleton()->compositor_effect_set_callback(get_rid(), RSE::COMPOSITOR_EFFECT_CALLBACK_TYPE_PRE_OPAQUE, callable_mp(this, &PhysXGIEffect::_render));
}

PhysXGIEffect::~PhysXGIEffect() {
	// Freed on the render thread, wherever the last reference goes.
	godot_physx_run_on_render_thread([this]() {
		RenderingDevice *rd = RenderingServer::get_singleton() ? RenderingServer::get_singleton()->get_rendering_device() : nullptr;
		if (rd == nullptr) {
			return;
		}
		for (int i = 0; i < PASS_MAX; i++) {
			for (RID r : { pipelines[i], shaders[i], mesh_walk_pipelines[i], mesh_walk_shaders[i] }) {
				if (r.is_valid()) {
					rd->free_rid(r);
				}
			}
		}
		for (RID r : { linear_sampler, mip_sampler, nearest_sampler, data_buffer, primitive_buffer, node_buffer, cache_keys, cache_cells, geometry_buffer }) {
			if (r.is_valid()) {
				rd->free_rid(r);
			}
		}
	});
}

bool PhysXGIEffect::_ensure(RenderingDevice *p_rd) {
	if (shaders[PASS_RESOLVE].is_valid()) {
		return true;
	}
	if (failed) {
		return false;
	}
	failed = true;
	for (int i = 0; i < PASS_MAX; i++) {
		// The tracing passes twice: walking the shapes alone, and the shapes and the meshes in one loop.
		const bool traces = i == PASS_TRACE || i == PASS_REFLECT || i == PASS_CACHE;
		for (int variant = 0; variant < (traces ? 2 : 1); variant++) {
			Ref<RDShaderFile> file;
			file.instantiate();
			// The sky's radiance map is an array (a layer per roughness) or one texture (a mip per roughness), as the
			// project sets it for Godot's own shaders.
			String defines = RendererSceneRenderRD::get_singleton()->is_using_radiance_octmap_array() ? "#define SKY_OCTMAP_ARRAY\n" : "";
			if (variant == 1) {
				defines += "#define MESH_WALK\n";
			}
			const String source = String(SHADERS[i]).replace("@WORLD@", WORLD).replace("@COMMON@", defines + COMMON);
			ERR_FAIL_COND_V_MSG(file->parse_versions_from_text(source) != OK, false, vformat("PhysXGI: %s shader failed to parse.", PASS_NAMES[i]));
			Ref<RDShaderSPIRV> spirv = file->get_spirv();
			ERR_FAIL_COND_V(spirv.is_null(), false);
			const String err = spirv->get_stage_compile_error(RD::SHADER_STAGE_COMPUTE);
			ERR_FAIL_COND_V_MSG(!err.is_empty(), false, vformat("PhysXGI: %s shader: %s", PASS_NAMES[i], err));
			RID &shader = variant == 1 ? mesh_walk_shaders[i] : shaders[i];
			RID &pipeline = variant == 1 ? mesh_walk_pipelines[i] : pipelines[i];
			shader = p_rd->shader_create_from_spirv(file->get_spirv_stages());
			ERR_FAIL_COND_V(!shader.is_valid(), false);
			pipeline = p_rd->compute_pipeline_create(shader);
			ERR_FAIL_COND_V(!pipeline.is_valid(), false);
		}
	}
	RD::SamplerState state;
	state.mag_filter = RD::SAMPLER_FILTER_LINEAR;
	state.min_filter = RD::SAMPLER_FILTER_LINEAR;
	state.repeat_u = RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE;
	state.repeat_v = RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE;
	linear_sampler = p_rd->sampler_create(state);
	state.mip_filter = RD::SAMPLER_FILTER_LINEAR; // a probe's capture, between its roughness levels
	mip_sampler = p_rd->sampler_create(state);
	state.mip_filter = RD::SAMPLER_FILTER_NEAREST;
	state.mag_filter = RD::SAMPLER_FILTER_NEAREST;
	state.min_filter = RD::SAMPLER_FILTER_NEAREST;
	nearest_sampler = p_rd->sampler_create(state);
	data_buffer = p_rd->uniform_buffer_create(sizeof(FrameData));
	{
		Vector<uint8_t> zeros;
		zeros.resize(CACHE_SLOTS * sizeof(uint32_t));
		zeros.fill(0);
		cache_keys = p_rd->storage_buffer_create(zeros.size(), zeros);
		cache_cells = p_rd->storage_buffer_create(CACHE_SLOTS * 5 * 4 * sizeof(float));
	}
	failed = false;
	return true;
}

void PhysXGIView::free_data() {
	RenderingDevice *rd = RenderingDevice::get_singleton();
	if (rd == nullptr) {
		return;
	}
	for (RID *r : { &trace, &accum[0], &accum[1], &accum_depth[0], &accum_depth[1], &moments[0], &moments[1], &variance, &filtered[0], &filtered[1], &filtered_variance[0], &filtered_variance[1], &reflect_trace, &reflect_accum[0], &reflect_accum[1], &reflect_depth[0], &reflect_depth[1], &reflect_filtered[0], &reflect_filtered[1] }) {
		if (r->is_valid()) {
			rd->free_rid(*r);
			*r = RID();
		}
	}
	targets_for = Size2i();
	have_history = false;
}

void PhysXGIView::ensure(RenderingDevice *p_rd, const Size2i &p_frame) {
	if (p_frame == targets_for && trace.is_valid()) {
		return;
	}
	free_data();
	targets_for = p_frame;
	trace_size = Size2i(MAX(p_frame.x / 4, 1), MAX(p_frame.y / 4, 1));
	RD::TextureFormat tf;
	tf.format = RD::DATA_FORMAT_R16G16B16A16_SFLOAT;
	tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT;
	tf.width = trace_size.x;
	tf.height = trace_size.y;
	trace = p_rd->texture_create(tf, RD::TextureView());
	for (int i = 0; i < 2; i++) {
		accum[i] = p_rd->texture_create(tf, RD::TextureView());
		moments[i] = p_rd->texture_create(tf, RD::TextureView());
		filtered[i] = p_rd->texture_create(tf, RD::TextureView());
	}
	tf.format = RD::DATA_FORMAT_R32_SFLOAT;
	for (int i = 0; i < 2; i++) {
		accum_depth[i] = p_rd->texture_create(tf, RD::TextureView());
		filtered_variance[i] = p_rd->texture_create(tf, RD::TextureView());
	}
	variance = p_rd->texture_create(tf, RD::TextureView());

	reflect_size = Size2i(MAX(p_frame.x / 2, 1), MAX(p_frame.y / 2, 1));
	tf.width = reflect_size.x;
	tf.height = reflect_size.y;
	for (int i = 0; i < 2; i++) {
		reflect_depth[i] = p_rd->texture_create(tf, RD::TextureView());
	}
	tf.format = RD::DATA_FORMAT_R16G16B16A16_SFLOAT;
	reflect_trace = p_rd->texture_create(tf, RD::TextureView());
	for (int i = 0; i < 2; i++) {
		reflect_accum[i] = p_rd->texture_create(tf, RD::TextureView());
		reflect_filtered[i] = p_rd->texture_create(tf, RD::TextureView());
	}
}

void PhysXGIEffect::_dispatch(RenderingDevice *p_rd, int p_pass, const Vector<RD::Uniform> &p_uniforms, const Size2i &p_size, const void *p_push, uint32_t p_push_size) {
	Vector<RD::Uniform> uniforms = p_uniforms;
	uniforms.push_back(RD::Uniform(RD::UNIFORM_TYPE_UNIFORM_BUFFER, 9, data_buffer));
	const bool walk_meshes = mesh_walk && mesh_walk_pipelines[p_pass].is_valid();
	const RID set = p_rd->uniform_set_create(uniforms, walk_meshes ? mesh_walk_shaders[p_pass] : shaders[p_pass], 0);
	ERR_FAIL_COND(!set.is_valid());
	// GPU timestamps per pass (the visual profiler shows them).
	p_rd->capture_timestamp(String("PXTGI ") + PASS_NAMES[p_pass]);
	const RD::ComputeListID list = p_rd->compute_list_begin();
	p_rd->compute_list_bind_compute_pipeline(list, walk_meshes ? mesh_walk_pipelines[p_pass] : pipelines[p_pass]);
	p_rd->compute_list_bind_uniform_set(list, set, 0);
	if (p_push_size > 0) {
		p_rd->compute_list_set_push_constant(list, p_push, p_push_size);
	}
	p_rd->compute_list_dispatch(list, (p_size.x + 7) / 8, (p_size.y + 7) / 8, 1);
	p_rd->compute_list_end();
	p_rd->free_rid(set);
}

void PhysXGIEffect::_render(int p_callback_type, const RenderData *p_render_data) {
	Snapshot s;
	{
		MutexLock lock(mutex);
		if (settings.is_empty()) {
			return;
		}
		s = settings.begin()->value;
	}
	if (p_render_data == nullptr) {
		return;
	}
	RenderSceneBuffersRD *buffers = Object::cast_to<RenderSceneBuffersRD>(p_render_data->get_render_scene_buffers().ptr());
	const RenderSceneData *scene = p_render_data->get_render_scene_data();
	RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
	if (buffers == nullptr || scene == nullptr || rd == nullptr || !_ensure(rd)) {
		return;
	}
	// Godot's GI step makes the buffers on its first run: nothing to write into until then.
	static const StringName gi_scope = "rbgi", ambient_name = "ambient", reflection_name = "reflection";
	static const StringName clustered_scope = "forward_clustered", normal_name = "normal_roughness";
	if (!buffers->has_texture(gi_scope, ambient_name) || !buffers->has_texture(clustered_scope, normal_name)) {
		return;
	}
	const Size2i size = buffers->get_internal_size();
	const RID ambient = buffers->get_texture(gi_scope, ambient_name);
	const RID reflection = buffers->get_texture(gi_scope, reflection_name);
	const RID normal = buffers->get_texture(clustered_scope, normal_name);
	const RID depth = buffers->get_depth_texture();
	const RID color = buffers->get_internal_texture(); // still last frame's image: the opaque pass hasn't run yet
	const RD::TextureFormat out_format = rd->texture_get_format(ambient);
	const Size2i out_size(out_format.width, out_format.height);
	// This viewport's targets and history.
	static const StringName view_scope = "pxtgi";
	Ref<PhysXGIView> view = buffers->has_custom_data(view_scope) ? Ref<PhysXGIView>(buffers->get_custom_data(view_scope)) : Ref<PhysXGIView>();
	if (view.is_null()) {
		view.instantiate();
		buffers->set_custom_data(view_scope, view);
	}
	PhysXGIView &v = *view.ptr();
	v.ensure(rd, size);

	const Transform3D cam_transform = scene->get_cam_transform();
	// The raw projection, as Godot's own GI step uses it: get_cam_projection() already has the depth correction (and TAA
	// jitter) applied, which the depth linearisation below doesn't expect.
	const RenderSceneDataRD *scene_rd = Object::cast_to<RenderSceneDataRD>(scene);
	ERR_FAIL_NULL(scene_rd);
	const Projection projection = scene_rd->cam_projection;
	if (v.have_history && (cam_transform != v.prev_cam_transform || projection != v.prev_projection)) {
		unsettle();
	}
	if (!v.have_history) {
		v.prev_cam_transform = cam_transform;
		v.prev_projection = projection;
	}
	FrameData data = {};
	RendererRD::MaterialStorage::store_transform(cam_transform, data.view_to_world);
	RendererRD::MaterialStorage::store_transform(cam_transform.affine_inverse(), data.world_to_view);
	RendererRD::MaterialStorage::store_transform(v.prev_cam_transform.affine_inverse(), data.world_to_prev_view);
	projection_info(projection, size, data.proj_info);
	projection_info(v.prev_projection, size, data.prev_proj_info);
	data.clip[0] = projection.get_z_near();
	data.clip[1] = projection.get_z_far();
	data.clip[2] = v.prev_projection.get_z_near();
	data.clip[3] = v.prev_projection.get_z_far();
	data.sizes[0] = size.x;
	data.sizes[1] = size.y;
	data.sizes[2] = v.trace_size.x;
	data.sizes[3] = v.trace_size.y;
	data.misc[0] = (float)(frame++ % 4096);
	data.misc[1] = s.max_distance;
	data.misc[2] = s.intensity;
	data.misc[3] = (float)s.debug;
	data.sun_direction[0] = s.sun_direction.x;
	data.sun_direction[1] = s.sun_direction.y;
	data.sun_direction[2] = s.sun_direction.z;
	data.sun_direction[3] = s.sun_color.get_luminance() > 0.0f ? 1.0f : 0.0f;
	// The camera's exposure, which Godot scales its lights and emission by (as the frame the screen rays read is).
	const float exposure = p_render_data->get_camera_attributes().is_valid() ? RSG::camera_attributes->camera_attributes_get_exposure_normalization_factor(p_render_data->get_camera_attributes()) : 1.0f;
	data.sun_color[0] = s.sun_color.r * exposure;
	data.sun_color[1] = s.sun_color.g * exposure;
	data.sun_color[2] = s.sun_color.b * exposure;
	data.sun_color[3] = exposure;
	// The ambient light every surface patch gets (the environment's ambient colour, when that's its source), and what a
	// ray that leaves the scene sees: the sky, or the background colour (as Godot draws them, exposure included).
	RID sky_map;
	{
		RendererSceneRenderRD *scene_render = RendererSceneRenderRD::get_singleton();
		const RID env = p_render_data->get_environment();
		Color ambient_color(0, 0, 0);
		if (env.is_valid()) {
			if (scene_render->environment_get_ambient_source(env) == RSE::ENV_AMBIENT_SOURCE_COLOR) {
				ambient_color = scene_render->environment_get_ambient_light(env).srgb_to_linear() * scene_render->environment_get_ambient_light_energy(env);
			}
			const RSE::EnvironmentBG bg = scene_render->environment_get_background(env);
			const float bg_energy = scene_render->environment_get_bg_energy_multiplier(env);
			if (bg == RSE::ENV_BG_CLEAR_COLOR || bg == RSE::ENV_BG_COLOR) {
				const Color c = (bg == RSE::ENV_BG_CLEAR_COLOR ? RSG::texture_storage->get_default_clear_color() : scene_render->environment_get_bg_color(env)).srgb_to_linear() * bg_energy * exposure;
				data.sky_flat[0] = c.r;
				data.sky_flat[1] = c.g;
				data.sky_flat[2] = c.b;
			} else if (bg == RSE::ENV_BG_SKY) {
				const RID sky = scene_render->environment_get_sky(env);
				const RendererRD::SkyRD::Sky *sky_data = sky.is_valid() ? scene_render->get_sky()->get_sky(sky) : nullptr;
				if (sky_data && sky_data->radiance.is_valid()) {
					// The radiance map is baked at an exposure of its own (Godot's IBL normalisation).
					float exposure_ibl = 1.0f;
					if (p_render_data->get_camera_attributes().is_valid()) {
						exposure_ibl = exposure * scene_render->environment_get_bg_intensity(env) / MAX(0.001f, scene_render->get_sky()->sky_get_baked_exposure(sky));
					}
					sky_map = sky_data->radiance;
					data.sky[0] = float(scene_render->get_roughness_layers() - 1);
					data.sky[1] = sky_data->uv_border_size;
					data.sky[2] = 1.0f - sky_data->uv_border_size * 2.0f;
					data.sky[3] = bg_energy * exposure_ibl;
					const Basis to_sky = scene_render->environment_get_sky_orientation(env).inverse();
					float *rows[3] = { data.sky_x, data.sky_y, data.sky_z };
					for (int k = 0; k < 3; k++) {
						rows[k][0] = to_sky.rows[k].x;
						rows[k][1] = to_sky.rows[k].y;
						rows[k][2] = to_sky.rows[k].z;
					}
				}
			}
		}
		data.ambient[0] = ambient_color.r;
		data.ambient[1] = ambient_color.g;
		data.ambient[2] = ambient_color.b;
	}
	if (sky_map.is_null()) {
		sky_map = RendererRD::TextureStorage::get_singleton()->texture_rd_get_default(RendererSceneRenderRD::get_singleton()->is_using_radiance_octmap_array() ? RendererRD::TextureStorage::DEFAULT_RD_TEXTURE_2D_ARRAY_BLACK : RendererRD::TextureStorage::DEFAULT_RD_TEXTURE_BLACK);
	}
	// Traced + probes: the reflection probes Godot draws this frame (those whose box the view reaches) and have been
	// captured. Their captures, in Godot's reflection atlas, colour the reflections' off-screen hits, and inside their box
	// fill in what the rays don't give (misses, the cutoff's fade, rougher surfaces) as Godot's own probe would.
	RID probe_atlas;
	if (s.reflection_mode == 3) {
		const RenderDataRD *render_data = Object::cast_to<RenderDataRD>(p_render_data);
		RendererRD::LightStorage *light_storage = RendererRD::LightStorage::get_singleton();
		if (render_data && render_data->reflection_atlas.is_valid() && render_data->reflection_probes) {
			const float border = light_storage->reflection_atlas_get_border_size(render_data->reflection_atlas);
			data.src_border[0] = border;
			data.src_border[1] = 1.0f - border * 2.0f;
			data.src_border[2] = float(RendererSceneRenderRD::get_singleton()->get_roughness_layers() - 1);
			int count = 0;
			for (uint64_t i = 0; i < render_data->reflection_probes->size() && count < 8; i++) {
				const RID instance = (*render_data->reflection_probes)[i];
				const int layer = light_storage->reflection_probe_instance_get_atlas_index(instance);
				if (layer < 0) {
					continue;
				}
				const RID probe = light_storage->reflection_probe_instance_get_probe(instance);
				const Transform3D xf = light_storage->reflection_probe_instance_get_transform(instance);
				const Transform3D to_box = xf.affine_inverse();
				const Vector3 extents = light_storage->reflection_probe_get_size(probe) * 0.5f;
				const Vector3 offset = light_storage->reflection_probe_get_origin_offset(probe);
				const Vector3 capture = xf.xform(offset);
				float *rows[3] = { data.src_x[count], data.src_y[count], data.src_z[count] };
				for (int k = 0; k < 3; k++) {
					rows[k][0] = to_box.basis.rows[k].x;
					rows[k][1] = to_box.basis.rows[k].y;
					rows[k][2] = to_box.basis.rows[k].z;
					rows[k][3] = to_box.origin[k];
				}
				data.src_box[count][0] = extents.x;
				data.src_box[count][1] = extents.y;
				data.src_box[count][2] = extents.z;
				data.src_box[count][3] = float(layer * 2 + (light_storage->reflection_probe_is_box_projection(probe) ? 1 : 0));
				data.src_offset[count][0] = offset.x;
				data.src_offset[count][1] = offset.y;
				data.src_offset[count][2] = offset.z;
				// Captured at an exposure of its own, as Godot's own reflection scales it.
				data.src_offset[count][3] = p_render_data->get_camera_attributes().is_valid() ? exposure / MAX(0.001f, light_storage->reflection_probe_get_baked_exposure(probe)) : 1.0f;
				data.src_capture[count][0] = capture.x;
				data.src_capture[count][1] = capture.y;
				data.src_capture[count][2] = capture.z;
				data.src_capture[count][3] = light_storage->reflection_probe_get_blend_distance(probe);
				count++;
			}
			data.srcs[0] = count;
			if (count > 0) {
				probe_atlas = light_storage->reflection_atlas_get_texture(render_data->reflection_atlas);
			}
		}
	}
	if (probe_atlas.is_null()) {
		data.srcs[0] = 0;
		probe_atlas = RendererRD::TextureStorage::get_singleton()->texture_rd_get_default(RendererRD::TextureStorage::DEFAULT_RD_TEXTURE_2D_ARRAY_BLACK);
	}
	data.ambient[3] = s.world_distance;
	data.counts[0] = s.primitive_count;
	data.counts[1] = s.node_count;
	data.counts[2] = s.omni_count;
	data.counts[3] = s.spot_count;
	for (int i = 0; i < s.spot_count; i++) {
		data.spot_position[i][0] = s.spot_position[i].x;
		data.spot_position[i][1] = s.spot_position[i].y;
		data.spot_position[i][2] = s.spot_position[i].z;
		data.spot_position[i][3] = s.spot_inv_range[i];
		data.spot_direction[i][0] = s.spot_direction[i].x;
		data.spot_direction[i][1] = s.spot_direction[i].y;
		data.spot_direction[i][2] = s.spot_direction[i].z;
		data.spot_direction[i][3] = s.spot_cos_angle[i];
		data.spot_color[i][0] = s.spot_color[i].r * exposure;
		data.spot_color[i][1] = s.spot_color[i].g * exposure;
		data.spot_color[i][2] = s.spot_color[i].b * exposure;
		data.spot_color[i][3] = s.spot_attenuation[i];
		data.spot_cone[i][0] = s.spot_cone_attenuation[i];
	}
	mesh_walk = s.mesh_walk;
	data.reflect[0] = (float)s.reflection_mode;
	data.reflect[1] = s.reflection_cutoff;
	data.reflect[2] = (float)s.emissive_count;
	data.reflect[3] = s.light_changed ? 1.0f : 0.0f;
	float running = 0.0f;
	for (int i = 0; i < s.emissive_count; i++) {
		const Vector3 c = s.emissive_center[i];
		data.emit_center[i][0] = c.x;
		data.emit_center[i][1] = c.y;
		data.emit_center[i][2] = c.z;
		data.emit_center[i][3] = (float)s.emissive_slot[i];
		running += s.emissive_pick[i];
		data.emit_cdf[i / 4][i % 4] = i == s.emissive_count - 1 ? 1.0f : running;
		float *axes[3] = { data.emit_x[i], data.emit_y[i], data.emit_z[i] };
		for (int k = 0; k < 3; k++) {
			axes[k][0] = s.emissive_axis[i][k].x;
			axes[k][1] = s.emissive_axis[i][k].y;
			axes[k][2] = s.emissive_axis[i][k].z;
			axes[k][3] = s.emissive_facing[i][k]; // the axes' w: which way a one-sided glow faces
		}
		data.emit_color[i][0] = s.emissive_color[i].r * exposure;
		data.emit_color[i][1] = s.emissive_color[i].g * exposure;
		data.emit_color[i][2] = s.emissive_color[i].b * exposure;
	}
	data.cache[0] = frame;
	data.cache[1] = CACHE_SLOTS - 1;
	data.cache[2] = s.multi_bounce ? 1 : 0;
	data.cache[3] = s.rays_per_texel;
	data.probes[0] = s.probe_count;
	for (int i = 0; i < s.probe_count; i++) {
		const Transform3D &t = s.probe_to_local[i];
		for (int k = 0; k < 3; k++) {
			float *row = k == 0 ? data.probe_x[i] : (k == 1 ? data.probe_y[i] : data.probe_z[i]);
			row[0] = t.basis.rows[k].x;
			row[1] = t.basis.rows[k].y;
			row[2] = t.basis.rows[k].z;
			row[3] = t.origin[k];
		}
		data.probe_core[i][0] = s.probe_core[i].x;
		data.probe_core[i][1] = s.probe_core[i].y;
		data.probe_core[i][2] = s.probe_core[i].z;
		data.probe_core[i][3] = (float)s.probe_overrides[i];
	}
	for (int i = 0; i < s.omni_count; i++) {
		data.omni_position[i][0] = s.omni_position[i].x;
		data.omni_position[i][1] = s.omni_position[i].y;
		data.omni_position[i][2] = s.omni_position[i].z;
		data.omni_position[i][3] = s.omni_inv_range[i];
		data.omni_color[i][0] = s.omni_color[i].r * exposure;
		data.omni_color[i][1] = s.omni_color[i].g * exposure;
		data.omni_color[i][2] = s.omni_color[i].b * exposure;
		data.omni_color[i][3] = s.omni_attenuation[i];
	}
	rd->buffer_update(data_buffer, 0, sizeof(data), &data);

	// The shapes, grown as needed (an empty buffer can't be bound: at least one); sent only when they changed.
	const bool send_primitives = s.primitives_version != primitives_uploaded;
	primitives_uploaded = s.primitives_version;
	if (s.primitive_count > primitive_capacity || primitive_buffer.is_null()) {
		if (primitive_buffer.is_valid()) {
			rd->free_rid(primitive_buffer);
		}
		primitive_capacity = MAX(s.primitive_count, 64);
		primitive_buffer = rd->storage_buffer_create(primitive_capacity * PRIMITIVE_FLOATS * sizeof(float));
		primitives_uploaded = UINT64_MAX;
	}
	if ((send_primitives || primitives_uploaded == UINT64_MAX) && s.primitive_count > 0) {
		rd->buffer_update(primitive_buffer, 0, s.primitive_count * PRIMITIVE_FLOATS * sizeof(float), s.primitives.ptr());
	}
	if (s.node_count > node_capacity || node_buffer.is_null()) {
		if (node_buffer.is_valid()) {
			rd->free_rid(node_buffer);
		}
		node_capacity = MAX(s.node_count, 128);
		node_buffer = rd->storage_buffer_create(node_capacity * NODE_FLOATS * sizeof(float));
		primitives_uploaded = UINT64_MAX;
	}
	if ((send_primitives || primitives_uploaded == UINT64_MAX) && s.node_count > 0) {
		rd->buffer_update(node_buffer, 0, s.node_count * NODE_FLOATS * sizeof(float), s.nodes.ptr());
	}
	if (primitives_uploaded == UINT64_MAX) {
		primitives_uploaded = s.primitives_version;
	}
	if (s.geometry_version != geometry_uploaded || geometry_buffer.is_null()) {
		if (geometry_buffer.is_valid()) {
			rd->free_rid(geometry_buffer);
		}
		Vector<uint8_t> bytes;
		bytes.resize(MAX(s.geometry.size(), 4) * sizeof(float));
		bytes.fill(0);
		if (s.geometry.size() > 0) {
			memcpy(bytes.ptrw(), s.geometry.ptr(), s.geometry.size() * sizeof(float));
		}
		geometry_buffer = rd->storage_buffer_create(bytes.size(), bytes);
		geometry_uploaded = s.geometry_version;
	}

	auto sampled = [&](int p_binding, const RID &p_sampler, const RID &p_texture) {
		return RD::Uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, p_binding, Vector<RID>({ p_sampler, p_texture }));
	};
	const int prev = v.current;
	v.current = 1 - v.current;

	if (s.multi_bounce && s.primitive_count > 0) {
		_dispatch(rd, PASS_CACHE, { sampled(0, nearest_sampler, depth), sampled(2, linear_sampler, color), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 4, primitive_buffer), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 5, node_buffer), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 6, cache_keys), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 7, cache_cells), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 10, geometry_buffer), sampled(11, linear_sampler, sky_map) }, Size2i(CACHE_ROW, CACHE_SLOTS / CACHE_ROW / 4), nullptr, 0);
	}
	_dispatch(rd, PASS_TRACE, { sampled(0, nearest_sampler, depth), sampled(1, nearest_sampler, normal), sampled(2, linear_sampler, color), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 3, v.trace), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 4, primitive_buffer), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 5, node_buffer), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 6, cache_keys), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 7, cache_cells), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 10, geometry_buffer), sampled(11, linear_sampler, sky_map) }, v.trace_size, nullptr, 0);

	int32_t history[4] = { v.have_history ? 1 : 0, 0, 0, 0 };
	_dispatch(rd, PASS_TEMPORAL, { sampled(0, nearest_sampler, v.trace), sampled(1, linear_sampler, v.accum[prev]), sampled(2, nearest_sampler, v.accum_depth[prev]), sampled(3, nearest_sampler, depth), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 4, v.accum[v.current]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 5, v.accum_depth[v.current]), sampled(6, linear_sampler, v.moments[prev]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 7, v.moments[v.current]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 8, v.variance) }, v.trace_size, history, sizeof(history));

	// Three filter passes, the taps 1, 2 and 4 texels apart.
	RID light_in = v.accum[v.current];
	RID variance_in = v.variance;
	for (int i = 0; i < 3; i++) {
		int32_t step[4] = { 1 << i, 0, 0, 0 };
		_dispatch(rd, PASS_ATROUS, { sampled(0, nearest_sampler, light_in), sampled(1, nearest_sampler, variance_in), sampled(2, nearest_sampler, v.accum_depth[v.current]), sampled(3, nearest_sampler, normal), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 4, v.filtered[i & 1]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 5, v.filtered_variance[i & 1]) }, v.trace_size, step, sizeof(step));
		light_in = v.filtered[i & 1];
		variance_in = v.filtered_variance[i & 1];
	}

	RID reflections = v.reflect_accum[v.current]; // what the resolve reads: the accumulated reflections, filtered
	{
		int32_t reflect_params[4] = { v.reflect_size.x, v.reflect_size.y, 0, 0 };
		_dispatch(rd, PASS_REFLECT, { sampled(0, nearest_sampler, depth), sampled(1, nearest_sampler, normal), sampled(2, linear_sampler, color), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 3, v.reflect_trace), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 4, primitive_buffer), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 5, node_buffer), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 6, cache_keys), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 7, cache_cells), RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 10, geometry_buffer), sampled(11, linear_sampler, sky_map), sampled(12, linear_sampler, probe_atlas) }, v.reflect_size, reflect_params, sizeof(reflect_params));
		reflect_params[2] = v.have_history ? 1 : 0;
		_dispatch(rd, PASS_REFLECT_TEMPORAL, { sampled(0, nearest_sampler, v.reflect_trace), sampled(1, linear_sampler, v.reflect_accum[prev]), sampled(2, nearest_sampler, v.reflect_depth[prev]), sampled(3, nearest_sampler, depth), sampled(4, nearest_sampler, normal), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 5, v.reflect_accum[v.current]), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 6, v.reflect_depth[v.current]) }, v.reflect_size, reflect_params, sizeof(reflect_params));
		// Two filter passes, the taps 1 and 2 texels apart, into their own textures (the history stays unfiltered).
		for (int i = 0; i < 2; i++) {
			int32_t step[4] = { 1 << i, 0, v.reflect_size.x, v.reflect_size.y };
			_dispatch(rd, PASS_REFLECT_FILTER, { sampled(0, nearest_sampler, reflections), sampled(1, nearest_sampler, v.reflect_depth[v.current]), sampled(2, nearest_sampler, normal), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 3, v.reflect_filtered[i]) }, v.reflect_size, step, sizeof(step));
			reflections = v.reflect_filtered[i];
		}
	}

	int32_t out[4] = { out_size.x, out_size.y, v.reflect_size.x, v.reflect_size.y };
	_dispatch(rd, PASS_RESOLVE, { sampled(0, nearest_sampler, light_in), sampled(1, nearest_sampler, v.accum_depth[v.current]), sampled(2, nearest_sampler, depth), sampled(3, nearest_sampler, normal), sampled(6, linear_sampler, color), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 4, ambient), RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 5, reflection), sampled(7, nearest_sampler, reflections), sampled(8, nearest_sampler, v.reflect_depth[v.current]), sampled(12, mip_sampler, probe_atlas) }, out_size, out, sizeof(out));

	rd->capture_timestamp("PXTGI end");
	v.have_history = true;
	v.prev_cam_transform = cam_transform;
	v.prev_projection = projection;
}

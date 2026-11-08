/**************************************************************************/
/*  groom_sim.cpp                                                         */
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

#include "groom_sim.h"

#include "../godot_physx_render_thread.h"
#include "physx_groom.h"

#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"

namespace {

// The texture's width in texels: a point at i is at (i % width, i / width).
constexpr int TEXTURE_WIDTH = 2048;
// Floats in the parameters buffer: see Params below.
constexpr int PARAMS_FLOATS = 52 + GroomSim::MAX_COLLIDERS * 8;

const char *SIM_SHADER = R"(
#[compute]
#version 450
layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout(set = 0, binding = 0, std430) restrict readonly buffer Params {
	mat4 model; // the groom's space to world, this frame
	mat4 model_prev; // and last frame
	vec4 step; // substep length (s), velocity kept per substep, substeps, colliders
	vec4 gravity; // xyz, pull back toward the groomed shape per substep (at the root)
	vec4 wind; // air velocity xyz, drag
	vec4 shape; // bend kept per substep, 1 to start over from the groomed shape, margin off the colliders
	vec4 counts; // strands, texture width
	vec4 colliders[]; // two a capsule: a.xyz, radius; b.xyz
} p;
layout(set = 0, binding = 1, std430) restrict readonly buffer Rest {
	vec4 rest[]; // the groom's space; w: the length of the segment from the point before
};
layout(set = 0, binding = 2, std430) restrict readonly buffer Offsets {
	uint offsets[]; // each strand's first point, then the point count
};
layout(set = 0, binding = 3, std430) restrict buffer Pos {
	vec4 pos[]; // world space
};
layout(set = 0, binding = 4, std430) restrict buffer Prev {
	vec4 prev[];
};
layout(set = 0, binding = 5, rgba32f) uniform restrict writeonly image2D points_out;

// The smallest rotation taking unit vector a onto unit vector b.
mat3 turn_onto(vec3 a, vec3 b) {
	vec3 v = cross(a, b);
	float c = dot(a, b);
	if (c < -0.9999) {
		return mat3(1.0);
	}
	mat3 vx = mat3(0.0, v.z, -v.y, -v.z, 0.0, v.x, v.y, -v.x, 0.0);
	return mat3(1.0) + vx + vx * vx / (1.0 + c);
}

vec3 collide(vec3 x) {
	int count = int(p.step.w);
	for (int c = 0; c < count; c++) {
		vec4 a = p.colliders[c * 2];
		vec3 b = p.colliders[c * 2 + 1].xyz;
		vec3 ab = b - a.xyz;
		float t = dot(ab, ab) > 1e-12 ? clamp(dot(x - a.xyz, ab) / dot(ab, ab), 0.0, 1.0) : 0.0;
		vec3 closest = a.xyz + ab * t;
		vec3 d = x - closest;
		float r = a.w + p.shape.z;
		float dist2 = dot(d, d);
		if (dist2 < r * r) {
			x = closest + (dist2 > 1e-12 ? d * inversesqrt(dist2) : vec3(0.0, 1.0, 0.0)) * r;
		}
	}
	return x;
}

void main() {
	uint s = gl_GlobalInvocationID.x;
	if (s >= uint(p.counts.x)) {
		return;
	}
	uint from = offsets[s];
	uint to = offsets[s + 1u];
	float dt = p.step.x;

	if (p.shape.y > 0.5) {
		for (uint i = from; i < to; i++) {
			vec3 w = (p.model * vec4(rest[i].xyz, 1.0)).xyz;
			pos[i] = vec4(w, 0.0);
			prev[i] = vec4(w, 0.0);
		}
	} else {
		int substeps = int(p.step.z);
		// TressFX's scheme (AMD, MIT): the root and the point after it follow the groom; the rest fall and fly, are pulled
		// back toward the groomed shape, keep each segment's bend from the one before it, and keep their lengths. The
		// bend and length corrections move both ends of a segment, so they push back on what pulls them: one-sided
		// corrections (follow the leader) add energy, and a strand nudged near the root grows into frizz.
		uint pinned = min(from + 2u, to);
		for (int k = 0; k < substeps; k++) {
			// The groom's transform through the frame.
			float f = float(k + 1) / float(substeps);
			mat4 m = p.model_prev * (1.0 - f) + p.model * f;
			mat3 turn = mat3(normalize(m[0].xyz), normalize(m[1].xyz), normalize(m[2].xyz));

			for (uint i = from; i < pinned; i++) {
				vec3 placed = (m * vec4(rest[i].xyz, 1.0)).xyz;
				pos[i] = vec4(placed, 0.0);
				prev[i] = vec4(placed, 0.0);
			}

			// Gravity, wind (across the strand: hair catches air side-on) and what it was doing, damped.
			for (uint i = pinned; i < to; i++) {
				vec3 x = pos[i].xyz;
				vec3 v = (x - prev[i].xyz) * p.step.y;
				vec3 a = p.gravity.xyz;
				if (p.wind.w > 0.0) {
					vec3 t = x - pos[i - 1u].xyz;
					t = dot(t, t) > 1e-20 ? normalize(t) : vec3(0.0, -1.0, 0.0);
					vec3 air = p.wind.xyz - v / dt;
					a += (air - t * dot(air, t)) * p.wind.w;
				}
				prev[i] = vec4(x, 0.0);
				pos[i] = vec4(x + v + a * dt * dt, 0.0);
			}

			// Back toward the groomed shape, most at the root, none at the tip.
			for (uint i = pinned; i < to; i++) {
				float along = float(i - from) / float(to - from - 1u);
				float hold = p.gravity.w * (1.0 - along) * (1.0 - along);
				pos[i].xyz = mix(pos[i].xyz, (m * vec4(rest[i].xyz, 1.0)).xyz, hold);
			}

			// Each segment's bend from the one before: the groomed next segment, turned as the segment before it has.
			for (int iteration = 0; iteration < 2; iteration++) {
				for (uint i = from + 1u; i + 1u < to; i++) {
					vec3 last = pos[i].xyz - pos[i - 1u].xyz;
					vec3 groomed_last = turn * (rest[i].xyz - rest[i - 1u].xyz);
					if (dot(last, last) < 1e-20 || dot(groomed_last, groomed_last) < 1e-20) {
						continue;
					}
					vec3 next = turn_onto(normalize(groomed_last), normalize(last)) * (turn * (rest[i + 1u].xyz - rest[i].xyz));
					vec3 d = (pos[i].xyz + next - pos[i + 1u].xyz) * p.shape.x;
					if (i >= pinned) {
						pos[i].xyz -= d;
					}
					pos[i + 1u].xyz += d;
				}
			}

			// Lengths: both ends of a segment move, a pinned one doesn't.
			for (int iteration = 0; iteration < 3; iteration++) {
				for (uint i = from + 1u; i < to; i++) {
					vec3 d = pos[i].xyz - pos[i - 1u].xyz;
					float len = max(sqrt(dot(d, d)), 1e-7);
					vec3 c = d * (1.0 - rest[i].w / len);
					if (i - 1u >= pinned) {
						pos[i - 1u].xyz += 0.5 * c;
						pos[i].xyz -= 0.5 * c;
					} else if (i >= pinned) {
						pos[i].xyz -= c;
					}
				}
			}

			for (uint i = pinned; i < to; i++) {
				pos[i].xyz = collide(pos[i].xyz);
			}
		}
	}

	int width = int(p.counts.y);
	for (uint i = from; i < to; i++) {
		imageStore(points_out, ivec2(int(i) % width, int(i) / width), vec4(pos[i].xyz, 1.0));
	}
}
)";

void put_mat4(float *r_dst, const Transform3D &p_t) {
	const Basis &b = p_t.basis;
	const float m[16] = {
		(float)b.rows[0][0], (float)b.rows[1][0], (float)b.rows[2][0], 0.0f,
		(float)b.rows[0][1], (float)b.rows[1][1], (float)b.rows[2][1], 0.0f,
		(float)b.rows[0][2], (float)b.rows[1][2], (float)b.rows[2][2], 0.0f,
		(float)p_t.origin.x, (float)p_t.origin.y, (float)p_t.origin.z, 1.0f
	};
	memcpy(r_dst, m, sizeof(m));
}

// Made once on the main RenderingDevice, shared by every groom.
RID shared_shader;
RID shared_pipeline;
bool shared_failed = false;

bool ensure_shared(RenderingDevice *p_rd) {
	if (shared_pipeline.is_valid()) {
		return true;
	}
	if (shared_failed) {
		return false;
	}
	shared_failed = true;
	Ref<RDShaderFile> file;
	file.instantiate();
	ERR_FAIL_COND_V_MSG(file->parse_versions_from_text(SIM_SHADER) != OK, false, "PhysXGroom3D: the simulation shader failed to compile.");
	Ref<RDShaderSPIRV> spirv = file->get_spirv();
	ERR_FAIL_COND_V(spirv.is_null(), false);
	const String err = spirv->get_stage_compile_error(RD::SHADER_STAGE_COMPUTE);
	ERR_FAIL_COND_V_MSG(!err.is_empty(), false, "PhysXGroom3D: simulation shader: " + err);
	shared_shader = p_rd->shader_create_from_spirv(file->get_spirv_stages());
	ERR_FAIL_COND_V(!shared_shader.is_valid(), false);
	shared_pipeline = p_rd->compute_pipeline_create(shared_shader);
	ERR_FAIL_COND_V(!shared_pipeline.is_valid(), false);
	shared_failed = false;
	return true;
}

} // namespace

// What lives on the GPU, touched only on the render thread.
class GroomSim::GPU : public RefCounted {
	GDSOFTCLASS(GPU, RefCounted);

public:
	RID params;
	RID rest;
	RID offsets;
	RID pos;
	RID prev;
	RID texture;
	RID uniform_set;
	int strand_count = 0;
	int width = TEXTURE_WIDTH;
	bool start_over = true;

	void free_all(RenderingDevice *p_rd) {
		for (RID *r : { &uniform_set, &params, &rest, &offsets, &pos, &prev, &texture }) {
			if (r->is_valid()) {
				p_rd->free_rid(*r);
				*r = RID();
			}
		}
		strand_count = 0;
	}
};

void GroomSim::build(const Ref<PhysXGroom> &p_groom) {
	clear();
	ERR_FAIL_COND(p_groom.is_null() || !p_groom->is_valid());
	const PackedVector3Array points = p_groom->get_points();
	const PackedInt32Array strand_offsets = p_groom->get_strand_offsets();
	const int strand_count = p_groom->get_strand_count();
	const int point_count = points.size();

	PackedByteArray rest;
	rest.resize(point_count * 4 * sizeof(float));
	float *r = (float *)rest.ptrw();
	for (int s = 0; s < strand_count; s++) {
		for (int i = strand_offsets[s]; i < strand_offsets[s + 1]; i++) {
			r[i * 4 + 0] = points[i].x;
			r[i * 4 + 1] = points[i].y;
			r[i * 4 + 2] = points[i].z;
			r[i * 4 + 3] = i > strand_offsets[s] ? points[i].distance_to(points[i - 1]) : 0.0f;
		}
	}
	PackedByteArray offsets;
	offsets.resize((strand_count + 1) * sizeof(uint32_t));
	uint32_t *o = (uint32_t *)offsets.ptrw();
	for (int s = 0; s <= strand_count; s++) {
		o[s] = (uint32_t)strand_offsets[s];
	}
	PackedByteArray zeros;
	zeros.resize(point_count * 4 * sizeof(float));
	zeros.fill(0);

	Ref<GPU> g;
	g.instantiate();
	g->strand_count = strand_count;
	const int height = (point_count + TEXTURE_WIDTH - 1) / TEXTURE_WIDTH;
	godot_physx_run_on_render_thread([&]() {
		RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
		if (rd == nullptr || !ensure_shared(rd)) {
			return;
		}
		g->params = rd->storage_buffer_create(PARAMS_FLOATS * sizeof(float));
		g->rest = rd->storage_buffer_create(rest.size());
		rd->buffer_update(g->rest, 0, rest.size(), rest.ptr());
		g->offsets = rd->storage_buffer_create(offsets.size());
		rd->buffer_update(g->offsets, 0, offsets.size(), offsets.ptr());
		g->pos = rd->storage_buffer_create(zeros.size());
		rd->buffer_update(g->pos, 0, zeros.size(), zeros.ptr());
		g->prev = rd->storage_buffer_create(zeros.size());
		rd->buffer_update(g->prev, 0, zeros.size(), zeros.ptr());
		RD::TextureFormat tf;
		tf.format = RD::DATA_FORMAT_R32G32B32A32_SFLOAT;
		tf.width = TEXTURE_WIDTH;
		tf.height = MAX(height, 1);
		tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT | RD::TEXTURE_USAGE_CAN_UPDATE_BIT | RD::TEXTURE_USAGE_CAN_COPY_TO_BIT;
		g->texture = rd->texture_create(tf, RD::TextureView());
		rd->texture_clear(g->texture, Color(0, 0, 0, 0), 0, 1, 0, 1);
		Vector<RD::Uniform> uniforms;
		uniforms.push_back(RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 0, g->params));
		uniforms.push_back(RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 1, g->rest));
		uniforms.push_back(RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 2, g->offsets));
		uniforms.push_back(RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 3, g->pos));
		uniforms.push_back(RD::Uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 4, g->prev));
		uniforms.push_back(RD::Uniform(RD::UNIFORM_TYPE_IMAGE, 5, g->texture));
		g->uniform_set = rd->uniform_set_create(uniforms, shared_shader, 0);
	});
	if (g->uniform_set.is_valid()) {
		gpu = g;
	} else {
		godot_physx_run_on_render_thread([&]() {
			RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
			if (rd != nullptr) {
				g->free_all(rd);
			}
		});
	}
}

void GroomSim::step(const Transform3D &p_from, const Transform3D &p_to, double p_delta, const Settings &p_settings, const LocalVector<Collider> &p_colliders) {
	if (gpu.is_null()) {
		return;
	}
	// About 240 substeps a second, at most 8 a frame; a long frame (a hitch) counts as 1/20 s.
	const double delta = CLAMP(p_delta, 1.0 / 1000.0, 1.0 / 20.0);
	const int substeps = CLAMP((int)Math::ceil(delta * 240.0), 1, 8);
	const float dt = (float)(delta / substeps);
	const auto per_substep = [dt](float p_per_60th) {
		return 1.0f - Math::pow(1.0f - CLAMP(p_per_60th, 0.0f, 0.999f), dt * 60.0f);
	};

	PackedFloat32Array params;
	params.resize(PARAMS_FLOATS);
	params.fill(0.0f);
	float *w = params.ptrw();
	put_mat4(w + 0, p_to);
	put_mat4(w + 16, p_from);
	const int collider_count = MIN((int)p_colliders.size(), MAX_COLLIDERS);
	w[32] = dt;
	// Velocity kept per substep: TressFX's damping, per 1/60 s.
	w[33] = Math::exp(-MAX(p_settings.damping, 0.0f) * dt * 60.0f);
	w[34] = (float)substeps;
	w[35] = (float)collider_count;
	w[36] = p_settings.gravity.x;
	w[37] = p_settings.gravity.y;
	w[38] = p_settings.gravity.z;
	w[39] = per_substep(p_settings.stiffness);
	w[40] = p_settings.wind.x;
	w[41] = p_settings.wind.y;
	w[42] = p_settings.wind.z;
	w[43] = p_settings.wind.is_zero_approx() ? 0.0f : MAX(p_settings.drag, 0.0f);
	// Per pass, two passes a substep; as TressFX, at most half (the full step overshoots).
	w[44] = 0.5f * MIN(CLAMP(p_settings.bend_stiffness, 0.0f, 1.0f), 0.95f);
	w[45] = 0.0f; // start over: set on the render thread
	w[46] = MAX(p_settings.margin, 0.0f);
	w[48] = (float)gpu->strand_count;
	w[49] = (float)gpu->width;
	for (int c = 0; c < collider_count; c++) {
		float *cw = w + 52 + c * 8;
		const Collider &col = p_colliders[c];
		cw[0] = col.a.x;
		cw[1] = col.a.y;
		cw[2] = col.a.z;
		cw[3] = col.radius;
		cw[4] = col.b.x;
		cw[5] = col.b.y;
		cw[6] = col.b.z;
	}

	Ref<GPU> g = gpu;
	auto run = [g, params]() {
		RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
		if (rd == nullptr || !g->uniform_set.is_valid() || !rd->uniform_set_is_valid(g->uniform_set)) {
			return;
		}
		PackedFloat32Array p = params;
		p.write[45] = g->start_over ? 1.0f : 0.0f;
		g->start_over = false;
		rd->buffer_update(g->params, 0, p.size() * sizeof(float), p.ptr());
		const RD::ComputeListID list = rd->compute_list_begin();
		rd->compute_list_bind_compute_pipeline(list, shared_pipeline);
		rd->compute_list_bind_uniform_set(list, g->uniform_set, 0);
		rd->compute_list_dispatch(list, (g->strand_count + 63) / 64, 1, 1);
		rd->compute_list_end();
	};
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs->is_on_render_thread()) {
		run();
	} else {
		rs->call_on_render_thread(Callable(memnew(GodotPhysXQueuedCall<decltype(run)>(run))));
	}
}

void GroomSim::reset() {
	if (gpu.is_null()) {
		return;
	}
	Ref<GPU> g = gpu;
	auto run = [g]() { g->start_over = true; };
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs->is_on_render_thread()) {
		run();
	} else {
		rs->call_on_render_thread(Callable(memnew(GodotPhysXQueuedCall<decltype(run)>(run))));
	}
}

void GroomSim::clear() {
	if (gpu.is_null()) {
		return;
	}
	Ref<GPU> g = gpu;
	gpu.unref();
	godot_physx_run_on_render_thread([g]() {
		RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
		if (rd != nullptr) {
			g->free_all(rd);
		}
	});
}

bool GroomSim::is_built() const {
	return gpu.is_valid();
}

RID GroomSim::get_texture() const {
	return gpu.is_valid() ? gpu->texture : RID();
}

int GroomSim::get_width() const {
	return gpu.is_valid() ? gpu->width : 0;
}

void GroomSim::free_shared() {
	if (!shared_shader.is_valid()) {
		return;
	}
	godot_physx_run_on_render_thread([]() {
		RenderingDevice *rd = RenderingServer::get_singleton()->get_rendering_device();
		if (rd != nullptr && shared_shader.is_valid()) {
			// The pipeline goes with its shader.
			rd->free_rid(shared_shader);
		}
		shared_shader = RID();
		shared_pipeline = RID();
	});
}

GroomSim::GroomSim() {}

GroomSim::~GroomSim() {
	clear();
}

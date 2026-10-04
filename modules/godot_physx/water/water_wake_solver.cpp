/**************************************************************************/
/*  water_wake_solver.cpp                                                 */
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

#include "water_wake_solver.h"

#include "water_wake.glsl.gen.h"

#include "core/math/math_funcs.h"
#include "core/object/callable_mp.h"
#include "servers/display/display_server.h"
#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"

namespace {
constexpr int PARAMS_BYTES = 96; // ivec4 grid, vec4 wave, cell, foam, track, spread (std140)
constexpr int READBACK_EVERY = 3; // physics ticks between height readbacks

RID storage(RenderingDevice *p_rd, int p_bytes) {
	Vector<uint8_t> zeros;
	zeros.resize(MAX(p_bytes, 16));
	memset(zeros.ptrw(), 0, zeros.size());
	return p_rd->storage_buffer_create(zeros.size(), zeros);
}
} // namespace

void WaterWakeSolverGPU::rt_compile(Ref<WaterWakeSolverGPU> p_self) {
	if (rd == nullptr) {
		return;
	}
	Ref<RDShaderFile> sf;
	sf.instantiate();
	if (sf->parse_versions_from_text(water_wake_shader_glsl) != OK) {
		ERR_PRINT("WaterWakeSolver: the compute shader failed to compile.");
		return;
	}
	shader = rd->shader_create_from_spirv(sf->get_spirv_stages());
	ERR_FAIL_COND(shader.is_null());
	pipeline = rd->compute_pipeline_create(shader);
}

void WaterWakeSolverGPU::rt_free_buffers() {
	if (rd == nullptr) {
		return;
	}
	built.clear();
	for (RID *r : { &uset_atob, &uset_btoa }) {
		if (r->is_valid() && rd->uniform_set_is_valid(*r)) {
			rd->free_rid(*r);
		}
		*r = RID();
	}
	for (RID *r : { &buf_params, &buf_state_a, &buf_state_b, &buf_sources, &tex_height }) {
		if (r->is_valid()) {
			rd->free_rid(*r);
			*r = RID();
		}
	}
}

void WaterWakeSolverGPU::rt_free(Ref<WaterWakeSolverGPU> p_self) {
	rt_free_buffers();
	if (rd == nullptr) {
		return;
	}
	if (pipeline.is_valid()) {
		rd->free_rid(pipeline);
		pipeline = RID();
	}
	if (shader.is_valid()) {
		rd->free_rid(shader);
		shader = RID();
	}
}

void WaterWakeSolverGPU::rt_build(Ref<WaterWakeSolverGPU> p_self, int p_n) {
	if (rd == nullptr || shader.is_null()) {
		return;
	}
	rt_free_buffers();
	n = p_n;
	buf_params = rd->uniform_buffer_create(PARAMS_BYTES);
	buf_state_a = storage(rd, n * n * 16);
	buf_state_b = storage(rd, n * n * 16);
	buf_sources = storage(rd, MAX_SOURCES * 32);

	RD::TextureFormat tf;
	tf.format = RD::DATA_FORMAT_R16G16B16A16_SFLOAT;
	tf.width = n;
	tf.height = n;
	tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT | RD::TEXTURE_USAGE_CAN_UPDATE_BIT;
	Vector<uint8_t> zeros;
	zeros.resize(n * n * 8); // rgba16f
	memset(zeros.ptrw(), 0, zeros.size());
	Vector<Vector<uint8_t>> data;
	data.push_back(zeros);
	tex_height = rd->texture_create(tf, RD::TextureView(), data);

	auto make_set = [&](RID p_in, RID p_out) {
		Vector<RD::Uniform> u;
		RD::Uniform p;
		p.uniform_type = RD::UNIFORM_TYPE_UNIFORM_BUFFER;
		p.binding = 0;
		p.append_id(buf_params);
		u.push_back(p);
		const RID bufs[3] = { p_in, p_out, buf_sources };
		for (int i = 0; i < 3; i++) {
			RD::Uniform s;
			s.uniform_type = RD::UNIFORM_TYPE_STORAGE_BUFFER;
			s.binding = i + 1;
			s.append_id(bufs[i]);
			u.push_back(s);
		}
		RD::Uniform img;
		img.uniform_type = RD::UNIFORM_TYPE_IMAGE;
		img.binding = 4;
		img.append_id(tex_height);
		u.push_back(img);
		return rd->uniform_set_create(u, shader, 0);
	};
	uset_atob = make_set(buf_state_a, buf_state_b);
	uset_btoa = make_set(buf_state_b, buf_state_a);
	a_is_current = true;
	{
		MutexLock lock(cache_mtx);
		height_cache.clear();
	}
	built.set();
}

void WaterWakeSolverGPU::rt_step(Ref<WaterWakeSolverGPU> p_self, PackedByteArray p_params, PackedFloat32Array p_sources, Vector2 p_origin, bool p_readback) {
	if (rd == nullptr || !built.is_set()) {
		return;
	}
	rd->buffer_update(buf_params, 0, PARAMS_BYTES, p_params.ptr());
	if (!p_sources.is_empty()) {
		rd->buffer_update(buf_sources, 0, MIN(p_sources.size(), MAX_SOURCES * 8) * 4, p_sources.ptr());
	}
	const RID set = a_is_current ? uset_atob : uset_btoa;
	RD::ComputeListID cl = rd->compute_list_begin();
	rd->compute_list_bind_compute_pipeline(cl, pipeline);
	rd->compute_list_bind_uniform_set(cl, set, 0);
	const uint32_t groups = (uint32_t)((n + 7) / 8);
	rd->compute_list_dispatch(cl, groups, groups, 1);
	rd->compute_list_end();
	a_is_current = !a_is_current;
	if (p_readback) {
		const RID current = a_is_current ? buf_state_a : buf_state_b;
		rd->buffer_get_data_async(current, callable_mp(this, &WaterWakeSolverGPU::rt_on_readback).bind(p_self, p_origin), 0, n * n * 16);
	}
	if (local) {
		rd->submit();
		rd->sync();
	}
}

void WaterWakeSolverGPU::rt_on_readback(const PackedByteArray &p_data, Ref<WaterWakeSolverGPU> p_self, Vector2 p_origin) {
	const int count = p_data.size() / 16;
	Vector<float> h;
	h.resize(count);
	const float *src = (const float *)p_data.ptr();
	float *dst = h.ptrw();
	for (int i = 0; i < count; i++) {
		dst[i] = src[i * 4];
	}
	MutexLock lock(cache_mtx);
	height_cache = h;
	cache_origin = p_origin;
}

/* ===================================================================== */

WaterWakeSolver::WaterWakeSolver() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	gpu.instantiate();
	gpu->rd = rs->get_rendering_device();
	if (gpu->rd == nullptr) {
		if (!DisplayServer::can_create_rendering_device()) {
			gpu.unref();
			return;
		}
		gpu->rd = rs->create_local_rendering_device();
		if (gpu->rd == nullptr) {
			gpu.unref();
			return;
		}
		gpu->local = true;
	}
	_dispatch(callable_mp(gpu.ptr(), &WaterWakeSolverGPU::rt_compile).bind(gpu));
}

WaterWakeSolver::~WaterWakeSolver() {
	if (gpu.is_valid() && gpu->rd != nullptr) {
		if (gpu->local) {
			gpu->rt_free(gpu);
			memdelete(gpu->rd);
			gpu->rd = nullptr;
		} else {
			RenderingServer *rs = RenderingServer::get_singleton();
			rs->call_on_render_thread(callable_mp(gpu.ptr(), &WaterWakeSolverGPU::rt_free).bind(gpu));
			rs->sync();
		}
	}
	gpu.unref();
}

void WaterWakeSolver::_dispatch(const Callable &p_call) const {
	if (gpu.is_null()) {
		return;
	}
	if (gpu->local) {
		p_call.call();
	} else {
		RenderingServer::get_singleton()->call_on_render_thread(p_call);
	}
}

void WaterWakeSolver::build(int p_cells, float p_size) {
	n = MAX(p_cells, 16);
	size = MAX(p_size, 1.0f);
	have_origin = false;
	_dispatch(callable_mp(gpu.ptr(), &WaterWakeSolverGPU::rt_build).bind(gpu, n));
	if (gpu.is_valid() && !gpu->local) {
		// The texture RID must exist before the caller wraps it.
		RenderingServer::get_singleton()->sync();
	}
}

void WaterWakeSolver::step(double p_delta, const Vector2 &p_center, const Vector<Source> &p_sources, float p_wave_speed, float p_damping, float p_border_damping, float p_border_width, float p_foam_persistence, float p_foam_slope, float p_foam_slope_gain, const Vector2 &p_bow, const Vector2 &p_stern, float p_foam_spread, float p_foam_spread_falloff, float p_hull_half_beam, float p_hull_foam) {
	if (!is_built()) {
		return;
	}
	const float dx = get_cell_size();
	const float dt = (float)CLAMP(p_delta, 1.0 / 240.0, 1.0 / 20.0);
	const Vector2i new_origin((int)Math::floor(p_center.x / dx) - n / 2, (int)Math::floor(p_center.y / dx) - n / 2);
	Vector2i shift;
	if (have_origin) {
		shift = new_origin - origin_cell;
		if (Math::abs(shift.x) >= n || Math::abs(shift.y) >= n) {
			shift = Vector2i(n, n); // teleported: start from still water
		}
	}
	origin_cell = new_origin;
	have_origin = true;

	PackedByteArray params;
	params.resize(PARAMS_BYTES);
	uint8_t *w = params.ptrw();
	const int32_t grid[4] = { n, MIN(p_sources.size(), WaterWakeSolverGPU::MAX_SOURCES), shift.x, shift.y };
	// The explicit scheme is stable up to 0.5; wave speed is capped to keep it there.
	const float k = MIN(p_wave_speed * p_wave_speed * dt * dt / (dx * dx), 0.45f);
	const float wave[4] = { k, p_damping * dt, p_border_damping * dt, p_border_width };
	const Vector2 origin = get_origin();
	const float cell[4] = { dx, CLAMP(6.0f * dt, 0.0f, 1.0f), origin.x, origin.y };
	memcpy(w, grid, 16);
	memcpy(w + 16, wave, 16);
	memcpy(w + 32, cell, 16);
	const float foam[4] = { Math::exp(-dt / MAX(p_foam_persistence, 0.01f)), dt, p_foam_slope, p_foam_slope_gain };
	memcpy(w + 48, foam, 16);
	const float track[4] = { p_bow.x, p_bow.y, p_stern.x, p_stern.y };
	memcpy(w + 64, track, 16);
	const float spread[4] = { p_foam_spread, p_foam_spread_falloff, p_hull_half_beam, p_hull_foam };
	memcpy(w + 80, spread, 16);

	PackedFloat32Array src;
	src.resize(MIN(p_sources.size(), WaterWakeSolverGPU::MAX_SOURCES) * 8);
	float *sw = src.ptrw();
	memset(sw, 0, src.size() * sizeof(float));
	for (int i = 0; i < src.size() / 8; i++) {
		sw[i * 8 + 0] = p_sources[i].world_xz.x;
		sw[i * 8 + 1] = p_sources[i].world_xz.y;
		sw[i * 8 + 2] = p_sources[i].radius;
		sw[i * 8 + 3] = p_sources[i].depth;
		sw[i * 8 + 4] = p_sources[i].foam;
		sw[i * 8 + 5] = p_sources[i].drift ? 1.0f : 0.0f;
	}
	const bool readback = (ticks++ % READBACK_EVERY) == 0;
	_dispatch(callable_mp(gpu.ptr(), &WaterWakeSolverGPU::rt_step).bind(gpu, params, src, origin, readback));
}

RID WaterWakeSolver::get_height_texture_rd_rid() const {
	return gpu.is_valid() ? gpu->tex_height : RID();
}

float WaterWakeSolver::sample_height(const Vector2 &p_world_xz) const {
	if (gpu.is_null()) {
		return 0.0f;
	}
	MutexLock lock(gpu->cache_mtx);
	const Vector<float> &h = gpu->height_cache;
	if (h.size() != n * n) {
		return 0.0f;
	}
	const float dx = get_cell_size();
	const float gx = (p_world_xz.x - gpu->cache_origin.x) / dx - 0.5f;
	const float gz = (p_world_xz.y - gpu->cache_origin.y) / dx - 0.5f;
	if (gx < 0.0f || gz < 0.0f || gx > n - 1 || gz > n - 1) {
		return 0.0f;
	}
	const int x0 = MIN((int)gx, n - 2);
	const int z0 = MIN((int)gz, n - 2);
	const float fx = gx - x0;
	const float fz = gz - z0;
	const float a = Math::lerp(h[z0 * n + x0], h[z0 * n + x0 + 1], fx);
	const float b = Math::lerp(h[(z0 + 1) * n + x0], h[(z0 + 1) * n + x0 + 1], fx);
	// Same edge fade the water shader applies.
	const float u = MIN(MIN(gx, gz), MIN((float)(n - 1) - gx, (float)(n - 1) - gz)) / n;
	return Math::lerp(a, b, fz) * Math::smoothstep(0.0f, 0.1f, u);
}

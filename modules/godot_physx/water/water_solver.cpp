/**************************************************************************/
/*  water_solver.cpp                                                     */
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

#include "water_solver.h"

#include "water_blit_ocean.glsl.gen.h"
#include "water_blit_ripple.glsl.gen.h"
#include "water_fft.glsl.gen.h"
#include "water_ripple.glsl.gen.h"
#include "water_spectrum_evolve.glsl.gen.h"
#include "water_spectrum_init.glsl.gen.h"

#include "core/object/callable_mp.h"
#include "core/os/memory.h"
#include "servers/display/display_server.h"
#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"

namespace {
constexpr int PARAMS_BYTES = 48; // 3 * vec4, std140 -- see water_inc.glsl's Params
constexpr int OCEAN_INIT_PARAMS_BYTES = 48; // 3 * vec4 -- see water_spectrum_init.glsl's Params
constexpr int OCEAN_EVOLVE_PARAMS_BYTES = 32; // 2 * vec4 -- see water_spectrum_evolve.glsl's Params
constexpr int FFT_PARAMS_BYTES = 16; // 1 * ivec4 -- see water_fft.glsl's Params
constexpr int BLIT_PARAMS_BYTES = 16; // 1 * ivec4 -- see water_blit_*.glsl's Params
constexpr uint32_t GROUP = 8; // matches every water_*.glsl's local_size_x/y

uint32_t groups_for(int p_count) {
	return (uint32_t)((p_count + (int)GROUP - 1) / (int)GROUP);
}

int log2i(int p_v) {
	int r = 0;
	while ((1 << r) < p_v) {
		r++;
	}
	return r;
}
} // namespace

/* ===================================================================== */
/*  WaterSolverGPU                                                        */
/* ===================================================================== */

WaterSolverGPU::~WaterSolverGPU() {
#ifdef DEV_ENABLED
	// rt_free() should have run first (via WaterSolver's destructor). If a
	// stale async callback kept us alive past it, everything is already
	// null and there is nothing to do -- same discipline as
	// MPMFluidSolverGPU's own destructor.
	DEV_ASSERT(shader_ripple.is_null() && pipeline_ripple.is_null());
#endif
}

void WaterSolverGPU::rt_compile(Ref<WaterSolverGPU> p_self) {
	if (rd == nullptr) {
		return;
	}
	struct Entry {
		const char *src;
		RID *shader;
		RID *pipeline;
	};
	Entry entries[] = {
		{ water_ripple_shader_glsl, &shader_ripple, &pipeline_ripple },
		{ water_spectrum_init_shader_glsl, &shader_spectrum_init, &pipeline_spectrum_init },
		{ water_spectrum_evolve_shader_glsl, &shader_spectrum_evolve, &pipeline_spectrum_evolve },
		{ water_fft_shader_glsl, &shader_fft, &pipeline_fft },
		{ water_blit_ripple_shader_glsl, &shader_blit_ripple, &pipeline_blit_ripple },
		{ water_blit_ocean_shader_glsl, &shader_blit_ocean, &pipeline_blit_ocean },
	};
	for (Entry &e : entries) {
		Ref<RDShaderFile> sf;
		sf.instantiate();
		if (sf->parse_versions_from_text(e.src) != OK) {
			ERR_PRINT("WaterSolver: a compute shader failed to compile.");
			return;
		}
		*e.shader = rd->shader_create_from_spirv(sf->get_spirv_stages());
		ERR_FAIL_COND(e.shader->is_null());
		*e.pipeline = rd->compute_pipeline_create(*e.shader);
		ERR_FAIL_COND(e.pipeline->is_null());
	}
	shaders_ok = true;
}

void WaterSolverGPU::_rt_free_buffers() {
	RID *usets[] = { &uset_atob, &uset_btoa, &uset_spectrum_init, &uset_spectrum_evolve,
		&uset_fft_spec_to_a, &uset_fft_atob, &uset_fft_btoa, &uset_blit_ripple, &uset_blit_ocean };
	for (RID *u : usets) {
		if (u->is_valid()) {
			rd->free_rid(*u);
			*u = RID();
		}
	}
	RID *bufs[] = { &buf_params, &buf_state_a, &buf_state_b, &buf_height, &buf_spheres, &buf_impulses,
		&buf_ocean_init_params, &buf_ocean_evolve_params, &buf_fft_params, &buf_h0, &buf_ocean_spec, &buf_fft_a, &buf_fft_b,
		&buf_blit_ripple_params, &buf_blit_ocean_params };
	for (RID *b : bufs) {
		if (b->is_valid()) {
			rd->free_rid(*b);
			*b = RID();
		}
	}
	RID *texs[] = { &tex_ripple_height, &tex_ocean_height };
	for (RID *t : texs) {
		if (t->is_valid()) {
			rd->free_rid(*t);
			*t = RID();
		}
	}
}

void WaterSolverGPU::_rt_build_buffers() {
	const int n = grid_resolution;
	const int cells = n * n;
	const int on = ocean_grid_resolution;
	const int ocells = on * on;
	ocean_log2n = log2i(on);
	ocean_time_accum = 0.0;
	a_is_current = true;

	buf_params = rd->uniform_buffer_create(PARAMS_BYTES);
	buf_state_a = rd->storage_buffer_create(cells * 4 * sizeof(float));
	buf_state_b = rd->storage_buffer_create(cells * 4 * sizeof(float));
	buf_height = rd->storage_buffer_create(cells * sizeof(float));
	buf_spheres = rd->storage_buffer_create(MAX_SPHERES * 4 * sizeof(float));
	buf_impulses = rd->storage_buffer_create(MAX_IMPULSES * 4 * sizeof(float));

	auto make_uset = [&](RID p_state_in, RID p_state_out) {
		const RID by_binding[6] = { buf_params, p_state_in, p_state_out, buf_height, buf_spheres, buf_impulses };
		Vector<RD::Uniform> uniforms;
		for (int bnd = 0; bnd < 6; bnd++) {
			RD::Uniform u;
			u.uniform_type = (bnd == 0) ? RD::UNIFORM_TYPE_UNIFORM_BUFFER : RD::UNIFORM_TYPE_STORAGE_BUFFER;
			u.binding = bnd;
			u.append_id(by_binding[bnd]);
			uniforms.push_back(u);
		}
		return rd->uniform_set_create(uniforms, shader_ripple, 0);
	};
	uset_atob = make_uset(buf_state_a, buf_state_b);
	uset_btoa = make_uset(buf_state_b, buf_state_a);

	// FFT ocean layer.
	buf_ocean_init_params = rd->uniform_buffer_create(OCEAN_INIT_PARAMS_BYTES);
	buf_ocean_evolve_params = rd->uniform_buffer_create(OCEAN_EVOLVE_PARAMS_BYTES);
	buf_fft_params = rd->uniform_buffer_create(FFT_PARAMS_BYTES);
	buf_h0 = rd->storage_buffer_create(ocells * 4 * sizeof(float));
	buf_ocean_spec = rd->storage_buffer_create(ocells * 2 * sizeof(float));
	buf_fft_a = rd->storage_buffer_create(ocells * 2 * sizeof(float));
	buf_fft_b = rd->storage_buffer_create(ocells * 2 * sizeof(float));

	{
		Vector<RD::Uniform> uniforms;
		const RID by_binding[2] = { buf_ocean_init_params, buf_h0 };
		for (int bnd = 0; bnd < 2; bnd++) {
			RD::Uniform u;
			u.uniform_type = (bnd == 0) ? RD::UNIFORM_TYPE_UNIFORM_BUFFER : RD::UNIFORM_TYPE_STORAGE_BUFFER;
			u.binding = bnd;
			u.append_id(by_binding[bnd]);
			uniforms.push_back(u);
		}
		uset_spectrum_init = rd->uniform_set_create(uniforms, shader_spectrum_init, 0);
	}
	auto make_uset3 = [&](RID p_shader, RID p_params, RID p_in, RID p_out) {
		Vector<RD::Uniform> uniforms;
		const RID by_binding[3] = { p_params, p_in, p_out };
		for (int bnd = 0; bnd < 3; bnd++) {
			RD::Uniform u;
			u.uniform_type = (bnd == 0) ? RD::UNIFORM_TYPE_UNIFORM_BUFFER : RD::UNIFORM_TYPE_STORAGE_BUFFER;
			u.binding = bnd;
			u.append_id(by_binding[bnd]);
			uniforms.push_back(u);
		}
		return rd->uniform_set_create(uniforms, p_shader, 0);
	};
	uset_spectrum_evolve = make_uset3(shader_spectrum_evolve, buf_ocean_evolve_params, buf_h0, buf_ocean_spec);
	uset_fft_spec_to_a = make_uset3(shader_fft, buf_fft_params, buf_ocean_spec, buf_fft_a);
	uset_fft_atob = make_uset3(shader_fft, buf_fft_params, buf_fft_a, buf_fft_b);
	uset_fft_btoa = make_uset3(shader_fft, buf_fft_params, buf_fft_b, buf_fft_a);

	// Blit-to-texture: real RD textures, sampled directly by the scene
	// renderer via a caller-side Texture2DRD -- see the header for why.
	RD::TextureFormat ripple_tf;
	ripple_tf.format = RD::DATA_FORMAT_R32_SFLOAT;
	ripple_tf.width = n;
	ripple_tf.height = n;
	ripple_tf.usage_bits = RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT | RD::TEXTURE_USAGE_CAN_UPDATE_BIT;
	tex_ripple_height = rd->texture_create(ripple_tf, RD::TextureView());

	RD::TextureFormat ocean_tf = ripple_tf;
	ocean_tf.width = on;
	ocean_tf.height = on;
	tex_ocean_height = rd->texture_create(ocean_tf, RD::TextureView());

	buf_blit_ripple_params = rd->uniform_buffer_create(BLIT_PARAMS_BYTES);
	buf_blit_ocean_params = rd->uniform_buffer_create(BLIT_PARAMS_BYTES);
	{
		int32_t v[4] = { n, 0, 0, 0 };
		PackedByteArray b;
		b.resize(BLIT_PARAMS_BYTES);
		memcpy(b.ptrw(), v, sizeof(v));
		rd->buffer_update(buf_blit_ripple_params, 0, BLIT_PARAMS_BYTES, b.ptr());
	}
	{
		int32_t v[4] = { on, 0, 0, 0 };
		PackedByteArray b;
		b.resize(BLIT_PARAMS_BYTES);
		memcpy(b.ptrw(), v, sizeof(v));
		rd->buffer_update(buf_blit_ocean_params, 0, BLIT_PARAMS_BYTES, b.ptr());
	}
	auto make_blit_uset = [&](RID p_shader, RID p_params, RID p_in_buf, RID p_out_tex) {
		Vector<RD::Uniform> uniforms;
		RD::Uniform u0;
		u0.uniform_type = RD::UNIFORM_TYPE_UNIFORM_BUFFER;
		u0.binding = 0;
		u0.append_id(p_params);
		uniforms.push_back(u0);
		RD::Uniform u1;
		u1.uniform_type = RD::UNIFORM_TYPE_STORAGE_BUFFER;
		u1.binding = 1;
		u1.append_id(p_in_buf);
		uniforms.push_back(u1);
		RD::Uniform u2;
		u2.uniform_type = RD::UNIFORM_TYPE_IMAGE;
		u2.binding = 2;
		u2.append_id(p_out_tex);
		uniforms.push_back(u2);
		return rd->uniform_set_create(uniforms, p_shader, 0);
	};
	uset_blit_ripple = make_blit_uset(shader_blit_ripple, buf_blit_ripple_params, buf_height, tex_ripple_height);
	uset_blit_ocean = make_blit_uset(shader_blit_ocean, buf_blit_ocean_params, buf_fft_b, tex_ocean_height);

	// h0 depends only on static settings (wind/domain/amplitude) -- generate
	// it once here, not every step.
	{
		Vector<uint8_t> ib;
		ib.resize(OCEAN_INIT_PARAMS_BYTES);
		uint8_t *iw = ib.ptrw();
		auto put_f = [&](int off, float v) { memcpy(iw + off, &v, sizeof(float)); };
		auto put_i = [&](int off, int32_t v) { memcpy(iw + off, &v, sizeof(int32_t)); };
		put_f(0, ocean_domain_size.x);
		put_f(4, ocean_domain_size.y);
		put_f(8, _init_wind_speed);
		put_f(12, _init_wind_direction.x);
		put_f(16, _init_wind_direction.y);
		put_f(20, _init_gravity);
		put_f(24, _init_wave_amplitude);
		put_f(28, 0.0f);
		put_i(32, on);
		put_i(36, 0);
		put_i(40, 0);
		put_i(44, 0);
		rd->buffer_update(buf_ocean_init_params, 0, OCEAN_INIT_PARAMS_BYTES, ib.ptr());

		RD::ComputeListID cl = rd->compute_list_begin();
		rd->compute_list_bind_compute_pipeline(cl, pipeline_spectrum_init);
		rd->compute_list_bind_uniform_set(cl, uset_spectrum_init, 0);
		rd->compute_list_dispatch(cl, groups_for(on), groups_for(on), 1);
		rd->compute_list_end();
		if (local) {
			rd->submit();
			rd->sync();
		}
		// Shared device: this one-time dispatch just gets recorded and rides
		// along with whatever the render thread submits next (the engine's
		// own per-frame flush) -- h0 only needs to be ready before the first
		// rt_step() runs, and rt_build()/rt_step() are always posted to the
		// render thread in that order, so ordering is already guaranteed.
	}
}

void WaterSolverGPU::rt_build(Ref<WaterSolverGPU> p_self, int p_grid_resolution, Vector2 p_domain_size, int p_ocean_grid_resolution, Vector2 p_ocean_domain_size, float p_wind_speed, Vector2 p_wind_direction, float p_wave_amplitude, float p_gravity) {
	if (rd == nullptr || !shaders_ok) {
		return;
	}
	built.set_to(false);
	_rt_free_buffers();
	grid_resolution = p_grid_resolution;
	domain_size = p_domain_size;
	ocean_grid_resolution = p_ocean_grid_resolution;
	ocean_domain_size = p_ocean_domain_size;
	_init_wind_speed = p_wind_speed;
	_init_wind_direction = p_wind_direction;
	_init_wave_amplitude = p_wave_amplitude;
	_init_gravity = p_gravity;
	_rt_build_buffers();
	built.set_to(true);
}

void WaterSolverGPU::rt_step(Ref<WaterSolverGPU> p_self, double p_delta, float p_depth, float p_damping, float p_gravity, float p_water_level, PackedFloat32Array p_spheres, PackedFloat32Array p_impulses) {
	if (rd == nullptr || !shaders_ok || buf_params.is_null()) {
		return;
	}
	const int n = grid_resolution;
	const int on = ocean_grid_resolution;

	// Ripple layer params + body data.
	{
		Vector<uint8_t> b;
		b.resize(PARAMS_BYTES);
		uint8_t *w = b.ptrw();
		auto put_f = [&](int off, float v) { memcpy(w + off, &v, sizeof(float)); };
		auto put_i = [&](int off, int32_t v) { memcpy(w + off, &v, sizeof(int32_t)); };
		put_f(0, domain_size.x * 0.5f);
		put_f(4, domain_size.y * 0.5f);
		put_f(8, p_depth);
		put_f(12, (float)p_delta);
		put_f(16, p_damping);
		put_f(20, p_gravity);
		put_f(24, domain_size.x / (float)n);
		put_f(28, p_water_level);
		put_i(32, n);
		put_i(36, p_spheres.size() / 4);
		put_i(40, p_impulses.size() / 4);
		put_i(44, 0);
		rd->buffer_update(buf_params, 0, PARAMS_BYTES, b.ptr());
	}
	{
		Vector<uint8_t> sb;
		sb.resize(MAX_SPHERES * 4 * sizeof(float));
		memset(sb.ptrw(), 0, sb.size());
		memcpy(sb.ptrw(), p_spheres.ptr(), MIN((size_t)p_spheres.size() * sizeof(float), (size_t)sb.size()));
		rd->buffer_update(buf_spheres, 0, sb.size(), sb.ptr());
	}
	{
		Vector<uint8_t> ib;
		ib.resize(MAX_IMPULSES * 4 * sizeof(float));
		memset(ib.ptrw(), 0, ib.size());
		memcpy(ib.ptrw(), p_impulses.ptr(), MIN((size_t)p_impulses.size() * sizeof(float), (size_t)ib.size()));
		rd->buffer_update(buf_impulses, 0, ib.size(), ib.ptr());
	}
	{
		RD::ComputeListID cl = rd->compute_list_begin();
		rd->compute_list_bind_compute_pipeline(cl, pipeline_ripple);
		rd->compute_list_bind_uniform_set(cl, a_is_current ? uset_atob : uset_btoa, 0);
		rd->compute_list_dispatch(cl, groups_for(n), groups_for(n), 1);
		rd->compute_list_end();
	}
	a_is_current = !a_is_current;

	// FFT ocean layer: evolve, then the Stockham butterfly passes.
	ocean_time_accum += p_delta;
	{
		Vector<uint8_t> eb;
		eb.resize(OCEAN_EVOLVE_PARAMS_BYTES);
		uint8_t *w = eb.ptrw();
		auto put_f = [&](int off, float v) { memcpy(w + off, &v, sizeof(float)); };
		auto put_i = [&](int off, int32_t v) { memcpy(w + off, &v, sizeof(int32_t)); };
		put_f(0, ocean_domain_size.x);
		put_f(4, ocean_domain_size.y);
		put_f(8, p_depth);
		put_f(12, (float)ocean_time_accum);
		put_i(16, on);
		put_i(20, 0);
		put_i(24, 0);
		put_i(28, 0);
		rd->buffer_update(buf_ocean_evolve_params, 0, OCEAN_EVOLVE_PARAMS_BYTES, eb.ptr());
	}
	{
		RD::ComputeListID cl = rd->compute_list_begin();
		rd->compute_list_bind_compute_pipeline(cl, pipeline_spectrum_evolve);
		rd->compute_list_bind_uniform_set(cl, uset_spectrum_evolve, 0);
		rd->compute_list_dispatch(cl, groups_for(on), groups_for(on), 1);
		rd->compute_list_end();
	}

	bool first = true;
	bool cur_is_a = false;
	for (int dir = 0; dir < 2; dir++) {
		for (int p = 0; p < ocean_log2n; p++) {
			const int ns = 1 << (p + 1);
			Vector<uint8_t> fb;
			fb.resize(FFT_PARAMS_BYTES);
			int32_t vals[4] = { on, ns, dir == 0 ? 1 : 0, 0 };
			memcpy(fb.ptrw(), vals, sizeof(vals));
			rd->buffer_update(buf_fft_params, 0, FFT_PARAMS_BYTES, fb.ptr());

			RID uset;
			if (first) {
				uset = uset_fft_spec_to_a;
				first = false;
				cur_is_a = true;
			} else {
				uset = cur_is_a ? uset_fft_atob : uset_fft_btoa;
				cur_is_a = !cur_is_a;
			}
			RD::ComputeListID cl = rd->compute_list_begin();
			rd->compute_list_bind_compute_pipeline(cl, pipeline_fft);
			rd->compute_list_bind_uniform_set(cl, uset, 0);
			rd->compute_list_dispatch(cl, groups_for(on), groups_for(on), 1);
			rd->compute_list_end();
		}
	}
	// Always lands in fft_b -- 2*log2(N) passes is always even (see the
	// header's note).

	// Blit both layers into their real RD textures for zero-copy rendering.
	{
		RD::ComputeListID cl = rd->compute_list_begin();
		rd->compute_list_bind_compute_pipeline(cl, pipeline_blit_ripple);
		rd->compute_list_bind_uniform_set(cl, uset_blit_ripple, 0);
		rd->compute_list_dispatch(cl, groups_for(n), groups_for(n), 1);
		rd->compute_list_end();
	}
	{
		RD::ComputeListID cl = rd->compute_list_begin();
		rd->compute_list_bind_compute_pipeline(cl, pipeline_blit_ocean);
		rd->compute_list_bind_uniform_set(cl, uset_blit_ocean, 0);
		rd->compute_list_dispatch(cl, groups_for(on), groups_for(on), 1);
		rd->compute_list_end();
	}

	if (local) {
		rd->submit();
		rd->sync();
		MutexLock lock(cache_mtx);
		Vector<uint8_t> rraw = rd->buffer_get_data(buf_height, 0, n * n * sizeof(float));
		ripple_height_cache.resize(n * n);
		if (rraw.size() > 0) {
			memcpy(ripple_height_cache.ptrw(), rraw.ptr(), MIN((size_t)rraw.size(), (size_t)(n * n * sizeof(float))));
		}
		Vector<uint8_t> oraw = rd->buffer_get_data(buf_fft_b, 0, on * on * 2 * sizeof(float));
		ocean_height_cache.resize(on * on);
		ocean_imag_cache.resize(on * on);
		if (oraw.size() > 0) {
			const float *od = (const float *)oraw.ptr();
			float *hw = ocean_height_cache.ptrw();
			float *iw = ocean_imag_cache.ptrw();
			for (int i = 0; i < on * on; i++) {
				hw[i] = od[i * 2 + 0];
				iw[i] = od[i * 2 + 1];
			}
		}
		height_ready.set();
		return;
	}

	// Shared device: async readback, two independent reads (ripple, ocean) --
	// lands a few frames later on the render thread and refreshes the
	// caches; each callback binds a Ref to us so we survive the wait, same
	// convention as every other async readback in this module.
	rd->buffer_get_data_async(buf_height, callable_mp(this, &WaterSolverGPU::rt_on_ripple_height).bind(p_self), 0, n * n * sizeof(float));
	rd->buffer_get_data_async(buf_fft_b, callable_mp(this, &WaterSolverGPU::rt_on_ocean_height).bind(p_self), 0, on * on * 2 * sizeof(float));
}

void WaterSolverGPU::rt_on_ripple_height(const PackedByteArray &p_data, Ref<WaterSolverGPU> p_self) {
	MutexLock lock(cache_mtx);
	const int n = grid_resolution;
	ripple_height_cache.resize(n * n);
	if (p_data.size() > 0) {
		memcpy(ripple_height_cache.ptrw(), p_data.ptr(), MIN((size_t)p_data.size(), (size_t)(n * n * sizeof(float))));
	}
	height_ready.set();
}

void WaterSolverGPU::rt_on_ocean_height(const PackedByteArray &p_data, Ref<WaterSolverGPU> p_self) {
	MutexLock lock(cache_mtx);
	const int on = ocean_grid_resolution;
	ocean_height_cache.resize(on * on);
	ocean_imag_cache.resize(on * on);
	if (p_data.size() > 0) {
		const float *od = (const float *)p_data.ptr();
		const int count = MIN((int)(p_data.size() / (2 * (int)sizeof(float))), on * on);
		float *hw = ocean_height_cache.ptrw();
		float *iw = ocean_imag_cache.ptrw();
		for (int i = 0; i < count; i++) {
			hw[i] = od[i * 2 + 0];
			iw[i] = od[i * 2 + 1];
		}
	}
}

void WaterSolverGPU::rt_free(Ref<WaterSolverGPU> p_self) {
	if (rd == nullptr) {
		return;
	}
	if (local) {
		rd->sync();
	}
	_rt_free_buffers();
	RID *shaders[] = { &shader_ripple, &shader_spectrum_init, &shader_spectrum_evolve, &shader_fft, &shader_blit_ripple, &shader_blit_ocean };
	for (RID *s : shaders) {
		if (s->is_valid()) {
			rd->free_rid(*s);
			*s = RID();
		}
	}
	shaders_ok = false;
}

/* ===================================================================== */
/*  WaterSolver -- CPU-side front. Preps payloads, posts render work.     */
/* ===================================================================== */

WaterSolver::WaterSolver() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	gpu.instantiate();
	gpu->rd = rs->get_rendering_device();
	if (gpu->rd == nullptr) {
		// Headless: no main device. Spin up a private local one and drive it
		// synchronously on this thread -- confirmed empirically this session
		// that this ALSO returns null under --headless, so this path only
		// ever actually engages for a real windowed run without a main
		// device for some other reason; matches MPM's own fallback shape.
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
	_dispatch(callable_mp(gpu.ptr(), &WaterSolverGPU::rt_compile).bind(gpu));
}

WaterSolver::~WaterSolver() {
	if (gpu.is_valid() && gpu->rd != nullptr) {
		if (gpu->local) {
			gpu->rt_free(gpu);
			memdelete(gpu->rd); // we own the local device
			gpu->rd = nullptr;
		} else {
			RenderingServer *rs = RenderingServer::get_singleton();
			rs->call_on_render_thread(callable_mp(gpu.ptr(), &WaterSolverGPU::rt_free).bind(gpu));
			rs->sync(); // wait for the render thread to run rt_free before we drop our Ref
		}
	}
	gpu.unref();
}

void WaterSolver::_dispatch(const Callable &p_call) const {
	if (gpu.is_null()) {
		return;
	}
	if (gpu->local) {
		p_call.call();
	} else {
		RenderingServer::get_singleton()->call_on_render_thread(p_call);
	}
}

void WaterSolver::configure(const Settings &p_settings) {
	settings = p_settings;
	for (SphereSlot &s : sphere_slots) {
		s = SphereSlot();
	}
	pending_impulses.clear();
	_dispatch(callable_mp(gpu.ptr(), &WaterSolverGPU::rt_build).bind(gpu, settings.grid_resolution, settings.domain_size, settings.ocean_grid_resolution, settings.ocean_domain_size, settings.wind_speed, settings.wind_direction, settings.wave_amplitude, settings.gravity));
}

PackedFloat32Array WaterSolver::_pack_spheres() const {
	PackedFloat32Array out;
	for (const SphereSlot &s : sphere_slots) {
		if (!s.enabled) {
			continue;
		}
		out.push_back(s.world_pos.x);
		out.push_back(s.world_pos.z);
		out.push_back(s.radius);
		out.push_back(s.strength);
	}
	return out;
}

PackedFloat32Array WaterSolver::_pack_impulses() const {
	PackedFloat32Array out;
	for (const ImpulseSlot &imp : pending_impulses) {
		out.push_back(imp.world_pos.x);
		out.push_back(imp.world_pos.z);
		out.push_back(imp.radius);
		out.push_back(imp.strength);
	}
	return out;
}

void WaterSolver::step(double p_delta) {
	if (!is_available()) {
		return;
	}
	_dispatch(callable_mp(gpu.ptr(), &WaterSolverGPU::rt_step).bind(gpu, p_delta, settings.depth, settings.damping, settings.gravity, settings.water_level, _pack_spheres(), _pack_impulses()));
	// One-shot: consumed by exactly one step, then cleared. Persistent
	// sphere proxies stay until explicitly cleared/overwritten by the caller.
	pending_impulses.clear();
}

void WaterSolver::submit_sphere(uint64_t p_owner, const Vector3 &p_world_pos, float p_radius, float p_strength) {
	int free_slot = -1;
	for (int i = 0; i < MAX_SPHERES; i++) {
		if (sphere_slots[i].enabled && sphere_slots[i].owner == p_owner) {
			sphere_slots[i].world_pos = p_world_pos;
			sphere_slots[i].radius = p_radius;
			sphere_slots[i].strength = p_strength;
			return;
		}
		if (free_slot < 0 && !sphere_slots[i].enabled) {
			free_slot = i;
		}
	}
	if (free_slot < 0) {
		return; // MAX_SPHERES exceeded -- silently drop, matches the module's other MAX_* slot conventions
	}
	sphere_slots[free_slot].owner = p_owner;
	sphere_slots[free_slot].world_pos = p_world_pos;
	sphere_slots[free_slot].radius = p_radius;
	sphere_slots[free_slot].strength = p_strength;
	sphere_slots[free_slot].enabled = true;
}

void WaterSolver::clear_sphere(uint64_t p_owner) {
	for (SphereSlot &s : sphere_slots) {
		if (s.enabled && s.owner == p_owner) {
			s = SphereSlot();
			return;
		}
	}
}

void WaterSolver::submit_impulse(const Vector3 &p_world_pos, float p_radius, float p_strength) {
	if (pending_impulses.size() >= MAX_IMPULSES) {
		return;
	}
	ImpulseSlot imp;
	imp.world_pos = p_world_pos;
	imp.radius = p_radius;
	imp.strength = p_strength;
	pending_impulses.push_back(imp);
}

RID WaterSolver::get_ripple_height_texture_rd_rid() const {
	if (gpu.is_null()) {
		return RID();
	}
	return gpu->tex_ripple_height;
}

RID WaterSolver::get_ocean_height_texture_rd_rid() const {
	if (gpu.is_null()) {
		return RID();
	}
	return gpu->tex_ocean_height;
}

void WaterSolver::get_height_grid(Vector<float> &r_height, int &r_n, Vector2 &r_domain_size) const {
	r_height.clear();
	r_n = settings.grid_resolution;
	r_domain_size = settings.domain_size;
	if (gpu.is_null()) {
		return;
	}
	MutexLock lock(gpu->cache_mtx);
	r_height = gpu->ripple_height_cache;
}

void WaterSolver::get_ocean_height_grid(Vector<float> &r_height, Vector<float> &r_imag, int &r_n, Vector2 &r_domain_size) const {
	r_height.clear();
	r_imag.clear();
	r_n = settings.ocean_grid_resolution;
	r_domain_size = settings.ocean_domain_size;
	if (gpu.is_null()) {
		return;
	}
	MutexLock lock(gpu->cache_mtx);
	r_height = gpu->ocean_height_cache;
	r_imag = gpu->ocean_imag_cache;
}

void WaterSolver::sync_now() const {
	if (gpu.is_valid() && !gpu->local) {
		RenderingServer::get_singleton()->sync();
	}
}

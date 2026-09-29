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

#include "water_fft.glsl.gen.h"
#include "water_ripple.glsl.gen.h"
#include "water_spectrum_evolve.glsl.gen.h"
#include "water_spectrum_init.glsl.gen.h"

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

WaterSolver::WaterSolver() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	// Always a private local device -- same simplification GasSolver already
	// makes over MPM fluid's shared-device/render-thread/async-readback
	// machinery. Confirmed empirically this session: this returns null under
	// --headless just like the shared device does, so there is no headless
	// path for this solver (or any RenderingDevice compute in this engine
	// build) -- real behavior can only be tested windowed.
	if (!DisplayServer::can_create_rendering_device()) {
		return;
	}
	rd = rs->create_local_rendering_device();
	if (rd == nullptr) {
		return;
	}
	_compile_shaders();
}

WaterSolver::~WaterSolver() {
	if (rd == nullptr) {
		return;
	}
	_free_buffers();
	RID *shaders[] = { &shader_ripple, &shader_spectrum_init, &shader_spectrum_evolve, &shader_fft };
	for (RID *s : shaders) {
		if (s->is_valid()) {
			rd->free_rid(*s);
		}
	}
	memdelete(rd); // we own this local device
	rd = nullptr;
}

void WaterSolver::_compile_shaders() {
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

void WaterSolver::_free_buffers() {
	RID *usets[] = { &uset_atob, &uset_btoa, &uset_spectrum_init, &uset_spectrum_evolve, &uset_fft_spec_to_a, &uset_fft_atob, &uset_fft_btoa };
	for (RID *u : usets) {
		if (u->is_valid()) {
			rd->free_rid(*u);
			*u = RID();
		}
	}
	RID *bufs[] = { &buf_params, &buf_state_a, &buf_state_b, &buf_height, &buf_spheres, &buf_impulses,
		&buf_ocean_init_params, &buf_ocean_evolve_params, &buf_fft_params, &buf_h0, &buf_ocean_spec, &buf_fft_a, &buf_fft_b };
	for (RID *b : bufs) {
		if (b->is_valid()) {
			rd->free_rid(*b);
			*b = RID();
		}
	}
}

void WaterSolver::_build_buffers() {
	const int n = settings.grid_resolution;
	const int cells = n * n;

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

	// FFT ocean layer (Phase 2).
	const int on = settings.ocean_grid_resolution;
	const int ocells = on * on;
	ocean_log2n = log2i(on);
	ocean_time_accum = 0.0;

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

	// h0 depends only on static settings (wind/domain/amplitude) -- generate
	// it once here, not every step.
	{
		Vector<uint8_t> ib;
		ib.resize(OCEAN_INIT_PARAMS_BYTES);
		uint8_t *iw = ib.ptrw();
		auto put_f = [&](int off, float v) { memcpy(iw + off, &v, sizeof(float)); };
		auto put_i = [&](int off, int32_t v) { memcpy(iw + off, &v, sizeof(int32_t)); };
		put_f(0, settings.ocean_domain_size.x);
		put_f(4, settings.ocean_domain_size.y);
		put_f(8, settings.wind_speed);
		put_f(12, settings.wind_direction.x);
		put_f(16, settings.wind_direction.y);
		put_f(20, settings.gravity);
		put_f(24, settings.wave_amplitude);
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
		rd->submit();
		rd->sync();
	}
}

void WaterSolver::_upload_params(double p_dt) {
	Vector<uint8_t> b;
	b.resize(PARAMS_BYTES);
	uint8_t *w = b.ptrw();
	auto put_f = [&](int off, float v) { memcpy(w + off, &v, sizeof(float)); };
	auto put_i = [&](int off, int32_t v) { memcpy(w + off, &v, sizeof(int32_t)); };

	put_f(0, settings.domain_size.x * 0.5f);
	put_f(4, settings.domain_size.y * 0.5f);
	put_f(8, settings.depth);
	put_f(12, (float)p_dt);

	put_f(16, settings.damping);
	put_f(20, settings.gravity);
	put_f(24, settings.domain_size.x / (float)settings.grid_resolution);
	put_f(28, settings.water_level);

	int num_spheres = 0;
	for (const SphereSlot &s : sphere_slots) {
		if (s.enabled) {
			num_spheres++;
		}
	}
	put_i(32, settings.grid_resolution);
	put_i(36, num_spheres);
	put_i(40, pending_impulses.size());
	put_i(44, 0);

	rd->buffer_update(buf_params, 0, PARAMS_BYTES, b.ptr());
}

void WaterSolver::_upload_bodies() {
	Vector<uint8_t> spheres_data;
	spheres_data.resize(MAX_SPHERES * 4 * sizeof(float));
	memset(spheres_data.ptrw(), 0, spheres_data.size());
	{
		float *w = (float *)spheres_data.ptrw();
		int slot = 0;
		for (const SphereSlot &s : sphere_slots) {
			if (!s.enabled) {
				continue;
			}
			w[slot * 4 + 0] = s.world_pos.x;
			w[slot * 4 + 1] = s.world_pos.z;
			w[slot * 4 + 2] = s.radius;
			w[slot * 4 + 3] = s.strength;
			slot++;
		}
	}
	rd->buffer_update(buf_spheres, 0, spheres_data.size(), spheres_data.ptr());

	Vector<uint8_t> impulses_data;
	impulses_data.resize(MAX_IMPULSES * 4 * sizeof(float));
	memset(impulses_data.ptrw(), 0, impulses_data.size());
	{
		float *w = (float *)impulses_data.ptrw();
		for (int i = 0; i < pending_impulses.size() && i < MAX_IMPULSES; i++) {
			const ImpulseSlot &imp = pending_impulses[i];
			w[i * 4 + 0] = imp.world_pos.x;
			w[i * 4 + 1] = imp.world_pos.z;
			w[i * 4 + 2] = imp.radius;
			w[i * 4 + 3] = imp.strength;
		}
	}
	rd->buffer_update(buf_impulses, 0, impulses_data.size(), impulses_data.ptr());
}

void WaterSolver::configure(const Settings &p_settings) {
	if (rd == nullptr || !shaders_ok) {
		return;
	}
	_free_buffers();
	settings = p_settings;
	a_is_current = true;
	for (SphereSlot &s : sphere_slots) {
		s = SphereSlot();
	}
	pending_impulses.clear();
	_build_buffers();
}

void WaterSolver::_dispatch(RID p_pipeline, RID p_uset, int p_groups_x, int p_groups_y) const {
	RD::ComputeListID cl = rd->compute_list_begin();
	rd->compute_list_bind_compute_pipeline(cl, p_pipeline);
	rd->compute_list_bind_uniform_set(cl, p_uset, 0);
	rd->compute_list_dispatch(cl, p_groups_x, p_groups_y, 1);
	rd->compute_list_end();
	rd->submit();
	rd->sync();
}

void WaterSolver::_step_ocean(double p_delta) {
	ocean_time_accum += p_delta;
	const int on = settings.ocean_grid_resolution;
	const uint32_t og = groups_for(on);

	{
		Vector<uint8_t> eb;
		eb.resize(OCEAN_EVOLVE_PARAMS_BYTES);
		uint8_t *w = eb.ptrw();
		auto put_f = [&](int off, float v) { memcpy(w + off, &v, sizeof(float)); };
		auto put_i = [&](int off, int32_t v) { memcpy(w + off, &v, sizeof(int32_t)); };
		put_f(0, settings.ocean_domain_size.x);
		put_f(4, settings.ocean_domain_size.y);
		put_f(8, settings.depth);
		put_f(12, (float)ocean_time_accum);
		put_i(16, on);
		put_i(20, 0);
		put_i(24, 0);
		put_i(28, 0);
		rd->buffer_update(buf_ocean_evolve_params, 0, OCEAN_EVOLVE_PARAMS_BYTES, eb.ptr());
	}
	_dispatch(pipeline_spectrum_evolve, uset_spectrum_evolve, og, og);

	// Stockham butterfly: log2(N) horizontal passes (ns: 2,4,...,N), reading
	// buf_ocean_spec on the very first pass only, then log2(N) vertical
	// passes, ping-ponging fft_a/fft_b throughout. Always lands in fft_b --
	// see the header's note on why (2*log2(N) passes is always even).
	bool first = true;
	bool cur_is_a = false; // "current" = the buffer the NEXT pass should read from; irrelevant until first==false
	for (int dir = 0; dir < 2; dir++) {
		for (int p = 0; p < ocean_log2n; p++) {
			const int ns = 1 << (p + 1);
			Vector<uint8_t> fb;
			fb.resize(FFT_PARAMS_BYTES);
			uint8_t *w = fb.ptrw();
			int32_t vals[4] = { on, ns, dir == 0 ? 1 : 0, 0 };
			memcpy(w, vals, sizeof(vals));
			rd->buffer_update(buf_fft_params, 0, FFT_PARAMS_BYTES, fb.ptr());

			RID uset;
			if (first) {
				uset = uset_fft_spec_to_a;
				first = false;
				cur_is_a = true; // result now in fft_a
			} else {
				uset = cur_is_a ? uset_fft_atob : uset_fft_btoa;
				cur_is_a = !cur_is_a;
			}
			_dispatch(pipeline_fft, uset, og, og);
		}
	}
}

void WaterSolver::step(double p_delta) {
	if (rd == nullptr || !shaders_ok || buf_params.is_null()) {
		return;
	}
	_upload_params(p_delta);
	_upload_bodies();
	_dispatch(pipeline_ripple, a_is_current ? uset_atob : uset_btoa, groups_for(settings.grid_resolution), groups_for(settings.grid_resolution));
	a_is_current = !a_is_current;
	// One-shot: consumed by exactly one step, then cleared. Persistent sphere
	// proxies stay until explicitly cleared/overwritten by the caller.
	pending_impulses.clear();

	_step_ocean(p_delta);
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

void WaterSolver::get_height_grid(Vector<float> &r_height, int &r_n, Vector2 &r_domain_size) const {
	r_height.clear();
	r_n = settings.grid_resolution;
	r_domain_size = settings.domain_size;
	if (rd == nullptr || buf_height.is_null()) {
		return;
	}
	const int cells = settings.grid_resolution * settings.grid_resolution;
	Vector<uint8_t> raw = rd->buffer_get_data(buf_height, 0, cells * sizeof(float));
	if (raw.size() < cells * (int)sizeof(float)) {
		return;
	}
	r_height.resize(cells);
	memcpy(r_height.ptrw(), raw.ptr(), cells * sizeof(float));
}

void WaterSolver::get_ocean_height_grid(Vector<float> &r_height, Vector<float> &r_imag, int &r_n, Vector2 &r_domain_size) const {
	r_height.clear();
	r_imag.clear();
	r_n = settings.ocean_grid_resolution;
	r_domain_size = settings.ocean_domain_size;
	if (rd == nullptr || buf_fft_b.is_null()) {
		return;
	}
	const int cells = settings.ocean_grid_resolution * settings.ocean_grid_resolution;
	Vector<uint8_t> raw = rd->buffer_get_data(buf_fft_b, 0, cells * 2 * sizeof(float));
	if (raw.size() < cells * 2 * (int)sizeof(float)) {
		return;
	}
	r_height.resize(cells);
	r_imag.resize(cells);
	const float *data = (const float *)raw.ptr();
	for (int i = 0; i < cells; i++) {
		r_height.set(i, data[i * 2 + 0]);
		r_imag.set(i, data[i * 2 + 1]);
	}
}

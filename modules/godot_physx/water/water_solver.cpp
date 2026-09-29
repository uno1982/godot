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

#include "water_ripple.glsl.gen.h"

#include "core/os/memory.h"
#include "servers/display/display_server.h"
#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"

namespace {
constexpr int PARAMS_BYTES = 48; // 3 * vec4, std140 -- see water_inc.glsl's Params
constexpr uint32_t GROUP = 8; // matches water_ripple.glsl's local_size_x/y

uint32_t groups_for(int p_count) {
	return (uint32_t)((p_count + (int)GROUP - 1) / (int)GROUP);
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
	if (shader_ripple.is_valid()) {
		rd->free_rid(shader_ripple);
	}
	memdelete(rd); // we own this local device
	rd = nullptr;
}

void WaterSolver::_compile_shaders() {
	Ref<RDShaderFile> sf;
	sf.instantiate();
	if (sf->parse_versions_from_text(water_ripple_shader_glsl) != OK) {
		ERR_PRINT("WaterSolver: the ripple compute shader failed to compile.");
		return;
	}
	shader_ripple = rd->shader_create_from_spirv(sf->get_spirv_stages());
	ERR_FAIL_COND(shader_ripple.is_null());
	pipeline_ripple = rd->compute_pipeline_create(shader_ripple);
	ERR_FAIL_COND(pipeline_ripple.is_null());
	shaders_ok = true;
}

void WaterSolver::_free_buffers() {
	RID *usets[] = { &uset_atob, &uset_btoa };
	for (RID *u : usets) {
		if (u->is_valid()) {
			rd->free_rid(*u);
			*u = RID();
		}
	}
	RID *bufs[] = { &buf_params, &buf_state_a, &buf_state_b, &buf_height, &buf_spheres, &buf_impulses };
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

void WaterSolver::step(double p_delta) {
	if (rd == nullptr || !shaders_ok || buf_params.is_null()) {
		return;
	}
	_upload_params(p_delta);
	_upload_bodies();

	RID uset = a_is_current ? uset_atob : uset_btoa;
	RD::ComputeListID cl = rd->compute_list_begin();
	rd->compute_list_bind_compute_pipeline(cl, pipeline_ripple);
	rd->compute_list_bind_uniform_set(cl, uset, 0);
	rd->compute_list_dispatch(cl, groups_for(settings.grid_resolution), groups_for(settings.grid_resolution), 1);
	rd->compute_list_end();
	rd->submit();
	rd->sync();

	a_is_current = !a_is_current;
	// One-shot: consumed by exactly one step, then cleared. Persistent sphere
	// proxies stay until explicitly cleared/overwritten by the caller.
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

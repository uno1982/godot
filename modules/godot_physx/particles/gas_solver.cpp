/**************************************************************************/
/*  gas_solver.cpp                                                        */
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

#include "gas_solver.h"

#include "gas_bs_clear.glsl.gen.h"
#include "gas_bs_clear_range.glsl.gen.h"
#include "gas_bs_curl.glsl.gen.h"
#include "gas_bs_step.glsl.gen.h"
#include "gas_bs_touch.glsl.gen.h"

#include "core/math/math_funcs.h"
#include "core/os/os.h"
#include "servers/display/display_server.h"
#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"

namespace {
constexpr int BLK = 4;
constexpr int BCELLS = 64;
constexpr int GROUP = 64;
constexpr uint32_t PARAMS_BYTES = 96 + GasSolver::MAX_COLLIDERS * 16 + 16; // 6 * vec4 + colliders[] + growth_range, std140 -- see gas_block_inc.glsl's Params
constexpr float GROWTH_BLOCKS_Y = 2.0f; // blocks added to the top of the box per growth pass
constexpr int GROWTH_EVERY_STEPS = 90; // ~1.5s at 60Hz -- growth is cheap but not free (extra touch dispatch + readback)

uint32_t groups_for(int count) {
	return (uint32_t)((count + GROUP - 1) / GROUP);
}
} //namespace

GasSolver::GasSolver() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	// Always a private local device for v1 -- see the header for why (no
	// render-thread/async-readback machinery yet, unlike the MPM fluid
	// solver). This is the same fallback MPM itself uses when headless.
	if (!DisplayServer::can_create_rendering_device()) {
		return;
	}
	rd = rs->create_local_rendering_device();
	if (rd == nullptr) {
		return;
	}
	_compile_shaders();
}

GasSolver::~GasSolver() {
	if (rd == nullptr) {
		return;
	}
	_free_buffers();
	RID *shaders[] = { &shader_touch, &shader_clear, &shader_clear_range, &shader_curl, &shader_step };
	for (RID *s : shaders) {
		if (s->is_valid()) {
			rd->free_rid(*s);
		}
	}
	memdelete(rd); // we own this local device
	rd = nullptr;
}

void GasSolver::_compile_shaders() {
	struct Entry {
		const char *src;
		RID *shader;
		RID *pipeline;
	};
	Entry entries[] = {
		{ gas_bs_touch_shader_glsl, &shader_touch, &pipeline_touch },
		{ gas_bs_clear_shader_glsl, &shader_clear, &pipeline_clear },
		{ gas_bs_clear_range_shader_glsl, &shader_clear_range, &pipeline_clear_range },
		{ gas_bs_curl_shader_glsl, &shader_curl, &pipeline_curl },
		{ gas_bs_step_shader_glsl, &shader_step, &pipeline_step },
	};
	for (Entry &e : entries) {
		Ref<RDShaderFile> sf;
		sf.instantiate();
		if (sf->parse_versions_from_text(e.src) != OK) {
			ERR_PRINT("GasSolver: a compute shader failed to compile.");
			return;
		}
		*e.shader = rd->shader_create_from_spirv(sf->get_spirv_stages());
		ERR_FAIL_COND(e.shader->is_null());
		*e.pipeline = rd->compute_pipeline_create(*e.shader);
		ERR_FAIL_COND(e.pipeline->is_null());
	}
	shaders_ok = true;
}

void GasSolver::_free_buffers() {
	RID *usets[] = { &uset_touch, &uset_clear, &uset_curl_a, &uset_curl_b, &uset_step_atob, &uset_step_btoa };
	for (RID *u : usets) {
		if (u->is_valid()) {
			rd->free_rid(*u);
			*u = RID();
		}
	}
	RID *bufs[] = { &buf_params, &buf_bhash, &buf_bhash_val, &buf_bkey, &buf_bcounts, &buf_grid_a, &buf_grid_b, &buf_curl };
	for (RID *b : bufs) {
		if (b->is_valid()) {
			rd->free_rid(*b);
			*b = RID();
		}
	}
}

RID GasSolver::_make_uniform_set(RID p_shader, RID p_grid5, RID p_grid6) const {
	const RID by_binding[8] = { buf_params, buf_bhash, buf_bhash_val, buf_bkey, buf_bcounts, p_grid5, p_grid6, buf_curl };
	Vector<RD::Uniform> uniforms;
	for (int bnd = 0; bnd < 8; bnd++) {
		RD::Uniform u;
		u.uniform_type = (bnd == 0) ? RD::UNIFORM_TYPE_UNIFORM_BUFFER : RD::UNIFORM_TYPE_STORAGE_BUFFER;
		u.binding = bnd;
		u.append_id(by_binding[bnd]);
		uniforms.push_back(u);
	}
	RID uset = rd->uniform_set_create(uniforms, p_shader, 0);
	ERR_FAIL_COND_V(uset.is_null(), RID());
	return uset;
}

void GasSolver::_build_buffers() {
	const Vector3i box_cells = settings.box_blocks * BLK;
	block_count = settings.box_blocks.x * settings.box_blocks.y * settings.box_blocks.z;
	cell_count = box_cells.x * box_cells.y * box_cells.z;
	// Reserve headroom for vertical growth (see _grow_if_needed) so it never
	// needs to reallocate a buffer -- just claim more of an already-sized
	// pool. 3x the initial block count comfortably covers tripling the box's
	// height; block_touch degrades gracefully (stops inserting) if a plume
	// somehow needs more than that.
	max_blocks = block_count * 3 + 16;
	hash_slots = 1;
	while (hash_slots < max_blocks * 2) {
		hash_slots *= 2;
	}

	buf_params = rd->uniform_buffer_create(PARAMS_BYTES);

	Vector<uint8_t> hash_init;
	hash_init.resize(hash_slots * (int)sizeof(uint32_t));
	{
		uint32_t *w = (uint32_t *)hash_init.ptrw();
		for (int i = 0; i < hash_slots; i++) {
			w[i] = 0xffffffffu;
		}
	}
	buf_bhash = rd->storage_buffer_create(hash_init.size(), hash_init);
	buf_bhash_val = rd->storage_buffer_create(hash_slots * sizeof(uint32_t));
	buf_bkey = rd->storage_buffer_create(max_blocks * sizeof(uint32_t));
	buf_bcounts = rd->storage_buffer_create(4 * sizeof(uint32_t));
	buf_grid_a = rd->storage_buffer_create(max_blocks * BCELLS * 4 * sizeof(float));
	buf_grid_b = rd->storage_buffer_create(max_blocks * BCELLS * 4 * sizeof(float));
	buf_curl = rd->storage_buffer_create(max_blocks * BCELLS * 4 * sizeof(float));

	uset_touch = _make_uniform_set(shader_touch, buf_grid_a, buf_grid_b);
	uset_clear = _make_uniform_set(shader_clear, buf_grid_a, buf_grid_b);
	uset_curl_a = _make_uniform_set(shader_curl, buf_grid_a, buf_grid_b);
	uset_curl_b = _make_uniform_set(shader_curl, buf_grid_b, buf_grid_a);
	uset_step_atob = _make_uniform_set(shader_step, buf_grid_a, buf_grid_b);
	uset_step_btoa = _make_uniform_set(shader_step, buf_grid_b, buf_grid_a);
}

void GasSolver::_dispatch(RID p_pipeline, RID p_uset, int p_thread_count) const {
	RD::ComputeListID cl = rd->compute_list_begin();
	rd->compute_list_bind_compute_pipeline(cl, p_pipeline);
	rd->compute_list_bind_uniform_set(cl, p_uset, 0);
	rd->compute_list_dispatch(cl, groups_for(p_thread_count), 1, 1);
	rd->compute_list_end();
	rd->submit();
	rd->sync();
}

void GasSolver::_upload_params(double p_dt) {
	Vector<uint8_t> b;
	b.resize(PARAMS_BYTES);
	uint8_t *w = b.ptrw();
	auto put_f = [&](int off, float v) { memcpy(w + off, &v, sizeof(float)); };
	auto put_i = [&](int off, int32_t v) { memcpy(w + off, &v, sizeof(int32_t)); };

	put_f(0, (float)p_dt);
	put_f(4, settings.buoyancy);
	put_f(8, settings.vorticity_strength);
	put_f(12, settings.dissipation);
	put_f(16, grid_anchor.x);
	put_f(20, grid_anchor.y);
	put_f(24, grid_anchor.z);
	put_f(28, settings.cell_size);
	put_i(32, settings.box_blocks.x);
	put_i(36, settings.box_blocks.y);
	put_i(40, settings.box_blocks.z);
	put_i(44, 0);
	const Vector3 source_world = domain_xform.xform(settings.source_position);
	put_f(48, source_world.x);
	put_f(52, source_world.y);
	put_f(56, source_world.z);
	put_f(60, settings.source_radius);
	const Vector3 source_vel_world = domain_xform.basis.xform(settings.source_velocity);
	put_f(64, source_vel_world.x);
	put_f(68, source_vel_world.y);
	put_f(72, source_vel_world.z);
	put_f(76, settings.source_density);
	put_f(80, (float)hash_slots);
	put_f(84, (float)max_blocks);
	put_f(88, (float)time_accum);
	put_f(92, 0.0f);
	for (int i = 0; i < MAX_COLLIDERS; i++) {
		const int off = 96 + i * 16;
		put_f(off, collider_world_positions[i].x);
		put_f(off + 4, collider_world_positions[i].y);
		put_f(off + 8, collider_world_positions[i].z);
		put_f(off + 12, collider_radii[i]);
	}
	const int growth_off = 96 + MAX_COLLIDERS * 16;
	put_i(growth_off, pending_growth_range.x);
	put_i(growth_off + 4, pending_growth_range.y);
	put_i(growth_off + 8, 0);
	put_i(growth_off + 12, 0);

	rd->buffer_update(buf_params, 0, PARAMS_BYTES, b.ptr());
}

void GasSolver::configure(const Settings &p_settings, const Transform3D &p_xform) {
	if (rd == nullptr || !shaders_ok) {
		return;
	}
	_free_buffers();
	settings = p_settings;
	domain_xform = p_xform;
	a_is_current = true;
	time_accum = 0.0;
	steps_since_growth = 0;
	initial_box_blocks_y = settings.box_blocks.y;
	max_box_blocks_y = settings.box_blocks.y * 3; // matches _build_buffers' 3x block-pool headroom
	pending_growth_range = Vector3i();

	const Vector3i box_cells = settings.box_blocks * BLK;
	// Frozen at configure, like the MPM block-sparse fluid's grid_anchor --
	// the box's WORLD FOOTPRINT does not track the transform afterwards (v1
	// scope cut: no lateral reflow, see the header). Only Y grows, and only
	// upward (see _grow_if_needed) -- grid_anchor stays the box's bottom
	// corner regardless, so already-touched cells never change world
	// position when that happens.
	grid_anchor = p_xform.xform(-Vector3(box_cells) * settings.cell_size * 0.5f);

	_build_buffers();
	_upload_params(0.0);
	_dispatch(pipeline_touch, uset_touch, block_count);
	_dispatch(pipeline_clear, uset_clear, cell_count);
	_read_slot_blocks();
}

void GasSolver::_grow_if_needed() {
	if (settings.box_blocks.y >= max_box_blocks_y) {
		return; // reserved pool headroom exhausted -- stop trying
	}
	// Read the current active-block count (bcounts[0]) before growing: any
	// slot claimed by the touch dispatch below is one atomicAdd(bcounts[0])
	// past whatever this was, so [old_count, new_count) is exactly "every
	// block gas_bs_touch just inserted this pass" -- see gas_bs_clear_range.
	Vector<uint8_t> raw = rd->buffer_get_data(buf_bcounts, 0, sizeof(uint32_t));
	const uint32_t old_count = *(const uint32_t *)raw.ptr();

	settings.box_blocks.y += (int)GROWTH_BLOCKS_Y;
	block_count = settings.box_blocks.x * settings.box_blocks.y * settings.box_blocks.z;
	cell_count = (settings.box_blocks * BLK).x * (settings.box_blocks * BLK).y * (settings.box_blocks * BLK).z;
	_upload_params(0.0); // pending_growth_range is (0,0,0) here -- this upload is just the new box size
	_dispatch(pipeline_touch, uset_touch, block_count);

	raw = rd->buffer_get_data(buf_bcounts, 0, sizeof(uint32_t));
	const uint32_t new_count = *(const uint32_t *)raw.ptr();
	if (new_count > old_count) {
		pending_growth_range = Vector3i((int)old_count, (int)new_count, 0);
		_upload_params(0.0); // re-upload with the real range so gas_bs_clear_range sees it
		_dispatch(pipeline_clear_range, uset_clear, ((int)(new_count - old_count)) * BCELLS);
		pending_growth_range = Vector3i();
	}
	_read_slot_blocks();
}

void GasSolver::_read_slot_blocks() {
	Vector<uint8_t> raw = rd->buffer_get_data(buf_bkey, 0, block_count * sizeof(uint32_t));
	slot_block.resize(block_count);
	const uint32_t *keys = (const uint32_t *)raw.ptr();
	for (int slot = 0; slot < block_count; slot++) {
		const uint32_t key = keys[slot];
		slot_block.write[slot] = Vector3i(
				int(key & 1023u) - 512,
				int((key >> 10) & 1023u) - 512,
				int((key >> 20) & 1023u) - 512);
	}
}

void GasSolver::step(double p_delta, const Transform3D &p_xform) {
	if (rd == nullptr || !shaders_ok || buf_params.is_null()) {
		return;
	}
	domain_xform = p_xform;
	time_accum += p_delta;

	steps_since_growth++;
	if (steps_since_growth >= GROWTH_EVERY_STEPS) {
		steps_since_growth = 0;
		_grow_if_needed(); // uploads its own params; harmless that step()'s own upload below follows right after
	}
	_upload_params(p_delta);

	RID curl_uset = a_is_current ? uset_curl_a : uset_curl_b;
	_dispatch(pipeline_curl, curl_uset, cell_count);
	RID step_uset = a_is_current ? uset_step_atob : uset_step_btoa;
	_dispatch(pipeline_step, step_uset, cell_count);
	a_is_current = !a_is_current;
}

void GasSolver::set_colliders(const Vector<Vector3> &p_world_positions, const Vector<float> &p_radii) {
	for (int i = 0; i < MAX_COLLIDERS; i++) {
		if (i < p_world_positions.size() && i < p_radii.size()) {
			collider_world_positions[i] = p_world_positions[i];
			collider_radii[i] = p_radii[i];
		} else {
			collider_radii[i] = 0.0f;
		}
	}
}

void GasSolver::get_render_cells(float p_density_threshold, Vector<Vector3> &r_positions, Vector<float> &r_density) const {
	r_positions.clear();
	r_density.clear();
	if (rd == nullptr || buf_grid_a.is_null()) {
		return;
	}
	RID cur = a_is_current ? buf_grid_a : buf_grid_b;
	Vector<uint8_t> raw = rd->buffer_get_data(cur, 0, max_blocks * BCELLS * 4 * sizeof(float));
	if (raw.size() < max_blocks * BCELLS * 4 * (int)sizeof(float)) {
		return;
	}
	const float *data = (const float *)raw.ptr();

	for (int slot = 0; slot < slot_block.size(); slot++) {
		const Vector3i bc = slot_block[slot];
		for (int lz = 0; lz < BLK; lz++) {
			for (int ly = 0; ly < BLK; ly++) {
				for (int lx = 0; lx < BLK; lx++) {
					const int lin = (lz * BLK + ly) * BLK + lx;
					const int idx = (slot * BCELLS + lin) * 4;
					const float dens = data[idx + 3];
					if (dens < p_density_threshold) {
						continue;
					}
					const Vector3i c(bc.x * BLK + lx, bc.y * BLK + ly, bc.z * BLK + lz);
					const Vector3 wpos = grid_anchor + Vector3(c) * settings.cell_size;
					r_positions.push_back(wpos);
					r_density.push_back(dens);
				}
			}
		}
	}
}

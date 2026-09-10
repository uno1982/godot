/**************************************************************************/
/*  mpm_fluid_solver.cpp                                                  */
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

#include "mpm_fluid_solver.h"

#include "mpm_clear.glsl.gen.h"
#include "mpm_couple.glsl.gen.h"
#include "mpm_g2p.glsl.gen.h"
#include "mpm_grid.glsl.gen.h"
#include "mpm_march.glsl.gen.h"
#include "mpm_p2g_mass.glsl.gen.h"
#include "mpm_p2g_mom.glsl.gen.h"
#include "mpm_render.glsl.gen.h"
#include "mpm_surface.glsl.gen.h"

#include "mpm_bs_binsert.glsl.gen.h"
#include "mpm_bs_bdispatch.glsl.gen.h"
#include "mpm_bs_clear.glsl.gen.h"
#include "mpm_bs_clearnodes.glsl.gen.h"
#include "mpm_bs_couple.glsl.gen.h"
#include "mpm_bs_g2p.glsl.gen.h"
#include "mpm_bs_grid.glsl.gen.h"
#include "mpm_bs_march.glsl.gen.h"
#include "mpm_bs_p2g_mass.glsl.gen.h"
#include "mpm_bs_p2g_mom.glsl.gen.h"
#include "mpm_bs_render.glsl.gen.h"
#include "mpm_bs_surface.glsl.gen.h"

#include "core/io/marshalls.h"
#include "core/math/math_funcs.h"
#include "core/object/callable_mp.h"
#include "core/os/os.h"
#include "servers/display/display_server.h"
#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"

namespace {
constexpr int FLOATS_PER_PARTICLE = 32; // 8 * vec4 (x/v/C x3/F x3)
constexpr int FLOATS_PER_COLLIDER = 16; // 4 * vec4 (c0..c3)
constexpr int MAX_COLLIDERS = 32; // analytic colliders per step (rigid bodies + expanded debris chunks)
constexpr uint32_t PARAMS_BYTES = 160; // 10 * vec4, std140 (dense uses the first 8; block-sparse adds blockp)
constexpr float IMP_FIXED = 1024.0f;
constexpr uint32_t GROUP = 64;
constexpr uint32_t TRI_BUDGET = 200000; // isosurface triangle cap
constexpr int BCELLS = 64; // block-sparse: cells per 4^3 block

uint32_t groups_for(int p_count) {
	return (uint32_t)((p_count + (int)GROUP - 1) / (int)GROUP);
}

uint32_t next_pow2(uint32_t v) {
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	return v + 1;
}
} // namespace

/* ===================================================================== */
/*  MPMFluidSolverGPU -- everything that touches the RenderingDevice.      */
/*  Every method here runs on the render thread (posted via                */
/*  RenderingServer::call_on_render_thread) except the destructor, which   */
/*  runs wherever the last Ref is dropped.                                 */
/* ===================================================================== */

void MPMFluidSolverGPU::rt_compile(Ref<MPMFluidSolverGPU> p_self) {
	if (rd == nullptr) {
		return;
	}
	const char *src[PASS_MAX] = {
		mpm_clear_shader_glsl,
		mpm_p2g_mass_shader_glsl,
		mpm_p2g_mom_shader_glsl,
		mpm_grid_shader_glsl,
		mpm_couple_shader_glsl,
		mpm_g2p_shader_glsl,
		mpm_surface_shader_glsl,
		mpm_march_shader_glsl,
		mpm_render_shader_glsl,
		mpm_bs_clear_shader_glsl,
		mpm_bs_binsert_shader_glsl,
		mpm_bs_bdispatch_shader_glsl,
		mpm_bs_clearnodes_shader_glsl,
		mpm_bs_p2g_mass_shader_glsl,
		mpm_bs_p2g_mom_shader_glsl,
		mpm_bs_grid_shader_glsl,
		mpm_bs_couple_shader_glsl,
		mpm_bs_g2p_shader_glsl,
		mpm_bs_surface_shader_glsl,
		mpm_bs_march_shader_glsl,
		mpm_bs_render_shader_glsl,
	};
	for (int i = 0; i < PASS_MAX; i++) {
		Ref<RDShaderFile> sf;
		sf.instantiate();
		if (sf->parse_versions_from_text(src[i]) != OK) {
			ERR_PRINT(vformat("MPMFluidSolver: compute shader %d failed to compile.", i));
			return;
		}
		shader[i] = rd->shader_create_from_spirv(sf->get_spirv_stages());
		ERR_FAIL_COND(shader[i].is_null());
		pipeline[i] = rd->compute_pipeline_create(shader[i]);
		ERR_FAIL_COND(pipeline[i].is_null());
	}
	_shaders_ok = true;
}

void MPMFluidSolverGPU::_rt_free_buffers() {
	if (uset_mesh.is_valid()) {
		rd->free_rid(uset_mesh);
		uset_mesh = RID();
	}
	// Uniform sets first: RD auto-frees a uniform set when a buffer it references
	// is freed, so freeing the buffers first leaves these RIDs dangling.
	for (int i = 0; i < PASS_MAX; i++) {
		if (uset[i].is_valid()) {
			rd->free_rid(uset[i]);
			uset[i] = RID();
		}
	}
	RID *bufs[] = { &buf_params, &buf_particles, &buf_grid_i, &buf_grid_v, &buf_colliders, &buf_cimp, &buf_mm, &buf_surf, &buf_mverts, &buf_mnorms, &buf_mcount, &buf_bhash, &buf_bhash_val, &buf_bkey, &buf_bcounts };
	for (RID *b : bufs) {
		if (b->is_valid()) {
			rd->free_rid(*b);
			*b = RID();
		}
	}
}

void MPMFluidSolverGPU::_rt_rebuild_uniform_sets() {
	for (int p = 0; p < PASS_MAX; p++) {
		if (uset[p].is_valid()) {
			rd->free_rid(uset[p]);
			uset[p] = RID();
		}
	}

	// Only the passes for this build's path get a uniform set. Dense passes bind
	// mpm_fluid_inc.glsl's 8 bindings; block-sparse passes bind mpm_block_inc's 12.
	const RID by_binding[12] = { buf_params, buf_particles, buf_grid_i, buf_grid_v, buf_colliders, buf_cimp, buf_mm, buf_surf, buf_bhash, buf_bhash_val, buf_bkey, buf_bcounts };
	const int lo = block_sparse ? (int)PASS_BS_CLEAR : (int)PASS_CLEAR;
	const int hi = block_sparse ? (int)PASS_MAX : (int)PASS_BS_CLEAR;
	const int nbind = block_sparse ? 12 : 8;
	for (int p = lo; p < hi; p++) {
		Vector<RD::Uniform> uniforms;
		for (int bnd = 0; bnd < nbind; bnd++) {
			RD::Uniform u;
			u.uniform_type = (bnd == 0) ? RD::UNIFORM_TYPE_UNIFORM_BUFFER : RD::UNIFORM_TYPE_STORAGE_BUFFER;
			u.binding = bnd;
			u.append_id(by_binding[bnd]);
			uniforms.push_back(u);
		}
		uset[p] = rd->uniform_set_create(uniforms, shader[p], 0);
		ERR_FAIL_COND(uset[p].is_null());
	}

	if (buf_mverts.is_valid()) {
		Vector<RD::Uniform> m;
		const RID mesh_bufs[3] = { buf_mverts, buf_mnorms, buf_mcount };
		for (int b = 0; b < 3; b++) {
			RD::Uniform u;
			u.uniform_type = RD::UNIFORM_TYPE_STORAGE_BUFFER;
			u.binding = b;
			u.append_id(mesh_bufs[b]);
			m.push_back(u);
		}
		uset_mesh = rd->uniform_set_create(m, shader[block_sparse ? PASS_BS_MARCH : PASS_MARCH], 1);
		ERR_FAIL_COND(uset_mesh.is_null());
	}
}

void MPMFluidSolverGPU::rt_build(Ref<MPMFluidSolverGPU> p_self, PackedByteArray p_params, PackedByteArray p_particles, int p_capacity, int p_node_count, int p_tri_budget, bool p_block_sparse, int p_max_blocks, int p_hash_slots) {
	if (rd == nullptr || !_shaders_ok) {
		return;
	}
	if (local && _submitted) {
		rd->sync(); // don't free buffers with GPU work still in flight
		_submitted = false;
	}
	built.set_to(false);
	_rt_free_buffers();

	capacity = p_capacity;
	node_count = p_node_count;
	tri_budget = p_tri_budget;
	block_sparse = p_block_sparse;
	max_blocks = p_max_blocks;
	hash_slots = p_hash_slots;

	{
		MutexLock lock(cache_mtx);
		mm_cache = PackedFloat32Array();
		imp_cache.clear();
		surf_verts_cache = PackedVector3Array();
		surf_norms_cache = PackedVector3Array();
		surf_tris_cache = 0;
	}

	buf_params = rd->uniform_buffer_create(PARAMS_BYTES, p_params);
	buf_particles = rd->storage_buffer_create(p_particles.size(), p_particles);

	PackedByteArray czero;
	czero.resize(MAX_COLLIDERS * FLOATS_PER_COLLIDER * 4);
	memset(czero.ptrw(), 0, czero.size());
	buf_colliders = rd->storage_buffer_create(czero.size(), czero);
	buf_cimp = rd->storage_buffer_create(MAX_COLLIDERS * 4 * sizeof(int32_t));

	{
		PackedByteArray mm_zero;
		mm_zero.resize(capacity * 12 * sizeof(float));
		memset(mm_zero.ptrw(), 0, mm_zero.size());
		buf_mm = rd->storage_buffer_create(mm_zero.size(), mm_zero);
	}

	if (block_sparse) {
		// Block pool: max_blocks * 64 cells, contiguous per block.
		const int cells = max_blocks * BCELLS;
		buf_grid_i = rd->storage_buffer_create(cells * 4 * sizeof(int32_t));
		buf_grid_v = rd->storage_buffer_create(cells * 4 * sizeof(float));
		buf_surf = rd->storage_buffer_create(cells * sizeof(int32_t)); // block-indexed density scatter (isosurface)
		buf_bhash = rd->storage_buffer_create(hash_slots * sizeof(uint32_t));
		buf_bhash_val = rd->storage_buffer_create(hash_slots * sizeof(uint32_t));
		buf_bkey = rd->storage_buffer_create(max_blocks * sizeof(uint32_t));
		buf_bcounts = rd->storage_buffer_create(8 * sizeof(uint32_t), Span<uint8_t>(), RD::STORAGE_BUFFER_USAGE_DISPATCH_INDIRECT);
		buf_mverts = rd->storage_buffer_create(tri_budget * 3 * 4 * sizeof(float));
		buf_mnorms = rd->storage_buffer_create(tri_budget * 3 * 4 * sizeof(float));
		PackedByteArray mc;
		mc.resize(2 * sizeof(uint32_t));
		encode_uint32(0, mc.ptrw());
		encode_uint32(tri_budget, mc.ptrw() + 4);
		buf_mcount = rd->storage_buffer_create(mc.size(), mc);
	} else {
		buf_grid_i = rd->storage_buffer_create(node_count * 4 * sizeof(int32_t));
		buf_grid_v = rd->storage_buffer_create(node_count * 4 * sizeof(float));
		buf_surf = rd->storage_buffer_create(node_count * sizeof(float));
		buf_mverts = rd->storage_buffer_create(tri_budget * 3 * 4 * sizeof(float)); // vec4 / vertex
		buf_mnorms = rd->storage_buffer_create(tri_budget * 3 * 4 * sizeof(float));
		PackedByteArray mc;
		mc.resize(2 * sizeof(uint32_t));
		encode_uint32(0, mc.ptrw()); // tri_count
		encode_uint32(tri_budget, mc.ptrw() + 4); // tri_budget
		buf_mcount = rd->storage_buffer_create(mc.size(), mc);
	}

	_rt_rebuild_uniform_sets();
	built.set_to(true);
}

void MPMFluidSolverGPU::rt_step(Ref<MPMFluidSolverGPU> p_self, PackedByteArray p_params, PackedByteArray p_colliders, int p_ncol, int p_pcount, int p_node_count, Vector3i p_grid_dims, int p_substeps, bool p_want_surface, bool p_bench) {
	if (rd == nullptr || !built.is_set() || p_pcount == 0) {
		return;
	}

	// Read back last frame's GPU timing capture (bench only).
	if (p_bench) {
		const uint32_t n = rd->get_captured_timestamps_count();
		uint64_t a = 0, b = 0;
		for (uint32_t i = 0; i < n; i++) {
			const String nm = rd->get_captured_timestamp_name(i);
			if (nm == "mpm_a") {
				a = rd->get_captured_timestamp_gpu_time(i);
			} else if (nm == "mpm_b") {
				b = rd->get_captured_timestamp_gpu_time(i);
			}
		}
		if (b > a) {
			MutexLock lock(cache_mtx);
			last_step_usec = (b - a) / 1000; // ns -> us
		}
	}

	rd->buffer_update(buf_params, 0, PARAMS_BYTES, p_params.ptr());
	if (p_ncol > 0 && p_colliders.size() > 0) {
		rd->buffer_update(buf_colliders, 0, p_colliders.size(), p_colliders.ptr());
	}
	rd->buffer_clear(buf_cimp, 0, MAX_COLLIDERS * 4 * sizeof(int32_t));
	if (p_want_surface) {
		rd->buffer_clear(buf_mcount, 0, sizeof(uint32_t)); // reset tri_count, keep tri_budget
		const uint32_t surf_bytes = block_sparse ? (uint32_t)(max_blocks * BCELLS * sizeof(int32_t)) : (uint32_t)(node_count * sizeof(int32_t));
		rd->buffer_clear(buf_surf, 0, surf_bytes); // reset the density scatter
	}

	const uint32_t ng = groups_for(node_count);
	const uint32_t pg = groups_for(p_pcount);
	const int cells = (p_grid_dims.x - 1) * (p_grid_dims.y - 1) * (p_grid_dims.z - 1);

	if (p_bench) {
		rd->capture_timestamp("mpm_a");
	}

	RD::ComputeListID cl = rd->compute_list_begin();

	if (block_sparse) {
		const uint32_t hg = groups_for(hash_slots);
		auto pass = [&](Pass pp, uint32_t gx) {
			rd->compute_list_bind_compute_pipeline(cl, pipeline[pp]);
			rd->compute_list_bind_uniform_set(cl, uset[pp], 0);
			rd->compute_list_dispatch(cl, gx, 1, 1);
			rd->compute_list_add_barrier(cl);
		};
		auto pass_indirect = [&](Pass pp) {
			rd->compute_list_bind_compute_pipeline(cl, pipeline[pp]);
			rd->compute_list_bind_uniform_set(cl, uset[pp], 0);
			rd->compute_list_dispatch_indirect(cl, buf_bcounts, 4); // bcounts[1..3]
			rd->compute_list_add_barrier(cl);
		};
		for (int s = 0; s < p_substeps; s++) {
			pass(PASS_BS_CLEAR, hg);
			pass(PASS_BS_BINSERT, pg);
			pass(PASS_BS_BDISPATCH, 1);
			pass_indirect(PASS_BS_CLEARNODES);
			pass(PASS_BS_P2G_MASS, pg);
			pass(PASS_BS_P2G_MOM, pg);
			pass_indirect(PASS_BS_GRID);
			if (p_ncol > 0) {
				pass_indirect(PASS_BS_COUPLE);
			}
			pass(PASS_BS_G2P, pg);
		}
		if (p_want_surface) {
			pass(PASS_BS_SURFACE, pg);
			rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_BS_MARCH]);
			rd->compute_list_bind_uniform_set(cl, uset[PASS_BS_MARCH], 0);
			rd->compute_list_bind_uniform_set(cl, uset_mesh, 1);
			rd->compute_list_dispatch_indirect(cl, buf_bcounts, 4); // one wg / active block
			rd->compute_list_add_barrier(cl);
		}
		rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_BS_RENDER]);
		rd->compute_list_bind_uniform_set(cl, uset[PASS_BS_RENDER], 0);
		rd->compute_list_dispatch(cl, pg, 1, 1);
		rd->compute_list_end();

		if (p_bench) {
			rd->capture_timestamp("mpm_b");
		}
		_submitted_ncol = p_ncol;
		_submitted_surface = p_want_surface;
		if (local) {
			rd->submit();
			_submitted = true;
			_local_reap();
			return;
		}
		rd->buffer_get_data_async(buf_mm, callable_mp(this, &MPMFluidSolverGPU::rt_on_mm).bind(p_self), 0, capacity * 12 * sizeof(float));
		if (p_ncol > 0) {
			rd->buffer_get_data_async(buf_cimp, callable_mp(this, &MPMFluidSolverGPU::rt_on_impulses).bind(p_self, p_ncol), 0, p_ncol * 4 * sizeof(int32_t));
		}
		if (p_want_surface) {
			const uint32_t est_tris = MIN((uint32_t)tri_budget, (uint32_t)(_last_surf_tris * 3 / 2) + 8192u);
			const uint32_t vbytes = est_tris * 3u * 4u * sizeof(float);
			rd->buffer_get_data_async(buf_mcount, callable_mp(this, &MPMFluidSolverGPU::rt_on_surf_count).bind(p_self), 0, sizeof(uint32_t));
			rd->buffer_get_data_async(buf_mverts, callable_mp(this, &MPMFluidSolverGPU::rt_on_surf_verts).bind(p_self), 0, vbytes);
			rd->buffer_get_data_async(buf_mnorms, callable_mp(this, &MPMFluidSolverGPU::rt_on_surf_norms).bind(p_self), 0, vbytes);
		}
		return;
	}

	for (int s = 0; s < p_substeps; s++) {
		rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_CLEAR]);
		rd->compute_list_bind_uniform_set(cl, uset[PASS_CLEAR], 0);
		rd->compute_list_dispatch(cl, ng, 1, 1);
		rd->compute_list_add_barrier(cl);

		rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_P2G_MASS]);
		rd->compute_list_bind_uniform_set(cl, uset[PASS_P2G_MASS], 0);
		rd->compute_list_dispatch(cl, pg, 1, 1);
		rd->compute_list_add_barrier(cl);

		rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_P2G_MOM]);
		rd->compute_list_bind_uniform_set(cl, uset[PASS_P2G_MOM], 0);
		rd->compute_list_dispatch(cl, pg, 1, 1);
		rd->compute_list_add_barrier(cl);

		rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_GRID]);
		rd->compute_list_bind_uniform_set(cl, uset[PASS_GRID], 0);
		rd->compute_list_dispatch(cl, ng, 1, 1);
		rd->compute_list_add_barrier(cl);

		if (p_ncol > 0) {
			rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_COUPLE]);
			rd->compute_list_bind_uniform_set(cl, uset[PASS_COUPLE], 0);
			rd->compute_list_dispatch(cl, ng, 1, 1);
			rd->compute_list_add_barrier(cl);
		}

		rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_G2P]);
		rd->compute_list_bind_uniform_set(cl, uset[PASS_G2P], 0);
		rd->compute_list_dispatch(cl, pg, 1, 1);
		rd->compute_list_add_barrier(cl);
	}
	if (p_want_surface) {
		rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_SURFACE]);
		rd->compute_list_bind_uniform_set(cl, uset[PASS_SURFACE], 0);
		rd->compute_list_dispatch(cl, pg, 1, 1);
		rd->compute_list_add_barrier(cl);

		rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_MARCH]);
		rd->compute_list_bind_uniform_set(cl, uset[PASS_MARCH], 0);
		rd->compute_list_bind_uniform_set(cl, uset_mesh, 1);
		rd->compute_list_dispatch(cl, groups_for(cells), 1, 1);
		rd->compute_list_add_barrier(cl);
	}
	rd->compute_list_bind_compute_pipeline(cl, pipeline[PASS_RENDER]);
	rd->compute_list_bind_uniform_set(cl, uset[PASS_RENDER], 0);
	rd->compute_list_dispatch(cl, pg, 1, 1);
	rd->compute_list_end();

	if (p_bench) {
		rd->capture_timestamp("mpm_b");
	}

	_submitted_ncol = p_ncol;
	_submitted_surface = p_want_surface;

	if (local) {
		// We own the device: run it now and pull results straight back.
		rd->submit();
		_submitted = true;
		_local_reap();
		return;
	}

	// Shared device: async readback. The callbacks land a few frames later on the
	// render thread and refresh the caches; each binds a Ref to us so we survive
	// the wait.
	rd->buffer_get_data_async(buf_mm, callable_mp(this, &MPMFluidSolverGPU::rt_on_mm).bind(p_self), 0, capacity * 12 * sizeof(float));
	if (p_ncol > 0) {
		rd->buffer_get_data_async(buf_cimp, callable_mp(this, &MPMFluidSolverGPU::rt_on_impulses).bind(p_self, p_ncol), 0, p_ncol * 4 * sizeof(int32_t));
	}
	if (p_want_surface) {
		// Size the vert/normal reads to a padded estimate of last frame's triangle
		// count rather than the full 200k budget (~19 MB) every surface frame. If
		// the count spiked past the estimate, get_surface_mesh() skips one update
		// while _last_surf_tris catches up -- imperceptible at the 20 Hz remesh.
		const uint32_t est_tris = MIN((uint32_t)tri_budget, (uint32_t)(_last_surf_tris * 3 / 2) + 8192u);
		const uint32_t vbytes = est_tris * 3u * 4u * sizeof(float);
		rd->buffer_get_data_async(buf_mcount, callable_mp(this, &MPMFluidSolverGPU::rt_on_surf_count).bind(p_self), 0, sizeof(uint32_t));
		rd->buffer_get_data_async(buf_mverts, callable_mp(this, &MPMFluidSolverGPU::rt_on_surf_verts).bind(p_self), 0, vbytes);
		rd->buffer_get_data_async(buf_mnorms, callable_mp(this, &MPMFluidSolverGPU::rt_on_surf_norms).bind(p_self), 0, vbytes);
	}
}

// Local device only: block on the pending submit, then pull the render buffer,
// collider impulses and (when it ran) the isosurface geometry into the caches.
void MPMFluidSolverGPU::_local_reap() {
	if (!_submitted) {
		return;
	}
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	rd->sync();
	last_step_usec = OS::get_singleton()->get_ticks_usec() - t0;
	_submitted = false;

	MutexLock lock(cache_mtx);

	Vector<uint8_t> mm = rd->buffer_get_data(buf_mm, 0, capacity * 12 * sizeof(float));
	mm_cache.resize(capacity * 12);
	if (mm.size() > 0) {
		memcpy(mm_cache.ptrw(), mm.ptr(), MIN((int)mm.size(), (int)(capacity * 12 * sizeof(float))));
	}

	imp_cache.clear();
	if (_submitted_ncol > 0) {
		Vector<uint8_t> raw = rd->buffer_get_data(buf_cimp, 0, _submitted_ncol * 4 * sizeof(int32_t));
		const int32_t *ci = (const int32_t *)raw.ptr();
		for (int i = 0; i < _submitted_ncol; i++) {
			imp_cache.push_back(Vector3(ci[i * 4 + 0], ci[i * 4 + 1], ci[i * 4 + 2]) / IMP_FIXED);
		}
	}

	if (_submitted_surface) {
		Vector<uint8_t> cnt = rd->buffer_get_data(buf_mcount, 0, sizeof(uint32_t));
		const uint32_t tris = (cnt.size() >= (int)sizeof(uint32_t)) ? MIN(decode_uint32(cnt.ptr()), (uint32_t)tri_budget) : 0;
		surf_tris_cache = (int)tris;
		if (tris > 0) {
			const uint32_t verts = tris * 3;
			Vector<uint8_t> vraw = rd->buffer_get_data(buf_mverts, 0, verts * 4 * sizeof(float));
			Vector<uint8_t> nraw = rd->buffer_get_data(buf_mnorms, 0, verts * 4 * sizeof(float));
			const float *vf = (const float *)vraw.ptr();
			const float *nf = (const float *)nraw.ptr();
			surf_verts_cache.resize(verts);
			surf_norms_cache.resize(verts);
			for (uint32_t i = 0; i < verts; i++) {
				surf_verts_cache.write[i] = Vector3(vf[i * 4 + 0], vf[i * 4 + 1], vf[i * 4 + 2]);
				surf_norms_cache.write[i] = Vector3(nf[i * 4 + 0], nf[i * 4 + 1], nf[i * 4 + 2]);
			}
		}
	}
}

void MPMFluidSolverGPU::rt_emit(Ref<MPMFluidSolverGPU> p_self, PackedByteArray p_blob, int p_head_bytes, int p_first_bytes) {
	if (rd == nullptr || !built.is_set() || p_blob.is_empty()) {
		return;
	}
	if (local && _submitted) {
		_local_reap(); // don't write the particle store while a step is reading it
	}
	rd->buffer_update(buf_particles, p_head_bytes, p_first_bytes, p_blob.ptr());
	const int rest = p_blob.size() - p_first_bytes;
	if (rest > 0) {
		rd->buffer_update(buf_particles, 0, rest, p_blob.ptr() + p_first_bytes);
	}
}

void MPMFluidSolverGPU::rt_read_positions(Ref<MPMFluidSolverGPU> p_self, int p_count) {
	if (rd == nullptr || !built.is_set() || buf_particles.is_null()) {
		pos_ready.set();
		return;
	}
	if (local && _submitted) {
		_local_reap();
	}
	Vector<uint8_t> raw = rd->buffer_get_data(buf_particles, 0, p_count * FLOATS_PER_PARTICLE * sizeof(float));
	const float *f = (const float *)raw.ptr();
	PackedVector3Array out;
	out.resize(p_count);
	for (int i = 0; i < p_count; i++) {
		const float *o = f + i * FLOATS_PER_PARTICLE;
		out.write[i] = Vector3(o[0], o[1], o[2]);
	}
	{
		MutexLock lock(cache_mtx);
		pos_cache = out;
	}
	pos_ready.set();
}

void MPMFluidSolverGPU::rt_on_mm(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self) {
	MutexLock lock(cache_mtx);
	mm_cache.resize(capacity * 12);
	const int n = MIN(p_data.size(), (int)(capacity * 12 * sizeof(float)));
	if (n > 0) {
		memcpy(mm_cache.ptrw(), p_data.ptr(), n);
	}
}

void MPMFluidSolverGPU::rt_on_impulses(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self, int p_ncol) {
	const int32_t *ci = (const int32_t *)p_data.ptr();
	const int have = p_data.size() / (int)sizeof(int32_t);
	MutexLock lock(cache_mtx);
	imp_cache.clear();
	for (int i = 0; i < p_ncol && i * 4 + 2 < have; i++) {
		imp_cache.push_back(Vector3(ci[i * 4 + 0], ci[i * 4 + 1], ci[i * 4 + 2]) / IMP_FIXED);
	}
}

void MPMFluidSolverGPU::rt_on_surf_count(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self) {
	if (p_data.size() < (int)sizeof(uint32_t)) {
		return;
	}
	const int tris = (int)MIN(decode_uint32(p_data.ptr()), (uint32_t)tri_budget);
	_last_surf_tris = tris; // render thread only -- sizes next frame's vert/normal reads
	MutexLock lock(cache_mtx);
	surf_tris_cache = tris;
}

void MPMFluidSolverGPU::rt_on_surf_verts(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self) {
	const float *vf = (const float *)p_data.ptr();
	const int verts = p_data.size() / (4 * (int)sizeof(float));
	PackedVector3Array out;
	out.resize(verts);
	for (int i = 0; i < verts; i++) {
		out.write[i] = Vector3(vf[i * 4 + 0], vf[i * 4 + 1], vf[i * 4 + 2]);
	}
	MutexLock lock(cache_mtx);
	surf_verts_cache = out;
}

void MPMFluidSolverGPU::rt_on_surf_norms(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self) {
	const float *nf = (const float *)p_data.ptr();
	const int verts = p_data.size() / (4 * (int)sizeof(float));
	PackedVector3Array out;
	out.resize(verts);
	for (int i = 0; i < verts; i++) {
		out.write[i] = Vector3(nf[i * 4 + 0], nf[i * 4 + 1], nf[i * 4 + 2]);
	}
	MutexLock lock(cache_mtx);
	surf_norms_cache = out;
}

// Free the pipelines + buffers on the render thread. MPMFluidSolver's destructor
// posts this and then RS::sync()s, so the RIDs are gone before the last Ref
// drops; the object destructor is then a no-op.
void MPMFluidSolverGPU::rt_free(Ref<MPMFluidSolverGPU> p_self) {
	if (rd == nullptr) {
		return;
	}
	if (local && _submitted) {
		rd->sync();
		_submitted = false;
	}
	built.set_to(false);
	_rt_free_buffers();
	for (int i = 0; i < PASS_MAX; i++) {
		if (pipeline[i].is_valid()) {
			rd->free_rid(pipeline[i]);
			pipeline[i] = RID();
		}
		if (shader[i].is_valid()) {
			rd->free_rid(shader[i]);
			shader[i] = RID();
		}
	}
	_shaders_ok = false;
}

MPMFluidSolverGPU::~MPMFluidSolverGPU() {
#ifdef DEV_ENABLED
	// rt_free() should have run first. If a stale async callback kept us alive
	// past it, everything is already null and there is nothing to do.
	for (int i = 0; i < PASS_MAX; i++) {
		DEV_ASSERT(shader[i].is_null() && pipeline[i].is_null());
	}
#endif
}

/* ===================================================================== */
/*  MPMFluidSolver -- CPU-side front. Preps payloads, posts render work.   */
/* ===================================================================== */

MPMFluidSolver::MPMFluidSolver() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs == nullptr) {
		return;
	}
	gpu.instantiate();
	gpu->rd = rs->get_rendering_device();
	if (gpu->rd == nullptr) {
		// Headless: no main device. Spin up a private local one and drive it
		// synchronously on this thread (the old path -- fine for automated tests,
		// where nothing is being rendered anyway).
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
	_dispatch(callable_mp(gpu.ptr(), &MPMFluidSolverGPU::rt_compile).bind(gpu));
}

MPMFluidSolver::~MPMFluidSolver() {
	if (gpu.is_valid() && gpu->rd != nullptr) {
		if (gpu->local) {
			gpu->rt_free(gpu);
			memdelete(gpu->rd); // we own the local device
			gpu->rd = nullptr;
		} else {
			RenderingServer *rs = RenderingServer::get_singleton();
			rs->call_on_render_thread(callable_mp(gpu.ptr(), &MPMFluidSolverGPU::rt_free).bind(gpu));
			rs->sync(); // wait for the render thread to run rt_free before we drop our Ref
		}
	}
	// Any in-flight async readback still holds a bound Ref; when it finally fires
	// it finds every RID null and the object destructor no-ops.
	gpu.unref();
}

void MPMFluidSolver::_dispatch(const Callable &p_call) const {
	if (gpu.is_null()) {
		return;
	}
	if (gpu->local) {
		p_call.call(); // run now, on this thread, on the device we own
	} else {
		RenderingServer::get_singleton()->call_on_render_thread(p_call);
	}
}

void MPMFluidSolver::_compute_scales() {
	dx = settings.cell_size > 0.0f ? settings.cell_size : settings.domain.x / (float)settings.grid_res;
	grid_dims = Vector3i(
			settings.grid_res,
			CLAMP((int)Math::round(settings.domain.y / dx), 4, 256),
			CLAMP((int)Math::round(settings.domain.z / dx), 4, 256));
	node_count = grid_dims.x * grid_dims.y * grid_dims.z;

	// Block-sparse fluid path: a 4^3-block pool, sized from the particle target
	// (~N/6/64 full blocks + partial surface/halo blocks -> N/6 is generous). The
	// grid is boundless so there is no box to cap against; overflow degrades
	// gracefully via the probe cap.
	if (!settings.granular) {
		const int64_t want = (int64_t)MAX(settings.particle_target, 1) / 6 + 512;
		max_blocks = (int)CLAMP(want, (int64_t)512, (int64_t)(1 << 21));
		hash_slots = (int)next_pow2((uint32_t)MAX(max_blocks * 4, 256));
	} else {
		max_blocks = 0;
		hash_slots = 0;
	}

	const float spacing = dx * 0.5f;
	pmass = MAX(settings.rest_density, 1.0f) * spacing * spacing * spacing;
	_recompute_surface_iso();
}

void MPMFluidSolver::_recompute_surface_iso() {
	const float sp = MAX(dx * 0.5f, 1e-4f);
	const float h = MAX(settings.surface_kernel, dx);
	const float h2 = h * h;
	const float norm = (315.0f / (64.0f * 3.14159265f * Math::pow(h, 9.0f))) * pmass;
	const int R = (int)Math::ceil(h / sp) + 1;
	float ref = 0.0f;
	for (int z = -R; z <= R; z++) {
		for (int y = -R; y <= R; y++) {
			for (int x = -R; x <= R; x++) {
				const float d2 = (float)(x * x + y * y + z * z) * sp * sp;
				if (d2 < h2) {
					const float t = h2 - d2;
					ref += norm * t * t * t;
				}
			}
		}
	}
	surf_iso_density = CLAMP(settings.surface_iso, 0.05f, 0.95f) * MAX(ref, 1.0f);
}

LocalVector<float> MPMFluidSolver::_seed_block(int &r_count) const {
	const float spacing = dx * 0.5f;

	const Vector3 margin = Vector3(dx, dx, dx) * 2.0f;
	const Vector3 region = settings.spawn_region.clamp(Vector3(spacing, spacing, spacing), settings.domain - margin * 2.0f);
	const int sx = MAX(1, (int)(region.x / spacing));
	const int sy = MAX(1, (int)(region.y / spacing));
	const int sz = MAX(1, (int)(region.z / spacing));

	const Vector3 center = domain_xform.origin;
	const Vector3 start = center - Vector3(sx, sy, sz) * (spacing * 0.5f);

	LocalVector<float> data;
	data.resize((int64_t)MIN((int64_t)sx * sy * sz, (int64_t)settings.particle_target) * FLOATS_PER_PARTICLE);
	memset(data.ptr(), 0, data.size() * sizeof(float));
	int idx = 0;
	const int cap = (int)(data.size() / FLOATS_PER_PARTICLE);
	for (int yi = 0; yi < sy && idx < cap; yi++) {
		for (int zi = 0; zi < sz && idx < cap; zi++) {
			for (int xi = 0; xi < sx && idx < cap; xi++) {
				Vector3 p = start + Vector3(xi, yi, zi) * spacing;
				if (settings.granular) {
					p.x += (float((yi + zi) & 1) - 0.5f) * spacing * 0.5f;
					p.z += (float(yi & 1) - 0.5f) * spacing * 0.5f;
					p += Vector3(Math::randf() - 0.5f, Math::randf() - 0.5f, Math::randf() - 0.5f) * spacing;
				} else {
					p += Vector3(Math::randf(), Math::randf(), Math::randf()) * spacing * 0.3f;
				}
				float *o = &data[idx * FLOATS_PER_PARTICLE];
				o[0] = p.x;
				o[1] = p.y;
				o[2] = p.z;
				o[3] = settings.rest_density;
				o[20] = o[25] = o[30] = 1.0f; // deformation gradient F = identity
				idx++;
			}
		}
	}
	r_count = idx;
	return data;
}

void MPMFluidSolver::_pack_params(double p_dt, int p_ncol, PackedByteArray &r_bytes) const {
	r_bytes.resize(PARAMS_BYTES);
	uint8_t *b = r_bytes.ptrw();
	const Vector3 center = domain_xform.origin;
	const Vector3 bmin = center - settings.domain * 0.5f;
	const Vector3 bmax = center + settings.domain * 0.5f;
	// Dense/granular: ORIGIN is the live domain-min corner (the box BC needs it).
	// Block-sparse: ORIGIN is the anchor frozen at configure() (boundless keys).
	const Vector3 origin = settings.granular ? bmin : grid_anchor;
	auto put_f = [&](uint32_t off, float v) { encode_float(v, b + off); };
	auto put_i = [&](uint32_t off, int32_t v) { encode_uint32((uint32_t)v, b + off); };

	put_f(0, settings.gravity.x);
	put_f(4, settings.gravity.y);
	put_f(8, settings.gravity.z);
	put_f(12, (float)p_dt);
	put_f(16, origin.x);
	put_f(20, origin.y);
	put_f(24, origin.z);
	put_f(28, dx);
	put_i(32, grid_dims.x);
	put_i(36, grid_dims.y);
	put_i(40, grid_dims.z);
	put_i(44, pcount);
	put_f(48, settings.rest_density);
	put_f(52, settings.stiffness);
	put_f(56, settings.viscosity);
	put_f(60, pmass);
	put_f(64, bmin.x);
	put_f(68, bmin.y);
	put_f(72, bmin.z);
	put_f(76, settings.surface_boost);
	put_f(80, bmax.x);
	put_f(84, bmax.y);
	put_f(88, bmax.z);
	put_f(92, settings.granular ? 1.0f : 0.0f);
	put_f(96, (float)p_ncol);
	put_f(100, settings.collider_friction);
	put_f(104, surf_iso_density);
	put_f(108, settings.surface_kernel);

	const float sinp = Math::sin(Math::deg_to_rad(CLAMP(settings.granular_friction_deg, 1.0f, 55.0f)));
	const float alpha = Math::sqrt(2.0f / 3.0f) * (2.0f * sinp) / (3.0f - sinp);
	const float E = MAX(settings.granular_hardness, 1.0f);
	const float nu = 0.3f;
	const float mu = E / (2.0f * (1.0f + nu));
	const float lambda = E * nu / ((1.0f + nu) * (1.0f - 2.0f * nu));
	put_f(112, alpha);
	put_f(116, MAX(settings.granular_cohesion, 0.0f));
	put_f(120, mu);
	put_f(124, lambda);

	// blockp (block-sparse fluid path; 0 for dense/granular)
	put_f(128, (float)hash_slots);
	put_f(132, (float)max_blocks);
	put_f(136, 0.0f);
	put_f(140, 0.0f);
}

void MPMFluidSolver::_pack_colliders(const LocalVector<SphereCollider> &p_colliders, PackedByteArray &r_bytes) const {
	r_bytes.resize(MAX_COLLIDERS * FLOATS_PER_COLLIDER * 4);
	memset(r_bytes.ptrw(), 0, r_bytes.size());
	uint8_t *b = r_bytes.ptrw();
	const int n = MIN((int)p_colliders.size(), MAX_COLLIDERS);
	for (int i = 0; i < n; i++) {
		const Collider &c = p_colliders[i];
		uint8_t *o = b + i * FLOATS_PER_COLLIDER * 4;
		encode_float(c.position.x, o + 0);
		encode_float(c.position.y, o + 4);
		encode_float(c.position.z, o + 8);
		encode_float((float)c.shape, o + 12);
		Vector3 ext = c.extents;
		if (c.shape == COLLIDER_PLANE) {
			ext = ext.normalized();
		}
		encode_float(ext.x, o + 16);
		encode_float(ext.y, o + 20);
		encode_float(ext.z, o + 24);
		encode_float(0.0f, o + 28);
		encode_float(c.velocity.x, o + 32);
		encode_float(c.velocity.y, o + 36);
		encode_float(c.velocity.z, o + 40);
		encode_float(0.0f, o + 44);
		const Quaternion q = c.rotation.normalized();
		encode_float(q.x, o + 48);
		encode_float(q.y, o + 52);
		encode_float(q.z, o + 56);
		encode_float(q.w, o + 60);
	}
}

void MPMFluidSolver::configure(const Settings &p_settings, const Transform3D &p_xform, bool p_prefill) {
	if (gpu.is_null()) {
		return;
	}
	settings = p_settings;
	settings.grid_res = CLAMP(settings.grid_res, 8, 128);
	settings.substeps = CLAMP(settings.substeps, 1, 40);
	domain_xform = p_xform;

	if (settings.granular) {
		settings.substeps = MAX(settings.substeps, 12);
		settings.collider_friction = MAX(settings.collider_friction, 0.85f);
	}

	_compute_scales();
	// Block-sparse: freeze the cell-key origin at configure so keys are stable
	// even if the emitter node moves; the boundless grid tracks the fluid.
	grid_anchor = (domain_xform.origin - settings.domain * 0.5f).snappedf(dx);
	write_head = 0;

	PackedByteArray job_particles;
	if (p_prefill) {
		int seeded = 0;
		LocalVector<float> pdata = _seed_block(seeded);
		capacity = seeded;
		pcount = seeded;
		job_particles.resize(pdata.size() * sizeof(float));
		if (pdata.size() > 0) {
			memcpy(job_particles.ptrw(), pdata.ptr(), pdata.size() * sizeof(float));
		}
	} else {
		capacity = MAX(settings.particle_target, 1);
		pcount = 0;
		job_particles.resize((int64_t)capacity * FLOATS_PER_PARTICLE * sizeof(float));
		memset(job_particles.ptrw(), 0, job_particles.size());
	}

	PackedByteArray params;
	_pack_params(1.0 / 60.0 / settings.substeps, 0, params);

	gpu->built.set_to(false);
	const bool bs = !settings.granular;
	_dispatch(callable_mp(gpu.ptr(), &MPMFluidSolverGPU::rt_build).bind(gpu, params, job_particles, capacity, node_count, (int)TRI_BUDGET, bs, max_blocks, hash_slots));
}

void MPMFluidSolver::emit(const LocalVector<EmittedParticle> &p_new) {
	if (!is_available() || p_new.is_empty() || capacity == 0) {
		return;
	}
	const int n = MIN((int)p_new.size(), capacity);

	LocalVector<float> blob;
	blob.resize((int64_t)n * FLOATS_PER_PARTICLE);
	memset(blob.ptr(), 0, blob.size() * sizeof(float));
	for (int i = 0; i < n; i++) {
		float *o = &blob[i * FLOATS_PER_PARTICLE];
		o[0] = p_new[i].position.x;
		o[1] = p_new[i].position.y;
		o[2] = p_new[i].position.z;
		o[3] = settings.rest_density;
		o[4] = p_new[i].velocity.x;
		o[5] = p_new[i].velocity.y;
		o[6] = p_new[i].velocity.z;
		o[20] = o[25] = o[30] = 1.0f;
	}

	const int stride = FLOATS_PER_PARTICLE * (int)sizeof(float);
	const int first = MIN(n, capacity - write_head); // fits before wrapping

	PackedByteArray bytes;
	bytes.resize(n * stride);
	memcpy(bytes.ptrw(), blob.ptr(), bytes.size());
	_dispatch(callable_mp(gpu.ptr(), &MPMFluidSolverGPU::rt_emit).bind(gpu, bytes, write_head * stride, first * stride));

	write_head = (write_head + n) % capacity;
	pcount = MIN(pcount + n, capacity);
}

void MPMFluidSolver::set_domain_transform(const Transform3D &p_xform) {
	domain_xform = p_xform;
}

void MPMFluidSolver::step(double p_delta, const LocalVector<SphereCollider> &p_colliders, LocalVector<Vector3> *r_impulses, bool p_want_surface, bool p_async) {
	if (!is_available() || pcount == 0 || p_delta <= 0.0) {
		return;
	}
	if (unlikely(OS::get_singleton()->has_environment("MPM_PROFILE"))) {
		WARN_PRINT_ONCE("MPM_PROFILE: per-pass timing is unavailable on the shared RenderingDevice (needs submit/sync).");
		return;
	}

	const int ncol = MIN((int)p_colliders.size(), MAX_COLLIDERS);
	const bool bench = OS::get_singleton()->has_environment("MPM_BENCH");

	PackedByteArray params;
	_pack_params(p_delta / (double)settings.substeps, ncol, params);
	PackedByteArray cdata;
	if (ncol > 0) {
		_pack_colliders(p_colliders, cdata);
	}

	// Hand back the reaped impulses and consume them: each readback carries one
	// step's accumulated reaction and must be applied exactly once. Frames where
	// no readback has landed yet apply nothing (rather than re-applying a stale
	// batch, which double-counts momentum and diverges a buoyancy spring).
	if (r_impulses != nullptr) {
		MutexLock lock(gpu->cache_mtx);
		*r_impulses = gpu->imp_cache;
		gpu->imp_cache.clear();
	}

	_dispatch(callable_mp(gpu.ptr(), &MPMFluidSolverGPU::rt_step).bind(gpu, params, cdata, ncol, pcount, node_count, grid_dims, settings.substeps, p_want_surface, bench));
}

double MPMFluidSolver::get_last_step_msec() const {
	if (gpu.is_null()) {
		return 0.0;
	}
	MutexLock lock(gpu->cache_mtx);
	return gpu->last_step_usec / 1000.0;
}

PackedFloat32Array MPMFluidSolver::get_multimesh_buffer() const {
	if (gpu.is_null()) {
		return PackedFloat32Array();
	}
	MutexLock lock(gpu->cache_mtx);
	return gpu->mm_cache;
}

PackedVector3Array MPMFluidSolver::get_positions() const {
	PackedVector3Array out;
	if (!is_available() || pcount == 0) {
		return out;
	}
	gpu->pos_ready.clear();
	_dispatch(callable_mp(gpu.ptr(), &MPMFluidSolverGPU::rt_read_positions).bind(gpu, pcount));
	if (!gpu->local) {
		RenderingServer::get_singleton()->sync(); // wait for the render thread to fill pos_cache
	}
	MutexLock lock(gpu->cache_mtx);
	return gpu->pos_cache;
}

int MPMFluidSolver::get_surface_mesh(PackedVector3Array &r_vertices, PackedVector3Array &r_normals) const {
	r_vertices.clear();
	r_normals.clear();
	if (gpu.is_null()) {
		return 0;
	}
	MutexLock lock(gpu->cache_mtx);
	const int tris = gpu->surf_tris_cache;
	const int verts = tris * 3;
	if (tris <= 0 || gpu->surf_verts_cache.size() < verts || gpu->surf_norms_cache.size() < verts) {
		return 0;
	}
	r_vertices.resize(verts);
	r_normals.resize(verts);
	memcpy(r_vertices.ptrw(), gpu->surf_verts_cache.ptr(), verts * sizeof(Vector3));
	memcpy(r_normals.ptrw(), gpu->surf_norms_cache.ptr(), verts * sizeof(Vector3));
	return tris;
}

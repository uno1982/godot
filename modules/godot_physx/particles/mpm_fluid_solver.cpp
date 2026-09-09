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

#include "core/io/marshalls.h"
#include "core/math/math_funcs.h"
#include "core/os/os.h"
#include "servers/display/display_server.h"
#include "servers/rendering/rendering_device_binds.h"
#include "servers/rendering/rendering_server.h"

namespace {
constexpr int FLOATS_PER_PARTICLE = 20; // 5 * vec4
constexpr int FLOATS_PER_COLLIDER = 16; // 4 * vec4 (c0..c3)
constexpr int MAX_COLLIDERS = 32; // analytic colliders per step (rigid bodies + expanded debris chunks)
constexpr uint32_t PARAMS_BYTES = 112; // 7 * vec4, std140
constexpr float IMP_FIXED = 1024.0f;
constexpr uint32_t GROUP = 64;
constexpr uint32_t TRI_BUDGET = 200000; // isosurface triangle cap

uint32_t groups_for(int p_count) {
	return (uint32_t)((p_count + (int)GROUP - 1) / (int)GROUP);
}
} // namespace

MPMFluidSolver::MPMFluidSolver() {
	if (!DisplayServer::can_create_rendering_device()) {
		return;
	}
	rd = RenderingServer::get_singleton()->create_local_rendering_device();
	if (rd == nullptr) {
		return;
	}
	if (!_compile_shaders()) {
		memdelete(rd);
		rd = nullptr;
	}
}

MPMFluidSolver::~MPMFluidSolver() {
	if (rd == nullptr) {
		return;
	}
	_free_buffers(); // uniform sets + buffers
	for (int i = 0; i < PASS_MAX; i++) {
		if (pipeline[i].is_valid()) {
			rd->free_rid(pipeline[i]);
			pipeline[i] = RID();
		}
	}
	for (int i = 0; i < PASS_MAX; i++) {
		if (shader[i].is_valid()) {
			rd->free_rid(shader[i]);
			shader[i] = RID();
		}
	}
	memdelete(rd);
	rd = nullptr;
}

bool MPMFluidSolver::_compile_shaders() {
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
	};
	for (int i = 0; i < PASS_MAX; i++) {
		Ref<RDShaderFile> sf;
		sf.instantiate();
		Error err = sf->parse_versions_from_text(src[i]);
		if (err != OK) {
			ERR_PRINT(vformat("MPMFluidSolver: compute shader %d failed to compile.", i));
			return false;
		}
		shader[i] = rd->shader_create_from_spirv(sf->get_spirv_stages());
		ERR_FAIL_COND_V(shader[i].is_null(), false);
		pipeline[i] = rd->compute_pipeline_create(shader[i]);
		ERR_FAIL_COND_V(pipeline[i].is_null(), false);
	}
	return true;
}

void MPMFluidSolver::_free_buffers() {
	// Uniform sets first: RD auto-frees a uniform set when a buffer it references
	// is freed, so freeing the buffers first leaves these RIDs dangling.
	for (int i = 0; i < PASS_MAX; i++) {
		if (uset[i].is_valid()) {
			rd->free_rid(uset[i]);
			uset[i] = RID();
		}
	}
	if (uset_mesh.is_valid()) {
		rd->free_rid(uset_mesh);
		uset_mesh = RID();
	}
	RID *bufs[] = { &buf_params, &buf_particles, &buf_grid_i, &buf_grid_v, &buf_colliders, &buf_cimp, &buf_mm, &buf_surf, &buf_mverts, &buf_mnorms, &buf_mcount };
	for (RID *b : bufs) {
		if (b->is_valid()) {
			rd->free_rid(*b);
			*b = RID();
		}
	}
}

void MPMFluidSolver::_compute_scales() {
	dx = settings.domain.x / (float)settings.grid_res;
	// Uniform cell size; the grid gets as many cells per axis as the domain needs
	// so a non-cube domain (e.g. a tall tank) is covered.
	grid_dims = Vector3i(
			settings.grid_res,
			CLAMP((int)Math::round(settings.domain.y / dx), 4, 256),
			CLAMP((int)Math::round(settings.domain.z / dx), 4, 256));
	node_count = grid_dims.x * grid_dims.y * grid_dims.z;
	const float spacing = dx * 0.5f;
	pmass = 1000.0f * spacing * spacing * spacing;
	_recompute_surface_iso();
}

// The poly6 kernel is heavily h-dependent (norm ~ 1/h^9), so the absolute
// density a well-packed fluid reads with a given kernel radius varies a lot.
// Evaluate that packed reference here and take the iso as a fraction of it, so
// the surface stays at a consistent "fullness" as particle_size changes.
void MPMFluidSolver::_recompute_surface_iso() {
	const float sp = MAX(dx * 0.5f, 1e-4f); // particle spacing
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

	// Fill spawn_region, centred on the node, clamped to fit inside the domain
	// (leave a 2-cell margin off the boundary), capped at particle_target.
	const Vector3 margin = Vector3(dx, dx, dx) * 2.0f;
	const Vector3 region = settings.spawn_region.clamp(Vector3(spacing, spacing, spacing), settings.domain - margin * 2.0f);
	const int sx = MAX(1, (int)(region.x / spacing));
	const int sy = MAX(1, (int)(region.y / spacing));
	const int sz = MAX(1, (int)(region.z / spacing));

	const Vector3 center = domain_xform.origin;
	const Vector3 start = center - Vector3(sx, sy, sz) * (spacing * 0.5f);

	LocalVector<float> data;
	data.resize((int64_t)MIN((int64_t)sx * sy * sz, (int64_t)settings.particle_target) * FLOATS_PER_PARTICLE);
	int idx = 0;
	const int cap = (int)(data.size() / FLOATS_PER_PARTICLE);
	for (int zi = 0; zi < sz && idx < cap; zi++) {
		for (int yi = 0; yi < sy && idx < cap; yi++) {
			for (int xi = 0; xi < sx && idx < cap; xi++) {
				Vector3 p = start + Vector3(xi, yi, zi) * spacing;
				p += Vector3(Math::randf(), Math::randf(), Math::randf()) * spacing * 0.3f;
				float *o = &data[idx * FLOATS_PER_PARTICLE];
				o[0] = p.x;
				o[1] = p.y;
				o[2] = p.z;
				o[3] = settings.rest_density;
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
	auto put_f = [&](uint32_t off, float v) { encode_float(v, b + off); };
	auto put_i = [&](uint32_t off, int32_t v) { encode_uint32((uint32_t)v, b + off); };

	put_f(0, settings.gravity.x);
	put_f(4, settings.gravity.y);
	put_f(8, settings.gravity.z);
	put_f(12, (float)p_dt);
	put_f(16, bmin.x);
	put_f(20, bmin.y);
	put_f(24, bmin.z);
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
	put_f(76, settings.surface_boost); // per-particle mass multiplier for the isosurface scatter
	put_f(80, bmax.x);
	put_f(84, bmax.y);
	put_f(88, bmax.z);
	put_f(92, 0.0f);
	put_f(96, (float)p_ncol);
	put_f(100, settings.collider_friction);
	put_f(104, surf_iso_density);
	put_f(108, settings.surface_kernel);
}

void MPMFluidSolver::_pack_colliders(const LocalVector<SphereCollider> &p_colliders, PackedByteArray &r_bytes) const {
	r_bytes.resize(MAX_COLLIDERS * FLOATS_PER_COLLIDER * 4);
	memset(r_bytes.ptrw(), 0, r_bytes.size());
	uint8_t *b = r_bytes.ptrw();
	const int n = MIN((int)p_colliders.size(), MAX_COLLIDERS);
	for (int i = 0; i < n; i++) {
		const Collider &c = p_colliders[i];
		uint8_t *o = b + i * FLOATS_PER_COLLIDER * 4;
		// c0: centre, shape
		encode_float(c.position.x, o + 0);
		encode_float(c.position.y, o + 4);
		encode_float(c.position.z, o + 8);
		encode_float((float)c.shape, o + 12);
		// c1: shape params
		Vector3 ext = c.extents;
		if (c.shape == COLLIDER_PLANE) {
			ext = ext.normalized();
		}
		encode_float(ext.x, o + 16);
		encode_float(ext.y, o + 20);
		encode_float(ext.z, o + 24);
		encode_float(0.0f, o + 28);
		// c2: velocity
		encode_float(c.velocity.x, o + 32);
		encode_float(c.velocity.y, o + 36);
		encode_float(c.velocity.z, o + 40);
		encode_float(0.0f, o + 44);
		// c3: orientation quaternion (identity for sphere / plane)
		const Quaternion q = c.rotation.normalized();
		encode_float(q.x, o + 48);
		encode_float(q.y, o + 52);
		encode_float(q.z, o + 56);
		encode_float(q.w, o + 60);
	}
}

void MPMFluidSolver::_rebuild_uniform_sets() {
	RID by_binding[8] = { buf_params, buf_particles, buf_grid_i, buf_grid_v, buf_colliders, buf_cimp, buf_mm, buf_surf };
	for (int p = 0; p < PASS_MAX; p++) {
		if (uset[p].is_valid()) {
			rd->free_rid(uset[p]);
			uset[p] = RID();
		}
		// Every pass includes mpm_fluid_inc.glsl, which declares all eight
		// bindings -- shader reflection puts them all in the set layout whether
		// or not a given pass references them, so provide all eight.
		Vector<RD::Uniform> uniforms;
		for (int bnd = 0; bnd < 8; bnd++) {
			RD::Uniform u;
			u.uniform_type = (bnd == 0) ? RD::UNIFORM_TYPE_UNIFORM_BUFFER : RD::UNIFORM_TYPE_STORAGE_BUFFER;
			u.binding = bnd;
			u.append_id(by_binding[bnd]);
			uniforms.push_back(u);
		}
		uset[p] = rd->uniform_set_create(uniforms, shader[p], 0);
		ERR_FAIL_COND(uset[p].is_null());
	}

	// The march pass also binds set 1: the mesh output buffers.
	if (uset_mesh.is_valid()) {
		rd->free_rid(uset_mesh);
		uset_mesh = RID();
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
		uset_mesh = rd->uniform_set_create(m, shader[PASS_MARCH], 1);
		ERR_FAIL_COND(uset_mesh.is_null());
	}
}

void MPMFluidSolver::configure(const Settings &p_settings, const Transform3D &p_xform, bool p_prefill) {
	if (rd == nullptr) {
		return;
	}
	settings = p_settings;
	settings.grid_res = CLAMP(settings.grid_res, 8, 128);
	settings.substeps = CLAMP(settings.substeps, 1, 40);
	domain_xform = p_xform;

	_free_buffers();
	_compute_scales(); // sets grid_dims + node_count
	write_head = 0;

	LocalVector<float> pdata;
	if (p_prefill) {
		int seeded = 0;
		pdata = _seed_block(seeded);
		capacity = seeded;
		pcount = seeded;
	} else {
		capacity = MAX(settings.particle_target, 1);
		pdata.resize((int64_t)capacity * FLOATS_PER_PARTICLE);
		memset(pdata.ptr(), 0, pdata.size() * sizeof(float)); // parked at the origin, PCOUNT keeps them out of the sim
		pcount = 0;
	}

	PackedByteArray params;
	_pack_params(1.0 / 60.0 / settings.substeps, 0, params);
	buf_params = rd->uniform_buffer_create(PARAMS_BYTES, params);

	buf_particles = rd->storage_buffer_create(pdata.size() * sizeof(float), Span<uint8_t>((const uint8_t *)pdata.ptr(), pdata.size() * sizeof(float)));
	buf_grid_i = rd->storage_buffer_create(node_count * 4 * sizeof(int32_t));
	buf_grid_v = rd->storage_buffer_create(node_count * 4 * sizeof(float));

	PackedByteArray cdata;
	_pack_colliders(LocalVector<SphereCollider>(), cdata);
	buf_colliders = rd->storage_buffer_create(cdata.size(), cdata);
	buf_cimp = rd->storage_buffer_create(MAX_COLLIDERS * 4 * sizeof(int32_t));
	{
		PackedByteArray mm_zero;
		mm_zero.resize(capacity * 12 * sizeof(float));
		memset(mm_zero.ptrw(), 0, mm_zero.size());
		buf_mm = rd->storage_buffer_create(mm_zero.size(), mm_zero);
	}
	buf_surf = rd->storage_buffer_create(node_count * sizeof(float));

	// Isosurface mesh output (set 1 of the march pass).
	buf_mverts = rd->storage_buffer_create(TRI_BUDGET * 3 * 4 * sizeof(float)); // vec4 / vertex
	buf_mnorms = rd->storage_buffer_create(TRI_BUDGET * 3 * 4 * sizeof(float));
	{
		PackedByteArray mc;
		mc.resize(2 * sizeof(uint32_t));
		encode_uint32(0, mc.ptrw()); // tri_count
		encode_uint32(TRI_BUDGET, mc.ptrw() + 4); // tri_budget
		buf_mcount = rd->storage_buffer_create(mc.size(), mc);
	}

	_rebuild_uniform_sets();
	built = true;
}

void MPMFluidSolver::emit(const LocalVector<EmittedParticle> &p_new) {
	if (!is_available() || p_new.is_empty() || capacity == 0) {
		return;
	}
	const int n = MIN((int)p_new.size(), capacity);

	// Build the packed particle records (only position + velocity; C and the
	// density slot start at zero / rest).
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
	}

	const uint32_t stride = FLOATS_PER_PARTICLE * sizeof(float);
	const int first = MIN(n, capacity - write_head); // fits before wrapping
	rd->buffer_update(buf_particles, write_head * stride, first * stride, blob.ptr());
	if (first < n) {
		rd->buffer_update(buf_particles, 0, (n - first) * stride, blob.ptr() + first * FLOATS_PER_PARTICLE);
	}
	write_head = (write_head + n) % capacity;
	pcount = MIN(pcount + n, capacity);
}

void MPMFluidSolver::set_domain_transform(const Transform3D &p_xform) {
	domain_xform = p_xform;
}

void MPMFluidSolver::step(double p_delta, const LocalVector<SphereCollider> &p_colliders, LocalVector<Vector3> *r_impulses, bool p_want_surface) {
	if (!is_available() || pcount == 0 || p_delta <= 0.0) {
		return;
	}

	const int ncol = MIN((int)p_colliders.size(), MAX_COLLIDERS);
	const double frame_dt = p_delta;
	const double dt = frame_dt / (double)settings.substeps;

	PackedByteArray params;
	_pack_params(dt, ncol, params);
	rd->buffer_update(buf_params, 0, PARAMS_BYTES, params.ptr());

	if (ncol > 0) {
		PackedByteArray cdata;
		_pack_colliders(p_colliders, cdata);
		rd->buffer_update(buf_colliders, 0, cdata.size(), cdata.ptr());
	}
	rd->buffer_clear(buf_cimp, 0, MAX_COLLIDERS * 4 * sizeof(int32_t));
	if (p_want_surface) {
		rd->buffer_clear(buf_mcount, 0, sizeof(uint32_t)); // reset tri_count, keep tri_budget
		rd->buffer_clear(buf_surf, 0, node_count * sizeof(int32_t)); // reset the density scatter
	}

	const uint32_t ng = groups_for(node_count);
	const uint32_t pg = groups_for(pcount);
	const int cells = (grid_dims.x - 1) * (grid_dims.y - 1) * (grid_dims.z - 1);

	RD::ComputeListID cl = rd->compute_list_begin();
	for (int s = 0; s < settings.substeps; s++) {
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

		if (ncol > 0) {
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
		rd->compute_list_dispatch(cl, pg, 1, 1); // one thread per particle (SPH scatter)
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

	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	rd->submit();
	rd->sync();
	last_step_usec = OS::get_singleton()->get_ticks_usec() - t0;

	if (r_impulses != nullptr) {
		r_impulses->clear();
		if (ncol > 0) {
			Vector<uint8_t> raw = rd->buffer_get_data(buf_cimp, 0, ncol * 4 * sizeof(int32_t));
			const int32_t *ci = (const int32_t *)raw.ptr();
			for (int i = 0; i < ncol; i++) {
				r_impulses->push_back(Vector3(ci[i * 4 + 0], ci[i * 4 + 1], ci[i * 4 + 2]) / IMP_FIXED);
			}
		}
	}
}

PackedFloat32Array MPMFluidSolver::get_multimesh_buffer() const {
	PackedFloat32Array out;
	if (!is_available() || pcount == 0) {
		return out;
	}
	// Full-capacity buffer: MultiMesh::set_buffer wants every instance slot. Slots
	// past the live count hold stale data but the caller only makes `pcount`
	// instances visible, so they are never drawn.
	Vector<uint8_t> raw = rd->buffer_get_data(buf_mm);
	out.resize(capacity * 12);
	memcpy(out.ptrw(), raw.ptr(), MIN((int)raw.size(), (int)(capacity * 12 * sizeof(float))));
	return out;
}

PackedVector3Array MPMFluidSolver::get_positions() const {
	PackedVector3Array out;
	if (!is_available() || pcount == 0) {
		return out;
	}
	Vector<uint8_t> raw = rd->buffer_get_data(buf_particles);
	const float *f = (const float *)raw.ptr();
	out.resize(pcount);
	for (int i = 0; i < pcount; i++) {
		const float *o = f + i * FLOATS_PER_PARTICLE;
		out.write[i] = Vector3(o[0], o[1], o[2]);
	}
	return out;
}

int MPMFluidSolver::get_surface_mesh(PackedVector3Array &r_vertices, PackedVector3Array &r_normals) const {
	r_vertices.clear();
	r_normals.clear();
	if (!is_available() || buf_mcount.is_null()) {
		return 0;
	}
	Vector<uint8_t> cnt = rd->buffer_get_data(buf_mcount, 0, sizeof(uint32_t));
	if (cnt.size() < (int)sizeof(uint32_t)) {
		return 0;
	}
	const uint32_t tris = MIN(decode_uint32(cnt.ptr()), TRI_BUDGET);
	if (tris == 0) {
		return 0;
	}
	const uint32_t verts = tris * 3;
	Vector<uint8_t> vraw = rd->buffer_get_data(buf_mverts, 0, verts * 4 * sizeof(float));
	Vector<uint8_t> nraw = rd->buffer_get_data(buf_mnorms, 0, verts * 4 * sizeof(float));
	const float *vf = (const float *)vraw.ptr();
	const float *nf = (const float *)nraw.ptr();
	r_vertices.resize(verts);
	r_normals.resize(verts);
	for (uint32_t i = 0; i < verts; i++) {
		r_vertices.write[i] = Vector3(vf[i * 4 + 0], vf[i * 4 + 1], vf[i * 4 + 2]);
		r_normals.write[i] = Vector3(nf[i * 4 + 0], nf[i * 4 + 1], nf[i * 4 + 2]);
	}
	return (int)tris;
}

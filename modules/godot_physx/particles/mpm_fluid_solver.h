/**************************************************************************/
/*  mpm_fluid_solver.h                                                    */
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

#pragma once

#include "core/math/quaternion.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/math/vector3i.h"
#include "core/object/ref_counted.h"
#include "core/os/mutex.h"
#include "core/templates/local_vector.h"
#include "core/templates/rid.h"
#include "core/templates/safe_refcount.h"
#include "core/variant/variant.h"

class RenderingDevice;

// A cross-vendor MLS-MPM fluid on plain compute shaders (RenderingDevice, no
// CUDA). This is the fallback backend for PhysXParticleFluid3D when the PhysX GPU
// (PBD / CUDA) path is unavailable -- it runs on any Vulkan/Metal/D3D12 device.
//
// Weakly-compressible APIC / MLS-MPM: five compute passes per substep (clear
// grid, P2G mass, P2G momentum, grid update, G2P) plus an optional rigid
// coupling pass (analytic colliders as moving velocity boundary conditions) and
// a render pass that packs positions into a MultiMesh instance buffer.
//
// The solver runs on the engine's *main* RenderingDevice. configure() / emit() /
// step() only do CPU prep on the calling thread, then post the GPU work to the
// render thread via RenderingServer::call_on_render_thread(). Results (collider
// reaction impulses, isosurface geometry, the packed render buffer) come back
// via RenderingDevice::buffer_get_data_async() a few frames later and are
// cached. Nothing stalls the caller; visuals and coupling run ~2-3 frames
// behind, which is imperceptible.

// GPU-side state plus every render-thread entry point. Held as a Ref by
// MPMFluidSolver. Each deferred call and async readback binds a Ref to this
// object into its Callable, so it outlives any work still in flight; the last
// drop runs the destructor (usually on the render thread) which frees the RIDs.
class MPMFluidSolverGPU : public RefCounted {
	GDSOFTCLASS(MPMFluidSolverGPU, RefCounted);

public:
	enum Pass { PASS_CLEAR,
		PASS_P2G_MASS,
		PASS_P2G_MOM,
		PASS_GRID,
		PASS_COUPLE,
		PASS_G2P,
		PASS_SURFACE,
		PASS_MARCH,
		PASS_RENDER,
		// Block-sparse fluid path (mpm_bs_*.glsl). Fluid only; granular uses the
		// dense passes above.
		PASS_BS_CLEAR,
		PASS_BS_BINSERT,
		PASS_BS_BDISPATCH,
		PASS_BS_CLEARNODES,
		PASS_BS_P2G_MASS,
		PASS_BS_P2G_MOM,
		PASS_BS_GRID,
		PASS_BS_COUPLE,
		PASS_BS_G2P,
		PASS_BS_SURFACE,
		PASS_BS_MARCH,
		PASS_BS_RENDER,
		PASS_MAX };

	RenderingDevice *rd = nullptr;
	// Headless (no main RenderingDevice) falls back to a private local device we
	// own and drive synchronously; windowed/editor uses the shared main device
	// via the render thread with async readback.
	bool local = false;
	SafeFlag built; // the current build job has finished

	RID shader[PASS_MAX];
	RID pipeline[PASS_MAX];
	RID uset[PASS_MAX];
	RID uset_mesh; // set 1 for the march pass

	RID buf_params;
	RID buf_particles;
	RID buf_grid_i;
	RID buf_grid_v;
	RID buf_colliders;
	RID buf_cimp;
	RID buf_mm;
	RID buf_surf;
	RID buf_mverts;
	RID buf_mnorms;
	RID buf_mcount;
	RID buf_bhash; // block-sparse: block keys (open addressing)
	RID buf_bhash_val; // hash slot -> block-pool slot
	RID buf_bkey; // block key per block-pool slot
	RID buf_bcounts; // [0] active blocks, [1..3] indirect dispatch (needs DISPATCH_INDIRECT usage)

	bool block_sparse = false; // this build uses the mpm_bs_* passes (fluid, boundless-capable)
	int capacity = 0;
	int node_count = 0; // dense: grid cells. block-sparse: cells in the M-b1 box (BC reference)
	int max_blocks = 0; // block-sparse: block-pool capacity
	int hash_slots = 0; // block-sparse: hash table size (pow2)
	int tri_budget = 0;

	// Readback caches. cache_mtx guards all of them.
	Mutex cache_mtx;
	PackedFloat32Array mm_cache;
	LocalVector<Vector3> imp_cache;
	PackedVector3Array surf_verts_cache;
	PackedVector3Array surf_norms_cache;
	int surf_tris_cache = 0;
	PackedVector3Array pos_cache;
	SafeFlag pos_ready;
	uint64_t last_step_usec = 0;

	// All posted to the render thread via RenderingServer::call_on_render_thread.
	// Every call binds a Ref to this object (p_self) so it outlives work in
	// flight; args are individually Variant-marshalled (no struct payloads).
	void rt_compile(Ref<MPMFluidSolverGPU> p_self);
	void rt_build(Ref<MPMFluidSolverGPU> p_self, PackedByteArray p_params, PackedByteArray p_particles, int p_capacity, int p_node_count, int p_tri_budget, bool p_block_sparse, int p_max_blocks, int p_hash_slots);
	void rt_step(Ref<MPMFluidSolverGPU> p_self, PackedByteArray p_params, PackedByteArray p_colliders, int p_ncol, int p_pcount, int p_node_count, Vector3i p_grid_dims, int p_substeps, bool p_want_surface, bool p_bench);
	void rt_emit(Ref<MPMFluidSolverGPU> p_self, PackedByteArray p_blob, int p_head_bytes, int p_first_bytes);
	void rt_read_positions(Ref<MPMFluidSolverGPU> p_self, int p_count);
	void rt_free(Ref<MPMFluidSolverGPU> p_self);

	// Async readback callbacks: RenderingDevice invokes them with the data as the
	// runtime arg; Callable::bind APPENDS the bound args, so the data comes first.
	void rt_on_mm(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self);
	void rt_on_impulses(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self, int p_ncol);
	void rt_on_surf_count(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self);
	void rt_on_surf_verts(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self);
	void rt_on_surf_norms(const PackedByteArray &p_data, Ref<MPMFluidSolverGPU> p_self);

	~MPMFluidSolverGPU();

private:
	bool _shaders_ok = false;
	bool _submitted = false; // local path: work in flight since the last sync
	int _submitted_ncol = 0;
	bool _submitted_surface = false;
	int _last_surf_tris = 0; // running estimate used to size the async vert/normal reads
	void _rt_free_buffers();
	void _rt_rebuild_uniform_sets();
	// Local path only: sync and pull mm / impulses / surface straight back.
	void _local_reap();
};

class MPMFluidSolver {
public:
	struct Settings {
		int particle_target = 60000;
		int substeps = 5;
		int grid_res = 48;
		float stiffness = 6000.0f;
		float viscosity = 0.65f;
		float rest_density = 1000.0f;
		float collider_friction = 0.25f;
		float surface_iso = 0.5f; // isosurface level as a fraction of the native packed kernel density
		float surface_kernel = 0.06f; // isosurface SPH kernel radius, world meters (from particle_size)
		float surface_boost = 1.0f; // per-particle mass multiplier in the surface scatter -- inflates the mesh
		// Granular (Drucker-Prager sand / snow) instead of a fluid EOS.
		bool granular = false;
		float granular_hardness = 1.5e5f; // Young's modulus (Pa); softer = more stable, mushier
		float granular_friction_deg = 35.0f; // internal friction angle -> angle of repose
		float granular_cohesion = 0.0f; // 0 = dry sand; small values -> wet sand / packing snow
		Vector3 gravity = Vector3(0, -9.8f, 0);
		Vector3 domain = Vector3(3, 3, 3); // MPM sim box / boundary, centered on the solver transform
		Vector3 spawn_region = Vector3(1, 1, 1); // prefill fills this, centered on the transform (clamped to the domain)
	};

	// An analytic collider coupled to the fluid.
	enum ColliderShape { COLLIDER_SPHERE,
		COLLIDER_BOX,
		COLLIDER_PLANE,
		COLLIDER_CAPSULE };
	struct Collider {
		ColliderShape shape = COLLIDER_SPHERE;
		Vector3 position; // world center (or a point on the plane)
		Vector3 extents; // sphere: x=radius | box: half-extents | plane: unit normal | capsule: (radius, half-height, -)
		Quaternion rotation; // box / capsule orientation
		Vector3 velocity;
	};
	using SphereCollider = Collider; // transitional alias

	MPMFluidSolver();
	~MPMFluidSolver();

	// A RenderingDevice is present (independent of whether a particle block has
	// been seeded or the pipelines have finished compiling).
	bool has_device() const { return gpu.is_valid() && gpu->rd != nullptr; }
	// Ready to step: device up and the render thread has finished building buffers
	// for the current configure().
	bool is_available() const { return has_device() && gpu->built.is_set(); }

	// (Re)build buffers. `p_prefill` seeds a full jittered block at rest (drop-in-
	// a-tank); otherwise the buffer starts empty and fills via emit() (a faucet).
	// `p_xform` places the domain center in world. The GPU build runs on the
	// render thread; is_available() flips true a frame or two later.
	void configure(const Settings &p_settings, const Transform3D &p_xform, bool p_prefill = true);

	// Move the domain center without reseeding.
	void set_domain_transform(const Transform3D &p_xform);

	// Live isosurface tweak -- no reseed.
	void set_surface_params(float p_iso, float p_kernel, float p_boost) {
		settings.surface_iso = p_iso;
		settings.surface_kernel = p_kernel;
		settings.surface_boost = p_boost;
		_recompute_surface_iso();
	}

	struct EmittedParticle {
		Vector3 position; // world
		Vector3 velocity;
	};
	// Insert new particles at the ring write head, overwriting the oldest once the
	// buffer is full.
	void emit(const LocalVector<EmittedParticle> &p_new);

	// Advance one rendered frame. `r_impulses`, when non-null, is filled with the
	// most recently reaped per-collider reaction impulse (world space, N*s). With
	// the async readback that is ~2-3 frames stale.
	// `p_want_surface` also runs the density-smoothing + GPU-march passes.
	// `p_async` is retained for source compatibility and ignored -- the shared
	// RenderingDevice cannot be synced on demand, so every step is async.
	void step(double p_delta, const LocalVector<SphereCollider> &p_colliders, LocalVector<Vector3> *r_impulses, bool p_want_surface = false, bool p_async = true);

	int get_particle_count() const { return pcount; } // live particles
	int get_capacity() const { return capacity; } // buffer / MultiMesh size
	double get_last_step_msec() const;

	// 12 floats / particle: MultiMesh TRANSFORM_3D rows, from the last reaped
	// step. Empty until the first readback lands.
	PackedFloat32Array get_multimesh_buffer() const;
	// Blocking: forces a render sync. Test / query helper only.
	PackedVector3Array get_positions() const;

	// GPU-marched isosurface geometry from the last reaped step run with
	// p_want_surface = true. Non-indexed triangle soup, world space; returns the
	// triangle count.
	int get_surface_mesh(PackedVector3Array &r_vertices, PackedVector3Array &r_normals) const;

private:
	Ref<MPMFluidSolverGPU> gpu;

	Settings settings;
	Transform3D domain_xform;

	int pcount = 0; // live particles (== capacity when prefilled)
	int capacity = 0; // particle-buffer slots
	int write_head = 0; // next slot emit() overwrites once full
	int node_count = 0;
	int max_blocks = 0; // block-sparse fluid path
	int hash_slots = 0;
	Vector3i grid_dims; // per-axis cell count (domain can be non-cube; dx is uniform)
	float dx = 0.0f;
	float pmass = 0.0f;
	float surf_iso_density = 500.0f; // settings.surface_iso resolved to a kg/m^3 threshold

	void _compute_scales();
	void _recompute_surface_iso();
	LocalVector<float> _seed_block(int &r_count) const;
	void _pack_params(double p_dt, int p_ncol, PackedByteArray &r_bytes) const;
	void _pack_colliders(const LocalVector<SphereCollider> &p_colliders, PackedByteArray &r_bytes) const;
	// Run render work now (local device) or hand it to the render thread (shared).
	void _dispatch(const Callable &p_call) const;
};

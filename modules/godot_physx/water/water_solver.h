/**************************************************************************/
/*  water_solver.h                                                       */
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

#include "core/math/vector2.h"
#include "core/math/vector3.h"
#include "core/object/ref_counted.h"
#include "core/os/mutex.h"
#include "core/templates/rid.h"
#include "core/templates/safe_refcount.h"
#include "core/templates/vector.h"
#include "core/variant/variant.h"

class RenderingDevice;

/* ===================================================================== */
/*  WaterSolverGPU -- everything that touches the RenderingDevice. Every   */
/*  method here runs on the render thread (posted via                      */
/*  RenderingServer::call_on_render_thread) except the destructor, which   */
/*  runs wherever the last Ref is dropped. Mirrors MPMFluidSolverGPU's     */
/*  exact shape (see mpm_fluid_solver.h) -- shared device when windowed    */
/*  (async, dispatched via the render thread), a private local device when */
/*  headless (synchronous, run on the calling thread directly).            */
/* ===================================================================== */
class WaterSolverGPU : public RefCounted {
	GDSOFTCLASS(WaterSolverGPU, RefCounted);

public:
	RenderingDevice *rd = nullptr;
	bool local = false; // headless fallback: a private device, driven synchronously
	SafeFlag built;

	static constexpr int MAX_SPHERES = 48;
	static constexpr int MAX_IMPULSES = 16;

	RID shader_ripple, pipeline_ripple;
	RID shader_spectrum_init, pipeline_spectrum_init;
	RID shader_spectrum_evolve, pipeline_spectrum_evolve;
	RID shader_fft, pipeline_fft;
	// Blit-to-texture: the only new shader shape in this module -- writes a
	// storage buffer's contents into a real RD image2D (R32_SFLOAT), which
	// Texture2DRD then wraps for direct, zero-copy sampling by the scene
	// renderer. Every earlier solver in this module only ever produced
	// buffers for more compute or a packed MultiMesh; this is genuinely new
	// territory, flagged as such in the project plan.
	RID shader_blit_ripple, pipeline_blit_ripple;
	RID shader_blit_ocean, pipeline_blit_ocean;

	RID buf_params;
	RID buf_state_a, buf_state_b; // ping-pong ripple state (h, h_prev, foam, _)
	RID buf_height; // composed ripple height, full grid resolution
	RID buf_spheres, buf_impulses;
	RID uset_atob, uset_btoa;

	RID buf_ocean_init_params, buf_ocean_evolve_params, buf_fft_params;
	RID buf_h0, buf_ocean_spec, buf_fft_a, buf_fft_b;
	RID uset_spectrum_init, uset_spectrum_evolve, uset_fft_spec_to_a, uset_fft_atob, uset_fft_btoa;

	RID tex_ripple_height, tex_ocean_height; // real RD textures, wrapped by Texture2DRD on the caller side
	RID buf_blit_ripple_params, buf_blit_ocean_params; // just {n}, uploaded once at build (grid resolution is fixed per configure())
	RID uset_blit_ripple, uset_blit_ocean;

	int ocean_log2n = 0;
	double ocean_time_accum = 0.0;
	bool a_is_current = true;

	// CPU readback caches -- see rt_read_height()/rt_on_*_height(). Guarded
	// by cache_mtx, same convention as mpm_fluid_solver.h's mm_cache et al.
	Mutex cache_mtx;
	Vector<float> ripple_height_cache;
	Vector<float> ocean_height_cache;
	Vector<float> ocean_imag_cache;
	SafeFlag height_ready;

	// All posted to the render thread via RenderingServer::call_on_render_thread
	// (or run synchronously now, on the calling thread, for the local/headless
	// path -- see WaterSolver::_dispatch()). Every call binds a Ref to this
	// object (p_self) so it outlives work in flight.
	void rt_compile(Ref<WaterSolverGPU> p_self);
	// Wind/amplitude only matter here -- they feed the ONE-TIME h0 spectrum
	// generation (see rt_build()'s body), not the per-step evolution.
	void rt_build(Ref<WaterSolverGPU> p_self, int p_grid_resolution, Vector2 p_domain_size, int p_ocean_grid_resolution, Vector2 p_ocean_domain_size, float p_wind_speed, Vector2 p_wind_direction, float p_wave_amplitude, float p_gravity);
	// p_spheres/p_impulses: flat, 4 floats/entry (world_x, world_z, radius,
	// strength), packed on the calling thread into an immutable value-copy
	// snapshot before dispatch -- same thread-safety pattern
	// MPMFluidSolver::step()'s p_colliders argument already uses, not a
	// pointer/reference into WaterSolver's own live sphere_slots (which live
	// on the calling thread, not the render thread). Also kicks the height
	// readback unconditionally every call (matching MPMFluidSolverGPU::
	// rt_step()'s own precedent -- no separate throttle needed, the async
	// callback landing "a few frames later" already provides natural,
	// implicit throttling).
	void rt_step(Ref<WaterSolverGPU> p_self, double p_delta, float p_depth, float p_damping, float p_gravity, float p_water_level, PackedFloat32Array p_spheres, PackedFloat32Array p_impulses);
	void rt_free(Ref<WaterSolverGPU> p_self);

	// Async readback callbacks: RenderingDevice invokes them with the data as
	// the runtime arg; Callable::bind APPENDS the bound args, so the data
	// comes first. Two separate reads/callbacks (ripple, ocean), same
	// pattern as MPMFluidSolverGPU's rt_on_mm/rt_on_impulses/rt_on_surf_* --
	// no ordering dependency between them, whichever lands first updates the
	// cache immediately.
	void rt_on_ripple_height(const PackedByteArray &p_data, Ref<WaterSolverGPU> p_self);
	void rt_on_ocean_height(const PackedByteArray &p_data, Ref<WaterSolverGPU> p_self);

	~WaterSolverGPU();

private:
	bool shaders_ok = false;
	int grid_resolution = 0;
	Vector2 domain_size;
	int ocean_grid_resolution = 0;
	Vector2 ocean_domain_size;

	// Static settings the one-time h0 spectrum init needs, stashed here by
	// rt_build() between being called and _rt_build_buffers() running.
	float _init_wind_speed = 8.0f;
	Vector2 _init_wind_direction = Vector2(1, 0);
	float _init_wave_amplitude = 1.0f;
	float _init_gravity = 9.81f;

	void _rt_free_buffers();
	void _rt_build_buffers();
};

/* ===================================================================== */
/*  WaterSolver -- CPU-side front. Preps payloads, posts render work.      */
/*  Same public shape as Phase 1/2's original single-class WaterSolver,   */
/*  so WaterRippleProbe/PhysXWaterSurface3D don't need to change how they */
/*  call it -- only the internals moved to the shared-device/async model. */
/* ===================================================================== */
class WaterSolver {
public:
	struct Settings {
		int grid_resolution = 128; // N, ripple grid cells per side (power of two not required for the ripple layer)
		Vector2 domain_size = Vector2(20, 20); // world-space extents (full width/depth, not half)
		float depth = 3.0f; // used by the FFT layer's dispersion relation
		// ALPHA in water_ripple.glsl. Originally tuned to 0.5 purely so the
		// Phase 1 probe test's impulse would settle within ~90 steps -- that
		// was optimizing for a fast, bounded test, not for how real water
		// actually looks. At 0.5, ripples die out almost as fast as they're
		// created and never travel far, reading as thick/viscous ("jello or
		// syrup", a real user observation) rather than water. Lowered so
		// ripples persist and travel a meaningfully longer distance before
		// dissipating -- see the probe test for the real before/after decay
		// numbers this was checked against, including over a longer horizon
		// than Phase 1's original ~90-step window to confirm it stays
		// bounded/stable, not just slower to settle.
		float damping = 0.08f;
		float gravity = 9.81f;
		float water_level = 0.0f; // mean surface world Y

		// FFT ocean spectrum layer (Phase 2). Independent grid/resolution from
		// the ripple layer above -- see water_solver.h's own note on why these
		// stay architecturally separate. MUST be a power of two -- the
		// Stockham FFT passes require it.
		int ocean_grid_resolution = 64;
		Vector2 ocean_domain_size = Vector2(40, 40);
		float wind_speed = 8.0f;
		Vector2 wind_direction = Vector2(1, 0); // need not be normalized, normalized on upload
		float wave_amplitude = 1.0f; // Phillips spectrum's A constant
	};

	static constexpr int MAX_SPHERES = WaterSolverGPU::MAX_SPHERES;
	static constexpr int MAX_IMPULSES = WaterSolverGPU::MAX_IMPULSES;

	WaterSolver();
	~WaterSolver();

	// A RenderingDevice is present (independent of whether configure() has
	// finished building on the render thread yet).
	bool has_device() const { return gpu.is_valid() && gpu->rd != nullptr; }
	// Ready to step: device up and the render thread has finished building
	// buffers/textures for the current configure().
	bool is_available() const { return has_device() && gpu->built.is_set(); }

	void configure(const Settings &p_settings);
	void step(double p_delta);

	// Persistent body proxy, world space, keyed by an arbitrary caller-chosen
	// id (ObjectID in practice) -- call every physics tick while the body is
	// in the water; entries not resubmitted this step stay at their last
	// value (no implicit aging -- callers should call clear_sphere() in
	// _exit_tree() for the clean-despawn case).
	void submit_sphere(uint64_t p_owner, const Vector3 &p_world_pos, float p_radius, float p_strength = 1.0f);
	void clear_sphere(uint64_t p_owner);

	// One-shot, event-driven (a splash) -- queued, consumed by the next
	// step(), then cleared. Not persistent like spheres.
	void submit_impulse(const Vector3 &p_world_pos, float p_radius, float p_strength);

	// Zero-copy RD texture RIDs (R32_SFLOAT height, meters) for a caller-side
	// Texture2DRD to wrap directly -- valid once is_available(). This is the
	// real payoff of the shared-device design: the scene renderer samples
	// these with no CPU round trip at all.
	RID get_ripple_height_texture_rd_rid() const;
	RID get_ocean_height_texture_rd_rid() const;
	Vector2 get_domain_size() const { return settings.domain_size; }
	Vector2 get_ocean_domain_size() const { return settings.ocean_domain_size; }

	// CPU-side cached readback, for buoyancy sampling (a consumer with very
	// different freshness/precision needs than rendering -- see the project
	// plan's note on why these stay separate paths from the zero-copy
	// texture above, not a redundant "same data twice"). Returns whatever
	// the last async callback landed (a few frames stale, same tradeoff
	// MPM's own mm_cache/pos_cache accept) -- cheap (a mutex lock + copy),
	// no dispatch, no sync, safe to call every physics tick from any number
	// of callers.
	void get_height_grid(Vector<float> &r_height, int &r_n, Vector2 &r_domain_size) const;
	void get_ocean_height_grid(Vector<float> &r_height, Vector<float> &r_imag, int &r_n, Vector2 &r_domain_size) const;

	// Forces the render thread to catch up right now (matches
	// MPMFluidSolver::get_positions()'s own "blocking, test/query helper"
	// precedent) -- for tests wanting a deterministic "definitely landed by
	// now" read after step(); real per-tick callers should NOT call this
	// (defeats the whole point of the async design) and should just call
	// get_height_grid()/get_ocean_height_grid() directly.
	void sync_now() const;

private:
	Ref<WaterSolverGPU> gpu;
	Settings settings;

	struct SphereSlot {
		uint64_t owner = 0;
		Vector3 world_pos;
		float radius = 0.0f;
		float strength = 1.0f;
		bool enabled = false;
	};
	SphereSlot sphere_slots[MAX_SPHERES];

	struct ImpulseSlot {
		Vector3 world_pos;
		float radius = 0.0f;
		float strength = 0.0f;
	};
	Vector<ImpulseSlot> pending_impulses;

	PackedFloat32Array _pack_spheres() const;
	PackedFloat32Array _pack_impulses() const;

	// Run render work now (local device) or hand it to the render thread
	// (shared device).
	void _dispatch(const Callable &p_call) const;
};

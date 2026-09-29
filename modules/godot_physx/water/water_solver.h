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
#include "core/templates/rid.h"
#include "core/templates/vector.h"

class RenderingDevice;

// Phase 1 of the water solver: a local iWave-style ripple layer only (2D
// damped wave equation + sphere/impulse body-disturbance coupling), port of
// caustic-volume's simulation half -- see water_ripple.glsl and the "Scoping:
// FFT ocean + ripple water solver" plan section for the full architecture.
// The FFT ocean spectrum layer (Phase 2) composes with this one later; for
// now this layer's own height IS the water surface.
//
// Owns a private local RenderingDevice and runs fully synchronously (submit +
// sync every step) -- same simplification GasSolver already makes over MPM
// fluid's shared-device/render-thread/async-readback machinery, and for the
// same reason (this solver's cell counts don't need a tight per-frame
// overlap budget). Confirmed empirically this session:
// RenderingServer::create_local_rendering_device() returns null under
// --headless, same as the shared device -- there is no headless path for any
// RenderingDevice compute in this engine build, this solver included. Any
// test of real behavior must run windowed.
class WaterSolver {
public:
	struct Settings {
		int grid_resolution = 128; // N, ripple grid cells per side (power of two not required for Phase 1, unlike the later FFT layer)
		Vector2 domain_size = Vector2(20, 20); // world-space extents (full width/depth, not half)
		float depth = 3.0f; // reserved for Phase 2's FFT dispersion relation; unused by the ripple layer itself
		float damping = 0.5f; // ALPHA in water_ripple.glsl
		float gravity = 9.81f;
		float water_level = 0.0f; // mean surface world Y
	};

	static constexpr int MAX_SPHERES = 48;
	static constexpr int MAX_IMPULSES = 16;

	WaterSolver();
	~WaterSolver();

	bool is_available() const { return rd != nullptr && shaders_ok; }
	bool is_configured() const { return !buf_state_a.is_null(); }

	void configure(const Settings &p_settings);
	void step(double p_delta);

	// Persistent body proxy, world space, keyed by an arbitrary caller-chosen
	// id (ObjectID in practice) -- call every physics tick while the body is
	// in the water; entries not resubmitted this step stay at their last
	// value (no implicit aging in Phase 1 -- see the plan's note that
	// buoyant_body.gd should call clear_sphere() in _exit_tree() for the
	// clean-despawn case).
	void submit_sphere(uint64_t p_owner, const Vector3 &p_world_pos, float p_radius, float p_strength = 1.0f);
	void clear_sphere(uint64_t p_owner);

	// One-shot, event-driven (a splash) -- queued, consumed by the next
	// step(), then cleared. Not persistent like spheres.
	void submit_impulse(const Vector3 &p_world_pos, float p_radius, float p_strength);

	// Synchronous full-grid readback (blocking -- fine at ripple-grid cell
	// counts, same tradeoff GasSolver::get_density_grid already makes; not
	// meant for a tight per-frame budget). r_height is r_n*r_n, row-major
	// (index = y*r_n + x); grid cell (x,y)'s world position is
	// domain center + ((x+0.5)/r_n*2-1, (y+0.5)/r_n*2-1) * r_domain_size/2.
	void get_height_grid(Vector<float> &r_height, int &r_n, Vector2 &r_domain_size) const;

private:
	RenderingDevice *rd = nullptr;
	bool shaders_ok = false;

	RID shader_ripple, pipeline_ripple;

	RID buf_params;
	RID buf_state_a, buf_state_b; // ping-pong ripple state (h, h_prev, foam, _)
	RID buf_height; // composed height output, full grid resolution
	RID buf_spheres, buf_impulses;

	RID uset_atob, uset_btoa; // ping-pong: read/write swap each step

	Settings settings;
	bool a_is_current = true;

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

	void _compile_shaders();
	void _free_buffers();
	void _build_buffers();
	void _upload_params(double p_dt);
	void _upload_bodies();
};

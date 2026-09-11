/**************************************************************************/
/*  gas_solver.h                                                          */
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

#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/math/vector3i.h"
#include "core/templates/rid.h"
#include "core/templates/vector.h"

class RenderingDevice;

// A sparse-grid Eulerian gas/smoke solver (velocity + density field, block-
// hash sparse grid), on plain compute shaders. Shares its block-hash lookup
// technique with the MPM block-sparse fluid grid (see gas_block_inc.glsl for
// the full design writeup) but the field is persistent -- advected and
// diffused in place across steps -- rather than rebuilt from particles every
// substep. v1 scope (see gas_block_inc.glsl header): the box's lateral (X/Z)
// footprint is fixed at configure() -- only its height grows automatically
// (see _grow_if_needed), up to 3x the configured height -- no pressure
// projection, up to MAX_COLLIDERS sphere colliders.
//
// Owns a private local RenderingDevice and runs fully synchronously (submit +
// sync every dispatch) -- simpler than the MPM fluid solver's shared-device /
// render-thread / async-readback machinery, at the cost of not overlapping
// with the main render thread's own GPU work. A reasonable place to start;
// revisit if this solver needs the same treatment MPM's Stage A/B did.
class GasSolver {
public:
	struct Settings {
		Vector3i box_blocks = Vector3i(6, 10, 6); // grid size in 4^3 blocks
		float cell_size = 0.08f;
		Vector3 source_position; // local, relative to the solver's transform
		float source_radius = 0.22f;
		Vector3 source_velocity = Vector3(0, 1.4f, 0);
		float source_density = 1.0f;
		float buoyancy = 9.0f;
		float vorticity_strength = 8.0f;
		float dissipation = 0.996f;
	};

	static constexpr int MAX_COLLIDERS = 4;

	GasSolver();
	~GasSolver();

	bool is_available() const { return rd != nullptr && shaders_ok; }

	// (Re)builds the grid for these settings and touches every block in the
	// fixed box. p_xform places the box in world space (its -half-extent
	// corner sits at the box's minimum corner); called again to resize.
	void configure(const Settings &p_settings, const Transform3D &p_xform);

	// Advance one step. p_xform updates the collider/source world positions
	// (both are stored local to the solver's transform) without reseeding.
	void step(double p_delta, const Transform3D &p_xform);

	// Up to MAX_COLLIDERS voxelized-solid sphere obstacles, world space. Extra
	// positions beyond p_radii's size (or a radius <= 0) disable that slot.
	void set_colliders(const Vector<Vector3> &p_world_positions, const Vector<float> &p_radii);

	// Synchronous readback: cell world positions + density for every active
	// cell above p_density_threshold. Used by the node to build a MultiMesh.
	// Blocking -- fine at this solver's cell counts (tens of thousands), not
	// meant for a tight per-frame budget the way the async MPM path is.
	void get_render_cells(float p_density_threshold, Vector<Vector3> &r_positions, Vector<float> &r_density) const;

private:
	RenderingDevice *rd = nullptr;
	bool shaders_ok = false;

	RID shader_touch, shader_clear, shader_clear_range, shader_curl, shader_step;
	RID pipeline_touch, pipeline_clear, pipeline_clear_range, pipeline_curl, pipeline_step;

	RID buf_params;
	RID buf_bhash, buf_bhash_val, buf_bkey, buf_bcounts;
	RID buf_grid_a, buf_grid_b;
	RID buf_curl;

	RID uset_touch, uset_clear;
	RID uset_curl_a, uset_curl_b; // read grid A (resp. B) as "current", write curl
	RID uset_step_atob, uset_step_btoa; // ping-pong: read/write swap each step

	Settings settings;
	Transform3D domain_xform;
	Vector3 grid_anchor; // world position of cell (0,0,0)'s min corner, frozen at configure()

	// World-space; unlike source_position these are not re-derived from a
	// local offset each step (a collider is an external node position handed
	// in directly by the caller each step, not part of Settings).
	Vector3 collider_world_positions[MAX_COLLIDERS];
	float collider_radii[MAX_COLLIDERS] = {};

	// Dynamic vertical growth: only ever adds blocks at the TOP of the box
	// (settings.box_blocks.y increases; grid_anchor and every already-touched
	// cell's coordinate stay valid, so this never disturbs existing smoke --
	// see gas_bs_clear_range.glsl for why the clear step is safe). Reserved
	// pool capacity is sized generously at configure() so growth never needs
	// to reallocate a buffer, only claim more of what's already allocated.
	int initial_box_blocks_y = 0;
	int max_box_blocks_y = 0; // cap so it stops trying once the reserved pool would be exhausted
	int steps_since_growth = 0;
	Vector3i pending_growth_range; // x = first newly-touched slot this pass, y = one past the last, z unused; (0,0,0) = no growth this upload
	void _grow_if_needed();

	int block_count = 0; // current active block count (grows over time)
	int cell_count = 0; // current active cell count (grows over time)
	int hash_slots = 0;
	int max_blocks = 0;
	bool a_is_current = true;
	double time_accum = 0.0;

	Vector<Vector3i> slot_block; // slot -> block coord, read back once after configure()

	void _compile_shaders();
	void _free_buffers();
	void _build_buffers();
	void _upload_params(double p_dt);
	void _read_slot_blocks();
	RID _make_uniform_set(RID p_shader, RID p_grid5, RID p_grid6) const;
	void _dispatch(RID p_pipeline, RID p_uset, int p_thread_count) const;
};

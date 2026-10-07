/**************************************************************************/
/*  flow_backend.h                                                        */
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

// The only file that talks to NVIDIA Flow's libraries. Flow's loader pulls
// in <windows.h>, so this boundary keeps Flow (and Windows) headers out of
// everything that includes Godot's own: plain types only, in and out.
//
// nvflow.dll / nvflowext.dll are opened at run time -- a build with Flow
// support still runs without them, Flow nodes just stay inert. Grids run on
// Godot's RenderingDevice (see flow_context_rd.h); their temperature and
// smoke can optionally come back to the CPU as NanoVDB grids.
//
// Units: Flow's tuned defaults assume about 10 units per meter (its gravity
// is -100, its default cell 0.5) -- everything here is in meters and
// converted with UNITS_PER_METER at this boundary.

#include <cstdint>

class RenderingDevice;

namespace FlowBackend {

constexpr float UNITS_PER_METER = 10.0f;

struct Grid;

struct Emitter {
	enum Shape {
		SHAPE_SPHERE,
		SHAPE_BOX,
	};
	Shape shape = SHAPE_SPHERE;
	// Godot Transform3D: basis columns, then origin (meters).
	float basis[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
	float origin[3] = { 0, 0, 0 };
	float radius = 0.5f; // sphere, local meters
	float half_size[3] = { 0.5f, 0.5f, 0.5f }; // box, local meters
	float velocity[3] = { 0, 0, 0 }; // local m/s
	float temperature = 1.0f;
	float fuel = 0.0f;
	float burn = 0.0f;
	float smoke = 1.0f;
	float divergence = 0.0f;
	float couple_rate = 2.0f;
	// A solid for the gas: zero smoke/heat inside, the gas takes on
	// body_velocity there. Allocates no grid of its own.
	bool collision = false;
	float body_velocity[3] = { 0, 0, 0 }; // world m/s
};

struct Settings {
	float cell_size = 0.05f; // meters
	float gravity[3] = { 0, -9.81f, 0 }; // m/s^2, sets "up" for buoyancy
	float buoyancy = 2.0f; // per unit temperature
	float cooling_rate = 1.5f;
	float smoke_fade = 0.65f;
	float vorticity = 0.6f;
	float ignition_temperature = 0.05f;
	bool combustion = true;
	// Copy temperature and smoke back to the CPU this step.
	bool readback = false;

	// Flow's renderer. The colormap maps temperature (0..temperature_range)
	// to linear RGB; opacity comes from smoke. Light direction points toward
	// the light.
	static constexpr int COLORMAP_POINTS = 32;
	float colormap_rgb[COLORMAP_POINTS][3] = {};
	float temperature_range = 2.0f;
	float attenuation = 4.0f; // per meter, per unit smoke
	float light_direction[3] = { 0, 1, 0 };
	float shadow_min_intensity = 0.125f;
	int shadow_steps = 48;
};

// Loads Flow's libraries on first use (shared by every grid); false, with
// the reason in r_error, if they aren't there.
bool acquire_library(char *r_error, int p_error_size);
void release_library();

// A grid on Godot's RenderingDevice. Must be called -- like every grid_*
// call -- on the device's (render) thread. The first one compiles Flow's
// pipelines (cached in Godot's shader cache folder afterwards).
// p_max_blocks: the sparse block budget; Flow allocates for all of it up front.
Grid *grid_create_rd(RenderingDevice *p_rd, uint32_t p_max_blocks);
// Draws a grid on RenderingDevice into Godot's frame with Flow's own ray
// marcher: depth / color are Godot's scene textures (RenderingDevice RID
// ids, width x height), view / projection are row-vector 4x4 matrices from
// Flow's world (its units) to Godot's clip space. Returns the RID id of the
// texture holding the scene color with the gas composited in (to copy back
// before the next Flow call), or 0.
uint64_t grid_render_rd(Grid *p_grid, const float p_view[16], const float p_projection[16], uint32_t p_width, uint32_t p_height, uint64_t p_depth_rid, uint32_t p_depth_width, uint32_t p_depth_height, uint64_t p_color_rid);
// Ends the render's Flow frame; call after the result has been copied.
void grid_render_rd_finish(Grid *p_grid);

// Diagnostics.
// Sparse blocks in use (of the grid's max_blocks).
uint32_t grid_active_blocks(Grid *p_grid);
bool grid_rd_stats(Grid *p_grid, int &r_pipelines, int &r_pipeline_failures, int &r_passes, int &r_buffers, int &r_textures, uint64_t &r_frame, uint64_t &r_completed);
void grid_destroy(Grid *p_grid);
// One simulation step. Emitter order must stay stable between steps (Flow
// tracks each emitter's previous transform by index, for motion blur and
// moving-collider velocity). False if skipped (the libraries are busy on
// another thread).
bool grid_step(Grid *p_grid, double p_time, float p_delta, const Settings &p_settings, const Emitter *p_emitters, int p_emitter_count);
// The newest completed readback; false until the first one lands. The
// pointers stay valid until the next grid_step.
bool grid_get_readback(Grid *p_grid, const uint8_t **r_temperature, uint64_t *r_temperature_size, const uint8_t **r_smoke, uint64_t *r_smoke_size);

// World-space bounds (meters) of a NanoVDB grid's active region; false if
// it's empty.
bool nanovdb_bounds(const uint8_t *p_nanovdb, uint64_t p_size, float r_min[3], float r_max[3]);

// Samples a NanoVDB float grid at the centers of a dense dims[0..2] box of
// cells (x fastest) whose min corner is p_origin (meters), p_cell meters
// apart -- only z slices [p_z_begin, p_z_end). Values land p_stride floats
// apart in r_out (indexed over the whole box, not the slice range).
void sample_dense(const uint8_t *p_nanovdb, uint64_t p_size, const float p_origin[3], float p_cell, const int p_dims[3], int p_z_begin, int p_z_end, float *r_out, int p_stride);

} // namespace FlowBackend

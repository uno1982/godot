/**************************************************************************/
/*  physx_water_surface_3d.h                                             */
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

#include "water_solver.h"

#include "scene/3d/node_3d.h"

class Mesh;
class MeshInstance3D;
class ShaderMaterial;
class Texture2DRD;
class Texture2D;

// The real water surface node: owns a WaterSolver (ripple + FFT ocean
// layers), renders it via a plain child MeshInstance3D (a subdivided
// PlaneMesh + a spatial ShaderMaterial that samples the solver's height
// fields), and exposes the sample_height()/submit_sphere()/submit_impulse()
// API buoyant_body.gd (or anything else) talks to.
//
// WaterSolver runs on the SHARED main RenderingDevice (dispatched via the
// render thread, same as MPMFluidSolverGPU) and blits its height fields into
// real RD textures every step. Those textures are wrapped here with
// Texture2DRD (a real, already-public Godot class -- see
// scene/resources/texture_rd.h -- no vanilla-engine changes needed) and fed
// straight to the ShaderMaterial: zero-copy, no CPU round trip, the data
// never leaves the GPU. Set once after the solver becomes available, not
// re-created every frame -- Texture2DRD just wraps an RID by reference, so
// whatever the blit shaders write each step is what the material samples
// automatically.
//
// Buoyancy sampling (sample_height()) is a separate, second consumer of the
// same underlying data with very different needs (CPU-side, a few frames of
// staleness is fine, precision doesn't need to match the rendered mesh
// exactly) -- it still goes through the solver's CPU-cached
// get_height_grid()/get_ocean_height_grid() readback, refreshed periodically
// into a local array this node bilinearly samples. Kept deliberately
// separate from the zero-copy texture path above per the project plan's own
// note: two consumers, two different freshness/precision needs, one
// underlying simulation.
class PhysXWaterSurface3D : public Node3D {
	GDCLASS(PhysXWaterSurface3D, Node3D);

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	void set_domain_size(Vector2 p_size);
	Vector2 get_domain_size() const { return domain_size; }
	void set_grid_resolution(int p_n);
	int get_grid_resolution() const { return grid_resolution; }
	void set_depth(float p_depth);
	float get_depth() const { return depth; }
	void set_water_level(float p_level);
	float get_water_level() const { return water_level; }
	void set_damping(float p_damping);
	float get_damping() const { return damping; }
	void set_ripple_amplitude(float p_amplitude);
	float get_ripple_amplitude() const { return ripple_amplitude; }

	void set_ocean_grid_resolution(int p_n);
	int get_ocean_grid_resolution() const { return ocean_grid_resolution; }
	void set_ocean_domain_size(Vector2 p_size);
	Vector2 get_ocean_domain_size() const { return ocean_domain_size; }
	void set_wind_speed(float p_speed);
	float get_wind_speed() const { return wind_speed; }
	void set_wind_direction(Vector2 p_dir);
	Vector2 get_wind_direction() const { return wind_direction; }
	void set_wave_amplitude(float p_amp);
	float get_wave_amplitude() const { return wave_amplitude; }

	// Optional shape: any flat mesh whose X/Z footprint (node-local) is the
	// water's outline -- a disc, a kidney bean, a lake with an island. When
	// set, the ripple grid is fitted to its bounds (domain_size is ignored),
	// cells outside it are dry land that waves reflect off, and the rendered
	// surface is the footprint resampled into an even grid clipped to its
	// outline. Unset = the square domain_size / ocean_domain_size plane.
	void set_surface_mesh(const Ref<Mesh> &p_mesh);
	Ref<Mesh> get_surface_mesh() const { return surface_mesh; }

	void set_water_material(const Ref<ShaderMaterial> &p_material);
	Ref<ShaderMaterial> get_water_material() const { return water_material; }

	// Light-space caustics: a direct RenderingDevice draw pass that projects
	// refracted surface-grid samples into a texture, without a SubViewport.
	void set_caustics_enabled(bool p_enabled);
	bool get_caustics_enabled() const { return caustics_enabled; }
	void set_caustics_sun_direction(Vector3 p_direction);
	Vector3 get_caustics_sun_direction() const { return caustics_sun_direction; }
	void set_caustics_reference_depth(float p_depth);
	float get_caustics_reference_depth() const { return caustics_reference_depth; }
	// The resulting texture -- assign it to a floor material's albedo/
	// emission using get_caustics_light_right(), get_caustics_light_up(),
	// get_caustics_origin(), and get_caustics_half_extent() as projection
	// metadata. Null until caustics_enabled and the node has built.
	Ref<Texture2D> get_caustics_texture() const;
	Vector3 get_caustics_light_right() const { return solver.get_caustics_light_right(); }
	Vector3 get_caustics_light_up() const { return solver.get_caustics_light_up(); }
	Vector3 get_caustics_origin() const { return solver.get_caustics_origin(); }
	float get_caustics_half_extent() const { return solver.get_caustics_half_extent(); }

	// CPU-side, safe every physics tick -- bilinear samples of the last
	// texture refresh (see _refresh_textures()'s throttle), summed across
	// both layers (ripple + ocean), matching what the rendered mesh shows.
	// Returns -INF over dry cells when a surface_mesh is set.
	float sample_height(Vector3 p_world_pos) const;
	// False outside the surface_mesh footprint (always true without one).
	bool is_wet(Vector3 p_world_pos) const;

	void submit_sphere(int p_owner, Vector3 p_world_pos, float p_radius, float p_strength = 1.0f);
	void clear_sphere(int p_owner);
	void submit_impulse(Vector3 p_world_pos, float p_radius, float p_strength);

private:
	WaterSolver solver;
	// NOT cached from immediately after configure() -- configure() dispatches
	// asynchronously (a render-thread post, see water_solver.h), so
	// solver.is_available() only flips true a frame or two later. _update()
	// re-checks it fresh every tick instead of trusting a stale snapshot.
	bool warned_no_device = false;

	Vector2 domain_size = Vector2(20, 20);
	int grid_resolution = 96;
	float depth = 3.0f;
	float water_level = 0.0f;
	float damping = 0.08f; // see WaterSolver::Settings::damping's own note on this value
	float ripple_amplitude = 1.0f;

	int ocean_grid_resolution = 64;
	Vector2 ocean_domain_size = Vector2(40, 40);
	float wind_speed = 8.0f;
	Vector2 wind_direction = Vector2(1, 0);
	float wave_amplitude = 1.0f;

	MeshInstance3D *mesh_instance = nullptr;
	Ref<Mesh> surface_mesh;
	Ref<Mesh> water_mesh; // what mesh_instance draws: a PlaneMesh, or the resampled surface_mesh footprint

	// World XZ the simulation grids are centred on (node origin, plus the
	// footprint's centre when surface_mesh is set), and the ripple domain
	// actually in use -- both fixed at _rebuild().
	Vector2 grid_center;
	Vector2 active_domain_size;
	// CPU copy of the wet/dry mask for is_wet()/sample_height(); empty = all wet.
	PackedByteArray wet_mask;
	Ref<ShaderMaterial> water_material;
	Ref<Texture2DRD> ripple_height_tex;
	Ref<Texture2DRD> ocean_height_tex;
	Ref<Texture2DRD> caustics_texture;
	bool textures_bound = false; // set once, after the solver's RD textures actually exist (see _update())

	bool caustics_enabled = true;
	Vector3 caustics_sun_direction = Vector3(-0.35f, -1.0f, -0.25f);
	float caustics_reference_depth = 2.5f;

	Vector<float> cached_ripple_height;
	int cached_ripple_n = 0;
	Vector2 cached_ripple_domain;
	Vector<float> cached_ocean_height;
	int cached_ocean_n = 0;
	Vector2 cached_ocean_domain;

	int frames_since_refresh = 0;
	static constexpr int REFRESH_EVERY_FRAMES = 4; // throttled CPU cache refresh for sample_height() only -- rendering is unthrottled/automatic via the zero-copy textures

	void _rebuild();
	// Footprint triangles of surface_mesh in node-local XZ (degenerates dropped).
	Vector<Vector2> _collect_footprint() const;
	static PackedByteArray _rasterize_wet_mask(const Vector<Vector2> &p_tris, Vector2 p_center, Vector2 p_domain, int p_n);
	Ref<Mesh> _build_footprint_mesh(const Vector<Vector2> &p_tris, const Rect2 &p_bounds, int p_cells) const;
	void _bind_textures();
	void _refresh_cpu_cache();
	void _update(double p_delta);
	float _bilinear_sample(const Vector<float> &p_grid, int p_n, Vector2 p_domain, float p_world_x, float p_world_z) const;
};

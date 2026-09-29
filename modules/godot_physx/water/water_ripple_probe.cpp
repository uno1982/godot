/**************************************************************************/
/*  water_ripple_probe.cpp                                               */
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

#include "water_ripple_probe.h"

#include "core/object/class_db.h"

void WaterRippleProbe::_bind_methods() {
	ClassDB::bind_method(D_METHOD("is_available"), &WaterRippleProbe::is_available);
	ClassDB::bind_method(D_METHOD("configure", "grid_resolution", "domain_size", "depth", "damping", "gravity", "water_level"), &WaterRippleProbe::configure);
	ClassDB::bind_method(D_METHOD("step", "delta"), &WaterRippleProbe::step);
	ClassDB::bind_method(D_METHOD("submit_sphere", "owner", "world_pos", "radius", "strength"), &WaterRippleProbe::submit_sphere);
	ClassDB::bind_method(D_METHOD("clear_sphere", "owner"), &WaterRippleProbe::clear_sphere);
	ClassDB::bind_method(D_METHOD("submit_impulse", "world_pos", "radius", "strength"), &WaterRippleProbe::submit_impulse);
	ClassDB::bind_method(D_METHOD("refresh_height_grid"), &WaterRippleProbe::refresh_height_grid);
	ClassDB::bind_method(D_METHOD("sample_height", "world_x", "world_z"), &WaterRippleProbe::sample_height);
}

bool WaterRippleProbe::is_available() const {
	return solver.is_available();
}

void WaterRippleProbe::configure(int p_grid_resolution, Vector2 p_domain_size, float p_depth, float p_damping, float p_gravity, float p_water_level) {
	WaterSolver::Settings s;
	s.grid_resolution = p_grid_resolution;
	s.domain_size = p_domain_size;
	s.depth = p_depth;
	s.damping = p_damping;
	s.gravity = p_gravity;
	s.water_level = p_water_level;
	solver.configure(s);
}

void WaterRippleProbe::step(double p_delta) {
	solver.step(p_delta);
}

void WaterRippleProbe::submit_sphere(int p_owner, Vector3 p_world_pos, float p_radius, float p_strength) {
	solver.submit_sphere((uint64_t)p_owner, p_world_pos, p_radius, p_strength);
}

void WaterRippleProbe::clear_sphere(int p_owner) {
	solver.clear_sphere((uint64_t)p_owner);
}

void WaterRippleProbe::submit_impulse(Vector3 p_world_pos, float p_radius, float p_strength) {
	solver.submit_impulse(p_world_pos, p_radius, p_strength);
}

void WaterRippleProbe::refresh_height_grid() {
	solver.get_height_grid(cached_height, cached_n, cached_domain_size);
}

float WaterRippleProbe::sample_height(float p_world_x, float p_world_z) const {
	if (cached_height.is_empty() || cached_n <= 0) {
		return 0.0f;
	}
	// Matches water_ripple.glsl's own cell->world mapping: cell (x,y)'s
	// center is at domain center + ((x+0.5)/N*2-1, (y+0.5)/N*2-1) * domain/2.
	const float gx = (p_world_x / (cached_domain_size.x * 0.5f) + 1.0f) * 0.5f * cached_n - 0.5f;
	const float gz = (p_world_z / (cached_domain_size.y * 0.5f) + 1.0f) * 0.5f * cached_n - 0.5f;
	const int x0 = CLAMP((int)floorf(gx), 0, cached_n - 1);
	const int z0 = CLAMP((int)floorf(gz), 0, cached_n - 1);
	const int x1 = CLAMP(x0 + 1, 0, cached_n - 1);
	const int z1 = CLAMP(z0 + 1, 0, cached_n - 1);
	const float fx = CLAMP(gx - x0, 0.0f, 1.0f);
	const float fz = CLAMP(gz - z0, 0.0f, 1.0f);
	const float h00 = cached_height[z0 * cached_n + x0];
	const float h10 = cached_height[z0 * cached_n + x1];
	const float h01 = cached_height[z1 * cached_n + x0];
	const float h11 = cached_height[z1 * cached_n + x1];
	return Math::lerp(Math::lerp(h00, h10, fx), Math::lerp(h01, h11, fx), fz);
}

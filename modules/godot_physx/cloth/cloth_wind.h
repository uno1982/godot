/**************************************************************************/
/*  cloth_wind.h                                                          */
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

#include "core/math/math_funcs.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "scene/3d/physics/area_3d.h"

// The air velocity a cloth feels: a constant breeze, plus an Area3D's wind settings read directly (the PhysX module
// doesn't expose areas to point queries), plus swirling gusts. p_center is where the cloth is, for the area's downwind
// attenuation; p_time drives the gusts.
inline Vector3 physx_cloth_sample_wind(const Vector3 &p_wind, const Area3D *p_area, float p_turbulence, double p_time, const Vector3 &p_center) {
	Vector3 w = p_wind;
	if (p_area != nullptr && p_area->get_wind_force_magnitude() > 0.0f) {
		const Node3D *src_node = Object::cast_to<Node3D>(p_area->get_node_or_null(p_area->get_wind_source_path()));
		Vector3 dir(-1, 0, 0);
		Vector3 src;
		if (src_node != nullptr) {
			const Transform3D sx = src_node->get_global_transform();
			dir = -sx.basis.get_column(Vector3::AXIS_Z).normalized();
			src = sx.origin;
		}
		const float att = p_area->get_wind_attenuation_factor();
		const float proj = MAX((p_center - src).dot(dir), 1.0f);
		w += dir * (p_area->get_wind_force_magnitude() * Math::pow(proj, -att));
	}

	if (p_turbulence > 0.0f && w != Vector3()) {
		const float speed = w.length();
		const Vector3 dir = w / speed;
		Vector3 perp = dir.cross(Vector3(0, 1, 0));
		if (perp.length_squared() < 1.0e-4f) {
			perp = dir.cross(Vector3(1, 0, 0));
		}
		perp.normalize();
		const Vector3 up = dir.cross(perp).normalized();
		const float t = (float)p_time;
		const float gust = 0.6f * Math::sin(t * 1.7f) + 0.4f * Math::sin(t * 4.3f + 1.3f);
		const float sway = 0.5f * Math::sin(t * 2.9f + 0.7f);
		w += dir * (speed * p_turbulence * gust);
		w += perp * (speed * p_turbulence * 0.5f * sway);
		w += up * (speed * p_turbulence * 0.3f * Math::sin(t * 3.7f));
	}
	return w;
}

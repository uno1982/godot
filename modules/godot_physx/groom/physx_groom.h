/**************************************************************************/
/*  physx_groom.h                                                         */
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

#include "core/io/resource.h"
#include "core/math/aabb.h"

// Hair strands, as polylines in the space of the node that draws them. Made by importing an Alembic (.abc) groom.
class PhysXGroom : public Resource {
	GDCLASS(PhysXGroom, Resource);

	PackedVector3Array points;
	// Strand i is points[strand_offsets[i]] up to points[strand_offsets[i + 1]], root first.
	PackedInt32Array strand_offsets;
	// Per point: the strand's thickness there (a diameter, as Alembic and Unreal have it).
	PackedFloat32Array widths;
	// Four numbers per point, or empty: see groom_self_shadow.h.
	PackedFloat32Array self_shadow;

protected:
	static void _bind_methods();

public:
	void set_points(const PackedVector3Array &p_points);
	PackedVector3Array get_points() const { return points; }
	void set_strand_offsets(const PackedInt32Array &p_offsets);
	PackedInt32Array get_strand_offsets() const { return strand_offsets; }
	void set_widths(const PackedFloat32Array &p_widths);
	PackedFloat32Array get_widths() const { return widths; }
	void set_self_shadow(const PackedFloat32Array &p_self_shadow);
	PackedFloat32Array get_self_shadow() const { return self_shadow; }
	// p_reach 0: 15% of the groom's size.
	void bake_self_shadow(int p_rays = 32, float p_reach = 0.0f);

	int get_strand_count() const { return MAX(strand_offsets.size() - 1, 0); }
	int get_point_count() const { return points.size(); }
	AABB get_aabb() const;
	// The arrays agree with each other: offsets rise from 0 to the point count, a width for every point.
	bool is_valid() const;
};

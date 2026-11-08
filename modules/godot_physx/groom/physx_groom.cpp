/**************************************************************************/
/*  physx_groom.cpp                                                       */
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

#include "physx_groom.h"

#include "core/object/class_db.h"

void PhysXGroom::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_points", "points"), &PhysXGroom::set_points);
	ClassDB::bind_method(D_METHOD("get_points"), &PhysXGroom::get_points);
	ClassDB::bind_method(D_METHOD("set_strand_offsets", "offsets"), &PhysXGroom::set_strand_offsets);
	ClassDB::bind_method(D_METHOD("get_strand_offsets"), &PhysXGroom::get_strand_offsets);
	ClassDB::bind_method(D_METHOD("set_widths", "widths"), &PhysXGroom::set_widths);
	ClassDB::bind_method(D_METHOD("get_widths"), &PhysXGroom::get_widths);
	ClassDB::bind_method(D_METHOD("get_strand_count"), &PhysXGroom::get_strand_count);
	ClassDB::bind_method(D_METHOD("get_point_count"), &PhysXGroom::get_point_count);
	ClassDB::bind_method(D_METHOD("get_aabb"), &PhysXGroom::get_aabb);

	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "points"), "set_points", "get_points");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_INT32_ARRAY, "strand_offsets"), "set_strand_offsets", "get_strand_offsets");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "widths"), "set_widths", "get_widths");
}

void PhysXGroom::set_points(const PackedVector3Array &p_points) {
	points = p_points;
	emit_changed();
}

void PhysXGroom::set_strand_offsets(const PackedInt32Array &p_offsets) {
	strand_offsets = p_offsets;
	emit_changed();
}

void PhysXGroom::set_widths(const PackedFloat32Array &p_widths) {
	widths = p_widths;
	emit_changed();
}

AABB PhysXGroom::get_aabb() const {
	if (points.is_empty()) {
		return AABB();
	}
	AABB box(points[0], Vector3());
	float widest = 0.0f;
	for (int i = 0; i < points.size(); i++) {
		box.expand_to(points[i]);
	}
	for (int i = 0; i < widths.size(); i++) {
		widest = MAX(widest, widths[i]);
	}
	return box.grow(widest * 0.5f);
}

bool PhysXGroom::is_valid() const {
	if (strand_offsets.size() < 2 || strand_offsets[0] != 0 || strand_offsets[strand_offsets.size() - 1] != points.size() || widths.size() != points.size()) {
		return false;
	}
	for (int i = 1; i < strand_offsets.size(); i++) {
		if (strand_offsets[i] < strand_offsets[i - 1]) {
			return false;
		}
	}
	return true;
}

/**************************************************************************/
/*  physx_lens_flare_settings.cpp                                         */
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

#include "physx_lens_flare_settings.h"

#include "core/object/class_db.h"

PhysXLensFlareSettings::PhysXLensFlareSettings() {
	// Eight ghosts of the bright image, scaled about the screen's center (negative: mirrored through it), each tinted.
	const Color colors[MAX_GHOSTS] = {
		Color(1.0, 0.8, 0.4),
		Color(1.0, 1.0, 0.6),
		Color(0.8, 0.8, 1.0),
		Color(0.5, 1.0, 0.4),
		Color(0.5, 0.8, 1.0),
		Color(0.9, 1.0, 0.8),
		Color(1.0, 0.8, 0.4),
		Color(0.9, 0.7, 0.7),
	};
	const float scales[MAX_GHOSTS] = { -1.5f, 2.5f, -5.0f, 10.0f, 0.7f, -0.4f, -0.2f, -0.1f };
	for (int i = 0; i < MAX_GHOSTS; i++) {
		ghost_colors.push_back(colors[i]);
		ghost_scales.push_back(scales[i]);
	}
}

void PhysXLensFlareSettings::set_intensity(float p_value) {
	intensity = MAX(p_value, 0.0f);
	emit_changed();
}

void PhysXLensFlareSettings::set_tint(const Color &p_value) {
	tint = p_value;
	emit_changed();
}

void PhysXLensFlareSettings::set_threshold(float p_value) {
	threshold = MAX(p_value, 0.0f);
	emit_changed();
}

void PhysXLensFlareSettings::set_threshold_range(float p_value) {
	threshold_range = MAX(p_value, 0.01f);
	emit_changed();
}

void PhysXLensFlareSettings::set_ghost_intensity(float p_value) {
	ghost_intensity = MAX(p_value, 0.0f);
	emit_changed();
}

void PhysXLensFlareSettings::set_ghost_chroma_shift(float p_value) {
	ghost_chroma_shift = p_value;
	emit_changed();
}

void PhysXLensFlareSettings::set_ghost_colors(const PackedColorArray &p_value) {
	ghost_colors = p_value;
	if (ghost_colors.size() > MAX_GHOSTS) {
		ghost_colors.resize(MAX_GHOSTS);
	}
	emit_changed();
}

void PhysXLensFlareSettings::set_ghost_scales(const PackedFloat32Array &p_value) {
	ghost_scales = p_value;
	if (ghost_scales.size() > MAX_GHOSTS) {
		ghost_scales.resize(MAX_GHOSTS);
	}
	emit_changed();
}

void PhysXLensFlareSettings::set_halo_intensity(float p_value) {
	halo_intensity = MAX(p_value, 0.0f);
	emit_changed();
}

void PhysXLensFlareSettings::set_halo_width(float p_value) {
	halo_width = p_value;
	emit_changed();
}

void PhysXLensFlareSettings::set_halo_mask(float p_value) {
	halo_mask = CLAMP(p_value, 0.0f, 0.99f);
	emit_changed();
}

void PhysXLensFlareSettings::set_halo_compression(float p_value) {
	halo_compression = CLAMP(p_value, 0.0f, 1.0f);
	emit_changed();
}

void PhysXLensFlareSettings::set_halo_chroma_shift(float p_value) {
	halo_chroma_shift = p_value;
	emit_changed();
}

void PhysXLensFlareSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_intensity", "intensity"), &PhysXLensFlareSettings::set_intensity);
	ClassDB::bind_method(D_METHOD("get_intensity"), &PhysXLensFlareSettings::get_intensity);
	ClassDB::bind_method(D_METHOD("set_tint", "tint"), &PhysXLensFlareSettings::set_tint);
	ClassDB::bind_method(D_METHOD("get_tint"), &PhysXLensFlareSettings::get_tint);
	ClassDB::bind_method(D_METHOD("set_threshold", "threshold"), &PhysXLensFlareSettings::set_threshold);
	ClassDB::bind_method(D_METHOD("get_threshold"), &PhysXLensFlareSettings::get_threshold);
	ClassDB::bind_method(D_METHOD("set_threshold_range", "range"), &PhysXLensFlareSettings::set_threshold_range);
	ClassDB::bind_method(D_METHOD("get_threshold_range"), &PhysXLensFlareSettings::get_threshold_range);
	ClassDB::bind_method(D_METHOD("set_ghost_intensity", "intensity"), &PhysXLensFlareSettings::set_ghost_intensity);
	ClassDB::bind_method(D_METHOD("get_ghost_intensity"), &PhysXLensFlareSettings::get_ghost_intensity);
	ClassDB::bind_method(D_METHOD("set_ghost_chroma_shift", "shift"), &PhysXLensFlareSettings::set_ghost_chroma_shift);
	ClassDB::bind_method(D_METHOD("get_ghost_chroma_shift"), &PhysXLensFlareSettings::get_ghost_chroma_shift);
	ClassDB::bind_method(D_METHOD("set_ghost_colors", "colors"), &PhysXLensFlareSettings::set_ghost_colors);
	ClassDB::bind_method(D_METHOD("get_ghost_colors"), &PhysXLensFlareSettings::get_ghost_colors);
	ClassDB::bind_method(D_METHOD("set_ghost_scales", "scales"), &PhysXLensFlareSettings::set_ghost_scales);
	ClassDB::bind_method(D_METHOD("get_ghost_scales"), &PhysXLensFlareSettings::get_ghost_scales);
	ClassDB::bind_method(D_METHOD("set_halo_intensity", "intensity"), &PhysXLensFlareSettings::set_halo_intensity);
	ClassDB::bind_method(D_METHOD("get_halo_intensity"), &PhysXLensFlareSettings::get_halo_intensity);
	ClassDB::bind_method(D_METHOD("set_halo_width", "width"), &PhysXLensFlareSettings::set_halo_width);
	ClassDB::bind_method(D_METHOD("get_halo_width"), &PhysXLensFlareSettings::get_halo_width);
	ClassDB::bind_method(D_METHOD("set_halo_mask", "mask"), &PhysXLensFlareSettings::set_halo_mask);
	ClassDB::bind_method(D_METHOD("get_halo_mask"), &PhysXLensFlareSettings::get_halo_mask);
	ClassDB::bind_method(D_METHOD("set_halo_compression", "compression"), &PhysXLensFlareSettings::set_halo_compression);
	ClassDB::bind_method(D_METHOD("get_halo_compression"), &PhysXLensFlareSettings::get_halo_compression);
	ClassDB::bind_method(D_METHOD("set_halo_chroma_shift", "shift"), &PhysXLensFlareSettings::set_halo_chroma_shift);
	ClassDB::bind_method(D_METHOD("get_halo_chroma_shift"), &PhysXLensFlareSettings::get_halo_chroma_shift);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "intensity", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_intensity", "get_intensity");
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "tint", PROPERTY_HINT_COLOR_NO_ALPHA), "set_tint", "get_tint");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "threshold", PROPERTY_HINT_RANGE, "0,4,0.01,or_greater"), "set_threshold", "get_threshold");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "threshold_range", PROPERTY_HINT_RANGE, "0.01,4,0.01,or_greater"), "set_threshold_range", "get_threshold_range");
	ADD_GROUP("Ghosts", "ghost_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ghost_intensity", PROPERTY_HINT_RANGE, "0,4,0.01,or_greater"), "set_ghost_intensity", "get_ghost_intensity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ghost_chroma_shift", PROPERTY_HINT_RANGE, "-0.2,0.2,0.001"), "set_ghost_chroma_shift", "get_ghost_chroma_shift");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_COLOR_ARRAY, "ghost_colors"), "set_ghost_colors", "get_ghost_colors");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "ghost_scales"), "set_ghost_scales", "get_ghost_scales");
	ADD_GROUP("Halo", "halo_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "halo_intensity", PROPERTY_HINT_RANGE, "0,4,0.01,or_greater"), "set_halo_intensity", "get_halo_intensity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "halo_width", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_halo_width", "get_halo_width");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "halo_mask", PROPERTY_HINT_RANGE, "0,0.99,0.01"), "set_halo_mask", "get_halo_mask");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "halo_compression", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_halo_compression", "get_halo_compression");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "halo_chroma_shift", PROPERTY_HINT_RANGE, "-0.2,0.2,0.001"), "set_halo_chroma_shift", "get_halo_chroma_shift");
}

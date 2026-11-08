/**************************************************************************/
/*  physx_groom_3d.h                                                      */
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

#include "physx_groom.h"

#include "scene/3d/visual_instance_3d.h"

// Draws a PhysXGroom: each strand a ribbon turned to face the camera in the vertex shader (and the light, in its
// shadow pass), at least a pixel wide, thinner strands fading to cover what they would. Hair shading: two highlights
// along the strand (Kajiya-Kay), the root darker than the tip.
class PhysXGroom3D : public GeometryInstance3D {
	GDCLASS(PhysXGroom3D, GeometryInstance3D);

	Ref<PhysXGroom> groom;
	Color root_color = Color(0.09, 0.055, 0.035);
	Color tip_color = Color(0.32, 0.2, 0.11);
	float width_scale = 1.0f;
	float self_shadow = 0.6f;
	int max_strands = 0;
	float strand_fraction = 1.0f;

	RID mesh;
	RID material;
	AABB aabb;

	static RID shared_shader;
	static RID _shader();
	void _rebuild();
	void _update_material();
	void _update_lod();

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	void set_groom(const Ref<PhysXGroom> &p_groom);
	Ref<PhysXGroom> get_groom() const { return groom; }
	void set_root_color(const Color &p_color);
	Color get_root_color() const { return root_color; }
	void set_tip_color(const Color &p_color);
	Color get_tip_color() const { return tip_color; }
	void set_width_scale(float p_scale);
	float get_width_scale() const { return width_scale; }
	// How much light a strand of the groom between a point and the light takes away (each one, as exp(-self_shadow)).
	void set_self_shadow(float p_amount);
	float get_self_shadow() const { return self_shadow; }

	void set_max_strands(int p_count);
	int get_max_strands() const { return max_strands; }
	// How many strands are drawn now, of get_groom()'s, after the distance and max_strands.
	int get_drawn_strand_count() const;

	virtual AABB get_aabb() const override { return aabb; }

	static void free_shared();

	PhysXGroom3D();
	~PhysXGroom3D();
};

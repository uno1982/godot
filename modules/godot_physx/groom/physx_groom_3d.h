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
// shadow pass), at least a pixel wide, thinner strands fading to cover what they would. Hair shading: Marschner's
// model (R, TT and TRT), lit through the groom's precomputed self-shadowing; the root darker than the tip.
class PhysXGroom3D : public GeometryInstance3D {
	GDCLASS(PhysXGroom3D, GeometryInstance3D);

public:
	enum RenderMode {
		// A ribbon per strand, an ordinary mesh: every renderer, every effect on it, strands thinner than a pixel dithered.
		RENDER_MODE_MESH,
		// Drawn by the module's own compute passes with each strand's exact share of each pixel, lit by Godot through a
		// screen-covering proxy (PhysXGroomRenderEffect): smooth thin hair with or without MSAA. Forward+ and Mobile.
		RENDER_MODE_SCREEN_SPACE,
	};

	enum ShadingModel {
		// Two highlights along the strand; warm, and the cheaper of the two.
		SHADING_MODEL_KAJIYA_KAY,
		// Light reflected, through and inside the strand: physically based, with the glow of backlit hair. More math per
		// pixel (see the class docs for what it costs).
		SHADING_MODEL_MARSCHNER,
	};

private:
	RenderMode render_mode = RENDER_MODE_MESH;
	ShadingModel shading_model = SHADING_MODEL_KAJIYA_KAY;
	// The world the screen-space effect knows this groom in, and the strands it has (see _send_points).
	ObjectID screen_space_world;
	uint64_t points_version = 0;
	bool sent_to_effect = false;
	void _send_points();
	void _update_screen_space();

	Ref<PhysXGroom> groom;
	Color root_color = Color(0.09, 0.055, 0.035);
	Color tip_color = Color(0.32, 0.2, 0.11);
	float width_scale = 1.0f;
	float self_shadow = 0.6f;
	float roughness = 0.35f;
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
	void set_render_mode(RenderMode p_mode);
	RenderMode get_render_mode() const { return render_mode; }
	void set_shading_model(ShadingModel p_model);
	ShadingModel get_shading_model() const { return shading_model; }
	// How wide the strands' highlights spread along them (Marschner's longitudinal lobe widths; Marschner only).
	void set_roughness(float p_roughness);
	float get_roughness() const { return roughness; }

	void set_max_strands(int p_count);
	int get_max_strands() const { return max_strands; }
	// How many strands are drawn now, of get_groom()'s, after the distance and max_strands.
	int get_drawn_strand_count() const;

	virtual AABB get_aabb() const override { return aabb; }

	static void free_shared();

	PhysXGroom3D();
	~PhysXGroom3D();
};

VARIANT_ENUM_CAST(PhysXGroom3D::RenderMode);
VARIANT_ENUM_CAST(PhysXGroom3D::ShadingModel);

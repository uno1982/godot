/**************************************************************************/
/*  pxtgi_emission_meter.h                                                */
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

#include "core/object/ref_counted.h"
#include "core/variant/typed_array.h"

class CameraAttributesPractical;
class Environment;
class Material;
class Node;
class ShaderMaterial;
class Sky;
class SubViewport;

// How brightly materials glow, and what share of light they reflect, measured by drawing them: each on a tile of one
// HDR viewport, first with no lights and no environment light, at a known exposure, then under an even white
// ambient light, then in a white sky's reflections. What comes out is exactly what the shader emits and reflects
// (textures, tints, any multiplier it hides), which PXTGI can't read from a ShaderMaterial (nor a texture's colors)
// itself. The glow is read several times over half a second or so, to tell one that changes over time (its mean is reported); the
// albedo is the light the second and third drawings add: a dielectric's diffuse color, a metal's reflection.
// The editor's material tool tags materials from it.
class PXTGIEmissionMeter : public RefCounted {
	GDCLASS(PXTGIEmissionMeter, RefCounted);

	TypedArray<Material> pending;
	Array results;
	TypedArray<Material> batch;
	ObjectID viewport;
	Ref<Environment> environment; // the batch's: its ambient light, then its reflections, turned on for the albedo
	Ref<CameraAttributesPractical> attributes; // the batch's camera exposure
	Vector<Color> sums; // each tile's readings added up
	Vector<Color> lit_sums; // and under the ambient light
	Vector<Color> shine_sums; // and in the reflections of a white sky
	Ref<ShaderMaterial> white_reference; // drawn after the materials in every batch: albedo 1, not metallic
	Ref<ShaderMaterial> black_reference; // and last: albedo 0
	Ref<Sky> white_sky; // the same white in every direction
	Vector<float> lowest; // and its dimmest and brightest reading
	Vector<float> highest;
	int columns = 0;
	int frame = 0;
	int reads = 0;
	bool shine_pass = false; // past the ambient readings, into the reflections
	bool measuring = false;

	void _start_batch();
	void _frame();
	void _frame_lit();
	void _frame_shine();
	Vector<Color> _read_tiles() const;
	void _clear_viewport();

protected:
	static void _bind_methods();

public:
	// Drawn at 1 / SCALE of their brightness, so a glow far above white (millions of nits) doesn't overflow the
	// half-float buffer, then scaled back.
	static constexpr float SCALE = 4096.0f;
	static constexpr int TILE = 16; // pixels a material's tile takes
	static constexpr int MAX_COLUMNS = 16; // tiles across: up to 254 materials a batch, and the two references

	Error measure(const TypedArray<Material> &p_materials);
	bool is_measuring() const { return measuring; }

	// The ShaderMaterials under p_root that PXTGI can't read a glow from: no pxtgi_emission on the material or on
	// its MeshInstance3D. Material -> Array of the MeshInstance3Ds using it.
	static Dictionary find_untagged(Node *p_root);
	// The pxtgi_emission Color that stands for a measured (linear) glow: sRGB encoded, as PXTGI reads it.
	static Color emission_tag(const Color &p_glow);
	// The materials under p_root whose albedo PXTGI can't read in full and that have no pxtgi_albedo: ShaderMaterials,
	// and BaseMaterial3Ds with an albedo texture (PXTGI reads their color, not the texture's). Material -> Array of
	// the MeshInstance3Ds using it.
	static Dictionary find_untagged_albedo(Node *p_root);
	// The pxtgi_albedo Color that stands for a measured (linear) albedo: sRGB encoded, as PXTGI reads it.
	static Color albedo_tag(const Color &p_albedo);

	~PXTGIEmissionMeter();
};

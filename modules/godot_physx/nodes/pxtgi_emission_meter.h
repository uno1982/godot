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

class Material;
class Node;
class SubViewport;

// How brightly materials glow, measured by drawing them: each on a tile of one HDR viewport, with no lights and no
// environment light, at a known exposure. What comes out is exactly what the shader emits (textures, tints, any
// multiplier it hides), which PXTGI can't read from a ShaderMaterial itself. Read several times over half a second
// or so, to tell a glow that changes over time (its mean is reported). The editor's emissive material tool tags materials from it.
class PXTGIEmissionMeter : public RefCounted {
	GDCLASS(PXTGIEmissionMeter, RefCounted);

	TypedArray<Material> pending;
	Array results;
	TypedArray<Material> batch;
	ObjectID viewport;
	Vector<Color> sums; // each tile's readings added up
	Vector<float> lowest; // and its dimmest and brightest reading
	Vector<float> highest;
	int columns = 0;
	int frame = 0;
	int reads = 0;
	bool measuring = false;

	void _start_batch();
	void _frame();
	Vector<Color> _read_tiles() const;
	void _clear_viewport();

protected:
	static void _bind_methods();

public:
	// Drawn at 1 / SCALE of their brightness, so a glow far above white (millions of nits) doesn't overflow the
	// half-float buffer, then scaled back.
	static constexpr float SCALE = 4096.0f;
	static constexpr int TILE = 16; // pixels a material's tile takes
	static constexpr int MAX_COLUMNS = 16; // tiles across: up to 256 materials a batch

	Error measure(const TypedArray<Material> &p_materials);
	bool is_measuring() const { return measuring; }

	// The ShaderMaterials under p_root that PXTGI can't read a glow from: no pxtgi_emission on the material or on
	// its MeshInstance3D. Material -> Array of the MeshInstance3Ds using it.
	static Dictionary find_untagged(Node *p_root);
	// The pxtgi_emission Color that stands for a measured (linear) glow: sRGB encoded, as PXTGI reads it.
	static Color emission_tag(const Color &p_glow);

	~PXTGIEmissionMeter();
};

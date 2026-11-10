/**************************************************************************/
/*  physx_blast_asset.h                                                   */
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
#include "core/variant/array.h"
#include "core/variant/typed_array.h"
#include "core/variant/variant.h"
#include "scene/resources/material.h"

// A fractured Blast asset, saveable/loadable as a normal Godot Resource
// (.tres/.res) -- the real replacement for the two-file (.asset/.chunks)
// convention the standalone throwaway authoring tool and the early runtime-
// bridge MVP (GodotPhysXBlastProbe, PhysXDestructible3D's asset_path/
// chunks_path) used before this Resource type existed. Produced by
// PhysXBlastAuthoring::fracture_mesh(); consumed by PhysXDestructible3D
// (once wired up -- not yet, as of this class's introduction).
class PhysXBlastAsset : public Resource {
	GDCLASS(PhysXBlastAsset, Resource);

protected:
	static void _bind_methods();

public:
	// The raw NvBlastAsset bytes -- a single relocatable memory block per
	// NvBlast's own design, so this is genuinely just "the asset", no
	// framing of our own on top.
	void set_asset_bytes(const PackedByteArray &p_bytes) { asset_bytes = p_bytes; }
	PackedByteArray get_asset_bytes() const { return asset_bytes; }

	// chunk_points[i] is a PackedVector3Array: chunk i's render-mesh
	// triangle-soup positions (object-local space), same data/format
	// GodotPhysXBlastProbe/PhysXDestructible3D already parse from the old
	// .chunks text format -- here it's just real Resource-serialized data,
	// no text parsing needed.
	void set_chunk_points(const Array &p_points) { chunk_points = p_points; }
	Array get_chunk_points() const { return chunk_points; }

	int get_chunk_count() const { return chunk_points.size(); }

	// Aligned with chunk_points, and empty in an asset fractured before these existed (flat, untextured pieces):
	// chunk_normals[i] a PackedVector3Array, chunk_uvs[i] a PackedVector2Array.
	void set_chunk_normals(const Array &p_normals) { chunk_normals = p_normals; }
	Array get_chunk_normals() const { return chunk_normals; }
	void set_chunk_uvs(const Array &p_uvs) { chunk_uvs = p_uvs; }
	Array get_chunk_uvs() const { return chunk_uvs; }
	// Where each chunk's interior (cut) faces start in its points: the source mesh's faces come first.
	void set_chunk_interior_starts(const PackedInt32Array &p_starts) { chunk_interior_starts = p_starts; }
	PackedInt32Array get_chunk_interior_starts() const { return chunk_interior_starts; }
	// chunk_surface_starts[i] is a PackedInt32Array: where each source surface's faces start in chunk i's points, in
	// surface order, before its interior ones. Empty in an asset fractured before these existed (one surface).
	void set_chunk_surface_starts(const Array &p_starts) { chunk_surface_starts = p_starts; }
	Array get_chunk_surface_starts() const { return chunk_surface_starts; }
	// The source mesh's surface materials, one per surface (null where it had none).
	void set_surface_materials(const TypedArray<Material> &p_materials) { surface_materials = p_materials; }
	TypedArray<Material> get_surface_materials() const { return surface_materials; }
	int get_surface_count() const { return MAX(1, (int)surface_materials.size()); }

private:
	PackedByteArray asset_bytes;
	Array chunk_points;
	Array chunk_normals;
	Array chunk_uvs;
	PackedInt32Array chunk_interior_starts;
	Array chunk_surface_starts;
	TypedArray<Material> surface_materials;
};

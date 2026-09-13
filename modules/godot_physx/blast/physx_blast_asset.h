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
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "core/io/resource.h"
#include "core/variant/array.h"
#include "core/variant/variant.h"

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

private:
	PackedByteArray asset_bytes;
	Array chunk_points;
};

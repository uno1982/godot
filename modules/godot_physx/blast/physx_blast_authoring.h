/**************************************************************************/
/*  physx_blast_authoring.h                                               */
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

#include "physx_blast_asset.h"

#include "core/object/ref_counted.h"

class Mesh;

// In-engine mesh fracturing -- the real replacement for the standalone
// throwaway blast_test_gen.cpp tool this whole Blast effort prototyped with.
// Ports that tool's proven-correct authoring call sequence
// (NvBlastExtAuthoringCreateFractureTool -> VoronoiSitesGenerator ->
// voronoiFracturing -> NvBlastExtAuthoringProcessFracture) into real module
// code, using the module's own PxPhysics (GodotPhysXServer3D) to cook
// collision hulls instead of a standalone PxCreatePhysics call.
//
// A RefCounted rather than a set of static functions so it's directly
// callable from GDScript (ClassDB.instantiate("PhysXBlastAuthoring")) for a
// headless test today, and so a future editor dock has an obvious object to
// hold onto without inventing a second calling convention.
class PhysXBlastAuthoring : public RefCounted {
	GDCLASS(PhysXBlastAuthoring, RefCounted);

public:
	// NvBlastExtAuthoring's FractureTool supports more than one fracture
	// algorithm -- PATTERN_VORONOI (organic/rock-like chunks, what this class
	// originally only offered) and PATTERN_SLICING (regular, brick-like
	// pieces from cutting planes -- FractureTool::slicing()/
	// SlicingConfiguration -- much better suited to man-made/architectural
	// shapes like crates or walls than Voronoi's random cells). A third,
	// PATTERN_CUTOUT (2D pattern extruded through the mesh, built for glass-
	// pane spiderweb cracks), exists in the SDK too but needs a separate
	// pattern asset and a narrower use case -- not exposed here yet.
	enum FracturePattern {
		PATTERN_VORONOI,
		PATTERN_SLICING,
	};

protected:
	static void _bind_methods();

public:
	// Fractures p_mesh's surface 0 into roughly p_site_count pieces (plus the
	// implicit root chunk = the whole unfractured mesh, chunk 0 -- same
	// convention PhysXDestructible3D/GodotPhysXBlastProbe already assume),
	// using p_pattern's algorithm. For PATTERN_SLICING, p_site_count is only
	// approximate -- it's converted to a roughly-cube-root split across the
	// 3 slicing axes, since slicing works in slice-counts-per-axis, not a
	// single site count. Returns a null Ref on failure (mesh has no
	// surfaces, cooking failed, etc).
	Ref<PhysXBlastAsset> fracture_mesh(const Ref<Mesh> &p_mesh, int p_site_count, int p_seed, FracturePattern p_pattern = PATTERN_VORONOI);
};

VARIANT_ENUM_CAST(PhysXBlastAuthoring::FracturePattern);

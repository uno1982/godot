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

// In-engine Voronoi mesh fracturing -- the real replacement for the
// standalone throwaway blast_test_gen.cpp tool this whole Blast effort
// prototyped with. Ports that tool's proven-correct authoring call sequence
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

protected:
	static void _bind_methods();

public:
	// Fractures p_mesh's surface 0 into p_site_count Voronoi cells (plus the
	// implicit root chunk = the whole unfractured mesh, chunk 0 -- same
	// convention PhysXDestructible3D/GodotPhysXBlastProbe already assume).
	// Returns a null Ref on failure (mesh has no surfaces, cooking failed, etc).
	Ref<PhysXBlastAsset> fracture_mesh(const Ref<Mesh> &p_mesh, int p_site_count, int p_seed);
};

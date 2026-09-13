/**************************************************************************/
/*  godot_physx_blast_probe.h                                             */
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

#include "core/math/transform_3d.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "core/templates/rid.h"
#include "core/variant/typed_array.h"

struct NvBlastAsset;
struct NvBlastFamily;
struct NvBlastActor;

// Runtime-bridge MVP probe for NVIDIA Blast (runtime mesh fracture), not a
// real node -- exists purely so a headless GDScript test can drive the real
// damage -> fracture -> split -> spawn-rigid-bodies mechanism end to end,
// same precedent as PhysXMPMGasProbe from the gas-solver work: prove the
// mechanism cheaply before building the real node + editor authoring tool
// (see the module's Blast planning notes). Loads a raw NvBlastAsset dump
// plus a companion per-chunk render-mesh dump (both produced today only by
// a throwaway standalone authoring tool, not by any in-engine tool yet --
// a real Resource-based asset format is later, separate scope).
class GodotPhysXBlastProbe : public RefCounted {
	GDCLASS(GodotPhysXBlastProbe, RefCounted);

protected:
	static void _bind_methods();

public:
	void set_space(RID p_space) { space = p_space; }
	void set_base_transform(const Transform3D &p_transform) { base_transform = p_transform; }
	void set_base_velocity(const Vector3 &p_velocity) { base_velocity = p_velocity; }

	// p_asset_path/p_chunks_path: any path FileAccess can open (res://, an
	// absolute OS path, etc.) -- p_asset_path is the raw NvBlastAsset bytes
	// (NvBlastAsset is a single relocatable memory block, so this is just its
	// exact byte size, no header of our own); p_chunks_path is the plain-text
	// per-chunk triangle-soup dump (see blast_test_gen.cpp's file header for
	// the exact format).
	bool load(const String &p_asset_path, const String &p_chunks_path);

	// Applies radial damage to every currently-live actor, splits any that
	// fracture, and spawns a real rigid body (PhysicsServer3D, generic API --
	// same convex_polygon_shape_create() path every other convex shape in
	// this module already goes through) for each newly-visible chunk of each
	// newly-created actor. Returns how many new bodies were spawned this call.
	int apply_radial_damage(const Vector3 &p_position, float p_damage, float p_min_radius, float p_max_radius);

	int get_body_count() const { return (int)bodies.size(); }
	RID get_body(int p_index) const;
	int get_live_actor_count() const { return (int)live_actors.size(); }

	GodotPhysXBlastProbe();
	~GodotPhysXBlastProbe();

private:
	RID space;
	Transform3D base_transform;
	Vector3 base_velocity;

	void *asset_mem = nullptr;
	void *family_mem = nullptr;
	NvBlastFamily *family = nullptr;
	LocalVector<NvBlastActor *> live_actors;

	uint32_t asset_chunk_count = 0;
	uint32_t asset_bond_count = 0;

	// chunk_points[chunk_index] holds that chunk's render-mesh triangle-soup
	// vertex positions (object-local space, straight from the authoring
	// dump) -- fed directly to PhysicsServer3D::shape_set_data() to cook a
	// real convex hull on demand. Blast's own authored CollisionHull data
	// (from NvBlastExtAuthoringProcessFracture) is never consumed here.
	LocalVector<PackedVector3Array> chunk_points;

	LocalVector<RID> bodies;
	LocalVector<RID> shapes;

	void _spawn_body_for_chunk(uint32_t p_chunk_index);
	void _free_all();
};

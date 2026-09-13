/**************************************************************************/
/*  physx_destructible_3d.h                                               */
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

#include "core/templates/local_vector.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/material.h"

struct NvBlastAsset;
struct NvBlastFamily;
struct NvBlastActor;

// Real node wrapping the Blast runtime bridge proved out by
// GodotPhysXBlastProbe (see godot_physx_blast_probe.h -- that class stays as
// a headless-testable RefCounted for regression tests; this node duplicates
// its core damage/fracture/split logic rather than sharing it, since the
// probe is already tested and working and refactoring it under this pass
// would risk it for no benefit). Lives in blast/ rather than nodes/ because
// it's unconditionally guarded by GODOT_PHYSX_BLAST -- see SCsub.
//
// While intact, renders/collides as chunk 0 (the whole unfractured asset
// mesh) via one static PhysicsServer3D body. apply_radial_damage() runs the
// real fracture lifecycle; each newly-visible leaf chunk becomes its own
// rigid body with its own cooked convex collision hull and its own
// RenderingServer mesh instance (built directly from that chunk's render-
// mesh triangle soup), synced to the body's live transform every physics
// tick once anything has broken off.
class PhysXDestructible3D : public Node3D {
	GDCLASS(PhysXDestructible3D, Node3D);

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	void set_asset_path(const String &p_path);
	String get_asset_path() const { return asset_path; }
	void set_chunks_path(const String &p_path);
	String get_chunks_path() const { return chunks_path; }

	void set_material_override(const Ref<Material> &p_material);
	Ref<Material> get_material_override() const { return material_override_res; }

	// p_world_position: world-space damage origin (converted to this node's
	// local space internally, since chunk geometry is authored local-space).
	// Returns how many new rigid-body pieces this call produced.
	int apply_radial_damage(const Vector3 &p_world_position, float p_damage, float p_min_radius, float p_max_radius);

	PackedStringArray get_configuration_warnings() const override;

	PhysXDestructible3D();
	~PhysXDestructible3D();

private:
	String asset_path;
	String chunks_path;
	Ref<Material> material_override_res;

	void *asset_mem = nullptr;
	void *family_mem = nullptr;
	NvBlastFamily *family = nullptr;
	LocalVector<NvBlastActor *> live_actors;
	uint32_t asset_chunk_count = 0;
	uint32_t asset_bond_count = 0;

	// chunk_points[chunk_index] = that chunk's render-mesh triangle-soup
	// positions (object-local space), same source data and format
	// GodotPhysXBlastProbe uses -- see that class's header for the format.
	LocalVector<PackedVector3Array> chunk_points;

	struct ChunkVisual {
		RID body;
		RID shape;
		RID mesh;
		RID instance;
	};
	LocalVector<ChunkVisual> pieces;

	bool loaded = false;
	bool fractured = false;

	bool _load();
	void _spawn_intact();
	void _spawn_piece(uint32_t p_chunk_index, const Transform3D &p_transform, const Vector3 &p_linear_velocity);
	void _free_all_pieces();
	void _sync_transforms();
};

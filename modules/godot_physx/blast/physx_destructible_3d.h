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

#include "physx_blast_asset.h"

#include "core/templates/local_vector.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/material.h"

struct NvBlastAsset;
struct NvBlastFamily;
struct NvBlastActor;
class RenderingServer;

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
	void _validate_property(PropertyInfo &p_property) const;

public:
	void set_asset_path(const String &p_path);
	String get_asset_path() const { return asset_path; }
	void set_chunks_path(const String &p_path);
	String get_chunks_path() const { return chunks_path; }

	// Preferred over asset_path/chunks_path when set -- a real Resource
	// (PhysXBlastAuthoring::fracture_mesh()'s output, or one loaded from
	// disk) instead of the old two-file convention. asset_path/chunks_path
	// remain supported as a fallback when this isn't set, not deprecated.
	void set_blast_asset(const Ref<PhysXBlastAsset> &p_asset);
	Ref<PhysXBlastAsset> get_blast_asset() const { return blast_asset; }

	void set_material_override(const Ref<Material> &p_material);
	Ref<Material> get_material_override() const { return material_override_res; }

	// Every piece's RenderingServer instance is created directly via
	// instance_create2() (see _spawn_piece()), not through a real
	// GeometryInstance3D node -- so unlike a MeshInstance3D, nothing was
	// ever calling instance_geometry_set_flag(INSTANCE_FLAG_USE_BAKED_LIGHT/
	// USE_DYNAMIC_GI), and a VoxelGI had no way to tell these pieces apart
	// from ordinary invisible-to-GI geometry. Mirrors
	// GeometryInstance3D::GIMode exactly (same enum, same two flags) so it's
	// a drop-in familiar property.
	//
	// Defaults to Static, matching both GeometryInstance3D's own default
	// and -- more importantly -- the RenderingServer Instance's built-in
	// defaults before this property existed at all (baked_light = true,
	// dynamic_gi = false; see renderer_scene_cull.h). Dynamic would make
	// more physical sense for something that's often about to go tumbling
	// as debris, but a single fracture can spawn a couple dozen pieces at
	// once and a VoxelGI's dynamic-object tracking cost scales with how many
	// of those it has to follow every frame -- opting a pile of debris into
	// that by default risked being expensive well before anyone asked for
	// it, and silently changing existing scenes' behavior underneath them.
	// Opt into Dynamic per-node instead.
	enum GIMode {
		GI_MODE_DISABLED,
		GI_MODE_STATIC,
		GI_MODE_DYNAMIC,
	};
	void set_gi_mode(GIMode p_mode);
	GIMode get_gi_mode() const { return gi_mode; }

	void set_shatter_speed(float p_speed) { shatter_speed = p_speed; }
	float get_shatter_speed() const { return shatter_speed; }

	// The whole intact object's mass (same meaning as RigidBody3D.mass).
	// Every piece used to get PhysicsServer3D's own flat default (1.0)
	// regardless of size -- checked how Unreal's own Blast integration
	// handles this (BlastMeshComponent.cpp) and it distributes mass across
	// split pieces proportional to each one's actual volume, conserving the
	// intact object's total mass; this does the same (see
	// _compute_chunk_volumes()/_chunk_mass()) rather than giving a huge
	// remaining chunk and a tiny sliver the same weight.
	//
	// mass is auto-computed (a fixed internal density, not its own exposed
	// property -- neither Godot nor Unreal actually expose raw density as a
	// per-component number; Unreal's lives on a shared PhysicalMaterial
	// asset, and Godot has no density concept anywhere) from the intact
	// mesh's real volume while auto_mass is true (the default) -- read-only
	// in the Inspector then, so it reads as "here's what the object would
	// weigh," not an editable field. Uncheck auto_mass for a true override:
	// mass becomes a plain editable value, exactly what you set it to,
	// until you check auto_mass again (which recomputes fresh and locks it
	// back to read-only) -- see set_auto_mass()'s own note on the one real
	// bug this went through before landing here (the Inspector not
	// noticing auto_mass had changed).
	// A script calling this directly (not through the Inspector, which is
	// read-only whenever auto_mass is true and so can't reach this in the
	// first place) still counts as an explicit override -- turns auto_mass
	// off too, so it can't get silently overwritten by a later reload.
	void set_mass(float p_mass);
	float get_mass() const { return mass; }

	// See set_mass()'s note. Calls notify_property_list_changed() --
	// without it, the Inspector never re-checks whether `mass` should still
	// be read-only after this toggles, so unchecking auto_mass looked like
	// it did nothing (the field stayed visually grayed out even though the
	// underlying state was correct) -- the actual bug in the first version
	// of this, not the read-only-while-auto design itself.
	void set_auto_mass(bool p_auto);
	bool get_auto_mass() const { return auto_mass; }

	// If true, the intact piece is a real dynamic (BODY_MODE_RIGID) body --
	// it falls under gravity and collides normally, like any other physics
	// object, instead of hanging in place until something explicitly calls
	// apply_radial_damage() on it. Also enables impact_strength/
	// impact_damage_scale: a hard enough collision auto-triggers fracture,
	// the same way it does not just gravity, matching stacked destructible
	// props in UE's Blast integration. False keeps the original behavior
	// (static intact placeholder, fracture only ever explicit) so existing
	// scenes built around that don't change under them.
	void set_dynamic(bool p_dynamic) { dynamic = p_dynamic; }
	bool get_dynamic() const { return dynamic; }

	// Initial NvBlast bond/chunk health (both, uniformly) -- how much
	// cumulative damage the support structure can absorb before bonds start
	// breaking. Higher health needs more damage (a harder impact, or more of
	// them) to fracture the same object.
	void set_health(float p_health) { health = p_health; }
	float get_health() const { return health; }

	// dynamic-only: the minimum contact impulse magnitude (mass * velocity
	// change, in Godot's physics units) that counts as an impact at all --
	// below this, collisions (landing softly, gentle bumps) are ignored.
	void set_impact_strength(float p_strength) { impact_strength = p_strength; }
	float get_impact_strength() const { return impact_strength; }

	// dynamic-only: damage fed into apply_radial_damage() per unit of
	// impulse magnitude above impact_strength -- how readily an impact that
	// does exceed the threshold actually breaks bonds.
	void set_impact_damage_scale(float p_scale) { impact_damage_scale = p_scale; }
	float get_impact_damage_scale() const { return impact_damage_scale; }

	// dynamic-only: max_radius passed to the auto-triggered apply_radial_damage
	// call, centered on the contact point -- how far a hard impact's damage
	// reaches into the rest of the object.
	void set_impact_radius(float p_radius) { impact_radius = p_radius; }
	float get_impact_radius() const { return impact_radius; }

	// A piece (fractured debris, or the intact body if dynamic) whose world Y
	// falls below this is freed automatically -- bounds memory/body count for
	// a scene where debris can fall indefinitely (no floor everywhere, a
	// gap, missing the pile entirely) instead of accumulating forever.
	// Default is low enough not to trigger on an ordinary scene; set it to
	// match this scene's actual floor/void.
	void set_kill_y(float p_y) { kill_y = p_y; }
	float get_kill_y() const { return kill_y; }

	// p_world_position: world-space damage origin (converted to this node's
	// local space internally, since chunk geometry is authored local-space).
	// Returns how many new rigid-body pieces this call produced. Each new
	// piece gets a real outward velocity -- radial direction from the damage
	// origin to that piece's own centroid, scaled by shatter_speed and the
	// same distance falloff the damage itself uses, plus a slight upward
	// bias -- same shape as demo/cpu/physx_playground.gd's own _blast(),
	// just computed here so it's intrinsic to fracturing, not a separate
	// demo-script concern layered on top.
	int apply_radial_damage(const Vector3 &p_world_position, float p_damage, float p_min_radius, float p_max_radius);

	PackedStringArray get_configuration_warnings() const override;

	PhysXDestructible3D();
	~PhysXDestructible3D();

private:
	String asset_path;
	String chunks_path;
	Ref<PhysXBlastAsset> blast_asset;
	Ref<Material> material_override_res;
	GIMode gi_mode = GI_MODE_STATIC;
	float shatter_speed = 8.0f;
	float mass = 1.0f;
	bool auto_mass = true;
	bool dynamic = false;
	float health = 1.0f;
	float impact_strength = 5.0f;
	float impact_damage_scale = 1.0f;
	float impact_radius = 5.0f;
	float kill_y = -1000.0f;

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

	// chunk_volumes[chunk_index] = that chunk's volume (object-local space,
	// so no transform/scale applied); total_leaf_volume = sum of every leaf
	// chunk's volume (excludes chunk 0, the whole unfractured mesh). Both
	// computed once in _compute_chunk_volumes(), called from _load(). See
	// set_mass() for why.
	LocalVector<double> chunk_volumes;
	double total_leaf_volume = 0.0;

	struct ChunkVisual {
		RID body;
		RID shape;
		RID mesh;
		RID instance;
	};
	LocalVector<ChunkVisual> pieces;

	bool loaded = false;
	bool fractured = false;

	// PxTransform (a physics body's pose) never carries scale -- confirmed
	// directly: a real repro with a (20, 10, 1) scaled destructible showed
	// the node's own global_transform basis reduced to identity after just
	// a few physics ticks. Every place that pushes a physics-read transform
	// onto the node or a piece's RenderingServer instance (the dynamic
	// intact body's own sync, and _sync_transforms() for fractured debris)
	// only has a scale-free rotation to work with at that point -- this is
	// the scale to re-apply on top of it, captured fresh whenever the
	// intact piece is (re)spawned or its transform is pushed by something
	// other than physics (see _spawn_intact()/NOTIFICATION_TRANSFORM_CHANGED),
	// i.e. always from a moment before physics has had any chance to
	// overwrite it.
	Vector3 spawn_scale = Vector3(1, 1, 1);

	bool _load();
	bool _load_asset_bytes(const PackedByteArray &p_bytes);
	void _compute_chunk_volumes();
	// mass distributed proportional to p_chunk_index's share of
	// total_leaf_volume (floored so a sliver never gets a near-zero mass),
	// except chunk 0 (the whole intact mesh), which just gets `mass` itself.
	float _chunk_mass(uint32_t p_chunk_index) const;
	// Tears down any existing family/pieces and, if already inside the world,
	// immediately re-loads and re-spawns from whatever asset_path/chunks_path/
	// blast_asset now point at -- called from those setters so assigning a
	// new asset (or dragging one onto an empty node already in the tree)
	// shows up right away instead of only on the next time the node enters
	// the tree (e.g. reopening the scene).
	void _reload();
	void _spawn_intact();
	// p_physics: create a PhysicsServer3D body+shape for this piece too, not
	// just its RenderingServer mesh instance. False in the editor (no physics
	// simulation runs there anyway) so the node still shows *something* in
	// the viewport -- ChunkVisual::body/shape stay RID() in that case.
	void _spawn_piece(uint32_t p_chunk_index, const Transform3D &p_transform, const Vector3 &p_linear_velocity, bool p_physics = true);
	// See set_gi_mode()'s own note on why this is needed at all.
	void _apply_gi_mode(RenderingServer *p_rs, RID p_instance) const;
	Vector3 _chunk_centroid_local(uint32_t p_chunk_index) const;
	void _free_all_pieces();
	void _sync_transforms();
	// dynamic-only, called every physics tick while still intact: reads the
	// intact body's reported contacts and auto-triggers apply_radial_damage()
	// if any single contact's impulse exceeds impact_strength.
	void _check_impact_fracture();
};

VARIANT_ENUM_CAST(PhysXDestructible3D::GIMode);

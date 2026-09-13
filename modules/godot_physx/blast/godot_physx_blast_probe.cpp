/**************************************************************************/
/*  godot_physx_blast_probe.cpp                                           */
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

#include "godot_physx_blast_probe.h"

#include "core/io/file_access.h"
#include "core/object/class_db.h"
#include "core/variant/variant.h"
#include "servers/physics_3d/physics_server_3d.h"

#include <NvBlast.h>
#include <NvBlastExtDamageShaders.h>
#include <NvBlastTypes.h>

#include <cstdlib>

namespace {

void blast_log(int32_t p_type, const char *p_msg, const char *p_file, int32_t p_line) {
	if (p_type <= 1) { // NvBlastMessage::Error / Warning
		ERR_PRINT(vformat("Blast: %s (%s:%d)", p_msg, p_file, p_line));
	}
}

void *aligned_alloc_16(size_t p_size) {
#if defined(_WIN32)
	return _aligned_malloc(p_size, 16);
#else
	void *p = nullptr;
	if (posix_memalign(&p, 16, p_size) != 0) {
		return nullptr;
	}
	return p;
#endif
}

void aligned_free_16(void *p_mem) {
#if defined(_WIN32)
	_aligned_free(p_mem);
#else
	free(p_mem);
#endif
}

} //namespace

void GodotPhysXBlastProbe::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_space", "space"), &GodotPhysXBlastProbe::set_space);
	ClassDB::bind_method(D_METHOD("set_base_transform", "transform"), &GodotPhysXBlastProbe::set_base_transform);
	ClassDB::bind_method(D_METHOD("set_base_velocity", "velocity"), &GodotPhysXBlastProbe::set_base_velocity);
	ClassDB::bind_method(D_METHOD("load", "asset_path", "chunks_path"), &GodotPhysXBlastProbe::load);
	ClassDB::bind_method(D_METHOD("apply_radial_damage", "position", "damage", "min_radius", "max_radius"), &GodotPhysXBlastProbe::apply_radial_damage);
	ClassDB::bind_method(D_METHOD("get_body_count"), &GodotPhysXBlastProbe::get_body_count);
	ClassDB::bind_method(D_METHOD("get_body", "index"), &GodotPhysXBlastProbe::get_body);
	ClassDB::bind_method(D_METHOD("get_live_actor_count"), &GodotPhysXBlastProbe::get_live_actor_count);
}

GodotPhysXBlastProbe::GodotPhysXBlastProbe() {}

GodotPhysXBlastProbe::~GodotPhysXBlastProbe() {
	_free_all();
	if (family_mem) {
		aligned_free_16(family_mem);
	}
	if (asset_mem) {
		aligned_free_16(asset_mem);
	}
}

void GodotPhysXBlastProbe::_free_all() {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	for (const RID &body : bodies) {
		ps->free_rid(body);
	}
	for (const RID &shape : shapes) {
		ps->free_rid(shape);
	}
	bodies.clear();
	shapes.clear();
}

bool GodotPhysXBlastProbe::load(const String &p_asset_path, const String &p_chunks_path) {
	// -- Raw NvBlastAsset bytes: a single relocatable memory block (per
	// NvBlast's own design), so loading is just "read the bytes into a
	// 16-byte-aligned buffer" -- no format of our own on top.
	Ref<FileAccess> af = FileAccess::open(p_asset_path, FileAccess::READ);
	ERR_FAIL_COND_V_MSG(af.is_null(), false, vformat("Blast probe: cannot open asset file '%s'.", p_asset_path));
	const uint64_t asset_size = af->get_length();
	asset_mem = aligned_alloc_16((size_t)asset_size);
	ERR_FAIL_NULL_V(asset_mem, false);
	af->get_buffer(reinterpret_cast<uint8_t *>(asset_mem), asset_size);

	NvBlastAsset *asset = reinterpret_cast<NvBlastAsset *>(asset_mem);
	asset_chunk_count = NvBlastAssetGetChunkCount(asset, blast_log);
	asset_bond_count = NvBlastAssetGetBondCount(asset, blast_log);

	const size_t family_size = NvBlastAssetGetFamilyMemorySize(asset, blast_log);
	family_mem = aligned_alloc_16(family_size);
	ERR_FAIL_NULL_V(family_mem, false);
	family = NvBlastAssetCreateFamily(family_mem, asset, blast_log);
	ERR_FAIL_NULL_V_MSG(family, false, "Blast probe: NvBlastAssetCreateFamily failed.");

	NvBlastActorDesc actor_desc;
	actor_desc.uniformInitialBondHealth = 1.0f;
	actor_desc.initialBondHealths = nullptr;
	actor_desc.uniformInitialLowerSupportChunkHealth = 1.0f;
	actor_desc.initialSupportChunkHealths = nullptr;

	const size_t scratch_size = NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, blast_log);
	LocalVector<uint8_t> scratch;
	scratch.resize((uint32_t)scratch_size);
	NvBlastActor *first_actor = NvBlastFamilyCreateFirstActor(family, &actor_desc, scratch.ptr(), blast_log);
	ERR_FAIL_NULL_V_MSG(first_actor, false, "Blast probe: NvBlastFamilyCreateFirstActor failed.");
	live_actors.push_back(first_actor);

	// -- Per-chunk render-mesh triangle soup: a plain text format written by
	// the standalone authoring tool (blast_test_gen.cpp), not part of the
	// Blast SDK itself. See that file's header comment for the exact grammar.
	Ref<FileAccess> cf = FileAccess::open(p_chunks_path, FileAccess::READ);
	ERR_FAIL_COND_V_MSG(cf.is_null(), false, vformat("Blast probe: cannot open chunks file '%s'.", p_chunks_path));

	const Vector<String> header = cf->get_line().split(" ", false);
	ERR_FAIL_COND_V_MSG(header.size() != 2 || header[0] != "chunks", false, "Blast probe: malformed chunks file header.");
	const uint32_t chunk_count = header[1].to_int();
	chunk_points.resize(chunk_count);

	for (uint32_t i = 0; i < chunk_count && !cf->eof_reached(); i++) {
		const Vector<String> chunk_header = cf->get_line().split(" ", false);
		ERR_FAIL_COND_V_MSG(chunk_header.size() != 3 || chunk_header[0] != "chunk", false, "Blast probe: malformed chunk header line.");
		const uint32_t chunk_index = chunk_header[1].to_int();
		const uint32_t tri_count = chunk_header[2].to_int();
		ERR_FAIL_COND_V(chunk_index >= chunk_count, false);

		PackedVector3Array points;
		points.resize(tri_count * 3);
		for (uint32_t t = 0; t < tri_count; t++) {
			const Vector<String> fields = cf->get_line().split(" ", false);
			ERR_FAIL_COND_V_MSG(fields.size() != 9, false, "Blast probe: malformed triangle line.");
			for (int v = 0; v < 3; v++) {
				points.write[t * 3 + v] = Vector3(
						fields[v * 3 + 0].to_float(),
						fields[v * 3 + 1].to_float(),
						fields[v * 3 + 2].to_float());
			}
		}
		chunk_points[chunk_index] = points;
	}

	return true;
}

void GodotPhysXBlastProbe::_spawn_body_for_chunk(uint32_t p_chunk_index) {
	ERR_FAIL_INDEX(p_chunk_index, chunk_points.size());
	const PackedVector3Array &points = chunk_points[p_chunk_index];
	ERR_FAIL_COND_MSG(points.size() < 4, vformat("Blast probe: chunk %d has too few vertices for a convex hull.", p_chunk_index));

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	RID shape = ps->convex_polygon_shape_create();
	ps->shape_set_data(shape, points);

	RID body = ps->body_create();
	ps->body_set_mode(body, PhysicsServer3D::BODY_MODE_RIGID);
	ps->body_add_shape(body, shape);
	ps->body_set_state(body, PhysicsServer3D::BODY_STATE_TRANSFORM, base_transform);
	ps->body_set_state(body, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, base_velocity);
	if (space.is_valid()) {
		ps->body_set_space(body, space);
	}

	bodies.push_back(body);
	shapes.push_back(shape);
}

int GodotPhysXBlastProbe::apply_radial_damage(const Vector3 &p_position, float p_damage, float p_min_radius, float p_max_radius) {
	ERR_FAIL_NULL_V_MSG(family, 0, "Blast probe: load() must succeed before apply_radial_damage().");

	NvBlastExtRadialDamageDesc damage_desc;
	damage_desc.damage = p_damage;
	damage_desc.position[0] = p_position.x;
	damage_desc.position[1] = p_position.y;
	damage_desc.position[2] = p_position.z;
	damage_desc.minRadius = p_min_radius;
	damage_desc.maxRadius = p_max_radius;

	// The falloff shaders cast programParams to NvBlastExtProgramParams
	// internally and dereference its .damageDesc field -- passing the bare
	// NvBlastExtRadialDamageDesc directly is a silent access violation (found
	// and fixed via the standalone runtime-mechanism prototype before this
	// bridge was written; see the module's Blast planning notes).
	NvBlastExtProgramParams program_params(&damage_desc);

	NvBlastDamageProgram program;
	program.graphShaderFunction = NvBlastExtFalloffGraphShader;
	program.subgraphShaderFunction = NvBlastExtFalloffSubgraphShader;

	LocalVector<NvBlastBondFractureData> bond_buf;
	LocalVector<NvBlastChunkFractureData> chunk_buf;
	bond_buf.resize(asset_bond_count);
	chunk_buf.resize(asset_chunk_count);

	int spawned = 0;
	// Snapshot the live set up front -- actors added to live_actors as a
	// result of this pass (this call's own new children) are next damage
	// event's concern, not this one's.
	LocalVector<NvBlastActor *> actors_to_process(live_actors);
	live_actors.clear();

	for (NvBlastActor *actor : actors_to_process) {
		NvBlastFractureBuffers commands;
		commands.bondFractureCount = bond_buf.size();
		commands.chunkFractureCount = chunk_buf.size();
		commands.bondFractures = bond_buf.ptr();
		commands.chunkFractures = chunk_buf.ptr();

		NvBlastActorGenerateFracture(&commands, actor, program, &program_params, blast_log, nullptr);
		NvBlastActorApplyFracture(&commands, actor, &commands, blast_log, nullptr);

		if (!NvBlastActorCanFracture(actor, blast_log)) {
			live_actors.push_back(actor); // untouched by this event, stays live
			continue;
		}

		const uint32_t max_new = NvBlastActorGetMaxActorCountForSplit(actor, blast_log);
		LocalVector<NvBlastActor *> new_actors;
		new_actors.resize(max_new);
		const size_t split_scratch_size = NvBlastActorGetRequiredScratchForSplit(actor, blast_log);
		LocalVector<uint8_t> split_scratch;
		split_scratch.resize((uint32_t)split_scratch_size);

		NvBlastActorSplitEvent split_result;
		split_result.newActors = new_actors.ptr();
		const uint32_t new_count = NvBlastActorSplit(&split_result, actor, max_new, split_scratch.ptr(), blast_log, nullptr);

		if (new_count == 0) {
			live_actors.push_back(actor); // no change -- still live, unsplit
			continue;
		}

		for (uint32_t i = 0; i < new_count; i++) {
			NvBlastActor *new_actor = new_actors[i];
			live_actors.push_back(new_actor);

			const uint32_t visible_count = NvBlastActorGetVisibleChunkCount(new_actor, blast_log);
			LocalVector<uint32_t> visible;
			visible.resize(visible_count);
			NvBlastActorGetVisibleChunkIndices(visible.ptr(), visible_count, new_actor, blast_log);
			for (uint32_t v = 0; v < visible_count; v++) {
				_spawn_body_for_chunk(visible[v]);
				spawned++;
			}
		}
		// split_result.deletedActor (== actor here, since it fractured) is
		// deactivated by NvBlastActorSplit itself -- not re-added to live_actors.
	}

	return spawned;
}

RID GodotPhysXBlastProbe::get_body(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)bodies.size(), RID());
	return bodies[p_index];
}

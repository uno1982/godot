/**************************************************************************/
/*  physx_destructible_3d.cpp                                             */
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

#include "physx_destructible_3d.h"

#include "core/config/engine.h"
#include "core/io/file_access.h"
#include "core/object/class_db.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/rendering/rendering_server.h"

#include <NvBlast.h>
#include <NvBlastExtDamageShaders.h>
#include <NvBlastTypes.h>

#include <cstdlib>

namespace {

void blast_log(int32_t p_type, const char *p_msg, const char *p_file, int32_t p_line) {
	if (p_type <= 1) {
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

void PhysXDestructible3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_asset_path", "path"), &PhysXDestructible3D::set_asset_path);
	ClassDB::bind_method(D_METHOD("get_asset_path"), &PhysXDestructible3D::get_asset_path);
	ClassDB::bind_method(D_METHOD("set_chunks_path", "path"), &PhysXDestructible3D::set_chunks_path);
	ClassDB::bind_method(D_METHOD("get_chunks_path"), &PhysXDestructible3D::get_chunks_path);
	ClassDB::bind_method(D_METHOD("set_material_override", "material"), &PhysXDestructible3D::set_material_override);
	ClassDB::bind_method(D_METHOD("get_material_override"), &PhysXDestructible3D::get_material_override);
	ClassDB::bind_method(D_METHOD("set_shatter_speed", "speed"), &PhysXDestructible3D::set_shatter_speed);
	ClassDB::bind_method(D_METHOD("get_shatter_speed"), &PhysXDestructible3D::get_shatter_speed);
	ClassDB::bind_method(D_METHOD("apply_radial_damage", "world_position", "damage", "min_radius", "max_radius"), &PhysXDestructible3D::apply_radial_damage);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "asset_path", PROPERTY_HINT_FILE, "*.asset"), "set_asset_path", "get_asset_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "chunks_path", PROPERTY_HINT_FILE, "*.chunks"), "set_chunks_path", "get_chunks_path");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material_override", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_material_override", "get_material_override");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "shatter_speed", PROPERTY_HINT_RANGE, "0,50,0.1"), "set_shatter_speed", "get_shatter_speed");
}

PhysXDestructible3D::PhysXDestructible3D() {
	set_notify_transform(true);
}

PhysXDestructible3D::~PhysXDestructible3D() {
	_free_all_pieces();
	if (family_mem) {
		aligned_free_16(family_mem);
	}
	if (asset_mem) {
		aligned_free_16(asset_mem);
	}
}

void PhysXDestructible3D::set_asset_path(const String &p_path) {
	asset_path = p_path;
	update_configuration_warnings();
}

void PhysXDestructible3D::set_chunks_path(const String &p_path) {
	chunks_path = p_path;
	update_configuration_warnings();
}

void PhysXDestructible3D::set_material_override(const Ref<Material> &p_material) {
	material_override_res = p_material;
	RenderingServer *rs = RenderingServer::get_singleton();
	const RID mat_rid = material_override_res.is_valid() ? material_override_res->get_rid() : RID();
	for (const ChunkVisual &piece : pieces) {
		rs->instance_geometry_set_material_override(piece.instance, mat_rid);
	}
}

PackedStringArray PhysXDestructible3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();
	if (asset_path.is_empty() || chunks_path.is_empty()) {
		warnings.push_back("PhysXDestructible3D needs both asset_path and chunks_path to render or collide.");
	}
	return warnings;
}

void PhysXDestructible3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			if (Engine::get_singleton()->is_editor_hint()) {
				break; // no PhysX/RenderingServer simulation state in the editor -- matches this module's other nodes
			}
			if (!loaded) {
				loaded = _load();
			}
			if (loaded && !fractured && pieces.is_empty()) {
				_spawn_intact();
			}
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			_free_all_pieces();
		} break;
		case NOTIFICATION_TRANSFORM_CHANGED: {
			if (!fractured && pieces.size() == 1) {
				// Still intact -- the single "piece" IS this node, so keep its
				// body/visual glued to wherever the node itself moves.
				PhysicsServer3D::get_singleton()->body_set_state(pieces[0].body, PhysicsServer3D::BODY_STATE_TRANSFORM, get_global_transform());
				RenderingServer::get_singleton()->instance_set_transform(pieces[0].instance, get_global_transform());
			}
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (fractured) {
				_sync_transforms();
			}
		} break;
	}
}

bool PhysXDestructible3D::_load() {
	Ref<FileAccess> af = FileAccess::open(asset_path, FileAccess::READ);
	ERR_FAIL_COND_V_MSG(af.is_null(), false, vformat("PhysXDestructible3D: cannot open asset_path '%s'.", asset_path));
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
	ERR_FAIL_NULL_V_MSG(family, false, "PhysXDestructible3D: NvBlastAssetCreateFamily failed.");

	NvBlastActorDesc actor_desc;
	actor_desc.uniformInitialBondHealth = 1.0f;
	actor_desc.initialBondHealths = nullptr;
	actor_desc.uniformInitialLowerSupportChunkHealth = 1.0f;
	actor_desc.initialSupportChunkHealths = nullptr;

	const size_t scratch_size = NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, blast_log);
	LocalVector<uint8_t> scratch;
	scratch.resize((uint32_t)scratch_size);
	NvBlastActor *first_actor = NvBlastFamilyCreateFirstActor(family, &actor_desc, scratch.ptr(), blast_log);
	ERR_FAIL_NULL_V_MSG(first_actor, false, "PhysXDestructible3D: NvBlastFamilyCreateFirstActor failed.");
	live_actors.push_back(first_actor);

	Ref<FileAccess> cf = FileAccess::open(chunks_path, FileAccess::READ);
	ERR_FAIL_COND_V_MSG(cf.is_null(), false, vformat("PhysXDestructible3D: cannot open chunks_path '%s'.", chunks_path));

	const Vector<String> header = cf->get_line().split(" ", false);
	ERR_FAIL_COND_V_MSG(header.size() != 2 || header[0] != "chunks", false, "PhysXDestructible3D: malformed chunks file header.");
	const uint32_t chunk_count = header[1].to_int();
	chunk_points.resize(chunk_count);

	for (uint32_t i = 0; i < chunk_count && !cf->eof_reached(); i++) {
		const Vector<String> chunk_header = cf->get_line().split(" ", false);
		ERR_FAIL_COND_V_MSG(chunk_header.size() != 3 || chunk_header[0] != "chunk", false, "PhysXDestructible3D: malformed chunk header line.");
		const uint32_t chunk_index = chunk_header[1].to_int();
		const uint32_t tri_count = chunk_header[2].to_int();
		ERR_FAIL_COND_V(chunk_index >= chunk_count, false);

		PackedVector3Array points;
		points.resize(tri_count * 3);
		for (uint32_t t = 0; t < tri_count; t++) {
			const Vector<String> fields = cf->get_line().split(" ", false);
			ERR_FAIL_COND_V_MSG(fields.size() != 9, false, "PhysXDestructible3D: malformed triangle line.");
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

void PhysXDestructible3D::_spawn_intact() {
	// Chunk 0 is always the fracture-tool's root chunk -- the whole
	// unfractured asset mesh (see blast_test_gen.cpp) -- so rendering/
	// colliding as chunk 0 while nothing has broken is exactly correct, not
	// an approximation.
	_spawn_piece(0, get_global_transform(), Vector3());
	PhysicsServer3D::get_singleton()->body_set_mode(pieces[0].body, PhysicsServer3D::BODY_MODE_STATIC);
}

void PhysXDestructible3D::_spawn_piece(uint32_t p_chunk_index, const Transform3D &p_transform, const Vector3 &p_linear_velocity) {
	ERR_FAIL_INDEX(p_chunk_index, chunk_points.size());
	const PackedVector3Array &points = chunk_points[p_chunk_index];
	const uint32_t tri_count = points.size() / 3;
	ERR_FAIL_COND_MSG(points.size() < 4, vformat("PhysXDestructible3D: chunk %d has too few vertices for a convex hull.", p_chunk_index));

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	RID shape = ps->convex_polygon_shape_create();
	ps->shape_set_data(shape, points);

	RID body = ps->body_create();
	ps->body_set_mode(body, PhysicsServer3D::BODY_MODE_RIGID);
	ps->body_add_shape(body, shape);
	ps->body_set_state(body, PhysicsServer3D::BODY_STATE_TRANSFORM, p_transform);
	ps->body_set_state(body, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, p_linear_velocity);
	if (is_inside_world() && get_world_3d().is_valid()) {
		ps->body_set_space(body, get_world_3d()->get_space());
	}

	// Flat per-triangle normals -- the authoring dump only stores positions.
	// (c-a).cross(b-a), not the more intuitive (b-a).cross(c-a) -- Blast's
	// AuthoringResult::geometry triangle winding is the opposite of what
	// PRIMITIVE_TRIANGLES/CCW-front-face expects here; the wrong order cooked
	// clean but rendered every visible face pitch-black (normals pointing
	// inward), confirmed by a real screenshot before/after flipping it.
	PackedVector3Array normals;
	normals.resize(points.size());
	for (uint32_t t = 0; t < tri_count; t++) {
		const Vector3 a = points[t * 3 + 0];
		const Vector3 b = points[t * 3 + 1];
		const Vector3 c = points[t * 3 + 2];
		const Vector3 n = (c - a).cross(b - a).normalized();
		normals.write[t * 3 + 0] = n;
		normals.write[t * 3 + 1] = n;
		normals.write[t * 3 + 2] = n;
	}

	RenderingServer *rs = RenderingServer::get_singleton();
	RID mesh = rs->mesh_create();
	Array arrays;
	arrays.resize(RSE::ARRAY_MAX);
	arrays[RSE::ARRAY_VERTEX] = points;
	arrays[RSE::ARRAY_NORMAL] = normals;
	rs->mesh_add_surface_from_arrays(mesh, RSE::PRIMITIVE_TRIANGLES, arrays);

	RID instance = rs->instance_create2(mesh, get_world_3d().is_valid() ? get_world_3d()->get_scenario() : RID());
	rs->instance_set_transform(instance, p_transform);
	if (material_override_res.is_valid()) {
		rs->instance_geometry_set_material_override(instance, material_override_res->get_rid());
	}

	ChunkVisual piece;
	piece.body = body;
	piece.shape = shape;
	piece.mesh = mesh;
	piece.instance = instance;
	pieces.push_back(piece);
}

Vector3 PhysXDestructible3D::_chunk_centroid_local(uint32_t p_chunk_index) const {
	ERR_FAIL_INDEX_V(p_chunk_index, chunk_points.size(), Vector3());
	const PackedVector3Array &points = chunk_points[p_chunk_index];
	if (points.is_empty()) {
		return Vector3();
	}
	Vector3 sum;
	for (int i = 0; i < points.size(); i++) {
		sum += points[i];
	}
	return sum / (real_t)points.size();
}

void PhysXDestructible3D::_free_all_pieces() {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		ps->free_rid(piece.body);
		ps->free_rid(piece.shape);
		rs->free_rid(piece.instance);
		rs->free_rid(piece.mesh);
	}
	pieces.clear();
}

void PhysXDestructible3D::_sync_transforms() {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		PhysicsDirectBodyState3D *state = ps->body_get_direct_state(piece.body);
		if (state) {
			rs->instance_set_transform(piece.instance, state->get_transform());
		}
	}
}

int PhysXDestructible3D::apply_radial_damage(const Vector3 &p_world_position, float p_damage, float p_min_radius, float p_max_radius) {
	ERR_FAIL_NULL_V_MSG(family, 0, "PhysXDestructible3D: node not loaded (missing/invalid asset_path or chunks_path?).");

	if (!fractured) {
		// First hit: the intact single piece is about to potentially split --
		// drop its placeholder body/visual regardless of whether this
		// particular hit actually breaks anything, since from here on pieces
		// are tracked per-actor, not as one intact whole.
		_free_all_pieces();
		fractured = true;
		set_physics_process_internal(true);
	}

	const Vector3 local_position = get_global_transform().affine_inverse().xform(p_world_position);

	NvBlastExtRadialDamageDesc damage_desc;
	damage_desc.damage = p_damage;
	damage_desc.position[0] = local_position.x;
	damage_desc.position[1] = local_position.y;
	damage_desc.position[2] = local_position.z;
	damage_desc.minRadius = p_min_radius;
	damage_desc.maxRadius = p_max_radius;

	// The falloff shaders cast programParams to NvBlastExtProgramParams
	// internally -- passing the bare desc directly is a silent access
	// violation (found and fixed in the standalone runtime-mechanism
	// prototype before this bridge was written).
	NvBlastExtProgramParams program_params(&damage_desc);

	NvBlastDamageProgram program;
	program.graphShaderFunction = NvBlastExtFalloffGraphShader;
	program.subgraphShaderFunction = NvBlastExtFalloffSubgraphShader;

	LocalVector<NvBlastBondFractureData> bond_buf;
	LocalVector<NvBlastChunkFractureData> chunk_buf;
	bond_buf.resize(asset_bond_count);
	chunk_buf.resize(asset_chunk_count);

	int spawned = 0;
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
			live_actors.push_back(actor);
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
			live_actors.push_back(actor);
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
				// No real pre-fracture body exists to inherit a genuine
				// per-actor velocity from (chunk 0's intact body was just a
				// placeholder, freed above) -- instead give each piece an
				// outward "shatter" kick from the damage origin to its own
				// centroid, same radial-direction + distance-falloff +
				// slight-upward-bias shape as demo/cpu/physx_playground.gd's
				// _blast(), just computed intrinsically here rather than
				// bolted on by whatever script happens to call this.
				const Vector3 centroid_world = get_global_transform().xform(_chunk_centroid_local(visible[v]));
				const Vector3 offset = centroid_world - p_world_position;
				const real_t dist = offset.length();
				const Vector3 dir = dist > 0.001 ? (offset / dist) : Vector3(0, 1, 0);
				const real_t falloff = CLAMP(1.0 - dist / (real_t)p_max_radius, 0.0, 1.0);
				const Vector3 piece_velocity = (dir + Vector3(0, 0.3, 0)).normalized() * shatter_speed * falloff;
				_spawn_piece(visible[v], get_global_transform(), piece_velocity);
				spawned++;
			}
		}
	}

	return spawned;
}

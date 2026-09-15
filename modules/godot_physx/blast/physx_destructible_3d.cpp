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
#include "core/math/triangle_mesh.h"
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
	ClassDB::bind_method(D_METHOD("set_blast_asset", "asset"), &PhysXDestructible3D::set_blast_asset);
	ClassDB::bind_method(D_METHOD("get_blast_asset"), &PhysXDestructible3D::get_blast_asset);
	ClassDB::bind_method(D_METHOD("set_material_override", "material"), &PhysXDestructible3D::set_material_override);
	ClassDB::bind_method(D_METHOD("get_material_override"), &PhysXDestructible3D::get_material_override);
	ClassDB::bind_method(D_METHOD("set_gi_mode", "mode"), &PhysXDestructible3D::set_gi_mode);
	ClassDB::bind_method(D_METHOD("get_gi_mode"), &PhysXDestructible3D::get_gi_mode);
	ClassDB::bind_method(D_METHOD("set_cast_shadow", "setting"), &PhysXDestructible3D::set_cast_shadow);
	ClassDB::bind_method(D_METHOD("get_cast_shadow"), &PhysXDestructible3D::get_cast_shadow);
	ClassDB::bind_method(D_METHOD("set_transparency", "transparency"), &PhysXDestructible3D::set_transparency);
	ClassDB::bind_method(D_METHOD("get_transparency"), &PhysXDestructible3D::get_transparency);
	ClassDB::bind_method(D_METHOD("set_material_overlay", "material"), &PhysXDestructible3D::set_material_overlay);
	ClassDB::bind_method(D_METHOD("get_material_overlay"), &PhysXDestructible3D::get_material_overlay);
	ClassDB::bind_method(D_METHOD("set_extra_cull_margin", "margin"), &PhysXDestructible3D::set_extra_cull_margin);
	ClassDB::bind_method(D_METHOD("get_extra_cull_margin"), &PhysXDestructible3D::get_extra_cull_margin);
	ClassDB::bind_method(D_METHOD("set_lod_bias", "bias"), &PhysXDestructible3D::set_lod_bias);
	ClassDB::bind_method(D_METHOD("get_lod_bias"), &PhysXDestructible3D::get_lod_bias);
	ClassDB::bind_method(D_METHOD("set_ignore_occlusion_culling", "ignore"), &PhysXDestructible3D::set_ignore_occlusion_culling);
	ClassDB::bind_method(D_METHOD("get_ignore_occlusion_culling"), &PhysXDestructible3D::get_ignore_occlusion_culling);
	ClassDB::bind_method(D_METHOD("set_visibility_range_begin", "distance"), &PhysXDestructible3D::set_visibility_range_begin);
	ClassDB::bind_method(D_METHOD("get_visibility_range_begin"), &PhysXDestructible3D::get_visibility_range_begin);
	ClassDB::bind_method(D_METHOD("set_visibility_range_end", "distance"), &PhysXDestructible3D::set_visibility_range_end);
	ClassDB::bind_method(D_METHOD("get_visibility_range_end"), &PhysXDestructible3D::get_visibility_range_end);
	ClassDB::bind_method(D_METHOD("set_visibility_range_begin_margin", "distance"), &PhysXDestructible3D::set_visibility_range_begin_margin);
	ClassDB::bind_method(D_METHOD("get_visibility_range_begin_margin"), &PhysXDestructible3D::get_visibility_range_begin_margin);
	ClassDB::bind_method(D_METHOD("set_visibility_range_end_margin", "distance"), &PhysXDestructible3D::set_visibility_range_end_margin);
	ClassDB::bind_method(D_METHOD("get_visibility_range_end_margin"), &PhysXDestructible3D::get_visibility_range_end_margin);
	ClassDB::bind_method(D_METHOD("set_visibility_range_fade_mode", "mode"), &PhysXDestructible3D::set_visibility_range_fade_mode);
	ClassDB::bind_method(D_METHOD("get_visibility_range_fade_mode"), &PhysXDestructible3D::get_visibility_range_fade_mode);
	ClassDB::bind_method(D_METHOD("set_shatter_speed", "speed"), &PhysXDestructible3D::set_shatter_speed);
	ClassDB::bind_method(D_METHOD("get_shatter_speed"), &PhysXDestructible3D::get_shatter_speed);
	ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"), &PhysXDestructible3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &PhysXDestructible3D::get_collision_layer);
	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &PhysXDestructible3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &PhysXDestructible3D::get_collision_mask);
	ClassDB::bind_method(D_METHOD("set_mass", "mass"), &PhysXDestructible3D::set_mass);
	ClassDB::bind_method(D_METHOD("set_auto_mass", "auto_mass"), &PhysXDestructible3D::set_auto_mass);
	ClassDB::bind_method(D_METHOD("get_auto_mass"), &PhysXDestructible3D::get_auto_mass);
	ClassDB::bind_method(D_METHOD("get_mass"), &PhysXDestructible3D::get_mass);
	ClassDB::bind_method(D_METHOD("set_dynamic", "dynamic"), &PhysXDestructible3D::set_dynamic);
	ClassDB::bind_method(D_METHOD("get_dynamic"), &PhysXDestructible3D::get_dynamic);
	ClassDB::bind_method(D_METHOD("set_health", "health"), &PhysXDestructible3D::set_health);
	ClassDB::bind_method(D_METHOD("get_health"), &PhysXDestructible3D::get_health);
	ClassDB::bind_method(D_METHOD("set_impact_strength", "strength"), &PhysXDestructible3D::set_impact_strength);
	ClassDB::bind_method(D_METHOD("get_impact_strength"), &PhysXDestructible3D::get_impact_strength);
	ClassDB::bind_method(D_METHOD("set_impact_damage_scale", "scale"), &PhysXDestructible3D::set_impact_damage_scale);
	ClassDB::bind_method(D_METHOD("get_impact_damage_scale"), &PhysXDestructible3D::get_impact_damage_scale);
	ClassDB::bind_method(D_METHOD("set_impact_radius", "radius"), &PhysXDestructible3D::set_impact_radius);
	ClassDB::bind_method(D_METHOD("get_impact_radius"), &PhysXDestructible3D::get_impact_radius);
	ClassDB::bind_method(D_METHOD("set_kill_y", "y"), &PhysXDestructible3D::set_kill_y);
	ClassDB::bind_method(D_METHOD("get_kill_y"), &PhysXDestructible3D::get_kill_y);
	ClassDB::bind_method(D_METHOD("apply_radial_damage", "world_position", "damage", "min_radius", "max_radius"), &PhysXDestructible3D::apply_radial_damage);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "asset_path", PROPERTY_HINT_FILE, "*.asset"), "set_asset_path", "get_asset_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "chunks_path", PROPERTY_HINT_FILE, "*.chunks"), "set_chunks_path", "get_chunks_path");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "blast_asset", PROPERTY_HINT_RESOURCE_TYPE, "PhysXBlastAsset"), "set_blast_asset", "get_blast_asset");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material_override", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_material_override", "get_material_override");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material_overlay", PROPERTY_HINT_RESOURCE_TYPE, "BaseMaterial3D,ShaderMaterial"), "set_material_overlay", "get_material_overlay");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "transparency", PROPERTY_HINT_RANGE, "0.0,1.0,0.01"), "set_transparency", "get_transparency");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "cast_shadow", PROPERTY_HINT_ENUM, "Off,On,Double-Sided,Shadows Only"), "set_cast_shadow", "get_cast_shadow");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "gi_mode", PROPERTY_HINT_ENUM, "Disabled,Static,Dynamic"), "set_gi_mode", "get_gi_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "extra_cull_margin", PROPERTY_HINT_RANGE, "0,16384,0.01,suffix:m"), "set_extra_cull_margin", "get_extra_cull_margin");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lod_bias", PROPERTY_HINT_RANGE, "0.001,128,0.001"), "set_lod_bias", "get_lod_bias");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "ignore_occlusion_culling"), "set_ignore_occlusion_culling", "get_ignore_occlusion_culling");
	ADD_GROUP("Visibility Range", "visibility_range_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "visibility_range_begin", PROPERTY_HINT_RANGE, "0.0,4096.0,0.01,or_greater,suffix:m"), "set_visibility_range_begin", "get_visibility_range_begin");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "visibility_range_begin_margin", PROPERTY_HINT_RANGE, "0.0,4096.0,0.01,or_greater,suffix:m"), "set_visibility_range_begin_margin", "get_visibility_range_begin_margin");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "visibility_range_end", PROPERTY_HINT_RANGE, "0.0,4096.0,0.01,or_greater,suffix:m"), "set_visibility_range_end", "get_visibility_range_end");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "visibility_range_end_margin", PROPERTY_HINT_RANGE, "0.0,4096.0,0.01,or_greater,suffix:m"), "set_visibility_range_end_margin", "get_visibility_range_end_margin");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "visibility_range_fade_mode", PROPERTY_HINT_ENUM, "Disabled,Self,Dependencies"), "set_visibility_range_fade_mode", "get_visibility_range_fade_mode");
	ADD_GROUP("", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "shatter_speed", PROPERTY_HINT_RANGE, "0,50,0.1"), "set_shatter_speed", "get_shatter_speed");
	ADD_GROUP("Physics", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_mass"), "set_auto_mass", "get_auto_mass");
	// Widened well past a typical auto-computed value (density * volume
	// easily lands in the thousands for a wall-sized object) -- or_greater
	// alone doesn't stop the slider/typed-value UI from reading as "capped"
	// once the actual value exceeds the printed max, so give it real
	// headroom instead of relying on that alone.
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "0.001,1000000,0.001,or_greater,exp"), "set_mass", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "dynamic"), "set_dynamic", "get_dynamic");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "health", PROPERTY_HINT_RANGE, "0.01,20,0.01"), "set_health", "get_health");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "impact_strength", PROPERTY_HINT_RANGE, "0,200,0.1"), "set_impact_strength", "get_impact_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "impact_damage_scale", PROPERTY_HINT_RANGE, "0,10,0.01"), "set_impact_damage_scale", "get_impact_damage_scale");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "impact_radius", PROPERTY_HINT_RANGE, "0.1,50,0.1"), "set_impact_radius", "get_impact_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "kill_y", PROPERTY_HINT_RANGE, "-10000,100,1,or_less,or_greater"), "set_kill_y", "get_kill_y");

	BIND_ENUM_CONSTANT(GI_MODE_DISABLED);
	BIND_ENUM_CONSTANT(GI_MODE_STATIC);
	BIND_ENUM_CONSTANT(GI_MODE_DYNAMIC);

	BIND_ENUM_CONSTANT(SHADOW_CASTING_SETTING_OFF);
	BIND_ENUM_CONSTANT(SHADOW_CASTING_SETTING_ON);
	BIND_ENUM_CONSTANT(SHADOW_CASTING_SETTING_DOUBLE_SIDED);
	BIND_ENUM_CONSTANT(SHADOW_CASTING_SETTING_SHADOWS_ONLY);

	BIND_ENUM_CONSTANT(VISIBILITY_RANGE_FADE_DISABLED);
	BIND_ENUM_CONSTANT(VISIBILITY_RANGE_FADE_SELF);
	BIND_ENUM_CONSTANT(VISIBILITY_RANGE_FADE_DEPENDENCIES);
}

PhysXDestructible3D::PhysXDestructible3D() {
	set_notify_transform(true);
}

void PhysXDestructible3D::set_mass(float p_mass) {
	mass = MAX(p_mass, 0.001f);
	if (auto_mass) {
		auto_mass = false;
		notify_property_list_changed();
	}
}

void PhysXDestructible3D::set_auto_mass(bool p_auto) {
	auto_mass = p_auto;
	if (auto_mass) {
		// Recompute right away rather than waiting for the next load/reload
		// -- same live-editing expectation this module's other properties
		// already set (see _reload()'s own note on why).
		_compute_chunk_volumes();
	}
	// The actual bug in the first version of this: without this call, the
	// Inspector never re-checks _validate_property() for `mass` after
	// auto_mass changes, so it stayed visually read-only (or visually
	// editable but silently reverting) regardless of which way you'd just
	// toggled it.
	notify_property_list_changed();
}

void PhysXDestructible3D::_validate_property(PropertyInfo &p_property) const {
	if (auto_mass && p_property.name == "mass") {
		p_property.usage |= PROPERTY_USAGE_READ_ONLY;
	}
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
	_reload();
}

void PhysXDestructible3D::set_chunks_path(const String &p_path) {
	chunks_path = p_path;
	update_configuration_warnings();
	_reload();
}

void PhysXDestructible3D::set_blast_asset(const Ref<PhysXBlastAsset> &p_asset) {
	blast_asset = p_asset;
	update_configuration_warnings();
	_reload();
}

void PhysXDestructible3D::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		if (piece.body.is_valid()) {
			ps->body_set_collision_layer(piece.body, collision_layer);
		}
	}
}

void PhysXDestructible3D::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		if (piece.body.is_valid()) {
			ps->body_set_collision_mask(piece.body, collision_mask);
		}
	}
}

void PhysXDestructible3D::set_material_override(const Ref<Material> &p_material) {
	material_override_res = p_material;
	RenderingServer *rs = RenderingServer::get_singleton();
	const RID mat_rid = material_override_res.is_valid() ? material_override_res->get_rid() : RID();
	for (const ChunkVisual &piece : pieces) {
		rs->instance_geometry_set_material_override(piece.instance, mat_rid);
	}
}

void PhysXDestructible3D::set_gi_mode(GIMode p_mode) {
	gi_mode = p_mode;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		_apply_gi_mode(rs, piece.instance);
	}
}

void PhysXDestructible3D::_apply_gi_mode(RenderingServer *p_rs, RID p_instance) const {
	// Same two flags GeometryInstance3D::set_gi_mode() itself sets (see
	// visual_instance_3d.cpp) -- every piece here is a raw instance_create2()
	// call, not a real GeometryInstance3D node, so nothing was ever setting
	// them without this.
	p_rs->instance_geometry_set_flag(p_instance, RSE::INSTANCE_FLAG_USE_BAKED_LIGHT, gi_mode == GI_MODE_STATIC);
	p_rs->instance_geometry_set_flag(p_instance, RSE::INSTANCE_FLAG_USE_DYNAMIC_GI, gi_mode == GI_MODE_DYNAMIC);
}

void PhysXDestructible3D::set_cast_shadow(ShadowCastingSetting p_setting) {
	cast_shadow = p_setting;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		rs->instance_geometry_set_cast_shadows_setting(piece.instance, (RSE::ShadowCastingSetting)cast_shadow);
	}
}

void PhysXDestructible3D::set_transparency(float p_transparency) {
	transparency = p_transparency;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		rs->instance_geometry_set_transparency(piece.instance, transparency);
	}
}

void PhysXDestructible3D::set_material_overlay(const Ref<Material> &p_material) {
	material_overlay_res = p_material;
	RenderingServer *rs = RenderingServer::get_singleton();
	const RID mat_rid = material_overlay_res.is_valid() ? material_overlay_res->get_rid() : RID();
	for (const ChunkVisual &piece : pieces) {
		rs->instance_geometry_set_material_overlay(piece.instance, mat_rid);
	}
}

void PhysXDestructible3D::set_extra_cull_margin(float p_margin) {
	extra_cull_margin = p_margin;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		rs->instance_set_extra_visibility_margin(piece.instance, extra_cull_margin);
	}
}

void PhysXDestructible3D::set_lod_bias(float p_bias) {
	lod_bias = p_bias;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		rs->instance_geometry_set_lod_bias(piece.instance, lod_bias);
	}
}

void PhysXDestructible3D::set_ignore_occlusion_culling(bool p_ignore) {
	ignore_occlusion_culling = p_ignore;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		rs->instance_geometry_set_flag(piece.instance, RSE::INSTANCE_FLAG_IGNORE_OCCLUSION_CULLING, ignore_occlusion_culling);
	}
}

void PhysXDestructible3D::_apply_visibility_range(RenderingServer *p_rs, RID p_instance) const {
	p_rs->instance_geometry_set_visibility_range(p_instance, visibility_range_begin, visibility_range_end,
			visibility_range_begin_margin, visibility_range_end_margin, (RSE::VisibilityRangeFadeMode)visibility_range_fade_mode);
}

void PhysXDestructible3D::set_visibility_range_begin(float p_distance) {
	visibility_range_begin = p_distance;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		_apply_visibility_range(rs, piece.instance);
	}
}

void PhysXDestructible3D::set_visibility_range_end(float p_distance) {
	visibility_range_end = p_distance;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		_apply_visibility_range(rs, piece.instance);
	}
}

void PhysXDestructible3D::set_visibility_range_begin_margin(float p_distance) {
	visibility_range_begin_margin = p_distance;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		_apply_visibility_range(rs, piece.instance);
	}
}

void PhysXDestructible3D::set_visibility_range_end_margin(float p_distance) {
	visibility_range_end_margin = p_distance;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		_apply_visibility_range(rs, piece.instance);
	}
}

void PhysXDestructible3D::set_visibility_range_fade_mode(VisibilityRangeFadeMode p_mode) {
	visibility_range_fade_mode = p_mode;
	RenderingServer *rs = RenderingServer::get_singleton();
	for (const ChunkVisual &piece : pieces) {
		_apply_visibility_range(rs, piece.instance);
	}
}

void PhysXDestructible3D::_apply_extra_render_settings(RenderingServer *p_rs, RID p_instance) const {
	p_rs->instance_geometry_set_cast_shadows_setting(p_instance, (RSE::ShadowCastingSetting)cast_shadow);
	p_rs->instance_geometry_set_transparency(p_instance, transparency);
	if (material_overlay_res.is_valid()) {
		p_rs->instance_geometry_set_material_overlay(p_instance, material_overlay_res->get_rid());
	}
	p_rs->instance_set_extra_visibility_margin(p_instance, extra_cull_margin);
	p_rs->instance_geometry_set_lod_bias(p_instance, lod_bias);
	p_rs->instance_geometry_set_flag(p_instance, RSE::INSTANCE_FLAG_IGNORE_OCCLUSION_CULLING, ignore_occlusion_culling);
	_apply_visibility_range(p_rs, p_instance);
}

PackedStringArray PhysXDestructible3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();
	if (blast_asset.is_null() && (asset_path.is_empty() || chunks_path.is_empty())) {
		warnings.push_back("PhysXDestructible3D needs either blast_asset, or both asset_path and chunks_path, to render or collide.");
	}
	return warnings;
}

AABB PhysXDestructible3D::get_aabb() const {
	if (chunk_points.is_empty() || chunk_points[0].is_empty()) {
		return AABB();
	}
	const PackedVector3Array &points = chunk_points[0];
	AABB aabb(points[0], Vector3());
	for (int i = 1; i < points.size(); i++) {
		aabb.expand_to(points[i]);
	}
	return aabb;
}

Ref<TriangleMesh> PhysXDestructible3D::generate_triangle_mesh() const {
	if (chunk_points.is_empty() || chunk_points[0].is_empty()) {
		return Ref<TriangleMesh>();
	}
	const PackedVector3Array &points = chunk_points[0];
	Vector<Vector3> faces;
	faces.resize(points.size());
	for (int i = 0; i < points.size(); i++) {
		faces.write[i] = points[i];
	}
	Ref<TriangleMesh> tm;
	tm.instantiate();
	tm->create(faces);
	return tm;
}

void PhysXDestructible3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			// Unlike this module's other (volumetric/particle) editor-invisible
			// nodes, this one replaces a real visible MeshInstance3D -- staying
			// invisible in the editor would be a real authoring regression, not
			// just a missing nicety. So the render mesh always spawns; only the
			// PhysicsServer3D body (no simulation runs in the editor anyway)
			// is skipped there -- see _spawn_intact()/_spawn_piece().
			if (!loaded) {
				loaded = _load();
			}
			if (loaded && !fractured && pieces.is_empty()) {
				_spawn_intact();
				// dynamic: the intact body falls/collides on its own from here,
				// so its visual needs syncing every tick even before anything
				// fractures, and contacts need checking for an auto-fracture-
				// worthy impact. Not in the editor -- _spawn_intact() never
				// creates a physics body there, so there's nothing to sync/check.
				if (dynamic && !Engine::get_singleton()->is_editor_hint()) {
					set_physics_process_internal(true);
				}
			}
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			_free_all_pieces();
		} break;
		case NOTIFICATION_TRANSFORM_CHANGED: {
			// !dynamic || editor: keep the single intact piece's body/visual
			// glued to wherever the node itself moves. Only skipped for a
			// dynamic piece actually simulating at runtime, where physics owns
			// that body's transform from here (falling, colliding) and forcing
			// the node's placement onto it on every notification would fight
			// the simulation -- see NOTIFICATION_INTERNAL_PHYSICS_PROCESS's own
			// node<-body sync for that case. In the editor there is no
			// simulation to fight regardless of `dynamic` (the body is RID()
			// there too, see _spawn_intact()), so moving/rotating the node in
			// the viewport or Inspector must still update the visual -- this
			// used to unconditionally skip on `dynamic` and broke exactly that.
			if (!fractured && pieces.size() == 1 && (!dynamic || Engine::get_singleton()->is_editor_hint())) {
				// This is a real (non-physics-driven) transform push -- e.g. an
				// editor edit before Play -- so it's also a good, cheap moment
				// to keep spawn_scale current for whenever physics does start
				// owning this piece's transform later (see spawn_scale's note).
				spawn_scale = get_global_transform().basis.get_scale();
				if (pieces[0].body.is_valid()) {
					PhysicsServer3D::get_singleton()->body_set_state(pieces[0].body, PhysicsServer3D::BODY_STATE_TRANSFORM, get_global_transform());
				}
				RenderingServer::get_singleton()->instance_set_transform(pieces[0].instance, get_global_transform());
			}
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (fractured) {
				_sync_transforms();
			} else if (dynamic && pieces.size() == 1) {
				// Keep the node's own Transform3D following the falling body too
				// (not just its RenderingServer instance, which _sync_transforms()
				// alone would update) -- apply_radial_damage() converts its
				// world-space position through get_global_transform(), and
				// _check_impact_fracture() calls it using the node's own
				// transform implicitly via that conversion, so a stale node
				// transform (still sitting wherever it was originally placed)
				// would compute a local damage position nowhere near the actual
				// chunk geometry once the body has moved away from it. The
				// NOTIFICATION_TRANSFORM_CHANGED handler above only pushes
				// node->body when !dynamic, so this doesn't loop back on itself.
				PhysicsDirectBodyState3D *state = PhysicsServer3D::get_singleton()->body_get_direct_state(pieces[0].body);
				if (state) {
					Transform3D t = state->get_transform();
					if (t.origin.y < kill_y) {
						// Fell out of the scene before ever taking a hit -- same
						// cleanup _sync_transforms() does for fractured debris,
						// just for the still-intact single piece.
						_free_all_pieces();
					} else {
						// t.basis is a pure rotation here (see spawn_scale's own
						// note) -- re-apply the real scale in the object's own
						// local axes before this goes anywhere, or it's lost.
						t.basis = t.basis.scaled_local(spawn_scale);
						set_global_transform(t);
						RenderingServer::get_singleton()->instance_set_transform(pieces[0].instance, t);
						_check_impact_fracture();
					}
				}
			}
		} break;
	}
}

bool PhysXDestructible3D::_load_asset_bytes(const PackedByteArray &p_bytes) {
	ERR_FAIL_COND_V_MSG(p_bytes.is_empty(), false, "PhysXDestructible3D: empty asset bytes.");
	asset_mem = aligned_alloc_16((size_t)p_bytes.size());
	ERR_FAIL_NULL_V(asset_mem, false);
	memcpy(asset_mem, p_bytes.ptr(), p_bytes.size());

	NvBlastAsset *asset = reinterpret_cast<NvBlastAsset *>(asset_mem);
	asset_chunk_count = NvBlastAssetGetChunkCount(asset, blast_log);
	asset_bond_count = NvBlastAssetGetBondCount(asset, blast_log);

	const size_t family_size = NvBlastAssetGetFamilyMemorySize(asset, blast_log);
	family_mem = aligned_alloc_16(family_size);
	ERR_FAIL_NULL_V(family_mem, false);
	family = NvBlastAssetCreateFamily(family_mem, asset, blast_log);
	ERR_FAIL_NULL_V_MSG(family, false, "PhysXDestructible3D: NvBlastAssetCreateFamily failed.");

	NvBlastActorDesc actor_desc;
	actor_desc.uniformInitialBondHealth = health;
	actor_desc.initialBondHealths = nullptr;
	actor_desc.uniformInitialLowerSupportChunkHealth = health;
	actor_desc.initialSupportChunkHealths = nullptr;

	const size_t scratch_size = NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, blast_log);
	LocalVector<uint8_t> scratch;
	scratch.resize((uint32_t)scratch_size);
	NvBlastActor *first_actor = NvBlastFamilyCreateFirstActor(family, &actor_desc, scratch.ptr(), blast_log);
	ERR_FAIL_NULL_V_MSG(first_actor, false, "PhysXDestructible3D: NvBlastFamilyCreateFirstActor failed.");
	live_actors.push_back(first_actor);
	return true;
}

bool PhysXDestructible3D::_load() {
	if (blast_asset.is_valid()) {
		if (!_load_asset_bytes(blast_asset->get_asset_bytes())) {
			return false;
		}
		const Array points = blast_asset->get_chunk_points();
		chunk_points.resize(points.size());
		for (int i = 0; i < points.size(); i++) {
			chunk_points[i] = points[i];
		}
		_compute_chunk_volumes();
		return true;
	}

	Ref<FileAccess> af = FileAccess::open(asset_path, FileAccess::READ);
	ERR_FAIL_COND_V_MSG(af.is_null(), false, vformat("PhysXDestructible3D: cannot open asset_path '%s'.", asset_path));
	if (!_load_asset_bytes(af->get_buffer(af->get_length()))) {
		return false;
	}

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

	_compute_chunk_volumes();
	return true;
}

void PhysXDestructible3D::_compute_chunk_volumes() {
	// Divergence-theorem volume of a closed triangle soup: sum each
	// triangle's signed tetrahedron volume against the origin
	// (a . (b x c) / 6); abs() makes it winding-independent, and object-
	// local space (chunk geometry is already authored there) means no
	// per-chunk transform is needed -- every chunk shares the node's own
	// scale equally, so it's fine (in fact necessary, see below) for
	// chunk_volumes[] itself to stay unscaled: _chunk_mass() only ever uses
	// it as a ratio against total_leaf_volume, and a shared scale factor
	// cancels out of that ratio exactly, uniform or not. Used to distribute
	// mass across pieces proportional to their actual size (see set_mass())
	// instead of every piece getting the same flat default regardless of
	// how big it is -- found missing here by checking how Unreal's own
	// Blast integration handles it (BlastMeshComponent.cpp: IdealChunkMass =
	// RootChunkMass * ThisChunkVolume / TotalVolume).
	chunk_volumes.resize(chunk_points.size());
	total_leaf_volume = 0.0;
	for (uint32_t c = 0; c < chunk_points.size(); c++) {
		const PackedVector3Array &points = chunk_points[c];
		double v6 = 0.0;
		for (int t = 0; t + 2 < points.size(); t += 3) {
			v6 += (double)points[t].dot(points[t + 1].cross(points[t + 2]));
		}
		const double volume = Math::abs(v6) / 6.0;
		chunk_volumes[c] = volume;
		if (c != 0) { // chunk 0 is the whole unfractured mesh, not a leaf
			total_leaf_volume += volume;
		}
	}

	// While auto_mass is true, recompute mass from the intact mesh's real
	// volume (a fixed internal density -- not its own exposed property, see
	// set_mass()'s own note on why) every time this runs -- a load, a
	// reload, or auto_mass being checked again after an override. Once
	// auto_mass is false, this is a no-op and whatever mass was explicitly
	// set stays exactly that.
	//
	// Unlike the per-chunk ratio above, this is an *absolute* value, so the
	// node's actual scale matters here and has to be applied explicitly --
	// chunk_volumes[0] alone is the unscaled local mesh volume, and a
	// heavily scaled destructible (e.g. 20x10x1, a real reported case) would
	// otherwise get a default mass computed as if it were still original
	// size, off by the product of the scale factors (200x too light there).
	if (auto_mass && chunk_volumes.size() > 0) {
		const double default_density = 2200.0; // roughly concrete/stone
		const Vector3 scale = get_global_transform().basis.get_scale();
		const double scaled_volume = chunk_volumes[0] * (double)scale.x * (double)scale.y * (double)scale.z;
		mass = MAX((float)(default_density * scaled_volume), 0.001f);
	}
}

float PhysXDestructible3D::_chunk_mass(uint32_t p_chunk_index) const {
	if (p_chunk_index == 0 || p_chunk_index >= chunk_volumes.size() || total_leaf_volume <= 0.0) {
		return mass;
	}
	// Minimum floor so a sliver chunk doesn't end up with a near-zero mass
	// that misbehaves in the solver -- same reasoning as Unreal's own 0.5kg
	// floor (FMath::Max(IdealChunkMass, 0.5f)), scaled down since this
	// module's demo assets are much smaller than Unreal's typical world
	// scale (see this class's own shatter_speed/health defaults).
	const float ideal = (float)(mass * chunk_volumes[p_chunk_index] / total_leaf_volume);
	return MAX(ideal, 0.05f);
}

void PhysXDestructible3D::_reload() {
	_free_all_pieces();
	if (family_mem) {
		aligned_free_16(family_mem);
		family_mem = nullptr;
	}
	if (asset_mem) {
		aligned_free_16(asset_mem);
		asset_mem = nullptr;
	}
	family = nullptr;
	live_actors.clear();
	asset_chunk_count = 0;
	asset_bond_count = 0;
	chunk_points.clear();
	chunk_volumes.clear();
	total_leaf_volume = 0.0;
	loaded = false;
	fractured = false;

	if (!is_inside_world()) {
		// Not in the tree/world yet -- NOTIFICATION_ENTER_WORLD will load and
		// spawn once it is, same as before this method existed.
		return;
	}

	loaded = _load();
	if (loaded && pieces.is_empty()) {
		_spawn_intact();
		if (dynamic && !Engine::get_singleton()->is_editor_hint()) {
			set_physics_process_internal(true);
		}
	}
}

void PhysXDestructible3D::_spawn_intact() {
	// Chunk 0 is always the fracture-tool's root chunk -- the whole
	// unfractured asset mesh (see blast_test_gen.cpp) -- so rendering/
	// colliding as chunk 0 while nothing has broken is exactly correct, not
	// an approximation. No physics body in the editor -- nothing simulates
	// there, and the render-only piece is enough for authoring/placement.
	const bool physics = !Engine::get_singleton()->is_editor_hint();
	// See spawn_scale's own note -- this is the last point before physics
	// could ever own this piece's transform, so it's the last clean read of
	// the node's real intended scale.
	spawn_scale = get_global_transform().basis.get_scale();
	_spawn_piece(0, get_global_transform(), Vector3(), physics);
	if (physics) {
		PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
		if (dynamic) {
			// A real falling/colliding body, not a fixed prop -- and contact
			// reporting so _check_impact_fracture() can see what it hit.
			ps->body_set_mode(pieces[0].body, PhysicsServer3D::BODY_MODE_RIGID);
			ps->body_set_max_contacts_reported(pieces[0].body, 8);
		} else {
			ps->body_set_mode(pieces[0].body, PhysicsServer3D::BODY_MODE_STATIC);
		}
	}
}

void PhysXDestructible3D::_spawn_piece(uint32_t p_chunk_index, const Transform3D &p_transform, const Vector3 &p_linear_velocity, bool p_physics) {
	ERR_FAIL_INDEX(p_chunk_index, chunk_points.size());
	const PackedVector3Array &points = chunk_points[p_chunk_index];
	const uint32_t tri_count = points.size() / 3;
	ERR_FAIL_COND_MSG(points.size() < 4, vformat("PhysXDestructible3D: chunk %d has too few vertices for a convex hull.", p_chunk_index));

	RID shape;
	RID body;
	if (p_physics) {
		PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
		shape = ps->convex_polygon_shape_create();
		ps->shape_set_data(shape, points);

		body = ps->body_create();
		// A real CollisionObject3D (RigidBody3D, StaticBody3D, ...) always
		// does this in its constructor -- it's how the *other* body's
		// RigidBody3D::_body_inout() resolves a Node to emit body_entered/
		// body_shape_entered against (see rigid_body_3d.cpp; without it,
		// ObjectDB::get_instance() on the raw body's RID returns null, the
		// node stays "not in tree", and those signals silently never fire,
		// even though the physics-server-level collision response -- the
		// literal bounce -- still happens). Missing this made every
		// PhysXDestructible3D body invisible to any script relying on
		// body_entered (e.g. destructible_demo_rig.gd's bomb): it would
		// bounce off every time and never register a hit.
		ps->body_attach_object_instance_id(body, get_instance_id());
		ps->body_set_mode(body, PhysicsServer3D::BODY_MODE_RIGID);
		ps->body_add_shape(body, shape);
		ps->body_set_collision_layer(body, collision_layer);
		ps->body_set_collision_mask(body, collision_mask);
		ps->body_set_param(body, PhysicsServer3D::BODY_PARAM_MASS, _chunk_mass(p_chunk_index));
		ps->body_set_state(body, PhysicsServer3D::BODY_STATE_TRANSFORM, p_transform);
		ps->body_set_state(body, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, p_linear_velocity);
		if (is_inside_world() && get_world_3d().is_valid()) {
			ps->body_set_space(body, get_world_3d()->get_space());
		}
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
	_apply_gi_mode(rs, instance);
	_apply_extra_render_settings(rs, instance);

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
		// body/shape are RID() for a render-only editor piece (see
		// _spawn_piece's p_physics) -- freeing an invalid RID is harmless on
		// most servers, but skip it explicitly rather than rely on that.
		if (piece.body.is_valid()) {
			ps->free_rid(piece.body);
		}
		if (piece.shape.is_valid()) {
			ps->free_rid(piece.shape);
		}
		rs->free_rid(piece.instance);
		rs->free_rid(piece.mesh);
	}
	pieces.clear();
}

void PhysXDestructible3D::_sync_transforms() {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	RenderingServer *rs = RenderingServer::get_singleton();
	// Backwards so remove_at_unordered() (swaps in the last element) doesn't
	// skip the piece it just moved into the current slot.
	for (int64_t i = (int64_t)pieces.size() - 1; i >= 0; i--) {
		const ChunkVisual &piece = pieces[i];
		PhysicsDirectBodyState3D *state = ps->body_get_direct_state(piece.body);
		if (!state) {
			continue;
		}
		Transform3D t = state->get_transform();
		if (t.origin.y < kill_y) {
			// Nothing else was ever going to free a piece that's just fallen
			// out of the scene entirely (there's no owning Node a kill-floor
			// Area3D could catch it with -- these are raw PhysicsServer3D
			// RIDs) -- without this, debris that misses the level keeps
			// simulating and consuming memory forever.
			ps->free_rid(piece.body);
			ps->free_rid(piece.shape);
			rs->free_rid(piece.instance);
			rs->free_rid(piece.mesh);
			pieces.remove_at_unordered((uint32_t)i);
			continue;
		}
		// t.basis is a pure rotation (see spawn_scale's own note) -- every
		// piece was authored from the same intact mesh at the same original
		// scale, so the same spawn_scale applies to all of them here too.
		t.basis = t.basis.scaled_local(spawn_scale);
		rs->instance_set_transform(piece.instance, t);
	}
}

void PhysXDestructible3D::_check_impact_fracture() {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	PhysicsDirectBodyState3D *state = ps->body_get_direct_state(pieces[0].body);
	if (!state) {
		return;
	}

	// The hardest single contact this tick, not the sum of all of them --
	// resting on a flat floor with several contact points shouldn't add up
	// to "impact" just because there are multiple points of an otherwise
	// gentle landing.
	real_t max_impulse = 0.0;
	Vector3 impact_position = get_global_transform().origin;
	const int contact_count = state->get_contact_count();
	for (int i = 0; i < contact_count; i++) {
		const real_t impulse_len = state->get_contact_impulse(i).length();
		if (impulse_len > max_impulse) {
			max_impulse = impulse_len;
			impact_position = state->get_contact_local_position(i);
		}
	}

	if (max_impulse <= (real_t)impact_strength) {
		return;
	}

	// apply_radial_damage() no longer commits to "fractured" until it
	// confirms real chunks actually broke off, so a too-weak hit now safely
	// no-ops instead of vanishing the object -- but a damage value too close
	// to health can still leave nothing behind on this particular impact
	// (known sharp edge at exactly damage == health, see
	// PhysXBlastAuthoring's own notes), which would silently swallow an
	// impact that visibly looked hard enough to break something. Once an
	// impact clears impact_strength at all, guarantee real overkill rather
	// than risk that; impact_damage_scale still lets a much harder impact
	// scale damage further above this floor.
	const float damage = MAX((float)(max_impulse - impact_strength) * impact_damage_scale, health * 1.5f);
	apply_radial_damage(impact_position, damage, 0.0f, impact_radius);
}

int PhysXDestructible3D::apply_radial_damage(const Vector3 &p_world_position, float p_damage, float p_min_radius, float p_max_radius) {
	ERR_FAIL_NULL_V_MSG(family, 0, "PhysXDestructible3D: node not loaded (missing/invalid asset_path or chunks_path?).");

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

	// Only now -- having confirmed real chunks actually broke off -- retire
	// the intact placeholder body/visual and commit to "fractured". Doing
	// this unconditionally on the *first call* (the old behavior) meant any
	// hit that reached the object at all, even one whose falloff-scaled
	// damage was too weak to break a single bond, silently destroyed the
	// intact piece and spawned nothing: the object would vanish with no
	// body and no visual, yet `fractured` was already true so it could never
	// be re-spawned, only (maybe) surfaced later once accumulated bond
	// damage from a subsequent hit finally cleared a split -- exactly the
	// "bounces off, then randomly disappears or breaks" behavior reported
	// against a single static sphere. `pieces[0]` is guaranteed to still be
	// exactly that placeholder here: nothing else could have been in
	// `pieces` before this call while `!fractured`.
	if (!fractured && spawned > 0) {
		PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
		RenderingServer *rs = RenderingServer::get_singleton();
		const ChunkVisual &placeholder = pieces[0];
		if (placeholder.body.is_valid()) {
			ps->free_rid(placeholder.body);
		}
		if (placeholder.shape.is_valid()) {
			ps->free_rid(placeholder.shape);
		}
		rs->free_rid(placeholder.instance);
		rs->free_rid(placeholder.mesh);
		pieces.remove_at_unordered(0);
		fractured = true;
		set_physics_process_internal(true);
	}

	return spawned;
}

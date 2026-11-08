/**************************************************************************/
/*  physx_groom_3d.cpp                                                    */
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

#include "physx_groom_3d.h"

#include "../cloth/cloth_wind.h"
#include "groom_shading.h"
#include "physx_groom_render_effect.h"

#include "core/config/engine.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "core/config/project_settings.h"
#include "scene/3d/camera_3d.h"
#include "core/math/geometry_3d.h"
#include "scene/3d/physics/area_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/3d/physics/physical_bone_3d.h"
#include "scene/3d/physics/physical_bone_simulator_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/resources/3d/capsule_shape_3d.h"
#include "scene/resources/3d/sphere_shape_3d.h"
#include "scene/resources/texture_rd.h"
#include "scene/resources/3d/world_3d.h"
#include "scene/main/viewport.h"
#include "servers/rendering/rendering_server.h"

RID PhysXGroom3D::shared_shader;

// Each point is two vertices at the same place: UV.x says which side of the strand (-1, 1), UV.y how far along it
// (0 root, 1 tip), UV2.x the radius there, UV2.y the strand's place in the groom's (random) order, from 0 to 1,
// NORMAL the strand's direction. The vertex shader spreads them apart across the strand, facing the camera, at least a
// pixel apart; a strand thinner than that draws less of its pixel (alpha hash). Strands past strand_fraction aren't
// drawn (collapsed to a point), and the rest widen to cover for them.
// Simulated (GroomSim), the points come from its texture instead, in world space: a point is two vertices, so the
// vertex's point is VERTEX_ID / 2, and its direction runs from the point before to the one after.
// The strand mesh's own part of its shader (see groom_shading.h for the rest).
static const char *GROOM_MESH_SHADER = R"(
uniform float width_scale = 1.0;
uniform float strand_fraction = 1.0;
uniform bool simulated = false;
uniform sampler2D sim_points : filter_nearest, repeat_disable;
uniform int sim_width = 1;

vec3 sim_point(int i) {
	return texelFetch(sim_points, ivec2(i % sim_width, i / sim_width), 0).xyz;
}

varying vec3 v_tangent;
varying float v_along;
varying float v_coverage;
varying vec4 v_shadow;

void vertex() {
	vec3 pos;
	vec3 tangent;
	if (simulated) {
		int i = VERTEX_ID / 2;
		vec3 here = sim_point(i);
		vec3 ahead = UV.y < 0.9999 ? sim_point(i + 1) : here;
		vec3 behind = UV.y > 0.0001 ? sim_point(i - 1) : here;
		pos = (VIEW_MATRIX * vec4(here, 1.0)).xyz;
		vec3 along = mat3(VIEW_MATRIX) * (ahead - behind);
		tangent = dot(along, along) > 1e-20 ? normalize(along) : normalize((MODELVIEW_MATRIX * vec4(NORMAL, 0.0)).xyz);
	} else {
		pos = (MODELVIEW_MATRIX * vec4(VERTEX, 1.0)).xyz;
		tangent = normalize((MODELVIEW_MATRIX * vec4(NORMAL, 0.0)).xyz);
	}
	vec3 across = cross(tangent, pos);
	across = dot(across, across) > 1e-12 ? normalize(across) : normalize(cross(tangent, vec3(0.0, 1.0, 0.0)));
	// The strands left out at a distance widen to cover for them on screen; in the shadow map they'd only blot.
	float radius = UV2.x * width_scale / (IN_SHADOW_PASS ? 1.0 : strand_fraction) * length(MODELVIEW_MATRIX[0].xyz);
	// A pixel's size here: it grows with distance in a perspective view, not in an orthographic one (a directional
	// light's shadow map, where taking it as perspective made every strand hundreds of pixels wide).
	bool orthographic = PROJECTION_MATRIX[3][3] > 0.5;
	float pixel = (orthographic ? 2.0 : -pos.z * 2.0) / (abs(PROJECTION_MATRIX[1][1]) * VIEWPORT_SIZE.y);
	float half_width = max(radius, pixel * 0.5);
	v_coverage = clamp(radius / half_width, 0.0, 1.0);
	VERTEX = pos + across * UV.x * half_width;
	vec3 facing = normalize(cross(across, tangent));
	NORMAL = dot(facing, -pos) < 0.0 ? -facing : facing;
	TANGENT = tangent;
	BINORMAL = across;
	v_tangent = tangent;
	v_along = UV.y;
	// Baked in the groom's space; lit in view space.
	mat3 turn = mat3(normalize(MODELVIEW_MATRIX[0].xyz), normalize(MODELVIEW_MATRIX[1].xyz), normalize(MODELVIEW_MATRIX[2].xyz));
	v_shadow = turn_strands(CUSTOM0, turn);
	// Past the strands drawn at this distance: every vertex at one point, nothing to rasterize.
	if (UV2.y >= strand_fraction) {
		VERTEX = vec3(0.0);
	}
}

void fragment() {
	ALBEDO = strand_albedo(v_along);
	if (shading_model == 1) {
		ROUGHNESS = hair_roughness;
		// Its own lighting carries the strand's highlights.
		SPECULAR = 0.0;
	} else {
		ROUGHNESS = 0.6;
	}
	ALPHA = v_coverage;
	ALPHA_HASH_SCALE = 1.0;
	light_tangent = normalize(v_tangent);
	light_shadow = v_shadow;
	light_roughness = hair_roughness;
	light_model = float(shading_model);
	// Ambient light, from every direction: the strands in the way on average.
	AO = exp(-self_shadow * max(0.0, v_shadow.x * 0.282095));
}
)";

RID PhysXGroom3D::_shader() {
	if (!shared_shader.is_valid()) {
		RenderingServer *rs = RenderingServer::get_singleton();
		shared_shader = rs->shader_create();
		rs->shader_set_code(shared_shader, String("shader_type spatial;\nrender_mode skip_vertex_transform, cull_disabled;\n") + GROOM_SHADING_COMMON + GROOM_MESH_SHADER + GROOM_SHADING_LIGHT);
	}
	return shared_shader;
}

void PhysXGroom3D::free_shared() {
	GroomSim::free_shared();
	if (shared_shader.is_valid()) {
		RenderingServer::get_singleton()->free_rid(shared_shader);
		shared_shader = RID();
	}
}

void PhysXGroom3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_groom", "groom"), &PhysXGroom3D::set_groom);
	ClassDB::bind_method(D_METHOD("get_groom"), &PhysXGroom3D::get_groom);
	ClassDB::bind_method(D_METHOD("set_root_color", "color"), &PhysXGroom3D::set_root_color);
	ClassDB::bind_method(D_METHOD("get_root_color"), &PhysXGroom3D::get_root_color);
	ClassDB::bind_method(D_METHOD("set_tip_color", "color"), &PhysXGroom3D::set_tip_color);
	ClassDB::bind_method(D_METHOD("get_tip_color"), &PhysXGroom3D::get_tip_color);
	ClassDB::bind_method(D_METHOD("set_width_scale", "scale"), &PhysXGroom3D::set_width_scale);
	ClassDB::bind_method(D_METHOD("get_width_scale"), &PhysXGroom3D::get_width_scale);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "groom", PROPERTY_HINT_RESOURCE_TYPE, "PhysXGroom"), "set_groom", "get_groom");
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "root_color", PROPERTY_HINT_COLOR_NO_ALPHA), "set_root_color", "get_root_color");
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "tip_color", PROPERTY_HINT_COLOR_NO_ALPHA), "set_tip_color", "get_tip_color");
	ClassDB::bind_method(D_METHOD("set_render_mode", "mode"), &PhysXGroom3D::set_render_mode);
	ClassDB::bind_method(D_METHOD("get_render_mode"), &PhysXGroom3D::get_render_mode);
	ClassDB::bind_method(D_METHOD("set_shading_model", "model"), &PhysXGroom3D::set_shading_model);
	ClassDB::bind_method(D_METHOD("get_shading_model"), &PhysXGroom3D::get_shading_model);
	ClassDB::bind_method(D_METHOD("set_roughness", "roughness"), &PhysXGroom3D::set_roughness);
	ClassDB::bind_method(D_METHOD("get_roughness"), &PhysXGroom3D::get_roughness);
	ClassDB::bind_method(D_METHOD("set_self_shadow", "amount"), &PhysXGroom3D::set_self_shadow);
	ClassDB::bind_method(D_METHOD("get_self_shadow"), &PhysXGroom3D::get_self_shadow);
	ClassDB::bind_method(D_METHOD("set_max_strands", "count"), &PhysXGroom3D::set_max_strands);
	ClassDB::bind_method(D_METHOD("get_max_strands"), &PhysXGroom3D::get_max_strands);
	ClassDB::bind_method(D_METHOD("get_drawn_strand_count"), &PhysXGroom3D::get_drawn_strand_count);
	ClassDB::bind_method(D_METHOD("set_simulate", "enabled"), &PhysXGroom3D::set_simulate);
	ClassDB::bind_method(D_METHOD("is_simulating"), &PhysXGroom3D::is_simulating);
	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &PhysXGroom3D::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &PhysXGroom3D::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_bend_stiffness", "stiffness"), &PhysXGroom3D::set_bend_stiffness);
	ClassDB::bind_method(D_METHOD("get_bend_stiffness"), &PhysXGroom3D::get_bend_stiffness);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &PhysXGroom3D::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &PhysXGroom3D::get_damping);
	ClassDB::bind_method(D_METHOD("set_gravity_scale", "scale"), &PhysXGroom3D::set_gravity_scale);
	ClassDB::bind_method(D_METHOD("get_gravity_scale"), &PhysXGroom3D::get_gravity_scale);
	ClassDB::bind_method(D_METHOD("set_wind", "wind"), &PhysXGroom3D::set_wind);
	ClassDB::bind_method(D_METHOD("get_wind"), &PhysXGroom3D::get_wind);
	ClassDB::bind_method(D_METHOD("set_wind_area", "path"), &PhysXGroom3D::set_wind_area);
	ClassDB::bind_method(D_METHOD("get_wind_area"), &PhysXGroom3D::get_wind_area);
	ClassDB::bind_method(D_METHOD("set_collide_with_head", "enabled"), &PhysXGroom3D::set_collide_with_head);
	ClassDB::bind_method(D_METHOD("is_colliding_with_head"), &PhysXGroom3D::is_colliding_with_head);
	ClassDB::bind_method(D_METHOD("set_colliders", "colliders"), &PhysXGroom3D::set_colliders);
	ClassDB::bind_method(D_METHOD("get_colliders"), &PhysXGroom3D::get_colliders);
	ClassDB::bind_method(D_METHOD("reset_simulation"), &PhysXGroom3D::reset_simulation);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "width_scale", PROPERTY_HINT_RANGE, "0.1,10,0.01"), "set_width_scale", "get_width_scale");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "render_mode", PROPERTY_HINT_ENUM, "Mesh,Screen Space"), "set_render_mode", "get_render_mode");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "shading_model", PROPERTY_HINT_ENUM, "Kajiya-Kay,Marschner"), "set_shading_model", "get_shading_model");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "roughness", PROPERTY_HINT_RANGE, "0.1,1,0.01"), "set_roughness", "get_roughness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "self_shadow", PROPERTY_HINT_RANGE, "0,4,0.01"), "set_self_shadow", "get_self_shadow");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_strands", PROPERTY_HINT_RANGE, "0,1000000,1,or_greater"), "set_max_strands", "get_max_strands");

	ADD_GROUP("Simulation", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "simulate"), "set_simulate", "is_simulating");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bend_stiffness", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_bend_stiffness", "get_bend_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_damping", "get_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity_scale", PROPERTY_HINT_RANGE, "-2,4,0.01"), "set_gravity_scale", "get_gravity_scale");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "wind", PROPERTY_HINT_NONE, "suffix:m/s"), "set_wind", "get_wind");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "wind_area", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Area3D"), "set_wind_area", "get_wind_area");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_with_head"), "set_collide_with_head", "is_colliding_with_head");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "colliders", PROPERTY_HINT_TYPE_STRING, vformat("%s/%s:%s", Variant::NODE_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "CollisionShape3D,CollisionObject3D,Skeleton3D")), "set_colliders", "get_colliders");

	BIND_ENUM_CONSTANT(RENDER_MODE_MESH);
	BIND_ENUM_CONSTANT(RENDER_MODE_SCREEN_SPACE);
	BIND_ENUM_CONSTANT(SHADING_MODEL_KAJIYA_KAY);
	BIND_ENUM_CONSTANT(SHADING_MODEL_MARSCHNER);
}

void PhysXGroom3D::set_groom(const Ref<PhysXGroom> &p_groom) {
	if (groom.is_valid()) {
		groom->disconnect_changed(callable_mp(this, &PhysXGroom3D::_rebuild));
	}
	groom = p_groom;
	if (groom.is_valid()) {
		groom->connect_changed(callable_mp(this, &PhysXGroom3D::_rebuild));
	}
	_rebuild();
}

void PhysXGroom3D::set_root_color(const Color &p_color) {
	root_color = p_color;
	_update_material();
}

void PhysXGroom3D::set_tip_color(const Color &p_color) {
	tip_color = p_color;
	_update_material();
}

void PhysXGroom3D::set_width_scale(float p_scale) {
	width_scale = MAX(p_scale, 0.0f);
	_update_material();
	_rebuild();
}

void PhysXGroom3D::set_render_mode(RenderMode p_mode) {
	if (render_mode == p_mode) {
		return;
	}
	render_mode = p_mode;
	if (render_mode == RENDER_MODE_SCREEN_SPACE) {
		_send_points();
	}
	_update_screen_space();
	// Back to drawing itself: the shadow setting it had.
	if (render_mode == RENDER_MODE_MESH) {
		set_cast_shadows_setting(get_cast_shadows_setting());
	}
}

// The strands as the screen-space passes read them: see PhysXGroomRenderEffect::set_points.
void PhysXGroom3D::_send_points() {
	if (render_mode != RENDER_MODE_SCREEN_SPACE) {
		return;
	}
	PackedFloat32Array packed;
	if (groom.is_valid() && groom->is_valid()) {
		const PackedVector3Array points = groom->get_points();
		const PackedInt32Array offsets = groom->get_strand_offsets();
		const PackedFloat32Array widths = groom->get_widths();
		const PackedFloat32Array shadow = groom->get_self_shadow();
		const bool has_shadow = shadow.size() == points.size() * 4;
		const int strand_count = groom->get_strand_count();
		packed.resize(points.size() * 12);
		float *w = packed.ptrw();
		for (int s = 0; s < strand_count; s++) {
			const int from = offsets[s];
			const int count = offsets[s + 1] - from;
			const float rank = ((float)s + 0.5f) / (float)strand_count;
			for (int j = 0; j < count; j++) {
				const int p = from + j;
				float *o = w + p * 12;
				o[0] = points[p].x;
				o[1] = points[p].y;
				o[2] = points[p].z;
				o[3] = widths[p] * 0.5f;
				o[4] = count > 1 ? (float)j / (float)(count - 1) : 0.0f;
				o[5] = rank;
				o[6] = j == count - 1 ? 1.0f : 0.0f;
				o[7] = 0.0f;
				for (int i = 0; i < 4; i++) {
					o[8 + i] = has_shadow ? shadow[p * 4 + i] : 0.0f;
				}
			}
		}
	}
	PhysXGroomRenderEffect::set_points(get_instance_id(), ++points_version, packed);
}

// Joins or leaves the screen-space effect's world, and in it hands the effect where the groom is each frame. The mesh
// still casts the shadows: it's drawn for nothing else.
void PhysXGroom3D::_update_screen_space() {
	World3D *world = is_inside_tree() && get_world_3d().is_valid() ? get_world_3d().ptr() : nullptr;
	const bool wanted = render_mode == RENDER_MODE_SCREEN_SPACE && world != nullptr && is_visible_in_tree();
	const ObjectID world_id = wanted ? world->get_instance_id() : ObjectID();
	if (screen_space_world != world_id) {
		if (screen_space_world.is_valid()) {
			PhysXGroomRenderEffect::detach_world(screen_space_world);
		}
		screen_space_world = world_id;
		if (wanted) {
			PhysXGroomRenderEffect::attach_world(world);
		}
	}
	if (!wanted) {
		PhysXGroomRenderEffect::remove_groom(get_instance_id());
		sent_to_effect = false;
		return;
	}
	// Gone from the effect while out of a world (or not yet sent): the strands again.
	if (!sent_to_effect) {
		sent_to_effect = true;
		_send_points();
	}
	PhysXGroomRenderEffect::Look look;
	look.root_color = root_color;
	look.tip_color = tip_color;
	look.self_shadow = self_shadow;
	look.roughness = roughness;
	look.shading_model = shading_model;
	PhysXGroomRenderEffect::set_state(get_instance_id(), get_global_transform(), strand_fraction, width_scale, look, sim_running ? sim->get_texture() : RID(), sim_running ? sim->get_width() : 0);
	Viewport *viewport = get_viewport();
	PhysXGroomRenderEffect::install(world, viewport ? viewport->get_camera_3d() : nullptr);
	RenderingServer::get_singleton()->instance_geometry_set_cast_shadows_setting(get_instance(), get_cast_shadows_setting() == SHADOW_CASTING_SETTING_OFF ? RSE::SHADOW_CASTING_SETTING_OFF : RSE::SHADOW_CASTING_SETTING_SHADOWS_ONLY);
}

void PhysXGroom3D::set_shading_model(ShadingModel p_model) {
	shading_model = p_model;
	_update_material();
	notify_property_list_changed();
}

void PhysXGroom3D::set_roughness(float p_roughness) {
	roughness = CLAMP(p_roughness, 0.1f, 1.0f);
	_update_material();
}

void PhysXGroom3D::set_self_shadow(float p_amount) {
	self_shadow = MAX(p_amount, 0.0f);
	_update_material();
}

void PhysXGroom3D::set_max_strands(int p_count) {
	max_strands = MAX(p_count, 0);
	_update_lod();
}

void PhysXGroom3D::set_simulate(bool p_enabled) {
	simulate = p_enabled;
}

void PhysXGroom3D::set_stiffness(float p_stiffness) {
	stiffness = CLAMP(p_stiffness, 0.0f, 1.0f);
}

void PhysXGroom3D::set_bend_stiffness(float p_stiffness) {
	bend_stiffness = CLAMP(p_stiffness, 0.0f, 1.0f);
}

void PhysXGroom3D::set_damping(float p_damping) {
	damping = CLAMP(p_damping, 0.0f, 1.0f);
}

void PhysXGroom3D::set_gravity_scale(float p_scale) {
	gravity_scale = p_scale;
}

void PhysXGroom3D::set_wind(const Vector3 &p_wind) {
	wind = p_wind;
}

void PhysXGroom3D::set_wind_area(const NodePath &p_path) {
	wind_area = p_path;
}

void PhysXGroom3D::set_collide_with_head(bool p_enabled) {
	collide_with_head = p_enabled;
}

void PhysXGroom3D::set_colliders(const TypedArray<NodePath> &p_colliders) {
	colliders = p_colliders;
}

void PhysXGroom3D::reset_simulation() {
	if (sim_running) {
		sim->reset();
		collider_ends.clear();
		sim_last = get_global_transform();
	}
}

bool PhysXGroom3D::_wants_simulation() const {
	return simulate && !Engine::get_singleton()->is_editor_hint() && groom.is_valid() && groom->is_valid() && groom->get_strand_count() > 0 && is_inside_tree() && is_visible_in_tree();
}

void PhysXGroom3D::_start_simulation() {
	sim->build(groom);
	if (!sim->is_built()) {
		WARN_PRINT_ONCE("PhysXGroom3D: the hair simulation needs a RenderingDevice (Forward+ or Mobile); the strands stay as groomed.");
		simulate = false;
		return;
	}
	sim_texture->set_texture_rd_rid(sim->get_texture());
	RenderingServer *rs = RenderingServer::get_singleton();
	rs->material_set_param(material, "sim_points", sim_texture->get_rid());
	rs->material_set_param(material, "sim_width", sim->get_width());
	rs->material_set_param(material, "simulated", true);
	// Room for the strands to swing out to their full length.
	rs->mesh_set_custom_aabb(mesh, aabb.grow(longest_strand));
	sim_last = get_global_transform();
	sim->reset();
	collider_ends.clear();
	sim_running = true;
}

void PhysXGroom3D::_stop_simulation() {
	if (!sim_running) {
		return;
	}
	sim_running = false;
	RenderingServer *rs = RenderingServer::get_singleton();
	rs->material_set_param(material, "simulated", false);
	rs->material_set_param(material, "sim_points", Variant());
	rs->mesh_set_custom_aabb(mesh, aabb);
	sim_texture->set_texture_rd_rid(RID());
	sim->clear();
}

void PhysXGroom3D::_update_simulation(double p_delta) {
	if (!_wants_simulation()) {
		_stop_simulation();
		return;
	}
	if (!sim_running) {
		_start_simulation();
		if (!sim_running) {
			return;
		}
	}
	const Transform3D xform = get_global_transform();
	// Moved further than the hair could swing in a frame: a teleport, not motion.
	if (xform.origin.distance_to(sim_last.origin) > MAX(longest_strand, 0.1f) * 4.0f) {
		sim->reset();
		collider_ends.clear();
		sim_last = xform;
	}
	GroomSim::Settings settings;
	settings.stiffness = stiffness;
	settings.bend_stiffness = bend_stiffness;
	settings.damping = damping;
	const Vector3 gravity_direction = GLOBAL_GET("physics/3d/default_gravity_vector");
	const float gravity = GLOBAL_GET("physics/3d/default_gravity");
	settings.gravity = gravity_direction * gravity * gravity_scale;
	const Area3D *area = Object::cast_to<Area3D>(get_node_or_null(wind_area));
	settings.wind = physx_cloth_sample_wind(wind, area, 0.0f, 0.0, xform.origin);
	LocalVector<GroomSim::Collider> shapes;
	_gather_colliders(xform, shapes);
	sim->step(sim_last, xform, p_delta, settings, shapes);
	sim_last = xform;
}

// A capsule or sphere CollisionShape3D's ends and radius where it is now. One on a physical bone is placed where its
// bone is, as the PhysicalBone3D follows it: the simulator moves the PhysicalBone3D only while it's active.
static bool shape_capsule(const CollisionShape3D *p_shape, Vector3 &r_a, Vector3 &r_b, float &r_radius) {
	if (p_shape->is_disabled() || !p_shape->is_inside_tree() || p_shape->get_shape().is_null()) {
		return false;
	}
	Transform3D x = p_shape->get_global_transform();
	PhysicalBone3D *bone = Object::cast_to<PhysicalBone3D>(p_shape->get_parent());
	const Skeleton3D *skeleton = bone ? bone->get_skeleton() : nullptr;
	if (skeleton) {
		const int id = bone->get_bone_id() >= 0 ? bone->get_bone_id() : skeleton->find_bone(bone->get_bone_name());
		if (id >= 0) {
			x = skeleton->get_global_transform() * skeleton->get_bone_global_pose(id) * bone->get_body_offset() * p_shape->get_transform();
		}
	}
	const Ref<Shape3D> shape = p_shape->get_shape();
	const float sx = x.basis.get_column(0).length();
	const float sy = x.basis.get_column(1).length();
	const float sz = x.basis.get_column(2).length();
	if (const CapsuleShape3D *capsule = Object::cast_to<CapsuleShape3D>(shape.ptr())) {
		const float half = MAX(capsule->get_height() * 0.5f - capsule->get_radius(), 0.0f);
		r_a = x.xform(Vector3(0, -half, 0));
		r_b = x.xform(Vector3(0, half, 0));
		r_radius = capsule->get_radius() * MAX(sx, sz);
		return true;
	}
	if (const SphereShape3D *sphere = Object::cast_to<SphereShape3D>(shape.ptr())) {
		r_a = x.origin;
		r_b = x.origin;
		r_radius = sphere->get_radius() * MAX(sx, MAX(sy, sz));
		return true;
	}
	return false;
}

static void add_shapes_under(Node *p_node, LocalVector<CollisionShape3D *> &r_shapes) {
	for (int i = 0; i < p_node->get_child_count(); i++) {
		if (CollisionShape3D *shape = Object::cast_to<CollisionShape3D>(p_node->get_child(i))) {
			r_shapes.push_back(shape);
		}
	}
}

// The head's sphere, then the colliders' shapes within the hair's reach, nearest first (at most GroomSim::MAX_COLLIDERS),
// each with where it was last frame.
void PhysXGroom3D::_gather_colliders(const Transform3D &p_xform, LocalVector<GroomSim::Collider> &r_colliders) {
	struct Found {
		ObjectID id;
		Vector3 a;
		Vector3 b;
		float radius = 0.0f;
	};
	LocalVector<Found> found;
	const Vector3 scale = p_xform.basis.get_scale().abs();
	const float largest = MAX(scale.x, MAX(scale.y, scale.z));
	if (collide_with_head && head_radius > 0.0f) {
		Found head;
		head.id = get_instance_id();
		head.a = p_xform.xform(head_center);
		head.b = head.a;
		head.radius = head_radius * largest;
		found.push_back(head);
	}
	LocalVector<CollisionShape3D *> shapes;
	for (int i = 0; i < colliders.size(); i++) {
		Node *node = get_node_or_null(colliders[i]);
		if (node == nullptr) {
			continue;
		}
		if (CollisionShape3D *shape = Object::cast_to<CollisionShape3D>(node)) {
			shapes.push_back(shape);
		} else if (Object::cast_to<Skeleton3D>(node)) {
			for (int j = 0; j < node->get_child_count(true); j++) {
				if (PhysicalBoneSimulator3D *simulator = Object::cast_to<PhysicalBoneSimulator3D>(node->get_child(j, true))) {
					for (int k = 0; k < simulator->get_child_count(); k++) {
						if (PhysicalBone3D *bone = Object::cast_to<PhysicalBone3D>(simulator->get_child(k))) {
							add_shapes_under(bone, shapes);
						}
					}
				}
			}
		} else {
			add_shapes_under(node, shapes);
		}
	}
	// As far as a strand can reach: the groom's box and its longest strand around it.
	const Vector3 reach_center = p_xform.xform(aabb.get_center());
	const float reach = (aabb.size.length() * 0.5f + longest_strand) * largest;
	// The nearest first: a whole ragdoll can be in reach (lying down), and its far side matters least.
	struct Near {
		Found found;
		float gap = 0.0f;
		bool operator<(const Near &p_other) const { return gap < p_other.gap; }
	};
	LocalVector<Near> near;
	for (CollisionShape3D *shape : shapes) {
		Near n;
		if (!shape_capsule(shape, n.found.a, n.found.b, n.found.radius)) {
			continue;
		}
		n.gap = Geometry3D::get_closest_point_to_segment(reach_center, n.found.a, n.found.b).distance_to(reach_center) - n.found.radius;
		if (n.gap > reach) {
			continue;
		}
		n.found.id = shape->get_instance_id();
		near.push_back(n);
	}
	near.sort();
	for (const Near &n : near) {
		if (found.size() >= (uint32_t)GroomSim::MAX_COLLIDERS) {
			break;
		}
		found.push_back(n.found);
	}
	LocalVector<ColliderEnds> ends;
	for (const Found &f : found) {
		GroomSim::Collider c;
		c.a = f.a;
		c.b = f.b;
		c.radius = f.radius;
		// New this frame: still.
		c.a_prev = f.a;
		c.b_prev = f.b;
		for (const ColliderEnds &e : collider_ends) {
			if (e.id == f.id) {
				c.a_prev = e.a;
				c.b_prev = e.b;
				break;
			}
		}
		r_colliders.push_back(c);
		ends.push_back({ f.id, f.a, f.b });
	}
	collider_ends = ends;
}

int PhysXGroom3D::get_drawn_strand_count() const {
	const int count = groom.is_valid() ? groom->get_strand_count() : 0;
	return MIN(count, (int)Math::ceil(strand_fraction * count));
}

void PhysXGroom3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			set_process_internal(true);
			_update_lod();
			_update_screen_space();
		} break;
		case NOTIFICATION_EXIT_TREE: {
			set_process_internal(false);
			_stop_simulation();
			if (screen_space_world.is_valid()) {
				PhysXGroomRenderEffect::detach_world(screen_space_world);
				screen_space_world = ObjectID();
			}
			PhysXGroomRenderEffect::remove_groom(get_instance_id());
			sent_to_effect = false;
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			_update_simulation(get_process_delta_time());
			_update_lod();
			_update_screen_space();
		} break;
	}
}

// At most this many strand segments per pixel of the groom's size on screen, at GeometryInstance3D.lod_bias 1 (higher
// keeps more): Indiana Jones and the Great Circle's 2000 per 64x64 tile. Beyond it they pile up on the same pixels,
// each drawn at least a pixel wide, and cost more the further away the groom is.
static constexpr float LOD_SEGMENTS_PER_PIXEL = 2000.0f / (64.0f * 64.0f);

void PhysXGroom3D::_update_lod() {
	const int count = groom.is_valid() ? groom->get_strand_count() : 0;
	float fraction = 1.0f;
	if (count > 0) {
		if (max_strands > 0) {
			fraction = MIN(fraction, (float)max_strands / (float)count);
		}
		Viewport *viewport = is_inside_tree() ? get_viewport() : nullptr;
		Camera3D *camera = viewport ? viewport->get_camera_3d() : nullptr;
		// Not in the editor: its camera isn't the viewport's.
		if (camera && !Engine::get_singleton()->is_editor_hint()) {
			const AABB box = get_global_transform().xform(aabb);
			const float radius = box.size.length() * 0.5f;
			const float height = viewport->get_visible_rect().size.y;
			// The groom's bounding sphere across, on screen.
			float pixels;
			if (camera->get_projection() == Camera3D::PROJECTION_ORTHOGONAL) {
				pixels = 2.0f * radius / MAX(camera->get_size(), 0.0001f) * height;
			} else {
				const float distance = MAX(camera->get_global_position().distance_to(box.get_center()) - radius, 0.01f);
				pixels = radius / (distance * Math::tan(Math::deg_to_rad(camera->get_fov()) * 0.5f)) * height;
			}
			const float segments = (float)MAX(groom->get_point_count() - count, 1);
			fraction = MIN(fraction, pixels * pixels * LOD_SEGMENTS_PER_PIXEL * get_lod_bias() / segments);
		}
		// At least a few strands, however far.
		fraction = CLAMP(fraction, MIN(64.0f / count, 1.0f), 1.0f);
	}
	if (!Math::is_equal_approx(fraction, strand_fraction)) {
		strand_fraction = fraction;
		RenderingServer::get_singleton()->material_set_param(material, "strand_fraction", strand_fraction);
	}
}

void PhysXGroom3D::_update_material() {
	RenderingServer *rs = RenderingServer::get_singleton();
	rs->material_set_param(material, "root_color", root_color);
	rs->material_set_param(material, "tip_color", tip_color);
	rs->material_set_param(material, "width_scale", width_scale);
	rs->material_set_param(material, "strand_fraction", strand_fraction);
	rs->material_set_param(material, "self_shadow", self_shadow);
	rs->material_set_param(material, "hair_roughness", roughness);
	rs->material_set_param(material, "shading_model", (int)shading_model);
}

// The sphere the points lie closest to (least squares), for the head the roots sit on; false when they don't make one.
static bool fit_sphere(const LocalVector<Vector3> &p_points, Vector3 &r_center, float &r_radius) {
	if (p_points.size() < 4) {
		return false;
	}
	// |p|^2 = 2 c . p + k, k = r^2 - |c|^2: linear in (c, k). Normal equations, solved by Gaussian elimination.
	double m[4][5] = {};
	for (const Vector3 &p : p_points) {
		const double row[4] = { 2.0 * p.x, 2.0 * p.y, 2.0 * p.z, 1.0 };
		const double rhs = (double)p.length_squared();
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				m[i][j] += row[i] * row[j];
			}
			m[i][4] += row[i] * rhs;
		}
	}
	for (int c = 0; c < 4; c++) {
		int pivot = c;
		for (int r = c + 1; r < 4; r++) {
			if (Math::abs(m[r][c]) > Math::abs(m[pivot][c])) {
				pivot = r;
			}
		}
		if (Math::abs(m[pivot][c]) < 1e-18) {
			return false;
		}
		for (int j = 0; j < 5; j++) {
			SWAP(m[c][j], m[pivot][j]);
		}
		for (int r = 0; r < 4; r++) {
			if (r != c) {
				const double f = m[r][c] / m[c][c];
				for (int j = c; j < 5; j++) {
					m[r][j] -= f * m[c][j];
				}
			}
		}
	}
	const Vector3 center((real_t)(m[0][4] / m[0][0]), (real_t)(m[1][4] / m[1][1]), (real_t)(m[2][4] / m[2][2]));
	const double r2 = m[3][4] / m[3][3] + (double)center.length_squared();
	if (r2 <= 0.0) {
		return false;
	}
	r_center = center;
	r_radius = (float)Math::sqrt(r2);
	return true;
}

void PhysXGroom3D::_rebuild() {
	_stop_simulation();
	RenderingServer *rs = RenderingServer::get_singleton();
	rs->mesh_clear(mesh);
	aabb = AABB();
	head_radius = 0.0f;
	longest_strand = 0.0f;
	if (groom.is_null() || groom->get_strand_count() == 0) {
		update_gizmos();
		return;
	}
	ERR_FAIL_COND_MSG(!groom->is_valid(), "PhysXGroom3D: the groom's points, strand_offsets and widths don't agree.");

	const PackedVector3Array points = groom->get_points();
	const PackedInt32Array offsets = groom->get_strand_offsets();
	const PackedFloat32Array widths = groom->get_widths();
	const PackedFloat32Array baked_shadow = groom->get_self_shadow();
	const bool has_shadow = baked_shadow.size() == points.size() * 4;
	const int strand_count = groom->get_strand_count();

	PackedVector3Array vertices;
	PackedVector3Array directions;
	PackedVector2Array uvs;
	PackedVector2Array uv2s;
	PackedInt32Array indices;
	PackedFloat32Array shadow;
	shadow.resize(points.size() * 2 * 4);
	float *sw = shadow.ptrw();
	vertices.resize(points.size() * 2);
	directions.resize(points.size() * 2);
	uvs.resize(points.size() * 2);
	uv2s.resize(points.size() * 2);
	Vector3 *vw = vertices.ptrw();
	Vector3 *dw = directions.ptrw();
	Vector2 *uw = uvs.ptrw();
	Vector2 *u2w = uv2s.ptrw();
	for (int s = 0; s < strand_count; s++) {
		const int from = offsets[s];
		const int count = offsets[s + 1] - from;
		for (int j = 0; j < count; j++) {
			const int p = from + j;
			const Vector3 ahead = points[from + MIN(j + 1, count - 1)];
			const Vector3 behind = points[from + MAX(j - 1, 0)];
			Vector3 direction = ahead - behind;
			direction = direction.is_zero_approx() ? Vector3(0, -1, 0) : direction.normalized();
			const float along = count > 1 ? (float)j / (float)(count - 1) : 0.0f;
			for (int side = 0; side < 2; side++) {
				const int v = p * 2 + side;
				vw[v] = points[p];
				dw[v] = direction;
				uw[v] = Vector2(side == 0 ? -1.0f : 1.0f, along);
				u2w[v] = Vector2(widths[p] * 0.5f, ((float)s + 0.5f) / (float)strand_count);
				for (int i = 0; i < 4; i++) {
					sw[v * 4 + i] = has_shadow ? baked_shadow[p * 4 + i] : 0.0f;
				}
			}
			if (j + 1 < count) {
				const int a = p * 2;
				indices.push_back(a);
				indices.push_back(a + 2);
				indices.push_back(a + 1);
				indices.push_back(a + 1);
				indices.push_back(a + 2);
				indices.push_back(a + 3);
			}
		}
	}

	Array arrays;
	arrays.resize(RSE::ARRAY_MAX);
	arrays[RSE::ARRAY_VERTEX] = vertices;
	arrays[RSE::ARRAY_NORMAL] = directions;
	arrays[RSE::ARRAY_TEX_UV] = uvs;
	arrays[RSE::ARRAY_TEX_UV2] = uv2s;
	arrays[RSE::ARRAY_INDEX] = indices;
	arrays[RSE::ARRAY_CUSTOM0] = shadow;
	rs->mesh_add_surface_from_arrays(mesh, RSE::PRIMITIVE_TRIANGLES, arrays, Array(), Dictionary(), (uint64_t)RSE::ARRAY_CUSTOM_RGBA_FLOAT << RSE::ARRAY_FORMAT_CUSTOM0_SHIFT);
	rs->mesh_surface_set_material(mesh, 0, material);
	// The vertex shader widens the ribbons, at least a pixel at any distance: a little room for that.
	aabb = groom->get_aabb();
	aabb = aabb.grow(MAX(aabb.get_longest_axis_size() * 0.01f, 0.001f) * MAX(width_scale, 1.0f));
	rs->mesh_set_custom_aabb(mesh, aabb);

	// For the simulation: the head the roots sit on (a sphere through them, if they make one no bigger than the groom),
	// and how far a strand can swing.
	LocalVector<Vector3> roots;
	for (int s = 0; s < strand_count; s++) {
		roots.push_back(points[offsets[s]]);
		float length = 0.0f;
		for (int i = offsets[s] + 1; i < offsets[s + 1]; i++) {
			length += points[i].distance_to(points[i - 1]);
		}
		longest_strand = MAX(longest_strand, length);
	}
	Vector3 center;
	float radius = 0.0f;
	if (fit_sphere(roots, center, radius) && radius < aabb.get_longest_axis_size()) {
		head_center = center;
		head_radius = radius;
	}
	update_gizmos();
	_update_lod();
	_send_points();
}

PhysXGroom3D::PhysXGroom3D() {
	RenderingServer *rs = RenderingServer::get_singleton();
	mesh = rs->mesh_create();
	material = rs->material_create();
	rs->material_set_shader(material, _shader());
	_update_material();
	set_base(mesh);
	sim = memnew(GroomSim);
	sim_texture.instantiate();
}

PhysXGroom3D::~PhysXGroom3D() {
	PhysXGroomRenderEffect::remove_groom(get_instance_id());
	sim_texture.unref();
	memdelete(sim);
	RenderingServer *rs = RenderingServer::get_singleton();
	rs->free_rid(mesh);
	rs->free_rid(material);
}

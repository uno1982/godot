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

#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "servers/rendering/rendering_server.h"

RID PhysXGroom3D::shared_shader;

// Each point is two vertices at the same place: UV.x says which side of the strand (-1, 1), UV.y how far along it
// (0 root, 1 tip), UV2.x the radius there, NORMAL the strand's direction. The vertex shader spreads them apart across
// the strand, facing the camera, at least a pixel apart; a strand thinner than that draws less of its pixel (alpha hash).
static const char *GROOM_SHADER = R"(
shader_type spatial;
render_mode skip_vertex_transform, cull_disabled;

uniform vec3 root_color : source_color;
uniform vec3 tip_color : source_color;
uniform float width_scale = 1.0;

varying vec3 v_tangent;
varying float v_along;
varying float v_coverage;
varying vec3 light_tangent;

void vertex() {
	vec3 pos = (MODELVIEW_MATRIX * vec4(VERTEX, 1.0)).xyz;
	vec3 tangent = normalize((MODELVIEW_MATRIX * vec4(NORMAL, 0.0)).xyz);
	vec3 across = cross(tangent, pos);
	across = dot(across, across) > 1e-12 ? normalize(across) : normalize(cross(tangent, vec3(0.0, 1.0, 0.0)));
	float radius = UV2.x * width_scale * length(MODELVIEW_MATRIX[0].xyz);
	float pixel = -pos.z * 2.0 / (PROJECTION_MATRIX[1][1] * VIEWPORT_SIZE.y);
	float half_width = max(radius, pixel * 0.5);
	v_coverage = clamp(radius / half_width, 0.0, 1.0);
	VERTEX = pos + across * UV.x * half_width;
	vec3 facing = normalize(cross(across, tangent));
	NORMAL = dot(facing, -pos) < 0.0 ? -facing : facing;
	TANGENT = tangent;
	BINORMAL = across;
	v_tangent = tangent;
	v_along = UV.y;
}

void fragment() {
	// Darker toward the root, where the strands shade each other.
	ALBEDO = mix(root_color, tip_color, v_along) * mix(0.55, 1.0, smoothstep(0.0, 0.35, v_along));
	ROUGHNESS = 0.6;
	ALPHA = v_coverage;
	ALPHA_HASH_SCALE = 1.0;
	light_tangent = normalize(v_tangent);
}

// Kajiya-Kay: light falls across a strand, not on a surface. Two highlights, the second shifted toward the tip and
// tinted by the hair's color.
float strand_highlight(vec3 tangent, vec3 half_vector, float exponent) {
	float th = dot(tangent, half_vector);
	return pow(sqrt(max(1.0 - th * th, 0.0)), exponent);
}

void light() {
	float tl = dot(light_tangent, LIGHT);
	float across = sqrt(max(1.0 - tl * tl, 0.0));
	// Lit from behind, a strand still passes some light through.
	float facing = mix(0.35, 1.0, clamp(dot(NORMAL, LIGHT) * 0.5 + 0.5, 0.0, 1.0));
	DIFFUSE_LIGHT += across * facing * ATTENUATION * LIGHT_COLOR / PI;
	vec3 h = normalize(LIGHT + VIEW);
	float primary = strand_highlight(normalize(light_tangent - NORMAL * 0.08), h, 90.0);
	float secondary = strand_highlight(normalize(light_tangent + NORMAL * 0.12), h, 24.0);
	SPECULAR_LIGHT += (primary * 0.06 + secondary * 0.12 * ALBEDO) * facing * ATTENUATION * LIGHT_COLOR;
}
)";

RID PhysXGroom3D::_shader() {
	if (!shared_shader.is_valid()) {
		RenderingServer *rs = RenderingServer::get_singleton();
		shared_shader = rs->shader_create();
		rs->shader_set_code(shared_shader, GROOM_SHADER);
	}
	return shared_shader;
}

void PhysXGroom3D::free_shared() {
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
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "width_scale", PROPERTY_HINT_RANGE, "0.1,10,0.01"), "set_width_scale", "get_width_scale");
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

void PhysXGroom3D::_update_material() {
	RenderingServer *rs = RenderingServer::get_singleton();
	rs->material_set_param(material, "root_color", root_color);
	rs->material_set_param(material, "tip_color", tip_color);
	rs->material_set_param(material, "width_scale", width_scale);
}

void PhysXGroom3D::_rebuild() {
	RenderingServer *rs = RenderingServer::get_singleton();
	rs->mesh_clear(mesh);
	aabb = AABB();
	if (groom.is_null() || groom->get_strand_count() == 0) {
		update_gizmos();
		return;
	}
	ERR_FAIL_COND_MSG(!groom->is_valid(), "PhysXGroom3D: the groom's points, strand_offsets and widths don't agree.");

	const PackedVector3Array points = groom->get_points();
	const PackedInt32Array offsets = groom->get_strand_offsets();
	const PackedFloat32Array widths = groom->get_widths();
	const int strand_count = groom->get_strand_count();

	PackedVector3Array vertices;
	PackedVector3Array directions;
	PackedVector2Array uvs;
	PackedVector2Array uv2s;
	PackedInt32Array indices;
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
				u2w[v] = Vector2(widths[p] * 0.5f, 0.0f);
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
	rs->mesh_add_surface_from_arrays(mesh, RSE::PRIMITIVE_TRIANGLES, arrays);
	rs->mesh_surface_set_material(mesh, 0, material);
	// The vertex shader widens the ribbons, at least a pixel at any distance: a little room for that.
	aabb = groom->get_aabb();
	aabb = aabb.grow(MAX(aabb.get_longest_axis_size() * 0.01f, 0.001f) * MAX(width_scale, 1.0f));
	rs->mesh_set_custom_aabb(mesh, aabb);
	update_gizmos();
}

PhysXGroom3D::PhysXGroom3D() {
	RenderingServer *rs = RenderingServer::get_singleton();
	mesh = rs->mesh_create();
	material = rs->material_create();
	rs->material_set_shader(material, _shader());
	_update_material();
	set_base(mesh);
}

PhysXGroom3D::~PhysXGroom3D() {
	RenderingServer *rs = RenderingServer::get_singleton();
	rs->free_rid(mesh);
	rs->free_rid(material);
}

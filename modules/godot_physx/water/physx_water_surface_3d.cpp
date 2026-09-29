/**************************************************************************/
/*  physx_water_surface_3d.cpp                                           */
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

#include "physx_water_surface_3d.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/resources/3d/primitive_meshes.h"
#include "scene/resources/material.h"
#include "scene/resources/shader.h"
#include "scene/resources/texture_rd.h"

namespace {
// A plain spatial Shader (Godot's own shading language, NOT the RD compute
// shaders the rest of this module uses). Displaces the mesh's own VERTEX.y
// by two independently-sampled height fields (ripple + FFT ocean, summed)
// sampled directly from the solver's shared-RenderingDevice textures via
// Texture2DRD; the separate CPU readback cache is only for sample_height().
//
// Real transparency via Godot's own GPU alpha blending (ALPHA, blend_mix),
// not manual screen-space refraction. An earlier version manually sampled
// hint_screen_texture and composited by hand -- confirmed via direct testing
// that this custom engine build's screen-texture capture reads severely
// overbright (even attenuated to 15% strength it still blew the surface to
// solid white; the engine has no REFRACTION built-in either, confirmed
// directly against servers/rendering/shader_types.cpp, so that wasn't an
// option). Real GPU alpha blending sidesteps the broken capture path
// entirely -- the renderer composites against whatever's already correctly
// drawn underneath, no re-sampling needed. Loses the distortion detail a
// real refraction offset would add; not worth it while the capture itself
// is broken.
//
// Real per-vertex normals from a central-difference of the SAME combined
// height field the vertex is displaced by -- the surface used to move but
// never actually shade like it moved, which was a real, separate
// contributor to an earlier "looks flat" complaint. Feeds a small, bounded,
// self-contained shading term in fragment() (unshaded, not Godot's real
// PBR NORMAL) -- see fragment()'s own comment for why real PBR lighting on
// this material caused a genuine overexposure bug.
const char *WATER_SHADER_SRC = R"(
shader_type spatial;
render_mode blend_mix, cull_disabled, unshaded;

uniform sampler2D ripple_height_tex : hint_default_black, filter_linear, repeat_disable;
uniform sampler2D ocean_height_tex : hint_default_black, filter_linear, repeat_disable;
uniform vec2 ripple_domain_size = vec2(20.0, 20.0);
uniform vec2 ocean_domain_size = vec2(40.0, 40.0);
uniform vec4 water_color : source_color = vec4(0.09, 0.32, 0.42, 0.65);

varying vec3 v_world_normal;

float sample_h(vec2 world_xz) {
	vec2 uv_r = world_xz / ripple_domain_size + 0.5;
	vec2 uv_o = world_xz / ocean_domain_size + 0.5;
	return texture(ripple_height_tex, clamp(uv_r, vec2(0.0), vec2(1.0))).r + texture(ocean_height_tex, clamp(uv_o, vec2(0.0), vec2(1.0))).r;
}

void vertex() {
	vec3 world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	VERTEX.y += sample_h(world_pos.xz);

	// Central-difference normal -- e is in world meters, small relative to
	// the ripple/ocean grids' own cell size so it stays a meaningful local
	// gradient estimate, not aliased noise between texels.
	const float e = 0.2;
	float hx1 = sample_h(world_pos.xz + vec2(e, 0.0));
	float hx0 = sample_h(world_pos.xz - vec2(e, 0.0));
	float hz1 = sample_h(world_pos.xz + vec2(0.0, e));
	float hz0 = sample_h(world_pos.xz - vec2(0.0, e));
	v_world_normal = normalize(vec3(hx0 - hx1, 2.0 * e, hz0 - hz1));
}

void fragment() {
	// unshaded + a small self-contained, bounded normal-based shading term --
	// NOT Godot's real diffuse/specular PBR response. Real root cause of the
	// "whitewash": this material used render_mode diffuse_burley,
	// specular_schlick_ggx, so it got its own full sun+ambient PBR light
	// response computed as if fully opaque, and THAT result was then alpha-
	// blended on top of the floor's OWN independently-lit PBR response.
	// Alpha blending two independently, fully-lit PBR surfaces in linear HDR
	// sums MORE light than either surface alone before the single tonemap
	// pass sees it -- exceeding any reasonable reference white, regardless
	// of how dark either material's own albedo is on its own (confirmed:
	// the bare floor alone renders correctly; only the blended combination
	// blows out). Going unshaded removes the water from that PBR light
	// budget entirely -- only the floor is now really lit by the scene, and
	// the water contributes a fixed, self-bounded tint on top of it.
	vec3 n = normalize((VIEW_MATRIX * vec4(v_world_normal, 0.0)).xyz);
	// Fixed, hardcoded pseudo-light direction -- not a real scene light, just
	// a bounded shaping term so wave normals still visibly modulate the
	// water's own tint (the whole reason per-vertex normals were added this
	// session in the first place), capped to a small multiplier so it can
	// never itself become a source of overexposure.
	float ndotl = clamp(dot(n, vec3(0.3, 0.85, 0.4)), 0.0, 1.0);
	ALBEDO = water_color.rgb * mix(0.75, 1.15, ndotl);
	ALPHA = water_color.a;
}
)";

} // namespace

void PhysXWaterSurface3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_domain_size", "size"), &PhysXWaterSurface3D::set_domain_size);
	ClassDB::bind_method(D_METHOD("get_domain_size"), &PhysXWaterSurface3D::get_domain_size);
	ClassDB::bind_method(D_METHOD("set_grid_resolution", "n"), &PhysXWaterSurface3D::set_grid_resolution);
	ClassDB::bind_method(D_METHOD("get_grid_resolution"), &PhysXWaterSurface3D::get_grid_resolution);
	ClassDB::bind_method(D_METHOD("set_depth", "depth"), &PhysXWaterSurface3D::set_depth);
	ClassDB::bind_method(D_METHOD("get_depth"), &PhysXWaterSurface3D::get_depth);
	ClassDB::bind_method(D_METHOD("set_water_level", "level"), &PhysXWaterSurface3D::set_water_level);
	ClassDB::bind_method(D_METHOD("get_water_level"), &PhysXWaterSurface3D::get_water_level);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &PhysXWaterSurface3D::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &PhysXWaterSurface3D::get_damping);
	ClassDB::bind_method(D_METHOD("set_ripple_amplitude", "amplitude"), &PhysXWaterSurface3D::set_ripple_amplitude);
	ClassDB::bind_method(D_METHOD("get_ripple_amplitude"), &PhysXWaterSurface3D::get_ripple_amplitude);

	ClassDB::bind_method(D_METHOD("set_ocean_grid_resolution", "n"), &PhysXWaterSurface3D::set_ocean_grid_resolution);
	ClassDB::bind_method(D_METHOD("get_ocean_grid_resolution"), &PhysXWaterSurface3D::get_ocean_grid_resolution);
	ClassDB::bind_method(D_METHOD("set_ocean_domain_size", "size"), &PhysXWaterSurface3D::set_ocean_domain_size);
	ClassDB::bind_method(D_METHOD("get_ocean_domain_size"), &PhysXWaterSurface3D::get_ocean_domain_size);
	ClassDB::bind_method(D_METHOD("set_wind_speed", "speed"), &PhysXWaterSurface3D::set_wind_speed);
	ClassDB::bind_method(D_METHOD("get_wind_speed"), &PhysXWaterSurface3D::get_wind_speed);
	ClassDB::bind_method(D_METHOD("set_wind_direction", "dir"), &PhysXWaterSurface3D::set_wind_direction);
	ClassDB::bind_method(D_METHOD("get_wind_direction"), &PhysXWaterSurface3D::get_wind_direction);
	ClassDB::bind_method(D_METHOD("set_wave_amplitude", "amplitude"), &PhysXWaterSurface3D::set_wave_amplitude);
	ClassDB::bind_method(D_METHOD("get_wave_amplitude"), &PhysXWaterSurface3D::get_wave_amplitude);

	ClassDB::bind_method(D_METHOD("set_water_material", "material"), &PhysXWaterSurface3D::set_water_material);
	ClassDB::bind_method(D_METHOD("get_water_material"), &PhysXWaterSurface3D::get_water_material);

	ClassDB::bind_method(D_METHOD("set_caustics_enabled", "enabled"), &PhysXWaterSurface3D::set_caustics_enabled);
	ClassDB::bind_method(D_METHOD("get_caustics_enabled"), &PhysXWaterSurface3D::get_caustics_enabled);
	ClassDB::bind_method(D_METHOD("set_caustics_sun_direction", "direction"), &PhysXWaterSurface3D::set_caustics_sun_direction);
	ClassDB::bind_method(D_METHOD("get_caustics_sun_direction"), &PhysXWaterSurface3D::get_caustics_sun_direction);
	ClassDB::bind_method(D_METHOD("set_caustics_reference_depth", "depth"), &PhysXWaterSurface3D::set_caustics_reference_depth);
	ClassDB::bind_method(D_METHOD("get_caustics_reference_depth"), &PhysXWaterSurface3D::get_caustics_reference_depth);
	ClassDB::bind_method(D_METHOD("get_caustics_texture"), &PhysXWaterSurface3D::get_caustics_texture);
	ClassDB::bind_method(D_METHOD("get_caustics_light_right"), &PhysXWaterSurface3D::get_caustics_light_right);
	ClassDB::bind_method(D_METHOD("get_caustics_light_up"), &PhysXWaterSurface3D::get_caustics_light_up);
	ClassDB::bind_method(D_METHOD("get_caustics_origin"), &PhysXWaterSurface3D::get_caustics_origin);
	ClassDB::bind_method(D_METHOD("get_caustics_half_extent"), &PhysXWaterSurface3D::get_caustics_half_extent);

	ClassDB::bind_method(D_METHOD("sample_height", "world_pos"), &PhysXWaterSurface3D::sample_height);
	ClassDB::bind_method(D_METHOD("submit_sphere", "owner", "world_pos", "radius", "strength"), &PhysXWaterSurface3D::submit_sphere, DEFVAL(1.0f));
	ClassDB::bind_method(D_METHOD("clear_sphere", "owner"), &PhysXWaterSurface3D::clear_sphere);
	ClassDB::bind_method(D_METHOD("submit_impulse", "world_pos", "radius", "strength"), &PhysXWaterSurface3D::submit_impulse);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "domain_size"), "set_domain_size", "get_domain_size");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "grid_resolution"), "set_grid_resolution", "get_grid_resolution");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "depth"), "set_depth", "get_depth");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "water_level"), "set_water_level", "get_water_level");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping"), "set_damping", "get_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ripple_amplitude", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), "set_ripple_amplitude", "get_ripple_amplitude");

	ADD_GROUP("Ocean", "ocean_");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "ocean_grid_resolution"), "set_ocean_grid_resolution", "get_ocean_grid_resolution");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "ocean_domain_size"), "set_ocean_domain_size", "get_ocean_domain_size");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_speed"), "set_wind_speed", "get_wind_speed");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "wind_direction"), "set_wind_direction", "get_wind_direction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wave_amplitude"), "set_wave_amplitude", "get_wave_amplitude");

	ADD_GROUP("Rendering", "");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "water_material", PROPERTY_HINT_RESOURCE_TYPE, "ShaderMaterial"), "set_water_material", "get_water_material");

	ADD_GROUP("Caustics", "caustic");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "caustics_enabled"), "set_caustics_enabled", "get_caustics_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "caustics_sun_direction"), "set_caustics_sun_direction", "get_caustics_sun_direction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "caustics_reference_depth", PROPERTY_HINT_RANGE, "0.1,50,0.1,or_greater"), "set_caustics_reference_depth", "get_caustics_reference_depth");
}

void PhysXWaterSurface3D::set_domain_size(Vector2 p_size) {
	domain_size = p_size;
	_rebuild();
}

void PhysXWaterSurface3D::set_grid_resolution(int p_n) {
	grid_resolution = MAX(p_n, 4);
	_rebuild();
}

void PhysXWaterSurface3D::set_depth(float p_depth) {
	depth = MAX(p_depth, 0.1f);
	_rebuild();
}

void PhysXWaterSurface3D::set_water_level(float p_level) {
	water_level = p_level;
	_rebuild();
}

void PhysXWaterSurface3D::set_damping(float p_damping) {
	damping = p_damping;
	_rebuild();
}

void PhysXWaterSurface3D::set_ripple_amplitude(float p_amplitude) {
	ripple_amplitude = MAX(p_amplitude, 0.0f);
	_rebuild();
}

void PhysXWaterSurface3D::set_ocean_grid_resolution(int p_n) {
	// Must stay a power of two -- the Stockham FFT passes require it (see
	// water_solver.h). Round up rather than silently misbehave.
	int n = MAX(p_n, 4);
	int pow2 = 4;
	while (pow2 < n) {
		pow2 <<= 1;
	}
	ocean_grid_resolution = pow2;
	_rebuild();
}

void PhysXWaterSurface3D::set_ocean_domain_size(Vector2 p_size) {
	ocean_domain_size = p_size;
	_rebuild();
}

void PhysXWaterSurface3D::set_wind_speed(float p_speed) {
	wind_speed = MAX(p_speed, 0.01f);
	_rebuild();
}

void PhysXWaterSurface3D::set_wind_direction(Vector2 p_dir) {
	wind_direction = p_dir;
	_rebuild();
}

void PhysXWaterSurface3D::set_wave_amplitude(float p_amp) {
	wave_amplitude = MAX(p_amp, 0.0f);
	_rebuild();
}

void PhysXWaterSurface3D::set_water_material(const Ref<ShaderMaterial> &p_material) {
	water_material = p_material;
	if (mesh_instance != nullptr && water_material.is_valid()) {
		mesh_instance->set_material_override(water_material);
	}
}

void PhysXWaterSurface3D::set_caustics_enabled(bool p_enabled) {
	caustics_enabled = p_enabled;
	_rebuild();
}

void PhysXWaterSurface3D::set_caustics_sun_direction(Vector3 p_direction) {
	caustics_sun_direction = p_direction;
}

void PhysXWaterSurface3D::set_caustics_reference_depth(float p_depth) {
	caustics_reference_depth = MAX(p_depth, 0.1f);
}

Ref<Texture2D> PhysXWaterSurface3D::get_caustics_texture() const {
	if (!caustics_enabled || !solver.is_available() || caustics_texture.is_null()) {
		return Ref<Texture2D>();
	}
	return caustics_texture;
}

void PhysXWaterSurface3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			_rebuild();
			set_physics_process_internal(true);
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			set_physics_process_internal(false);
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			_update(get_physics_process_delta_time());
		} break;
	}
}

void PhysXWaterSurface3D::_rebuild() {
	if (!is_inside_world()) {
		return;
	}
	if (mesh_instance == nullptr) {
		mesh_instance = memnew(MeshInstance3D);
		mesh_instance->set_cast_shadows_setting(GeometryInstance3D::SHADOW_CASTING_SETTING_OFF);
		add_child(mesh_instance, false, INTERNAL_MODE_BACK);
	}
	if (water_material.is_null()) {
		Ref<Shader> shader;
		shader.instantiate();
		shader->set_code(WATER_SHADER_SRC);
		Ref<ShaderMaterial> m;
		m.instantiate();
		m->set_shader(shader);
		water_material = m;
	}
	mesh_instance->set_material_override(water_material);

	Ref<PlaneMesh> plane;
	plane.instantiate();
	plane->set_size(ocean_domain_size);
	// Subdivisions, not vertex count -- PlaneMesh's own convention.
	plane->set_subdivide_width(ocean_grid_resolution);
	plane->set_subdivide_depth(ocean_grid_resolution);
	water_mesh = plane;
	mesh_instance->set_mesh(water_mesh);

	water_material->set_shader_parameter("ripple_domain_size", domain_size);
	water_material->set_shader_parameter("ocean_domain_size", ocean_domain_size);

	// NOTIFICATION_INTERNAL_PHYSICS_PROCESS fires in the editor too, not just
	// Play -- same gotcha PhysXVehicle3D/PhysXDestructible3D already guard
	// against (see PhysXVehicle3D::_build()'s own comment on it). Without
	// this, just having the scene open dispatches real GPU compute work
	// every physics tick, which is exactly what a user reported seeing (the
	// water visibly animating while only hovering nodes in the editor,
	// followed by an editor crash likely from the render-thread dispatch
	// getting into an inconsistent state under the editor's non-realtime
	// update loop). No solver setup at all in the editor -- the node shows
	// a static flat mesh there instead, matching this class's own doc
	// comment ("only runs in an actual running game").
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	WaterSolver::Settings s;
	s.grid_resolution = grid_resolution;
	s.domain_size = domain_size;
	s.depth = depth;
	s.damping = damping;
	s.ripple_amplitude = ripple_amplitude;
	s.gravity = 9.81f;
	s.water_level = water_level;
	s.ocean_grid_resolution = ocean_grid_resolution;
	s.ocean_domain_size = ocean_domain_size;
	s.wind_speed = wind_speed;
	s.wind_direction = wind_direction;
	s.wave_amplitude = wave_amplitude;
	s.caustics_enabled = caustics_enabled;
	solver.configure(s);
	if (!solver.has_device() && !warned_no_device) {
		warned_no_device = true;
		WARN_PRINT("PhysXWaterSurface3D: the water compute solver could not start (no RenderingDevice / compute support).");
	}

	textures_bound = false; // solver was just rebuilt -- its old texture RIDs (if any) are gone, rebind once available
	if (caustics_texture.is_valid()) {
		caustics_texture->set_texture_rd_rid(RID());
	}
	frames_since_refresh = REFRESH_EVERY_FRAMES; // force an immediate CPU cache refresh on the next _update()
}

void PhysXWaterSurface3D::_update(double p_delta) {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	if (!solver.has_device()) {
		return;
	}
	solver.step(p_delta); // no-ops internally until solver.is_available() (see water_solver.cpp)
	if (!textures_bound && solver.is_available()) {
		_bind_textures();
	}
	if (solver.is_available() && caustics_enabled) {
		solver.render_caustics(get_global_position(), caustics_sun_direction, caustics_reference_depth);
	}
	frames_since_refresh++;
	if (frames_since_refresh >= REFRESH_EVERY_FRAMES) {
		frames_since_refresh = 0;
		_refresh_cpu_cache();
	}
}

void PhysXWaterSurface3D::_bind_textures() {
	// Zero-copy: wrap the solver's own RD texture RIDs directly. Set once --
	// Texture2DRD holds the RID by reference, so whatever the blit shaders
	// write each step (see water_solver.cpp's rt_step()) is what the
	// material samples automatically; no per-frame reassignment needed.
	if (ripple_height_tex.is_null()) {
		ripple_height_tex.instantiate();
	}
	if (ocean_height_tex.is_null()) {
		ocean_height_tex.instantiate();
	}
	ripple_height_tex->set_texture_rd_rid(solver.get_ripple_height_texture_rd_rid());
	ocean_height_tex->set_texture_rd_rid(solver.get_ocean_height_texture_rd_rid());
	if (caustics_enabled) {
		if (caustics_texture.is_null()) {
			caustics_texture.instantiate();
		}
		caustics_texture->set_texture_rd_rid(solver.get_caustics_texture_rd_rid());
	}
	if (water_material.is_valid()) {
		water_material->set_shader_parameter("ripple_height_tex", ripple_height_tex);
		water_material->set_shader_parameter("ocean_height_tex", ocean_height_tex);
	}
	textures_bound = true;
}

void PhysXWaterSurface3D::_refresh_cpu_cache() {
	// CPU-side cache for sample_height() only -- rendering doesn't touch
	// this at all (see _bind_textures()). Non-blocking: just whatever the
	// solver's last async readback landed, a few frames stale -- fine for
	// buoyancy, see water_solver.h's own note on why this stays a separate
	// path from the zero-copy texture above.
	solver.get_height_grid(cached_ripple_height, cached_ripple_n, cached_ripple_domain);
	Vector<float> imag_scratch; // the FFT's imaginary-part residual -- not needed here, see water_solver.h
	solver.get_ocean_height_grid(cached_ocean_height, imag_scratch, cached_ocean_n, cached_ocean_domain);
}

float PhysXWaterSurface3D::_bilinear_sample(const Vector<float> &p_grid, int p_n, Vector2 p_domain, float p_world_x, float p_world_z) const {
	if (p_grid.is_empty() || p_n <= 0) {
		return 0.0f;
	}
	const float gx = (p_world_x / (p_domain.x * 0.5f) + 1.0f) * 0.5f * p_n - 0.5f;
	const float gz = (p_world_z / (p_domain.y * 0.5f) + 1.0f) * 0.5f * p_n - 0.5f;
	const int x0 = CLAMP((int)floorf(gx), 0, p_n - 1);
	const int z0 = CLAMP((int)floorf(gz), 0, p_n - 1);
	const int x1 = CLAMP(x0 + 1, 0, p_n - 1);
	const int z1 = CLAMP(z0 + 1, 0, p_n - 1);
	const float fx = CLAMP(gx - x0, 0.0f, 1.0f);
	const float fz = CLAMP(gz - z0, 0.0f, 1.0f);
	const float h00 = p_grid[z0 * p_n + x0];
	const float h10 = p_grid[z0 * p_n + x1];
	const float h01 = p_grid[z1 * p_n + x0];
	const float h11 = p_grid[z1 * p_n + x1];
	return Math::lerp(Math::lerp(h00, h10, fx), Math::lerp(h01, h11, fx), fz);
}

float PhysXWaterSurface3D::sample_height(Vector3 p_world_pos) const {
	const float ripple = _bilinear_sample(cached_ripple_height, cached_ripple_n, cached_ripple_domain, p_world_pos.x, p_world_pos.z);
	const float ocean = _bilinear_sample(cached_ocean_height, cached_ocean_n, cached_ocean_domain, p_world_pos.x, p_world_pos.z);
	return water_level + ripple + ocean;
}

void PhysXWaterSurface3D::submit_sphere(int p_owner, Vector3 p_world_pos, float p_radius, float p_strength) {
	solver.submit_sphere((uint64_t)p_owner, p_world_pos, p_radius, p_strength);
}

void PhysXWaterSurface3D::clear_sphere(int p_owner) {
	solver.clear_sphere((uint64_t)p_owner);
}

void PhysXWaterSurface3D::submit_impulse(Vector3 p_world_pos, float p_radius, float p_strength) {
	solver.submit_impulse(p_world_pos, p_radius, p_strength);
}

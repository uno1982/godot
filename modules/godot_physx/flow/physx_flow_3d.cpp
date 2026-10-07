/**************************************************************************/
/*  physx_flow_3d.cpp                                                     */
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

#include "physx_flow_3d.h"

#include "flow_backend.h"
#include "physx_flow_emitter_3d.h"
#include "physx_flow_render_effect.h"

#include "core/config/engine.h"
#include "core/config/project_settings.h"
#include "core/object/class_db.h"
#include "core/object/object_id.h"
#include "core/os/os.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/light_3d.h"
#include "scene/main/scene_tree.h"
#include "scene/main/viewport.h"
#include "scene/resources/3d/world_3d.h"
#include "scene/resources/environment.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_server.h"
#include "servers/rendering/rendering_server_default.h"

#ifdef TOOLS_ENABLED
#include "editor/editor_interface.h"
#endif

namespace {

void fill_transform(const Transform3D &p_xform, FlowBackend::Emitter &r_emitter) {
	for (int c = 0; c < 3; c++) {
		const Vector3 col = p_xform.basis.get_column(c);
		r_emitter.basis[c][0] = col.x;
		r_emitter.basis[c][1] = col.y;
		r_emitter.basis[c][2] = col.z;
		r_emitter.origin[c] = p_xform.origin[c];
	}
}

FlowBackend::Emitter make_solid(const Transform3D &p_xform, const Vector3 &p_velocity) {
	FlowBackend::Emitter e;
	fill_transform(p_xform, e);
	for (int c = 0; c < 3; c++) {
		e.body_velocity[c] = p_velocity[c];
	}
	e.collision = true;
	return e;
}

} // namespace

PhysXFlow3D::PhysXFlow3D() {
}

Color PhysXFlow3D::_default_heat_at(float p_t) {
	// Used while heat_ramp is empty (a resource can't be a property default):
	// cool -> nothing, warm -> deep orange glow, hot -> yellow-white.
	const Color cool(0, 0, 0, 0);
	const Color warm(0.9, 0.25, 0.03, 0.6);
	const Color hot(1.0, 0.85, 0.5, 1.0);
	constexpr float WARM_AT = 0.35f;
	if (p_t < WARM_AT) {
		return cool.lerp(warm, p_t / WARM_AT);
	}
	return warm.lerp(hot, (p_t - WARM_AT) / (1.0f - WARM_AT));
}

PhysXFlow3D::~PhysXFlow3D() {
	_stop();
	if (library_acquired) {
		FlowBackend::release_library();
		library_acquired = false;
	}
}

void PhysXFlow3D::_start() {
	if (grid != nullptr) {
		return;
	}
	if (!library_acquired) {
		char err[512] = {};
		library_acquired = FlowBackend::acquire_library(err, sizeof(err));
		if (!library_acquired) {
			device_error = String::utf8(err);
			WARN_PRINT("PhysXFlow3D: NVIDIA Flow unavailable: " + device_error);
			update_configuration_warnings();
			return;
		}
	}
	rd = RenderingServer::get_singleton()->get_rendering_device();
	if (rd == nullptr) {
		device_error = "it needs a RenderingDevice (Forward+ or Mobile renderer, not Compatibility or headless)";
		update_configuration_warnings();
		return;
	}
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	grid = FlowBackend::grid_create_rd(rd, (uint32_t)max_blocks);
	max_blocks_warned = false;
	stat_start_ms = (OS::get_singleton()->get_ticks_usec() - t0) / 1000.0;
	device_error = grid != nullptr ? String() : String("Flow couldn't create its grid");
	update_configuration_warnings();
	sim_time = 0.0;
	step_count = 0;
	if (grid != nullptr && get_world_3d().is_valid()) {
		PhysXFlowRenderEffect::register_flow(this);
		PhysXFlowRenderEffect::attach_world(get_world_3d().ptr());
		attached_world = get_world_3d()->get_instance_id();
		PhysXFlowRenderEffect::install(get_world_3d().ptr(), get_viewport() ? get_viewport()->get_camera_3d() : nullptr);
	}
}

void PhysXFlow3D::_stop() {
	PhysXFlowRenderEffect::unregister_flow(this);
	if (attached_world.is_valid()) {
		PhysXFlowRenderEffect::detach_world(attached_world);
		attached_world = ObjectID();
	}
	if (query_shape.is_valid()) {
		PhysicsServer3D::get_singleton()->free_rid(query_shape);
		query_shape = RID();
	}
	if (grid != nullptr) {
		FlowBackend::grid_destroy(grid);
		grid = nullptr;
	}
	rd = nullptr;
	last_smoke = nullptr;
	last_smoke_size = 0;
	gas_bounds = AABB();
}

void PhysXFlow3D::render_rd(PhysXFlowRenderEffect *p_effect, RenderingDevice *p_rd, const float p_view[16], const float p_projection[16], const Size2i &p_size, const RID &p_depth, const RID &p_color) {
	if (grid == nullptr || p_rd != rd) {
		return;
	}
	const uint64_t out = FlowBackend::grid_render_rd(grid, p_view, p_projection, p_size.x, p_size.y, p_depth.get_id(), p_size.x, p_size.y, p_color.get_id());
	stat_render_calls++;
	stat_render_outputs += out != 0 ? 1 : 0;
	if (out != 0 && out != p_color.get_id()) {
		p_effect->copy_texture(p_rd, RID::from_uint64(out), p_color, p_size);
	}
	FlowBackend::grid_render_rd_finish(grid);
}

void PhysXFlow3D::_gather_emitters(LocalVector<FlowBackend::Emitter> &r_emitters, AABB &r_emitter_bounds) {
	bool first = true;
	for (Node *n : get_tree()->get_nodes_in_group(PhysXFlowEmitter3D::GROUP)) {
		PhysXFlowEmitter3D *e = Object::cast_to<PhysXFlowEmitter3D>(n);
		if (e == nullptr || !e->is_enabled() || !e->is_visible_in_tree() || e->get_flow() != this) {
			continue;
		}
		FlowBackend::Emitter fe;
		const Transform3D xf = e->get_global_transform();
		fill_transform(xf, fe);
		const bool box = e->get_shape() == PhysXFlowEmitter3D::SHAPE_BOX;
		fe.shape = box ? FlowBackend::Emitter::SHAPE_BOX : FlowBackend::Emitter::SHAPE_SPHERE;
		fe.radius = e->get_radius();
		const Vector3 half = e->get_size() * 0.5f;
		const Vector3 vel = e->get_velocity();
		const Vector3 body_vel = e->get_motion_velocity();
		for (int c = 0; c < 3; c++) {
			fe.half_size[c] = half[c];
			fe.velocity[c] = vel[c];
			fe.body_velocity[c] = body_vel[c];
		}
		fe.temperature = e->get_temperature();
		fe.fuel = e->get_fuel();
		fe.burn = e->get_burn();
		fe.smoke = e->get_smoke();
		fe.divergence = e->get_divergence();
		fe.couple_rate = e->get_couple_rate();
		fe.collision = e->is_collision();
		r_emitters.push_back(fe);
		if (first) {
			r_emitter_bounds = AABB(xf.origin, Vector3());
			first = false;
		} else {
			r_emitter_bounds.expand_to(xf.origin);
		}
	}
}

void PhysXFlow3D::_gather_colliders(const AABB &p_region, LocalVector<FlowBackend::Emitter> &r_emitters) {
	if (!collide_with_bodies || get_world_3d().is_null() || !p_region.has_volume()) {
		return;
	}
	PhysicsDirectSpaceState3D *space = get_world_3d()->get_direct_space_state();
	if (space == nullptr) {
		return;
	}
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	if (!query_shape.is_valid()) {
		query_shape = ps->box_shape_create();
	}
	ps->shape_set_data(query_shape, p_region.size * 0.5f);
	PhysicsDirectSpaceState3D::ShapeParameters query;
	query.shape_rid = query_shape;
	query.transform = Transform3D(Basis(), p_region.get_center());
	query.collision_mask = collision_mask;
	query.collide_with_bodies = true;
	query.collide_with_areas = false;
	constexpr int MAX_RESULTS = 64;
	PhysicsDirectSpaceState3D::ShapeResult results[MAX_RESULTS];
	const int count = space->intersect_shape(query, results, MAX_RESULTS);
	for (int i = 0; i < count; i++) {
		const RID body = results[i].rid;
		const int index = results[i].shape;
		const RID shape = ps->body_get_shape(body, index);
		if (!shape.is_valid()) {
			continue;
		}
		const Transform3D body_xform = ps->body_get_state(body, PhysicsServer3D::BODY_STATE_TRANSFORM);
		const Transform3D xform = body_xform * ps->body_get_shape_transform(body, index);
		const Vector3 velocity = ps->body_get_state(body, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY);
		const Variant shape_data = ps->shape_get_data(shape);
		switch (ps->shape_get_type(shape)) {
			case PhysicsServer3D::SHAPE_BOX: {
				FlowBackend::Emitter e = make_solid(xform, velocity);
				e.shape = FlowBackend::Emitter::SHAPE_BOX;
				const Vector3 half = shape_data;
				for (int c = 0; c < 3; c++) {
					e.half_size[c] = half[c];
				}
				r_emitters.push_back(e);
			} break;
			case PhysicsServer3D::SHAPE_SPHERE: {
				FlowBackend::Emitter e = make_solid(xform, velocity);
				e.shape = FlowBackend::Emitter::SHAPE_SPHERE;
				e.radius = shape_data;
				r_emitters.push_back(e);
			} break;
			case PhysicsServer3D::SHAPE_CAPSULE: {
				// A row of spheres along the capsule's axis (local Y).
				const Dictionary d = shape_data;
				const float radius = d["radius"];
				const float half_line = MAX(float(d["height"]) * 0.5f - radius, 0.0f);
				const int spheres = MAX(2, (int)Math::ceil(2.0f * half_line / MAX(radius, 0.01f)) + 1);
				for (int k = 0; k < spheres; k++) {
					const float y = -half_line + 2.0f * half_line * k / (spheres - 1);
					FlowBackend::Emitter e = make_solid(xform * Transform3D(Basis(), Vector3(0, y, 0)), velocity);
					e.shape = FlowBackend::Emitter::SHAPE_SPHERE;
					e.radius = radius;
					r_emitters.push_back(e);
				}
			} break;
			case PhysicsServer3D::SHAPE_CYLINDER: {
				const Dictionary d = shape_data;
				const float radius = d["radius"];
				FlowBackend::Emitter e = make_solid(xform, velocity);
				e.shape = FlowBackend::Emitter::SHAPE_BOX;
				e.half_size[0] = radius;
				e.half_size[1] = float(d["height"]) * 0.5f;
				e.half_size[2] = radius;
				r_emitters.push_back(e);
			} break;
			default: {
				// Convex / trimesh / heightmap: not yet (needs Flow's mesh
				// collider).
			} break;
		}
	}
}

void PhysXFlow3D::_gather_light(Vector3 &r_direction, Color &r_color, Color &r_ambient) {
	DirectionalLight3D *light = Object::cast_to<DirectionalLight3D>(ObjectDB::get_instance(light_id));
	if (light == nullptr || !light->is_inside_tree() || light_search_countdown-- <= 0) {
		light_search_countdown = 60;
		light = nullptr;
		Node *root = get_tree()->get_edited_scene_root();
		if (root == nullptr) {
			root = get_tree()->get_current_scene();
		}
		if (root != nullptr) {
			TypedArray<Node> lights = root->find_children("*", "DirectionalLight3D", true, false);
			for (int i = 0; i < lights.size(); i++) {
				DirectionalLight3D *l = Object::cast_to<DirectionalLight3D>(lights[i]);
				if (l != nullptr && l->is_visible_in_tree()) {
					light = l;
					break;
				}
			}
		}
		light_id = light != nullptr ? light->get_instance_id() : ObjectID();
	}
	r_direction = Vector3(0.3, 1.0, 0.2).normalized();
	r_color = Color(1, 1, 1);
	if (light != nullptr && light->is_visible_in_tree()) {
		// A directional light shines along its -Z: toward it is +Z.
		r_direction = light->get_global_transform().basis.get_column(2).normalized();
		r_color = light->get_color().srgb_to_linear() * light->get_param(Light3D::PARAM_ENERGY);
	}
	r_ambient = Color(0.25, 0.27, 0.3);
	if (get_world_3d().is_valid() && get_world_3d()->get_environment().is_valid()) {
		Ref<Environment> env = get_world_3d()->get_environment();
		r_ambient = env->get_ambient_light_color().srgb_to_linear() * env->get_ambient_light_energy();
	}
}

void PhysXFlow3D::_fill_render_settings(FlowBackend::Settings &r_settings) {
	// Flow colors by temperature through a colormap and lights it with its
	// own self-shadowing: the smoke's color under the sun, with the heat
	// ramp's glow; opacity comes from the smoke itself.
	Vector3 dir;
	Color light;
	Color ambient;
	_gather_light(dir, light, ambient);
	for (int i = 0; i < FlowBackend::Settings::COLORMAP_POINTS; i++) {
		const float t = (float)i / (float)(FlowBackend::Settings::COLORMAP_POINTS - 1);
		const Color smoke = (smoke_ramp.is_valid() ? smoke_ramp->get_color_at_offset(t) : smoke_color).srgb_to_linear();
		const Color heat = heat_ramp.is_valid() ? heat_ramp->get_color_at_offset(t) : _default_heat_at(t);
		// Where it glows, the glow replaces the lit smoke instead of adding to
		// it (bright smoke plus orange reads pink).
		const float glow_amount = emission_strength > 0.0f ? CLAMP(heat.a, 0.0f, 1.0f) : 0.0f;
		const Color glow = heat.srgb_to_linear() * (heat.a * emission_strength);
		r_settings.colormap_rgb[i][0] = smoke.r * light.r * (1.0f - glow_amount) + glow.r;
		r_settings.colormap_rgb[i][1] = smoke.g * light.g * (1.0f - glow_amount) + glow.g;
		r_settings.colormap_rgb[i][2] = smoke.b * light.b * (1.0f - glow_amount) + glow.b;
	}
	r_settings.temperature_range = temperature_range;
	r_settings.attenuation = smoke_density;
	r_settings.light_direction[0] = dir.x;
	r_settings.light_direction[1] = dir.y;
	r_settings.light_direction[2] = dir.z;
	// Ambient as the floor of the self-shadowing, relative to the sun.
	r_settings.shadow_min_intensity = CLAMP(ambient.get_luminance() / MAX(light.get_luminance(), 0.001f), 0.02f, 1.0f);
	r_settings.shadow_steps = shadow_steps;
}

void PhysXFlow3D::_step(double p_delta) {
	if (grid == nullptr) {
		return;
	}
#ifdef TOOLS_ENABLED
	// The editor's copy holds still while the project runs from the editor:
	// the game simulates its own, and this one would only take GPU time (and
	// keep the editor redrawing) in the background.
	if (Engine::get_singleton()->is_editor_hint() && EditorInterface::get_singleton() != nullptr && EditorInterface::get_singleton()->is_playing_scene()) {
		return;
	}
#endif
	LocalVector<FlowBackend::Emitter> emitters;
	AABB emitter_bounds;
	_gather_emitters(emitters, emitter_bounds);
	draw_center = emitters.is_empty() ? get_global_position() : emitter_bounds.get_center();
	// Solids near where the gas comes from -- and where it has spread to,
	// when it's read back.
	AABB region = emitters.is_empty() ? AABB() : emitter_bounds.grow(collision_range);
	if (gas_bounds.has_volume()) {
		region = region.has_volume() ? region.merge(gas_bounds.grow(1.0)) : gas_bounds.grow(1.0);
	}
	_gather_colliders(region, emitters);

	FlowBackend::Settings settings;
	settings.cell_size = cell_size;
	settings.buoyancy = buoyancy;
	settings.cooling_rate = cooling_rate;
	settings.smoke_fade = smoke_fade;
	settings.vorticity = vorticity;
	settings.ignition_temperature = ignition_temperature;
	settings.combustion = combustion;
	// A full field each time; often enough for queries, and it stalls
	// RenderingDevice's staging memory when done every step on large gas.
	settings.readback = cpu_readback && (step_count++ % READBACK_INTERVAL) == 0;
	_fill_render_settings(settings);
	// The project's gravity sets which way is "up" for hot gas.
	const Vector3 gravity_vector = GLOBAL_GET("physics/3d/default_gravity_vector");
	const float gravity_strength = GLOBAL_GET("physics/3d/default_gravity");
	const Vector3 g = gravity_vector * gravity_strength;
	settings.gravity[0] = g.x;
	settings.gravity[1] = g.y;
	settings.gravity[2] = g.z;

	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	if (!FlowBackend::grid_step(grid, sim_time + p_delta, (float)p_delta, settings, emitters.ptr(), (int)emitters.size())) {
		return;
	}
	sim_time += p_delta;
	if (!max_blocks_warned && FlowBackend::grid_active_blocks(grid) >= (uint32_t)max_blocks) {
		max_blocks_warned = true;
		WARN_PRINT(vformat("PhysXFlow3D %s: the gas has used all %d max_blocks and is being cut off; raise max_blocks (more GPU memory) or make the effect smaller.", get_path(), max_blocks));
	}
	stat_step_ms = (OS::get_singleton()->get_ticks_usec() - t0) / 1000.0;

	if (cpu_readback) {
		_read_back();
	}
	if (get_world_3d().is_valid() && attached_world.is_valid()) {
		PhysXFlowRenderEffect::install(get_world_3d().ptr(), get_viewport() ? get_viewport()->get_camera_3d() : nullptr);
	}
	if (Engine::get_singleton()->is_editor_hint()) {
		// Nothing the RenderingServer sees changes when the gas moves, and the
		// editor only redraws its viewports on changes.
		RenderingServerDefault::redraw_request();
	}
}

void PhysXFlow3D::_read_back() {
	const uint8_t *temp = nullptr;
	const uint8_t *smoke = nullptr;
	uint64_t temp_size = 0;
	uint64_t smoke_size = 0;
	if (!FlowBackend::grid_get_readback(grid, &temp, &temp_size, &smoke, &smoke_size)) {
		return;
	}
	stat_readback_bytes = (int64_t)(temp_size + smoke_size);
	last_smoke = smoke;
	last_smoke_size = smoke_size;
	float lo[3];
	float hi[3];
	if (FlowBackend::nanovdb_bounds(smoke, smoke_size, lo, hi)) {
		gas_bounds = AABB(Vector3(lo[0], lo[1], lo[2]), Vector3(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]));
	}
}

void PhysXFlow3D::set_max_blocks(int p_v) {
	p_v = CLAMP(p_v, 64, 16384);
	if (p_v == max_blocks) {
		return;
	}
	max_blocks = p_v;
	// The budget is fixed when the grid is made: start over with the new one.
	if (grid != nullptr) {
		_stop();
		_start();
	}
}

void PhysXFlow3D::set_cpu_readback(bool p_v) {
	cpu_readback = p_v;
	if (!cpu_readback) {
		last_smoke = nullptr;
		last_smoke_size = 0;
		gas_bounds = AABB();
		stat_readback_bytes = 0;
	}
}

float PhysXFlow3D::sample_smoke(const Vector3 &p_position) const {
	if (!cpu_readback || last_smoke == nullptr) {
		return -1.0f;
	}
	const float origin[3] = { (float)p_position.x - 0.5f * cell_size, (float)p_position.y - 0.5f * cell_size, (float)p_position.z - 0.5f * cell_size };
	const int dims[3] = { 1, 1, 1 };
	float value = 0.0f;
	FlowBackend::sample_dense(last_smoke, last_smoke_size, origin, cell_size, dims, 0, 1, &value, 1);
	return value;
}

Dictionary PhysXFlow3D::get_stats() const {
	Dictionary d;
	d["running"] = grid != nullptr;
	d["error"] = device_error;
	d["start_ms"] = stat_start_ms;
	d["step_ms"] = stat_step_ms;
	d["render_calls"] = stat_render_calls;
	d["render_outputs"] = stat_render_outputs;
	d["readback_bytes"] = stat_readback_bytes;
	d["bounds"] = gas_bounds;
	int pipelines = 0;
	int failures = 0;
	int passes = 0;
	int buffers = 0;
	int textures = 0;
	uint64_t frame = 0;
	uint64_t completed = 0;
	if (FlowBackend::grid_rd_stats(grid, pipelines, failures, passes, buffers, textures, frame, completed)) {
		d["pipelines"] = pipelines;
		d["pipeline_failures"] = failures;
		d["passes"] = passes;
		d["buffers"] = buffers;
		d["textures"] = textures;
		d["frame"] = frame;
		d["frame_completed"] = completed;
		d["active_blocks"] = FlowBackend::grid_active_blocks(grid);
	}
	return d;
}

void PhysXFlow3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
		case NOTIFICATION_EXIT_TREE: {
			if (p_what == NOTIFICATION_ENTER_TREE) {
				add_to_group(GROUP);
			} else {
				remove_from_group(GROUP);
			}
			// Which flow an emitter without a `flow` path feeds depends on how
			// many there are.
			if (is_inside_tree()) {
				for (Node *n : get_tree()->get_nodes_in_group(PhysXFlowEmitter3D::GROUP)) {
					n->update_configuration_warnings();
				}
			}
		} break;
		case NOTIFICATION_ENTER_WORLD: {
			_start();
			set_process_internal(true);
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			set_process_internal(false);
			_stop();
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			// Once per frame, as Flow is meant to be driven: its own fixed-rate
			// stepper takes the frame time and steps at most once. Driven by
			// physics ticks instead, a slow frame is followed by several steps
			// in the next, each uploading Flow's sparse table again -- with a
			// few grids that runs RenderingDevice's staging memory out, which
			// stalls the frame, which makes for more ticks, and so on.
			_step(get_process_delta_time());
		} break;
	}
}

PackedStringArray PhysXFlow3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();
	if (!device_error.is_empty()) {
		warnings.push_back(vformat(RTR("NVIDIA Flow isn't running: %s"), device_error));
	}
	return warnings;
}

void PhysXFlow3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_cell_size", "size"), &PhysXFlow3D::set_cell_size);
	ClassDB::bind_method(D_METHOD("get_cell_size"), &PhysXFlow3D::get_cell_size);
	ClassDB::bind_method(D_METHOD("set_buoyancy", "buoyancy"), &PhysXFlow3D::set_buoyancy);
	ClassDB::bind_method(D_METHOD("get_buoyancy"), &PhysXFlow3D::get_buoyancy);
	ClassDB::bind_method(D_METHOD("set_cooling_rate", "rate"), &PhysXFlow3D::set_cooling_rate);
	ClassDB::bind_method(D_METHOD("get_cooling_rate"), &PhysXFlow3D::get_cooling_rate);
	ClassDB::bind_method(D_METHOD("set_smoke_fade", "fade"), &PhysXFlow3D::set_smoke_fade);
	ClassDB::bind_method(D_METHOD("get_smoke_fade"), &PhysXFlow3D::get_smoke_fade);
	ClassDB::bind_method(D_METHOD("set_vorticity", "vorticity"), &PhysXFlow3D::set_vorticity);
	ClassDB::bind_method(D_METHOD("get_vorticity"), &PhysXFlow3D::get_vorticity);
	ClassDB::bind_method(D_METHOD("set_ignition_temperature", "temperature"), &PhysXFlow3D::set_ignition_temperature);
	ClassDB::bind_method(D_METHOD("get_ignition_temperature"), &PhysXFlow3D::get_ignition_temperature);
	ClassDB::bind_method(D_METHOD("set_combustion", "enabled"), &PhysXFlow3D::set_combustion);
	ClassDB::bind_method(D_METHOD("is_combustion"), &PhysXFlow3D::is_combustion);
	ClassDB::bind_method(D_METHOD("set_collide_with_bodies", "enabled"), &PhysXFlow3D::set_collide_with_bodies);
	ClassDB::bind_method(D_METHOD("is_collide_with_bodies"), &PhysXFlow3D::is_collide_with_bodies);
	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &PhysXFlow3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &PhysXFlow3D::get_collision_mask);
	ClassDB::bind_method(D_METHOD("set_collision_range", "range"), &PhysXFlow3D::set_collision_range);
	ClassDB::bind_method(D_METHOD("get_collision_range"), &PhysXFlow3D::get_collision_range);
	ClassDB::bind_method(D_METHOD("set_smoke_density", "density"), &PhysXFlow3D::set_smoke_density);
	ClassDB::bind_method(D_METHOD("get_smoke_density"), &PhysXFlow3D::get_smoke_density);
	ClassDB::bind_method(D_METHOD("set_smoke_color", "color"), &PhysXFlow3D::set_smoke_color);
	ClassDB::bind_method(D_METHOD("get_smoke_color"), &PhysXFlow3D::get_smoke_color);
	ClassDB::bind_method(D_METHOD("set_smoke_ramp", "ramp"), &PhysXFlow3D::set_smoke_ramp);
	ClassDB::bind_method(D_METHOD("get_smoke_ramp"), &PhysXFlow3D::get_smoke_ramp);
	ClassDB::bind_method(D_METHOD("set_heat_ramp", "ramp"), &PhysXFlow3D::set_heat_ramp);
	ClassDB::bind_method(D_METHOD("get_heat_ramp"), &PhysXFlow3D::get_heat_ramp);
	ClassDB::bind_method(D_METHOD("set_temperature_range", "range"), &PhysXFlow3D::set_temperature_range);
	ClassDB::bind_method(D_METHOD("get_temperature_range"), &PhysXFlow3D::get_temperature_range);
	ClassDB::bind_method(D_METHOD("set_emission_strength", "strength"), &PhysXFlow3D::set_emission_strength);
	ClassDB::bind_method(D_METHOD("get_emission_strength"), &PhysXFlow3D::get_emission_strength);
	ClassDB::bind_method(D_METHOD("set_shadow_steps", "steps"), &PhysXFlow3D::set_shadow_steps);
	ClassDB::bind_method(D_METHOD("get_shadow_steps"), &PhysXFlow3D::get_shadow_steps);
	ClassDB::bind_method(D_METHOD("set_max_blocks", "blocks"), &PhysXFlow3D::set_max_blocks);
	ClassDB::bind_method(D_METHOD("get_max_blocks"), &PhysXFlow3D::get_max_blocks);
	ClassDB::bind_method(D_METHOD("set_cpu_readback", "enabled"), &PhysXFlow3D::set_cpu_readback);
	ClassDB::bind_method(D_METHOD("is_cpu_readback"), &PhysXFlow3D::is_cpu_readback);
	ClassDB::bind_method(D_METHOD("sample_smoke", "position"), &PhysXFlow3D::sample_smoke);
	ClassDB::bind_method(D_METHOD("is_running"), &PhysXFlow3D::is_running);
	ClassDB::bind_method(D_METHOD("get_stats"), &PhysXFlow3D::get_stats);

	ADD_GROUP("Simulation", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cell_size", PROPERTY_HINT_RANGE, "0.005,1,0.001,or_greater,suffix:m"), "set_cell_size", "get_cell_size");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_blocks", PROPERTY_HINT_RANGE, "64,16384,1,or_greater"), "set_max_blocks", "get_max_blocks");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "buoyancy", PROPERTY_HINT_RANGE, "-10,10,0.01"), "set_buoyancy", "get_buoyancy");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cooling_rate", PROPERTY_HINT_RANGE, "0,10,0.01"), "set_cooling_rate", "get_cooling_rate");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "smoke_fade", PROPERTY_HINT_RANGE, "0,10,0.01"), "set_smoke_fade", "get_smoke_fade");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vorticity", PROPERTY_HINT_RANGE, "0,5,0.01"), "set_vorticity", "get_vorticity");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "combustion"), "set_combustion", "is_combustion");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ignition_temperature", PROPERTY_HINT_RANGE, "0,2,0.001"), "set_ignition_temperature", "get_ignition_temperature");
	ADD_GROUP("Collision", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_with_bodies"), "set_collide_with_bodies", "is_collide_with_bodies");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collision_range", PROPERTY_HINT_RANGE, "0,100,0.1,or_greater,suffix:m"), "set_collision_range", "get_collision_range");
	ADD_GROUP("Look", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "smoke_density", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater"), "set_smoke_density", "get_smoke_density");
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "smoke_color", PROPERTY_HINT_COLOR_NO_ALPHA), "set_smoke_color", "get_smoke_color");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "smoke_ramp", PROPERTY_HINT_RESOURCE_TYPE, "Gradient"), "set_smoke_ramp", "get_smoke_ramp");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "heat_ramp", PROPERTY_HINT_RESOURCE_TYPE, "Gradient"), "set_heat_ramp", "get_heat_ramp");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "temperature_range", PROPERTY_HINT_RANGE, "0.01,20,0.01,or_greater"), "set_temperature_range", "get_temperature_range");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "emission_strength", PROPERTY_HINT_RANGE, "0,50,0.01,or_greater"), "set_emission_strength", "get_emission_strength");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "shadow_steps", PROPERTY_HINT_RANGE, "0,128,1"), "set_shadow_steps", "get_shadow_steps");
	ADD_GROUP("Queries", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "cpu_readback"), "set_cpu_readback", "is_cpu_readback");
}

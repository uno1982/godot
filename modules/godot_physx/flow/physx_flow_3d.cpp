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
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "core/object/object_id.h"
#include "core/os/os.h"
#include "core/templates/hash_set.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/light_3d.h"
#include "scene/main/window.h"
#include "servers/rendering/renderer_rd/renderer_scene_render_rd.h"
#include "servers/rendering/renderer_rd/storage_rd/light_storage.h"
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

// One queued step: what the main thread gathered, for the render thread to
// simulate with.
struct PhysXFlow3D::StepQueue {
	struct Step {
		double time = 0.0;
		float delta = 0.0f;
		bool readback = false;
		FlowBackend::Settings settings;
		LocalVector<FlowBackend::Emitter> emitters;
	};
	LocalVector<Step> queue;
};

PhysXFlow3D::PhysXFlow3D() {
	steps = memnew(StepQueue);
}

void PhysXFlow3D::_post(const Callable &p_call) {
	// The RenderingDevice may only be driven from the render thread. With
	// rendering on its own thread this queues the call in order with the
	// frame's other rendering commands; single-threaded it runs right here.
	RenderingServer::get_singleton()->call_on_render_thread(p_call);
}

void PhysXFlow3D::_rt_create(int p_max_blocks) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	FlowBackend::Grid *g = FlowBackend::grid_create_rd(rd, (uint32_t)p_max_blocks);
	MutexLock lock(step_mutex);
	grid = g;
	stat_start_ms = (OS::get_singleton()->get_ticks_usec() - t0) / 1000.0;
}

void PhysXFlow3D::_rt_destroy() {
	FlowBackend::Grid *g = nullptr;
	{
		MutexLock lock(step_mutex);
		g = grid;
		grid = nullptr;
		steps->queue.clear();
	}
	if (g != nullptr) {
		FlowBackend::grid_destroy(g);
	}
}

void PhysXFlow3D::_rt_step() {
	StepQueue::Step st;
	{
		MutexLock lock(step_mutex);
		if (steps->queue.is_empty() || grid == nullptr) {
			return;
		}
		st = steps->queue[0];
		steps->queue.remove_at(0);
	}
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	if (!FlowBackend::grid_step(grid, st.time, st.delta, st.settings, st.emitters.ptr(), (int)st.emitters.size())) {
		return;
	}
	const bool full = FlowBackend::grid_active_blocks(grid) >= (uint32_t)max_blocks;
	const double step_ms = (OS::get_singleton()->get_ticks_usec() - t0) / 1000.0;
	MutexLock lock(step_mutex);
	stat_step_ms = step_ms;
	blocks_full = blocks_full || full;
	if (st.readback) {
		_read_back();
	}
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
	memdelete(steps);
	steps = nullptr;
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
	_post(callable_mp(this, &PhysXFlow3D::_rt_create).bind(max_blocks));
	// Wait for it (one stall when a grid starts): everything below -- and the
	// render effect -- wants the grid there.
	RenderingServer::get_singleton()->sync();
	max_blocks_warned = false;
	blocks_full = false;
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
		// After the steps already queued; waited for, so the grid (and this
		// node, when it is being deleted) outlive every call that uses them.
		_post(callable_mp(this, &PhysXFlow3D::_rt_destroy));
		RenderingServer::get_singleton()->sync();
	}
	rd = nullptr;
	MutexLock lock(step_mutex);
	last_smoke.clear();
	last_temperature.clear();
	last_smoke_source = nullptr;
	gas_bounds = AABB();
}

void PhysXFlow3D::_fill_scatter_shadows(LocalVector<FlowScatterRD::Light> &r_lights, const PagedArray<RID> *p_instances, const RID &p_shadow_atlas, uint32_t p_flags) const {
	// Each light's shadow as Godot's renderer filled it for this frame
	// (LightStorage::update_light_buffers), rebuilt from world space: the
	// scattering pass works in world meters, not this view's camera space.
	RendererRD::LightStorage *ls = RendererRD::LightStorage::get_singleton();
	if (ls == nullptr || p_instances == nullptr || !p_shadow_atlas.is_valid() || !ls->owns_shadow_atlas(p_shadow_atlas)) {
		return;
	}
	for (FlowScatterRD::Light &l : r_lights) {
		// Light3D keeps its RenderingServer light to itself: find the frame's
		// instance of it by where it is and what it is.
		const bool spot = l.direction_cutoff[3] > -1.5f;
		if (!(p_flags & (spot ? SCATTERING_SHADOW_SPOT : SCATTERING_SHADOW_OMNI))) {
			continue; // this kind of light's shadows aren't received
		}
		const Vector3 at(l.position_range[0], l.position_range[1], l.position_range[2]);
		RID instance;
		RID base;
		for (uint64_t j = 0; j < p_instances->size(); j++) {
			const RID candidate = (*p_instances)[j];
			const RID candidate_base = ls->light_instance_get_base_light(candidate);
			const RSE::LightType type = ls->light_get_type(candidate_base);
			if (type == (spot ? RSE::LIGHT_SPOT : RSE::LIGHT_OMNI) && ls->light_instance_get_base_transform(candidate).origin.distance_squared_to(at) < 1e-6f) {
				instance = candidate;
				base = candidate_base;
				break;
			}
		}
		if (!instance.is_valid() || !ls->light_has_shadow(base) || !ls->shadow_atlas_owns_light_instance(p_shadow_atlas, instance)) {
			continue;
		}
		Vector2i omni_offset;
		const Rect2 rect = ls->light_instance_get_shadow_atlas_rect(instance, p_shadow_atlas, omni_offset);
		const Transform3D light_xf = ls->light_instance_get_base_transform(instance);
		Projection m;
		float bias = ls->light_get_param(base, RSE::LIGHT_PARAM_SHADOW_BIAS);
		if (spot) {
			Projection light_bias;
			light_bias.set_light_bias();
			Projection correction;
			correction.set_depth_correction(false, true, false);
			m = light_bias * correction * ls->light_instance_get_shadow_camera(instance, 0) * Projection(light_xf.affine_inverse());
			bias = bias / 100.0f * ls->light_get_param(base, RSE::LIGHT_PARAM_SHADOW_BLUR) * RendererSceneRenderRD::get_singleton()->shadows_quality_radius_get();
		} else {
			m = Projection(light_xf.affine_inverse());
			l.shadow[2] = omni_offset.x * rect.size.width;
			l.shadow[3] = omni_offset.y * rect.size.height;
		}
		for (int c = 0; c < 4; c++) {
			for (int r = 0; r < 4; r++) {
				l.shadow_matrix[c * 4 + r] = (float)m.columns[c][r];
			}
		}
		l.atlas_rect[0] = rect.position.x;
		l.atlas_rect[1] = rect.position.y;
		l.atlas_rect[2] = rect.size.width;
		l.atlas_rect[3] = rect.size.height;
		l.shadow[0] = ls->light_get_param(base, RSE::LIGHT_PARAM_SHADOW_OPACITY);
		l.shadow[1] = bias;
	}
}

void PhysXFlow3D::render_rd(PhysXFlowRenderEffect *p_effect, RenderingDevice *p_rd, const float p_view[16], const float p_projection[16], const Size2i &p_size, const RID &p_depth, const RID &p_color, const Projection &p_inv_view_projection, const Vector3 &p_camera, const PagedArray<RID> *p_lights, const RID &p_shadow_atlas, bool p_first_view, bool p_last_view) {
	if (grid == nullptr || p_rd != rd) {
		return;
	}
	LocalVector<FlowScatterRD::Light> lights;
	if (light_scattering) {
		MutexLock lock(scatter_mutex);
		lights = scatter_lights;
	}
	const uint32_t receive = scattering_receive_shadows;
	if (receive != 0 && !lights.is_empty()) {
		_fill_scatter_shadows(lights, p_lights, p_shadow_atlas, receive);
	}
	FlowBackend::ScatterSource source;
	const uint64_t out = FlowBackend::grid_render_rd(grid, p_view, p_projection, p_size.x, p_size.y, p_depth.get_id(), p_size.x, p_size.y, p_color.get_id(), lights.is_empty() ? nullptr : &source, p_first_view);
	stat_render_calls++;
	stat_render_outputs += out != 0 ? 1 : 0;
	if (out != 0 && out != p_color.get_id()) {
		p_effect->copy_texture(p_rd, RID::from_uint64(out), p_color, p_size);
	}
	if (!lights.is_empty() && !source.valid) {
		stat_scatter_no_source++;
	}
	if (out != 0 && source.valid) {
		stat_scatter_passes++;
		// Before the Flow frame ends: its volume and table are this frame's.
		FlowScatterRD::Params params;
		params.inv_view_projection = p_inv_view_projection;
		params.camera_position = p_camera;
		params.sigma = smoke_density;
		const Color albedo = smoke_color.srgb_to_linear();
		params.color[0] = albedo.r;
		params.color[1] = albedo.g;
		params.color[2] = albedo.b;
		params.strength = scattering_strength;
		params.anisotropy = scattering_anisotropy;
		params.shadow_steps = scattering_shadow_steps;
		params.shadow_reach = scattering_shadow_reach;
		params.shadows = receive != 0;
		params.resolution_divisor = scattering_resolution == SCATTERING_RESOLUTION_FULL ? 1 : (scattering_resolution == SCATTERING_RESOLUTION_HALF ? 2 : 4);
		RID atlas;
		if (receive != 0 && p_shadow_atlas.is_valid() && RendererRD::LightStorage::get_singleton()->owns_shadow_atlas(p_shadow_atlas)) {
			atlas = RendererRD::LightStorage::get_singleton()->shadow_atlas_get_texture(p_shadow_atlas);
		}
		p_effect->get_scatter().run(p_rd, source, params, lights.ptr(), (int)lights.size(), p_depth, p_color, atlas, p_size);
	}
	if (p_last_view) {
		FlowBackend::grid_render_rd_finish(grid);
	}
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
		fe.allocate = e->is_allocating();
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
		// The body's velocity where this shape is, spin included: a shape on a
		// spinning or swinging body (a rotor, a door, a walker on a turntable)
		// moves although the body's centre may not.
		Vector3 velocity = ps->body_get_state(body, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY);
		if (PhysicsDirectBodyState3D *state = ps->body_get_direct_state(body)) {
			velocity = state->get_velocity_at_local_position(xform.origin - body_xform.origin);
		}
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

// The scene's lights, for the flows to look theirs up in every second or so: found by one walk of the tree when the
// first flow enters it, then kept from the SceneTree's node_added / node_removed while any flow is in it. (A walk of
// the whole scene per lookup, per flow, cost tens of milliseconds in scenes of a few hundred thousand nodes.)
static ObjectID flow_lights_tree;
static int flow_lights_users = 0;
static HashSet<ObjectID> flow_lights;

static void _flow_light_added(Node *p_node) {
	if (Object::cast_to<Light3D>(p_node) != nullptr) {
		flow_lights.insert(p_node->get_instance_id());
	}
}

static void _flow_light_removed(Node *p_node) {
	if (Object::cast_to<Light3D>(p_node) != nullptr) {
		flow_lights.erase(p_node->get_instance_id());
	}
}

static void _flow_lights_watch(SceneTree *p_tree) {
	if (flow_lights_users++ > 0) {
		return;
	}
	flow_lights_tree = p_tree->get_instance_id();
	p_tree->connect(SNAME("node_added"), callable_mp_static(&_flow_light_added));
	p_tree->connect(SNAME("node_removed"), callable_mp_static(&_flow_light_removed));
	TypedArray<Node> found = p_tree->get_root()->find_children("*", "Light3D", true, false);
	for (int i = 0; i < found.size(); i++) {
		flow_lights.insert(Object::cast_to<Node>(found[i])->get_instance_id());
	}
}

static void _flow_lights_unwatch() {
	if (--flow_lights_users > 0) {
		return;
	}
	SceneTree *tree = Object::cast_to<SceneTree>(ObjectDB::get_instance(flow_lights_tree));
	if (tree != nullptr) {
		tree->disconnect(SNAME("node_added"), callable_mp_static(&_flow_light_added));
		tree->disconnect(SNAME("node_removed"), callable_mp_static(&_flow_light_removed));
	}
	flow_lights_tree = ObjectID();
	flow_lights.clear();
}

// The lights of class p_class under p_root, in the order p_root->find_children() would list them (tree order).
static void _flow_lights_under(const Node *p_root, const char *p_class, LocalVector<Node *> &r_lights) {
	r_lights.clear();
	for (const ObjectID &id : flow_lights) {
		Node *n = Object::cast_to<Node>(ObjectDB::get_instance(id));
		if (n != nullptr && n->is_inside_tree() && n->is_class(p_class) && p_root->is_ancestor_of(n)) {
			r_lights.push_back(n);
		}
	}
	struct TreeOrder {
		bool operator()(const Node *p_a, const Node *p_b) const { return p_b->is_greater_than(p_a); }
	};
	r_lights.sort_custom<TreeOrder>();
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
		if (root == nullptr) {
			root = get_tree()->get_root(); // a scene built from a script has no current scene
		}
		if (root != nullptr) {
			LocalVector<Node *> lights;
			_flow_lights_under(root, "DirectionalLight3D", lights);
			for (uint32_t i = 0; i < lights.size(); i++) {
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

void PhysXFlow3D::_gather_scatter_lights() {
	LocalVector<FlowScatterRD::Light> picked;
	if (light_scattering) {
		if (scatter_search_countdown-- <= 0) {
			scatter_search_countdown = 60;
			scatter_light_ids.clear();
			Node *root = get_tree()->get_edited_scene_root();
			if (root == nullptr) {
				root = get_tree()->get_current_scene();
			}
			if (root == nullptr) {
				root = get_tree()->get_root(); // a scene built from a script has no current scene
			}
			if (root != nullptr) {
				for (const char *type : { "OmniLight3D", "SpotLight3D" }) {
					LocalVector<Node *> found;
					_flow_lights_under(root, type, found);
					for (Node *l : found) {
						scatter_light_ids.push_back(l->get_instance_id());
					}
				}
			}
		}
		// The lights that can reach the gas, strongest at it first (energy over
		// squared distance to where the gas is, roughly).
		struct Candidate {
			FlowScatterRD::Light light;
			float score = 0.0f;
		};
		LocalVector<Candidate> candidates;
		constexpr float GAS_REACH = 20.0f; // how far gas may have spread from draw_center
		for (const ObjectID &id : scatter_light_ids) {
			Light3D *l = Object::cast_to<Light3D>(ObjectDB::get_instance(id));
			if (l == nullptr || !l->is_inside_tree() || !l->is_visible_in_tree()) {
				continue;
			}
			// Light3D.light_volumetric_fog_energy: the same per-light knob Godot's
			// volumetric fog uses (0 leaves the gas dark).
			const float energy = l->get_param(Light3D::PARAM_ENERGY) * l->get_param(Light3D::PARAM_VOLUMETRIC_FOG_ENERGY);
			const float range = l->get_param(Light3D::PARAM_RANGE);
			const Transform3D xf = l->get_global_transform();
			const float d = xf.origin.distance_to(draw_center);
			if (energy <= 0.0f || range <= 0.0f || d > range + GAS_REACH) {
				continue;
			}
			Candidate c;
			const Color col = l->get_color().srgb_to_linear() * energy;
			c.light.position_range[0] = xf.origin.x;
			c.light.position_range[1] = xf.origin.y;
			c.light.position_range[2] = xf.origin.z;
			c.light.position_range[3] = range;
			c.light.color_decay[0] = col.r;
			c.light.color_decay[1] = col.g;
			c.light.color_decay[2] = col.b;
			c.light.color_decay[3] = l->get_param(Light3D::PARAM_ATTENUATION);
			c.light.direction_cutoff[3] = -2.0f; // omni
			if (Object::cast_to<SpotLight3D>(l) != nullptr) {
				// A spot shines along its -Z.
				const Vector3 dir = -xf.basis.get_column(2).normalized();
				c.light.direction_cutoff[0] = dir.x;
				c.light.direction_cutoff[1] = dir.y;
				c.light.direction_cutoff[2] = dir.z;
				c.light.direction_cutoff[3] = Math::cos(Math::deg_to_rad(l->get_param(Light3D::PARAM_SPOT_ANGLE)));
				c.light.spot[0] = l->get_param(Light3D::PARAM_SPOT_ATTENUATION);
			}
			c.score = energy / MAX(d * d, 1.0f);
			candidates.push_back(c);
		}
		for (uint32_t i = 1; i < candidates.size(); i++) {
			for (uint32_t j = i; j > 0 && candidates[j - 1].score < candidates[j].score; j--) {
				SWAP(candidates[j - 1], candidates[j]);
			}
		}
		for (uint32_t i = 0; i < candidates.size() && (int)i < scattering_max_lights; i++) {
			picked.push_back(candidates[i].light);
		}
	}
	stat_scatter_lights = (int)picked.size();
	MutexLock lock(scatter_mutex);
	scatter_lights = picked;
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
	AABB gas;
	bool full = false;
	{
		MutexLock lock(step_mutex);
		gas = gas_bounds;
		full = blocks_full;
	}
	if (gas.has_volume()) {
		region = region.has_volume() ? region.merge(gas.grow(1.0)) : gas.grow(1.0);
	}
	_gather_colliders(region, emitters);
	_gather_scatter_lights();
	if (floor_enabled) {
		// The FLOOR: one wide solid box whose top is floor_height, under everything this grid can reach. A solid
		// reserves no blocks and holds no gas, so whatever sinks below the floor is gone (and its blocks with it).
		constexpr float FLOOR_HALF_W = 4096.0f;
		constexpr float FLOOR_HALF_T = 64.0f;
		FlowBackend::Emitter floor_e = make_solid(Transform3D(Basis(), Vector3(draw_center.x, floor_height - FLOOR_HALF_T, draw_center.z)), Vector3());
		floor_e.shape = FlowBackend::Emitter::SHAPE_BOX;
		floor_e.half_size[0] = FLOOR_HALF_W;
		floor_e.half_size[1] = FLOOR_HALF_T;
		floor_e.half_size[2] = FLOOR_HALF_W;
		emitters.push_back(floor_e);
	}

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

	{
		MutexLock lock(step_mutex);
		steps->queue.push_back(StepQueue::Step());
		StepQueue::Step &q = steps->queue[steps->queue.size() - 1];
		q.time = sim_time + p_delta;
		q.delta = (float)p_delta;
		q.readback = cpu_readback;
		q.settings = settings;
		q.emitters = emitters;
	}
	_post(callable_mp(this, &PhysXFlow3D::_rt_step));
	sim_time += p_delta;
	if (!max_blocks_warned && full) {
		max_blocks_warned = true;
		WARN_PRINT(vformat("PhysXFlow3D %s: the gas has used all %d max_blocks and is being cut off; raise max_blocks (more GPU memory) or make the effect smaller.", get_path(), max_blocks));
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
	// (Render thread, step_mutex held.) Copied: the mapped readback is the
	// GPU's to overwrite a few frames on, while the main thread samples it.
	stat_readback_bytes = (int64_t)(temp_size + smoke_size);
	if (smoke == last_smoke_source) {
		return; // the same readback as last step -- the newer one isn't back yet
	}
	last_smoke_source = smoke;
	last_smoke.resize(smoke != nullptr ? (int64_t)smoke_size : 0);
	if (smoke != nullptr && smoke_size > 0) {
		memcpy(last_smoke.ptrw(), smoke, smoke_size);
	}
	last_temperature.resize(temp != nullptr ? (int64_t)temp_size : 0);
	if (temp != nullptr && temp_size > 0) {
		memcpy(last_temperature.ptrw(), temp, temp_size);
	}
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
		MutexLock lock(step_mutex);
		last_smoke.clear();
		last_temperature.clear();
		last_smoke_source = nullptr;
		gas_bounds = AABB();
		stat_readback_bytes = 0;
	}
}

float PhysXFlow3D::_sample_field(const Vector<uint8_t> &p_field, const Vector3 &p_position) const {
	MutexLock lock(step_mutex);
	if (!cpu_readback || p_field.is_empty()) {
		return -1.0f;
	}
	const float origin[3] = { (float)p_position.x - 0.5f * cell_size, (float)p_position.y - 0.5f * cell_size, (float)p_position.z - 0.5f * cell_size };
	const int dims[3] = { 1, 1, 1 };
	float value = 0.0f;
	FlowBackend::sample_dense(p_field.ptr(), (uint64_t)p_field.size(), origin, cell_size, dims, 0, 1, &value, 1);
	return value;
}

float PhysXFlow3D::sample_smoke(const Vector3 &p_position) const {
	return _sample_field(last_smoke, p_position);
}

float PhysXFlow3D::sample_temperature(const Vector3 &p_position) const {
	return _sample_field(last_temperature, p_position);
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
	{
		MutexLock lock(step_mutex);
		d["bounds"] = gas_bounds;
	}
	d["scatter_lights"] = stat_scatter_lights;
	d["scatter_passes"] = stat_scatter_passes;
	d["scatter_no_source"] = stat_scatter_no_source;
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
				_flow_lights_watch(get_tree());
			} else {
				remove_from_group(GROUP);
				_flow_lights_unwatch();
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
	ClassDB::bind_method(D_METHOD("set_floor_enabled", "enabled"), &PhysXFlow3D::set_floor_enabled);
	ClassDB::bind_method(D_METHOD("is_floor_enabled"), &PhysXFlow3D::is_floor_enabled);
	ClassDB::bind_method(D_METHOD("set_floor_height", "height"), &PhysXFlow3D::set_floor_height);
	ClassDB::bind_method(D_METHOD("get_floor_height"), &PhysXFlow3D::get_floor_height);
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
	ClassDB::bind_method(D_METHOD("set_light_scattering", "enabled"), &PhysXFlow3D::set_light_scattering);
	ClassDB::bind_method(D_METHOD("is_light_scattering"), &PhysXFlow3D::is_light_scattering);
	ClassDB::bind_method(D_METHOD("set_scattering_max_lights", "count"), &PhysXFlow3D::set_scattering_max_lights);
	ClassDB::bind_method(D_METHOD("get_scattering_max_lights"), &PhysXFlow3D::get_scattering_max_lights);
	ClassDB::bind_method(D_METHOD("set_scattering_strength", "strength"), &PhysXFlow3D::set_scattering_strength);
	ClassDB::bind_method(D_METHOD("get_scattering_strength"), &PhysXFlow3D::get_scattering_strength);
	ClassDB::bind_method(D_METHOD("set_scattering_anisotropy", "anisotropy"), &PhysXFlow3D::set_scattering_anisotropy);
	ClassDB::bind_method(D_METHOD("get_scattering_anisotropy"), &PhysXFlow3D::get_scattering_anisotropy);
	ClassDB::bind_method(D_METHOD("set_scattering_shadow_steps", "steps"), &PhysXFlow3D::set_scattering_shadow_steps);
	ClassDB::bind_method(D_METHOD("get_scattering_shadow_steps"), &PhysXFlow3D::get_scattering_shadow_steps);
	ClassDB::bind_method(D_METHOD("set_scattering_resolution", "resolution"), &PhysXFlow3D::set_scattering_resolution);
	ClassDB::bind_method(D_METHOD("get_scattering_resolution"), &PhysXFlow3D::get_scattering_resolution);
	ClassDB::bind_method(D_METHOD("set_scattering_receive_shadows", "flags"), &PhysXFlow3D::set_scattering_receive_shadows);
	ClassDB::bind_method(D_METHOD("get_scattering_receive_shadows"), &PhysXFlow3D::get_scattering_receive_shadows);
	ClassDB::bind_method(D_METHOD("set_scattering_shadow_reach", "reach"), &PhysXFlow3D::set_scattering_shadow_reach);
	ClassDB::bind_method(D_METHOD("get_scattering_shadow_reach"), &PhysXFlow3D::get_scattering_shadow_reach);
	ClassDB::bind_method(D_METHOD("set_max_blocks", "blocks"), &PhysXFlow3D::set_max_blocks);
	ClassDB::bind_method(D_METHOD("get_max_blocks"), &PhysXFlow3D::get_max_blocks);
	ClassDB::bind_method(D_METHOD("set_cpu_readback", "enabled"), &PhysXFlow3D::set_cpu_readback);
	ClassDB::bind_method(D_METHOD("is_cpu_readback"), &PhysXFlow3D::is_cpu_readback);
	ClassDB::bind_method(D_METHOD("sample_smoke", "position"), &PhysXFlow3D::sample_smoke);
	ClassDB::bind_method(D_METHOD("sample_temperature", "position"), &PhysXFlow3D::sample_temperature);
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
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "floor_enabled"), "set_floor_enabled", "is_floor_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "floor_height", PROPERTY_HINT_NONE, "suffix:m"), "set_floor_height", "get_floor_height");
	ADD_GROUP("Look", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "smoke_density", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater"), "set_smoke_density", "get_smoke_density");
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "smoke_color", PROPERTY_HINT_COLOR_NO_ALPHA), "set_smoke_color", "get_smoke_color");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "smoke_ramp", PROPERTY_HINT_RESOURCE_TYPE, "Gradient"), "set_smoke_ramp", "get_smoke_ramp");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "heat_ramp", PROPERTY_HINT_RESOURCE_TYPE, "Gradient"), "set_heat_ramp", "get_heat_ramp");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "temperature_range", PROPERTY_HINT_RANGE, "0.01,20,0.01,or_greater"), "set_temperature_range", "get_temperature_range");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "emission_strength", PROPERTY_HINT_RANGE, "0,50,0.01,or_greater"), "set_emission_strength", "get_emission_strength");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "shadow_steps", PROPERTY_HINT_RANGE, "0,128,1"), "set_shadow_steps", "get_shadow_steps");
	ADD_GROUP("Light Scattering", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "light_scattering"), "set_light_scattering", "is_light_scattering");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "scattering_max_lights", PROPERTY_HINT_RANGE, vformat("1,%d,1", FlowScatterRD::MAX_LIGHTS)), "set_scattering_max_lights", "get_scattering_max_lights");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "scattering_strength", PROPERTY_HINT_RANGE, "0,8,0.01,or_greater"), "set_scattering_strength", "get_scattering_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "scattering_anisotropy", PROPERTY_HINT_RANGE, "-0.95,0.95,0.01"), "set_scattering_anisotropy", "get_scattering_anisotropy");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "scattering_shadow_steps", PROPERTY_HINT_RANGE, "0,64,1"), "set_scattering_shadow_steps", "get_scattering_shadow_steps");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "scattering_receive_shadows", PROPERTY_HINT_FLAGS, "Omni:1,Spot:2"), "set_scattering_receive_shadows", "get_scattering_receive_shadows");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "scattering_resolution", PROPERTY_HINT_ENUM, "Full,Half,Quarter"), "set_scattering_resolution", "get_scattering_resolution");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "scattering_shadow_reach", PROPERTY_HINT_RANGE, "0.05,20,0.05,or_greater,suffix:m"), "set_scattering_shadow_reach", "get_scattering_shadow_reach");
	BIND_ENUM_CONSTANT(SCATTERING_RESOLUTION_FULL);
	BIND_ENUM_CONSTANT(SCATTERING_RESOLUTION_HALF);
	BIND_ENUM_CONSTANT(SCATTERING_RESOLUTION_QUARTER);
	BIND_BITFIELD_FLAG(SCATTERING_SHADOW_OMNI);
	BIND_BITFIELD_FLAG(SCATTERING_SHADOW_SPOT);
	ADD_GROUP("Queries", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "cpu_readback"), "set_cpu_readback", "is_cpu_readback");
}

/**************************************************************************/
/*  godot_physx_soft_body_3d.cpp                                          */
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

#include "godot_physx_soft_body_3d.h"

#include "../godot_physx_project_settings.h"
#include "../spaces/godot_physx_direct_state_3d.h"
#include "../spaces/godot_physx_space_3d.h"

#include "core/config/project_settings.h"
#include "core/object/object.h"
#include "core/templates/hash_map.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/rendering/rendering_server.h"
#include "servers/rendering/rendering_server_enums.h"

GodotPhysXSoftBody3D::~GodotPhysXSoftBody3D() {
	set_space(nullptr);
	if (volume) {
		memdelete(volume);
	}
	if (collision_sphere.is_valid()) {
		PhysicsServer3D::get_singleton()->free_rid(collision_sphere);
	}
}

void GodotPhysXSoftBody3D::set_space(GodotPhysXSpace3D *p_space) {
	if (space == p_space) {
		return;
	}
	if (space) {
		space->unregister_soft_body(this);
	}
	// The GPU volume is bound to a scene; drop it on space change and let the
	// next set_mesh / rebuild decide the path again.
	if (volume) {
		memdelete(volume);
		volume = nullptr;
		using_gpu = false;
	}
	space = p_space;
	if (space) {
		space->register_soft_body(this);
		if (mesh_ready) {
			_rebuild_from_mesh();
		}
	}
}

void GodotPhysXSoftBody3D::_apply_solver_settings() {
	if (using_gpu && volume) {
		volume->apply_params(_gpu_params());
		return;
	}
	XPBDClothSolver::Settings s = solver.settings;
	s.substeps = MAX(simulation_precision, 1);
	// linear_stiffness in [0,1] -> stretch compliance: 1 is rigid (0 compliance),
	// lower is springier. Shear/bend ride along, softer.
	const float k = CLAMP((float)linear_stiffness, 0.0f, 1.0f);
	s.stretch_compliance = (1.0f - k) * 1.0e-3f;
	s.shear_compliance = MAX(s.stretch_compliance, 2.0e-4f);
	s.bend_compliance = 1.0e-3f + (1.0f - k) * 4.0e-3f;
	s.damping = MAX((float)damping_coefficient, 0.0f);
	s.drag = MAX((float)drag_coefficient, 0.0f);
	s.lift = 0.0f;
	// Cap per-vertex speed so a fast body can't outrun the once-per-frame
	// contact refresh and tunnel through thin geometry.
	s.max_speed = 12.0f;
	// pressure_coefficient (Godot: ~0..100, 0 = off) preserves the rest volume;
	// larger values just make that preservation stiffer, not bigger.
	if (pressure_coefficient > CMP_EPSILON) {
		s.pressure = 1.0f; // hold the rest volume
		s.pressure_stiffness = CLAMP((float)pressure_coefficient / 60.0f, 0.05f, 1.0f);
	} else {
		s.pressure = 0.0f;
	}
	if (mesh_ready) {
		const float area = solver.rest_surface_area();
		s.density = area > CMP_EPSILON ? MAX((float)total_mass, 0.001f) / area : 0.2f;
	}
	solver.settings = s;
}

void GodotPhysXSoftBody3D::_rebuild_from_mesh(bool p_keep_state) {
	const bool was_running = p_keep_state && mesh_ready && placed;
	Vector<Vector3> prev_positions;
	Vector<Vector3> prev_velocities;
	if (was_running) {
		const LocalVector<Vector3> &sp = solver.get_positions();
		const LocalVector<Vector3> &sv = solver.get_velocities();
		prev_positions.resize(sp.size());
		prev_velocities.resize(sv.size());
		for (uint32_t i = 0; i < sp.size(); i++) {
			prev_positions.write[i] = sp[i];
			prev_velocities.write[i] = sv[i];
		}
	}

	mesh_ready = false;
	map_visual_to_physics.clear();
	visual_vertex_count = 0;
	solver.clear();
	if (volume) {
		memdelete(volume);
		volume = nullptr;
	}
	using_gpu = false;

	if (mesh.is_null()) {
		return;
	}

	Array arrays = RenderingServer::get_singleton()->mesh_surface_get_arrays(mesh, 0);
	if (arrays.is_empty()) {
		return;
	}
	const Vector<Vector3> vertices = arrays[RSE::ARRAY_VERTEX];
	const Vector<int> indices = arrays[RSE::ARRAY_INDEX];
	ERR_FAIL_COND_MSG(vertices.is_empty(), "Soft body mesh needs vertices.");
	ERR_FAIL_COND_MSG(indices.is_empty(), "Soft body mesh needs indices.");

	visual_vertex_count = vertices.size();
	map_visual_to_physics.resize(visual_vertex_count);

	// Weld coincident render vertices (UV/normal seams) to one solver vertex.
	HashMap<Vector3, uint32_t> unique;
	PackedVector3Array welded;
	for (uint32_t i = 0; i < visual_vertex_count; i++) {
		const Vector3 &v = vertices[i];
		HashMap<Vector3, uint32_t>::Iterator e = unique.find(v);
		uint32_t id;
		if (e) {
			id = e->value;
		} else {
			id = welded.size();
			unique[v] = id;
			welded.push_back(v);
		}
		map_visual_to_physics[i] = id;
	}

	PackedInt32Array welded_indices;
	welded_indices.resize(indices.size());
	for (int i = 0; i < indices.size(); i++) {
		welded_indices.write[i] = (int32_t)map_visual_to_physics[indices[i]];
	}

	// GPU path first (per-body Auto): a PxDeformableVolume if the mesh cooks and
	// CUDA is up. Only once the node has handed us its world transform -- a
	// volume is a live scene actor the moment it's added, so building it at the
	// origin would have it eject off the floor. Before that, fall through to the
	// CPU solver (which is gated in step()); set_transform() rebuilds.
	if (placed && _try_build_gpu(welded, welded_indices)) {
		using_gpu = true;
		mesh_ready = true;
		_sync_gpu_pins(); // re-apply any pins set before the volume existed
		return;
	}

	solver.build_mesh(welded, welded_indices);
	if (!solver.is_built()) {
		return;
	}
	mesh_ready = true;
	normals.resize(solver.vertex_count());

	// Re-apply pins requested before the mesh was available (scene load order,
	// deferred pin_point calls) -- matches GodotPhysics/Jolt behavior.
	for (const int rp : pinned_render_points) {
		if (rp >= 0 && rp < (int)visual_vertex_count) {
			solver.set_pinned((int)map_visual_to_physics[rp], true);
		}
	}

	_apply_solver_settings();
	solver.set_rest_length_scale(1.0f - CLAMP((float)shrinking_factor, 0.0f, 0.99f));
	solver.reset(transform);
	if (was_running && prev_positions.size() == solver.vertex_count()) {
		// Same geometry, just a mesh-handle swap -- carry the live state over.
		LocalVector<Vector3> &p = solver.positions_mut();
		LocalVector<Vector3> &v = solver.velocities_mut();
		for (int i = 0; i < prev_positions.size(); i++) {
			p[i] = prev_positions[i];
			v[i] = prev_velocities[i];
		}
	}
	_update_normals_and_bounds();
}

GodotPhysXSoftVolume3D::Params GodotPhysXSoftBody3D::_gpu_params() const {
	GodotPhysXSoftVolume3D::Params p;
	const float k = CLAMP((float)linear_stiffness, 0.0f, 1.0f);
	p.youngs_modulus = Math::lerp(2.0e4f, 5.0e7f, k * k);
	// pressure_coefficient (0..~100) -> Poisson ratio toward the 0.49 near-
	// incompressible limit; 0 leaves it a soft, compressible blob.
	p.poisson_ratio = CLAMP(0.30f + (float)pressure_coefficient / 100.0f * 0.19f, 0.05f, 0.49f);
	p.damping = MAX((float)damping_coefficient, 0.0f);
	p.dynamic_friction = 0.5f;
	p.total_mass = MAX((float)total_mass, 0.001f);
	p.solver_iterations = CLAMP(simulation_precision, 4, 60);
	p.max_speed = 30.0f;
	p.collision_layer = collision_layer;
	p.collision_mask = collision_mask;
	return p;
}

bool GodotPhysXSoftBody3D::_try_build_gpu(const PackedVector3Array &p_welded, const PackedInt32Array &p_indices) {
	if (!space || !space->get_px_cuda()) {
		return false;
	}
	// Mode resolution: node metadata "physx_soft_mode" overrides the project
	// setting; both can force CPU or GPU, otherwise it's Auto (try GPU). Read
	// the setting live so a runtime change (e.g. a test or a demo toggle) takes
	// effect on the next rebuild.
	int mode = 0; // 0 Auto, 1 CPU, 2 GPU
	if (ProjectSettings *ps = ProjectSettings::get_singleton()) {
		const Variant v = ps->get("physics/physx_3d/soft_body/mode");
		if (v.get_type() == Variant::INT) {
			mode = (int)v;
		}
	}
	if (instance_id.is_valid()) {
		Object *obj = ObjectDB::get_instance(instance_id);
		if (obj && obj->has_meta("physx_soft_mode")) {
			const String m = String(obj->get_meta("physx_soft_mode")).to_lower();
			if (m == "cpu") {
				mode = 1;
			} else if (m == "gpu") {
				mode = 2;
			}
		}
	}
	if (mode == 1) {
		return false;
	}

	Vector<Vector3> wv;
	wv.resize(p_welded.size());
	for (int i = 0; i < p_welded.size(); i++) {
		wv.write[i] = p_welded[i];
	}
	Vector<int32_t> idx;
	idx.resize(p_indices.size());
	for (int i = 0; i < p_indices.size(); i++) {
		idx.write[i] = p_indices[i];
	}

	GodotPhysXSoftVolume3D *v = memnew(GodotPhysXSoftVolume3D);
	if (!v->build(space, wv, idx, transform, _gpu_params())) {
		memdelete(v);
		if (mode == 2) {
			ERR_PRINT("PhysX: SoftBody3D forced to GPU but the mesh could not be tetrahedralized; it will be inert.");
		}
		return false;
	}
	volume = v;
	print_verbose("PhysX: SoftBody3D -> GPU PxDeformableVolume.");
	if (GodotPhysXProjectSettings::solver_type != 1) { // not TGS
		static bool warned = false;
		if (!warned) {
			warned = true;
			WARN_PRINT("PhysX: GPU soft bodies (PxDeformableVolume) resolve soft-vs-soft contact reliably only under "
					   "the TGS solver. On PGS the contact is soft and transient -- a pile settles but slowly compacts "
					   "as bodies sink through each other. Set physics/physx_3d/simulation/solver_type = TGS for firm "
					   "soft-vs-soft stacking. Soft-vs-rigid is unaffected.");
		}
	}
	return true;
}

void GodotPhysXSoftBody3D::set_mesh(RID p_mesh) {
	if (p_mesh == mesh) {
		return;
	}
	// The node swaps in a private duplicate of the mesh on its first draw. Same
	// geometry, so keep the running sim rather than snapping back to the rest
	// pose -- _rebuild_from_mesh() preserves positions when the vertex count is
	// unchanged.
	mesh = p_mesh;
	_rebuild_from_mesh(true);
}

void GodotPhysXSoftBody3D::set_transform(const Transform3D &p_transform) {
	if (placed) {
		// After its first placement the node makes itself top-level and re-sends
		// its transform as identity -- that is bookkeeping, not a move. Only a
		// real, different transform (an explicit runtime reposition) re-seeds
		// the body.
		if (p_transform.is_equal_approx(Transform3D()) || p_transform.is_equal_approx(transform)) {
			return;
		}
	}
	const bool first_placement = !placed;
	transform = p_transform;
	placed = true;
	if (mesh_ready && (using_gpu || first_placement)) {
		// GPU: the volume bakes its placement into the cooked vertices, so any
		// move is a rebuild. First real placement: rebuild too, so the GPU path
		// (which was skipped while unplaced) now gets its shot.
		_rebuild_from_mesh();
	} else if (mesh_ready) {
		solver.reset(transform);
		_update_normals_and_bounds();
	}
}

void GodotPhysXSoftBody3D::set_simulation_precision(int p_precision) {
	simulation_precision = MAX(p_precision, 1);
	_apply_solver_settings();
}

void GodotPhysXSoftBody3D::set_total_mass(real_t p_mass) {
	total_mass = MAX(p_mass, 0.001);
	_apply_solver_settings();
}

void GodotPhysXSoftBody3D::set_linear_stiffness(real_t p_stiffness) {
	linear_stiffness = CLAMP(p_stiffness, 0.0, 1.0);
	_apply_solver_settings();
}

void GodotPhysXSoftBody3D::set_shrinking_factor(real_t p_factor) {
	shrinking_factor = p_factor;
	if (mesh_ready) {
		solver.set_rest_length_scale(1.0f - CLAMP((float)shrinking_factor, 0.0f, 0.99f));
	}
}

void GodotPhysXSoftBody3D::set_pressure_coefficient(real_t p_pressure) {
	pressure_coefficient = MAX(p_pressure, 0.0);
	_apply_solver_settings();
}

void GodotPhysXSoftBody3D::set_damping_coefficient(real_t p_damping) {
	damping_coefficient = MAX(p_damping, 0.0);
	_apply_solver_settings();
}

void GodotPhysXSoftBody3D::set_drag_coefficient(real_t p_drag) {
	drag_coefficient = MAX(p_drag, 0.0);
	_apply_solver_settings();
}

void GodotPhysXSoftBody3D::_sync_gpu_pins() {
	if (!using_gpu || !volume) {
		return;
	}
	Vector<int> idx;
	Vector<Vector3> targets;
	for (const int rp : pinned_render_points) {
		idx.push_back(rp);
		const HashMap<int, Vector3>::ConstIterator t = pin_targets.find(rp);
		targets.push_back(t ? t->value : Vector3(NAN, 0, 0));
	}
	volume->set_pins(idx, targets);
}

void GodotPhysXSoftBody3D::move_point(int p_point_index, const Vector3 &p_global_position) {
	if (p_point_index < 0) {
		return;
	}
	pinned_render_points.insert(p_point_index); // a moved point is a pinned point
	pin_targets[p_point_index] = p_global_position;
	if (using_gpu) {
		_sync_gpu_pins();
	} else if (mesh_ready && p_point_index < (int)visual_vertex_count) {
		solver.set_pin_target((int)map_visual_to_physics[p_point_index], p_global_position);
	}
}

Vector3 GodotPhysXSoftBody3D::get_point_global_position(int p_point_index) const {
	ERR_FAIL_INDEX_V(p_point_index, (int)visual_vertex_count, Vector3());
	const uint32_t v = map_visual_to_physics[p_point_index];
	if (using_gpu && volume) {
		return volume->get_vertex_position(v);
	}
	return solver.get_positions()[v];
}

void GodotPhysXSoftBody3D::pin_point(int p_point_index, bool p_pin) {
	if (p_point_index < 0) {
		return;
	}
	if (p_pin) {
		pinned_render_points.insert(p_point_index);
	} else {
		pinned_render_points.erase(p_point_index);
		pin_targets.erase(p_point_index);
	}
	if (using_gpu) {
		_sync_gpu_pins();
	} else if (mesh_ready && p_point_index < (int)visual_vertex_count) {
		solver.set_pinned((int)map_visual_to_physics[p_point_index], p_pin);
	}
}

bool GodotPhysXSoftBody3D::is_point_pinned(int p_point_index) const {
	if (mesh_ready && p_point_index >= 0 && p_point_index < (int)visual_vertex_count) {
		return solver.is_pinned((int)map_visual_to_physics[p_point_index]);
	}
	return pinned_render_points.has(p_point_index);
}

void GodotPhysXSoftBody3D::unpin_all() {
	pinned_render_points.clear();
	pin_targets.clear();
	if (using_gpu) {
		_sync_gpu_pins();
		return;
	}
	const int n = solver.vertex_count();
	for (int i = 0; i < n; i++) {
		solver.set_pinned(i, false);
	}
}

void GodotPhysXSoftBody3D::apply_point_impulse(int p_point_index, const Vector3 &p_impulse) {
	ERR_FAIL_INDEX(p_point_index, (int)visual_vertex_count);
	const uint32_t v = map_visual_to_physics[p_point_index];
	if (using_gpu && volume) {
		volume->add_point_impulse(v, p_impulse);
		return;
	}
	if (solver.is_pinned((int)v)) {
		return;
	}
	// impulse = mass * dv; solver mass is areal, approximate per-vertex via total.
	const float inv_n = 1.0f / (float)MAX(solver.vertex_count(), 1);
	const float inv_mass = (float)solver.vertex_count() / MAX((float)total_mass, 0.001f) * inv_n;
	solver.velocities_mut()[v] += p_impulse * inv_mass;
}

void GodotPhysXSoftBody3D::apply_point_force(int p_point_index, const Vector3 &p_force, double p_delta) {
	apply_point_impulse(p_point_index, p_force * p_delta);
}

void GodotPhysXSoftBody3D::apply_central_impulse(const Vector3 &p_impulse) {
	if (using_gpu && volume) {
		volume->add_central_impulse(p_impulse);
		return;
	}
	const int n = solver.vertex_count();
	if (n == 0) {
		return;
	}
	const Vector3 dv = p_impulse / MAX((float)total_mass, 0.001f);
	LocalVector<Vector3> &vel = solver.velocities_mut();
	for (int i = 0; i < n; i++) {
		if (!solver.is_pinned(i)) {
			vel[i] += dv;
		}
	}
}

void GodotPhysXSoftBody3D::apply_central_force(const Vector3 &p_force, double p_delta) {
	apply_central_impulse(p_force * p_delta);
}

void GodotPhysXSoftBody3D::_update_normals_and_bounds() {
	const LocalVector<Vector3> &pos = solver.get_positions();
	const LocalVector<int> &idx = solver.get_indices();
	const int n = pos.size();
	if (n == 0) {
		bounds = AABB();
		return;
	}
	normals.resize(n);
	for (int i = 0; i < n; i++) {
		normals[i] = Vector3();
	}
	for (uint32_t t = 0; t + 2 < idx.size(); t += 3) {
		const int a = idx[t];
		const int b = idx[t + 1];
		const int c = idx[t + 2];
		const Vector3 fn = (pos[b] - pos[a]).cross(pos[c] - pos[a]);
		normals[a] += fn;
		normals[b] += fn;
		normals[c] += fn;
	}
	AABB b(pos[0], Vector3());
	for (int i = 0; i < n; i++) {
		const float len = normals[i].length();
		normals[i] = len > CMP_EPSILON ? normals[i] / len : Vector3(0, 1, 0);
		b.expand_to(pos[i]);
	}
	bounds = b;
}

// One physics query per vertex per frame -- the expensive part -- caching each
// contact as a plane. _resolve_contacts() then re-projects against those cached
// planes every substep for free, which is what stops a fast body tunnelling
// through thin geometry (a once-per-frame push can't).
void GodotPhysXSoftBody3D::_refresh_contacts() {
	const int n = solver.vertex_count();
	contact_n.resize(n);
	contact_p.resize(n);
	contact_hit.resize(n);
	for (int i = 0; i < n; i++) {
		contact_hit[i] = 0;
	}
	contact_count = 0;

	if (!space) {
		return;
	}
	PhysicsDirectSpaceState3D *ss = space->get_direct_state();
	if (!ss) {
		return;
	}
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	const float radius = MAX(solver.vertex_radius(), 0.02f);
	if (collision_sphere.is_null()) {
		collision_sphere = ps->sphere_shape_create();
	}
	ps->shape_set_data(collision_sphere, radius);

	PhysicsDirectSpaceState3D::ShapeParameters params;
	params.shape_rid = collision_sphere;
	params.collision_mask = collision_mask;
	params.collide_with_bodies = true;
	params.collide_with_areas = false;
	params.exclude = collision_exceptions;

	const LocalVector<Vector3> &pos = solver.get_positions();
	for (int i = 0; i < n; i++) {
		if (solver.is_pinned(i)) {
			continue;
		}
		params.transform = Transform3D(Basis(), pos[i]);
		PhysicsDirectSpaceState3D::ShapeRestInfo info;
		if (!ss->rest_info(params, &info)) {
			continue;
		}
		// rest_info reports info.point = query_center - normal * penetration, so
		// penetration = (pos - info.point) . normal (> 0 while overlapping). Cache
		// the actual surface plane -- point on the surface + its outward normal --
		// so _resolve_contacts() can keep the vertex on the outside all frame
		// even as it moves.
		const float penetration = (pos[i] - info.point).dot(info.normal);
		contact_n[i] = info.normal;
		contact_p[i] = pos[i] + info.normal * penetration; // point on the surface
		contact_hit[i] = 1;
		if (penetration > 0.0f) {
			contact_count++;
		}
	}
}

void GodotPhysXSoftBody3D::_resolve_contacts() {
	LocalVector<Vector3> &pos = solver.positions_mut();
	LocalVector<Vector3> &vel = solver.velocities_mut();
	const int n = solver.vertex_count();
	for (int i = 0; i < n; i++) {
		if (!contact_hit[i] || solver.is_pinned(i)) {
			continue;
		}
		const Vector3 dir = contact_n[i];
		const float sd = (pos[i] - contact_p[i]).dot(dir); // signed dist to surface
		if (sd >= 0.0f) {
			continue; // on the allowed side
		}
		pos[i] -= dir * sd; // back up to the surface plane
		const float vn = vel[i].dot(dir);
		const Vector3 v_tangent = vel[i] - dir * vn;
		// Light kinetic friction only -- a body slides down a ramp freely.
		// Settling (resting creep/spin) is handled by _damp_rigid_drift(), so
		// this must NOT clamp slow tangential motion or a body can't get moving.
		vel[i] = v_tangent * 0.88f + dir * MAX(vn, 0.0f);
	}
}

void GodotPhysXSoftBody3D::step(double p_delta, const Vector3 &p_gravity) {
	if (using_gpu) {
		return; // the PxDeformableVolume advances inside px_scene->simulate()
	}
	// Wait for the node to hand us its world transform -- stepping from the
	// mesh-local rest pose would drop the body through the floor and the
	// collision pass would then fling it out.
	if (!mesh_ready || !placed || p_delta <= 0.0) {
		return;
	}
	solver.step((float)p_delta, p_gravity, Vector3());
	// One contact query per frame, then project positions out. The max_speed
	// clamp keeps a body slow enough that this once-per-frame pass still catches
	// it before it tunnels through thin geometry; a per-substep re-query is too
	// expensive at useful body counts.
	_refresh_contacts();
	_resolve_contacts();
	_damp_rigid_drift();
	_update_normals_and_bounds();
}

void GodotPhysXSoftBody3D::read_back() {
	if (!using_gpu || !volume) {
		return;
	}
	volume->read_back();
	bounds = volume->get_bounds();
}

// Gauss-Seidel constraint ordering and the volume solve each leave a tiny,
// undamped rigid-body component in the velocities every step. A soft body has
// no angular damping to shed it, so a resting one slowly spins and wanders off.
// Split the motion into its rigid part (centre-of-mass translation + rotation)
// and its deformation part, and bleed the rigid part -- hard when the body is
// nearly at rest, gently otherwise so a thrown blob can still tumble.
void GodotPhysXSoftBody3D::_damp_rigid_drift() {
	// Only meaningful when the body is resting on something -- an airborne body
	// falling or thrown IS rigid motion and must be left alone.
	if (contact_count == 0) {
		return;
	}
	LocalVector<Vector3> &pos = solver.positions_mut();
	LocalVector<Vector3> &vel = solver.velocities_mut();
	const int n = solver.vertex_count();
	if (n < 2) {
		return;
	}
	Vector3 com;
	Vector3 com_vel;
	int free_count = 0;
	for (int i = 0; i < n; i++) {
		if (solver.is_pinned(i)) {
			continue;
		}
		com += pos[i];
		com_vel += vel[i];
		free_count++;
	}
	if (free_count < 2) {
		return;
	}
	com /= (float)free_count;
	com_vel /= (float)free_count;

	Vector3 ang_mom;
	float inertia = 0.0f;
	for (int i = 0; i < n; i++) {
		if (solver.is_pinned(i)) {
			continue;
		}
		const Vector3 r = pos[i] - com;
		ang_mom += r.cross(vel[i] - com_vel);
		inertia += r.length_squared();
	}
	const Vector3 omega = inertia > 1.0e-6f ? ang_mom / inertia : Vector3();

	const float rigid_speed = com_vel.length() + omega.length();
	// Bleed ramps smoothly from a barely-there trickle while the body is still
	// travelling up to a strong pull as it comes to rest -- no braking of a
	// moving body, but resting creep/spin still dies.
	const float t = CLAMP((0.45f - rigid_speed) / 0.45f, 0.0f, 1.0f);
	const float damp = 0.004f + (0.55f - 0.004f) * t * t;
	for (int i = 0; i < n; i++) {
		if (solver.is_pinned(i)) {
			continue;
		}
		const Vector3 rigid_v = com_vel + omega.cross(pos[i] - com);
		vel[i] -= rigid_v * damp;
	}
}

void GodotPhysXSoftBody3D::update_rendering_server(PhysicsServer3DRenderingServerHandler *p_handler) {
	if (!mesh_ready || !p_handler) {
		return;
	}
	if (using_gpu && volume) {
		for (uint32_t i = 0; i < visual_vertex_count; i++) {
			const uint32_t v = map_visual_to_physics[i];
			p_handler->set_vertex(i, volume->get_vertex_position(v));
			p_handler->set_normal(i, volume->get_vertex_normal(v));
		}
		p_handler->set_aabb(volume->get_bounds());
		return;
	}
	const LocalVector<Vector3> &pos = solver.get_positions();
	for (uint32_t i = 0; i < visual_vertex_count; i++) {
		const uint32_t v = map_visual_to_physics[i];
		p_handler->set_vertex(i, pos[v]);
		p_handler->set_normal(i, normals[v]);
	}
	p_handler->set_aabb(bounds);
}

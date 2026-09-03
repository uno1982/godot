/**************************************************************************/
/*  physx_particle_fluid_3d.cpp                                           */
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

#include "physx_particle_fluid_3d.h"

#include "../godot_physx_server_3d.h"
#include "../objects/godot_physx_particle_fluid_3d.h"

#include "core/config/engine.h"
#include "core/config/project_settings.h"
#include "core/object/class_db.h"
#include "core/templates/hash_map.h"
#include "scene/main/viewport.h"
#include "scene/resources/3d/primitive_meshes.h"
#include "scene/resources/3d/world_3d.h"
#include "scene/resources/material.h"
#include "servers/rendering/rendering_server.h"

void PhysXParticleFluid3D::_make_fluid() {
	if (fluid.is_valid()) {
		return;
	}
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server) {
		WARN_PRINT_ONCE("PhysXParticleFluid3D does nothing unless the 3D physics engine is set to \"PhysX\".");
		return;
	}
	ERR_FAIL_NULL(get_world_3d());

	fluid = server->particle_fluid_create();
	server->particle_fluid_set_space(fluid, get_world_3d()->get_space());
	server->particle_fluid_set_capacity(fluid, particle_count);
	_apply_params();

	// Render the particles as a MultiMesh of small spheres.
	Ref<SphereMesh> sphere;
	sphere.instantiate();
	sphere->set_radius(particle_size * 0.5);
	sphere->set_height(particle_size);
	sphere->set_radial_segments(6);
	sphere->set_rings(3);
	particle_mesh = sphere;

	RenderingServer *rs = RenderingServer::get_singleton();
	multimesh = rs->multimesh_create();
	rs->multimesh_allocate_data(multimesh, particle_count, RSE::MULTIMESH_TRANSFORM_3D);
	rs->multimesh_set_mesh(multimesh, particle_mesh->get_rid());
	rs->multimesh_set_visible_instances(multimesh, 0);

	set_base(multimesh);

	// Foam/spray/bubble particles: a second MultiMesh in its own world-space
	// instance (particle positions come back in world space). Used only when the
	// isosurface path is off -- otherwise foam is drawn as its own coarse
	// isosurface layer (foam_array_mesh) so it depth-sorts with the water.
	Ref<SphereMesh> foam_sphere;
	foam_sphere.instantiate();
	foam_sphere->set_radius(_effective_foam_size() * 0.5);
	foam_sphere->set_height(_effective_foam_size());
	foam_sphere->set_radial_segments(5);
	foam_sphere->set_rings(2);
	foam_mesh = foam_sphere;

	Ref<StandardMaterial3D> foam_material;
	foam_material.instantiate();
	foam_material->set_albedo(Color(1, 1, 1, 0.85));
	foam_material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	foam_material->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
	foam_sphere->set_material(foam_material);

	foam_multimesh = rs->multimesh_create();
	rs->multimesh_allocate_data(foam_multimesh, foam_particle_count, RSE::MULTIMESH_TRANSFORM_3D);
	rs->multimesh_set_mesh(foam_multimesh, foam_mesh->get_rid());
	rs->multimesh_set_visible_instances(foam_multimesh, 0);

	foam_instance = rs->instance_create2(foam_multimesh, get_world_3d()->get_scenario());
	rs->instance_set_transform(foam_instance, Transform3D());
	// World-space instance that never moves; keep it from being culled.
	rs->instance_set_custom_aabb(foam_instance, AABB(Vector3(-100000, -100000, -100000), Vector3(200000, 200000, 200000)));

	_apply_foam();

	// GPU isosurface mesh path: PhysX marching-cubes a triangle mesh we draw as
	// an ArrayMesh with an ordinary water material.
	server->particle_fluid_set_surface_mesh(fluid, surface_mesh);
	server->particle_fluid_set_surface_anisotropy(fluid, surface_anisotropy);
	if (surface_mesh) {
		array_mesh = rs->mesh_create();
		if (water_material.is_null()) {
			Ref<StandardMaterial3D> m;
			m.instantiate();
			m->set_albedo(Color(0.12, 0.42, 0.62, 0.6));
			m->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
			m->set_metallic(0.1);
			m->set_roughness(0.06);
			m->set_feature(BaseMaterial3D::FEATURE_REFRACTION, true);
			m->set_refraction(0.05);
			water_material = m;
		}
		set_base(array_mesh);

		// Foam layer: a coarser isosurface over the diffuse particles, drawn as a
		// whiter, rougher, less transparent skin sitting on the water. Its own
		// world-space instance (the mesh comes back in world space).
		foam_array_mesh = rs->mesh_create();
		if (foam_water_material.is_null()) {
			Ref<StandardMaterial3D> fm;
			fm.instantiate();
			fm->set_albedo(Color(0.95, 0.97, 1.0, 0.85));
			fm->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
			fm->set_metallic(0.0);
			fm->set_roughness(0.9);
			fm->set_feature(BaseMaterial3D::FEATURE_REFRACTION, true);
			fm->set_refraction(0.02);
			foam_water_material = fm;
		}
		foam_mesh_instance = rs->instance_create2(foam_array_mesh, get_world_3d()->get_scenario());
		rs->instance_set_transform(foam_mesh_instance, Transform3D());
		rs->instance_set_custom_aabb(foam_mesh_instance, AABB(Vector3(-100000, -100000, -100000), Vector3(200000, 200000, 200000)));
	}
}

void PhysXParticleFluid3D::_apply_foam() {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return;
	}
	server->particle_fluid_set_foam(fluid, foam_enabled, foam_particle_count, foam_lifetime, foam_threshold, foam_buoyancy, _effective_foam_size());
}

void PhysXParticleFluid3D::_free_fluid() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (multimesh.is_valid()) {
		set_base(RID());
		rs->free_rid(multimesh);
		multimesh = RID();
	}
	particle_mesh.unref();

	if (foam_instance.is_valid()) {
		rs->free_rid(foam_instance);
		foam_instance = RID();
	}
	if (foam_multimesh.is_valid()) {
		rs->free_rid(foam_multimesh);
		foam_multimesh = RID();
	}
	foam_mesh.unref();

	if (foam_mesh_instance.is_valid()) {
		rs->free_rid(foam_mesh_instance);
		foam_mesh_instance = RID();
	}
	if (foam_array_mesh.is_valid()) {
		rs->free_rid(foam_array_mesh);
		foam_array_mesh = RID();
	}
	foam_water_material.unref();

	if (array_mesh.is_valid()) {
		rs->free_rid(array_mesh);
		array_mesh = RID();
	}
	water_material.unref();

	if (fluid.is_valid()) {
		GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
		if (server) {
			server->free_rid(fluid);
		}
		fluid = RID();
	}
	spawned = false;
}

void PhysXParticleFluid3D::_apply_params() {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return;
	}
	server->particle_fluid_set_param(fluid, GodotPhysXParticleFluid3D::PARAM_PARTICLE_SIZE, particle_size);
	server->particle_fluid_set_param(fluid, GodotPhysXParticleFluid3D::PARAM_VISCOSITY, viscosity);
	server->particle_fluid_set_param(fluid, GodotPhysXParticleFluid3D::PARAM_SURFACE_TENSION, surface_tension);
	server->particle_fluid_set_param(fluid, GodotPhysXParticleFluid3D::PARAM_COHESION, cohesion);
	server->particle_fluid_set_param(fluid, GodotPhysXParticleFluid3D::PARAM_VORTICITY, vorticity);
}

void PhysXParticleFluid3D::spawn() {
	_make_fluid();
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return;
	}

	// Jittered grid filling spawn_region_size, centered on the node.
	const Vector3 half = spawn_region_size * 0.5;
	const float spacing = MAX(particle_size, 0.001f);
	const Vector3i counts(
			MAX(1, int(spawn_region_size.x / spacing)),
			MAX(1, int(spawn_region_size.y / spacing)),
			MAX(1, int(spawn_region_size.z / spacing)));
	const Transform3D xf = get_global_transform();

	Vector<Vector3> positions;
	positions.resize(MIN(particle_count, counts.x * counts.y * counts.z));
	Vector3 *w = positions.ptrw();
	int n = 0;
	const int cap = positions.size();
	const float jitter = spacing * 0.2;
	for (int ix = 0; ix < counts.x && n < cap; ix++) {
		for (int iy = 0; iy < counts.y && n < cap; iy++) {
			for (int iz = 0; iz < counts.z && n < cap; iz++) {
				Vector3 local(
						-half.x + (ix + 0.5f) * spacing + Math::randf() * jitter,
						-half.y + (iy + 0.5f) * spacing + Math::randf() * jitter,
						-half.z + (iz + 0.5f) * spacing + Math::randf() * jitter);
				w[n++] = xf.xform(local);
			}
		}
	}
	positions.resize(n);

	server->particle_fluid_set_particles(fluid, positions, Vector3());
	spawned = true;
	_update_render();
}

void PhysXParticleFluid3D::_emit_step(double p_delta) {
	_make_fluid();
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return;
	}
	emit_accum += emission_rate * p_delta;
	const int count = (int)emit_accum;
	if (count <= 0) {
		return;
	}
	emit_accum -= count;

	const Transform3D xf = get_global_transform();
	const Vector3 origin = xf.origin;
	const Vector3 world_vel = xf.basis.xform(emission_velocity);

	Vector<Vector3> positions;
	positions.resize(count);
	Vector3 *w = positions.ptrw();
	for (int i = 0; i < count; i++) {
		// A jittered point in the emission sphere, nudged along the flow so a
		// whole batch does not spawn on top of itself.
		const Vector3 j(
				Math::randf() * 2.0f - 1.0f,
				Math::randf() * 2.0f - 1.0f,
				Math::randf() * 2.0f - 1.0f);
		w[i] = origin + j * emission_radius + world_vel * (0.004f * (float)i);
	}
	server->particle_fluid_emit(fluid, positions, world_vel);
}

void PhysXParticleFluid3D::clear() {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (server && fluid.is_valid()) {
		server->particle_fluid_clear(fluid);
	}
	if (multimesh.is_valid()) {
		RenderingServer::get_singleton()->multimesh_set_visible_instances(multimesh, 0);
	}
	if (foam_multimesh.is_valid()) {
		RenderingServer::get_singleton()->multimesh_set_visible_instances(foam_multimesh, 0);
	}
	spawned = false;
}

int PhysXParticleFluid3D::get_live_particle_count() const {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return 0;
	}
	return server->particle_fluid_get_particle_count(fluid);
}

PackedVector3Array PhysXParticleFluid3D::get_particle_positions() const {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return PackedVector3Array();
	}
	return server->particle_fluid_get_positions(fluid);
}

float PhysXParticleFluid3D::get_submersion(const AABB &p_world_aabb) const {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return 0.0f;
	}
	return server->particle_fluid_get_submersion(fluid, p_world_aabb);
}

void PhysXParticleFluid3D::_update_render() {
	if (multimesh.is_null()) {
		return;
	}
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return;
	}
	RenderingServer *rs = RenderingServer::get_singleton();

	if (surface_mesh) {
		// The isosurface mesh draws the fluid -- do not read back every particle
		// position or rebuild the sphere buffer. (Foam is still updated below.)
		rs->multimesh_set_visible_instances(multimesh, 0);
	} else {
		const Vector<Vector3> positions = server->particle_fluid_get_positions(fluid);
		const int n = MIN(positions.size(), particle_count);

		// MultiMesh is in this node's local space; particle positions are world.
		const Transform3D inv = get_global_transform().affine_inverse();

		PackedFloat32Array buffer;
		buffer.resize(particle_count * 12);
		float *b = buffer.ptrw();
		const Vector3 *p = positions.ptr();
		for (int i = 0; i < n; i++) {
			const Vector3 local = inv.xform(p[i]);
			float *t = &b[i * 12];
			t[0] = 1;
			t[1] = 0;
			t[2] = 0;
			t[3] = local.x;
			t[4] = 0;
			t[5] = 1;
			t[6] = 0;
			t[7] = local.y;
			t[8] = 0;
			t[9] = 0;
			t[10] = 1;
			t[11] = local.z;
		}
		rs->multimesh_set_buffer(multimesh, buffer);
		rs->multimesh_set_visible_instances(multimesh, n);
	}

	if (foam_multimesh.is_null()) {
		return;
	}
	// With the isosurface path on, foam is drawn as foam_array_mesh instead.
	if (!foam_enabled || surface_mesh) {
		rs->multimesh_set_visible_instances(foam_multimesh, 0);
		return;
	}
	// Foam instance is world-space, so write particle world positions directly.
	const Vector<Vector3> foam = server->particle_fluid_get_foam_positions(fluid);
	const int fn = MIN(foam.size(), foam_particle_count);
	PackedFloat32Array foam_buffer;
	foam_buffer.resize(foam_particle_count * 12);
	float *fb = foam_buffer.ptrw();
	const Vector3 *fp = foam.ptr();
	for (int i = 0; i < fn; i++) {
		float *t = &fb[i * 12];
		t[0] = 1;
		t[1] = 0;
		t[2] = 0;
		t[3] = fp[i].x;
		t[4] = 0;
		t[5] = 1;
		t[6] = 0;
		t[7] = fp[i].y;
		t[8] = 0;
		t[9] = 0;
		t[10] = 1;
		t[11] = fp[i].z;
	}
	rs->multimesh_set_buffer(foam_multimesh, foam_buffer);
	rs->multimesh_set_visible_instances(foam_multimesh, fn);
}

void PhysXParticleFluid3D::_update_surface_mesh() {
	if (!surface_mesh || array_mesh.is_null()) {
		return;
	}
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return;
	}

	{
		PackedVector3Array verts, normals;
		PackedInt32Array indices;
		const int tris = server->particle_fluid_get_surface_mesh(fluid, verts, normals, indices, surface_mesh_version);
		if (tris >= 0) {
			// Keep only the largest connected component: stray marching-cubes
			// debris (a blade, a floating blob) is dropped.
			_commit_iso_mesh(array_mesh, verts, normals, indices, water_material, true, true, particle_size);
		}
	}
	if (foam_mesh_instance.is_valid()) {
		PackedVector3Array verts, normals;
		PackedInt32Array indices;
		const int tris = server->particle_fluid_get_foam_mesh(fluid, verts, normals, indices, foam_surface_mesh_version);
		if (tris >= 0) {
			// Foam is naturally many disconnected clumps -- keep them all.
			_commit_iso_mesh(foam_array_mesh, verts, normals, indices, foam_water_material, false, false, _effective_foam_size());
		}
	}
}

void PhysXParticleFluid3D::_commit_iso_mesh(RID p_mesh, PackedVector3Array &verts, PackedVector3Array &normals, PackedInt32Array &indices, const Ref<Material> &p_material, bool p_to_local, bool p_keep_largest_component, float p_feature_size) {
	RenderingServer *rs = RenderingServer::get_singleton();
	rs->mesh_clear(p_mesh);
	if (verts.is_empty() || indices.is_empty()) {
		return;
	}

	if (p_to_local) {
		// Isosurface vertices come back in world space; the node's instance
		// carries its own transform, so bring them into local space.
		const Transform3D inv = get_global_transform().affine_inverse();
		const Basis nb = inv.basis;
		Vector3 *vw = verts.ptrw();
		Vector3 *nw = normals.ptrw();
		for (int i = 0; i < verts.size(); i++) {
			vw[i] = inv.xform(vw[i]);
			nw[i] = nb.xform(nw[i]).normalized();
		}
	}

	// Drop any triangle stretched well past a grid cell (a stray particle
	// smeared across the level). When p_keep_largest_component, also keep only
	// the biggest connected triangle island (union-find over the index buffer).
	const float max_edge_sq = (p_feature_size * 4.0f) * (p_feature_size * 4.0f);
	AABB local_aabb;
	bool have_aabb = false;
	{
		const int *ip = indices.ptr();
		const Vector3 *vp = verts.ptr();
		const int vcount = verts.size();

		LocalVector<int> parent;
		parent.resize(vcount);
		for (int i = 0; i < vcount; i++) {
			parent[i] = i;
		}
		// Iterative find with path halving.
		auto find = [&](int x) {
			while (parent[x] != x) {
				parent[x] = parent[parent[x]];
				x = parent[x];
			}
			return x;
		};

		PackedInt32Array unstretched;
		unstretched.resize(indices.size());
		int u = 0;
		for (int t = 0; t + 2 < indices.size(); t += 3) {
			const int ia = ip[t], ib = ip[t + 1], ic = ip[t + 2];
			const Vector3 &a = vp[ia];
			const Vector3 &b = vp[ib];
			const Vector3 &c = vp[ic];
			if (a.distance_squared_to(b) > max_edge_sq || b.distance_squared_to(c) > max_edge_sq || c.distance_squared_to(a) > max_edge_sq) {
				continue;
			}
			unstretched.write[u++] = ia;
			unstretched.write[u++] = ib;
			unstretched.write[u++] = ic;
			const int ra = find(ia), rb = find(ib), rc = find(ic);
			parent[rb] = ra;
			parent[find(ic)] = find(ia);
			(void)rc;
		}
		unstretched.resize(u);

		// Tally triangles per component, find the biggest.
		int best_root = -1;
		if (p_keep_largest_component) {
			HashMap<int, int> comp_tris;
			int best_count = 0;
			for (int t = 0; t + 2 < u; t += 3) {
				const int r = find(unstretched[t]);
				const int cnt = (comp_tris[r] += 1);
				if (cnt > best_count) {
					best_count = cnt;
					best_root = r;
				}
			}
		}

		PackedInt32Array kept;
		kept.resize(u);
		int k = 0;
		const int *up = unstretched.ptr();
		for (int t = 0; t + 2 < u; t += 3) {
			if (p_keep_largest_component && find(up[t]) != best_root) {
				continue;
			}
			for (int j = 0; j < 3; j++) {
				const Vector3 &p = vp[up[t + j]];
				kept.write[k++] = up[t + j];
				if (!have_aabb) {
					local_aabb = AABB(p, Vector3());
					have_aabb = true;
				}
				local_aabb.expand_to(p);
			}
		}
		kept.resize(k);
		indices = kept;
	}
	if (indices.is_empty()) {
		return;
	}

	Array arrays;
	arrays.resize(RSE::ARRAY_MAX);
	arrays[RSE::ARRAY_VERTEX] = verts;
	arrays[RSE::ARRAY_NORMAL] = normals;
	arrays[RSE::ARRAY_INDEX] = indices;
	rs->mesh_add_surface_from_arrays(p_mesh, RSE::PRIMITIVE_TRIANGLES, arrays);
	if (p_material.is_valid()) {
		rs->mesh_surface_set_material(p_mesh, 0, p_material->get_rid());
	}
	rs->mesh_set_custom_aabb(p_mesh, local_aabb.grow(p_feature_size * 4.0f));
}

void PhysXParticleFluid3D::_notification(int p_what) {
	const bool editor = Engine::get_singleton()->is_editor_hint();
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			if (editor) {
				_editor_preview_enter();
				set_process_internal(true);
			} else {
				if (spawn_on_ready) {
					spawn();
				}
				set_physics_process_internal(true);
			}
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			set_physics_process_internal(false);
			set_process_internal(false);
			_editor_preview_exit();
			_free_fluid();
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			if (editor) {
				_editor_preview_step(get_process_delta_time());
			}
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (emitting) {
				_emit_step(get_physics_process_delta_time());
			}
			if (spawned || emitting) {
				_update_render();
				_update_surface_mesh();
			}
		} break;
	}
}

void PhysXParticleFluid3D::_editor_preview_enter() {
	RenderingServer *rs = RenderingServer::get_singleton();

	Ref<SphereMesh> s;
	s.instantiate();
	s->set_radius(particle_size * 0.5);
	s->set_height(particle_size);
	s->set_radial_segments(6);
	s->set_rings(3);
	Ref<StandardMaterial3D> m;
	m.instantiate();
	m->set_albedo(Color(0.3, 0.6, 1.0));
	m->set_shading_mode(BaseMaterial3D::SHADING_MODE_PER_PIXEL);
	s->set_material(m);
	preview_mesh = s;

	const int cap = 400;
	preview_pos.resize(cap);
	preview_vel.resize(cap);
	preview_age.resize(cap);
	for (int i = 0; i < cap; i++) {
		preview_age[i] = Math::randf() * 4.0f; // staggered so they don't all spawn at once
		preview_pos[i] = Vector3();
		preview_vel[i] = Vector3();
	}

	preview_multimesh = rs->multimesh_create();
	rs->multimesh_allocate_data(preview_multimesh, cap, RSE::MULTIMESH_TRANSFORM_3D);
	rs->multimesh_set_mesh(preview_multimesh, preview_mesh->get_rid());
	rs->multimesh_set_visible_instances(preview_multimesh, 0);
	set_base(preview_multimesh);
}

void PhysXParticleFluid3D::_editor_preview_exit() {
	if (preview_multimesh.is_valid()) {
		set_base(RID());
		RenderingServer::get_singleton()->free_rid(preview_multimesh);
		preview_multimesh = RID();
	}
	preview_mesh.unref();
	preview_pos.clear();
	preview_vel.clear();
	preview_age.clear();
}

void PhysXParticleFluid3D::_editor_preview_step(double p_delta) {
	if (preview_multimesh.is_null()) {
		return;
	}
	// Throttle to ~24 Hz. A node that processes every frame forces the editor to
	// redraw its viewport continuously (it normally idles); this keeps that cost
	// low, and the preview is only a rough hint anyway.
	preview_throttle += p_delta;
	if (preview_throttle < 1.0 / 24.0) {
		return;
	}
	p_delta = preview_throttle;
	preview_throttle = 0.0;

	const float dt = MIN((float)p_delta, 0.08f);
	const float lifetime = 2.5f;
	const int cap = preview_pos.size();

	// Trickle a steady rate of respawns when emitting; a gentle bubble otherwise.
	const bool jet = emitting && !emission_velocity.is_zero_approx();
	preview_accum += (jet ? 220.0 : 40.0) * p_delta;
	int budget = (int)preview_accum;
	preview_accum -= budget;

	PackedFloat32Array buffer;
	buffer.resize(cap * 12);
	float *b = buffer.ptrw();
	int visible = 0;

	for (int i = 0; i < cap; i++) {
		preview_age[i] += dt;
		bool respawn = preview_age[i] > lifetime || preview_pos[i].y < -spawn_region_size.y * 2.0f - 2.0f;
		if (respawn && budget > 0) {
			budget--;
			preview_age[i] = 0.0f;
			const Vector3 j(Math::randf() * 2 - 1, Math::randf() * 2 - 1, Math::randf() * 2 - 1);
			if (jet) {
				preview_pos[i] = j * emission_radius;
				preview_vel[i] = emission_velocity + j * (emission_velocity.length() * 0.12f);
			} else {
				preview_pos[i] = Vector3(j.x, j.y, j.z) * (spawn_region_size * 0.5f);
				preview_vel[i] = Vector3();
			}
		} else if (respawn) {
			preview_age[i] = lifetime + 1.0f; // park it until there is budget
		}

		if (preview_age[i] <= lifetime) {
			preview_vel[i].y -= 9.8f * dt;
			preview_pos[i] += preview_vel[i] * dt;
			const Vector3 &p = preview_pos[i];
			float *t = &b[visible * 12];
			t[0] = 1;
			t[1] = 0;
			t[2] = 0;
			t[3] = p.x;
			t[4] = 0;
			t[5] = 1;
			t[6] = 0;
			t[7] = p.y;
			t[8] = 0;
			t[9] = 0;
			t[10] = 1;
			t[11] = p.z;
			visible++;
		}
	}

	RenderingServer *rs = RenderingServer::get_singleton();
	rs->multimesh_set_buffer(preview_multimesh, buffer);
	rs->multimesh_set_visible_instances(preview_multimesh, visible);
}

void PhysXParticleFluid3D::set_particle_count(int p_count) {
	particle_count = MAX(p_count, 1);
	update_configuration_warnings();
}

void PhysXParticleFluid3D::set_particle_size(float p_size) {
	particle_size = MAX(p_size, 0.001f);
	_apply_params();
	if (foam_size <= 0.0f) {
		_apply_foam(); // foam tracks particle_size when foam_size is auto
	}
}

void PhysXParticleFluid3D::set_viscosity(float p_v) {
	viscosity = p_v;
	_apply_params();
}

void PhysXParticleFluid3D::set_surface_tension(float p_v) {
	surface_tension = p_v;
	_apply_params();
}

void PhysXParticleFluid3D::set_cohesion(float p_v) {
	cohesion = p_v;
	_apply_params();
}

void PhysXParticleFluid3D::set_vorticity(float p_v) {
	vorticity = p_v;
	_apply_params();
}

void PhysXParticleFluid3D::set_spawn_region_size(const Vector3 &p_size) {
	spawn_region_size = p_size.maxf(0.0);
	update_gizmos();
}

void PhysXParticleFluid3D::set_spawn_on_ready(bool p_enable) {
	spawn_on_ready = p_enable;
}

void PhysXParticleFluid3D::set_emitting(bool p_emitting) {
	emitting = p_emitting;
}

void PhysXParticleFluid3D::set_emission_rate(float p_rate) {
	emission_rate = MAX(p_rate, 0.0f);
}

void PhysXParticleFluid3D::set_emission_radius(float p_radius) {
	emission_radius = MAX(p_radius, 0.0f);
}

void PhysXParticleFluid3D::set_emission_velocity(const Vector3 &p_velocity) {
	emission_velocity = p_velocity;
}

void PhysXParticleFluid3D::set_surface_anisotropy(bool p_enabled) {
	if (surface_anisotropy == p_enabled) {
		return;
	}
	surface_anisotropy = p_enabled;
	// Live toggle -- no rebuild; the isosurface callback reads it each step.
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (fluid.is_valid() && server) {
		server->particle_fluid_set_surface_anisotropy(fluid, surface_anisotropy);
	}
	update_configuration_warnings();
}

void PhysXParticleFluid3D::set_surface_mesh(bool p_enabled) {
	if (surface_mesh == p_enabled) {
		return;
	}
	surface_mesh = p_enabled;
	// Rebuild so the isosurface extractor and ArrayMesh come up (or tear down).
	if (fluid.is_valid()) {
		const bool was_spawned = spawned || emitting;
		_free_fluid();
		if (is_inside_tree() && !Engine::get_singleton()->is_editor_hint()) {
			_make_fluid();
			if (was_spawned && spawn_on_ready) {
				spawn();
			}
		}
	}
	update_configuration_warnings();
}

void PhysXParticleFluid3D::set_foam_enabled(bool p_enabled) {
	foam_enabled = p_enabled;
	_apply_foam();
}

void PhysXParticleFluid3D::set_foam_particle_count(int p_count) {
	foam_particle_count = MAX(p_count, 1);
}

void PhysXParticleFluid3D::set_foam_lifetime(float p_v) {
	foam_lifetime = MAX(p_v, 0.01f);
	_apply_foam();
}

void PhysXParticleFluid3D::set_foam_threshold(float p_v) {
	foam_threshold = MAX(p_v, 0.0f);
	_apply_foam();
}

void PhysXParticleFluid3D::set_foam_buoyancy(float p_v) {
	foam_buoyancy = CLAMP(p_v, 0.0f, 1.0f);
	_apply_foam();
}

void PhysXParticleFluid3D::set_foam_size(float p_v) {
	foam_size = p_v > 0.0f ? MAX(p_v, 0.001f) : 0.0f; // 0 = follow particle_size
	Ref<SphereMesh> s = foam_mesh;
	if (s.is_valid()) {
		s->set_radius(_effective_foam_size() * 0.5);
		s->set_height(_effective_foam_size());
	}
	_apply_foam();
}

int PhysXParticleFluid3D::get_live_foam_count() const {
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return 0;
	}
	return server->particle_fluid_get_foam_count(fluid);
}

AABB PhysXParticleFluid3D::get_aabb() const {
	// Particles roam past the node; a box a few times the spawn region keeps the
	// MultiMesh from being culled without making the editor bounds huge. The
	// isosurface mesh sets its own tight custom AABB when surface_mesh is on.
	const Vector3 e = spawn_region_size.maxf(1.0) * 1.5;
	return AABB(-e, e * 2.0);
}

PackedStringArray PhysXParticleFluid3D::get_configuration_warnings() const {
	PackedStringArray warnings = GeometryInstance3D::get_configuration_warnings();

	const String engine = GLOBAL_GET("physics/3d/physics_engine");
	if (engine != "PhysX" && engine != "DEFAULT") {
		warnings.push_back(RTR("This node only does anything when Project Settings > Physics > 3D > Physics Engine is set to \"PhysX\"."));
	}
	if (!GodotPhysXServer3D::get_singleton()) {
		warnings.push_back(RTR("The PhysX physics server is not active. Set the 3D physics engine to \"PhysX\"."));
	}
	if (surface_anisotropy && !surface_mesh) {
		warnings.push_back(RTR("\"Surface Anisotropy\" has no effect unless \"Surface Mesh\" is enabled."));
	}
	if (surface_anisotropy && emitting) {
		warnings.push_back(RTR("\"Surface Anisotropy\" sharpens the isosurface but can mesh fast particles along the emission stream as thin spikes. It is meant for a settled pool; disable it while emitting."));
	}
	if (particle_count > 200000) {
		warnings.push_back(RTR("Very high particle count. GPU fluids above ~100k particles get expensive."));
	}

	return warnings;
}

void PhysXParticleFluid3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("spawn"), &PhysXParticleFluid3D::spawn);
	ClassDB::bind_method(D_METHOD("clear"), &PhysXParticleFluid3D::clear);
	ClassDB::bind_method(D_METHOD("get_live_particle_count"), &PhysXParticleFluid3D::get_live_particle_count);
	ClassDB::bind_method(D_METHOD("get_particle_positions"), &PhysXParticleFluid3D::get_particle_positions);
	ClassDB::bind_method(D_METHOD("get_submersion", "world_aabb"), &PhysXParticleFluid3D::get_submersion);

	ClassDB::bind_method(D_METHOD("set_particle_count", "count"), &PhysXParticleFluid3D::set_particle_count);
	ClassDB::bind_method(D_METHOD("get_particle_count"), &PhysXParticleFluid3D::get_particle_count);
	ClassDB::bind_method(D_METHOD("set_particle_size", "size"), &PhysXParticleFluid3D::set_particle_size);
	ClassDB::bind_method(D_METHOD("get_particle_size"), &PhysXParticleFluid3D::get_particle_size);
	ClassDB::bind_method(D_METHOD("set_viscosity", "viscosity"), &PhysXParticleFluid3D::set_viscosity);
	ClassDB::bind_method(D_METHOD("get_viscosity"), &PhysXParticleFluid3D::get_viscosity);
	ClassDB::bind_method(D_METHOD("set_surface_tension", "surface_tension"), &PhysXParticleFluid3D::set_surface_tension);
	ClassDB::bind_method(D_METHOD("get_surface_tension"), &PhysXParticleFluid3D::get_surface_tension);
	ClassDB::bind_method(D_METHOD("set_cohesion", "cohesion"), &PhysXParticleFluid3D::set_cohesion);
	ClassDB::bind_method(D_METHOD("get_cohesion"), &PhysXParticleFluid3D::get_cohesion);
	ClassDB::bind_method(D_METHOD("set_vorticity", "vorticity"), &PhysXParticleFluid3D::set_vorticity);
	ClassDB::bind_method(D_METHOD("get_vorticity"), &PhysXParticleFluid3D::get_vorticity);
	ClassDB::bind_method(D_METHOD("set_spawn_region_size", "size"), &PhysXParticleFluid3D::set_spawn_region_size);
	ClassDB::bind_method(D_METHOD("get_spawn_region_size"), &PhysXParticleFluid3D::get_spawn_region_size);
	ClassDB::bind_method(D_METHOD("set_spawn_on_ready", "enable"), &PhysXParticleFluid3D::set_spawn_on_ready);
	ClassDB::bind_method(D_METHOD("get_spawn_on_ready"), &PhysXParticleFluid3D::get_spawn_on_ready);

	ClassDB::bind_method(D_METHOD("set_emitting", "emitting"), &PhysXParticleFluid3D::set_emitting);
	ClassDB::bind_method(D_METHOD("is_emitting"), &PhysXParticleFluid3D::is_emitting);
	ClassDB::bind_method(D_METHOD("set_emission_rate", "rate"), &PhysXParticleFluid3D::set_emission_rate);
	ClassDB::bind_method(D_METHOD("get_emission_rate"), &PhysXParticleFluid3D::get_emission_rate);
	ClassDB::bind_method(D_METHOD("set_emission_radius", "radius"), &PhysXParticleFluid3D::set_emission_radius);
	ClassDB::bind_method(D_METHOD("get_emission_radius"), &PhysXParticleFluid3D::get_emission_radius);
	ClassDB::bind_method(D_METHOD("set_emission_velocity", "velocity"), &PhysXParticleFluid3D::set_emission_velocity);
	ClassDB::bind_method(D_METHOD("get_emission_velocity"), &PhysXParticleFluid3D::get_emission_velocity);
	ClassDB::bind_method(D_METHOD("set_surface_mesh", "enabled"), &PhysXParticleFluid3D::set_surface_mesh);
	ClassDB::bind_method(D_METHOD("is_surface_mesh"), &PhysXParticleFluid3D::is_surface_mesh);
	ClassDB::bind_method(D_METHOD("set_surface_anisotropy", "enabled"), &PhysXParticleFluid3D::set_surface_anisotropy);
	ClassDB::bind_method(D_METHOD("is_surface_anisotropy"), &PhysXParticleFluid3D::is_surface_anisotropy);

	ClassDB::bind_method(D_METHOD("set_foam_enabled", "enabled"), &PhysXParticleFluid3D::set_foam_enabled);
	ClassDB::bind_method(D_METHOD("is_foam_enabled"), &PhysXParticleFluid3D::is_foam_enabled);
	ClassDB::bind_method(D_METHOD("set_foam_particle_count", "count"), &PhysXParticleFluid3D::set_foam_particle_count);
	ClassDB::bind_method(D_METHOD("get_foam_particle_count"), &PhysXParticleFluid3D::get_foam_particle_count);
	ClassDB::bind_method(D_METHOD("set_foam_lifetime", "seconds"), &PhysXParticleFluid3D::set_foam_lifetime);
	ClassDB::bind_method(D_METHOD("get_foam_lifetime"), &PhysXParticleFluid3D::get_foam_lifetime);
	ClassDB::bind_method(D_METHOD("set_foam_threshold", "threshold"), &PhysXParticleFluid3D::set_foam_threshold);
	ClassDB::bind_method(D_METHOD("get_foam_threshold"), &PhysXParticleFluid3D::get_foam_threshold);
	ClassDB::bind_method(D_METHOD("set_foam_buoyancy", "buoyancy"), &PhysXParticleFluid3D::set_foam_buoyancy);
	ClassDB::bind_method(D_METHOD("get_foam_buoyancy"), &PhysXParticleFluid3D::get_foam_buoyancy);
	ClassDB::bind_method(D_METHOD("set_foam_size", "size"), &PhysXParticleFluid3D::set_foam_size);
	ClassDB::bind_method(D_METHOD("get_foam_size"), &PhysXParticleFluid3D::get_foam_size);
	ClassDB::bind_method(D_METHOD("get_live_foam_count"), &PhysXParticleFluid3D::get_live_foam_count);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "particle_count", PROPERTY_HINT_RANGE, "1,262144,1"), "set_particle_count", "get_particle_count");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "particle_size", PROPERTY_HINT_RANGE, "0.01,1,0.001,suffix:m"), "set_particle_size", "get_particle_size");
	ADD_GROUP("Spawn", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "spawn_region_size", PROPERTY_HINT_NONE, "suffix:m"), "set_spawn_region_size", "get_spawn_region_size");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "spawn_on_ready"), "set_spawn_on_ready", "get_spawn_on_ready");
	ADD_GROUP("Emission", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "emitting"), "set_emitting", "is_emitting");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "emission_rate", PROPERTY_HINT_RANGE, "0,50000,1,or_greater,suffix:1/s"), "set_emission_rate", "get_emission_rate");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "emission_radius", PROPERTY_HINT_RANGE, "0,2,0.001,or_greater,suffix:m"), "set_emission_radius", "get_emission_radius");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "emission_velocity", PROPERTY_HINT_NONE, "suffix:m/s"), "set_emission_velocity", "get_emission_velocity");
	ADD_GROUP("Rendering", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "surface_mesh"), "set_surface_mesh", "is_surface_mesh");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "surface_anisotropy"), "set_surface_anisotropy", "is_surface_anisotropy");
	ADD_GROUP("Foam", "foam_");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "foam_enabled"), "set_foam_enabled", "is_foam_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "foam_particle_count", PROPERTY_HINT_RANGE, "1,262144,1"), "set_foam_particle_count", "get_foam_particle_count");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "foam_lifetime", PROPERTY_HINT_RANGE, "0.01,20,0.01,or_greater,suffix:s"), "set_foam_lifetime", "get_foam_lifetime");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "foam_threshold", PROPERTY_HINT_RANGE, "0,2000,1,or_greater"), "set_foam_threshold", "get_foam_threshold");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "foam_buoyancy", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_foam_buoyancy", "get_foam_buoyancy");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "foam_size", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_foam_size", "get_foam_size");
	ADD_GROUP("Fluid", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "viscosity", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater"), "set_viscosity", "get_viscosity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "surface_tension", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater"), "set_surface_tension", "get_surface_tension");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cohesion", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater"), "set_cohesion", "get_cohesion");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vorticity", PROPERTY_HINT_RANGE, "0,50,0.01,or_greater"), "set_vorticity", "get_vorticity");
}

PhysXParticleFluid3D::PhysXParticleFluid3D() {
}

PhysXParticleFluid3D::~PhysXParticleFluid3D() {
}

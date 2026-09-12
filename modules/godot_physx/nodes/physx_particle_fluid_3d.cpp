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
#include "../particles/mpm_fluid_solver.h"
#include "physx_chunk_emitter_3d.h"

#include "core/config/engine.h"
#include "core/config/project_settings.h"
#include "core/object/class_db.h"
#include "core/templates/hash_map.h"
#include "scene/3d/physics/animatable_body_3d.h"
#include "scene/3d/physics/collision_object_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/3d/physics/rigid_body_3d.h"
#include "scene/3d/physics/static_body_3d.h"
#include "scene/main/viewport.h"
#include "scene/resources/3d/box_shape_3d.h"
#include "scene/resources/3d/capsule_shape_3d.h"
#include "scene/resources/3d/primitive_meshes.h"
#include "scene/resources/3d/sphere_shape_3d.h"
#include "scene/resources/3d/world_3d.h"
#include "scene/resources/3d/world_boundary_shape_3d.h"
#include "scene/resources/material.h"
#include "servers/rendering/rendering_server.h"

PhysXParticleFluid3D::SolverBackend PhysXParticleFluid3D::_resolved_solver() const {
	if (solver != SOLVER_AUTO) {
		return solver;
	}
	// Granular: the MPM solver is the one that actually piles and holds a slope
	// (PhysX PBD friction cannot), so Auto prefers it even when CUDA is present.
	// Pick PBD granular explicitly for a pure pour / cascade where the grains
	// stay in motion.
	if (_is_granular()) {
		return SOLVER_MPM;
	}
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	return (server && server->has_gpu()) ? SOLVER_PBD : SOLVER_MPM;
}

// particle_size inflates the MPM isosurface the way it does on the PBD path
// (which keys the extractor off the particle radius): it widens the SPH scatter
// kernel *and* boosts the per-particle mass, so the same positions mesh as a
// fatter, smoother body of fluid. The iso level stays a fixed fraction of the
// native packed density.
int PhysXParticleFluid3D::_mpm_resolved_grid_res() const {
	if (mpm_grid_resolution > 0) {
		return mpm_grid_resolution;
	}
	return CLAMP((int)Math::round(mpm_domain_size.x / MAX(particle_size * 2.0f, 0.001f)), 12, 96);
}

void PhysXParticleFluid3D::_mpm_surface_params(float &r_iso, float &r_kernel, float &r_boost) const {
	// With the auto grid the particle spacing tracks particle_size, so the SPH
	// scatter mostly just needs a kernel a few cells wide; a small residual boost
	// covers a pinned grid that is finer than particle_size wants.
	const float spacing = _is_granular()
			? MAX(mpm_domain_size.x / MAX(_mpm_resolved_grid_res(), 1), 0.001f) * 0.5f
			: MAX(particle_size, 0.001f); // fluid: dx = 2 * particle_size, so spacing = particle_size
	const float rel = particle_size / spacing;
	r_kernel = CLAMP(particle_size, spacing * 2.0f, spacing * 6.0f); // scatter radius (meters)
	r_boost = CLAMP(rel * rel * rel, 1.0f, 24.0f);
	r_iso = 0.5f; // fraction of the native packed kernel density (the solver resolves it)

	if (_mpm_emit_mode) {
		// A poured stream is genuinely sparse in freefall (grid-transfer fluid
		// has no particle cohesion to draw it into a thread the way the PBD path
		// does), so with the normal iso level it reconstructs as disconnected
		// drips. Lower the threshold for the emitter case: fewer accumulated
		// neighbors are needed to cross the surface, so the falling column
		// fuses -- with no change to the scatter-loop cost, and the dense pool
		// (well above any threshold) barely moves.
		r_iso = 0.18f;
	}
}

void PhysXParticleFluid3D::_mpm_apply_surface_params() {
	if (mpm == nullptr) {
		return;
	}
	float iso, kernel, boost;
	_mpm_surface_params(iso, kernel, boost);
	mpm->set_surface_params(iso, kernel, boost);
}

void PhysXParticleFluid3D::_mpm_configure(bool p_prefill) {
	if (mpm == nullptr) {
		return;
	}
	// Set before _mpm_surface_params() below -- it widens the scatter kernel in
	// emit mode.
	_mpm_emit_mode = !p_prefill;

	MPMFluidSolver::Settings s;
	s.particle_target = particle_count;
	s.substeps = mpm_substeps;
	s.stiffness = mpm_stiffness;
	s.viscosity = viscosity;
	s.domain = mpm_domain_size;
	s.spawn_region = spawn_region_size;
	s.granular = _is_granular();
	if (s.granular) {
		s.granular_hardness = _granular_hardness();
		s.granular_friction_deg = _granular_friction_deg();
		s.granular_cohesion = _granular_cohesion();
		s.rest_density = _granular_density(); // grain "weight" -- scales the collider coupling
	}

	// The MPM grid and the fluid particle spacing are coupled: too few particles
	// per cell and the sim is noisy. particle_size sets the particle spacing.
	// Granular runs a dense box grid, so it takes a resolution (cells across the
	// box). The fluid path is block-sparse and boundless -- there is no box, so
	// the cell size is just 2x particle_size and mpm_grid_resolution / the x,z of
	// mpm_domain_size don't affect the grid.
	s.grid_res = _mpm_resolved_grid_res();
	if (!s.granular) {
		s.cell_size = MAX(particle_size * 2.0f, 0.001f);
	}
	_mpm_surface_params(s.surface_iso, s.surface_kernel, s.surface_boost);

	mpm->configure(s, get_global_transform(), p_prefill);
	// The GPU build runs on the render thread; is_available() only flips true a
	// frame or two later. Capacity is known synchronously, so treat "configure
	// requested" as configured and let the per-step is_available() guards handle
	// the warm-up.
	_mpm_configured = mpm->has_device() && mpm->get_capacity() > 0;

	// Size the MultiMesh to the buffer capacity (prefill: the seeded slab count;
	// emit: particle_count).
	if (multimesh.is_valid() && mpm->get_capacity() > 0) {
		RenderingServer *rs = RenderingServer::get_singleton();
		rs->multimesh_allocate_data(multimesh, mpm->get_capacity(), RSE::MULTIMESH_TRANSFORM_3D);
		rs->multimesh_set_mesh(multimesh, particle_mesh->get_rid());
		rs->multimesh_set_visible_instances(multimesh, 0);
	}
}

// Resolve a node to an analytic MPM collider. Accepts a CollisionShape3D
// directly, or a CollisionObject3D (uses its first shape). Sphere, box, capsule
// and infinite-plane map exactly; anything else falls back to a bounding sphere.
static bool resolve_mpm_collider(Node3D *p_node, MPMFluidSolver::Collider &out) {
	out.velocity = Vector3();

	Ref<Shape3D> shape;
	Transform3D xf = p_node->get_global_transform();

	if (CollisionShape3D *cshape = Object::cast_to<CollisionShape3D>(p_node)) {
		shape = cshape->get_shape();
	} else if (CollisionObject3D *co = Object::cast_to<CollisionObject3D>(p_node)) {
		List<uint32_t> owners;
		co->get_shape_owners(&owners);
		for (uint32_t owner : owners) {
			if (co->shape_owner_get_shape_count(owner) > 0) {
				xf = xf * co->shape_owner_get_transform(owner);
				shape = co->shape_owner_get_shape(owner, 0);
				break;
			}
		}
	}

	const Vector3 scale = xf.basis.get_scale().abs();
	const float uniform_scale = MAX(scale.x, MAX(scale.y, scale.z));
	out.position = xf.origin;
	out.rotation = xf.basis.get_rotation_quaternion();

	Ref<SphereShape3D> sph = shape;
	if (sph.is_valid()) {
		out.shape = MPMFluidSolver::COLLIDER_SPHERE;
		out.extents = Vector3(MAX(sph->get_radius() * uniform_scale, 0.01f), 0, 0);
		return true;
	}
	Ref<BoxShape3D> box = shape;
	if (box.is_valid()) {
		out.shape = MPMFluidSolver::COLLIDER_BOX;
		out.extents = (box->get_size() * 0.5f) * scale;
		return true;
	}
	Ref<CapsuleShape3D> cap = shape;
	if (cap.is_valid()) {
		const float r = cap->get_radius() * MAX(scale.x, scale.z);
		const float half_h = MAX(cap->get_height() * 0.5f * scale.y - r, 0.0f);
		out.shape = MPMFluidSolver::COLLIDER_CAPSULE;
		out.extents = Vector3(MAX(r, 0.01f), half_h, 0);
		return true;
	}
	Ref<WorldBoundaryShape3D> wb = shape;
	if (wb.is_valid()) {
		const Plane p = xf.xform(wb->get_plane());
		out.shape = MPMFluidSolver::COLLIDER_PLANE;
		out.position = p.normal * p.d; // a point on the plane
		out.extents = p.normal;
		out.rotation = Quaternion();
		return true;
	}

	// Fallback: bounding sphere from the visual bounds.
	float radius = 0.5f;
	if (VisualInstance3D *vi = Object::cast_to<VisualInstance3D>(p_node)) {
		const AABB ab = vi->get_aabb();
		if (ab.has_volume()) {
			radius = ab.get_longest_axis_size() * 0.5f;
		}
	}
	out.shape = MPMFluidSolver::COLLIDER_SPHERE;
	out.position = xf.origin;
	out.extents = Vector3(MAX(radius * uniform_scale, 0.01f), 0, 0);
	return true;
}

// Where a fed MPM collider came from, so the fluid's reaction impulse can be
// routed back. Exactly one of node / chunk_emitter is set.
struct ColliderSource {
	Node3D *node = nullptr;
	PhysXChunkEmitter3D *chunk_emitter = nullptr;
	int chunk_index = -1;
};

void PhysXParticleFluid3D::_mpm_step(double p_delta) {
	if (mpm == nullptr || !mpm->is_available() || multimesh.is_null()) {
		return;
	}
	mpm->set_domain_transform(get_global_transform());

	// Gather coupled bodies as analytic colliders. Budget is the solver's
	// MAX_COLLIDERS (32): explicitly-listed nodes first, then the nearest active
	// debris chunks of any PhysXChunkEmitter3D in the list fill what is left.
	static const int COLLIDER_BUDGET = 32;
	LocalVector<MPMFluidSolver::Collider> cols;
	LocalVector<ColliderSource> col_src;
	HashMap<ObjectID, Vector3> seen_pos;
	LocalVector<PhysXChunkEmitter3D *> chunk_emitters;

	for (int i = 0; i < mpm_colliders.size() && (int)cols.size() < COLLIDER_BUDGET; i++) {
		Node3D *n = Object::cast_to<Node3D>(get_node_or_null(mpm_colliders[i]));
		if (n == nullptr) {
			continue;
		}
		if (PhysXChunkEmitter3D *ce = Object::cast_to<PhysXChunkEmitter3D>(n)) {
			chunk_emitters.push_back(ce);
			continue; // expanded below, once the fixed colliders are counted
		}
		MPMFluidSolver::Collider c;
		if (!resolve_mpm_collider(n, c)) {
			continue;
		}

		RigidBody3D *rb = Object::cast_to<RigidBody3D>(n);
		if (rb != nullptr && !rb->is_freeze_enabled()) {
			c.velocity = rb->get_linear_velocity();
		} else if (p_delta > 0.0) {
			const Vector3 *prev = _mpm_prev_pos.getptr(n->get_instance_id());
			c.velocity = prev ? (c.position - *prev) / (float)p_delta : Vector3();
		}
		seen_pos[n->get_instance_id()] = c.position;
		cols.push_back(c);
		col_src.push_back(ColliderSource{ n, nullptr, -1 });
	}

	// Auto colliders: every non-static body overlapping the domain this step,
	// nearest first, filling whatever the explicit list leaves of the budget.
	if (mpm_auto_colliders && (int)cols.size() < COLLIDER_BUDGET && is_inside_tree()) {
		Ref<World3D> world = get_world_3d();
		PhysicsDirectSpaceState3D *ss = world.is_valid() ? world->get_direct_space_state() : nullptr;
		if (ss != nullptr) {
			if (_mpm_query_shape.is_null()) {
				_mpm_query_shape = PhysicsServer3D::get_singleton()->box_shape_create();
			}
			PhysicsServer3D::get_singleton()->shape_set_data(_mpm_query_shape, mpm_domain_size * 0.5f);

			PhysicsDirectSpaceState3D::ShapeParameters qp;
			qp.shape_rid = _mpm_query_shape;
			qp.transform = get_global_transform();
			qp.collide_with_bodies = true;
			qp.collide_with_areas = false;

			PhysicsDirectSpaceState3D::ShapeResult res[COLLIDER_BUDGET];
			const int hits = ss->intersect_shape(qp, res, COLLIDER_BUDGET);

			struct AutoCand {
				MPMFluidSolver::Collider c;
				Node3D *node;
				float dist2;
			};
			struct Closest {
				bool operator()(const AutoCand &a, const AutoCand &b) const { return a.dist2 < b.dist2; }
			};
			const Vector3 dcentre = get_global_transform().origin;
			LocalVector<AutoCand> acands;
			for (int h = 0; h < hits; h++) {
				Node3D *n = Object::cast_to<Node3D>(res[h].collider);
				if (n == nullptr) {
					continue;
				}
				// Skip immovable bodies (walls / floor -- list those once); keep
				// AnimatableBody3D (moving platforms) and every dynamic body.
				if (Object::cast_to<StaticBody3D>(n) != nullptr && Object::cast_to<AnimatableBody3D>(n) == nullptr) {
					continue;
				}
				if (seen_pos.has(n->get_instance_id())) {
					continue; // already fed from the explicit list
				}
				MPMFluidSolver::Collider c;
				if (!resolve_mpm_collider(n, c)) {
					continue;
				}
				RigidBody3D *rb = Object::cast_to<RigidBody3D>(n);
				if (rb != nullptr && !rb->is_freeze_enabled()) {
					c.velocity = rb->get_linear_velocity();
				} else if (p_delta > 0.0) {
					const Vector3 *prev = _mpm_prev_pos.getptr(n->get_instance_id());
					c.velocity = prev ? (c.position - *prev) / (float)p_delta : Vector3();
				}
				acands.push_back(AutoCand{ c, n, (float)(c.position - dcentre).length_squared() });
			}
			acands.sort_custom<Closest>();
			const int take = MIN((int)acands.size(), COLLIDER_BUDGET - (int)cols.size());
			for (int i = 0; i < take; i++) {
				seen_pos[acands[i].node->get_instance_id()] = acands[i].c.position;
				cols.push_back(acands[i].c);
				col_src.push_back(ColliderSource{ acands[i].node, nullptr, -1 });
			}
		}
	}
	_mpm_prev_pos = seen_pos;

	// Debris chunks: keep only those overlapping the fluid domain, nearest first.
	if (!chunk_emitters.is_empty() && (int)cols.size() < COLLIDER_BUDGET) {
		const Transform3D dxf = get_global_transform();
		const Vector3 dcentre = dxf.origin;
		const Vector3 dhalf = mpm_domain_size * 0.5f + Vector3(0.25f, 0.25f, 0.25f);

		struct Cand {
			MPMFluidSolver::Collider c;
			PhysXChunkEmitter3D *emitter;
			int index;
			float dist2;
		};
		struct ClosestChunk {
			bool operator()(const Cand &a, const Cand &b) const { return a.dist2 < b.dist2; }
		};
		LocalVector<Cand> cands;
		LocalVector<PhysXChunkEmitter3D::ChunkBody> bodies;
		for (PhysXChunkEmitter3D *ce : chunk_emitters) {
			ce->get_active_chunk_bodies(bodies);
			for (const PhysXChunkEmitter3D::ChunkBody &b : bodies) {
				const Vector3 rel = b.xform.origin - dcentre;
				const float reach = b.half_extents.length();
				if (Math::abs(rel.x) > dhalf.x + reach || Math::abs(rel.y) > dhalf.y + reach || Math::abs(rel.z) > dhalf.z + reach) {
					continue;
				}
				MPMFluidSolver::Collider c;
				c.shape = b.sphere ? MPMFluidSolver::COLLIDER_SPHERE : MPMFluidSolver::COLLIDER_BOX;
				c.position = b.xform.origin;
				c.rotation = b.xform.basis.get_rotation_quaternion();
				c.extents = b.sphere ? Vector3(MAX(b.half_extents.x, 0.01f), 0, 0) : b.half_extents;
				c.velocity = b.velocity;
				cands.push_back(Cand{ c, ce, b.index, (float)rel.length_squared() });
			}
		}
		cands.sort_custom<ClosestChunk>();
		const int take = MIN((int)cands.size(), COLLIDER_BUDGET - (int)cols.size());
		for (int i = 0; i < take; i++) {
			cols.push_back(cands[i].c);
			col_src.push_back(ColliderSource{ nullptr, cands[i].emitter, cands[i].index });
		}
	}

	// The isosurface march + triple GPU->CPU readback + ArrayMesh rebuild is by
	// far the most expensive part of a step -- and it dominates when a collider
	// churns the pool and the triangle count spikes. The surface only needs to
	// look continuous, not track every substep, so re-mesh it every 3rd step
	// (~20 Hz) and let the sim run full-rate underneath.
	static const uint32_t SURFACE_EVERY = 3;
	const bool want_surface = surface_mesh && array_mesh.is_valid() && !_is_granular() && (_mpm_surface_tick++ % SURFACE_EVERY == 0);

	if (mpm->get_particle_count() == 0) {
		// Emit mode, nothing spawned yet.
		RenderingServer::get_singleton()->multimesh_set_visible_instances(multimesh, 0);
		return;
	}

	LocalVector<Vector3> impulses;
	// Granular runs async (visuals a frame late, GPU overlapped); fluid syncs so
	// its coupling reaction stays in phase with the rigid bodies.
	mpm->step(p_delta, cols, &impulses, want_surface, _is_granular());

	// Reaction: push the coupled bodies back with the fluid's impulse. Clamp to a
	// sane per-frame velocity change so a solver blow-up can't launch anything
	// across the level.
	for (uint32_t i = 0; i < impulses.size() && i < col_src.size(); i++) {
		const ColliderSource &src = col_src[i];
		if (src.chunk_emitter != nullptr) {
			Vector3 imp = impulses[i];
			const float m = MAX(cols[i].extents.x * cols[i].extents.y * cols[i].extents.z * 8.0f * 1200.0f, 0.001f);
			const float cap = m * 20.0f;
			if (imp.length() > cap) {
				imp = imp.normalized() * cap;
			}
			src.chunk_emitter->apply_chunk_impulse(src.chunk_index, imp);
			continue;
		}
		RigidBody3D *rb = Object::cast_to<RigidBody3D>(src.node);
		if (rb == nullptr || rb->is_freeze_enabled()) {
			continue;
		}
		Vector3 imp = impulses[i];
		// A liquid splash can legitimately shove a body hard; a granular bed can
		// only *support* one. Cap the granular reaction at a few g of holding
		// force (dt-scaled) so a body resting in sand is held up, not launched --
		// grid nodes stopped under a half-buried collider otherwise sum to a
		// large net-up impulse every step.
		const float cap = _is_granular()
				? rb->get_mass() * 35.0f * float(MAX(p_delta, 0.0))
				: rb->get_mass() * 20.0f;
		if (imp.length() > cap) {
			imp = imp.normalized() * cap;
		}
		if (_is_granular() && rb->get_mass() > 0.0) {
			const float vy = rb->get_linear_velocity().y;
			if (imp.y > 0.0f) {
				// The lagged grid state overshoots the support impulse, so a
				// body stood in sand bobs and floats up. The upward reaction
				// may only arrest the body sinking -- never push past zero,
				// nothing at all while it is already rising.
				imp.y = MIN(imp.y, MAX(0.0f, -vy) * rb->get_mass());
			} else if (vy > 0.0f) {
				// Body leaving the bed (a jump): grains cling to the capsule
				// flank and the couple pass's tangential damping would drag the
				// whole jump back down. Let the sand shave the climb, not kill
				// it -- at most a fifth of the upward speed per solve.
				imp.y = MAX(imp.y, -0.2f * vy * rb->get_mass());
			}
		}
		rb->apply_central_impulse(imp);
	}

	RenderingServer *rs = RenderingServer::get_singleton();

	if (want_surface) {
		// The solver marches the isosurface on the GPU; we get back a triangle
		// soup in world space and hand it to array_mesh as-is -- it draws
		// through array_mesh_instance's own always-identity world-space
		// instance, not this node's transform (see that member's comment: the
		// mesh is only re-marched every SURFACE_EVERY steps, so baking a
		// local-space conversion here would go stale the moment this node's
		// transform changes again before the next re-mesh -- exactly the case
		// for an emitter mounted on a moving/aiming camera).
		PackedVector3Array verts, normals;
		const int tris = mpm->get_surface_mesh(verts, normals);
		rs->mesh_clear(array_mesh);
		if (tris > 0) {
			Array arrays;
			arrays.resize(RSE::ARRAY_MAX);
			arrays[RSE::ARRAY_VERTEX] = verts;
			arrays[RSE::ARRAY_NORMAL] = normals;
			rs->mesh_add_surface_from_arrays(array_mesh, RSE::PRIMITIVE_TRIANGLES, arrays);
			if (water_material.is_valid()) {
				rs->mesh_surface_set_material(array_mesh, 0, water_material->get_rid());
			}
			// array_mesh no longer draws through this node's own instance (see
			// array_mesh_instance's header comment), so the usual automatic
			// GeometryInstance3D::material_override behaviour doesn't reach it
			// anymore -- reapply it by hand to array_mesh_instance instead.
			const Ref<Material> override_mat = get_material_override();
			rs->instance_geometry_set_material_override(array_mesh_instance, override_mat.is_valid() ? override_mat->get_rid() : RID());
			_apply_gi_mode(array_mesh_instance);
		}
		rs->multimesh_set_visible_instances(multimesh, 0);
		return;
	}
	if (surface_mesh) {
		// Surface path, but this step's re-mesh was skipped for cost -- keep the
		// mesh from the last update and don't fall through to the sphere buffer.
		rs->multimesh_set_visible_instances(multimesh, 0);
		return;
	}

	// The solver packs world-space transforms; the MultiMesh is in node-local
	// space, so shift each instance origin by the node inverse. (Basis stays
	// identity -- the solver ignores domain rotation.)
	PackedFloat32Array buffer = mpm->get_multimesh_buffer();
	const int n = mpm->get_particle_count();
	const int cap = mpm->get_capacity();
	if (buffer.size() < cap * 12) {
		return;
	}
	const Transform3D inv = get_global_transform().affine_inverse();
	float *b = buffer.ptrw();
	for (int i = 0; i < n; i++) {
		float *t = &b[i * 12];
		const Vector3 local = inv.xform(Vector3(t[3], t[7], t[11]));
		t[3] = local.x;
		t[7] = local.y;
		t[11] = local.z;
	}
	rs->multimesh_set_buffer(multimesh, buffer);
	rs->multimesh_set_visible_instances(multimesh, n);
}

void PhysXParticleFluid3D::_make_fluid() {
	if (multimesh.is_valid()) {
		return;
	}

	if (_mpm_path()) {
		Ref<SphereMesh> sphere;
		sphere.instantiate();
		sphere->set_radius(particle_size * 0.5);
		sphere->set_height(particle_size);
		sphere->set_radial_segments(6);
		sphere->set_rings(3);
		{
			// The particle spheres get their own opaque material; water_material is
			// left for the isosurface path below (a shared handle shadowed it).
			Ref<StandardMaterial3D> pm;
			pm.instantiate();
			pm->set_albedo(Color(0.16, 0.44, 0.66));
			pm->set_metallic(0.0);
			pm->set_roughness(0.25);
			sphere->set_material(pm);
		}
		particle_mesh = sphere;

		RenderingServer *rs = RenderingServer::get_singleton();
		multimesh = rs->multimesh_create();
		rs->multimesh_allocate_data(multimesh, particle_count, RSE::MULTIMESH_TRANSFORM_3D);
		rs->multimesh_set_mesh(multimesh, particle_mesh->get_rid());
		rs->multimesh_set_visible_instances(multimesh, 0);
		set_base(multimesh);

		if (surface_mesh) {
			// Marching-tetrahedra mesh over the solver's density grid, drawn with a
			// translucent water material. The particle MultiMesh stays hidden.
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
				// Marching-tetrahedra output is not guaranteed watertight; render
				// both sides so a stray inward-facing triangle does not read as a
				// hole, and so the surface is still there when viewed from under.
				m->set_cull_mode(BaseMaterial3D::CULL_DISABLED);
				water_material = m;
			}
			array_mesh_instance = rs->instance_create2(array_mesh, get_world_3d()->get_scenario());
			rs->instance_set_transform(array_mesh_instance, Transform3D());
			rs->instance_set_custom_aabb(array_mesh_instance, AABB(Vector3(-100000, -100000, -100000), Vector3(200000, 200000, 200000)));
			_apply_gi_mode(array_mesh_instance);
		}

		mpm = memnew(MPMFluidSolver);
		if (!mpm->has_device()) {
			WARN_PRINT_ONCE("PhysXParticleFluid3D: the MPM compute solver could not start (no RenderingDevice / compute support).");
		}
		return;
	}

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
	const bool pbd_granular = _is_granular();
	server->particle_fluid_set_granular(fluid, pbd_granular, 2.0f * Math::tan(Math::deg_to_rad(CLAMP(_granular_friction_deg(), 1.0f, 55.0f))));
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
	_apply_gi_mode(foam_instance);

	_apply_foam();

	// GPU isosurface mesh path: PhysX marching-cubes a triangle mesh we draw as
	// an ArrayMesh with an ordinary water material. Granular grains render as the
	// sphere MultiMesh -- no surface.
	const bool pbd_surface = surface_mesh && !pbd_granular;
	server->particle_fluid_set_surface_mesh(fluid, pbd_surface);
	server->particle_fluid_set_surface_anisotropy(fluid, surface_anisotropy && !pbd_granular);
	if (pbd_surface) {
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
			// Thin fluid features (a stream, a spreading splash) mesh as a thin
			// shell; render both sides so they do not read as a hollow front face.
			m->set_cull_mode(BaseMaterial3D::CULL_DISABLED);
			water_material = m;
		}
		array_mesh_instance = rs->instance_create2(array_mesh, get_world_3d()->get_scenario());
		rs->instance_set_transform(array_mesh_instance, Transform3D());
		rs->instance_set_custom_aabb(array_mesh_instance, AABB(Vector3(-100000, -100000, -100000), Vector3(200000, 200000, 200000)));
		_apply_gi_mode(array_mesh_instance);

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
		_apply_gi_mode(foam_mesh_instance);
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
	if (mpm != nullptr) {
		memdelete(mpm);
		mpm = nullptr;
	}
	_mpm_configured = false;
	_mpm_emit_mode = false;
	emit_accum = 0.0;

	if (_mpm_query_shape.is_valid()) {
		PhysicsServer3D::get_singleton()->free_rid(_mpm_query_shape);
		_mpm_query_shape = RID();
	}

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

	if (array_mesh_instance.is_valid()) {
		rs->free_rid(array_mesh_instance);
		array_mesh_instance = RID();
	}
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

	if (_mpm_path()) {
		_mpm_configure();
		spawned = mpm != nullptr && mpm->is_available();
		return;
	}

	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return;
	}

	// Jittered grid filling spawn_region_size, centered on the node. y is the
	// outer loop so a fill that runs out of particles before the region is full
	// spreads across the whole footprint as a shallow bed -- matching the MPM
	// prefill (_seed_block) instead of banking a slab against one wall.
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
	for (int iy = 0; iy < counts.y && n < cap; iy++) {
		for (int iz = 0; iz < counts.z && n < cap; iz++) {
			for (int ix = 0; ix < counts.x && n < cap; ix++) {
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

void PhysXParticleFluid3D::_mpm_emit_step(double p_delta) {
	_make_fluid();
	if (mpm == nullptr || !mpm->has_device()) {
		return;
	}
	if (!_mpm_configured) {
		_mpm_configure(false); // empty buffer, fills from the stream
		spawned = _mpm_configured;
	}
	if (!_mpm_configured) {
		return;
	}

	emit_accum += emission_rate * p_delta;
	const int count = (int)emit_accum;
	if (count <= 0) {
		return;
	}
	emit_accum -= count;

	const Transform3D xf = get_global_transform();
	const Vector3 world_vel = xf.basis.xform(emission_velocity);

	LocalVector<MPMFluidSolver::EmittedParticle> batch;
	batch.resize(count);
	for (int i = 0; i < count; i++) {
		const Vector3 j(
				Math::randf() * 2.0f - 1.0f,
				Math::randf() * 2.0f - 1.0f,
				Math::randf() * 2.0f - 1.0f);
		batch[i].position = xf.origin + j * emission_radius + world_vel * (0.004f * (float)i);
		batch[i].velocity = world_vel;
	}
	mpm->emit(batch);
}

void PhysXParticleFluid3D::clear() {
	if (_mpm_path()) {
		if (multimesh.is_valid()) {
			RenderingServer::get_singleton()->multimesh_set_visible_instances(multimesh, 0);
		}
		spawned = false;
		_mpm_configured = false; // next spawn()/emit rebuilds the buffer
		emit_accum = 0.0;
		return;
	}

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
	if (mpm != nullptr) {
		return spawned ? mpm->get_particle_count() : 0;
	}
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!server || fluid.is_null()) {
		return 0;
	}
	return server->particle_fluid_get_particle_count(fluid);
}

double PhysXParticleFluid3D::get_mpm_step_msec() const {
	return mpm != nullptr ? mpm->get_last_step_msec() : 0.0;
}

PackedVector3Array PhysXParticleFluid3D::get_particle_positions() const {
	if (mpm != nullptr) {
		return mpm->get_positions();
	}
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

	if (surface_mesh && !_is_granular()) {
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
			// debris (a blade, a floating blob) is dropped. p_to_local=false --
			// array_mesh draws through its own always-identity world-space
			// instance (array_mesh_instance), not this node's own transform, so
			// the vertices stay in the world space they already came back in.
			_commit_iso_mesh(array_mesh, verts, normals, indices, water_material, false, true, particle_size);
			// See the MPM path's identical comment: array_mesh_instance is a
			// raw instance now, not this node's own, so material_override has
			// to be reapplied by hand instead of relying on the automatic
			// GeometryInstance3D behaviour.
			const Ref<Material> override_mat = get_material_override();
			RenderingServer::get_singleton()->instance_geometry_set_material_override(array_mesh_instance, override_mat.is_valid() ? override_mat->get_rid() : RID());
			_apply_gi_mode(array_mesh_instance);
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

void PhysXParticleFluid3D::_apply_gi_mode(RID p_instance) const {
	if (!p_instance.is_valid()) {
		return;
	}
	RenderingServer *rs = RenderingServer::get_singleton();
	switch (get_gi_mode()) {
		case GI_MODE_DISABLED:
			rs->instance_geometry_set_flag(p_instance, RSE::INSTANCE_FLAG_USE_BAKED_LIGHT, false);
			rs->instance_geometry_set_flag(p_instance, RSE::INSTANCE_FLAG_USE_DYNAMIC_GI, false);
			break;
		case GI_MODE_STATIC:
			rs->instance_geometry_set_flag(p_instance, RSE::INSTANCE_FLAG_USE_BAKED_LIGHT, true);
			rs->instance_geometry_set_flag(p_instance, RSE::INSTANCE_FLAG_USE_DYNAMIC_GI, false);
			break;
		case GI_MODE_DYNAMIC:
			rs->instance_geometry_set_flag(p_instance, RSE::INSTANCE_FLAG_USE_BAKED_LIGHT, false);
			rs->instance_geometry_set_flag(p_instance, RSE::INSTANCE_FLAG_USE_DYNAMIC_GI, true);
			break;
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

		// Tally triangles per connected component. When p_keep_largest_component,
		// drop only genuine speck debris -- a component is kept if it has at least
		// a small fraction of the biggest one's triangles (or a hard floor). This
		// is deliberately not winner-take-all: a scene where the fluid splits into
		// two similar blobs would otherwise flicker as "the largest" flips.
		HashMap<int, int> comp_tris;
		int keep_min = 0;
		int best_root = -1;
		if (p_keep_largest_component) {
			int best_count = 0;
			for (int t = 0; t + 2 < u; t += 3) {
				const int r = find(unstretched[t]);
				const int cnt = (comp_tris[r] += 1);
				if (cnt > best_count) {
					best_count = cnt;
					best_root = r;
				}
			}
			keep_min = MAX(24, (int)(best_count * 0.05f)); // others must clear this; the biggest is always kept
		}

		PackedInt32Array kept;
		kept.resize(u);
		int k = 0;
		const int *up = unstretched.ptr();
		for (int t = 0; t + 2 < u; t += 3) {
			if (p_keep_largest_component) {
				const int r = find(up[t]);
				if (r != best_root && comp_tris[r] < keep_min) {
					continue;
				}
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

void PhysXParticleFluid3D::_validate_property(PropertyInfo &p_property) const {
	// The MPM fluid path is block-sparse and boundless: the grid cell size is
	// 2 x particle_size and there is no box, so mpm_grid_resolution has no effect.
	// PhysXGranular3D's dense box grid still uses it (this runs for that subclass
	// too, via the validate chain, so gate on _is_granular()).
	if (p_property.name == SNAME("mpm_grid_resolution") && !_is_granular()) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
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
				// PBD is a physics actor (physics tick); the MPM compute solver is
				// a visual effect and runs per rendered frame with its own
				// substepping, like GPUParticles3D -- stepping it on the fixed
				// physics tick means physics catch-up runs it several times per
				// frame under load, which spirals.
				set_physics_process_internal(true);
				set_process_internal(true);
			}
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			set_physics_process_internal(false);
			set_process_internal(false);
			_editor_preview_exit();
			_free_fluid();
		} break;
		case NOTIFICATION_PREDELETE: {
			// Belt and braces: drop the MPM solver (and its local RenderingDevice)
			// while the rendering subsystem is still alive, in case EXIT_WORLD did
			// not run (e.g. freed without ever being parented).
			_free_fluid();
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			if (editor) {
				_editor_preview_step(get_process_delta_time());
				break;
			}
			// Granular MPM is a pure visual effect at high particle counts, so it
			// runs frame-rate driven (like GPUParticles3D) with its own
			// substepping -- stepping it on the fixed physics tick means physics
			// catch-up re-solves it several times per frame under load. The
			// fluid MPM path stays physics-tick driven so its rigid coupling
			// reaction stays in phase with the bodies it pushes.
			if (_mpm_path() && _is_granular()) {
				_mpm_accum += MIN(get_process_delta_time(), 1.0 / 20.0);
				if (_mpm_accum >= 1.0 / 60.0) {
					// Cap the solved step at ~2 physics ticks -- a stiff granular
					// plasticity solve blows up if a hitch feeds it a big dt.
					const double fdt = MIN(_mpm_accum, 1.0 / 30.0);
					_mpm_accum = 0.0;
					if (emitting) {
						_mpm_emit_step(fdt);
					}
					if (spawned) {
						_mpm_step(fdt);
					}
				}
			}
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			const bool mpm_here = _mpm_path() && !_is_granular();
			const double pdt = get_physics_process_delta_time();
			if (emitting) {
				if (mpm_here) {
					_mpm_emit_step(pdt);
				} else if (!_mpm_path()) {
					_emit_step(pdt);
				}
			}
			if (mpm_here) {
				if (spawned) {
					_mpm_step(pdt);
				}
			} else if (!_mpm_path() && (spawned || emitting)) {
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
	// This is a fake CPU-animated preview, not the real simulated fluid --
	// gi_mode is still a normal exported property, though, so Godot's own
	// GeometryInstance3D::set_gi_mode() already applied INSTANCE_FLAG_USE_
	// DYNAMIC_GI to this node's own instance (the one preview_multimesh just
	// became the base of) the moment the scene loaded, regardless of editor
	// vs runtime. Voxelizing a handful of small drifting preview spheres is
	// what actually triggers VoxelGI's dynamic-object pass to produce a
	// negative-size Rect2i and a garbage compute-dispatch count (reproduced
	// and confirmed by isolating this node's gi_mode in an otherwise-empty
	// editor session) -- force it back off here, since a fake preview has
	// nothing real to contribute to GI anyway.
	rs->instance_geometry_set_flag(get_instance(), RSE::INSTANCE_FLAG_USE_DYNAMIC_GI, false);
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

void PhysXParticleFluid3D::set_solver(SolverBackend p_solver) {
	if (solver == p_solver) {
		return;
	}
	const bool was_running = spawned || emitting;
	solver = p_solver;
	if (multimesh.is_valid() || fluid.is_valid()) {
		_free_fluid();
		if (is_inside_tree() && !Engine::get_singleton()->is_editor_hint()) {
			if (was_running && spawn_on_ready) {
				spawn();
			}
		}
	}
	update_configuration_warnings();
	notify_property_list_changed();
}

void PhysXParticleFluid3D::set_mpm_domain_size(const Vector3 &p_size) {
	mpm_domain_size = p_size.maxf(0.1);
	if (spawned && _mpm_path()) {
		_mpm_configure(!_mpm_emit_mode);
	}
	update_gizmos();
}

void PhysXParticleFluid3D::set_mpm_grid_resolution(int p_res) {
	mpm_grid_resolution = p_res <= 0 ? 0 : CLAMP(p_res, 8, 128); // 0 = auto from particle_size
	if (spawned && _mpm_path()) {
		_mpm_configure(!_mpm_emit_mode);
	}
}

void PhysXParticleFluid3D::set_mpm_substeps(int p_substeps) {
	mpm_substeps = CLAMP(p_substeps, 1, 40);
	if (spawned && _mpm_path()) {
		_mpm_configure(!_mpm_emit_mode);
	}
}

void PhysXParticleFluid3D::set_mpm_stiffness(float p_stiffness) {
	mpm_stiffness = MAX(p_stiffness, 1.0f);
	if (spawned && _mpm_path()) {
		_mpm_configure(!_mpm_emit_mode);
	}
}

void PhysXParticleFluid3D::set_mpm_colliders(const TypedArray<NodePath> &p_colliders) {
	mpm_colliders = p_colliders;
	_mpm_prev_pos.clear();
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
	if (mpm != nullptr) {
		Ref<SphereMesh> s = particle_mesh;
		if (s.is_valid()) {
			s->set_radius(particle_size * 0.5);
			s->set_height(particle_size);
		}
		if (spawned && mpm_grid_resolution <= 0) {
			_mpm_configure(!_mpm_emit_mode); // auto grid tracks particle_size -> full rebuild
		} else {
			_mpm_apply_surface_params(); // pinned grid -> just the isosurface params
		}
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
	if (spawned && _mpm_path() && !_mpm_emit_mode) {
		_mpm_configure(true); // the MPM prefill fills this region
	}
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

	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	const bool on_mpm = _resolved_solver() == SOLVER_MPM;

	const String engine = GLOBAL_GET("physics/3d/physics_engine");
	if (!on_mpm && engine != "PhysX" && engine != "DEFAULT") {
		warnings.push_back(RTR("The PBD backend only does anything when Project Settings > Physics > 3D > Physics Engine is set to \"PhysX\"."));
	}
	if (!on_mpm && !server) {
		warnings.push_back(RTR("The PhysX physics server is not active. Set the 3D physics engine to \"PhysX\", or set Solver to \"MPM (compute)\"."));
	}
	if (solver == SOLVER_PBD && server && !server->has_gpu()) {
		warnings.push_back(RTR("Solver is \"PBD (CUDA)\" but no CUDA device is available (or Enhanced Determinism is on) -- the fluid will be inert. Use \"Auto\" or \"MPM (compute)\"."));
	}
	if (on_mpm && (int)mpm_colliders.size() > 32) {
		warnings.push_back(RTR("MPM coupling uses at most 32 colliders (rigid bodies plus any nearby debris chunks); the rest are ignored."));
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
	ClassDB::bind_method(D_METHOD("get_mpm_step_msec"), &PhysXParticleFluid3D::get_mpm_step_msec);
	ClassDB::bind_method(D_METHOD("get_particle_positions"), &PhysXParticleFluid3D::get_particle_positions);
	ClassDB::bind_method(D_METHOD("get_submersion", "world_aabb"), &PhysXParticleFluid3D::get_submersion);

	ClassDB::bind_method(D_METHOD("set_solver", "solver"), &PhysXParticleFluid3D::set_solver);
	ClassDB::bind_method(D_METHOD("get_solver"), &PhysXParticleFluid3D::get_solver);
	ClassDB::bind_method(D_METHOD("set_mpm_domain_size", "size"), &PhysXParticleFluid3D::set_mpm_domain_size);
	ClassDB::bind_method(D_METHOD("get_mpm_domain_size"), &PhysXParticleFluid3D::get_mpm_domain_size);
	ClassDB::bind_method(D_METHOD("set_mpm_grid_resolution", "resolution"), &PhysXParticleFluid3D::set_mpm_grid_resolution);
	ClassDB::bind_method(D_METHOD("get_mpm_grid_resolution"), &PhysXParticleFluid3D::get_mpm_grid_resolution);
	ClassDB::bind_method(D_METHOD("set_mpm_substeps", "substeps"), &PhysXParticleFluid3D::set_mpm_substeps);
	ClassDB::bind_method(D_METHOD("get_mpm_substeps"), &PhysXParticleFluid3D::get_mpm_substeps);
	ClassDB::bind_method(D_METHOD("set_mpm_stiffness", "stiffness"), &PhysXParticleFluid3D::set_mpm_stiffness);
	ClassDB::bind_method(D_METHOD("get_mpm_stiffness"), &PhysXParticleFluid3D::get_mpm_stiffness);
	ClassDB::bind_method(D_METHOD("set_mpm_colliders", "colliders"), &PhysXParticleFluid3D::set_mpm_colliders);
	ClassDB::bind_method(D_METHOD("get_mpm_colliders"), &PhysXParticleFluid3D::get_mpm_colliders);
	ClassDB::bind_method(D_METHOD("set_mpm_auto_colliders", "enabled"), &PhysXParticleFluid3D::set_mpm_auto_colliders);
	ClassDB::bind_method(D_METHOD("is_mpm_auto_colliders"), &PhysXParticleFluid3D::is_mpm_auto_colliders);

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

	BIND_ENUM_CONSTANT(SOLVER_AUTO);
	BIND_ENUM_CONSTANT(SOLVER_PBD);
	BIND_ENUM_CONSTANT(SOLVER_MPM);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "solver", PROPERTY_HINT_ENUM, "Auto,PBD (CUDA),MPM (compute)"), "set_solver", "get_solver");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "particle_count", PROPERTY_HINT_RANGE, "1,262144,1"), "set_particle_count", "get_particle_count");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "particle_size", PROPERTY_HINT_RANGE, "0.01,1,0.001,suffix:m"), "set_particle_size", "get_particle_size");
	ADD_GROUP("MPM", "mpm_");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "mpm_domain_size", PROPERTY_HINT_NONE, "suffix:m"), "set_mpm_domain_size", "get_mpm_domain_size");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "mpm_grid_resolution", PROPERTY_HINT_RANGE, "0,128,1", PROPERTY_USAGE_DEFAULT), "set_mpm_grid_resolution", "get_mpm_grid_resolution"); // 0 = auto from particle_size
	ADD_PROPERTY(PropertyInfo(Variant::INT, "mpm_substeps", PROPERTY_HINT_RANGE, "1,40,1"), "set_mpm_substeps", "get_mpm_substeps");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mpm_stiffness", PROPERTY_HINT_RANGE, "1,40000,1,or_greater"), "set_mpm_stiffness", "get_mpm_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "mpm_colliders", PROPERTY_HINT_ARRAY_TYPE, "NodePath"), "set_mpm_colliders", "get_mpm_colliders");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "mpm_auto_colliders"), "set_mpm_auto_colliders", "is_mpm_auto_colliders");
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
	if (mpm != nullptr) {
		memdelete(mpm);
		mpm = nullptr;
	}
}

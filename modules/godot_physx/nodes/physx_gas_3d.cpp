/**************************************************************************/
/*  physx_gas_3d.cpp                                                      */
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

#include "physx_gas_3d.h"

#include "../particles/gas_solver.h"

#include "core/io/image.h"
#include "core/object/class_db.h"
#include "core/os/os.h"
#include "scene/3d/fog_volume.h"
#include "scene/3d/physics/collision_object_3d.h"
#include "scene/3d/physics/collision_shape_3d.h"
#include "scene/main/viewport.h"
#include "scene/resources/3d/fog_material.h"
#include "scene/resources/3d/primitive_meshes.h"
#include "scene/resources/3d/sphere_shape_3d.h"
#include "scene/resources/environment.h"
#include "scene/resources/image_texture.h"
#include "scene/resources/material.h"
#include "scene/resources/3d/world_3d.h"
#include "servers/rendering/rendering_server.h"

PhysXGas3D::~PhysXGas3D() {
	if (solver != nullptr) {
		memdelete(solver);
		solver = nullptr;
	}
}

void PhysXGas3D::set_domain_size(const Vector3 &p_size) {
	domain_size = p_size.maxf(0.1f);
	configured = false; // reconfigure next tick
}

void PhysXGas3D::set_cell_size(float p_size) {
	cell_size = MAX(p_size, 0.005f);
	configured = false;
}

void PhysXGas3D::set_volumetric_render(bool p_enabled) {
	volumetric_render = p_enabled;
	if (!volumetric_render && fog_volume != nullptr) {
		fog_volume->queue_free();
		fog_volume = nullptr;
		density_texture.unref();
		density_texture_dims = Vector3i();
	}
}

void PhysXGas3D::set_fog_density(float p_density) {
	fog_density = MAX(p_density, 0.0f);
	if (fog_material.is_valid()) {
		fog_material->set_density(fog_density);
	}
}

void PhysXGas3D::set_fog_albedo(const Color &p_color) {
	fog_albedo = p_color;
	if (fog_material.is_valid()) {
		fog_material->set_albedo(fog_albedo);
	}
}

void PhysXGas3D::set_debug_point_cloud(bool p_enabled) {
	debug_point_cloud = p_enabled;
	if (!debug_point_cloud && multimesh.is_valid()) {
		RenderingServer *rs = RenderingServer::get_singleton();
		if (mm_instance.is_valid()) {
			rs->free_rid(mm_instance);
			mm_instance = RID();
		}
		rs->free_rid(multimesh);
		multimesh = RID();
	}
}

void PhysXGas3D::_ensure_configured() {
	if (configured || solver == nullptr) {
		return;
	}
	// Round the domain up to whole 4-cell blocks, at least one block per axis.
	Vector3i box_blocks(
			MAX(1, (int)Math::ceil(domain_size.x / (cell_size * 4.0f))),
			MAX(1, (int)Math::ceil(domain_size.y / (cell_size * 4.0f))),
			MAX(1, (int)Math::ceil(domain_size.z / (cell_size * 4.0f))));

	GasSolver::Settings s;
	s.box_blocks = box_blocks;
	s.cell_size = cell_size;
	s.source_position = source_position;
	s.source_radius = source_radius;
	s.source_velocity = source_velocity;
	s.source_density = source_density;
	s.buoyancy = buoyancy;
	s.vorticity_strength = vorticity_strength;
	s.dissipation = dissipation;

	solver->configure(s, get_global_transform());
	configured = true;
}

// A CollisionShape3D (or CollisionObject3D with one) carrying a SphereShape3D
// gives its own world-scaled radius; anything else (a plain Node3D, or a
// shape this collider doesn't understand -- box/capsule/etc.) falls back to
// the shared collider_radius. Mirrors how PhysXParticleFluid3D's mpm_colliders
// resolves an analytic collider from a node, at a fraction of the scope (this
// solver only has a sphere obstacle, not a general shape catalog).
float PhysXGas3D::_resolve_collider_radius(Node3D *p_node) const {
	Ref<Shape3D> shape;
	if (CollisionShape3D *cshape = Object::cast_to<CollisionShape3D>(p_node)) {
		shape = cshape->get_shape();
	} else if (CollisionObject3D *co = Object::cast_to<CollisionObject3D>(p_node)) {
		List<uint32_t> owners;
		co->get_shape_owners(&owners);
		if (owners.size() > 0) {
			const uint32_t owner = owners.front()->get();
			if (co->shape_owner_get_shape_count(owner) > 0) {
				shape = co->shape_owner_get_shape(owner, 0);
			}
		}
	}
	Ref<SphereShape3D> sphere = shape;
	if (sphere.is_valid()) {
		const Vector3 scale = p_node->get_global_transform().basis.get_scale().abs();
		const float uniform_scale = MAX(scale.x, MAX(scale.y, scale.z));
		return MAX(sphere->get_radius() * uniform_scale, 0.01f);
	}
	return collider_radius;
}

void PhysXGas3D::_step(double p_delta) {
	if (solver == nullptr || !solver->is_available()) {
		return;
	}
	_ensure_configured();

	Vector<Vector3> collider_positions;
	Vector<float> collider_radii;
	for (int i = 0; i < colliders.size() && i < GasSolver::MAX_COLLIDERS; i++) {
		Node3D *n = Object::cast_to<Node3D>(get_node_or_null(colliders[i]));
		if (n == nullptr) {
			continue;
		}
		collider_positions.push_back(n->get_global_position());
		collider_radii.push_back(_resolve_collider_radius(n));
	}
	solver->set_colliders(collider_positions, collider_radii);

	solver->step(p_delta, get_global_transform());
	_update_render();
}

void PhysXGas3D::_update_render() {
	if (solver == nullptr) {
		return;
	}
	if (volumetric_render) {
		_ensure_fog_volume();
		_update_volumetric_render();
	}
	if (debug_point_cloud) {
		_ensure_point_cloud();
		_update_point_cloud_render();
	}
}

void PhysXGas3D::_ensure_fog_volume() {
	if (fog_volume != nullptr) {
		return;
	}
	fog_material.instantiate();
	fog_material->set_density(fog_density);
	fog_material->set_albedo(fog_albedo);

	fog_volume = memnew(FogVolume);
	fog_volume->set_shape(RSE::FOG_VOLUME_SHAPE_BOX);
	fog_volume->set_material(fog_material);
	// The solver's box is frozen in WORLD space at configure() (see
	// GasSolver::configure / _grow_if_needed) and does not follow this
	// node's transform -- top_level keeps the FogVolume's own position/size
	// in world space too, set directly from grid_anchor each update instead
	// of composing with this node's (possibly since-moved) transform.
	fog_volume->set_as_top_level(true);
	add_child(fog_volume, false, INTERNAL_MODE_BACK);
}

void PhysXGas3D::_update_volumetric_render() {
	Vector<float> density;
	Vector3i dims;
	Vector3 anchor;
	float grid_cell_size = cell_size;
	solver->get_density_grid(density, dims, anchor, grid_cell_size);
	if (dims.x <= 0 || dims.y <= 0 || dims.z <= 0) {
		return;
	}

	// One Image slice per Z layer (dims.x * dims.y each) -- ImageTexture3D's
	// native layout.
	Vector<Ref<Image>> slices;
	slices.resize(dims.z);
	const int slice_floats = dims.x * dims.y;
	for (int z = 0; z < dims.z; z++) {
		Vector<uint8_t> bytes;
		bytes.resize(slice_floats * (int)sizeof(float));
		memcpy(bytes.ptrw(), density.ptr() + z * slice_floats, bytes.size());
		slices.write[z] = Image::create_from_data(dims.x, dims.y, false, Image::FORMAT_RF, bytes);
	}

	if (density_texture.is_null() || density_texture_dims != dims) {
		density_texture.instantiate();
		density_texture->create(Image::FORMAT_RF, dims.x, dims.y, dims.z, false, slices);
		density_texture_dims = dims;
		fog_material->set_density_texture(density_texture);
	} else {
		density_texture->update(slices);
	}

	const Vector3 world_size = Vector3(dims) * grid_cell_size;
	fog_volume->set_size(world_size);
	fog_volume->set_position(anchor + world_size * 0.5f);
}

void PhysXGas3D::_ensure_point_cloud() {
	if (multimesh.is_valid()) {
		return;
	}
	Ref<BoxMesh> box;
	box.instantiate();
	box->set_size(Vector3(1, 1, 1) * (cell_size * 0.9f));
	Ref<StandardMaterial3D> mat;
	mat.instantiate();
	mat->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	mat->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
	mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	box->surface_set_material(0, mat);
	cell_mesh = box;

	RenderingServer *rs = RenderingServer::get_singleton();
	// Sized for the largest domain a 96-block box at 4 cells/block could
	// hold; get_render_cells() clamps to whatever is actually allocated,
	// this just bounds the MultiMesh instance buffer.
	const int cap = 128 * 128 * 128;
	multimesh = rs->multimesh_create();
	rs->multimesh_allocate_data(multimesh, cap, RSE::MULTIMESH_TRANSFORM_3D, true);
	rs->multimesh_set_mesh(multimesh, cell_mesh->get_rid());
	rs->multimesh_set_visible_instances(multimesh, 0);

	// World-space positions are written directly into the instance
	// transforms (see _update_point_cloud_render), so the RS instance itself
	// never moves -- same pattern as the fluid node's foam layer.
	mm_instance = rs->instance_create2(multimesh, get_world_3d()->get_scenario());
	rs->instance_set_transform(mm_instance, Transform3D());
	rs->instance_set_custom_aabb(mm_instance, AABB(Vector3(-100000, -100000, -100000), Vector3(200000, 200000, 200000)));
}

void PhysXGas3D::_update_point_cloud_render() {
	if (multimesh.is_null()) {
		return;
	}
	Vector<Vector3> positions;
	Vector<float> density;
	solver->get_render_cells(render_threshold, positions, density);

	RenderingServer *rs = RenderingServer::get_singleton();
	const int n = MIN(positions.size(), rs->multimesh_get_instance_count(multimesh));
	for (int i = 0; i < n; i++) {
		rs->multimesh_instance_set_transform(multimesh, i, Transform3D(Basis(), positions[i]));
		const float a = CLAMP(density[i], 0.0f, 1.0f);
		// Dim-blue -> white heat-ish ramp so the density gradient actually
		// reads, instead of every cell above the threshold looking equally
		// opaque (the prototype's first pass, before this ramp).
		rs->multimesh_instance_set_color(multimesh, i, Color(0.5f + 0.5f * a, 0.6f + 0.4f * a, 1.0f, CLAMP(a * 1.4f, 0.0f, 1.0f)));
	}
	rs->multimesh_set_visible_instances(multimesh, n);
}

void PhysXGas3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			if (solver == nullptr) {
				solver = memnew(GasSolver);
			}
			// The FogVolume and the point-cloud MultiMesh are both created
			// lazily, in _update_render, gated on volumetric_render /
			// debug_point_cloud -- no point paying for either until the
			// corresponding render path is actually enabled.
			set_physics_process_internal(true);
		} break;
		case NOTIFICATION_EXIT_WORLD: {
			set_physics_process_internal(false);
			if (mm_instance.is_valid()) {
				RenderingServer::get_singleton()->free_rid(mm_instance);
				mm_instance = RID();
			}
			if (multimesh.is_valid()) {
				RenderingServer::get_singleton()->free_rid(multimesh);
				multimesh = RID();
			}
			if (fog_volume != nullptr) {
				fog_volume->queue_free();
				fog_volume = nullptr;
			}
			density_texture.unref();
			density_texture_dims = Vector3i();
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			_step(get_physics_process_delta_time());
		} break;
	}
}

PackedStringArray PhysXGas3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();
	if (!volumetric_render) {
		return warnings;
	}
	if (OS::get_singleton()->get_current_rendering_method() != "forward_plus") {
		warnings.push_back(RTR("PhysXGas3D's volumetric render needs the Forward+ renderer (it draws through a FogVolume)."));
		return warnings;
	}
	if (is_inside_tree() && get_viewport() != nullptr && get_viewport()->find_world_3d().is_valid()) {
		Ref<Environment> environment = get_viewport()->find_world_3d()->get_environment();
		if (environment.is_valid() && !environment->is_volumetric_fog_enabled()) {
			warnings.push_back(RTR("PhysXGas3D needs volumetric fog enabled in the scene's Environment to be visible (it renders through a FogVolume) -- otherwise it silently draws nothing. Turn off Volumetric Render to use the debug point cloud instead."));
		}
	}
	return warnings;
}

void PhysXGas3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_domain_size", "size"), &PhysXGas3D::set_domain_size);
	ClassDB::bind_method(D_METHOD("get_domain_size"), &PhysXGas3D::get_domain_size);
	ClassDB::bind_method(D_METHOD("set_cell_size", "size"), &PhysXGas3D::set_cell_size);
	ClassDB::bind_method(D_METHOD("get_cell_size"), &PhysXGas3D::get_cell_size);
	ClassDB::bind_method(D_METHOD("set_source_position", "position"), &PhysXGas3D::set_source_position);
	ClassDB::bind_method(D_METHOD("get_source_position"), &PhysXGas3D::get_source_position);
	ClassDB::bind_method(D_METHOD("set_source_radius", "radius"), &PhysXGas3D::set_source_radius);
	ClassDB::bind_method(D_METHOD("get_source_radius"), &PhysXGas3D::get_source_radius);
	ClassDB::bind_method(D_METHOD("set_source_velocity", "velocity"), &PhysXGas3D::set_source_velocity);
	ClassDB::bind_method(D_METHOD("get_source_velocity"), &PhysXGas3D::get_source_velocity);
	ClassDB::bind_method(D_METHOD("set_source_density", "density"), &PhysXGas3D::set_source_density);
	ClassDB::bind_method(D_METHOD("get_source_density"), &PhysXGas3D::get_source_density);
	ClassDB::bind_method(D_METHOD("set_buoyancy", "buoyancy"), &PhysXGas3D::set_buoyancy);
	ClassDB::bind_method(D_METHOD("get_buoyancy"), &PhysXGas3D::get_buoyancy);
	ClassDB::bind_method(D_METHOD("set_vorticity_strength", "strength"), &PhysXGas3D::set_vorticity_strength);
	ClassDB::bind_method(D_METHOD("get_vorticity_strength"), &PhysXGas3D::get_vorticity_strength);
	ClassDB::bind_method(D_METHOD("set_dissipation", "dissipation"), &PhysXGas3D::set_dissipation);
	ClassDB::bind_method(D_METHOD("get_dissipation"), &PhysXGas3D::get_dissipation);
	ClassDB::bind_method(D_METHOD("set_colliders", "paths"), &PhysXGas3D::set_colliders);
	ClassDB::bind_method(D_METHOD("get_colliders"), &PhysXGas3D::get_colliders);
	ClassDB::bind_method(D_METHOD("set_collider_radius", "radius"), &PhysXGas3D::set_collider_radius);
	ClassDB::bind_method(D_METHOD("get_collider_radius"), &PhysXGas3D::get_collider_radius);
	ClassDB::bind_method(D_METHOD("set_volumetric_render", "enabled"), &PhysXGas3D::set_volumetric_render);
	ClassDB::bind_method(D_METHOD("get_volumetric_render"), &PhysXGas3D::get_volumetric_render);
	ClassDB::bind_method(D_METHOD("set_fog_density", "density"), &PhysXGas3D::set_fog_density);
	ClassDB::bind_method(D_METHOD("get_fog_density"), &PhysXGas3D::get_fog_density);
	ClassDB::bind_method(D_METHOD("set_fog_albedo", "color"), &PhysXGas3D::set_fog_albedo);
	ClassDB::bind_method(D_METHOD("get_fog_albedo"), &PhysXGas3D::get_fog_albedo);
	ClassDB::bind_method(D_METHOD("set_debug_point_cloud", "enabled"), &PhysXGas3D::set_debug_point_cloud);
	ClassDB::bind_method(D_METHOD("get_debug_point_cloud"), &PhysXGas3D::get_debug_point_cloud);
	ClassDB::bind_method(D_METHOD("set_render_threshold", "threshold"), &PhysXGas3D::set_render_threshold);
	ClassDB::bind_method(D_METHOD("get_render_threshold"), &PhysXGas3D::get_render_threshold);

	ADD_GROUP("Domain", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "domain_size", PROPERTY_HINT_NONE, "suffix:m"), "set_domain_size", "get_domain_size");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cell_size", PROPERTY_HINT_RANGE, "0.01,0.5,0.005,suffix:m"), "set_cell_size", "get_cell_size");
	ADD_GROUP("Source", "source_");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "source_position", PROPERTY_HINT_NONE, "suffix:m"), "set_source_position", "get_source_position");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "source_radius", PROPERTY_HINT_RANGE, "0.01,2.0,0.01,suffix:m"), "set_source_radius", "get_source_radius");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "source_velocity", PROPERTY_HINT_NONE, "suffix:m/s"), "set_source_velocity", "get_source_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "source_density", PROPERTY_HINT_RANGE, "0.0,4.0,0.01"), "set_source_density", "get_source_density");
	ADD_GROUP("Motion", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "buoyancy", PROPERTY_HINT_RANGE, "-20.0,20.0,0.1"), "set_buoyancy", "get_buoyancy");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vorticity_strength", PROPERTY_HINT_RANGE, "0.0,30.0,0.1"), "set_vorticity_strength", "get_vorticity_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dissipation", PROPERTY_HINT_RANGE, "0.9,1.0,0.0005"), "set_dissipation", "get_dissipation");
	ADD_GROUP("Collider", "collider");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "colliders", PROPERTY_HINT_ARRAY_TYPE, "NodePath"), "set_colliders", "get_colliders");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collider_radius", PROPERTY_HINT_RANGE, "0.0,3.0,0.01,suffix:m"), "set_collider_radius", "get_collider_radius");
	ADD_GROUP("Rendering", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "volumetric_render"), "set_volumetric_render", "get_volumetric_render");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fog_density", PROPERTY_HINT_RANGE, "0.0,32.0,0.1"), "set_fog_density", "get_fog_density");
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "fog_albedo"), "set_fog_albedo", "get_fog_albedo");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "debug_point_cloud"), "set_debug_point_cloud", "get_debug_point_cloud");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "render_threshold", PROPERTY_HINT_RANGE, "0.0,1.0,0.005"), "set_render_threshold", "get_render_threshold");
}

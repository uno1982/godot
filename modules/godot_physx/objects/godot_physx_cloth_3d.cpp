/**************************************************************************/
/*  godot_physx_cloth_3d.cpp                                              */
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

#include "godot_physx_cloth_3d.h"

#include "../godot_physx_conversions.h"
#include "../spaces/godot_physx_space_3d.h"

#include "core/error/error_macros.h"
#include "core/math/math_funcs.h"

#include <PxPhysicsAPI.h>
#include <extensions/PxCudaHelpersExt.h>
#include <extensions/PxDeformableSurfaceExt.h>

using namespace physx;

GodotPhysXCloth3D::~GodotPhysXCloth3D() {
	set_space(nullptr);
}

void GodotPhysXCloth3D::set_space(GodotPhysXSpace3D *p_space) {
	if (space == p_space) {
		return;
	}
	clear();
	if (space) {
		space->unregister_cloth(this);
	}
	space = p_space;
	if (space) {
		space->register_cloth(this);
	}
}

void GodotPhysXCloth3D::clear() {
	_destroy_surface();
	MutexLock lock(mesh_mutex);
	read_positions.clear();
	indices.clear();
	vertex_count = 0;
	mesh_version++;
}

void GodotPhysXCloth3D::_destroy_surface() {
	if (space && space->get_px_scene() && surface) {
		space->get_px_scene()->removeActor(*surface);
	}
	if (surface) {
		surface->release();
		surface = nullptr;
	}
	if (shape) {
		shape->release();
		shape = nullptr;
	}
	if (tri_mesh) {
		tri_mesh->release();
		tri_mesh = nullptr;
	}
	if (material) {
		material->release();
		material = nullptr;
	}
	if (cuda) {
		if (host_pos) {
			PxVec4 *hp = static_cast<PxVec4 *>(host_pos);
			PX_EXT_PINNED_MEMORY_FREE(*cuda, hp);
			host_pos = nullptr;
		}
		if (host_vel) {
			PxVec4 *hv = static_cast<PxVec4 *>(host_vel);
			PX_EXT_PINNED_MEMORY_FREE(*cuda, hv);
			host_vel = nullptr;
		}
	}
	pinned.clear();
	pin_target.clear();
}

void GodotPhysXCloth3D::set_params(float p_thickness, float p_density, float p_stretch, float p_bend, float p_damping, uint32_t p_collision_mask) {
	thickness = MAX(p_thickness, 1.0e-4f);
	density = MAX(p_density, 1.0e-3f);
	stretch_stiffness = CLAMP(p_stretch, 0.0f, 1.0f);
	bend_stiffness = CLAMP(p_bend, 0.0f, 1.0f);
	damping = MAX(p_damping, 0.0f);
	collision_mask = p_collision_mask;
	params_dirty = true;
	if (surface) {
		_apply_material();
	}
}

void GodotPhysXCloth3D::_apply_material() {
	if (!material || !surface) {
		return;
	}
	// stretch 0..1 -> Young's modulus; bend 0..1 -> bending stiffness.
	const float youngs = Math::lerp(2.0e4f, 5.0e9f, stretch_stiffness * stretch_stiffness);
	const float bending = Math::lerp(1.0e-6f, 5.0e-2f, bend_stiffness);
	material->setYoungsModulus(youngs);
	material->setBendingStiffness(bending);
	material->setThickness(thickness);
	surface->setLinearDamping(damping);
	params_dirty = false;
}

bool GodotPhysXCloth3D::_cook_and_create(const Vector<Vector3> &p_positions, const Vector<int32_t> &p_indices) {
	PxPhysics *physics = space->get_px_physics();
	PxScene *scene = space->get_px_scene();
	cuda = space->get_px_cuda();
	if (!physics || !scene || !cuda) {
		return false;
	}
	const int nv = p_positions.size();
	const int ni = p_indices.size();
	if (nv < 3 || ni < 3 || (ni % 3) != 0) {
		return false;
	}

	// The surface simulates directly in world space (identity pose), so the
	// node's transform -- scale included -- is baked into these vertices.
	LocalVector<PxVec3> verts;
	verts.resize(nv);
	for (int i = 0; i < nv; i++) {
		verts[i] = to_px(p_positions[i]);
	}
	LocalVector<PxU32> tris;
	tris.resize(ni);
	for (int i = 0; i < ni; i++) {
		tris[i] = (PxU32)p_indices[i];
	}

	PxCookingParams cook(physics->getTolerancesScale());
	cook.buildGPUData = true;
	cook.meshWeldTolerance = 1.0e-5f;
	cook.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES) | PxMeshPreprocessingFlag::eENABLE_VERT_MAPPING;
	cook.midphaseDesc = PxMeshMidPhase::eBVH34;

	PxTriangleMeshDesc desc;
	desc.points.count = (PxU32)nv;
	desc.points.stride = sizeof(PxVec3);
	desc.points.data = verts.ptr();
	desc.triangles.count = (PxU32)(ni / 3);
	desc.triangles.stride = 3 * sizeof(PxU32);
	desc.triangles.data = tris.ptr();

	tri_mesh = PxCreateTriangleMesh(cook, desc, physics->getPhysicsInsertionCallback());
	ERR_FAIL_NULL_V_MSG(tri_mesh, false, "PhysX: cloth triangle mesh cook failed.");

	surface = physics->createDeformableSurface(*cuda);
	ERR_FAIL_NULL_V_MSG(surface, false, "PhysX: createDeformableSurface failed.");

	material = physics->createDeformableSurfaceMaterial(1.0e8f, 0.3f, 0.5f, thickness, 1.0e-4f);
	const PxShapeFlags shape_flags = PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;
	shape = physics->createShape(PxTriangleMeshGeometry(tri_mesh), &material, 1, true, shape_flags);
	ERR_FAIL_NULL_V_MSG(shape, false, "PhysX: cloth shape creation failed.");
	PxFilterData fd(collision_mask, collision_mask, 0, 0);
	shape->setSimulationFilterData(fd);
	shape->setQueryFilterData(fd);
	shape->setContactOffset(2.0f * thickness);
	shape->setRestOffset(thickness);
	surface->attachShape(*shape);
	surface->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, true);
	surface->setSelfCollisionFilterDistance(thickness * 2.5f);
	surface->setMaxLinearVelocity(500.0f);

	scene->addActor(*surface);

	// Host mirror. The cooked mesh may have reordered/welded vertices; use its
	// own vertices as the authoritative rest set.
	const PxU32 cooked_nv = tri_mesh->getNbVertices();
	const PxVec3 *cooked_v = tri_mesh->getVertices();
	LocalVector<PxVec3> zero_vel;
	zero_vel.resize(cooked_nv);
	for (PxU32 i = 0; i < cooked_nv; i++) {
		zero_vel[i] = PxVec3(0.0f);
	}

	PxVec4 *hp = nullptr;
	PxVec4 *hv = nullptr;
	PxVec4 *rest_pinned = nullptr;
	vertex_count = PxDeformableSurfaceExt::allocateAndInitializeHostMirror(*surface,
			cooked_v, zero_vel.ptr(), cooked_v, 1.0f, PxTransform(PxIdentity), cuda,
			hp, hv, rest_pinned);
	host_pos = hp;
	host_vel = hv;

	// Areal density: distributeDensityToVertices multiplies by thickness, so pass
	// density / thickness to end up with kg/m^2.
	PxDeformableSurfaceExt::distributeDensityToVertices(*surface, density / thickness, thickness, hp);

	// Pins: zero the inverse mass of pinned vertices.
	pinned.resize(vertex_count);
	pin_target.resize(vertex_count);
	for (uint32_t i = 0; i < vertex_count; i++) {
		pinned[i] = 0;
		pin_target[i] = Vector3(NAN, 0, 0);
	}

	PxDeformableSurfaceExt::copyToDevice(*surface, PxDeformableSurfaceDataFlag::eALL, vertex_count, hp, hv, rest_pinned);
	if (rest_pinned) {
		PX_EXT_PINNED_MEMORY_FREE(*cuda, rest_pinned);
	}

	// Cooked triangle list for the node's render mesh.
	const PxU32 nt = tri_mesh->getNbTriangles();
	const void *tri_data = tri_mesh->getTriangles();
	const bool u16 = tri_mesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
	indices.resize(nt * 3);
	for (PxU32 i = 0; i < nt * 3; i++) {
		indices[i] = u16 ? (int32_t)((const PxU16 *)tri_data)[i] : (int32_t)((const PxU32 *)tri_data)[i];
	}

	_apply_material();

	// Publish the initial (rest) mesh from the host mirror -- the GPU device
	// buffers are not valid until the surface has been simulated once.
	simulated_once = false;
	{
		MutexLock lock(mesh_mutex);
		read_positions.resize(vertex_count);
		for (uint32_t i = 0; i < vertex_count; i++) {
			read_positions[i] = Vector3(hp[i].x, hp[i].y, hp[i].z);
		}
		mesh_version++;
	}
	return true;
}

void GodotPhysXCloth3D::build(const Vector<Vector3> &p_positions, const Vector<int32_t> &p_indices, const Transform3D &p_xform) {
	(void)p_xform; // the positions are already world-space
	_destroy_surface();
	if (!space) {
		return;
	}
	if (!_cook_and_create(p_positions, p_indices)) {
		_destroy_surface();
	}
}

void GodotPhysXCloth3D::set_pinned(const Vector<int32_t> &p_pinned_indices) {
	if (!surface || pinned.is_empty()) {
		return;
	}
	// Inverse mass does not change during simulation, so the host mirror stays
	// authoritative -- no device readback needed. Recompute the base mass from
	// density (clears old pins) and zero the inverse mass of the new pinned set.
	PxVec4 *hp = static_cast<PxVec4 *>(host_pos);
	PxVec4 *hv = static_cast<PxVec4 *>(host_vel);
	PxDeformableSurfaceExt::distributeDensityToVertices(*surface, density / thickness, thickness, hp);
	for (uint32_t i = 0; i < vertex_count; i++) {
		pinned[i] = 0;
		pin_target[i] = Vector3(NAN, 0, 0);
	}
	for (int k = 0; k < p_pinned_indices.size(); k++) {
		const int v = p_pinned_indices[k];
		if (v >= 0 && v < (int)vertex_count) {
			pinned[v] = 1;
			hp[v].w = 0.0f;
		}
	}
	if (simulated_once) {
		Ext::PxCudaHelpersExt::copyHToD(*cuda, surface->getPositionInvMassBufferD(), hp, vertex_count);
		surface->markDirty(PxDeformableSurfaceDataFlag::ePOSITION_INVMASS);
	} else {
		PxDeformableSurfaceExt::copyToDevice(*surface, PxDeformableSurfaceDataFlag::ePOSITION_INVMASS, vertex_count, hp, hv, nullptr);
	}
}

void GodotPhysXCloth3D::set_pin_targets(const Vector<Vector3> &p_world_targets) {
	if (pin_target.is_empty()) {
		return;
	}
	const int n = MIN(p_world_targets.size(), (int)vertex_count);
	for (int i = 0; i < n; i++) {
		pin_target[i] = p_world_targets[i];
	}
}

void GodotPhysXCloth3D::apply_wind(const Vector3 &p_wind, float p_drag, float p_lift, float p_dt) {
	if (!surface || vertex_count == 0 || p_dt <= 0.0f || !simulated_once) {
		return;
	}
	// hp / hv hold the state from the last read_back (== start of this step).
	// Accumulate a per-triangle aerodynamic impulse into the velocities, honor
	// any moving pins, and push both back to the device.
	PxVec4 *hp = static_cast<PxVec4 *>(host_pos);
	PxVec4 *hv = static_cast<PxVec4 *>(host_vel);
	bool moved_pin = false;
	for (uint32_t i = 0; i < vertex_count; i++) {
		if (pinned[i] && !Math::is_nan((float)pin_target[i].x)) {
			const Vector3 &t = pin_target[i];
			hp[i].x = t.x;
			hp[i].y = t.y;
			hp[i].z = t.z;
			hv[i] = PxVec4(0.0f);
			moved_pin = true;
		}
	}

	if ((p_drag > 0.0f || p_lift > 0.0f) && p_wind.length_squared() >= 0.0f) {
		const PxVec3 wind = to_px(p_wind);
		for (uint32_t t = 0; t + 2 < indices.size(); t += 3) {
			const uint32_t a = indices[t];
			const uint32_t b = indices[t + 1];
			const uint32_t c = indices[t + 2];
			const PxVec3 pa = hp[a].getXYZ();
			const PxVec3 e1 = hp[b].getXYZ() - pa;
			const PxVec3 e2 = hp[c].getXYZ() - pa;
			PxVec3 cr = e1.cross(e2);
			const float twice_area = cr.magnitude();
			if (twice_area < 1.0e-9f) {
				continue;
			}
			const PxVec3 n = cr / twice_area;
			const float area = 0.5f * twice_area;
			const PxVec3 v_tri = (hv[a].getXYZ() + hv[b].getXYZ() + hv[c].getXYZ()) * (1.0f / 3.0f);
			const PxVec3 rel = wind - v_tri;
			const float vn = n.dot(rel);
			const PxVec3 tangential = rel - n * vn;
			const PxVec3 force = n * (p_drag * area * vn * PxAbs(vn)) + tangential * (p_lift * area * PxAbs(vn));
			const PxVec3 dv = force * (p_dt / 3.0f);
			for (uint32_t vi : { a, b, c }) {
				const float w = hp[vi].w;
				hv[vi].x += dv.x * w;
				hv[vi].y += dv.y * w;
				hv[vi].z += dv.z * w;
			}
		}
	}

	Ext::PxCudaHelpersExt::copyHToD(*cuda, surface->getVelocityBufferD(), hv, vertex_count);
	surface->markDirty(PxDeformableSurfaceDataFlag::eVELOCITY);
	if (moved_pin) {
		Ext::PxCudaHelpersExt::copyHToD(*cuda, surface->getPositionInvMassBufferD(), hp, vertex_count);
		surface->markDirty(PxDeformableSurfaceDataFlag::ePOSITION_INVMASS);
	}
}

void GodotPhysXCloth3D::read_back() {
	if (!surface || vertex_count == 0) {
		return;
	}
	simulated_once = true; // read_back only runs after fetchResults()
	PxVec4 *hp = static_cast<PxVec4 *>(host_pos);
	PxVec4 *hv = static_cast<PxVec4 *>(host_vel);
	Ext::PxCudaHelpersExt::copyDToH(*cuda, hp, surface->getPositionInvMassBufferD(), vertex_count);
	Ext::PxCudaHelpersExt::copyDToH(*cuda, hv, surface->getVelocityBufferD(), vertex_count);

	MutexLock lock(mesh_mutex);
	read_positions.resize(vertex_count);
	for (uint32_t i = 0; i < vertex_count; i++) {
		read_positions[i] = Vector3(hp[i].x, hp[i].y, hp[i].z);
	}
	mesh_version++;
}

uint32_t GodotPhysXCloth3D::copy_mesh(LocalVector<Vector3> &r_positions, LocalVector<int32_t> &r_indices, uint32_t &p_have_version) const {
	MutexLock lock(mesh_mutex);
	if (p_have_version == mesh_version) {
		return UINT32_MAX;
	}
	p_have_version = mesh_version;
	r_positions = read_positions;
	r_indices = indices;
	return indices.size() / 3;
}

/**************************************************************************/
/*  godot_physx_shape_3d.cpp                                              */
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

#include "godot_physx_shape_3d.h"

#include "../godot_physx_conversions.h"
#include "../godot_physx_server_3d.h"

#include "core/error/error_macros.h"
#include "core/templates/local_vector.h"

#include <PxPhysicsAPI.h>
#include <cooking/PxCooking.h>

using namespace physx;

const PxGeometry &GodotPhysXShapeGeometry::geometry() const {
	switch (type) {
		case PxGeometryType::eBOX:
			return box;
		case PxGeometryType::eSPHERE:
			return sphere;
		case PxGeometryType::eCAPSULE:
			return capsule;
		case PxGeometryType::ePLANE:
			return plane;
		case PxGeometryType::eCONVEXMESH:
			return convex;
		case PxGeometryType::eTRIANGLEMESH:
			return trimesh;
		default:
			return box;
	}
}

GodotPhysXShape3D::~GodotPhysXShape3D() {
	_release_meshes();
}

void GodotPhysXShape3D::_release_meshes() {
	if (convex_mesh) {
		convex_mesh->release();
		convex_mesh = nullptr;
	}
	if (triangle_mesh) {
		triangle_mesh->release();
		triangle_mesh = nullptr;
	}
}

void GodotPhysXShape3D::set_data(const Variant &p_data) {
	data = p_data;
	geom_valid = false;
	geom = GodotPhysXShapeGeometry();

	switch (type) {
		case PhysicsServer3D::SHAPE_SPHERE: {
			const real_t radius = p_data;
			ERR_FAIL_COND(radius <= 0.0);
			geom.type = PxGeometryType::eSPHERE;
			geom.sphere = PxSphereGeometry((PxReal)radius);
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_BOX: {
			const Vector3 half_extents = p_data;
			ERR_FAIL_COND(half_extents.x <= 0.0 || half_extents.y <= 0.0 || half_extents.z <= 0.0);
			geom.type = PxGeometryType::eBOX;
			geom.box = PxBoxGeometry(to_px(half_extents));
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_CAPSULE: {
			const Dictionary d = p_data;
			ERR_FAIL_COND(!d.has("radius") || !d.has("height"));
			const real_t radius = d["radius"];
			const real_t height = d["height"];
			ERR_FAIL_COND(radius <= 0.0 || height <= 0.0);
			// Godot capsule height is the full height including both hemispheres
			// and its axis is +Y. PhysX capsules use a cylinder half-height
			// (excluding caps) with a +X axis, so rotate -90 deg about Z.
			const real_t half_cyl = MAX((real_t)0.0, (height - 2.0 * radius) * 0.5);
			geom.type = PxGeometryType::eCAPSULE;
			geom.capsule = PxCapsuleGeometry((PxReal)radius, (PxReal)half_cyl);
			geom.local_pose = PxTransform(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_WORLD_BOUNDARY: {
			const Plane plane = p_data;
			geom.type = PxGeometryType::ePLANE;
			geom.plane = PxPlaneGeometry();
			// PxPlaneGeometry is fixed at the YZ plane (normal +X). Orient/position
			// it to match the Godot plane.
			const PxPlane px_plane(to_px(plane.normal), (PxReal)-plane.d);
			geom.local_pose = PxTransformFromPlaneEquation(px_plane);
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_CONVEX_POLYGON: {
			const Vector<Vector3> points = p_data;
			ERR_FAIL_COND(points.size() < 4);
			PxPhysics *physics = GodotPhysXServer3D::get_singleton() ? GodotPhysXServer3D::get_singleton()->get_px_physics() : nullptr;
			ERR_FAIL_NULL(physics);

			LocalVector<PxVec3> px_points;
			px_points.resize(points.size());
			for (int i = 0; i < points.size(); i++) {
				px_points[i] = to_px(points[i]);
			}

			PxConvexMeshDesc desc;
			desc.points.count = px_points.size();
			desc.points.stride = sizeof(PxVec3);
			desc.points.data = px_points.ptr();
			desc.flags = PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eDISABLE_MESH_VALIDATION | PxConvexFlag::eFAST_INERTIA_COMPUTATION;

			PxCookingParams params(physics->getTolerancesScale());
#ifdef GODOT_PHYSX_GPU
			params.buildGPUData = true; // required for GPU dynamics to collide against the mesh
#endif
			_release_meshes();
			PxConvexMeshCookingResult::Enum cvx_result = PxConvexMeshCookingResult::eSUCCESS;
			convex_mesh = PxCreateConvexMesh(params, desc, *PxGetStandaloneInsertionCallback(), &cvx_result);
			ERR_FAIL_NULL_MSG(convex_mesh, vformat("PhysX: convex hull cooking failed (result %d).", (int)cvx_result));
			geom.type = PxGeometryType::eCONVEXMESH;
			geom.convex = PxConvexMeshGeometry(convex_mesh);
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_CONCAVE_POLYGON: {
			Vector<Vector3> faces;
			if (p_data.get_type() == Variant::DICTIONARY) {
				faces = ((Dictionary)p_data).get("faces", Vector<Vector3>());
			} else {
				faces = p_data;
			}
			ERR_FAIL_COND(faces.size() < 3 || faces.size() % 3 != 0);
			PxPhysics *physics = GodotPhysXServer3D::get_singleton() ? GodotPhysXServer3D::get_singleton()->get_px_physics() : nullptr;
			ERR_FAIL_NULL(physics);

			const int tri_count = faces.size() / 3;
			LocalVector<PxVec3> verts;
			LocalVector<PxU32> indices;
			verts.resize(faces.size());
			indices.resize(faces.size());
			for (int i = 0; i < faces.size(); i++) {
				verts[i] = to_px(faces[i]);
			}
			// Godot winds triangles clockwise (front face); PhysX expects
			// counter-clockwise, so swap the last two indices of each triangle.
			for (int t = 0; t < tri_count; t++) {
				indices[t * 3 + 0] = t * 3 + 0;
				indices[t * 3 + 1] = t * 3 + 2;
				indices[t * 3 + 2] = t * 3 + 1;
			}

			PxTriangleMeshDesc desc;
			desc.points.count = verts.size();
			desc.points.stride = sizeof(PxVec3);
			desc.points.data = verts.ptr();
			desc.triangles.count = tri_count;
			desc.triangles.stride = 3 * sizeof(PxU32);
			desc.triangles.data = indices.ptr();

			PxCookingParams params(physics->getTolerancesScale());
			params.meshWeldTolerance = 0.001f;
			params.meshPreprocessParams |= PxMeshPreprocessingFlag::eWELD_VERTICES;
#ifdef GODOT_PHYSX_GPU
			params.buildGPUData = true; // required for GPU dynamics to collide against the mesh
#endif
			_release_meshes();
			PxTriangleMeshCookingResult::Enum tri_result = PxTriangleMeshCookingResult::eSUCCESS;
			triangle_mesh = PxCreateTriangleMesh(params, desc, *PxGetStandaloneInsertionCallback(), &tri_result);
			ERR_FAIL_NULL_MSG(triangle_mesh, vformat("PhysX: triangle mesh cooking failed (result %d, %d tris).", (int)tri_result, tri_count));
			print_verbose(vformat("PhysX: cooked triangle mesh (%d tris, result %d).", tri_count, (int)tri_result));
			geom.type = PxGeometryType::eTRIANGLEMESH;
			geom.trimesh = PxTriangleMeshGeometry(triangle_mesh);
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_CYLINDER: {
			// PhysX has no native cylinder; approximate with a convex prism.
			const Dictionary d = p_data;
			ERR_FAIL_COND(!d.has("radius") || !d.has("height"));
			const real_t radius = d["radius"];
			const real_t half_h = (real_t)d["height"] * 0.5;
			ERR_FAIL_COND(radius <= 0.0 || half_h <= 0.0);
			PxPhysics *physics = GodotPhysXServer3D::get_singleton() ? GodotPhysXServer3D::get_singleton()->get_px_physics() : nullptr;
			ERR_FAIL_NULL(physics);

			const int sides = 16;
			LocalVector<PxVec3> pts;
			pts.resize(sides * 2);
			for (int i = 0; i < sides; i++) {
				const float a = (float)i / sides * (float)Math::TAU;
				const PxReal cx = Math::cos(a) * (PxReal)radius;
				const PxReal cz = Math::sin(a) * (PxReal)radius;
				pts[i * 2 + 0] = PxVec3(cx, (PxReal)half_h, cz);
				pts[i * 2 + 1] = PxVec3(cx, (PxReal)-half_h, cz);
			}

			PxConvexMeshDesc desc;
			desc.points.count = pts.size();
			desc.points.stride = sizeof(PxVec3);
			desc.points.data = pts.ptr();
			desc.flags = PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eDISABLE_MESH_VALIDATION | PxConvexFlag::eFAST_INERTIA_COMPUTATION;

			PxCookingParams params(physics->getTolerancesScale());
#ifdef GODOT_PHYSX_GPU
			params.buildGPUData = true;
#endif
			_release_meshes();
			convex_mesh = PxCreateConvexMesh(params, desc, *PxGetStandaloneInsertionCallback());
			ERR_FAIL_NULL_MSG(convex_mesh, "PhysX: cylinder (convex) cooking failed.");
			geom.type = PxGeometryType::eCONVEXMESH;
			geom.convex = PxConvexMeshGeometry(convex_mesh);
			geom_valid = true;
		} break;

		default: {
			// Separation ray, heightmap and custom shapes aren't implemented;
			// keep the shape valid-but-inert so RID lifecycle stays clean.
			WARN_PRINT_ONCE(vformat("PhysX: shape type %d not implemented; treated as no collision.", (int)type));
		} break;
	}
}

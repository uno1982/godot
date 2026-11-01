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
#include "core/math/math_funcs.h"
#include "core/templates/local_vector.h"

#include <PxPhysicsAPI.h>
#include <cooking/PxCooking.h>
#include <float.h>
#include <geometry/PxHeightField.h>
#include <geometry/PxHeightFieldDesc.h>
#include <geometry/PxHeightFieldSample.h>

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
		case PxGeometryType::eHEIGHTFIELD:
			return heightfield;
		default:
			return box;
	}
}

GodotPhysXShape3D::~GodotPhysXShape3D() {
	_release_meshes();
}

void GodotPhysXShape3D::remove_owner(GodotPhysXBody3D *p_body) {
	HashMap<GodotPhysXBody3D *, int>::Iterator it = body_owners.find(p_body);
	if (it && --it->value <= 0) {
		body_owners.remove(it);
	}
}

void GodotPhysXShape3D::remove_owner(GodotPhysXArea3D *p_area) {
	HashMap<GodotPhysXArea3D *, int>::Iterator it = area_owners.find(p_area);
	if (it && --it->value <= 0) {
		area_owners.remove(it);
	}
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
	if (height_field) {
		height_field->release();
		height_field = nullptr;
	}
}

int GodotPhysXShape3D::source_face_index(uint32_t p_cooked_index, bool &r_back) const {
	r_back = false;
	if (!triangle_mesh || source_triangle_count <= 0 || p_cooked_index >= triangle_mesh->getNbTriangles()) {
		return -1;
	}
	const PxU32 *remap = triangle_mesh->getTrianglesRemap();
	uint32_t source = remap ? remap[p_cooked_index] : p_cooked_index;
	if (source >= (uint32_t)source_triangle_count) {
		r_back = true;
		source -= (uint32_t)source_triangle_count;
	}
	return (int)source;
}

bool GodotPhysXShape3D::set_data(const Variant &p_data) {
	// Godot sends a shape's data again when none of it changed: in debug builds CollisionShape3D::set_shape() gives
	// the shape its debug colour, and that resends the data. The same faces, heights or points cook to the same mesh,
	// so keep the one already cooked rather than paying for the cook twice.
	if ((geom_valid || type == PhysicsServer3D::SHAPE_SEPARATION_RAY) && data.get_type() == p_data.get_type() && data == p_data) {
		return false;
	}
	data = p_data;
	geom_valid = false;
	geom = GodotPhysXShapeGeometry();
	backface_collision = false;
	source_triangle_count = 0;

	switch (type) {
		case PhysicsServer3D::SHAPE_SPHERE: {
			const real_t radius = p_data;
			ERR_FAIL_COND_V(radius <= 0.0, true);
			geom.type = PxGeometryType::eSPHERE;
			geom.sphere = PxSphereGeometry((PxReal)radius);
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_BOX: {
			const Vector3 half_extents = p_data;
			ERR_FAIL_COND_V(half_extents.x <= 0.0 || half_extents.y <= 0.0 || half_extents.z <= 0.0, true);
			geom.type = PxGeometryType::eBOX;
			geom.box = PxBoxGeometry(to_px(half_extents));
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_CAPSULE: {
			const Dictionary d = p_data;
			ERR_FAIL_COND_V(!d.has("radius") || !d.has("height"), true);
			const real_t radius = d["radius"];
			const real_t height = d["height"];
			ERR_FAIL_COND_V(radius <= 0.0 || height <= 0.0, true);
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
			ERR_FAIL_COND_V(points.size() < 4, true);
			PxPhysics *physics = GodotPhysXServer3D::get_singleton() ? GodotPhysXServer3D::get_singleton()->get_px_physics() : nullptr;
			ERR_FAIL_NULL_V(physics, true);

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
			ERR_FAIL_NULL_V_MSG(convex_mesh, true, vformat("PhysX: convex hull cooking failed (result %d).", (int)cvx_result));
			geom.type = PxGeometryType::eCONVEXMESH;
			geom.convex = PxConvexMeshGeometry(convex_mesh);
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_CONCAVE_POLYGON: {
			Vector<Vector3> faces;
			if (p_data.get_type() == Variant::DICTIONARY) {
				const Dictionary d = p_data;
				faces = d.get("faces", Vector<Vector3>());
				backface_collision = d.get("backface_collision", false);
			} else {
				faces = p_data;
			}
			ERR_FAIL_COND_V(faces.size() < 3 || faces.size() % 3 != 0, true);
			PxPhysics *physics = GodotPhysXServer3D::get_singleton() ? GodotPhysXServer3D::get_singleton()->get_px_physics() : nullptr;
			ERR_FAIL_NULL_V(physics, true);

			const int tri_count = faces.size() / 3;
			// Back faces on: a flipped copy of every triangle after the
			// originals (see source_face_index()).
			const int cooked_tri_count = backface_collision ? tri_count * 2 : tri_count;
			LocalVector<PxVec3> verts;
			LocalVector<PxU32> indices;
			verts.resize(faces.size());
			indices.resize(cooked_tri_count * 3);
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
			for (int t = tri_count; t < cooked_tri_count; t++) {
				const int s = t - tri_count;
				indices[t * 3 + 0] = s * 3 + 0;
				indices[t * 3 + 1] = s * 3 + 1;
				indices[t * 3 + 2] = s * 3 + 2;
			}
			source_triangle_count = tri_count;

			PxTriangleMeshDesc desc;
			desc.points.count = verts.size();
			desc.points.stride = sizeof(PxVec3);
			desc.points.data = verts.ptr();
			desc.triangles.count = cooked_tri_count;
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
			ERR_FAIL_NULL_V_MSG(triangle_mesh, true, vformat("PhysX: triangle mesh cooking failed (result %d, %d tris).", (int)tri_result, tri_count));
			print_verbose(vformat("PhysX: cooked triangle mesh (%d tris, result %d).", tri_count, (int)tri_result));
			geom.type = PxGeometryType::eTRIANGLEMESH;
			geom.trimesh = PxTriangleMeshGeometry(triangle_mesh);
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_CYLINDER: {
			// PhysX has no native cylinder; approximate with a convex prism.
			const Dictionary d = p_data;
			ERR_FAIL_COND_V(!d.has("radius") || !d.has("height"), true);
			const real_t radius = d["radius"];
			const real_t half_h = (real_t)d["height"] * 0.5;
			ERR_FAIL_COND_V(radius <= 0.0 || half_h <= 0.0, true);
			PxPhysics *physics = GodotPhysXServer3D::get_singleton() ? GodotPhysXServer3D::get_singleton()->get_px_physics() : nullptr;
			ERR_FAIL_NULL_V(physics, true);

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
			ERR_FAIL_NULL_V_MSG(convex_mesh, true, "PhysX: cylinder (convex) cooking failed.");
			geom.type = PxGeometryType::eCONVEXMESH;
			geom.convex = PxConvexMeshGeometry(convex_mesh);
			geom_valid = true;
		} break;

		case PhysicsServer3D::SHAPE_HEIGHTMAP: {
			const Dictionary d = p_data;
			ERR_FAIL_COND_V(!d.has("width") || !d.has("depth") || !d.has("heights"), true);
			const int width = d["width"]; // samples along X
			const int depth = d["depth"]; // samples along Z
			const Vector<real_t> heights = d["heights"];
			ERR_FAIL_COND_V_MSG(width < 2 || depth < 2, true, "PhysX: height map must be at least 2x2.");
			ERR_FAIL_COND_V_MSG(heights.size() != width * depth, true,
					vformat("PhysX: height map has %d samples, expected width*depth = %d.", heights.size(), width * depth));

			const real_t *h_ptr = heights.ptr();
			real_t min_h = d.get("min_height", 0.0);
			real_t max_h = d.get("max_height", 0.0);
			// Stale/unset -> derive from the data. HeightMapShape3D sends NaN for both when its first sample is a hole.
			if (!(min_h < max_h)) {
				min_h = FLT_MAX;
				max_h = -FLT_MAX;
				for (int i = 0; i < heights.size(); i++) {
					if (!Math::is_nan(h_ptr[i])) {
						min_h = MIN(min_h, h_ptr[i]);
						max_h = MAX(max_h, h_ptr[i]);
					}
				}
				if (min_h > max_h) { // all holes
					min_h = 0.0;
					max_h = 0.0;
				}
			}
			// PhysX samples are PxI16. Map [min_h, max_h] onto the full signed
			// range so quantization error is (max-min)/65535 and never clips.
			const float range = MAX((float)(max_h - min_h), 0.001f);
			const float height_scale = range / 65535.0f;
			const float inv_scale = 1.0f / height_scale;

			LocalVector<PxHeightFieldSample> samples;
			samples.resize((uint32_t)(width * depth));
			// PhysX sample(row, col) = samples[row * nbColumns + col], at local
			// position (row * rowScale, height * heightScale, col * columnScale).
			// row = x, col = z; the shape's local pose centers and flips it.
			bool any_hole = false;
			for (int z = 0; z < depth; z++) {
				for (int x = 0; x < width; x++) {
					const float h = (float)h_ptr[z * width + x];
					int q;
					if (Math::is_nan(h)) {
						q = -32768;
						any_hole = true;
					} else {
						q = CLAMP((int)Math::round((h - (float)min_h) * inv_scale) - 32768, -32768, 32767);
					}
					PxHeightFieldSample &s = samples[(uint32_t)(x * depth + z)];
					s.height = (PxI16)q;
					s.materialIndex0 = 0;
					s.materialIndex1 = 0;
				}
			}
			// A NaN height is a hole (Jolt's rule): every triangle with a NaN corner is left out. The sample at
			// (x, z) carries the materials of its cell's two triangles, which (tess flag off) are
			// 0: (x, z) (x, z+1) (x+1, z) and 1: (x+1, z+1) (x+1, z) (x, z+1).
			if (any_hole) {
				auto mark = [&](int p_x, int p_z, bool p_first, bool p_second) {
					if (p_x >= 0 && p_z >= 0 && p_x < width - 1 && p_z < depth - 1) {
						PxHeightFieldSample &s = samples[(uint32_t)(p_x * depth + p_z)];
						if (p_first) {
							s.materialIndex0 = PxHeightFieldMaterial::eHOLE;
						}
						if (p_second) {
							s.materialIndex1 = PxHeightFieldMaterial::eHOLE;
						}
					}
				};
				for (int z = 0; z < depth; z++) {
					for (int x = 0; x < width; x++) {
						if (Math::is_nan(h_ptr[z * width + x])) {
							// The four cells around the sample, and which of their triangles have it as a corner.
							mark(x, z, true, false);
							mark(x - 1, z, true, true);
							mark(x, z - 1, true, true);
							mark(x - 1, z - 1, false, true);
						}
					}
				}
			}

			PxHeightFieldDesc hf_desc;
			hf_desc.nbRows = (PxU32)width;
			hf_desc.nbColumns = (PxU32)depth;
			hf_desc.samples.data = samples.ptr();
			hf_desc.samples.stride = sizeof(PxHeightFieldSample);
			hf_desc.flags = PxHeightFieldFlags();

			_release_meshes();
			height_field = PxCreateHeightField(hf_desc, *PxGetStandaloneInsertionCallback());
			ERR_FAIL_NULL_V_MSG(height_field, true, "PhysX: height field cooking failed.");

			geom.type = PxGeometryType::eHEIGHTFIELD;
			geom.heightfield = PxHeightFieldGeometry(height_field, PxMeshGeometryFlags(), height_scale, 1.0f, 1.0f);
			// Godot centers the map on the origin; PhysX puts sample (0,0) there.
			// Also lift by the baked height offset (sample 0 == world Y min_h + 32768*scale).
			geom.local_pose = PxTransform(PxVec3(
					-(float)(width - 1) * 0.5f,
					(float)min_h + 32768.0f * height_scale,
					-(float)(depth - 1) * 0.5f));
			geom_valid = true;
			print_verbose(vformat("PhysX: built %dx%d height field (range %.2f).", width, depth, (double)range));
		} break;

		case PhysicsServer3D::SHAPE_SEPARATION_RAY: {
			const Dictionary d = p_data;
			ray_length = d.get("length", 1.0);
			ray_slide_on_slope = d.get("slide_on_slope", false);
			// No geometry: see is_separation_ray().
		} break;

		default: {
			// Custom shapes aren't implemented; keep the shape valid-but-inert
			// so RID lifecycle stays clean.
			WARN_PRINT_ONCE(vformat("PhysX: shape type %d not implemented; treated as no collision.", (int)type));
		} break;
	}
	return true;
}

GodotPhysXShape3D::ScaledGeometry GodotPhysXShape3D::scaled_geometry(const Vector3 &p_scale) const {
	ScaledGeometry out;
	out.local_pose = geom.local_pose;

	const PxVec3 a((PxReal)Math::abs(p_scale.x), (PxReal)Math::abs(p_scale.y), (PxReal)Math::abs(p_scale.z));

	switch (geom.type) {
		case PxGeometryType::eBOX: {
			const PxVec3 he = geom.box.halfExtents;
			out.geom = PxBoxGeometry(he.x * a.x, he.y * a.y, he.z * a.z);
		} break;

		case PxGeometryType::eSPHERE: {
			if (!(Math::is_equal_approx(a.x, a.y) && Math::is_equal_approx(a.y, a.z))) {
				WARN_PRINT_ONCE("PhysX: a sphere collision shape can't take non-uniform scale; using the mean axis.");
			}
			out.geom = PxSphereGeometry(geom.sphere.radius * (a.x + a.y + a.z) / 3.0f);
		} break;

		case PxGeometryType::eCAPSULE: {
			// The module rotates the capsule's local pose so PhysX +X maps to
			// Godot +Y: half-height follows Godot Y, radius follows Godot X/Z.
			if (!Math::is_equal_approx(a.x, a.z)) {
				WARN_PRINT_ONCE("PhysX: a capsule collision shape can't take non-uniform X/Z scale; using the mean.");
			}
			out.geom = PxCapsuleGeometry(geom.capsule.radius * (a.x + a.z) * 0.5f, geom.capsule.halfHeight * a.y);
		} break;

		case PxGeometryType::eCONVEXMESH: {
			// PxMeshScale takes signed components -- a mirrored convex is valid.
			PxConvexMeshGeometry g = geom.convex;
			g.scale = PxMeshScale(to_px(p_scale));
			out.geom = g;
		} break;

		case PxGeometryType::eTRIANGLEMESH: {
			PxTriangleMeshGeometry g = geom.trimesh;
			g.scale = PxMeshScale(to_px(p_scale));
			out.geom = g;
		} break;

		case PxGeometryType::eHEIGHTFIELD: {
			PxHeightFieldGeometry g = geom.heightfield;
			g.rowScale *= a.x;
			g.columnScale *= a.z;
			g.heightScale *= a.y;
			out.geom = g;
			// The local pose centers the grid and lifts by the baked height
			// offset -- both are in unscaled sample units, so scale them too.
			out.local_pose.p.x *= a.x;
			out.local_pose.p.y *= a.y;
			out.local_pose.p.z *= a.z;
		} break;

		default: {
			out.geom = geom.geometry();
		} break;
	}
	return out;
}

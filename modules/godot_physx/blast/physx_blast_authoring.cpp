/**************************************************************************/
/*  physx_blast_authoring.cpp                                             */
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
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "physx_blast_authoring.h"

#include "../godot_physx_server_3d.h"

#include "core/object/class_db.h"
#include "scene/resources/mesh.h"

#include <NvBlast.h>
#include <NvBlastExtAuthoring.h>
#include <NvBlastExtAuthoringBondGenerator.h>
#include <NvBlastExtAuthoringConvexMeshBuilder.h>
#include <NvBlastExtAuthoringFractureTool.h>
#include <NvBlastExtAuthoringMesh.h>
#include <NvBlastExtAuthoringTypes.h>
#include <NvBlastTypes.h>
#include <PxPhysicsAPI.h>

#include <cstdlib>
#include <vector>

// Deliberately NOT `using namespace Nv::Blast;` -- Nv::Blast::Mesh collides
// with Godot's own ::Mesh (scene/resources/mesh.h), which this file also
// uses for the p_mesh parameter. Every Blast SDK type below is qualified
// explicitly instead.
using namespace physx;

namespace {

void blast_authoring_log(int32_t p_type, const char *p_msg, const char *p_file, int32_t p_line) {
	if (p_type <= 1) {
		ERR_PRINT(vformat("Blast authoring: %s (%s:%d)", p_msg, p_file, p_line));
	}
}

// Ported verbatim from the standalone blast_test_gen.cpp prototype that
// proved this whole bridge out -- ConvexMeshBuilder ships as a pure
// interface in this SDK (no built-in implementation, see that file's
// comments), so every caller has to provide one. Same "cook a real hull so
// ProcessFracture doesn't choke on a degenerate one, but the result is never
// actually consumed downstream" reasoning as the prototype: PhysXDestructible3D
// cooks its own PhysX convex hulls from each chunk's render-mesh points, not
// from Blast's authored CollisionHull data.
class GodotConvexMeshBuilder : public Nv::Blast::ConvexMeshBuilder {
public:
	explicit GodotConvexMeshBuilder(PxPhysics *p_physics) :
			physics(p_physics) {}

	void release() override { delete this; }

	Nv::Blast::CollisionHull *buildCollisionGeometry(uint32_t p_count, const NvcVec3 *p_verts) override {
		std::vector<PxVec3> pts(p_count);
		for (uint32_t i = 0; i < p_count; i++) {
			pts[i] = PxVec3(p_verts[i].x, p_verts[i].y, p_verts[i].z);
		}
		PxConvexMeshDesc desc;
		desc.points.count = p_count;
		desc.points.stride = sizeof(PxVec3);
		desc.points.data = pts.data();
		desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
		PxCookingParams cook_params(physics->getTolerancesScale());
		PxConvexMesh *mesh = PxCreateConvexMesh(cook_params, desc, *PxGetStandaloneInsertionCallback());

		Nv::Blast::CollisionHull *hull = new Nv::Blast::CollisionHull();
		memset(hull, 0, sizeof(Nv::Blast::CollisionHull));
		if (!mesh) {
			return hull; // empty hull -- fine, unused downstream
		}

		const PxU32 nb_verts = mesh->getNbVertices();
		const PxVec3 *src_verts = mesh->getVertices();
		hull->pointsCount = nb_verts;
		hull->points = new NvcVec3[nb_verts];
		for (PxU32 i = 0; i < nb_verts; i++) {
			hull->points[i] = { src_verts[i].x, src_verts[i].y, src_verts[i].z };
		}

		const PxU32 nb_polys = mesh->getNbPolygons();
		hull->polygonDataCount = nb_polys;
		hull->polygonData = new Nv::Blast::HullPolygon[nb_polys];
		std::vector<PxU8> all_indices;
		for (PxU32 i = 0; i < nb_polys; i++) {
			PxHullPolygon pd;
			mesh->getPolygonData(i, pd);
			hull->polygonData[i].plane[0] = pd.mPlane[0];
			hull->polygonData[i].plane[1] = pd.mPlane[1];
			hull->polygonData[i].plane[2] = pd.mPlane[2];
			hull->polygonData[i].plane[3] = pd.mPlane[3];
			hull->polygonData[i].vertexCount = pd.mNbVerts;
			hull->polygonData[i].indexBase = (uint16_t)all_indices.size();
			const PxU8 *ib = mesh->getIndexBuffer();
			for (PxU16 j = 0; j < pd.mNbVerts; j++) {
				all_indices.push_back(ib[pd.mIndexBase + j]);
			}
		}
		hull->indicesCount = (uint32_t)all_indices.size();
		hull->indices = new uint32_t[all_indices.size()];
		for (size_t i = 0; i < all_indices.size(); i++) {
			hull->indices[i] = all_indices[i];
		}

		mesh->release();
		return hull;
	}

	void releaseCollisionHull(Nv::Blast::CollisionHull *hull) const override {
		if (!hull) {
			return;
		}
		delete[] hull->points;
		delete[] hull->indices;
		delete[] hull->polygonData;
		delete hull;
	}

private:
	PxPhysics *physics;
};

class GodotRandomGenerator : public Nv::Blast::RandomGeneratorBase {
public:
	void seed(int32_t p_seed) override { std::srand((unsigned)p_seed); }
	float getRandomValue() override { return (float)std::rand() / (float)RAND_MAX; }
};

} //namespace

void PhysXBlastAuthoring::_bind_methods() {
	ClassDB::bind_method(D_METHOD("fracture_mesh", "mesh", "site_count", "seed", "pattern"), &PhysXBlastAuthoring::fracture_mesh, DEFVAL(PATTERN_VORONOI));

	BIND_ENUM_CONSTANT(PATTERN_VORONOI);
	BIND_ENUM_CONSTANT(PATTERN_SLICING);
}

Ref<PhysXBlastAsset> PhysXBlastAuthoring::fracture_mesh(const Ref<Mesh> &p_mesh, int p_site_count, int p_seed, FracturePattern p_pattern) {
	ERR_FAIL_COND_V_MSG(p_mesh.is_null() || p_mesh->get_surface_count() < 1, Ref<PhysXBlastAsset>(),
			"PhysXBlastAuthoring: mesh is null or has no surfaces.");
	PxPhysics *physics = GodotPhysXServer3D::get_singleton() ? GodotPhysXServer3D::get_singleton()->get_px_physics() : nullptr;
	ERR_FAIL_NULL_V_MSG(physics, Ref<PhysXBlastAsset>(), "PhysXBlastAuthoring: PhysX not initialized.");

	const Array arrays = p_mesh->surface_get_arrays(0);
	const PackedVector3Array verts = arrays[Mesh::ARRAY_VERTEX];
	ERR_FAIL_COND_V_MSG(verts.size() < 4, Ref<PhysXBlastAsset>(), "PhysXBlastAuthoring: mesh has too few vertices.");
	PackedVector3Array src_normals = arrays[Mesh::ARRAY_NORMAL];
	PackedInt32Array src_indices = arrays[Mesh::ARRAY_INDEX];

	LocalVector<NvcVec3> positions;
	LocalVector<NvcVec3> normals;
	LocalVector<NvcVec2> uvs;
	positions.resize(verts.size());
	normals.resize(verts.size());
	uvs.resize(verts.size());
	for (int i = 0; i < verts.size(); i++) {
		positions[i] = { verts[i].x, verts[i].y, verts[i].z };
		// A flat placeholder if the mesh has none -- Blast's Voronoi split
		// doesn't need correct normals, and nothing downstream ever reads
		// them back (see GodotConvexMeshBuilder's own comment).
		const Vector3 n = (i < src_normals.size()) ? src_normals[i] : Vector3(0, 1, 0);
		normals[i] = { n.x, n.y, n.z };
		uvs[i] = { 0.0f, 0.0f };
	}

	LocalVector<uint32_t> indices;
	if (src_indices.size() >= 3) {
		indices.resize(src_indices.size());
		for (int i = 0; i < src_indices.size(); i++) {
			indices[i] = (uint32_t)src_indices[i];
		}
	} else {
		// Non-indexed mesh -- vertices are already one triangle list.
		indices.resize(verts.size());
		for (uint32_t i = 0; i < (uint32_t)verts.size(); i++) {
			indices[i] = i;
		}
	}

	Nv::Blast::Mesh *blast_mesh = NvBlastExtAuthoringCreateMesh(positions.ptr(), normals.ptr(), uvs.ptr(),
			(uint32_t)positions.size(), indices.ptr(), (uint32_t)indices.size());
	ERR_FAIL_NULL_V_MSG(blast_mesh, Ref<PhysXBlastAsset>(), "PhysXBlastAuthoring: NvBlastExtAuthoringCreateMesh failed.");

	Nv::Blast::FractureTool *fTool = NvBlastExtAuthoringCreateFractureTool();
	const Nv::Blast::Mesh *src_meshes[1] = { blast_mesh };
	fTool->setSourceMeshes(src_meshes, 1);

	GodotRandomGenerator rng;
	rng.seed(p_seed);

	Ref<PhysXBlastAsset> result;
	Nv::Blast::VoronoiSitesGenerator *sites_gen = nullptr;
	int32_t frac_result;

	if (p_pattern == PATTERN_SLICING) {
		// slicing() works in slice-counts-per-axis, not a single site count --
		// approximate p_site_count with a roughly-cube-root split so the
		// property still reads as "about this many pieces" regardless of
		// which pattern is picked. A small amount of offset/angle variation
		// by default so the cuts don't look like a perfectly uniform grid.
		int per_axis = 1;
		const int target = MAX(p_site_count, 1);
		while ((per_axis + 1) * (per_axis + 1) * (per_axis + 1) <= target) {
			per_axis++;
		}
		Nv::Blast::SlicingConfiguration conf;
		conf.x_slices = per_axis;
		conf.y_slices = per_axis;
		conf.z_slices = per_axis;
		conf.offset_variations = 0.2f;
		conf.angle_variations = 0.2f;
		frac_result = fTool->slicing(0, conf, false, &rng);
		if (frac_result != 0) {
			ERR_PRINT(vformat("PhysXBlastAuthoring: slicing failed, code=%d.", frac_result));
		}
	} else {
		sites_gen = NvBlastExtAuthoringCreateVoronoiSitesGenerator(blast_mesh, &rng);
		sites_gen->uniformlyGenerateSitesInMesh((uint32_t)MAX(p_site_count, 1));
		const NvcVec3 *sites = nullptr;
		const uint32_t site_count = sites_gen->getVoronoiSites(sites);
		frac_result = fTool->voronoiFracturing(0, site_count, sites, false);
		if (frac_result != 0) {
			ERR_PRINT(vformat("PhysXBlastAuthoring: voronoiFracturing failed, code=%d.", frac_result));
		}
	}

	if (frac_result == 0) {
		fTool->finalizeFracturing();

		GodotConvexMeshBuilder collision_builder(physics);
		Nv::Blast::BlastBondGenerator *bond_gen = NvBlastExtAuthoringCreateBondGenerator(&collision_builder);
		Nv::Blast::ConvexDecompositionParams cparams;
		cparams.maximumNumberOfHulls = 1; // chunks are already convex (both Voronoi cells and slicing planes of a convex source stay convex)

		Nv::Blast::AuthoringResult *ares = NvBlastExtAuthoringProcessFracture(*fTool, *bond_gen, collision_builder, cparams);
		if (!ares) {
			ERR_PRINT("PhysXBlastAuthoring: NvBlastExtAuthoringProcessFracture failed.");
		} else {
			const uint32_t asset_size = NvBlastAssetGetSize(ares->asset, blast_authoring_log);
			PackedByteArray asset_bytes;
			asset_bytes.resize(asset_size);
			memcpy(asset_bytes.ptrw(), ares->asset, asset_size);

			Array chunk_points;
			chunk_points.resize(ares->chunkCount);
			for (uint32_t c = 0; c < ares->chunkCount; c++) {
				const uint32_t begin = ares->geometryOffset[c];
				const uint32_t end = ares->geometryOffset[c + 1];
				PackedVector3Array points;
				points.resize((end - begin) * 3);
				for (uint32_t t = begin; t < end; t++) {
					const Nv::Blast::Triangle &tri = ares->geometry[t];
					points.write[(t - begin) * 3 + 0] = Vector3(tri.a.p.x, tri.a.p.y, tri.a.p.z);
					points.write[(t - begin) * 3 + 1] = Vector3(tri.b.p.x, tri.b.p.y, tri.b.p.z);
					points.write[(t - begin) * 3 + 2] = Vector3(tri.c.p.x, tri.c.p.y, tri.c.p.z);
				}
				chunk_points[c] = points;
			}

			result.instantiate();
			result->set_asset_bytes(asset_bytes);
			result->set_chunk_points(chunk_points);

			NvBlastExtAuthoringReleaseAuthoringResult(collision_builder, ares);
		}
		bond_gen->release();
	}

	if (sites_gen) {
		sites_gen->release();
	}
	fTool->release();
	blast_mesh->release();
	return result;
}

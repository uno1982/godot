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
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "physx_blast_authoring.h"

#include "../godot_physx_server_3d.h"
#include "../shapes/godot_physx_shape_3d.h"

#include "core/io/image.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "core/templates/hash_map.h"
#include "core/variant/typed_array.h"
#include "scene/resources/material.h"
#include "scene/resources/mesh.h"
#include "scene/resources/texture.h"

#include <NvBlast.h>
#include <NvBlastExtAuthoring.h>
#include <NvBlastExtAuthoringBondGenerator.h>
#include <NvBlastExtAuthoringConvexMeshBuilder.h>
#include <NvBlastExtAuthoringCutout.h>
#include <NvBlastExtAuthoringFractureTool.h>
#include <NvBlastExtAuthoringMesh.h>
#include <NvBlastExtAuthoringMeshCleaner.h>
#include <NvBlastExtAuthoringTypes.h>
#include <NvBlastTypes.h>
#include <PxPhysicsAPI.h>

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
class GodotConvexMeshBuilder final : public Nv::Blast::ConvexMeshBuilder {
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

// Each fracture's own sequence, so fractures on other threads can't take numbers from it. The same sequence as MSVC's
// srand()/rand(), which this used before: a Windows build fractures exactly as it did, and every platform now does too
// (glibc's rand() is another sequence).
class GodotRandomGenerator : public Nv::Blast::RandomGeneratorBase {
	uint32_t state = 1;

public:
	void seed(int32_t p_seed) override { state = (uint32_t)p_seed; }
	float getRandomValue() override {
		state = state * 214013u + 2531011u;
		return (float)((state >> 16) & 0x7fff) / 32767.0f;
	}
};

// Pre-flight check for PATTERN_CUTOUT patterns, added after two different
// real crashes deep inside NvBlastExtAuthoringFractureToolImpl::cutout()
// (SweepingAccelerator construction) -- both traced to the same cause: a
// crack/wall shape that doesn't reach the pattern's edge leaves the fill
// region wrapping around it with a hole in the middle (unlike a real broken
// pane, where every shard is a closed polygon and every crack reaches a
// boundary). Confirmed by fixing exactly that -- extending the crack lines
// to the image edge -- and nothing else, which made an otherwise-identical
// pattern fracture cleanly.
//
// Rule enforced here, directly from that finding: every connected wall
// (crack-line) shape must touch the image boundary somewhere. A wall
// component with zero boundary-touching pixels is a "floating island" --
// the exact topology that crashed. Uses the same weighted-RGB threshold
// NvBlastExtAuthoringCutoutImpl.cpp's own createCutoutSet() applies (see
// PhysXBlastAuthoring::fracture_mesh()'s own comment on why it's 3 bytes/
// pixel, not the 1-byte/pixel the public header claims) so this reads the
// pattern exactly the way the SDK itself will.
bool cutout_pattern_has_floating_wall_island(const PackedByteArray &p_gray, uint32_t p_w, uint32_t p_h) {
	const int64_t n = (int64_t)p_w * (int64_t)p_h;
	if (n == 0) {
		return false;
	}

	LocalVector<bool> is_wall;
	is_wall.resize((uint32_t)n);
	{
		const uint8_t *g = p_gray.ptr();
		for (int64_t i = 0; i < n; i++) {
			const uint32_t pix = 16777216u * (uint32_t)g[i]; // (5033165+9898557+1845494) * gray, R=G=B
			is_wall[(uint32_t)i] = (pix >> 28) != 0;
		}
	}

	LocalVector<int32_t> visited; // -1 = unvisited wall pixel, else already assigned to a checked component
	visited.resize((uint32_t)n);
	for (uint32_t i = 0; i < (uint32_t)n; i++) {
		visited[i] = -1;
	}

	LocalVector<uint32_t> stack;
	for (uint32_t y = 0; y < p_h; y++) {
		for (uint32_t x = 0; x < p_w; x++) {
			const uint32_t start = y * p_w + x;
			if (!is_wall[start] || visited[start] != -1) {
				continue;
			}

			bool touches_border = false;
			stack.clear();
			stack.push_back(start);
			visited[start] = 1;
			while (!stack.is_empty()) {
				const uint32_t cur = stack[stack.size() - 1];
				stack.remove_at(stack.size() - 1);
				const uint32_t cx = cur % p_w;
				const uint32_t cy = cur / p_w;
				if (cx == 0 || cx == p_w - 1 || cy == 0 || cy == p_h - 1) {
					touches_border = true;
				}
				if (cx > 0) {
					const uint32_t nb = cur - 1;
					if (is_wall[nb] && visited[nb] == -1) {
						visited[nb] = 1;
						stack.push_back(nb);
					}
				}
				if (cx + 1 < p_w) {
					const uint32_t nb = cur + 1;
					if (is_wall[nb] && visited[nb] == -1) {
						visited[nb] = 1;
						stack.push_back(nb);
					}
				}
				if (cy > 0) {
					const uint32_t nb = cur - p_w;
					if (is_wall[nb] && visited[nb] == -1) {
						visited[nb] = 1;
						stack.push_back(nb);
					}
				}
				if (cy + 1 < p_h) {
					const uint32_t nb = cur + p_w;
					if (is_wall[nb] && visited[nb] == -1) {
						visited[nb] = 1;
						stack.push_back(nb);
					}
				}
			}

			if (!touches_border) {
				return true; // a floating island -- reject the whole pattern
			}
		}
	}
	return false;
}

// Whether every edge (by vertex index, the way Blast sees it) is shared by exactly two triangles: a closed mesh
// that doesn't need the mesh cleaner.
bool is_closed(const LocalVector<uint32_t> &p_indices) {
	HashMap<uint64_t, int> edges;
	for (uint32_t t = 0; t + 2 < p_indices.size(); t += 3) {
		for (int e = 0; e < 3; e++) {
			const uint64_t a = p_indices[t + e];
			const uint64_t b = p_indices[t + (e + 1) % 3];
			if (a == b) {
				return false; // a degenerate triangle
			}
			edges[MIN(a, b) << 32 | MAX(a, b)]++;
		}
	}
	for (const KeyValue<uint64_t, int> &kv : edges) {
		if (kv.value != 2) {
			return false;
		}
	}
	return !edges.is_empty();
}

// The UVs of a cut face, in mesh units (1 UV = 1 unit, so a material's uv1_scale sets the tiling). Blast's own come from
// the corners of a huge cutting box: their density has nothing to do with the mesh's size and each cut gets its own. Each
// cut plane is projected straight on instead, so nothing stretches and all the pieces share one density. The plane's
// normal is taken with one sign, so the two faces of a crack read the same part of a texture.
class CutFaceUVs {
	struct Plane {
		Vector3 normal;
		Vector3 u;
		Vector3 v;
	};
	LocalVector<Plane> planes;

public:
	Vector2 uv(const Vector3 &p_point, const Vector3 &p_face_normal) {
		// About 5 degrees: a face's triangles differ by float error, slivers the most, and sharing a plane with one
		// this close is invisible.
		const real_t same_plane = 0.996;
		const Plane *plane = nullptr;
		for (const Plane &candidate : planes) {
			if (Math::abs(candidate.normal.dot(p_face_normal)) > same_plane) {
				plane = &candidate;
				break;
			}
		}
		if (!plane) {
			Plane added;
			const int axis = p_face_normal.abs().max_axis_index();
			added.normal = p_face_normal[axis] < 0.0 ? -p_face_normal : p_face_normal;
			// Up stays up on a wall.
			const Vector3 up = Math::abs(added.normal.y) < 0.9 ? Vector3(0, 1, 0) : Vector3(0, 0, -1);
			added.u = up.cross(added.normal).normalized();
			added.v = added.normal.cross(added.u);
			planes.push_back(added);
			plane = &planes[planes.size() - 1];
		}
		// Texture V runs down.
		return Vector2(plane->u.dot(p_point), -plane->v.dot(p_point));
	}
};

} //namespace

namespace {

// Printed as before, and kept for get_last_error() / get_last_warnings() (the fracture dialog shows them).
void fail(PhysXBlastFractureOutput &r_output, const String &p_message) {
	ERR_PRINT(p_message);
	r_output.error = p_message.trim_prefix("PhysXBlastAuthoring: ");
}

void warn(PhysXBlastFractureOutput &r_output, const String &p_message) {
	WARN_PRINT(p_message);
	r_output.warnings.push_back(p_message.trim_prefix("PhysXBlastAuthoring: "));
}

// The Blast objects one fracture makes, released however it ends, in this order.
struct BlastObjects {
	Nv::Blast::Mesh *raw_mesh = nullptr;
	Nv::Blast::MeshCleaner *mesh_cleaner = nullptr;
	Nv::Blast::Mesh *cleaned_mesh = nullptr;
	Nv::Blast::FractureTool *fracture_tool = nullptr;
	Nv::Blast::VoronoiSitesGenerator *sites_gen = nullptr;
	Nv::Blast::BlastBondGenerator *bond_gen = nullptr;

	~BlastObjects() {
		if (bond_gen) {
			bond_gen->release();
		}
		if (sites_gen) {
			sites_gen->release();
		}
		if (fracture_tool) {
			fracture_tool->release();
		}
		if (cleaned_mesh) {
			cleaned_mesh->release();
		}
		if (raw_mesh) {
			raw_mesh->release();
		}
		if (mesh_cleaner) {
			mesh_cleaner->release();
		}
	}
};

} //namespace

void PhysXBlastAuthoring::_bind_methods() {
	ClassDB::bind_method(D_METHOD("fracture_mesh", "mesh", "site_count", "seed", "pattern", "cutout_pattern"), &PhysXBlastAuthoring::fracture_mesh, DEFVAL(PATTERN_VORONOI), DEFVAL(Ref<Texture2D>()));
	ClassDB::bind_method(D_METHOD("fracture_mesh_async", "mesh", "site_count", "seed", "pattern", "cutout_pattern"), &PhysXBlastAuthoring::fracture_mesh_async, DEFVAL(PATTERN_VORONOI), DEFVAL(Ref<Texture2D>()));

	ClassDB::bind_method(D_METHOD("get_last_error"), &PhysXBlastAuthoring::get_last_error);
	ClassDB::bind_method(D_METHOD("get_last_warnings"), &PhysXBlastAuthoring::get_last_warnings);

	BIND_ENUM_CONSTANT(PATTERN_VORONOI);
	BIND_ENUM_CONSTANT(PATTERN_SLICING);
	BIND_ENUM_CONSTANT(PATTERN_CUTOUT);
}

String PhysXBlastAuthoring::stage_name(int p_stage) {
	switch (p_stage) {
		case STAGE_CHECKING:
			return "Checking the mesh";
		case STAGE_CLEANING:
			return "Cleaning up the mesh";
		case STAGE_FRACTURING:
			return "Fracturing";
		case STAGE_FINALIZING:
			return "Finalizing the pieces";
		case STAGE_BONDS:
			return "Building bonds and collision";
		case STAGE_HULLS:
			return "Cooking collision hulls";
	}
	return String();
}

Ref<PhysXBlastAsset> PhysXBlastAuthoring::fracture_mesh(const Ref<Mesh> &p_mesh, int p_site_count, int p_seed, FracturePattern p_pattern, const Ref<Texture2D> &p_cutout_pattern) {
	PhysXBlastFractureInput input;
	PhysXBlastFractureOutput output;
	if (read_input(p_mesh, p_site_count, p_seed, p_pattern, p_cutout_pattern, input, output)) {
		run(input, output, nullptr);
	}
	last_error = output.error;
	last_warnings = output.warnings;
	return make_asset(input, output);
}

Ref<PhysXBlastFractureTask> PhysXBlastAuthoring::fracture_mesh_async(const Ref<Mesh> &p_mesh, int p_site_count, int p_seed, FracturePattern p_pattern, const Ref<Texture2D> &p_cutout_pattern) {
	Ref<PhysXBlastFractureTask> task;
	task.instantiate();
	task->running.set();
	task->keep_alive = task;
	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (!read_input(p_mesh, p_site_count, p_seed, p_pattern, p_cutout_pattern, task->input, task->output)) {
		// Finished on the next frame, like a fracture that fails later on, so the caller can connect to it first.
		callable_mp(task.ptr(), &PhysXBlastFractureTask::_finish).call_deferred();
	} else if (!server || !server->begin_background_job()) {
		fail(task->output, "PhysXBlastAuthoring: PhysX is shutting down.");
		callable_mp(task.ptr(), &PhysXBlastFractureTask::_finish).call_deferred();
	} else {
		task->task_id = WorkerThreadPool::get_singleton()->add_task(callable_mp(task.ptr(), &PhysXBlastFractureTask::_work), false, "Blast fracture");
	}
	return task;
}

bool PhysXBlastAuthoring::read_input(const Ref<Mesh> &p_mesh, int p_site_count, int p_seed, FracturePattern p_pattern, const Ref<Texture2D> &p_cutout_pattern, PhysXBlastFractureInput &r_input, PhysXBlastFractureOutput &r_output) {
	if (p_mesh.is_null() || p_mesh->get_surface_count() < 1) {
		fail(r_output, "PhysXBlastAuthoring: mesh is null or has no surfaces.");
		return false;
	}
	PxPhysics *physics = GodotPhysXServer3D::get_singleton() ? GodotPhysXServer3D::get_singleton()->get_px_physics() : nullptr;
	if (!physics) {
		fail(r_output, "PhysXBlastAuthoring: PhysX not initialized.");
		return false;
	}
	r_input.site_count = p_site_count;
	r_input.seed = p_seed;
	r_input.pattern = p_pattern;

	// Every triangle surface goes into one mesh, each triangle tagged with its surface (Blast's material id, carried
	// through the cuts), so the pieces keep each surface's faces apart and the asset keeps each surface's material.
	PackedVector3Array &verts = r_input.verts;
	PackedVector3Array &src_normals = r_input.normals;
	PackedVector2Array &src_uvs = r_input.uvs;
	LocalVector<uint32_t> &indices = r_input.indices;
	const int surface_count = MIN(p_mesh->get_surface_count(), (int)Nv::Blast::kMaterialInteriorId);
	r_input.surface_count = surface_count;
	bool any_normals = false;
	for (int s = 0; s < surface_count; s++) {
		if (p_mesh->surface_get_primitive_type(s) != Mesh::PRIMITIVE_TRIANGLES) {
			warn(r_output, vformat("PhysXBlastAuthoring: surface %d isn't a triangle list; left out.", s));
			r_input.surface_materials.push_back(Ref<Material>());
			continue;
		}
		const Array arrays = p_mesh->surface_get_arrays(s);
		const PackedVector3Array surface_verts = arrays[Mesh::ARRAY_VERTEX];
		const PackedVector3Array surface_normals = arrays[Mesh::ARRAY_NORMAL];
		const PackedVector2Array surface_uvs = arrays[Mesh::ARRAY_TEX_UV];
		const PackedInt32Array surface_indices = arrays[Mesh::ARRAY_INDEX];
		r_input.surface_materials.push_back(p_mesh->surface_get_material(s));
		const uint32_t base = (uint32_t)verts.size();
		for (int i = 0; i < surface_verts.size(); i++) {
			verts.push_back(surface_verts[i]);
			// A placeholder if the surface has none: Blast's cuts don't need correct normals, and the pieces get flat
			// ones then.
			src_normals.push_back(i < surface_normals.size() ? surface_normals[i] : Vector3(0, 1, 0));
			src_uvs.push_back(i < surface_uvs.size() ? surface_uvs[i] : Vector2());
		}
		any_normals = any_normals || !surface_normals.is_empty();
		const uint32_t first_index = indices.size();
		if (surface_indices.size() >= 3) {
			for (int i = 0; i < surface_indices.size(); i++) {
				if (surface_indices[i] < 0 || surface_indices[i] >= surface_verts.size()) {
					fail(r_output, vformat("PhysXBlastAuthoring: surface %d's index %d points past its %d vertices.", s, surface_indices[i], surface_verts.size()));
					return false;
				}
				indices.push_back(base + (uint32_t)surface_indices[i]);
			}
		} else {
			// Non-indexed -- vertices are already one triangle list.
			for (uint32_t i = 0; i < (uint32_t)surface_verts.size(); i++) {
				indices.push_back(base + i);
			}
		}
		if ((indices.size() - first_index) % 3 != 0) {
			fail(r_output, vformat("PhysXBlastAuthoring: surface %d's %d indices aren't a whole number of triangles.", s, (int)(indices.size() - first_index)));
			return false;
		}
		for (uint32_t t = first_index; t < indices.size(); t += 3) {
			r_input.triangle_surfaces.push_back(s);
		}
	}
	if (p_mesh->get_surface_count() > surface_count) {
		warn(r_output, vformat("PhysXBlastAuthoring: only the first %d surfaces are fractured.", surface_count));
	}
	if (verts.size() < 4) {
		fail(r_output, "PhysXBlastAuthoring: mesh has too few vertices.");
		return false;
	}
	if (!any_normals) {
		src_normals.clear();
	}

	// PATTERN_CUTOUT's pattern, read here (a texture is the main thread's); a problem with it is reported where it was
	// before, when fracturing starts.
	if (p_pattern == PATTERN_CUTOUT) {
		// Unlike Voronoi/Slicing, NvBlast doesn't generate this pattern itself
		// -- same as Unreal's own Blast integration, the caller supplies a
		// grayscale bitmap (see this class's header for why) and
		// NvBlastExtAuthoringBuildCutoutSet segments it into per-region loops.
		if (p_cutout_pattern.is_null()) {
			r_input.cutout_error = "PhysXBlastAuthoring: PATTERN_CUTOUT requires a cutout_pattern texture.";
		} else {
			Ref<Image> pattern_img = p_cutout_pattern->get_image();
			if (pattern_img.is_null() || pattern_img->is_empty()) {
				r_input.cutout_error = "PhysXBlastAuthoring: cutout_pattern texture has no image data.";
			} else {
				if (pattern_img->is_compressed()) {
					pattern_img->decompress();
				}
				pattern_img->convert(Image::FORMAT_L8);
				r_input.cutout_gray = pattern_img->get_data();
				r_input.cutout_width = (uint32_t)pattern_img->get_width();
				r_input.cutout_height = (uint32_t)pattern_img->get_height();
			}
		}
	}
	return true;
}

void PhysXBlastAuthoring::run(const PhysXBlastFractureInput &p_input, PhysXBlastFractureOutput &r_output, PhysXBlastFractureTask *p_task) {
	// Between stages (and hulls), the task can stop it: a stage already running can't be interrupted.
	const auto stage = [&](int p_stage, const String &p_detail) -> bool {
		if (p_task && !p_task->report_stage(p_stage, p_detail)) {
			r_output.cancelled = true;
			r_output.error = "Cancelled.";
			return false;
		}
		return true;
	};
	PxPhysics *physics = GodotPhysXServer3D::get_singleton() ? GodotPhysXServer3D::get_singleton()->get_px_physics() : nullptr;
	if (!physics) {
		fail(r_output, "PhysXBlastAuthoring: PhysX not initialized.");
		return;
	}
	if (!stage(STAGE_CHECKING, String())) {
		return;
	}

	const PackedVector3Array &verts = p_input.verts;
	const PackedVector3Array &src_normals = p_input.normals;
	const PackedVector2Array &src_uvs = p_input.uvs;
	LocalVector<uint32_t> indices(p_input.indices); // flipped below if the mesh is inside out
	const int surface_count = p_input.surface_count;

	LocalVector<NvcVec3> positions;
	LocalVector<NvcVec3> normals;
	LocalVector<NvcVec2> uvs;
	positions.resize(verts.size());
	normals.resize(verts.size());
	uvs.resize(verts.size());
	for (int i = 0; i < verts.size(); i++) {
		positions[i] = { verts[i].x, verts[i].y, verts[i].z };
		const Vector3 n = i < src_normals.size() ? src_normals[i] : Vector3(0, 1, 0);
		normals[i] = { n.x, n.y, n.z };
		uvs[i] = { src_uvs[i].x, src_uvs[i].y };
	}

	// Blast trusts its input: an index past the vertices was read out of bounds, a partial triangle crashed, and a
	// NaN vertex or a mesh collapsed to a point spun the mesh cleaner forever. Checked here instead.
	if (indices.size() < 3 || indices.size() % 3 != 0) {
		fail(r_output, vformat("PhysXBlastAuthoring: %d indices isn't a whole number of triangles.", (int)indices.size()));
		return;
	}
	for (uint32_t i = 0; i < indices.size(); i++) {
		if (indices[i] >= (uint32_t)verts.size()) {
			fail(r_output, vformat("PhysXBlastAuthoring: index %d points past the mesh's %d vertices.", (int)indices[i], verts.size()));
			return;
		}
	}
	AABB bounds(verts[0], Vector3());
	for (int i = 0; i < verts.size(); i++) {
		if (!verts[i].is_finite()) {
			fail(r_output, vformat("PhysXBlastAuthoring: vertex %d isn't a finite position.", i));
			return;
		}
		bounds.expand_to(verts[i]);
	}
	const real_t thinnest = bounds.size[bounds.get_shortest_axis_index()];
	if (thinnest <= MAX(bounds.get_longest_axis_size(), (real_t)1.0) * (real_t)1e-5) {
		fail(r_output, vformat("PhysXBlastAuthoring: the mesh has no volume to fracture (its bounds are %s).", bounds.size));
		return;
	}

	// Inside out (its triangles face inward: a negative volume, Godot's front faces being clockwise): Blast keeps the
	// winding it's given, so every piece's outer faces would face in, culled -- the pieces look hollow. Flipped.
	real_t signed_volume = 0.0;
	for (uint32_t t = 0; t < indices.size(); t += 3) {
		const Vector3 &a = verts[indices[t]];
		const Vector3 &b = verts[indices[t + 1]];
		const Vector3 &c = verts[indices[t + 2]];
		signed_volume += a.dot(c.cross(b));
	}
	const bool inside_out = signed_volume < 0.0;
	if (inside_out) {
		for (uint32_t t = 0; t < indices.size(); t += 3) {
			SWAP(indices[t + 1], indices[t + 2]);
		}
		warn(r_output, "PhysXBlastAuthoring: the mesh was inside out (its triangles faced inward); fractured with its triangles flipped.");
	}

	// Declared before the Blast objects: the bond generator holds it until it's released.
	GodotConvexMeshBuilder collision_builder(physics);
	BlastObjects blast;
	blast.raw_mesh = NvBlastExtAuthoringCreateMesh(positions.ptr(), normals.ptr(), uvs.ptr(),
			(uint32_t)positions.size(), indices.ptr(), (uint32_t)indices.size());
	if (!blast.raw_mesh) {
		fail(r_output, "PhysXBlastAuthoring: NvBlastExtAuthoringCreateMesh failed.");
		return;
	}
	// One facet per triangle, in order.
	blast.raw_mesh->setMaterialId(p_input.triangle_surfaces.ptr());

	if (!stage(STAGE_CLEANING, String())) {
		return;
	}
	// FractureTool requires a closed (watertight), self-intersection-free,
	// open-edge-free mesh (NvBlastExtAuthoringMeshCleaner.h) -- a Godot
	// PrimitiveMesh like BoxMesh doesn't weld vertices across face
	// boundaries (24 verts for 6 faces, not 8 shared corners), which reads
	// as "open edges" to Blast. Voronoi/Slicing tolerated that in practice;
	// Cutout's boolean CSG did not (crashed/produced a 1-chunk degenerate
	// result with "Not equal number of starting and ending vertices" until
	// this was added) -- clean unconditionally rather than special-case it
	// per pattern, since a strictly-valid mesh can only help the others too.
	// A mesh that's already closed (welded, every edge shared by two triangles) skips it: there's nothing to clean.
	// What the cleaner returns is checked against the source: empty or reaching outside the source's bounds (it broke
	// the shape), and the raw mesh is fractured instead; smaller than the source (it dropped pieces that weren't part
	// of a closed solid, a stray triangle), and it's kept, with a warning.
	if (!is_closed(indices)) {
		blast.mesh_cleaner = NvBlastExtAuthoringCreateMeshCleaner();
		blast.cleaned_mesh = blast.mesh_cleaner->cleanMesh(blast.raw_mesh);
		if (!blast.cleaned_mesh) {
			warn(r_output, "PhysXBlastAuthoring: the mesh cleaner failed; fracturing the mesh as it is.");
		} else {
			const NvcBounds3 &cb = blast.cleaned_mesh->getBoundingBox();
			const AABB cleaned_bounds(Vector3(cb.minimum.x, cb.minimum.y, cb.minimum.z),
					Vector3(cb.maximum.x - cb.minimum.x, cb.maximum.y - cb.minimum.y, cb.maximum.z - cb.minimum.z));
			const real_t tolerance = bounds.get_longest_axis_size() * (real_t)0.01;
			const AABB allowed = bounds.grow(tolerance);
			if (blast.cleaned_mesh->getFacetCount() == 0 || !allowed.encloses(cleaned_bounds)) {
				warn(r_output, vformat("PhysXBlastAuthoring: the mesh cleaner broke the mesh (bounds %s, was %s); fracturing the mesh as it is.", cleaned_bounds, bounds));
				blast.cleaned_mesh->release();
				blast.cleaned_mesh = nullptr;
			} else if (!cleaned_bounds.grow(tolerance).encloses(bounds)) {
				warn(r_output, vformat("PhysXBlastAuthoring: the mesh cleaner dropped parts of the mesh that weren't a closed solid (bounds %s, was %s).", cleaned_bounds, bounds));
			}
		}
	}
	Nv::Blast::Mesh *blast_mesh = blast.cleaned_mesh ? blast.cleaned_mesh : blast.raw_mesh;

	if (!stage(STAGE_FRACTURING, String())) {
		return;
	}
	blast.fracture_tool = NvBlastExtAuthoringCreateFractureTool();
	const Nv::Blast::Mesh *src_meshes[1] = { blast_mesh };
	blast.fracture_tool->setSourceMeshes(src_meshes, 1);

	GodotRandomGenerator rng;
	rng.seed(p_input.seed);

	if (p_input.pattern == PATTERN_SLICING) {
		// slicing() works in slice-counts-per-axis, not a single site count --
		// approximate p_site_count with a roughly-cube-root split so the
		// property still reads as "about this many pieces" regardless of
		// which pattern is picked. A small amount of offset/angle variation
		// by default so the cuts don't look like a perfectly uniform grid.
		int per_axis = 1;
		const int target = MAX(p_input.site_count, 1);
		while ((per_axis + 1) * (per_axis + 1) * (per_axis + 1) <= target) {
			per_axis++;
		}
		Nv::Blast::SlicingConfiguration conf;
		conf.x_slices = per_axis;
		conf.y_slices = per_axis;
		conf.z_slices = per_axis;
		conf.offset_variations = 0.2f;
		conf.angle_variations = 0.2f;
		const int32_t frac_result = blast.fracture_tool->slicing(0, conf, false, &rng);
		if (frac_result != 0) {
			fail(r_output, vformat("PhysXBlastAuthoring: slicing failed, code=%d.", frac_result));
			return;
		}
	} else if (p_input.pattern == PATTERN_CUTOUT) {
		if (!p_input.cutout_error.is_empty()) {
			fail(r_output, p_input.cutout_error);
			return;
		}
		// NvBlastExtAuthoringCutout.h's own doc comment claims "each
		// pixel is represented by one byte," but this SDK build's
		// actual createCutoutSet() (NvBlastExtAuthoringCutoutImpl.cpp)
		// reads 3 bytes/pixel -- a weighted RGB luminance, thresholded
		// near-binary (roughly: value below ~1/16 of max is interior/
		// fill, anything brighter is a wall/crack line). Confirmed by
		// reading that source directly after the stale 1-byte-per-
		// pixel assumption crashed with an out-of-bounds read -- it
		// walks 3 bytes per pixel regardless of what the header says.
		// Matches Unreal's own Blast integration, which independently
		// packs the same 3-bytes-per-pixel format for the same call.
		const PackedByteArray &gray = p_input.cutout_gray;
		const uint32_t pw = p_input.cutout_width;
		const uint32_t ph = p_input.cutout_height;
		PackedByteArray pixels;
		pixels.resize((int64_t)pw * ph * 3);
		{
			uint8_t *dst = pixels.ptrw();
			const uint8_t *src = gray.ptr();
			const int64_t n = (int64_t)pw * ph;
			for (int64_t i = 0; i < n; i++) {
				dst[i * 3 + 0] = src[i];
				dst[i * 3 + 1] = src[i];
				dst[i * 3 + 2] = src[i];
			}
		}

		// Refuse rather than risk the crash a floating (edge-
		// unreached) crack island caused twice while building this
		// pattern -- see cutout_pattern_has_floating_wall_island()'s
		// own comment for the full story. A native SDK crash can't
		// be caught from here (it takes the whole process down), so
		// this has to be ruled out before ever calling cutout().
		if (cutout_pattern_has_floating_wall_island(gray, pw, ph)) {
			fail(r_output, "PhysXBlastAuthoring: cutout_pattern rejected -- it has a crack/wall shape that "
						   "doesn't reach the image edge (a 'floating island'). Every crack line needs to "
						   "reach the pattern's boundary, the same way a real broken pane's cracks do -- "
						   "otherwise the fill region wraps around it with a hole in the middle, which is "
						   "known to crash the underlying Blast SDK's Cutout code rather than fail cleanly.");
			return;
		}
		Nv::Blast::CutoutSet *cutout_set = NvBlastExtAuthoringCreateCutoutSet();
		NvBlastExtAuthoringBuildCutoutSet(*cutout_set, pixels.ptr(), pw, ph,
				/*segmentationErrorThreshold*/ 1e-3f, /*snapThreshold*/ 1.0f,
				/*periodic*/ false, /*expandGaps*/ false);

		Nv::Blast::CutoutConfiguration conf;
		conf.cutoutSet = cutout_set;
		// Defaults otherwise: scale (-1,-1) auto-fits the pattern
		// to the chunk's own AABB, isRelativeTransform=true
		// centers it on the chunk -- exactly what a "just works"
		// first pass wants without per-mesh sizing math of our own.
		const int32_t frac_result = blast.fracture_tool->cutout(0, conf, false, &rng);
		cutout_set->release();
		if (frac_result != 0) {
			fail(r_output, vformat("PhysXBlastAuthoring: cutout failed, code=%d.", frac_result));
			return;
		}
	} else {
		blast.sites_gen = NvBlastExtAuthoringCreateVoronoiSitesGenerator(blast_mesh, &rng);
		blast.sites_gen->uniformlyGenerateSitesInMesh((uint32_t)MAX(p_input.site_count, 1));
		const NvcVec3 *sites = nullptr;
		const uint32_t site_count = blast.sites_gen->getVoronoiSites(sites);
		const int32_t frac_result = blast.fracture_tool->voronoiFracturing(0, site_count, sites, false);
		if (frac_result != 0) {
			fail(r_output, vformat("PhysXBlastAuthoring: voronoiFracturing failed, code=%d.", frac_result));
			return;
		}
	}

	if (!stage(STAGE_FINALIZING, String())) {
		return;
	}
	blast.fracture_tool->finalizeFracturing();

	if (!stage(STAGE_BONDS, String())) {
		return;
	}
	blast.bond_gen = NvBlastExtAuthoringCreateBondGenerator(&collision_builder);
	Nv::Blast::ConvexDecompositionParams cparams;
	// Voronoi cells and slicing planes of a convex source stay convex,
	// but a cutout region can easily be concave (an L-shaped brick, a
	// crack loop that isn't itself convex) -- allow real decomposition
	// there instead of silently producing a wrong/degenerate hull.
	cparams.maximumNumberOfHulls = (p_input.pattern == PATTERN_CUTOUT) ? 8 : 1;

	Nv::Blast::AuthoringResult *ares = NvBlastExtAuthoringProcessFracture(*blast.fracture_tool, *blast.bond_gen, collision_builder, cparams);
	if (!ares) {
		fail(r_output, "PhysXBlastAuthoring: NvBlastExtAuthoringProcessFracture failed.");
		return;
	}
	const uint32_t asset_size = NvBlastAssetGetSize(ares->asset, blast_authoring_log);
	r_output.asset_bytes.resize(asset_size);
	memcpy(r_output.asset_bytes.ptrw(), ares->asset, asset_size);

	Array &chunk_points = r_output.chunk_points;
	Array &chunk_normals = r_output.chunk_normals;
	Array &chunk_uvs = r_output.chunk_uvs;
	Array &chunk_surface_starts = r_output.chunk_surface_starts;
	PackedInt32Array &chunk_interior_starts = r_output.chunk_interior_starts;
	chunk_points.resize(ares->chunkCount);
	chunk_normals.resize(ares->chunkCount);
	chunk_uvs.resize(ares->chunkCount);
	chunk_surface_starts.resize(ares->chunkCount);
	chunk_interior_starts.resize(ares->chunkCount);
	CutFaceUVs cut_face_uvs;
	for (uint32_t c = 0; c < ares->chunkCount; c++) {
		PackedVector3Array points;
		PackedVector3Array vertex_normals;
		PackedVector2Array vertex_uvs;
		PackedInt32Array surface_starts;
		surface_starts.resize(surface_count);
		const uint32_t begin = ares->geometryOffset[c];
		const uint32_t end = ares->geometryOffset[c + 1];
		// Each source surface's faces in turn, then the cut ones (slot surface_count).
		for (int slot = 0; slot <= surface_count; slot++) {
			const bool interior = slot == surface_count;
			if (interior) {
				chunk_interior_starts.write[c] = points.size();
			} else {
				surface_starts.write[slot] = points.size();
			}
			for (uint32_t t = begin; t < end; t++) {
				const Nv::Blast::Triangle &tri = ares->geometry[t];
				const int tri_slot = tri.materialId == (int32_t)Nv::Blast::kMaterialInteriorId ? surface_count : (tri.materialId >= 0 && tri.materialId < surface_count ? tri.materialId : 0);
				if (tri_slot != slot) {
					continue;
				}
				const Nv::Blast::Vertex *corners[3] = { &tri.a, &tri.b, &tri.c };
				Vector3 p[3];
				for (int v = 0; v < 3; v++) {
					p[v] = Vector3(corners[v]->p.x, corners[v]->p.y, corners[v]->p.z);
				}
				// The winding PhysXDestructible3D renders them with: see _compute_chunk_surfaces().
				Vector3 face_normal = (p[2] - p[0]).cross(p[1] - p[0]);
				face_normal = face_normal.is_zero_approx() ? Vector3(0, 1, 0) : face_normal.normalized();
				for (int v = 0; v < 3; v++) {
					points.push_back(p[v]);
					if (interior) {
						vertex_normals.push_back(face_normal);
						vertex_uvs.push_back(cut_face_uvs.uv(p[v], face_normal));
					} else {
						Vector3 n(corners[v]->n.x, corners[v]->n.y, corners[v]->n.z);
						n = inside_out ? -n : n;
						vertex_normals.push_back(src_normals.is_empty() || n.is_zero_approx() ? face_normal : n.normalized());
						vertex_uvs.push_back(Vector2(corners[v]->uv[0].x, corners[v]->uv[0].y));
					}
				}
			}
		}
		chunk_points[c] = points;
		chunk_normals[c] = vertex_normals;
		chunk_uvs[c] = vertex_uvs;
		chunk_surface_starts[c] = surface_starts;
	}
	NvBlastExtAuthoringReleaseAuthoringResult(collision_builder, ares);

	// Each chunk's hull, so breaking doesn't cook it (as PhysXBlastAsset::cook_hulls() does). If none cooks, the asset
	// has none and they're cooked at runtime instead, as before.
	Array hulls;
	hulls.resize(chunk_points.size());
	int cooked = 0;
	for (int c = 0; c < chunk_points.size(); c++) {
		if (!stage(STAGE_HULLS, vformat("%d of %d", c + 1, chunk_points.size()))) {
			return;
		}
		const PackedByteArray hull = GodotPhysXShape3D::cook_convex(chunk_points[c]);
		cooked += hull.is_empty() ? 0 : 1;
		hulls[c] = hull;
	}
	if (cooked > 0 || chunk_points.is_empty()) {
		r_output.chunk_hulls = hulls;
		r_output.hull_format = GodotPhysXShape3D::cooked_convex_format();
	} else {
		r_output.warnings.push_back("The pieces' collision hulls couldn't be cooked ahead of time; they're cooked when the pieces first break off.");
	}
	r_output.done = true;
}

Ref<PhysXBlastAsset> PhysXBlastAuthoring::make_asset(const PhysXBlastFractureInput &p_input, const PhysXBlastFractureOutput &p_output) {
	if (!p_output.done) {
		return Ref<PhysXBlastAsset>();
	}
	Ref<PhysXBlastAsset> result;
	result.instantiate();
	result->set_asset_bytes(p_output.asset_bytes);
	result->set_chunk_points(p_output.chunk_points);
	result->set_chunk_normals(p_output.chunk_normals);
	result->set_chunk_uvs(p_output.chunk_uvs);
	result->set_chunk_interior_starts(p_output.chunk_interior_starts);
	result->set_chunk_surface_starts(p_output.chunk_surface_starts);
	result->set_surface_materials(p_input.surface_materials);
	if (!p_output.hull_format.is_empty()) {
		result->set_chunk_hulls(p_output.chunk_hulls);
		result->set_hull_format(p_output.hull_format);
	}
	return result;
}

void PhysXBlastFractureTask::_bind_methods() {
	ClassDB::bind_method(D_METHOD("cancel"), &PhysXBlastFractureTask::cancel);
	ClassDB::bind_method(D_METHOD("is_running"), &PhysXBlastFractureTask::is_running);
	ClassDB::bind_method(D_METHOD("is_cancelled"), &PhysXBlastFractureTask::is_cancelled);
	ClassDB::bind_method(D_METHOD("get_stage"), &PhysXBlastFractureTask::get_stage);
	ClassDB::bind_method(D_METHOD("get_stage_count"), &PhysXBlastFractureTask::get_stage_count);
	ClassDB::bind_method(D_METHOD("get_stage_description"), &PhysXBlastFractureTask::get_stage_description);
	ClassDB::bind_method(D_METHOD("get_asset"), &PhysXBlastFractureTask::get_asset);
	ClassDB::bind_method(D_METHOD("get_last_error"), &PhysXBlastFractureTask::get_last_error);
	ClassDB::bind_method(D_METHOD("get_last_warnings"), &PhysXBlastFractureTask::get_last_warnings);

	ADD_SIGNAL(MethodInfo("stage_changed", PropertyInfo(Variant::INT, "stage"), PropertyInfo(Variant::INT, "stage_count"), PropertyInfo(Variant::STRING, "description")));
	ADD_SIGNAL(MethodInfo("finished", PropertyInfo(Variant::OBJECT, "asset", PROPERTY_HINT_RESOURCE_TYPE, "PhysXBlastAsset")));
}

bool PhysXBlastFractureTask::report_stage(int p_stage, const String &p_detail) {
	const GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	if (cancel_requested.is_set() || !server || server->is_finishing()) {
		return false;
	}
	String text = PhysXBlastAuthoring::stage_name(p_stage);
	if (!p_detail.is_empty()) {
		text += vformat(" (%s)", p_detail);
	}
	stage.set(p_stage);
	{
		MutexLock lock(description_mutex);
		description = text;
	}
	callable_mp(this, &PhysXBlastFractureTask::_emit_stage).call_deferred(p_stage, text);
	return true;
}

void PhysXBlastFractureTask::_work() {
	PhysXBlastAuthoring::run(input, output, this);
	GodotPhysXServer3D::get_singleton()->end_background_job();
	callable_mp(this, &PhysXBlastFractureTask::_finish).call_deferred();
}

void PhysXBlastFractureTask::_emit_stage(int p_stage, const String &p_description) {
	emit_signal(SNAME("stage_changed"), p_stage, (int)PhysXBlastAuthoring::STAGE_COUNT, p_description);
}

void PhysXBlastFractureTask::_finish() {
	if (task_id != WorkerThreadPool::INVALID_TASK_ID) {
		WorkerThreadPool::get_singleton()->wait_for_task_completion(task_id);
		task_id = WorkerThreadPool::INVALID_TASK_ID;
	}
	cancelled = output.cancelled;
	last_error = output.error;
	last_warnings = output.warnings;
	if (!cancelled) {
		asset = PhysXBlastAuthoring::make_asset(input, output);
	}
	input = PhysXBlastFractureInput();
	output = PhysXBlastFractureOutput();
	running.clear();
	// Whoever started it may have let go of it already: this may be its last reference, kept until the signal is out.
	Ref<PhysXBlastFractureTask> self = keep_alive;
	keep_alive.unref();
	emit_signal(SNAME("finished"), asset);
}

void PhysXBlastFractureTask::cancel() {
	cancel_requested.set();
}

String PhysXBlastFractureTask::get_stage_description() {
	MutexLock lock(description_mutex);
	return description;
}

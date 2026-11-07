/**************************************************************************/
/*  physx_blast_preview.cpp                                               */
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

#include "physx_blast_preview.h"

#ifdef GODOT_PHYSX_BLAST

#include "../blast/physx_blast_asset.h"

#include "scene/resources/material.h"
#include "scene/resources/mesh.h"

static Color _blast_preview_chunk_color(int p_index) {
	// Golden-ratio hue step: successive chunks get visually distinct colors
	// with no lookup table and no risk of two adjacent chunks landing on the
	// same hue.
	const double hue = Math::fmod(p_index * 0.6180339887, 1.0);
	return Color::from_hsv(hue, 0.55, 0.85);
}

// The center of a closed triangle soup's volume: the mean of its vertices would lean toward wherever it's most finely
// tessellated. Tetrahedra to the first point, so a mesh far from the origin keeps its precision.
static Vector3 _blast_preview_center_of_volume(const PackedVector3Array &p_points) {
	const Vector3 base = p_points[0];
	Vector3 weighted;
	real_t volume = 0.0;
	for (int t = 0; t + 2 < p_points.size(); t += 3) {
		const Vector3 a = p_points[t] - base;
		const Vector3 b = p_points[t + 1] - base;
		const Vector3 c = p_points[t + 2] - base;
		const real_t v = a.dot(b.cross(c));
		weighted += (a + b + c) * v;
		volume += v;
	}
	if (Math::is_zero_approx(volume)) {
		// Flat, or not closed: no volume to go by.
		Vector3 mean;
		for (int i = 0; i < p_points.size(); i++) {
			mean += p_points[i];
		}
		return mean / p_points.size();
	}
	// A tetrahedron's center is its corners' mean: (a + b + c) / 4 from the shared corner.
	return base + weighted / (volume * 4.0);
}

Ref<ArrayMesh> physx_blast_build_preview_mesh(const Ref<PhysXBlastAsset> &p_asset) {
	Ref<ArrayMesh> preview_mesh;
	preview_mesh.instantiate();
	physx_blast_update_preview_mesh(preview_mesh, p_asset, 0.0);
	return preview_mesh;
}

void physx_blast_update_preview_mesh(const Ref<ArrayMesh> &p_mesh, const Ref<PhysXBlastAsset> &p_asset, real_t p_explode) {
	ERR_FAIL_COND(p_mesh.is_null());
	p_mesh->clear_surfaces();
	if (p_asset.is_null()) {
		return;
	}

	const Array points = p_asset->get_chunk_points();
	const real_t explode = CLAMP(p_explode, (real_t)0.0, (real_t)1.0);
	const PackedVector3Array whole = points.is_empty() ? PackedVector3Array() : (PackedVector3Array)points[0];
	const Vector3 origin = (explode > 0.0 && whole.size() >= 3) ? _blast_preview_center_of_volume(whole) : Vector3();
	for (int i = 1; i < points.size(); i++) {
		PackedVector3Array tri = points[i];
		if (tri.size() < 3) {
			continue;
		}
		if (explode > 0.0) {
			const Vector3 push = (_blast_preview_center_of_volume(tri) - origin) * explode;
			Vector3 *tw = tri.ptrw();
			for (int v = 0; v < tri.size(); v++) {
				tw[v] += push;
			}
		}

		PackedVector3Array normals;
		normals.resize(tri.size());
		for (int t = 0; t + 2 < tri.size(); t += 3) {
			// Same winding fix PhysXDestructible3D uses for this same
			// triangle-soup data -- (c-a).cross(b-a), not the more intuitive
			// (b-a).cross(c-a) -- see that node's normal computation.
			const Vector3 n = (tri[t + 2] - tri[t]).cross(tri[t + 1] - tri[t]).normalized();
			normals.set(t, n);
			normals.set(t + 1, n);
			normals.set(t + 2, n);
		}

		Array arrays;
		arrays.resize(Mesh::ARRAY_MAX);
		arrays[Mesh::ARRAY_VERTEX] = tri;
		arrays[Mesh::ARRAY_NORMAL] = normals;
		p_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_albedo(_blast_preview_chunk_color(i));
		p_mesh->surface_set_material(p_mesh->get_surface_count() - 1, mat);
	}
}

#endif // GODOT_PHYSX_BLAST

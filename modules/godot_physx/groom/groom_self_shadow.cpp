/**************************************************************************/
/*  groom_self_shadow.cpp                                                 */
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

#include "groom_self_shadow.h"

#include "core/math/aabb.h"
#include "core/math/basis.h"
#include "core/math/vector3i.h"
#include "core/object/worker_thread_pool.h"
#include "core/templates/hashfuncs.h"
#include "core/templates/local_vector.h"

namespace {

// The grid's cells are about a segment long, at most this many across: in dense hair a bigger cell holds hundreds of
// segments for every ray through it to test.
constexpr int MAX_GRID_CELLS = 256;
// Above this many points, every other point (or fewer) is baked and the rest are blended along their strand: shadowing
// changes slowly along a strand, and the braids demo's 1.4M points would otherwise take minutes.
constexpr int MAX_BAKED_POINTS = 100000;

struct Segment {
	Vector3 a;
	Vector3 b;
	float radius = 0.0f;
	int strand = 0;
};

// The squared distance between segments [p0, p1] and [q0, q1] (Ericson, Real-Time Collision Detection, 5.1.9). Hair is
// small: a segment is millimeters long, its squared length far under CMP_EPSILON, so the tests for a segment that's a
// point, and for two that are parallel, scale with the segments.
float segment_distance_squared(const Vector3 &p0, const Vector3 &p1, const Vector3 &q0, const Vector3 &q1) {
	constexpr float POINT_LENGTH_SQUARED = 1e-16f; // 0.1 nm, squared
	const Vector3 d1 = p1 - p0;
	const Vector3 d2 = q1 - q0;
	const Vector3 r = p0 - q0;
	const float a = d1.dot(d1);
	const float e = d2.dot(d2);
	const float f = d2.dot(r);
	float s;
	float t;
	if (e <= POINT_LENGTH_SQUARED) {
		t = 0.0f;
		s = a > POINT_LENGTH_SQUARED ? CLAMP(-d1.dot(r) / a, 0.0f, 1.0f) : 0.0f;
	} else {
		const float c = d1.dot(r);
		const float b = d1.dot(d2);
		const float denom = a * e - b * b;
		// Parallel: the sine of the angle between them squared, under a millionth.
		s = denom > 1e-6f * a * e ? CLAMP((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
		t = (b * s + f) / e;
		if (t < 0.0f) {
			t = 0.0f;
			s = a > POINT_LENGTH_SQUARED ? CLAMP(-c / a, 0.0f, 1.0f) : 0.0f;
		} else if (t > 1.0f) {
			t = 1.0f;
			s = a > POINT_LENGTH_SQUARED ? CLAMP((b - c) / a, 0.0f, 1.0f) : 0.0f;
		}
	}
	return (p0 + d1 * s).distance_squared_to(q0 + d2 * t);
}

struct Baker {
	const Vector3 *points = nullptr;
	const int *offsets = nullptr;
	const float *widths = nullptr;
	int strand_count = 0;
	int point_count = 0;
	float reach = 0.0f;

	LocalVector<int> strand_of_point;
	LocalVector<Segment> segments;
	Vector3 origin;
	float cell = 1.0f;
	Vector3i dims;
	LocalVector<int> cell_start;
	LocalVector<int> cell_items;
	LocalVector<Vector3> directions;
	LocalVector<int> samples;
	float *out = nullptr;

	bool cell_of(const Vector3 &p_point, Vector3i &r_cell) const {
		const Vector3 f = (p_point - origin) / cell;
		r_cell = Vector3i((int)Math::floor(f.x), (int)Math::floor(f.y), (int)Math::floor(f.z));
		return r_cell.x >= 0 && r_cell.y >= 0 && r_cell.z >= 0 && r_cell.x < dims.x && r_cell.y < dims.y && r_cell.z < dims.z;
	}
	int cell_index(const Vector3i &p_cell) const { return (p_cell.z * dims.y + p_cell.y) * dims.x + p_cell.x; }

	void build() {
		strand_of_point.resize(point_count);
		AABB box(points[0], Vector3());
		float widest = 0.0f;
		for (int s = 0; s < strand_count; s++) {
			for (int p = offsets[s]; p < offsets[s + 1]; p++) {
				strand_of_point[p] = s;
				box.expand_to(points[p]);
				widest = MAX(widest, widths[p]);
			}
			for (int p = offsets[s]; p + 1 < offsets[s + 1]; p++) {
				Segment segment;
				segment.a = points[p];
				segment.b = points[p + 1];
				segment.radius = (widths[p] + widths[p + 1]) * 0.25f;
				segment.strand = s;
				segments.push_back(segment);
			}
		}
		box = box.grow(widest + reach * 0.01f);
		float length = 0.0f;
		for (const Segment &segment : segments) {
			length += segment.a.distance_to(segment.b);
		}
		length /= MAX((float)segments.size(), 1.0f);
		cell = CLAMP(length, box.get_longest_axis_size() / MAX_GRID_CELLS, reach * 0.25f);
		origin = box.position;
		dims = Vector3i(MAX(1, (int)Math::ceil(box.size.x / cell)), MAX(1, (int)Math::ceil(box.size.y / cell)), MAX(1, (int)Math::ceil(box.size.z / cell)));

		// Each segment in every cell its bounds touch: counted, then placed.
		const int cell_count = dims.x * dims.y * dims.z;
		cell_start.resize(cell_count + 1);
		for (int i = 0; i <= cell_count; i++) {
			cell_start[i] = 0;
		}
		for (int pass = 0; pass < 2; pass++) {
			LocalVector<int> fill;
			if (pass == 1) {
				for (int i = 0; i < cell_count; i++) {
					cell_start[i + 1] += cell_start[i];
				}
				cell_items.resize(cell_start[cell_count]);
				fill.resize(cell_count);
				for (int i = 0; i < cell_count; i++) {
					fill[i] = cell_start[i];
				}
			}
			for (uint32_t i = 0; i < segments.size(); i++) {
				const Segment &segment = segments[i];
				AABB bounds(segment.a, Vector3());
				bounds.expand_to(segment.b);
				bounds = bounds.grow(segment.radius);
				Vector3i lo;
				Vector3i hi;
				cell_of(bounds.position, lo);
				cell_of(bounds.get_end(), hi);
				lo = lo.clamp(Vector3i(), dims - Vector3i(1, 1, 1));
				hi = hi.clamp(Vector3i(), dims - Vector3i(1, 1, 1));
				for (int z = lo.z; z <= hi.z; z++) {
					for (int y = lo.y; y <= hi.y; y++) {
						for (int x = lo.x; x <= hi.x; x++) {
							const int c = cell_index(Vector3i(x, y, z));
							if (pass == 0) {
								cell_start[c + 1]++;
							} else {
								cell_items[fill[c]++] = (int)i;
							}
						}
					}
				}
			}
		}
	}

	// The strands along one ray, up to GROOM_MAX_SCATTER_EVENTS, its own strand left out.
	int count_hits(const Vector3 &p_from, const Vector3 &p_direction, int p_strand) const {
		int hits[GROOM_MAX_SCATTER_EVENTS];
		int hit_count = 0;
		const Vector3 to = p_from + p_direction * reach;
		int last_cell = -1;
		// Half-cell steps: a cell only grazed by the ray can be missed, which costs a little shadow, not a wrong one.
		for (float t = cell * 0.25f; t < reach + cell * 0.5f; t += cell * 0.5f) {
			Vector3i c;
			if (!cell_of(p_from + p_direction * MIN(t, reach), c)) {
				break;
			}
			const int index = cell_index(c);
			if (index == last_cell) {
				continue;
			}
			last_cell = index;
			for (int k = cell_start[index]; k < cell_start[index + 1]; k++) {
				const int si = cell_items[k];
				const Segment &segment = segments[si];
				if (segment.strand == p_strand) {
					continue;
				}
				if (segment_distance_squared(p_from, to, segment.a, segment.b) > segment.radius * segment.radius) {
					continue;
				}
				// A strand is one scattering event, however many of its segments the ray grazes.
				bool seen = false;
				for (int h = 0; h < hit_count; h++) {
					seen = seen || segments[hits[h]].strand == segment.strand;
				}
				if (!seen) {
					hits[hit_count++] = si;
					if (hit_count == GROOM_MAX_SCATTER_EVENTS) {
						return hit_count;
					}
				}
			}
		}
		return hit_count;
	}

	void bake(uint32_t p_sample) {
		const int point = samples[p_sample];
		const Vector3 from = points[point];
		const int strand = strand_of_point[point];
		// The directions turned differently at every point: the same few would show as a pattern along the strands.
		const uint32_t h1 = hash_murmur3_one_32((uint32_t)point);
		const uint32_t h2 = hash_murmur3_one_32(h1);
		const float z = (h1 & 0xffff) / 32767.5f - 1.0f;
		const float phi = (h2 & 0xffff) / 65535.0f * Math::TAU;
		const float rr = Math::sqrt(MAX(1.0f - z * z, 0.0f));
		const Vector3 axis(rr * Math::cos(phi), rr * Math::sin(phi), z);
		const Basis turn(axis.normalized(), ((h2 >> 16) / 65535.0f) * Math::TAU);

		float sh[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
		for (const Vector3 &base : directions) {
			const Vector3 direction = turn.xform(base);
			const int hits = count_hits(from, direction, strand);
			const float events = hits > 0 ? (float)hits : -(float)GROOM_MAX_SCATTER_EVENTS;
			sh[0] += events * 0.282095f;
			sh[1] += events * 0.488603f * direction.y;
			sh[2] += events * 0.488603f * direction.z;
			sh[3] += events * 0.488603f * direction.x;
		}
		const float weight = 4.0f * Math::PI / directions.size();
		for (int i = 0; i < 4; i++) {
			out[point * 4 + i] = sh[i] * weight;
		}
	}

	static void bake_task(void *p_userdata, uint32_t p_index) {
		static_cast<Baker *>(p_userdata)->bake(p_index);
	}
};

} // namespace

PackedFloat32Array groom_bake_self_shadow(const PackedVector3Array &p_points, const PackedInt32Array &p_strand_offsets, const PackedFloat32Array &p_widths, int p_rays, float p_reach) {
	PackedFloat32Array result;
	const int strand_count = p_strand_offsets.size() - 1;
	ERR_FAIL_COND_V(strand_count < 1 || p_widths.size() != p_points.size() || p_strand_offsets[strand_count] != p_points.size(), result);
	ERR_FAIL_COND_V(p_rays < 1 || p_reach <= 0.0f, result);
	result.resize(p_points.size() * 4);

	Baker baker;
	baker.points = p_points.ptr();
	baker.offsets = p_strand_offsets.ptr();
	baker.widths = p_widths.ptr();
	baker.strand_count = strand_count;
	baker.point_count = p_points.size();
	baker.reach = p_reach;
	baker.out = result.ptrw();
	baker.build();

	// Evenly over the sphere (a Fibonacci spiral).
	for (int k = 0; k < p_rays; k++) {
		const float y = 1.0f - 2.0f * (k + 0.5f) / p_rays;
		const float r = Math::sqrt(MAX(1.0f - y * y, 0.0f));
		const float phi = k * 2.39996323f;
		baker.directions.push_back(Vector3(r * Math::cos(phi), y, r * Math::sin(phi)));
	}

	// Every stride-th point of each strand, and always its tip.
	const int stride = MAX(1, (int)Math::ceil((double)p_points.size() / MAX_BAKED_POINTS));
	for (int s = 0; s < strand_count; s++) {
		const int from = p_strand_offsets[s];
		const int to = p_strand_offsets[s + 1];
		for (int p = from; p < to; p += stride) {
			baker.samples.push_back(p);
		}
		if (to > from && (to - 1 - from) % stride != 0) {
			baker.samples.push_back(to - 1);
		}
	}
	const WorkerThreadPool::GroupID group = WorkerThreadPool::get_singleton()->add_native_group_task(&Baker::bake_task, &baker, baker.samples.size(), -1, false, "PhysXGroom self-shadow bake");
	WorkerThreadPool::get_singleton()->wait_for_group_task_completion(group);

	// The points between baked ones, blended along their strand.
	if (stride > 1) {
		float *w = result.ptrw();
		for (int s = 0; s < strand_count; s++) {
			const int from = p_strand_offsets[s];
			const int to = p_strand_offsets[s + 1];
			for (int p = from; p < to; p++) {
				const int offset = (p - from) % stride;
				if (offset == 0 || p == to - 1) {
					continue;
				}
				const int a = p - offset;
				const int b = MIN(a + stride, to - 1);
				const float t = (float)(p - a) / (float)(b - a);
				for (int i = 0; i < 4; i++) {
					w[p * 4 + i] = Math::lerp(w[a * 4 + i], w[b * 4 + i], t);
				}
			}
		}
	}
	return result;
}

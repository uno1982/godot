#[compute]
#version 450

#include "mpm_block_inc.glsl"

// GPU marching tetrahedra over the block-sparse density field (surf_i). Indirect
// dispatch: one workgroup per active block, one thread per cell. Each cell is
// split into six tets sharing the body diagonal; triangles are appended
// non-indexed via an atomic counter, normals from the density gradient.

layout(set = 1, binding = 0, std430) restrict buffer MeshVerts { vec4 mverts[]; };
layout(set = 1, binding = 1, std430) restrict buffer MeshNorms { vec4 mnorms[]; };
layout(set = 1, binding = 2, std430) restrict buffer MeshCount {
	uint tri_count;
	uint tri_budget;
};

layout(local_size_x = BCELLS) in;

#define ISO (extra.z)

const ivec3 CORNER[8] = ivec3[8](
		ivec3(0, 0, 0), ivec3(1, 0, 0), ivec3(1, 1, 0), ivec3(0, 1, 0),
		ivec3(0, 0, 1), ivec3(1, 0, 1), ivec3(1, 1, 1), ivec3(0, 1, 1));
const int TET[24] = int[24](0, 1, 2, 6, 0, 2, 3, 6, 0, 3, 7, 6, 0, 7, 4, 6, 0, 4, 5, 6, 0, 5, 1, 6);
const int EA[6] = int[6](0, 0, 0, 1, 1, 2);
const int EB[6] = int[6](1, 2, 3, 2, 3, 3);
const int INC[12] = int[12](0, 1, 2, 0, 3, 4, 1, 3, 5, 2, 4, 5);

float dsample(ivec3 c) {
	int slot = block_lookup(c >> 2);
	if (slot < 0) {
		return 0.0;
	}
	ivec3 lc = c & 3;
	return float(surf_i[slot * BCELLS + (lc.z * 4 + lc.y) * 4 + lc.x]) / SURF_FIXED;
}

float dsample_w(vec3 wp) {
	vec3 g = (wp - ORIGIN) / DX;
	ivec3 b = ivec3(floor(g));
	vec3 f = g - vec3(b);
	float c00 = mix(dsample(b + ivec3(0, 0, 0)), dsample(b + ivec3(1, 0, 0)), f.x);
	float c10 = mix(dsample(b + ivec3(0, 1, 0)), dsample(b + ivec3(1, 1, 0)), f.x);
	float c01 = mix(dsample(b + ivec3(0, 0, 1)), dsample(b + ivec3(1, 0, 1)), f.x);
	float c11 = mix(dsample(b + ivec3(0, 1, 1)), dsample(b + ivec3(1, 1, 1)), f.x);
	return mix(mix(c00, c10, f.y), mix(c01, c11, f.y), f.z);
}

vec3 dnormal(vec3 wp) {
	float e = DX * 0.5;
	vec3 n = vec3(
			dsample_w(wp + vec3(e, 0, 0)) - dsample_w(wp - vec3(e, 0, 0)),
			dsample_w(wp + vec3(0, e, 0)) - dsample_w(wp - vec3(0, e, 0)),
			dsample_w(wp + vec3(0, 0, e)) - dsample_w(wp - vec3(0, 0, e)));
	float len = length(n);
	return len > 1e-6 ? -n / len : vec3(0.0, 1.0, 0.0);
}

void write_tri(vec3 p0, vec3 p1, vec3 p2) {
	float mx = DX * 3.0;
	if (distance(p0, p1) > mx || distance(p1, p2) > mx || distance(p2, p0) > mx) {
		return;
	}
	vec3 fn = cross(p1 - p0, p2 - p0);
	vec3 outn = dnormal((p0 + p1 + p2) * (1.0 / 3.0));
	if (dot(fn, outn) < 0.0) {
		vec3 t = p1;
		p1 = p2;
		p2 = t;
	}
	uint slot = atomicAdd(tri_count, 1u);
	if (slot >= tri_budget) {
		return;
	}
	uint o = slot * 3u;
	mverts[o + 0] = vec4(p0, 0.0);
	mverts[o + 1] = vec4(p1, 0.0);
	mverts[o + 2] = vec4(p2, 0.0);
	mnorms[o + 0] = vec4(dnormal(p0), 0.0);
	mnorms[o + 1] = vec4(dnormal(p1), 0.0);
	mnorms[o + 2] = vec4(dnormal(p2), 0.0);
}

void main() {
	uint bslot = gl_WorkGroupID.x;
	if (bslot >= bcounts[0]) {
		return;
	}
	uint lin = gl_LocalInvocationID.x;
	ivec3 bc = unpack_block(bkey[bslot]);
	ivec3 lc = ivec3(int(lin) & 3, (int(lin) >> 2) & 3, int(lin) >> 4);
	ivec3 cell = bc * BLK + lc;

	float cv[8];
	vec3 cp[8];
	for (int k = 0; k < 8; k++) {
		ivec3 cc = cell + CORNER[k];
		cv[k] = dsample(cc);
		cp[k] = ORIGIN + vec3(cc) * DX;
	}

	for (int t = 0; t < 6; t++) {
		int lcn[4] = int[4](TET[t * 4 + 0], TET[t * 4 + 1], TET[t * 4 + 2], TET[t * 4 + 3]);
		int mask = 0;
		for (int k = 0; k < 4; k++) {
			if (cv[lcn[k]] >= ISO) {
				mask |= 1 << k;
			}
		}
		if (mask == 0 || mask == 15) {
			continue;
		}

		vec3 ev[6];
		for (int e = 0; e < 6; e++) {
			int i = EA[e], j = EB[e];
			if (((mask >> i) & 1) != ((mask >> j) & 1)) {
				float vi = cv[lcn[i]], vj = cv[lcn[j]];
				float tt = clamp((ISO - vi) / (vj - vi + 1e-8), 0.0, 1.0);
				ev[e] = mix(cp[lcn[i]], cp[lcn[j]], tt);
			}
		}

		int inside = bitCount(mask);
		if (inside == 1 || inside == 3) {
			int lone = 0;
			for (int k = 0; k < 4; k++) {
				bool bit = ((mask >> k) & 1) != 0;
				if ((inside == 1) == bit) {
					lone = k;
				}
			}
			write_tri(ev[INC[lone * 3 + 0]], ev[INC[lone * 3 + 1]], ev[INC[lone * 3 + 2]]);
		} else {
			int in0 = -1, in1 = -1, out0 = -1, out1 = -1;
			for (int k = 0; k < 4; k++) {
				if (((mask >> k) & 1) != 0) {
					if (in0 < 0) {
						in0 = k;
					} else {
						in1 = k;
					}
				} else {
					if (out0 < 0) {
						out0 = k;
					} else {
						out1 = k;
					}
				}
			}
			int e_ac = -1, e_ad = -1, e_bc = -1, e_bd = -1;
			for (int e = 0; e < 6; e++) {
				int a = EA[e], b = EB[e];
				if ((a == in0 && b == out0) || (a == out0 && b == in0)) { e_ac = e; }
				if ((a == in0 && b == out1) || (a == out1 && b == in0)) { e_ad = e; }
				if ((a == in1 && b == out0) || (a == out0 && b == in1)) { e_bc = e; }
				if ((a == in1 && b == out1) || (a == out1 && b == in1)) { e_bd = e; }
			}
			write_tri(ev[e_ac], ev[e_ad], ev[e_bd]);
			write_tri(ev[e_ac], ev[e_bd], ev[e_bc]);
		}
	}
}

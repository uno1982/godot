#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = GROUP) in;

#define SURF_H (extra.w) // isosurface SPH kernel radius, world metres
#define SURF_BOOST (bmin.w) // per-particle mass multiplier

// Per particle: poly6 SPH scatter of mass into surf_i (block-indexed, same
// layout as the grid). Kernel radius is clamped so the reach fits a 3x3 block
// neighbourhood -- a bigger particle still inflates the surface, just not past
// one extra block (edge erosion, tuned out by the iso level). surf_i is cleared
// by the caller.
void main() {
	uint id = gl_GlobalInvocationID.x;
	if (id >= uint(PCOUNT)) {
		return;
	}
	vec3 x = particles[id].x_d.xyz;

	float h = max(SURF_H, DX);
	float h2 = h * h;
	float norm = (315.0 / (64.0 * 3.14159265 * pow(h, 9.0))) * PMASS * SURF_BOOST;

	int R = min(int(ceil(h / DX)) + 1, 4);
	ivec3 base = ivec3(floor((x - ORIGIN) / DX));

	// Resolve the block neighbourhood covering base-R .. base+R.
	ivec3 lo_bc = (base - R) >> 2;
	int kbs[27];
	for (int a = 0; a < 3; a++) {
		for (int b = 0; b < 3; b++) {
			for (int c = 0; c < 3; c++) {
				kbs[(c * 3 + b) * 3 + a] = block_lookup(lo_bc + ivec3(a, b, c));
			}
		}
	}

	for (int dz = -R; dz <= R; dz++) {
		for (int dy = -R; dy <= R; dy++) {
			for (int dx = -R; dx <= R; dx++) {
				ivec3 nc = base + ivec3(dx, dy, dz);
				ivec3 d = (nc >> 2) - lo_bc;
				if (any(lessThan(d, ivec3(0))) || any(greaterThan(d, ivec3(2)))) {
					continue;
				}
				int slot = kbs[(d.z * 3 + d.y) * 3 + d.x];
				if (slot < 0) {
					continue;
				}
				vec3 np = ORIGIN + vec3(nc) * DX;
				float dd = dot(np - x, np - x);
				if (dd >= h2) {
					continue;
				}
				ivec3 lc = nc & 3;
				int idx = slot * BCELLS + (lc.z * 4 + lc.y) * 4 + lc.x;
				float t = h2 - dd;
				atomicAdd(surf_i[idx], int(norm * t * t * t * SURF_FIXED));
			}
		}
	}
}

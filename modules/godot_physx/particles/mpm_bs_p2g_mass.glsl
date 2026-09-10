#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = GROUP) in;

// P2G pass 1 (fluid): scatter mass so pass 2 can read a local density.
void main() {
	uint id = gl_GlobalInvocationID.x;
	if (id >= uint(PCOUNT)) {
		return;
	}
	vec3 x = particles[id].x_d.xyz;
	ivec3 base;
	vec3 fx;
	grid_local(x, base, fx);
	vec3 wx = bspline(fx.x), wy = bspline(fx.y), wz = bspline(fx.z);

	ivec3 base_bc = base >> 2;
	int bs[8];
	resolve_blocks(base_bc, (base + 2) >> 2, bs);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				int idx = cell_slot(base + ivec3(i, j, k), base_bc, bs);
				if (idx < 0) {
					continue;
				}
				float w = wx[i] * wy[j] * wz[k];
				atomicAdd(grid_i[idx * 4 + 0], int(w * PMASS * FIXED));
			}
		}
	}
}

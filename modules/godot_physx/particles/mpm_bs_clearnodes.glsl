#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = BCELLS) in;

// Indirect dispatch: one workgroup per active block, one thread per cell. Zero
// the block's grid accumulators (slots are reassigned every substep, so last
// step's data is stale).
void main() {
	uint bslot = gl_WorkGroupID.x;
	if (bslot >= bcounts[0]) {
		return;
	}
	uint idx = bslot * uint(BCELLS) + gl_LocalInvocationID.x;
	int b = int(idx) * 4;
	grid_i[b + 0] = 0;
	grid_i[b + 1] = 0;
	grid_i[b + 2] = 0;
	grid_i[b + 3] = 0;
	grid_v[idx] = vec4(0.0);
}

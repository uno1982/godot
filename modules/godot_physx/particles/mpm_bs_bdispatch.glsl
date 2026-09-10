#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = 1) in;

// One thread. Clamp the active-block count and write the indirect group count
// for the per-block passes (grid / couple / clearnodes): one workgroup per
// block, local_size_x = 64 = one thread per cell.
void main() {
	uint n = min(bcounts[0], MAXB);
	bcounts[0] = n;
	bcounts[1] = n;
	bcounts[2] = 1u;
	bcounts[3] = 1u;
}

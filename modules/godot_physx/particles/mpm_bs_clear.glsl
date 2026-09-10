#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = GROUP) in;

// Reset the block hash table and the active-block counter. The node pool is
// cleared per-block by mpm_bs_clearnodes (indirect) once the count is known.
void main() {
	uint i = gl_GlobalInvocationID.x;
	if (i < HASHN) {
		bhash[i] = HASH_EMPTY;
		bhash_val[i] = HASH_EMPTY;
	}
	if (i == 0u) {
		bcounts[0] = 0u;
	}
}

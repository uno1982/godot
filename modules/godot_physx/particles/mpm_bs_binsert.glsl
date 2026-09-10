#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = GROUP) in;

// Per particle: register the (<=8) blocks its 27-cell P2G stencil spans.
void main() {
	uint id = gl_GlobalInvocationID.x;
	if (id >= uint(PCOUNT)) {
		return;
	}
	vec3 x = particles[id].x_d.xyz;
	ivec3 base;
	vec3 fx;
	grid_local(x, base, fx);
	ivec3 base_bc = base >> 2;

	for (int dz = 0; dz < 2; dz++) {
		for (int dy = 0; dy < 2; dy++) {
			for (int dx = 0; dx < 2; dx++) {
				block_touch(base_bc + ivec3(dx, dy, dz));
			}
		}
	}
}

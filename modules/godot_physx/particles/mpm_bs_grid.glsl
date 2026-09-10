#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = BCELLS) in;

// Indirect: one workgroup per active block, one thread per cell. Normalise
// momentum, apply gravity. Boundless -- no side walls or ceiling; only an
// implicit floor a couple of cells above the anchor keeps a floor-less scene
// from falling forever (demos add explicit floor/wall colliders).
void main() {
	uint bslot = gl_WorkGroupID.x;
	if (bslot >= bcounts[0]) {
		return;
	}
	uint lin = gl_LocalInvocationID.x;
	uint idx = bslot * uint(BCELLS) + lin;

	int b = int(idx) * 4;
	float m = float(grid_i[b + 0]) / FIXED;
	if (m <= 1e-9) {
		grid_v[idx] = vec4(0.0);
		return;
	}
	vec3 mom = vec3(grid_i[b + 1], grid_i[b + 2], grid_i[b + 3]) / FIXED;
	vec3 vel = mom / m + DT * GRAV;

	ivec3 bc = unpack_block(bkey[bslot]);
	ivec3 lc = ivec3(int(lin) & 3, (int(lin) >> 2) & 3, int(lin) >> 4);
	ivec3 c = bc * BLK + lc;

	if (c.y < 2 && vel.y < 0.0) {
		vel.y = 0.0; // implicit floor at the anchor
	}

	grid_v[idx] = vec4(vel, m);
}

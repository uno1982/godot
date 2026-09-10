#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = BCELLS) in;

// Indirect: one workgroup per active block, one thread per cell. Normalise
// momentum, apply gravity, box boundary (M-b1 keeps the box; M-b2 drops it).
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

	const int BW = 2;
	if (c.x < BW && vel.x < 0.0) { vel.x = 0.0; }
	if (c.y < BW && vel.y < 0.0) { vel.y = 0.0; }
	if (c.z < BW && vel.z < 0.0) { vel.z = 0.0; }
	if (c.x >= RES.x - BW && vel.x > 0.0) { vel.x = 0.0; }
	if (c.y >= RES.y - BW && vel.y > 0.0) { vel.y = 0.0; }
	if (c.z >= RES.z - BW && vel.z > 0.0) { vel.z = 0.0; }

	grid_v[idx] = vec4(vel, m);
}

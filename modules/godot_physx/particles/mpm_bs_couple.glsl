#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = BCELLS) in;

// Indirect: one workgroup per active block, one thread per cell. Analytic
// colliders as moving velocity boundary conditions; reaction impulse summed
// per collider. (Mirrors mpm_couple.glsl.)
void main() {
	uint bslot = gl_WorkGroupID.x;
	if (bslot >= bcounts[0]) {
		return;
	}
	uint lin = gl_LocalInvocationID.x;
	uint idx = bslot * uint(BCELLS) + lin;

	vec4 gv = grid_v[idx];
	if (gv.w <= 1e-9) {
		return;
	}

	ivec3 bc = unpack_block(bkey[bslot]);
	ivec3 lc = ivec3(int(lin) & 3, (int(lin) >> 2) & 3, int(lin) >> 4);
	vec3 wp = ORIGIN + vec3(bc * BLK + lc) * DX;
	vec3 vel = gv.xyz;
	float m = gv.w;

	const float skin = 0.5 * DX;
	const float buried = 1.0 * DX;

	for (int ci = 0; ci < NCOL; ci++) {
		float sd = collider_sdf(ci, wp);
		if (sd >= skin) {
			continue;
		}
		vec3 cv = colliders[ci].c2.xyz;

		if (sd < -buried) {
			vec3 dvv = cv - vel;
			atomicAdd(cimp[ci * 4 + 0], int(-m * dvv.x * IMP_FIXED));
			atomicAdd(cimp[ci * 4 + 1], int(-m * dvv.y * IMP_FIXED));
			atomicAdd(cimp[ci * 4 + 2], int(-m * dvv.z * IMP_FIXED));
			vel = cv;
			continue;
		}

		vec3 nh = collider_normal(ci, wp);
		vec3 rel = vel - cv;
		float vn = dot(rel, nh);
		bool penetrating = sd < 0.0;
		if (vn >= 0.0 && !penetrating) {
			continue;
		}

		vec3 rel_t = rel - min(vn, 0.0) * nh;
		vec3 nvel = cv + rel_t * clamp(1.0 - CFRIC, 0.0, 1.0);
		vec3 dv = nvel - vel;

		vec3 imp = -m * dv;
		atomicAdd(cimp[ci * 4 + 0], int(imp.x * IMP_FIXED));
		atomicAdd(cimp[ci * 4 + 1], int(imp.y * IMP_FIXED));
		atomicAdd(cimp[ci * 4 + 2], int(imp.z * IMP_FIXED));

		vel = nvel;
	}

	grid_v[idx].xyz = vel;
}

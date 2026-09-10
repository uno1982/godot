#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = GROUP) in;

// P2G pass 2 (fluid): gather local density off the grid mass, run a Tait EOS,
// scatter APIC momentum + the MLS-MPM stress contribution.
void main() {
	uint id = gl_GlobalInvocationID.x;
	if (id >= uint(PCOUNT)) {
		return;
	}

	Particle p = particles[id];
	vec3 x = p.x_d.xyz;
	vec3 v = p.v.xyz;
	mat3 C = mat3(p.c0.xyz, p.c1.xyz, p.c2.xyz);

	ivec3 base;
	vec3 fx;
	grid_local(x, base, fx);
	vec3 wx = bspline(fx.x), wy = bspline(fx.y), wz = bspline(fx.z);

	ivec3 base_bc = base >> 2;
	int bs[8];
	resolve_blocks(base_bc, (base + 2) >> 2, bs);

	float mass_here = 0.0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				int idx = cell_slot(base + ivec3(i, j, k), base_bc, bs);
				if (idx < 0) {
					continue;
				}
				float w = wx[i] * wy[j] * wz[k];
				mass_here += w * float(grid_i[idx * 4 + 0]) / FIXED;
			}
		}
	}
	float density = mass_here / (DX * DX * DX);
	float ratio = clamp(density / RHO0, 0.6, 2.0);
	float pressure = STIFF * (pow(ratio, GAMMA) - 1.0);
	pressure = max(pressure, -0.01 * STIFF);

	mat3 stress = mat3(-pressure);
	stress += VISC * (C + transpose(C));

	float vol = PMASS / max(density, 1.0);
	mat3 affine = PMASS * C - (DT * vol * 4.0 / (DX * DX)) * stress;

	particles[id].x_d.w = density;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				ivec3 node = base + ivec3(i, j, k);
				int idx = cell_slot(node, base_bc, bs);
				if (idx < 0) {
					continue;
				}
				float w = wx[i] * wy[j] * wz[k];
				vec3 dpos = (ORIGIN + vec3(node) * DX) - x;
				vec3 contrib = w * (PMASS * v + affine * dpos);
				atomicAdd(grid_i[idx * 4 + 1], int(contrib.x * FIXED));
				atomicAdd(grid_i[idx * 4 + 2], int(contrib.y * FIXED));
				atomicAdd(grid_i[idx * 4 + 3], int(contrib.z * FIXED));
			}
		}
	}
}

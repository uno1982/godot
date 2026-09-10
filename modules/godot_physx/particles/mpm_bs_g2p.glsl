#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = GROUP) in;

void main() {
	uint id = gl_GlobalInvocationID.x;
	if (id >= uint(PCOUNT)) {
		return;
	}

	Particle p = particles[id];
	vec3 x = p.x_d.xyz;

	ivec3 base;
	vec3 fx;
	grid_local(x, base, fx);
	vec3 wx = bspline(fx.x), wy = bspline(fx.y), wz = bspline(fx.z);

	ivec3 base_bc = base >> 2;
	int bs[8];
	resolve_blocks(base_bc, (base + 2) >> 2, bs);

	vec3 new_v = vec3(0.0);
	mat3 new_C = mat3(0.0);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				ivec3 node = base + ivec3(i, j, k);
				int idx = cell_slot(node, base_bc, bs);
				if (idx < 0) {
					continue;
				}
				float w = wx[i] * wy[j] * wz[k];
				vec3 gv = grid_v[idx].xyz;
				vec3 dpos = (ORIGIN + vec3(node) * DX) - x;
				new_v += w * gv;
				new_C += (4.0 / (DX * DX)) * w * outerProduct(gv, dpos);
			}
		}
	}

	new_v *= 0.996;

	float sp = length(new_v);
	if (sp > 25.0) {
		new_v *= 25.0 / sp;
	}

	x += DT * new_v;

	for (int ci = 0; ci < NCOL; ci++) {
		float sd = collider_sdf(ci, x);
		if (sd < 0.0) {
			vec3 nh = collider_normal(ci, x);
			x += nh * (-sd + 1e-4);
			float vn = dot(new_v - colliders[ci].c2.xyz, nh);
			if (vn < 0.0) {
				new_v -= vn * nh;
			}
		}
	}

	// Boundless: implicit floor at the anchor (matches the grid BC), no walls.
	float floor_y = ORIGIN.y + DX * 1.5;
	if (x.y < floor_y) { x.y = floor_y; new_v.y *= -0.2; }

	// Sanitise a diverged particle and keep it inside the packed-key range
	// (+-500 blocks = +-2000 cells) so a runaway can't wrap a key onto occupied
	// space far away.
	vec3 far = vec3(1950.0) * DX;
	if (!all(equal(x, x)) || any(greaterThan(abs(x - ORIGIN), far))) {
		x = clamp(x, ORIGIN - far, ORIGIN + far);
		if (!all(equal(x, x))) {
			x = ORIGIN;
		}
		new_v = vec3(0.0);
	}

	particles[id].x_d = vec4(x, p.x_d.w);
	particles[id].v = vec4(new_v, 0.0);
	particles[id].c0 = vec4(new_C[0], 0.0);
	particles[id].c1 = vec4(new_C[1], 0.0);
	particles[id].c2 = vec4(new_C[2], 0.0);
}

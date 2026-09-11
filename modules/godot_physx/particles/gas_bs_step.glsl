#[compute]
#version 450

#include "gas_block_inc.glsl"

// Binding 5 = read (this frame's "current" grid), binding 6 = write (the
// other one), binding 7 = curl field already computed this frame by
// gas_bs_curl from the same binding-5 buffer.

// Boundless-within-the-box lookup used for the backtrace sample and the
// 6-neighbour diffusion taps -- out-of-box or unallocated cells read as zero
// (open boundary, matches a plume that hasn't reached the box edge yet).
vec4 sample_field(ivec3 c) {
	if (any(lessThan(c, ivec3(0))) || any(greaterThanEqual(c, BOXC))) {
		return vec4(0.0);
	}
	int slot = block_lookup(c >> 2);
	if (slot < 0) {
		return vec4(0.0);
	}
	return data5[slot * BCELLS + lin3(c & 3)];
}

// Trilinear resample at a fractional cell-space position (same -0.5
// cell-centre convention as the MPM path's grid_local). Nearest-cell sampling
// left visible per-cell banding in the prototype's first pass.
vec4 sample_trilinear(vec3 gp) {
	vec3 gp0 = gp - 0.5;
	ivec3 base = ivec3(floor(gp0));
	vec3 f = gp0 - vec3(base);
	vec4 c000 = sample_field(base + ivec3(0, 0, 0));
	vec4 c100 = sample_field(base + ivec3(1, 0, 0));
	vec4 c010 = sample_field(base + ivec3(0, 1, 0));
	vec4 c110 = sample_field(base + ivec3(1, 1, 0));
	vec4 c001 = sample_field(base + ivec3(0, 0, 1));
	vec4 c101 = sample_field(base + ivec3(1, 0, 1));
	vec4 c011 = sample_field(base + ivec3(0, 1, 1));
	vec4 c111 = sample_field(base + ivec3(1, 1, 1));
	vec4 x00 = mix(c000, c100, f.x), x10 = mix(c010, c110, f.x);
	vec4 x01 = mix(c001, c101, f.x), x11 = mix(c011, c111, f.x);
	vec4 y0 = mix(x00, x10, f.y), y1 = mix(x01, x11, f.y);
	return mix(y0, y1, f.z);
}

float curl_mag_at(ivec3 c) {
	if (any(lessThan(c, ivec3(0))) || any(greaterThanEqual(c, BOXC))) {
		return 0.0;
	}
	int slot = block_lookup(c >> 2);
	if (slot < 0) {
		return 0.0;
	}
	return curl_data[slot * BCELLS + lin3(c & 3)].w;
}

layout(local_size_x = GROUP) in;

void main() {
	uint i = gl_GlobalInvocationID.x;
	if (int(i) >= BOXC.x * BOXC.y * BOXC.z) {
		return;
	}
	int cx = BOXC.x, cy = BOXC.y;
	ivec3 c = ivec3(int(i) % cx, (int(i) / cx) % cy, int(i) / (cx * cy));
	int slot = block_lookup(c >> 2);
	if (slot < 0) {
		return;
	}
	int idx = slot * BCELLS + lin3(c & 3);

	vec3 wpos = ORIGIN + (vec3(c) + 0.5) * CDX;
	vec4 self_v = data5[idx];

	// Pure pull-based (semi-Lagrangian) advection alone can't spread momentum
	// to a cell whose own velocity is still zero -- it just samples itself and
	// stays zero forever, so a plume can never climb past the cells the
	// source directly touches. A real solver gets its spread from the
	// pressure-projection coupling every cell each step (not implemented in
	// v1 -- see gas_block_inc.glsl's header); the cheap stand-in is Stam's own
	// diffusion term: blend in the 6-face-neighbour average so momentum/
	// density leak outward one cell's worth per step even where self_v is
	// zero.
	vec3 back_gp = (wpos - self_v.xyz * DT - ORIGIN) / CDX;
	vec4 advected = sample_trilinear(back_gp);

	vec4 nsum = sample_field(c + ivec3(1, 0, 0)) + sample_field(c - ivec3(1, 0, 0)) +
			sample_field(c + ivec3(0, 1, 0)) + sample_field(c - ivec3(0, 1, 0)) +
			sample_field(c + ivec3(0, 0, 1)) + sample_field(c - ivec3(0, 0, 1));
	vec4 nbr_avg = nsum * (1.0 / 6.0);

	const float DIFFUSE = 0.12;
	vec4 blended = mix(advected, nbr_avg, DIFFUSE);

	vec3 new_vel = blended.xyz;
	float new_dens = blended.w;

	new_vel.y += BUOY * new_dens * DT;

	// Real vorticity confinement (Fedkiw et al.): eta = normalize(grad|curl|),
	// force = VORT * dx * cross(eta, curl) -- pumps energy back into small
	// eddies a coarse grid's numerical diffusion would otherwise smear away.
	vec4 self_curl = curl_data[idx];
	vec3 grad;
	grad.x = curl_mag_at(c + ivec3(1, 0, 0)) - curl_mag_at(c - ivec3(1, 0, 0));
	grad.y = curl_mag_at(c + ivec3(0, 1, 0)) - curl_mag_at(c - ivec3(0, 1, 0));
	grad.z = curl_mag_at(c + ivec3(0, 0, 1)) - curl_mag_at(c - ivec3(0, 0, 1));
	float glen = length(grad);
	if (glen > 1e-6) {
		vec3 eta = grad / glen;
		new_vel += VORT * CDX * cross(eta, self_curl.xyz) * DT;
	}

	new_dens *= DISS;
	new_vel *= 0.998;

	float d = distance(wpos, source_pos_radius.xyz);
	if (d < source_pos_radius.w) {
		new_dens = max(new_dens, source_vel_density.w);
		vec3 jitter = vec3(
				hash13(vec3(c) + vec3(TIME, 0.0, 0.0)) - 0.5,
				0.0,
				hash13(vec3(c) + vec3(0.0, 0.0, TIME)) - 0.5);
		new_vel += (source_vel_density.xyz + jitter * source_vel_density.y * 0.6) * DT;
	}

	// Solid obstacles: voxelized spheres, forced empty+still every step. A
	// cell just outside one still pulls its backtrace/diffusion samples FROM
	// inside (always zero), so the field naturally thins and parts near the
	// surface without any extra boundary-condition code -- the same
	// "missing/zero neighbour" mechanism that makes the box edges an open
	// boundary.
	for (int ci = 0; ci < MAX_GAS_COLLIDERS; ci++) {
		if (colliders[ci].w > 0.0 && distance(wpos, colliders[ci].xyz) < colliders[ci].w) {
			new_vel = vec3(0.0);
			new_dens = 0.0;
			break;
		}
	}

	data6[idx] = vec4(new_vel, new_dens);
}

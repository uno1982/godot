// Shared definitions for the sparse-grid gas/smoke solver (gas_bs_*.glsl).
//
// Reuses the exact block-hash technique the MPM block-sparse fluid grid uses
// (see mpm_block_inc.glsl) -- hash the 4^3 block a cell belongs to, not the
// cell itself, so a stencil/neighbourhood touches a handful of hash lookups
// instead of one per cell. The two solvers differ in what lives in a cell:
// MPM's grid is a disposable per-substep P2G/G2P scratch buffer rebuilt from
// particles every step; this grid is a PERSISTENT Eulerian velocity+density
// field that only advects/diffuses in place. Validated in a standalone
// RenderingDevice prototype before porting here -- see the backlog memory
// entry for the milestone history.
//
// Scope cuts (v1): the box's lateral (X/Z) footprint and grid_anchor (its
// bottom corner) are frozen at configure() and never move or reflow -- only
// its height grows automatically as a plume rises past it (GasSolver::
// _grow_if_needed periodically block_touch()'s more blocks at the top, safe
// because it never touches an already-populated block); no pressure
// projection (not divergence-free -- spread comes from diffusion, matching
// Stam's stable-fluids viscosity term, not incompressible flow); up to
// MAX_GAS_COLLIDERS sphere colliders (voxelized solid: forced empty+still
// every step, not a real no-slip/free-slip boundary condition).

#define HASH_EMPTY 0xffffffffu
#define PROBE_MAX 48u
#define BLK 4
#define BCELLS 64
#define GROUP 64
#define MAX_GAS_COLLIDERS 4

layout(set = 0, binding = 0, std140) uniform Params {
	vec4 dt_buoy_vort_diss; // x dt, y buoyancy, z vorticity confinement strength, w dissipation
	vec4 origin_dx_pad; // xyz grid anchor (world, frozen at configure), w cell size
	ivec4 box_blocks_cells; // xyz box size in BLOCKS, w unused
	vec4 source_pos_radius; // xyz world, w radius
	vec4 source_vel_density; // xyz injected velocity, w injected density
	vec4 hash_maxb_time_pad; // x hash slot count (pow2), y block-pool capacity, z time (seconds, for source jitter)
	vec4 colliders[MAX_GAS_COLLIDERS]; // xyz world centre, w radius (<=0 = unused slot)
	ivec4 growth_range; // x = first newly-touched pool slot, y = one past the last, zw unused -- gas_bs_clear_range only
};

layout(set = 0, binding = 1, std430) restrict buffer BHashKey { uint bhash[]; };
layout(set = 0, binding = 2, std430) restrict buffer BHashVal { uint bhash_val[]; };
layout(set = 0, binding = 3, std430) restrict buffer BKey { uint bkey[]; };
layout(set = 0, binding = 4, std430) restrict buffer BCounts { uint bcounts[]; };
// Binding 5/6 swap roles (read/write, or read-for-curl/write-curl-out) between
// passes -- see each gas_bs_*.glsl for which is which this pass. The uniform
// set alternates which physical buffer (grid A or B) occupies which slot each
// step (ping-pong), matching the prototype's us_step_atob / us_step_btoa.
layout(set = 0, binding = 5, std430) restrict buffer Grid5 { vec4 data5[]; };
layout(set = 0, binding = 6, std430) restrict buffer Grid6 { vec4 data6[]; };
layout(set = 0, binding = 7, std430) restrict buffer Curl { vec4 curl_data[]; };

#define DT (dt_buoy_vort_diss.x)
#define BUOY (dt_buoy_vort_diss.y)
#define VORT (dt_buoy_vort_diss.z)
#define DISS (dt_buoy_vort_diss.w)
#define ORIGIN (origin_dx_pad.xyz)
#define CDX (origin_dx_pad.w)
#define BOXB (box_blocks_cells.xyz)
#define BOXC (box_blocks_cells.xyz * BLK)
#define HMASK (uint(hash_maxb_time_pad.x) - 1u)
#define MAXB (uint(hash_maxb_time_pad.y))
#define TIME (hash_maxb_time_pad.z)

uint pack_block(ivec3 bc) {
	uvec3 u = uvec3(bc + 512);
	return u.x | (u.y << 10) | (u.z << 20);
}

ivec3 unpack_block(uint k) {
	return ivec3(int(k & 1023u), int((k >> 10) & 1023u), int((k >> 20) & 1023u)) - 512;
}

uint hash_u32(uint k) {
	k ^= k >> 16;
	k *= 0x7feb352du;
	k ^= k >> 15;
	k *= 0x846ca68bu;
	k ^= k >> 16;
	return k;
}

// Register a block, claiming a pool slot on first sight. Idempotent.
void block_touch(ivec3 bc) {
	if (bcounts[0] >= MAXB) {
		return;
	}
	uint key = pack_block(bc);
	uint h = hash_u32(key) & HMASK;
	for (uint t = 0u; t < PROBE_MAX; t++) {
		uint prev = atomicCompSwap(bhash[h], HASH_EMPTY, key);
		if (prev == HASH_EMPTY) {
			uint slot = atomicAdd(bcounts[0], 1u);
			if (slot >= MAXB) {
				slot = MAXB - 1u;
			}
			bkey[slot] = key;
			bhash_val[h] = slot;
			return;
		}
		if (prev == key) {
			return;
		}
		h = (h + 1u) & HMASK;
	}
}

// Look up a block's pool slot, or -1.
int block_lookup(ivec3 bc) {
	uint key = pack_block(bc);
	uint h = hash_u32(key) & HMASK;
	for (uint t = 0u; t < PROBE_MAX; t++) {
		uint k = bhash[h];
		if (k == key) {
			return int(bhash_val[h]);
		}
		if (k == HASH_EMPTY) {
			return -1;
		}
		h = (h + 1u) & HMASK;
	}
	return -1;
}

int lin3(ivec3 c) {
	return (c.z * 4 + c.y) * 4 + c.x;
}

// Cheap per-cell, per-frame pseudo-random jitter -- seeds a little asymmetry
// into the source injection. A perfectly symmetric source has no curl for
// real vorticity confinement to amplify, so a laminar stable-fluids sim needs
// *something* non-symmetric to kick off instabilities, the way real smoke
// seeds off nozzle turbulence.
float hash13(vec3 p) {
	p = fract(p * 0.3183099 + vec3(0.1, 0.2, 0.3));
	p *= 17.0;
	return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

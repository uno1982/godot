// Shared definitions for the BLOCK-SPARSE MLS-MPM fluid path (mpm_bs_*.glsl).
//
// The grid is divided into BLK^3 blocks. Only blocks the fluid touches are
// allocated: a particle registers the <=8 blocks its 27-cell transfer stencil
// spans (hash the block coord, open addressing), and the node passes iterate the
// compacted active-block list via indirect dispatch. Within a block, cell
// addressing is contiguous arithmetic -- coalesced, cache-friendly -- so the
// per-particle P2G/G2P cost stays close to the dense path while the grid follows
// the fluid instead of a fixed box.
//
// This path is fluid only. Granular keeps the dense mpm_*.glsl passes untouched.

#define FIXED 8388608.0
#define IMP_FIXED 1024.0
#define SURF_FIXED 256.0
#define GROUP 64
#define GAMMA 7.0
#define MAX_COLLIDERS 32
#define HASH_EMPTY 0xffffffffu
#define PROBE_MAX 48u

#define BLK 4     // block edge in cells
#define BCELLS 64 // BLK^3

struct Particle {
	vec4 x_d;
	vec4 v;
	vec4 c0;
	vec4 c1;
	vec4 c2;
	vec4 f0;
	vec4 f1;
	vec4 f2;
};

struct Collider {
	vec4 c0;
	vec4 c1;
	vec4 c2;
	vec4 c3;
};

layout(set = 0, binding = 0, std140) uniform Params {
	vec4 gravity_dt; // xyz gravity, w dt
	vec4 origin_dx; // xyz grid anchor (world, frozen at configure), w cell size
	ivec4 res_count; // xyz box cell count (M-b1 box BC), w particle count
	vec4 fluid; // x rest_density, y stiffness, z viscosity, w particle_mass
	vec4 bmin; // xyz domain min (world), w = surface mass boost
	vec4 bmax; // xyz domain max (world), w unused (fluid only)
	vec4 extra; // x collider count, y collider friction, z surface iso density, w surface kernel
	vec4 gran; // unused on this path
	vec4 blockp; // x = hash slot count (pow2), y = block-pool capacity (MAX_BLOCKS)
};

layout(set = 0, binding = 1, std430) restrict buffer Particles { Particle particles[]; };
layout(set = 0, binding = 2, std430) restrict buffer GridAcc { int grid_i[]; }; // 4 ints / cell: mass, mom.xyz -- indexed (blockslot*64 + celllin)
layout(set = 0, binding = 3, std430) restrict buffer GridVel { vec4 grid_v[]; }; // xyz vel, w mass -- same index
layout(set = 0, binding = 4, std430) restrict buffer Colliders { Collider colliders[]; };
layout(set = 0, binding = 5, std430) restrict buffer ColliderImp { int cimp[]; };
layout(set = 0, binding = 6, std430) restrict buffer MMData { float mm[]; };
layout(set = 0, binding = 7, std430) restrict buffer SurfaceField { int surf_i[]; };
layout(set = 0, binding = 8, std430) restrict buffer BHashKey { uint bhash[]; }; // block key, HASH_EMPTY = free
layout(set = 0, binding = 9, std430) restrict buffer BHashVal { uint bhash_val[]; }; // -> block-pool slot
layout(set = 0, binding = 10, std430) restrict buffer BKey { uint bkey[]; }; // block key per pool slot
layout(set = 0, binding = 11, std430) restrict buffer BCounts { uint bcounts[]; }; // [0] active blocks, [1..3] indirect dispatch

#define DT (gravity_dt.w)
#define GRAV (gravity_dt.xyz)
#define ORIGIN (origin_dx.xyz)
#define DX (origin_dx.w)
#define RES (res_count.xyz)
#define PCOUNT (res_count.w)
#define RHO0 (fluid.x)
#define STIFF (fluid.y)
#define VISC (fluid.z)
#define PMASS (fluid.w)
#define NCOL (int(extra.x))
#define CFRIC (extra.y)
#define HASHN (uint(blockp.x))
#define HMASK (uint(blockp.x) - 1u)
#define MAXB (uint(blockp.y))

// --- quadratic B-spline transfer ---

vec3 bspline(float fx) {
	vec3 w;
	w.x = 0.5 * (1.5 - fx) * (1.5 - fx);
	w.y = 0.75 - (fx - 1.0) * (fx - 1.0);
	w.z = 0.5 * (fx - 0.5) * (fx - 0.5);
	return w;
}

void grid_local(vec3 x, out ivec3 base, out vec3 fx) {
	vec3 gp = (x - ORIGIN) / DX;
	base = ivec3(floor(gp - 0.5));
	fx = gp - vec3(base);
}

// --- block hash ---

// Block coords biased by +512 -> unsigned key, 10 bits/axis (+-512 blocks =
// +-2048 cells, boundless within fp range).
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
		return; // pool full -- stop inserting (over-spread degrades locally, not a probe-walk cliff)
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
			uint v = bhash_val[h];
			for (int g = 0; v == HASH_EMPTY && g < 1024; g++) {
				v = bhash_val[h];
			}
			return int(v);
		}
		if (k == HASH_EMPTY) {
			return -1;
		}
		h = (h + 1u) & HMASK;
	}
	return -1;
}

// Resolve the 8 corner blocks of a particle's stencil (base .. base+2) into pool
// slots. bs[dz*4 + dy*2 + dx]. Redundant when the stencil fits in one block.
void resolve_blocks(ivec3 base_bc, out int bs[8]) {
	for (int dz = 0; dz < 2; dz++) {
		for (int dy = 0; dy < 2; dy++) {
			for (int dx = 0; dx < 2; dx++) {
				bs[dz * 4 + dy * 2 + dx] = block_lookup(base_bc + ivec3(dx, dy, dz));
			}
		}
	}
}

// Grid buffer index for cell c, given the particle's base block and resolved
// slots. Returns -1 if the owning block is missing.
int cell_slot(ivec3 c, ivec3 base_bc, int bs[8]) {
	ivec3 cbc = c >> 2; // floor(c/4)
	ivec3 d = cbc - base_bc; // 0 or 1 per axis
	int slot = bs[d.z * 4 + d.y * 2 + d.x];
	if (slot < 0) {
		return -1;
	}
	ivec3 lc = c & 3; // c - cbc*4, in [0,3]
	int lin = (lc.z * 4 + lc.y) * 4 + lc.x;
	return slot * BCELLS + lin;
}

// --- analytic collider SDF (same as the dense path) ---

vec3 _qrot(vec4 q, vec3 v) {
	return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float collider_sdf(int i, vec3 wp) {
	int type = int(colliders[i].c0.w);
	vec3 ctr = colliders[i].c0.xyz;
	if (type == 2) {
		return dot(wp - ctr, normalize(colliders[i].c1.xyz));
	}
	vec4 q = colliders[i].c3;
	vec3 p = _qrot(vec4(-q.xyz, q.w), wp - ctr);
	if (type == 1) {
		vec3 d = abs(p) - colliders[i].c1.xyz;
		return length(max(d, 0.0)) + min(max(d.x, max(d.y, d.z)), 0.0);
	}
	if (type == 3) {
		p.y -= clamp(p.y, -colliders[i].c1.y, colliders[i].c1.y);
		return length(p) - colliders[i].c1.x;
	}
	return length(p) - colliders[i].c1.x;
}

vec3 collider_normal(int i, vec3 wp) {
	float e = DX * 0.25;
	vec2 k = vec2(1.0, -1.0);
	vec3 n = k.xyy * collider_sdf(i, wp + k.xyy * e) +
			k.yyx * collider_sdf(i, wp + k.yyx * e) +
			k.yxy * collider_sdf(i, wp + k.yxy * e) +
			k.xxx * collider_sdf(i, wp + k.xxx * e);
	float len = length(n);
	return len > 1e-6 ? n / len : vec3(0.0, 1.0, 0.0);
}

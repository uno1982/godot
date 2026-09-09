// Shared MLS-MPM definitions -- #include'd by the pass shaders.
//
// Weakly-compressible fluid on plain compute (RenderingDevice, no CUDA): this is
// the cross-vendor backend for PhysXParticleFluid3D when the PhysX GPU (PBD/CUDA)
// path is unavailable. Particles carry position, velocity and an affine velocity
// matrix C (APIC / MLS-MPM). Pressure comes from the grid mass field: P2G runs
// in two passes -- mass first, then momentum, sampling the local density back
// off the grid for a Tait equation of state (no per-particle volume drift). The
// grid is an integer fixed-point accumulator so it needs only plain atomicAdd on
// int (no float-atomic extension).

#define FIXED 8388608.0   // 2^23 -- exact in fp32, plenty of per-cell headroom
#define IMP_FIXED 1024.0  // coarser fixed-point for the collider impulse sum (small values, many adds)
#define SURF_FIXED 256.0 // fixed-point for the isosurface density scatter (boosted density can run high)
#define GROUP 64
#define GAMMA 7.0         // Tait exponent -- stiff under compression, soft under expansion
#define MAX_COLLIDERS 32 // keep in sync with mpm_fluid_solver.cpp (loops use the dynamic NCOL, not this)

struct Particle {
	vec4 x_d; // xyz = position (world), w = last sampled density (debug/readback)
	vec4 v;   // xyz = velocity, w unused
	vec4 c0;  // affine velocity matrix C, column 0 (w unused)
	vec4 c1;
	vec4 c2;
};

// Analytic collider: a moving velocity boundary condition for the grid.
//   c0: xyz = centre (world), w = shape (0 sphere, 1 box, 2 plane, 3 capsule)
//   c1: sphere x=radius | box xyz=half-extents | plane xyz=unit normal | capsule x=radius y=half-height
//   c2: xyz = linear velocity
//   c3: orientation quaternion (box / capsule; sphere and plane ignore it)
struct Collider {
	vec4 c0;
	vec4 c1;
	vec4 c2;
	vec4 c3;
};

layout(set = 0, binding = 0, std140) uniform Params {
	vec4 gravity_dt;  // xyz gravity, w dt
	vec4 origin_dx;   // xyz grid origin (world), w cell size
	ivec4 res_count;  // xyz grid resolution (nodes), w particle count
	vec4 fluid;       // x rest_density, y stiffness, z dynamic_viscosity, w particle_mass
	vec4 bmin;        // xyz domain min (world)
	vec4 bmax;        // xyz domain max (world)
	vec4 extra;       // x collider count, y collider friction, zw unused
};

layout(set = 0, binding = 1, std430) restrict buffer Particles { Particle particles[]; };
layout(set = 0, binding = 2, std430) restrict buffer GridAcc { int grid_i[]; };  // 4 ints / node: mass, mom.xyz
layout(set = 0, binding = 3, std430) restrict buffer GridVel { vec4 grid_v[]; }; // xyz velocity, w mass
layout(set = 0, binding = 4, std430) restrict buffer Colliders { Collider colliders[]; };
layout(set = 0, binding = 5, std430) restrict buffer ColliderImp { int cimp[]; }; // 4 ints / collider: reaction impulse xyz
layout(set = 0, binding = 6, std430) restrict buffer MMData { float mm[]; };       // 12 floats / instance: MultiMesh transform rows
layout(set = 0, binding = 7, std430) restrict buffer SurfaceField { int surf_i[]; }; // 1 int / node: SPH density scatter (SURF_FIXED), for isosurfacing

#define DT       (gravity_dt.w)
#define GRAV     (gravity_dt.xyz)
#define ORIGIN   (origin_dx.xyz)
#define DX       (origin_dx.w)
#define RES      (res_count.xyz)
#define PCOUNT   (res_count.w)
#define RHO0     (fluid.x)
#define STIFF    (fluid.y)
#define VISC     (fluid.z)
#define PMASS    (fluid.w)
#define NCOL     (int(extra.x))
#define CFRIC    (extra.y)

int node_index(ivec3 c) {
	return (c.z * RES.y + c.y) * RES.x + c.x;
}

ivec3 node_coord(int n) {
	int x = n % RES.x;
	int y = (n / RES.x) % RES.y;
	int z = n / (RES.x * RES.y);
	return ivec3(x, y, z);
}

// Quadratic B-spline weights for the 3 nodes touched along one axis. `fx` is the
// particle's fractional position relative to the base node, in [0.5, 1.5).
vec3 bspline(float fx) {
	vec3 w;
	w.x = 0.5 * (1.5 - fx) * (1.5 - fx);
	w.y = 0.75 - (fx - 1.0) * (fx - 1.0);
	w.z = 0.5 * (fx - 0.5) * (fx - 0.5);
	return w;
}

void grid_local(vec3 x, out ivec3 base, out vec3 fx) {
	vec3 gp = (x - ORIGIN) / DX;
	base = ivec3(gp - 0.5);
	fx = gp - vec3(base);
}

// --- analytic collider SDF (shared by the coupling pass and G2P clamp) ---

vec3 _qrot(vec4 q, vec3 v) {
	return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float collider_sdf(int i, vec3 wp) {
	int type = int(colliders[i].c0.w);
	vec3 ctr = colliders[i].c0.xyz;
	if (type == 2) { // plane
		return dot(wp - ctr, normalize(colliders[i].c1.xyz));
	}
	vec4 q = colliders[i].c3;
	vec3 p = _qrot(vec4(-q.xyz, q.w), wp - ctr);
	if (type == 1) { // box
		vec3 d = abs(p) - colliders[i].c1.xyz;
		return length(max(d, 0.0)) + min(max(d.x, max(d.y, d.z)), 0.0);
	}
	if (type == 3) { // capsule along local Y
		p.y -= clamp(p.y, -colliders[i].c1.y, colliders[i].c1.y);
		return length(p) - colliders[i].c1.x;
	}
	return length(p) - colliders[i].c1.x; // sphere
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

#[compute]
#version 450

// One-time (per configure()) generation of the base Tessendorf/Phillips
// ocean spectrum h0, packed per texel as (xy = h0(k), zw = conj(h0(-k))) --
// both evaluated directly in this single pass via two independently-hashed
// Gaussian pairs (one seeded by (mx,mz), one by (-mx,-mz)), so there's no
// need to read a neighboring texel's already-written data and no ordering
// hazard. Standard technique, same shape as public Tessendorf-ocean
// reference implementations (and caustic-volume's own, though that one
// precomputes h0 differently -- this pass ports the FORMULA, not their
// upload path, since we generate it on GPU instead -- see water_solver.h's
// note on why). Written flat -- runs once at configure(), not every frame,
// but kept simple anyway: no nested helper calls beyond one level, no
// dynamic branching beyond simple guards -- see water_fft.glsl's header for
// why that discipline matters in this module right now.

layout(local_size_x = 8, local_size_y = 8) in;

layout(set = 0, binding = 0, std140) uniform Params {
	vec4 domain_wind; // xy domain size (world, full extent), z wind_speed, w wind_dir_x
	vec4 misc; // x wind_dir_z, y gravity, z amplitude, w unused
	ivec4 res; // x N, yzw unused
};

layout(set = 0, binding = 1, std430) restrict buffer H0 { vec4 h0[]; };

uint hash_u32(uint k) {
	k ^= k >> 16;
	k *= 0x7feb352du;
	k ^= k >> 15;
	k *= 0x846ca68bu;
	k ^= k >> 16;
	return k;
}

float rand01(uint seed) {
	return float(hash_u32(seed) & 0x00ffffffu) / 16777216.0;
}

vec2 gaussian_pair(ivec2 m) {
	uint seed = (uint(m.x + 32768) & 0xffffu) | ((uint(m.y + 32768) & 0xffffu) << 16);
	float u1 = max(rand01(seed * 2u + 1u), 1e-6);
	float u2 = rand01(seed * 2u + 2u);
	float r = sqrt(-2.0 * log(u1));
	return vec2(r * cos(6.28318530718 * u2), r * sin(6.28318530718 * u2));
}

float phillips(vec2 k, vec2 wind_dir, float wind_speed, float gravity, float amplitude) {
	float kl = length(k);
	if (kl < 1e-6) {
		return 0.0;
	}
	float L = wind_speed * wind_speed / gravity;
	vec2 k_hat = k / kl;
	float align = dot(k_hat, wind_dir);
	align = align >= 0.0 ? align : align * 0.2;
	float small_supp = exp(-kl * kl * L * L * 0.0001);
	float kl2 = kl * kl;
	return amplitude * exp(-1.0 / (kl2 * L * L)) / (kl2 * kl2) * align * align * small_supp;
}

void main() {
	ivec2 c = ivec2(gl_GlobalInvocationID.xy);
	int n = res.x;
	if (c.x >= n || c.y >= n) {
		return;
	}
	int mx = c.x < n / 2 ? c.x : c.x - n;
	int mz = c.y < n / 2 ? c.y : c.y - n;

	if ((mx == -n / 2) || (mz == -n / 2) || (mx == 0 && mz == 0)) {
		h0[c.y * n + c.x] = vec4(0.0);
		return;
	}

	vec2 domain = domain_wind.xy;
	vec2 wind_dir = normalize(vec2(domain_wind.w, misc.x));
	float wind_speed = domain_wind.z;
	float gravity = misc.y;
	float amplitude = misc.z;

	vec2 k = 6.28318530718 * vec2(float(mx), float(mz)) / domain;
	vec2 g1 = gaussian_pair(ivec2(mx, mz));
	vec2 g2 = gaussian_pair(ivec2(-mx, -mz));
	float p1 = phillips(k, wind_dir, wind_speed, gravity, amplitude);
	float p2 = phillips(-k, wind_dir, wind_speed, gravity, amplitude);
	// water_fft.glsl's Stockham pass is an unnormalized DFT summation (no
	// per-pass 1/2 factor anywhere) -- ported directly from the reference,
	// which apparently compensates for this in its own amplitude tuning
	// rather than an explicit normalization step (nothing in dispFinal
	// divides by N either). Empirically calibrated here instead of derived:
	// unscaled output measured ~1/N too large relative to `amplitude`'s
	// intended meaning (a probe test compared unscaled variance against a
	// target ballpark for amplitude=2.0 at wind_speed=10.0) -- 1/N, not the
	// naive 1/N^2 guess a textbook 2D IDFT normalization would suggest.
	float norm = 1.0 / float(n);
	vec2 a = g1 * sqrt(p1 * 0.5) * norm;
	vec2 b_pre = g2 * sqrt(p2 * 0.5) * norm;
	vec2 b = vec2(b_pre.x, -b_pre.y); // conj(h0(-k))

	h0[c.y * n + c.x] = vec4(a, b);
}

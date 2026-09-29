#[compute]
#version 450

// Per-step time evolution of the base spectrum h0 into the current complex
// height field, via the standard dispersion relation w = sqrt(g*k*tanh(k*depth))
// and h(k,t) = h0(k)*e^{iwt} + conj(h0(-k))*e^{-iwt}. Direct port of
// caustic-volume's `spectrum` shader (sandbox/src/10_sim.js) -- proven
// working reference math, not derived from scratch. Output feeds
// water_fft.glsl's first butterfly pass. Written flat, same discipline as
// every other pass in this file (see water_fft.glsl's header).

layout(local_size_x = 8, local_size_y = 8) in;

layout(set = 0, binding = 0, std140) uniform Params {
	vec4 domain_time; // xy domain size, z depth, w time
	ivec4 res; // x N, yzw unused
};

layout(set = 0, binding = 1, std430) restrict buffer H0 { vec4 h0[]; };
layout(set = 0, binding = 2, std430) restrict buffer SpecOut { vec2 spec_out[]; };

void main() {
	ivec2 c = ivec2(gl_GlobalInvocationID.xy);
	int n = res.x;
	if (c.x >= n || c.y >= n) {
		return;
	}
	int mx = c.x < n / 2 ? c.x : c.x - n;
	int mz = c.y < n / 2 ? c.y : c.y - n;
	vec2 domain = domain_time.xy;
	float depth = domain_time.z;
	float time = domain_time.w;

	vec2 k = 6.28318530718 * vec2(float(mx), float(mz)) / domain;
	float kl = max(length(k), 1e-6);
	float w = sqrt(9.81 * kl * tanh(min(kl * depth, 20.0)));
	float cw = cos(w * time), sw = sin(w * time);

	vec4 packed = h0[c.y * n + c.x];
	vec2 a = packed.xy, b = packed.zw;
	vec2 h = vec2(a.x * cw - a.y * sw, a.x * sw + a.y * cw) + vec2(b.x * cw + b.y * sw, b.y * cw - b.x * sw);
	spec_out[c.y * n + c.x] = h;
}

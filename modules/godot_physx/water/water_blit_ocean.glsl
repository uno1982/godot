#[compute]
#version 450

// Same as water_blit_ripple.glsl but for the FFT ocean layer: writes the
// height (real part of buf_fft_b) and the choppy horizontal displacement
// (x from buf_fft_b's imaginary part, z from buf_fft_d's real part -- see
// water_spectrum_evolve.glsl) into their textures.

layout(local_size_x = 8, local_size_y = 8) in;

layout(set = 0, binding = 0, std140) uniform Params {
	ivec4 res; // x = n, yzw unused
};
layout(set = 0, binding = 1, std430) restrict readonly buffer HeightIn {
	vec2 height_in[]; // x = height, y = x displacement (see water_spectrum_evolve.glsl)
};
layout(set = 0, binding = 2, r32f) uniform writeonly image2D height_out_tex;
layout(set = 0, binding = 3, std430) restrict readonly buffer DzIn {
	vec2 dz_in[]; // x = z displacement
};
layout(set = 0, binding = 4, rg32f) uniform writeonly image2D disp_out_tex;

void main() {
	ivec2 c = ivec2(gl_GlobalInvocationID.xy);
	int n = res.x;
	if (c.x >= n || c.y >= n) {
		return;
	}
	vec2 hx = height_in[c.y * n + c.x];
	imageStore(height_out_tex, c, vec4(hx.x, 0.0, 0.0, 0.0));
	imageStore(disp_out_tex, c, vec4(hx.y, dz_in[c.y * n + c.x].x, 0.0, 0.0));
}

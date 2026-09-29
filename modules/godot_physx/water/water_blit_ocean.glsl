#[compute]
#version 450

// Same as water_blit_ripple.glsl but for the FFT ocean layer's final
// complex height field (buf_fft_b, see water_solver.cpp) -- takes only the
// real component (the imaginary part is a correctness residual, not part of
// the rendered surface, see water_solver.h).

layout(local_size_x = 8, local_size_y = 8) in;

layout(set = 0, binding = 0, std140) uniform Params {
	ivec4 res; // x = n, yzw unused
};
layout(set = 0, binding = 1, std430) restrict readonly buffer HeightIn {
	vec2 height_in[];
};
layout(set = 0, binding = 2, r32f) uniform writeonly image2D height_out_tex;

void main() {
	ivec2 c = ivec2(gl_GlobalInvocationID.xy);
	int n = res.x;
	if (c.x >= n || c.y >= n) {
		return;
	}
	imageStore(height_out_tex, c, vec4(height_in[c.y * n + c.x].x, 0.0, 0.0, 0.0));
}

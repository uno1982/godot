#[compute]
#version 450

#include "mpm_block_inc.glsl"

layout(local_size_x = GROUP) in;

// Pack particle positions into the MultiMesh instance buffer (12 floats /
// instance: identity basis + origin).
void main() {
	uint id = gl_GlobalInvocationID.x;
	if (id >= uint(PCOUNT)) {
		return;
	}
	vec3 x = particles[id].x_d.xyz;
	uint o = id * 12u;
	mm[o + 0] = 1.0; mm[o + 1] = 0.0; mm[o + 2] = 0.0; mm[o + 3] = x.x;
	mm[o + 4] = 0.0; mm[o + 5] = 1.0; mm[o + 6] = 0.0; mm[o + 7] = x.y;
	mm[o + 8] = 0.0; mm[o + 9] = 0.0; mm[o + 10] = 1.0; mm[o + 11] = x.z;
}

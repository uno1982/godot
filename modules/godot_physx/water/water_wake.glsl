#[compute]
#version 450

// Moving wake grid (PhysXWaterWake3D): a small ripple simulation that
// travels with a boat. The grid is fixed to the water, not the boat: when
// the boat has moved whole cells, each step reads its state from `shift`
// cells over (the window slides; the waves stay where they were made), with
// still water entering at the leading edge. Same damped wave equation as the
// shore ripple grid (water_ripple.glsl), at one wave speed, with an
// absorbing band around the border so waves fade out instead of reflecting
// back at the boat. Sources pull the surface down toward a hull-shaped dip.

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(set = 0, binding = 0, std140) uniform Params {
	ivec4 grid; // x cells per side, y source count, zw shift (cells, this step)
	vec4 wave; // x wave coefficient c^2 dt^2 / dx^2, y damping * dt, z border damping * dt, w border width (cells)
	vec4 cell; // x cell size (m), y source pull rate (0..1 per step), zw origin (world xz of cell 0's corner)
};

layout(set = 0, binding = 1, std430) restrict readonly buffer StateIn {
	vec2 state_in[]; // h, h_prev
};
layout(set = 0, binding = 2, std430) restrict writeonly buffer StateOut {
	vec2 state_out[];
};
layout(set = 0, binding = 3, std430) restrict readonly buffer Sources {
	vec4 sources[]; // world x, world z, radius, depth (m)
};
layout(set = 0, binding = 4, r32f) uniform restrict writeonly image2D height_tex;

int n() {
	return grid.x;
}

vec2 read_state(ivec2 c) {
	// Shifted read: this step's cell c was cell c + shift last step.
	ivec2 s = c + grid.zw;
	if (s.x < 0 || s.y < 0 || s.x >= n() || s.y >= n()) {
		return vec2(0.0); // still water entering the window
	}
	return state_in[s.y * n() + s.x];
}

void main() {
	ivec2 c = ivec2(gl_GlobalInvocationID.xy);
	if (c.x >= n() || c.y >= n()) {
		return;
	}
	vec2 st = read_state(c);
	float h = st.x;
	float hp = st.y;
	// Neighbours; the grid edge reads still water (the absorbing band has
	// already taken the waves out by then).
	float sum = read_state(c + ivec2(1, 0)).x + read_state(c - ivec2(1, 0)).x + read_state(c + ivec2(0, 1)).x + read_state(c - ivec2(0, 1)).x;
	float flux = wave.x * (sum - 4.0 * h);

	// Absorbing border: damping ramps up toward the edge.
	int edge = min(min(c.x, c.y), min(n() - 1 - c.x, n() - 1 - c.y));
	float b = clamp(1.0 - float(edge) / max(wave.w, 1.0), 0.0, 1.0);
	float a = min(wave.y + wave.z * b * b, 0.9);
	float hn = h * (2.0 - a) - hp * (1.0 - a) + flux;

	// Sources: a soft hull-shaped dip each source pulls the surface toward,
	// weighted so the pull vanishes away from them (see water_ripple.glsl
	// for why the weight matters).
	vec2 xz = cell.zw + (vec2(c) + 0.5) * cell.x;
	float target = 0.0;
	float weight = 0.0;
	for (int i = 0; i < grid.y; i++) {
		vec4 s = sources[i];
		float t = length(xz - s.xy) / max(s.z, 1e-4);
		if (t < 1.6) {
			float w = exp(-pow(t * 1.1, 6.0));
			target = min(target, -s.w * w);
			weight = max(weight, w);
		}
	}
	hn += (target - hn) * weight * cell.y;

	state_out[c.y * n() + c.x] = vec2(hn, h);
	imageStore(height_tex, c, vec4(hn, 0.0, 0.0, 0.0));
}

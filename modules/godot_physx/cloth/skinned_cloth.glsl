#[compute]
#version 450

// Character cloth on any GPU (RenderingDevice compute): position-based cloth
// on a region of a skinned mesh. Every particle has an animated target -- its
// linear-blend-skinned rest position -- and a max distance it may stray from
// it (0 = follows the animation exactly). Per frame:
//   MODE_SKIN       targets from this frame's bone matrices (previous kept for
//                   interpolating across substeps).
// Per substep:
//   MODE_INTEGRATE  pinned particles snap to their (interpolated) target, the
//                   rest take a Verlet step under gravity with damping.
//   MODE_DISTANCE   one colour batch of distance constraints (structural +
//                   bending). Within a batch no two constraints share a
//                   particle, so each thread can write its pair directly.
//   MODE_LIMITS     tethers (stay within the rest distance of the nearest
//                   pinned particle -- keeps long robes from stretching with
//                   few iterations), body capsules (interpolated across the
//                   substeps, so fast limbs don't jump through the cloth),
//                   the backstop (no further inward, along the animated
//                   normal, than a small distance -- keeps clothing outside
//                   the body), then the max-distance sphere around the
//                   animated target.
// Once per frame after the substeps:
//   MODE_RESET      (first frame / teleport) particles placed on the animated pose.
//   MODE_OUTPUT     per-particle normal from the incident triangles, and
//                   position + normal written to RGBA32F images the render
//                   material reads (no CPU readback).

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

#define MODE_SKIN 0
#define MODE_INTEGRATE 1
#define MODE_DISTANCE 2
#define MODE_LIMITS 3
#define MODE_OUTPUT 4
#define MODE_RESET 5

layout(set = 0, binding = 0, std140) uniform Params {
	ivec4 counts; // x particles, y unused, z unused, w unused
	ivec4 counts2; // x capsules, y texture width, z bones, w unused
	vec4 step; // x dt (substep), y damping (velocity kept per substep), z unused, w stiffness
	vec4 gravity; // xyz gravity (m/s^2), w thickness (m)
	vec4 limits; // x tether slack (allowed stretch, e.g. 1.03), y tether stiffness, z capsule friction, w backstop distance (m, < 0 = off)
};

// Per dispatch (push constants -- the uniform buffer can't change between
// dispatches inside one compute list).
layout(push_constant, std430) uniform Pass {
	int mode;
	int batch_start;
	int batch_count;
	float frac; // substep fraction, for interpolating the animated targets
} pass;

layout(set = 0, binding = 1, std430) restrict readonly buffer Rest {
	vec4 rest[]; // xyz bind-space position, w max distance (m)
};
layout(set = 0, binding = 2, std430) restrict readonly buffer SkinIdx {
	uvec4 skin_idx[];
};
layout(set = 0, binding = 3, std430) restrict readonly buffer SkinW {
	vec4 skin_w[];
};
layout(set = 0, binding = 4, std430) restrict readonly buffer Bones {
	mat4 bones[]; // world-space skinning matrices (world * bone global * bind)
};
layout(set = 0, binding = 5, std430) restrict buffer Target {
	vec4 target[];
};
layout(set = 0, binding = 6, std430) restrict buffer TargetPrev {
	vec4 target_prev[];
};
layout(set = 0, binding = 7, std430) restrict buffer Pos {
	vec4 pos[]; // xyz world position, w inverse mass (0 = pinned)
};
layout(set = 0, binding = 8, std430) restrict buffer Prev {
	vec4 prev[];
};
layout(set = 0, binding = 9, std430) restrict readonly buffer Constraints {
	vec4 constraints[]; // x,y particle indices (as float bits), z rest length, w stiffness 0..1
};
layout(set = 0, binding = 10, std430) restrict readonly buffer Tethers {
	vec2 tethers[]; // x anchor particle (float bits; ~0 = none), y rest distance
};
layout(set = 0, binding = 11, std430) restrict readonly buffer Capsules {
	vec4 capsules[]; // per capsule: previous frame (a.xyz, ra), (b.xyz, rb), then this frame's
};
layout(set = 0, binding = 12, std430) restrict readonly buffer AdjOffsets {
	uint adj_offsets[]; // particle -> start in adj_pairs, count = next - this
};
layout(set = 0, binding = 13, std430) restrict readonly buffer AdjPairs {
	uvec2 adj_pairs[]; // the other two corners of each incident triangle, winding order
};
layout(set = 0, binding = 14, rgba32f) uniform restrict writeonly image2D pos_image;
layout(set = 0, binding = 15, rgba32f) uniform restrict writeonly image2D nrm_image;
layout(set = 0, binding = 16, std430) restrict readonly buffer RestNormal {
	vec4 rest_normal[]; // bind-space normal
};
layout(set = 0, binding = 17, std430) restrict buffer TargetNormal {
	vec4 target_normal[]; // skinned normal, this frame
};

mat4 skin_matrix(uint i) {
	uvec4 b = skin_idx[i];
	vec4 w = skin_w[i];
	return bones[b.x] * w.x + bones[b.y] * w.y + bones[b.z] * w.z + bones[b.w] * w.w;
}

vec3 skin(uint i) {
	return (skin_matrix(i) * vec4(rest[i].xyz, 1.0)).xyz;
}

vec3 skin_normal(uint i) {
	vec3 n = mat3(skin_matrix(i)) * rest_normal[i].xyz;
	float l = length(n);
	return l > 1e-8 ? n / l : vec3(0.0, 1.0, 0.0);
}

void project_capsules(inout vec3 p, vec3 prev_p, float frac) {
	float thickness = gravity.w;
	for (int c = 0; c < counts2.x; c++) {
		vec4 a = mix(capsules[c * 4], capsules[c * 4 + 2], frac);
		vec4 b = mix(capsules[c * 4 + 1], capsules[c * 4 + 3], frac);
		vec3 ab = b.xyz - a.xyz;
		float len2 = max(dot(ab, ab), 1e-8);
		float t = clamp(dot(p - a.xyz, ab) / len2, 0.0, 1.0);
		vec3 q = a.xyz + ab * t;
		float r = mix(a.w, b.w, t) + thickness;
		vec3 d = p - q;
		float dist = length(d);
		if (dist < r) {
			vec3 n = dist > 1e-6 ? d / dist : vec3(0.0, 1.0, 0.0);
			p = q + n * r;
			// Friction: bleed off tangential motion against the capsule.
			vec3 v = p - prev_p;
			vec3 vt = v - n * dot(v, n);
			p -= vt * limits.z;
		}
	}
}

void main() {
	uint i = gl_GlobalInvocationID.x;
	int mode = pass.mode;

	if (mode == MODE_DISTANCE) {
		if (i >= uint(pass.batch_count)) {
			return;
		}
		vec4 c = constraints[uint(pass.batch_start) + i];
		uint a = floatBitsToUint(c.x);
		uint b = floatBitsToUint(c.y);
		vec4 pa = pos[a];
		vec4 pb = pos[b];
		float wsum = pa.w + pb.w;
		if (wsum <= 0.0) {
			return;
		}
		vec3 d = pb.xyz - pa.xyz;
		float len = length(d);
		if (len < 1e-7) {
			return;
		}
		vec3 corr = d * ((len - c.z) / (len * wsum)) * c.w * step.w;
		pos[a].xyz = pa.xyz + corr * pa.w;
		pos[b].xyz = pb.xyz - corr * pb.w;
		return;
	}

	if (i >= uint(counts.x)) {
		return;
	}

	if (mode == MODE_SKIN) {
		target_prev[i] = target[i];
		target[i] = vec4(skin(i), 0.0);
		target_normal[i] = vec4(skin_normal(i), 0.0);
		return;
	}

	if (mode == MODE_RESET) {
		// First frame (or a teleport): start at rest on the animated pose.
		vec4 tt = vec4(skin(i), 0.0);
		target[i] = tt;
		target_normal[i] = vec4(skin_normal(i), 0.0);
		target_prev[i] = tt;
		pos[i] = vec4(tt.xyz, rest[i].w > 0.0 ? 1.0 : 0.0);
		prev[i] = tt;
		return;
	}

	vec3 t = mix(target_prev[i].xyz, target[i].xyz, pass.frac);
	float max_dist = rest[i].w;

	if (mode == MODE_INTEGRATE) {
		vec4 p = pos[i];
		if (max_dist <= 0.0) {
			prev[i] = vec4(p.xyz, 0.0);
			pos[i] = vec4(t, 0.0);
			return;
		}
		vec3 v = (p.xyz - prev[i].xyz) * step.y;
		prev[i] = vec4(p.xyz, 0.0);
		pos[i] = vec4(p.xyz + v + gravity.xyz * step.x * step.x, p.w);
		return;
	}

	if (mode == MODE_LIMITS) {
		vec4 p = pos[i];
		if (p.w <= 0.0) {
			return;
		}
		vec3 q = p.xyz;
		vec2 te = tethers[i];
		uint anchor = floatBitsToUint(te.x);
		if (anchor != 0xffffffffu) {
			vec3 ap = pos[anchor].xyz;
			vec3 d = q - ap;
			float len = length(d);
			float allowed = te.y * limits.x;
			if (len > allowed) {
				q = mix(q, ap + d * (allowed / len), limits.y);
			}
		}
		project_capsules(q, prev[i].xyz, pass.frac);
		if (limits.w >= 0.0) {
			vec3 n = target_normal[i].xyz;
			float inward = dot(q - t, n);
			if (inward < -limits.w) {
				q -= n * (inward + limits.w);
			}
		}
		vec3 off = q - t;
		float ol = length(off);
		if (ol > max_dist) {
			q = t + off * (max_dist / ol);
		}
		pos[i].xyz = q;
		return;
	}

	if (mode == MODE_OUTPUT) {
		vec3 p = pos[i].xyz;
		vec3 n = vec3(0.0);
		for (uint k = adj_offsets[i]; k < adj_offsets[i + 1]; k++) {
			uvec2 o = adj_pairs[k];
			n += cross(pos[o.x].xyz - p, pos[o.y].xyz - p);
		}
		float nl = length(n);
		n = nl > 1e-12 ? n / nl : vec3(0.0, 1.0, 0.0);
		ivec2 px = ivec2(int(i) % counts2.y, int(i) / counts2.y);
		imageStore(pos_image, px, vec4(p, 1.0));
		imageStore(nrm_image, px, vec4(n, 0.0));
		return;
	}
}

#[compute]
#version 450

// Shore foam: the milky, foamy sheet where waves wash in over the shallows.
// Lives on the ripple grid (fixed to the ground, unlike the ocean foam that
// rides with the water). Fed where the still water is shallower than the
// band depth -- strongest at the waterline -- whenever an arriving crest
// raises the surface there, so it pulses with each wave, then fades.
//
// Driven by the ocean height *before* the shallow-water fade: the rendered
// chop dies out at the waterline, but the waves still arrive there.

layout(local_size_x = 8, local_size_y = 8) in;

layout(set = 0, binding = 0, std140) uniform Params {
	vec4 foam; // x decay factor this step, y band depth (m), z crest rise for full foam (m, injection starts at 30% of it), w water_level
	vec4 domains; // xy ripple domain size, zw ocean domain size
	ivec4 res; // x = ripple grid n, y = 1 if shore foam is enabled, zw unused
};
layout(set = 0, binding = 1) uniform sampler2D cell_depth_tex;
layout(set = 0, binding = 2) uniform sampler2D ripple_height_tex;
layout(set = 0, binding = 3) uniform sampler2D ocean_height_tex;
layout(set = 0, binding = 4, r32f) uniform restrict image2D shore_foam_tex;

void main() {
	ivec2 c = ivec2(gl_GlobalInvocationID.xy);
	int n = res.x;
	if (c.x >= n || c.y >= n) {
		return;
	}
	if (res.y == 0) {
		imageStore(shore_foam_tex, c, vec4(0.0));
		return;
	}
	float decayed = imageLoad(shore_foam_tex, c).r * foam.x;
	float depth = texelFetch(cell_depth_tex, c, 0).r;
	if (depth <= 0.0 || depth >= foam.y) {
		imageStore(shore_foam_tex, c, vec4(decayed, 0.0, 0.0, 0.0));
		return;
	}
	vec2 rel = ((vec2(c) + 0.5) / float(n) - 0.5) * domains.xy;
	vec2 uv_o = clamp(rel / domains.zw + 0.5, vec2(0.0), vec2(1.0));
	float rise = texelFetch(ripple_height_tex, c, 0).r - foam.w + texture(ocean_height_tex, uv_o).r;
	float band = 1.0 - smoothstep(0.0, foam.y, depth);
	// Only real crests feed it, so the sheet pulses with each wave instead of
	// sitting there as a constant stripe.
	float inject = band * smoothstep(0.3 * foam.z, max(foam.z, 1e-3), rise);
	imageStore(shore_foam_tex, c, vec4(max(decayed, inject), 0.0, 0.0, 0.0));
}

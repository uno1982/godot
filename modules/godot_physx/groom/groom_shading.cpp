/**************************************************************************/
/*  groom_shading.cpp                                                     */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "groom_shading.h"

const char *GROOM_SHADING_COMMON = R"(
uniform vec3 root_color : source_color;
uniform vec3 tip_color : source_color;
uniform float self_shadow = 0.6;
uniform float hair_roughness = 0.35;
// 0: Kajiya-Kay, 1: Marschner (PhysXGroom3D.ShadingModel).
uniform int shading_model = 0;
uniform float multiple_scattering = 0.6;

varying vec3 light_tangent;
varying vec4 light_shadow;
// The roughness and shading model the light pass uses (fragment() sets them: per pixel in screen space, one groom to a
// pixel there).
varying float light_roughness;
varying float light_model;

// The groom's own strands between a point and the direction d (both in the same space): see groom_self_shadow.h.
float strands_toward(vec4 sh, vec3 d) {
	return max(0.0, dot(sh, vec4(0.282095, 0.488603 * d.y, 0.488603 * d.z, 0.488603 * d.x)));
}

// The self-shadowing's linear part turned by a rotation (it turns as a vector does: x from w, y from y, z from z).
vec4 turn_strands(vec4 sh, mat3 rotation) {
	vec3 v = rotation * vec3(sh.w, sh.y, sh.z);
	return vec4(sh.x, v.y, v.z, v.x);
}

// The hair's color along the strand, darker toward the root, where the strands shade each other.
vec3 strand_albedo(float along) {
	return mix(root_color, tip_color, along) * mix(0.55, 1.0, smoothstep(0.0, 0.35, along));
}

// Kajiya-Kay: light falls across a strand, not on a surface. Two highlights, the second shifted toward the tip and
// tinted by the hair's color.
float strand_highlight(vec3 tangent, vec3 half_vector, float exponent) {
	float th = dot(tangent, half_vector);
	return pow(sqrt(max(1.0 - th * th, 0.0)), exponent);
}

// Marschner's hair model (Marschner et al. 2003), with the real-time approximations of Karis (Physically Based Hair
// Shading in Unreal, 2016). Light leaves a strand three ways: R, reflected off its surface (untinted, a highlight
// shifted toward the root); TT, through it (tinted by the hair, the glow of backlit hair); TRT, in, reflected inside
// and out again (tinted, a second highlight shifted toward the tip). Each is a longitudinal lobe M (along the strand)
// times an azimuthal one N (around it) times its attenuation A.
const float HAIR_IOR = 1.55;

float hair_fresnel(float cos_theta) {
	// Schlick, F0 of hair's index of refraction.
	float f0 = (1.0 - HAIR_IOR) * (1.0 - HAIR_IOR) / ((1.0 + HAIR_IOR) * (1.0 + HAIR_IOR));
	return f0 + (1.0 - f0) * pow(1.0 - clamp(cos_theta, 0.0, 1.0), 5.0);
}

float hair_longitudinal(float beta, float theta_h) {
	return exp(-theta_h * theta_h / (2.0 * beta * beta)) / (sqrt(2.0 * PI) * beta);
}
)";

const char *GROOM_SHADING_LIGHT = R"(
void light() {
	// The hair between this point and the light, from the baked self-shadowing.
	float through = exp(-self_shadow * strands_toward(light_shadow, LIGHT)) * ATTENUATION;

	if (light_model < 0.5) {
		float tl = dot(light_tangent, LIGHT);
		float across = sqrt(max(1.0 - tl * tl, 0.0));
		// Lit from behind, a strand still passes some light through.
		float facing = mix(0.35, 1.0, clamp(dot(NORMAL, LIGHT) * 0.5 + 0.5, 0.0, 1.0));
		DIFFUSE_LIGHT += across * facing * through * LIGHT_COLOR / PI;
		vec3 h = normalize(LIGHT + VIEW);
		// Rougher, wider highlights: 90 and 24 at the default roughness of 0.35.
		float sharpness = 11.0 / max(light_roughness * light_roughness, 0.01);
		float primary = strand_highlight(normalize(light_tangent - NORMAL * 0.08), h, sharpness);
		float secondary = strand_highlight(normalize(light_tangent + NORMAL * 0.12), h, sharpness / 3.75);
		SPECULAR_LIGHT += (primary * 0.06 + secondary * 0.12 * ALBEDO) * facing * through * LIGHT_COLOR;
	} else {
		vec3 t = light_tangent;
		float sin_i = clamp(dot(LIGHT, t), -1.0, 1.0);
		float sin_o = clamp(dot(VIEW, t), -1.0, 1.0);
		float theta_i = asin(sin_i);
		float theta_o = asin(sin_o);
		float cos_theta_d = cos(0.5 * (theta_i - theta_o));
		float theta_h = 0.5 * (theta_i + theta_o);
		// The angle between light and view around the strand.
		vec3 l_across = LIGHT - t * sin_i;
		vec3 v_across = VIEW - t * sin_o;
		float cos_phi = dot(l_across, v_across) * inversesqrt(max(dot(l_across, l_across) * dot(v_across, v_across), 1e-8));
		float cos_half_phi = sqrt(clamp(0.5 + 0.5 * cos_phi, 0.0, 1.0));

		// Lobe widths and shifts (radians), from the roughness.
		float r2 = light_roughness * light_roughness;
		const float shift = 0.035;
		float m_r = hair_longitudinal(max(r2, 0.02), theta_h + 2.0 * shift);
		float m_tt = hair_longitudinal(max(r2 * 0.5, 0.01), theta_h - shift);
		float m_trt = hair_longitudinal(max(r2 * 2.0, 0.04), theta_h - 4.0 * shift);

		// R.
		float n_r = 0.25 * cos_half_phi;
		float a_r = hair_fresnel(sqrt(clamp(0.5 + 0.5 * dot(LIGHT, VIEW), 0.0, 1.0)));
		// TT: the path through the strand, attenuated by its color (absorption) over the path's length.
		float ior_prime = 1.19 / cos_theta_d + 0.36 * cos_theta_d;
		float a = 1.0 / ior_prime;
		float h_tt = cos_half_phi * (1.0 + a * (0.6 - 0.8 * cos_phi));
		vec3 absorb_tt = pow(max(ALBEDO, vec3(1e-4)), vec3(sqrt(max(1.0 - h_tt * h_tt * a * a, 0.0)) / (2.0 * cos_theta_d)));
		float f_tt = hair_fresnel(cos_theta_d * sqrt(max(1.0 - h_tt * h_tt, 0.0)));
		vec3 a_tt = (1.0 - f_tt) * (1.0 - f_tt) * absorb_tt;
		float n_tt = exp(-3.65 * cos_phi - 3.98);
		// TRT.
		vec3 absorb_trt = pow(max(ALBEDO, vec3(1e-4)), vec3(0.8 / cos_theta_d));
		float f_trt = hair_fresnel(cos_theta_d * 0.5);
		vec3 a_trt = (1.0 - f_trt) * (1.0 - f_trt) * f_trt * absorb_trt * absorb_trt;
		float n_trt = exp(17.0 * cos_phi - 16.78);

		vec3 strand = m_r * n_r * a_r + m_tt * n_tt * a_tt + m_trt * n_trt * a_trt;
		// The hair rendering equation's cos(theta_i), over cos^2(theta_d) of the far-field model.
		float cos_i = sqrt(max(1.0 - sin_i * sin_i, 0.0));
		SPECULAR_LIGHT += strand * cos_i / max(cos_theta_d * cos_theta_d, 0.05) * through * LIGHT_COLOR;

		// Light scattered between strands, beyond what the three paths above carry: soft, colored, from every side.
		DIFFUSE_LIGHT += cos_i * mix(0.3, 1.0, clamp(dot(NORMAL, LIGHT) * 0.5 + 0.5, 0.0, 1.0)) * multiple_scattering * through * LIGHT_COLOR / PI;
	}
}
)";

/**************************************************************************/
/*  pxtgi_emission_meter.cpp                                              */
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

#include "pxtgi_emission_meter.h"

#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/world_environment.h"
#include "scene/main/scene_tree.h"
#include "scene/main/viewport.h"
#include "scene/main/window.h"
#include "scene/resources/3d/primitive_meshes.h"
#include "scene/resources/camera_attributes.h"
#include "scene/resources/environment.h"
#include "scene/resources/material.h"
#include "scene/resources/sky.h"

#include <cfloat>

namespace {

// Frames from a batch's start to its readings: the first once the shaders have compiled and drawn, then READS of
// them READ_STEP frames apart, so a glow animated by TIME is seen at several points of its cycle (two readings can
// land on the same brightness of a pulse).
constexpr int FIRST_READ = 4;
constexpr int READ_STEP = 6;
constexpr int READS = 6;
// Then the ambient light is turned on: its first reading once the drawing has caught up, then LIT_READS of them.
constexpr int LIT_FIRST = 3;
constexpr int LIT_READS = 2;
// Then it's turned off and a white sky is reflected: its first reading once the sky's reflections are made, then
// LIT_READS of them.
constexpr int SHINE_FIRST = 8;

const StringName &meta_emission() {
	static const StringName name = "pxtgi_emission";
	return name;
}

} // namespace

Error PXTGIEmissionMeter::measure(const TypedArray<Material> &p_materials) {
	ERR_FAIL_COND_V_MSG(measuring, ERR_BUSY, "PXTGIEmissionMeter: already measuring.");
	SceneTree *tree = SceneTree::get_singleton();
	ERR_FAIL_NULL_V_MSG(tree, ERR_UNCONFIGURED, "PXTGIEmissionMeter: needs a running SceneTree.");
	pending.clear();
	for (int i = 0; i < p_materials.size(); i++) {
		Ref<Material> material = p_materials[i];
		if (material.is_valid()) {
			pending.push_back(material);
		}
	}
	results.clear();
	measuring = true;
	tree->connect(SNAME("process_frame"), callable_mp(this, &PXTGIEmissionMeter::_frame));
	_start_batch();
	return OK;
}

// The next up to MAX_COLUMNS^2 - 2 materials, each on a quad that fills its tile of an orthographic view, and two
// reference tiles last: white (albedo 1, not metallic: the albedos are read as shares of the light it takes in) and
// black (the same but albedo 0: what any plain surface reflects of a sky).
void PXTGIEmissionMeter::_start_batch() {
	_clear_viewport();
	batch.clear();
	const int count = MIN(pending.size(), MAX_COLUMNS * MAX_COLUMNS - 2);
	for (int i = 0; i < count; i++) {
		batch.push_back(pending[0]);
		pending.remove_at(0);
	}
	if (!batch.is_empty()) {
		if (white_reference.is_null()) {
			Ref<Shader> shader;
			shader.instantiate();
			shader->set_code("shader_type spatial;\nvoid fragment() { ALBEDO = vec3(1.0); }\n");
			white_reference.instantiate();
			white_reference->set_shader(shader);
			Ref<Shader> black;
			black.instantiate();
			black->set_code("shader_type spatial;\nvoid fragment() { ALBEDO = vec3(0.0); }\n");
			black_reference.instantiate();
			black_reference->set_shader(black);
		}
		batch.push_back(white_reference);
		batch.push_back(black_reference);
	}
	if (batch.is_empty()) {
		measuring = false;
		SceneTree *tree = SceneTree::get_singleton();
		if (tree && tree->is_connected(SNAME("process_frame"), callable_mp(this, &PXTGIEmissionMeter::_frame))) {
			tree->disconnect(SNAME("process_frame"), callable_mp(this, &PXTGIEmissionMeter::_frame));
		}
		emit_signal(SNAME("finished"), results);
		return;
	}
	columns = (int)Math::ceil(Math::sqrt((double)batch.size()));
	const int rows = (batch.size() + columns - 1) / columns;

	SubViewport *vp = memnew(SubViewport);
	vp->set_size(Size2i(columns * TILE, rows * TILE));
	vp->set_use_own_world_3d(true);
	vp->set_use_hdr_2d(true);
	vp->set_update_mode(SubViewport::UPDATE_ALWAYS);

	// Nothing lights the tiles: what's drawn is the emission alone, at exposure 1 / SCALE. With physical light
	// units, CameraAttributesPractical's exposure normalization is ISO / 3072007 (1 at that ISO); without them
	// the sensitivity isn't used and the normalization is 1.
	Ref<Environment> env;
	env.instantiate();
	environment = env;
	env->set_background(Environment::BG_COLOR);
	env->set_bg_color(Color(0, 0, 0));
	env->set_ambient_source(Environment::AMBIENT_SOURCE_DISABLED);
	env->set_reflection_source(Environment::REFLECTION_SOURCE_DISABLED);
	env->set_tonemapper(Environment::TONE_MAPPER_LINEAR);
	env->set_tonemap_exposure(1.0f);
	env->set_glow_enabled(false);
	attributes.instantiate();
	attributes->set_exposure_sensitivity(3072007.0f);
	attributes->set_exposure_multiplier(1.0f / SCALE);
	WorldEnvironment *world_env = memnew(WorldEnvironment);
	world_env->set_environment(env);
	world_env->set_camera_attributes(attributes);
	vp->add_child(world_env);

	Camera3D *camera = memnew(Camera3D);
	camera->set_projection(Camera3D::PROJECTION_ORTHOGONAL);
	camera->set_keep_aspect_mode(Camera3D::KEEP_HEIGHT);
	camera->set_size(rows);
	camera->set_position(Vector3(0, 0, 1));
	vp->add_child(camera);
	camera->set_current(true);

	Ref<QuadMesh> quad;
	quad.instantiate();
	quad->set_size(Size2(1, 1));
	for (int i = 0; i < batch.size(); i++) {
		MeshInstance3D *tile = memnew(MeshInstance3D);
		tile->set_mesh(quad);
		tile->set_cast_shadows_setting(GeometryInstance3D::SHADOW_CASTING_SETTING_OFF);
		tile->set_material_override(batch[i]);
		tile->set_position(Vector3((i % columns) + 0.5f - columns * 0.5f, rows * 0.5f - (i / columns) - 0.5f, 0.0f));
		vp->add_child(tile);
	}
	SceneTree::get_singleton()->get_root()->add_child(vp, false, Node::INTERNAL_MODE_BACK);
	viewport = vp->get_instance_id();
	frame = 0;
	reads = 0;
	shine_pass = false;
	sums.clear();
	lit_sums.clear();
	shine_sums.clear();
	lowest.clear();
	highest.clear();
}

// Each tile's average, away from its edges, back at full brightness.
Vector<Color> PXTGIEmissionMeter::_read_tiles() const {
	Vector<Color> tiles;
	SubViewport *vp = ObjectDB::get_instance<SubViewport>(viewport);
	ERR_FAIL_NULL_V(vp, tiles);
	const Ref<Image> image = vp->get_texture()->get_image();
	ERR_FAIL_COND_V(image.is_null(), tiles);
	constexpr int BORDER = 2;
	for (int i = 0; i < batch.size(); i++) {
		const int x0 = (i % columns) * TILE;
		const int y0 = (i / columns) * TILE;
		Color sum(0, 0, 0, 0);
		int n = 0;
		for (int y = y0 + BORDER; y < y0 + TILE - BORDER; y++) {
			for (int x = x0 + BORDER; x < x0 + TILE - BORDER; x++) {
				sum += image->get_pixel(x, y);
				n++;
			}
		}
		Color c = sum / float(MAX(n, 1)) * SCALE;
		c.a = 1.0f;
		tiles.push_back(c);
	}
	return tiles;
}

void PXTGIEmissionMeter::_frame() {
	frame++;
	if (shine_pass) {
		_frame_shine();
		return;
	}
	if (reads >= READS) {
		_frame_lit();
		return;
	}
	if (frame < FIRST_READ || (frame - FIRST_READ) % READ_STEP != 0) {
		return;
	}
	const Vector<Color> tiles = _read_tiles();
	if (sums.size() != batch.size()) {
		sums.resize(batch.size());
		lowest.resize(batch.size());
		highest.resize(batch.size());
		for (int i = 0; i < batch.size(); i++) {
			sums.write[i] = Color(0, 0, 0, 0);
			lowest.write[i] = FLT_MAX;
			highest.write[i] = 0.0f;
		}
	}
	for (int i = 0; i < batch.size() && i < tiles.size(); i++) {
		sums.write[i] += tiles[i];
		lowest.write[i] = MIN(lowest[i], tiles[i].get_luminance());
		highest.write[i] = MAX(highest[i], tiles[i].get_luminance());
	}
	reads++;
	if (reads < READS) {
		return;
	}
	// An even white ambient light. How strongly it's drawn (the camera's exposure doesn't apply to it) is read off
	// the batch's reference tile, white, rather than assumed.
	environment->set_ambient_source(Environment::AMBIENT_SOURCE_COLOR);
	environment->set_ambient_light_color(Color(1, 1, 1));
	environment->set_ambient_light_energy(1.0f);
	frame = 0;
}

void PXTGIEmissionMeter::_frame_lit() {
	if (frame < LIT_FIRST) {
		return;
	}
	const Vector<Color> tiles = _read_tiles();
	if (lit_sums.size() != batch.size()) {
		lit_sums.resize(batch.size());
		for (int i = 0; i < batch.size(); i++) {
			lit_sums.write[i] = Color(0, 0, 0, 0);
		}
	}
	for (int i = 0; i < batch.size() && i < tiles.size(); i++) {
		lit_sums.write[i] += tiles[i];
	}
	if (frame < LIT_FIRST + LIT_READS - 1) {
		return;
	}
	// A white sky for the ambient light and the reflections (a metal reflects no diffuse light, only the sky). The
	// ambient light can't be left off: Godot dims reflections where it's dark (specular occlusion). What it adds
	// is the diffuse share already read, taken back off. A sky doesn't come out at a low exposure (it reads black
	// at 1 / SCALE), so this pass is drawn at exposure 1, with a sky bright enough that a glow beside it doesn't
	// swamp it; the references are read again in it, so its brightness isn't assumed.
	if (white_sky.is_null()) {
		Ref<Shader> shader;
		shader.instantiate();
		shader->set_code("shader_type sky;\nvoid sky() { COLOR = vec3(64.0); }\n");
		Ref<ShaderMaterial> material;
		material.instantiate();
		material->set_shader(shader);
		white_sky.instantiate();
		white_sky->set_material(material);
		white_sky->set_radiance_size(Sky::RADIANCE_SIZE_32);
		white_sky->set_process_mode(Sky::PROCESS_MODE_QUALITY);
	}
	environment->set_sky(white_sky);
	attributes->set_exposure_multiplier(1.0f);
	environment->set_ambient_source(Environment::AMBIENT_SOURCE_SKY);
	environment->set_ambient_light_sky_contribution(1.0f);
	environment->set_reflection_source(Environment::REFLECTION_SOURCE_SKY);
	shine_pass = true;
	frame = 0;
}

void PXTGIEmissionMeter::_frame_shine() {
	if (frame < SHINE_FIRST) {
		return;
	}
	const Vector<Color> tiles = _read_tiles();
	if (shine_sums.size() != batch.size()) {
		shine_sums.resize(batch.size());
		for (int i = 0; i < batch.size(); i++) {
			shine_sums.write[i] = Color(0, 0, 0, 0);
		}
	}
	for (int i = 0; i < batch.size() && i < tiles.size(); i++) {
		shine_sums.write[i] += tiles[i];
	}
	if (frame < SHINE_FIRST + LIT_READS - 1) {
		return;
	}
	// The albedo: what the ambient light added to a tile, as a share of what it added to the white reference (d), and
	// what the sky's reflections added past the black reference's (the little any plain surface reflects, already in
	// a dielectric's albedo), as a share of the sky's light. Under the sky a tile reads d times the sky's light (the
	// white reference's less the black's: a white diffuser takes in all of it) plus its reflection. So a
	// dielectric's albedo is its diffuse color, as PXTGI reads a BaseMaterial3D's albedo, and a metal's is the share
	// of light it reflects as Godot draws it (a rough metal's is well under its color).
	const int white_tile = batch.size() - 2;
	const int black_tile = batch.size() - 1;
	// What a pass added to a tile past its glow (drawn SCALE times brighter in the sky's pass, at exposure 1).
	const auto added = [&](const Vector<Color> &p_sums, int p_tile, float p_glow_scale) {
		Color a = p_sums[p_tile] / float(LIT_READS) - sums[p_tile] * (p_glow_scale / float(READS));
		a.a = 1.0f;
		return a;
	};
	const Color white = added(lit_sums, white_tile, 1.0f);
	const Color sky_white = added(shine_sums, white_tile, SCALE);
	const Color sky_black = added(shine_sums, black_tile, SCALE);
	const auto share = [](float p_value, float p_reference) {
		return p_reference > 0.0f ? p_value / p_reference : 0.0f;
	};
	for (int i = 0; i < white_tile; i++) {
		Color glow = sums[i] / float(READS);
		glow.a = 1.0f;
		const Color diffuse = added(lit_sums, i, 1.0f);
		const Color sky = added(shine_sums, i, SCALE);
		Color albedo(0, 0, 0, 1);
		for (int c = 0; c < 3; c++) {
			const float d = share(diffuse.components[c], white.components[c]);
			const float sky_diffuse = sky_white.components[c] - sky_black.components[c];
			const float reflected = sky.components[c] - d * sky_diffuse - sky_black.components[c];
			const float s = share(reflected, sky_diffuse);
			albedo.components[c] = CLAMP(d + MAX(s, 0.0f), 0.0f, 1.0f);
		}
		Dictionary result;
		result["material"] = batch[i];
		result["glow"] = glow; // the average over the readings: a pulsing glow's mean
		result["luminance"] = glow.get_luminance();
		// A glow that moved across the readings (animated by TIME).
		result["changes"] = highest[i] - lowest[i] > 0.02f * highest[i] + 0.01f;
		result["albedo"] = albedo; // linear
		results.push_back(result);
	}
	_start_batch();
}

void PXTGIEmissionMeter::_clear_viewport() {
	if (Node *vp = ObjectDB::get_instance<Node>(viewport)) {
		vp->queue_free();
	}
	viewport = ObjectID();
	environment.unref();
	attributes.unref();
}

Dictionary PXTGIEmissionMeter::find_untagged(Node *p_root) {
	Dictionary found;
	ERR_FAIL_NULL_V(p_root, found);
	TypedArray<Node> nodes = p_root->find_children("*", "MeshInstance3D", true, false);
	nodes.push_front(p_root);
	for (int k = 0; k < nodes.size(); k++) {
		MeshInstance3D *mesh = Object::cast_to<MeshInstance3D>(nodes[k]);
		if (mesh == nullptr || mesh->get_mesh().is_null() || mesh->has_meta(meta_emission())) {
			continue;
		}
		for (int i = 0; i < mesh->get_mesh()->get_surface_count(); i++) {
			const Ref<ShaderMaterial> material = mesh->get_active_material(i);
			if (material.is_null() || material->has_meta(meta_emission())) {
				continue;
			}
			Array users = found.get(material, Array());
			if (!users.has(mesh)) {
				users.push_back(mesh);
			}
			found[material] = users;
		}
	}
	return found;
}

Dictionary PXTGIEmissionMeter::find_untagged_albedo(Node *p_root) {
	Dictionary found;
	ERR_FAIL_NULL_V(p_root, found);
	const StringName meta_albedo = "pxtgi_albedo";
	TypedArray<Node> nodes = p_root->find_children("*", "MeshInstance3D", true, false);
	nodes.push_front(p_root);
	for (int k = 0; k < nodes.size(); k++) {
		MeshInstance3D *mesh = Object::cast_to<MeshInstance3D>(nodes[k]);
		if (mesh == nullptr || mesh->get_mesh().is_null()) {
			continue;
		}
		for (int i = 0; i < mesh->get_mesh()->get_surface_count(); i++) {
			const Ref<Material> material = mesh->get_active_material(i);
			if (material.is_null() || material->has_meta(meta_albedo)) {
				continue;
			}
			// PXTGI reads a BaseMaterial3D's albedo color, but not its texture's colors; nothing of a ShaderMaterial's.
			const Ref<BaseMaterial3D> base = material;
			if (base.is_valid() && base->get_texture(BaseMaterial3D::TEXTURE_ALBEDO).is_null()) {
				continue;
			}
			if (base.is_null() && Ref<ShaderMaterial>(material).is_null()) {
				continue;
			}
			Array users = found.get(material, Array());
			if (!users.has(mesh)) {
				users.push_back(mesh);
			}
			found[material] = users;
		}
	}
	return found;
}

Color PXTGIEmissionMeter::albedo_tag(const Color &p_albedo) {
	Color tag = Color(CLAMP(p_albedo.r, 0.0f, 1.0f), CLAMP(p_albedo.g, 0.0f, 1.0f), CLAMP(p_albedo.b, 0.0f, 1.0f)).linear_to_srgb();
	tag.a = 1.0f;
	return tag;
}

Color PXTGIEmissionMeter::emission_tag(const Color &p_glow) {
	Color tag = Color(MAX(p_glow.r, 0.0f), MAX(p_glow.g, 0.0f), MAX(p_glow.b, 0.0f)).linear_to_srgb();
	tag.a = 1.0f;
	return tag;
}

void PXTGIEmissionMeter::_bind_methods() {
	ClassDB::bind_method(D_METHOD("measure", "materials"), &PXTGIEmissionMeter::measure);
	ClassDB::bind_method(D_METHOD("is_measuring"), &PXTGIEmissionMeter::is_measuring);
	ClassDB::bind_static_method("PXTGIEmissionMeter", D_METHOD("find_untagged", "root"), &PXTGIEmissionMeter::find_untagged);
	ClassDB::bind_static_method("PXTGIEmissionMeter", D_METHOD("emission_tag", "glow"), &PXTGIEmissionMeter::emission_tag);
	ClassDB::bind_static_method("PXTGIEmissionMeter", D_METHOD("find_untagged_albedo", "root"), &PXTGIEmissionMeter::find_untagged_albedo);
	ClassDB::bind_static_method("PXTGIEmissionMeter", D_METHOD("albedo_tag", "albedo"), &PXTGIEmissionMeter::albedo_tag);
	ADD_SIGNAL(MethodInfo("finished", PropertyInfo(Variant::ARRAY, "results")));
}

PXTGIEmissionMeter::~PXTGIEmissionMeter() {
	_clear_viewport();
	SceneTree *tree = SceneTree::get_singleton();
	if (measuring && tree && tree->is_connected(SNAME("process_frame"), callable_mp(this, &PXTGIEmissionMeter::_frame))) {
		tree->disconnect(SNAME("process_frame"), callable_mp(this, &PXTGIEmissionMeter::_frame));
	}
}

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

namespace {

// Frames from a batch's start to its readings: the first once the shaders have compiled and drawn, then READS of
// them READ_STEP frames apart, so a glow animated by TIME is seen at several points of its cycle (two readings can
// land on the same brightness of a pulse).
constexpr int FIRST_READ = 4;
constexpr int READ_STEP = 6;
constexpr int READS = 6;

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

// The next up to MAX_COLUMNS^2 materials, each on a quad that fills its tile of an orthographic view.
void PXTGIEmissionMeter::_start_batch() {
	_clear_viewport();
	batch.clear();
	const int count = MIN(pending.size(), MAX_COLUMNS * MAX_COLUMNS);
	for (int i = 0; i < count; i++) {
		batch.push_back(pending[0]);
		pending.remove_at(0);
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
	env->set_background(Environment::BG_COLOR);
	env->set_bg_color(Color(0, 0, 0));
	env->set_ambient_source(Environment::AMBIENT_SOURCE_DISABLED);
	env->set_reflection_source(Environment::REFLECTION_SOURCE_DISABLED);
	env->set_tonemapper(Environment::TONE_MAPPER_LINEAR);
	env->set_tonemap_exposure(1.0f);
	env->set_glow_enabled(false);
	Ref<CameraAttributesPractical> attributes;
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
	sums.clear();
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
	for (int i = 0; i < batch.size(); i++) {
		Color glow = sums[i] / float(READS);
		glow.a = 1.0f;
		Dictionary result;
		result["material"] = batch[i];
		result["glow"] = glow; // the average over the readings: a pulsing glow's mean
		result["luminance"] = glow.get_luminance();
		// A glow that moved across the readings (animated by TIME).
		result["changes"] = highest[i] - lowest[i] > 0.02f * highest[i] + 0.01f;
		results.push_back(result);
	}
	_start_batch();
}

void PXTGIEmissionMeter::_clear_viewport() {
	if (Node *vp = ObjectDB::get_instance<Node>(viewport)) {
		vp->queue_free();
	}
	viewport = ObjectID();
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
	ADD_SIGNAL(MethodInfo("finished", PropertyInfo(Variant::ARRAY, "results")));
}

PXTGIEmissionMeter::~PXTGIEmissionMeter() {
	_clear_viewport();
	SceneTree *tree = SceneTree::get_singleton();
	if (measuring && tree && tree->is_connected(SNAME("process_frame"), callable_mp(this, &PXTGIEmissionMeter::_frame))) {
		tree->disconnect(SNAME("process_frame"), callable_mp(this, &PXTGIEmissionMeter::_frame));
	}
}

/**************************************************************************/
/*  physx_blast_fracture_dialog.cpp                                       */
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
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "physx_blast_fracture_dialog.h"

#ifdef GODOT_PHYSX_BLAST

#include "../blast/physx_blast_asset.h"
#include "../blast/physx_blast_authoring.h"
#include "../blast/physx_destructible_3d.h"

#include "core/io/resource_loader.h"
#include "core/io/resource_saver.h"
#include "core/object/callable_mp.h"
#include "editor/editor_node.h"
#include "editor/editor_string_names.h"
#include "editor/editor_undo_redo_manager.h"
#include "editor/file_system/editor_file_system.h"
#include "editor/scene/3d/mesh_editor_plugin.h"
#include "editor/themes/editor_scale.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/node_3d.h"
#include "scene/gui/box_container.h"
#include "scene/gui/button.h"
#include "scene/gui/label.h"
#include "scene/gui/spin_box.h"
#include "scene/resources/material.h"
#include "scene/resources/mesh.h"
#include "scene/scene_string_names.h"

static Color _blast_preview_chunk_color(int p_index) {
	// Golden-ratio hue step: successive chunks get visually distinct colors
	// with no lookup table and no risk of two adjacent chunks landing on the
	// same hue.
	const double hue = Math::fmod(p_index * 0.6180339887, 1.0);
	return Color::from_hsv(hue, 0.55, 0.85);
}

void PhysXBlastFractureDialog::_start(const Ref<Mesh> &p_mesh) {
	source_mesh = p_mesh;
	seed = 1;
	_regenerate();
	popup_centered(Size2(640, 560));
}

void PhysXBlastFractureDialog::_regenerate() {
	seed += 1;
	const int site_count = site_count_spin ? (int)site_count_spin->get_value() : 12;

	Ref<PhysXBlastAuthoring> authoring;
	authoring.instantiate();
	authored_asset = authoring->fracture_mesh(source_mesh, site_count, seed);

	if (authored_asset.is_null()) {
		if (chunk_count_label) {
			chunk_count_label->set_text(TTR("Fracture failed -- check the mesh has real geometry."));
		}
		get_ok_button()->set_disabled(true);
		return;
	}

	get_ok_button()->set_disabled(false);
	const int piece_count = MAX(authored_asset->get_chunk_count() - 1, 0);
	if (chunk_count_label) {
		chunk_count_label->set_text(vformat(TTR("%d pieces"), piece_count));
	}

	// One surface per leaf chunk (index 0 is the unfractured root chunk, per
	// GodotPhysXBlastProbe/PhysXDestructible3D's own convention -- never
	// rendered on its own here), each a flat distinct color so the fracture
	// cells read clearly regardless of scene lighting in the preview.
	Ref<ArrayMesh> preview_mesh;
	preview_mesh.instantiate();
	const Array points = authored_asset->get_chunk_points();
	for (int i = 1; i < points.size(); i++) {
		const PackedVector3Array tri = points[i];
		if (tri.size() < 3) {
			continue;
		}

		PackedVector3Array normals;
		normals.resize(tri.size());
		for (int t = 0; t + 2 < tri.size(); t += 3) {
			// Same winding fix PhysXDestructible3D uses for this same
			// triangle-soup data -- (c-a).cross(b-a), not the more intuitive
			// (b-a).cross(c-a) -- see that node's normal computation.
			const Vector3 n = (tri[t + 2] - tri[t]).cross(tri[t + 1] - tri[t]).normalized();
			normals.set(t, n);
			normals.set(t + 1, n);
			normals.set(t + 2, n);
		}

		Array arrays;
		arrays.resize(Mesh::ARRAY_MAX);
		arrays[Mesh::ARRAY_VERTEX] = tri;
		arrays[Mesh::ARRAY_NORMAL] = normals;
		preview_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_albedo(_blast_preview_chunk_color(i));
		preview_mesh->surface_set_material(preview_mesh->get_surface_count() - 1, mat);
	}

	if (preview) {
		preview->edit(preview_mesh);
	}
}

void PhysXBlastFractureDialog::_on_confirmed() {
	if (authored_asset.is_null()) {
		return;
	}

	String save_dir;
	String base_name;
	if (mode == MODE_SCENE_NODE && target_node) {
		Node *scene_root = EditorNode::get_singleton()->get_edited_scene();
		const String scene_path = scene_root ? scene_root->get_scene_file_path() : String();
		save_dir = scene_path.is_empty() ? "res://" : scene_path.get_base_dir();
		base_name = target_node->get_name();
	} else if (mode == MODE_MESH_RESOURCE) {
		save_dir = target_mesh_path.get_base_dir();
		base_name = target_mesh_path.get_file().get_basename();
	} else {
		return;
	}

	const String asset_path = save_dir.path_join(base_name + "_blast.tres");
	const Error err = ResourceSaver::save(authored_asset, asset_path);
	if (err != OK) {
		ERR_PRINT(vformat("PhysXBlastFractureDialog: failed to save '%s' (error %d).", asset_path, (int)err));
		return;
	}
	authored_asset->set_path(asset_path, true);
	EditorFileSystem::get_singleton()->update_file(asset_path);

	if (mode == MODE_SCENE_NODE && target_node) {
		Node3D *old_node = Object::cast_to<Node3D>(target_node);
		if (!old_node || !old_node->get_parent()) {
			return;
		}

		PhysXDestructible3D *destructible = memnew(PhysXDestructible3D);
		destructible->set_name(old_node->get_name());
		destructible->set_transform(old_node->get_transform());
		destructible->set_blast_asset(authored_asset);

		EditorUndoRedoManager *ur = EditorUndoRedoManager::get_singleton();
		ur->create_action(TTR("Fracture Mesh with Blast"));
		ur->add_do_method(old_node, "replace_by", destructible, true);
		ur->add_do_reference(destructible);
		ur->add_undo_method(destructible, "replace_by", old_node, true);
		ur->add_undo_reference(old_node);
		ur->commit_action();
	}
}

void PhysXBlastFractureDialog::open_for_node(Node *p_mesh_instance) {
	MeshInstance3D *mi = Object::cast_to<MeshInstance3D>(p_mesh_instance);
	if (!mi || mi->get_mesh().is_null()) {
		return;
	}
	mode = MODE_SCENE_NODE;
	target_node = p_mesh_instance;
	target_mesh_path = String();
	_start(mi->get_mesh());
}

void PhysXBlastFractureDialog::open_for_mesh_resource(const String &p_mesh_path) {
	Ref<Mesh> mesh = ResourceLoader::load(p_mesh_path);
	if (mesh.is_null()) {
		return;
	}
	mode = MODE_MESH_RESOURCE;
	target_node = nullptr;
	target_mesh_path = p_mesh_path;
	_start(mesh);
}

PhysXBlastFractureDialog::PhysXBlastFractureDialog() {
	set_title(TTR("Fracture Mesh with Blast"));

	VBoxContainer *root = memnew(VBoxContainer);
	add_child(root);

	preview = memnew(MeshEditor);
	preview->set_custom_minimum_size(Size2(0, 360) * EDSCALE);
	preview->set_v_size_flags(Control::SIZE_EXPAND_FILL);
	root->add_child(preview);

	HBoxContainer *controls = memnew(HBoxContainer);
	root->add_child(controls);

	controls->add_child(memnew(Label(TTR("Chunks:"))));
	site_count_spin = memnew(SpinBox);
	site_count_spin->set_min(4);
	site_count_spin->set_max(200);
	site_count_spin->set_step(1);
	site_count_spin->set_value(12);
	site_count_spin->set_custom_minimum_size(Size2(80, 0) * EDSCALE);
	controls->add_child(site_count_spin);

	Button *regenerate_button = memnew(Button);
	regenerate_button->set_text(TTR("Regenerate"));
	regenerate_button->connect(SceneStringName(pressed), callable_mp(this, &PhysXBlastFractureDialog::_regenerate));
	controls->add_child(regenerate_button);

	chunk_count_label = memnew(Label);
	chunk_count_label->set_h_size_flags(Control::SIZE_EXPAND_FILL);
	chunk_count_label->set_horizontal_alignment(HORIZONTAL_ALIGNMENT_RIGHT);
	controls->add_child(chunk_count_label);

	set_ok_button_text(TTR("Accept"));
}

void PhysXBlastFractureDialog::_notification(int p_what) {
	// Self-signal connect deferred to ENTER_TREE (not the constructor):
	// this class is never ClassDB-registered (it's editor-internal, not
	// scriptable), so its own GDType isn't initialized yet when the
	// constructor runs -- connecting "confirmed" (declared on the AcceptDialog
	// ancestor) on `this` that early fails to find it. Matches the working
	// pattern editor/gui/create_dialog.cpp's CreateDialog already uses for
	// the exact same reason.
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			connect(SceneStringName(confirmed), callable_mp(this, &PhysXBlastFractureDialog::_on_confirmed));
		} break;
		case NOTIFICATION_EXIT_TREE: {
			disconnect(SceneStringName(confirmed), callable_mp(this, &PhysXBlastFractureDialog::_on_confirmed));
		} break;
	}
}

#endif // GODOT_PHYSX_BLAST

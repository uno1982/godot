/**************************************************************************/
/*  pxtgi_emission_dialog.cpp                                             */
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

#include "pxtgi_emission_dialog.h"

#include "../nodes/pxtgi_3d.h"

#include "core/config/project_settings.h"
#include "core/io/resource_saver.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "editor/editor_data.h"
#include "editor/editor_node.h"
#include "editor/editor_undo_redo_manager.h"
#include "editor/scene/3d/node_3d_editor_plugin.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/gui/box_container.h"
#include "scene/gui/button.h"
#include "scene/gui/label.h"
#include "scene/gui/tree.h"
#include "scene/resources/material.h"

namespace {

enum Column {
	COLUMN_MATERIAL,
	COLUMN_GLOW,
	COLUMN_MESHES,
	COLUMN_NOTES,
};

// Where a material lives: its own file (saved here), a scene (saved with it), or an imported file (can't be).
enum Home {
	HOME_FILE,
	HOME_SCENE,
	HOME_IMPORTED,
};

Home material_home(const Ref<Material> &p_material, String &r_file) {
	const String path = p_material->get_path();
	r_file = path.get_slice("::", 0);
	if (path.is_empty() || !path.contains("::")) {
		return path.is_empty() ? HOME_SCENE : HOME_FILE;
	}
	const String ext = r_file.get_extension().to_lower();
	return (ext == "tscn" || ext == "scn" || ext == "tres" || ext == "res") ? HOME_SCENE : HOME_IMPORTED;
}

String material_label(const Ref<Material> &p_material) {
	if (!p_material->get_name().is_empty()) {
		return p_material->get_name();
	}
	const String path = p_material->get_path();
	return path.is_empty() ? String("(built-in)") : path.get_file();
}

bool physical_units() {
	return GLOBAL_GET("rendering/lights_and_shadows/use_physical_light_units");
}

} // namespace

PXTGIEmissionDialog::PXTGIEmissionDialog() {
	set_title("Find Emissive Materials");
	set_ok_button_text("Tag Selected");
	VBoxContainer *box = memnew(VBoxContainer);
	add_child(box);
	status = memnew(Label);
	status->set_autowrap_mode(TextServer::AUTOWRAP_WORD_SMART);
	// A real width: wrapping at 1 pixel, the dialog would size itself to that text's height (screen tall).
	status->set_custom_minimum_size(Size2(860, 0));
	box->add_child(status);
	tree = memnew(Tree);
	tree->set_columns(4);
	tree->set_column_titles_visible(true);
	tree->set_column_title(COLUMN_MATERIAL, "Material");
	tree->set_column_title(COLUMN_GLOW, physical_units() ? "Glow (nits)" : "Glow (energy)");
	tree->set_column_title(COLUMN_MESHES, "Meshes");
	tree->set_column_title(COLUMN_NOTES, "Notes");
	tree->set_column_title_tooltip_text(COLUMN_NOTES, "Empty unless something needs attention: a glow that changes over time (its mean is shown), a material inside an imported file (a tag there is lost on reimport, so it can't be ticked), or one built into another scene (kept when that scene is saved).");
	tree->set_column_expand(COLUMN_MATERIAL, true);
	tree->set_column_expand(COLUMN_GLOW, false);
	tree->set_column_custom_minimum_width(COLUMN_GLOW, 140);
	tree->set_column_expand(COLUMN_MESHES, false);
	tree->set_column_custom_minimum_width(COLUMN_MESHES, 70);
	tree->set_column_expand(COLUMN_NOTES, true);
	tree->set_hide_root(true);
	tree->set_v_size_flags(Control::SIZE_EXPAND_FILL);
	tree->set_custom_minimum_size(Size2(860, 240));
	tree->connect("item_activated", callable_mp(this, &PXTGIEmissionDialog::_item_activated));
	box->add_child(tree);
	rescan = add_button("Measure Again", false, "rescan");
	rescan->connect(SceneStringName(pressed), callable_mp(this, &PXTGIEmissionDialog::_scan));
	connect(SceneStringName(confirmed), callable_mp(this, &PXTGIEmissionDialog::_apply));
	meter.instantiate();
	meter->connect("finished", callable_mp(this, &PXTGIEmissionDialog::_measured));
}

void PXTGIEmissionDialog::popup_for_scene() {
	popup_centered_clamped(Size2(920, 560), 0.7);
	_scan();
}

void PXTGIEmissionDialog::_scan() {
	tree->clear();
	get_ok_button()->set_disabled(true);
	Node *root = EditorNode::get_singleton()->get_edited_scene();
	if (root == nullptr) {
		status->set_text("Open a scene first.");
		return;
	}
	if (meter->is_measuring()) {
		return;
	}
	users = PXTGIEmissionMeter::find_untagged(root);
	if (users.is_empty()) {
		status->set_text("Every ShaderMaterial in this scene already has a pxtgi_emission tag (on the material or its mesh), or there are none.");
		return;
	}
	status->set_text(vformat("Measuring %d untagged ShaderMaterials...", users.size()));
	rescan->set_disabled(true);
	TypedArray<Material> materials;
	for (const Variant &key : users.keys()) {
		materials.push_back(key);
	}
	meter->measure(materials);
}

void PXTGIEmissionDialog::_measured(const Array &p_results) {
	rescan->set_disabled(false);
	if (!is_visible()) {
		return;
	}
	Array results = p_results.duplicate();
	results.sort_custom(callable_mp_static(+[](const Variant &p_a, const Variant &p_b) -> bool {
		return float(Dictionary(p_a)["luminance"]) > float(Dictionary(p_b)["luminance"]);
	}));
	TreeItem *root = tree->create_item();
	int glowing = 0;
	for (const Variant &v : results) {
		const Dictionary result = v;
		const Ref<Material> material = result["material"];
		const float luminance = result["luminance"];
		const bool glows = luminance > 0.01f;
		glowing += glows ? 1 : 0;
		String file;
		const Home home = material_home(material, file);
		PackedStringArray notes;
		if (bool(result["changes"])) {
			notes.push_back("changes over time: its mean");
		}
		if (home == HOME_IMPORTED) {
			notes.push_back("inside imported " + file.get_file() + ": a tag there is lost on reimport (make the material unique or save it to a file first)");
		} else if (home == HOME_SCENE && !file.is_empty()) {
			const Node *scene = EditorNode::get_singleton()->get_edited_scene();
			if (scene == nullptr || scene->get_scene_file_path() != file) {
				notes.push_back("built into " + file.get_file() + ": kept when that scene is saved");
			}
		}

		TreeItem *item = tree->create_item(root);
		item->set_cell_mode(COLUMN_MATERIAL, TreeItem::CELL_MODE_CHECK);
		item->set_text(COLUMN_MATERIAL, material_label(material));
		item->set_tooltip_text(COLUMN_MATERIAL, material->get_path());
		item->set_editable(COLUMN_MATERIAL, home != HOME_IMPORTED);
		item->set_checked(COLUMN_MATERIAL, glows && home != HOME_IMPORTED);
		item->set_metadata(COLUMN_MATERIAL, material);
		item->set_metadata(COLUMN_GLOW, result["glow"]);
		item->set_cell_mode(COLUMN_GLOW, TreeItem::CELL_MODE_RANGE);
		item->set_range_config(COLUMN_GLOW, 0.0, 1.0e9, 0.001);
		item->set_range(COLUMN_GLOW, luminance);
		item->set_editable(COLUMN_GLOW, true);
		const Array meshes = users.get(material, Array());
		item->set_text(COLUMN_MESHES, itos(meshes.size()));
		item->set_tooltip_text(COLUMN_MESHES, "Double-click the row to select one and move the 3D view to it.");
		item->set_text(COLUMN_NOTES, String("; ").join(notes));
		item->set_tooltip_text(COLUMN_NOTES, String("\n").join(notes));
	}
	status->set_text(vformat("%d of %d untagged ShaderMaterials glow (PXTGI gets no light from them off screen). The ticked ones get a pxtgi_emission tag of the glow shown, which you can edit: it should be what the material draws.", glowing, results.size()));
	get_ok_button()->set_disabled(false);
}

void PXTGIEmissionDialog::_apply() {
	EditorUndoRedoManager *ur = EditorUndoRedoManager::get_singleton();
	const StringName meta = "pxtgi_emission";
	int count = 0;
	TreeItem *root = tree->get_root();
	for (TreeItem *item = root ? root->get_first_child() : nullptr; item; item = item->get_next()) {
		if (!item->is_checked(COLUMN_MATERIAL)) {
			continue;
		}
		const Ref<Material> material = item->get_metadata(COLUMN_MATERIAL);
		if (material.is_null()) {
			continue;
		}
		if (count == 0) {
			ur->create_action("Tag Emissive Materials for PXTGI");
		}
		count++;
		// The measured color, scaled to the brightness in the list (edited or not).
		const Color measured = item->get_metadata(COLUMN_GLOW);
		const float wanted = item->get_range(COLUMN_GLOW);
		const float measured_luminance = measured.get_luminance();
		const Color glow = measured_luminance > 1.0e-6f ? measured * (wanted / measured_luminance) : Color(wanted, wanted, wanted);
		ur->add_do_method(material.ptr(), "set_meta", meta, PXTGIEmissionMeter::emission_tag(glow));
		if (material->has_meta(meta)) {
			ur->add_undo_method(material.ptr(), "set_meta", meta, material->get_meta(meta));
		} else {
			ur->add_undo_method(material.ptr(), "remove_meta", meta);
		}
		ur->add_do_method(this, "save_material", material);
		ur->add_undo_method(this, "save_material", material);
	}
	if (count > 0) {
		ur->commit_action();
	}
}

void PXTGIEmissionDialog::save_material(const Ref<Material> &p_material) {
	String file;
	if (p_material.is_valid() && material_home(p_material, file) == HOME_FILE) {
		ResourceSaver::save(p_material, file);
	}
}

void PXTGIEmissionDialog::_item_activated() {
	TreeItem *item = tree->get_selected();
	if (item == nullptr) {
		return;
	}
	const Array meshes = users.get(item->get_metadata(COLUMN_MATERIAL), Array());
	if (meshes.is_empty()) {
		return;
	}
	Node *mesh = Object::cast_to<Node>(meshes[0]);
	if (mesh && mesh->is_inside_tree()) {
		EditorSelection *selection = EditorNode::get_singleton()->get_editor_selection();
		selection->clear();
		selection->add_node(mesh);
		// And the 3D view to it, as Focus Selection (F) does.
		if (Node3DEditor::get_singleton()) {
			if (Node3DEditorViewport *view = Node3DEditor::get_singleton()->get_last_used_viewport()) {
				view->focus_selection();
			}
		}
	}
}

void PXTGIEmissionDialog::_bind_methods() {
	ClassDB::bind_method(D_METHOD("save_material", "material"), &PXTGIEmissionDialog::save_material);
}

bool EditorInspectorPluginPXTGI::can_handle(Object *p_object) {
	return Object::cast_to<PXTGI3D>(p_object) != nullptr;
}

void EditorInspectorPluginPXTGI::parse_begin(Object *p_object) {
	Button *find = memnew(Button);
	find->set_text("Find Emissive Materials...");
	find->set_tooltip_text("PXTGI can't read a ShaderMaterial's glow: measure the scene's untagged ones and tag them with a pxtgi_emission of what they draw.");
	PXTGIEmissionDialog *target = dialog;
	find->connect(SceneStringName(pressed), callable_mp(target, &PXTGIEmissionDialog::popup_for_scene));
	add_custom_control(find);
}

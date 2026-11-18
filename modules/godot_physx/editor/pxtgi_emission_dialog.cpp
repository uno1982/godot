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
	COLUMN_TAG_GLOW,
	COLUMN_ALBEDO,
	COLUMN_TAG_ALBEDO,
	COLUMN_MESHES,
	COLUMN_NOTES,
	COLUMN_MAX,
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

// A tick box cell, or an empty one when there's nothing to tag.
void set_tag_cell(TreeItem *p_item, int p_column, bool p_offered, bool p_editable, bool p_checked) {
	if (!p_offered) {
		return;
	}
	p_item->set_cell_mode(p_column, TreeItem::CELL_MODE_CHECK);
	p_item->set_editable(p_column, p_editable);
	p_item->set_checked(p_column, p_checked);
}

} // namespace

PXTGIEmissionDialog::PXTGIEmissionDialog() {
	set_title("Measure Materials");
	set_ok_button_text("Tag Selected");
	VBoxContainer *box = memnew(VBoxContainer);
	add_child(box);
	status = memnew(Label);
	status->set_autowrap_mode(TextServer::AUTOWRAP_WORD_SMART);
	// A real width: wrapping at 1 pixel, the dialog would size itself to that text's height (screen tall).
	status->set_custom_minimum_size(Size2(1000, 0));
	box->add_child(status);
	tree = memnew(Tree);
	tree->set_columns(COLUMN_MAX);
	tree->set_column_titles_visible(true);
	tree->set_column_title(COLUMN_MATERIAL, "Material");
	tree->set_column_title(COLUMN_GLOW, physical_units() ? "Glow (nits)" : "Glow (energy)");
	tree->set_column_title(COLUMN_TAG_GLOW, "Tag Glow");
	tree->set_column_title(COLUMN_ALBEDO, "Albedo");
	tree->set_column_title_tooltip_text(COLUMN_ALBEDO, "The share of light the material bounces (textures included, averaged over the surface), as PXTGI bounces it off screen: a dielectric's diffuse color, a metal's reflection (about its color when glossy, dimmer when rough).");
	tree->set_column_title(COLUMN_TAG_ALBEDO, "Tag Albedo");
	tree->set_column_title(COLUMN_MESHES, "Meshes");
	tree->set_column_title(COLUMN_NOTES, "Notes");
	tree->set_column_title_tooltip_text(COLUMN_NOTES, "Empty unless something needs attention: a glow that changes over time (its mean is shown), a material inside an imported file (a tag there is lost on reimport, so it can't be ticked), or one built into another scene (kept when that scene is saved).");
	tree->set_column_expand(COLUMN_MATERIAL, true);
	tree->set_column_expand(COLUMN_GLOW, false);
	tree->set_column_custom_minimum_width(COLUMN_GLOW, 120);
	tree->set_column_expand(COLUMN_TAG_GLOW, false);
	tree->set_column_custom_minimum_width(COLUMN_TAG_GLOW, 80);
	tree->set_column_expand(COLUMN_ALBEDO, false);
	tree->set_column_custom_minimum_width(COLUMN_ALBEDO, 90);
	tree->set_column_expand(COLUMN_TAG_ALBEDO, false);
	tree->set_column_custom_minimum_width(COLUMN_TAG_ALBEDO, 90);
	tree->set_column_expand(COLUMN_MESHES, false);
	tree->set_column_custom_minimum_width(COLUMN_MESHES, 60);
	tree->set_column_expand(COLUMN_NOTES, true);
	tree->set_hide_root(true);
	tree->set_v_size_flags(Control::SIZE_EXPAND_FILL);
	tree->set_custom_minimum_size(Size2(1000, 240));
	tree->connect("item_activated", callable_mp(this, &PXTGIEmissionDialog::_item_activated));
	box->add_child(tree);
	rescan = add_button("Measure Again", false, "rescan");
	rescan->connect(SceneStringName(pressed), callable_mp(this, &PXTGIEmissionDialog::_scan));
	connect(SceneStringName(confirmed), callable_mp(this, &PXTGIEmissionDialog::_apply));
	meter.instantiate();
	meter->connect("finished", callable_mp(this, &PXTGIEmissionDialog::_measured));
}

void PXTGIEmissionDialog::popup_for_scene() {
	popup_centered_clamped(Size2(1060, 560), 0.8);
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
	glow_users = PXTGIEmissionMeter::find_untagged(root);
	albedo_users = PXTGIEmissionMeter::find_untagged_albedo(root);
	TypedArray<Material> materials;
	for (const Variant &key : glow_users.keys()) {
		materials.push_back(key);
	}
	for (const Variant &key : albedo_users.keys()) {
		if (!glow_users.has(key)) {
			materials.push_back(key);
		}
	}
	if (materials.is_empty()) {
		status->set_text("Every material in this scene that PXTGI can't read in full already has its pxtgi_emission and pxtgi_albedo tags, or there are none.");
		return;
	}
	status->set_text(vformat("Measuring %d materials...", materials.size()));
	rescan->set_disabled(true);
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
	int albedos = 0;
	for (const Variant &v : results) {
		const Dictionary result = v;
		const Ref<Material> material = result["material"];
		const bool needs_glow = glow_users.has(material);
		const bool needs_albedo = albedo_users.has(material);
		const float luminance = result["luminance"];
		const bool glows = needs_glow && luminance > 0.01f;
		glowing += glows ? 1 : 0;
		albedos += needs_albedo ? 1 : 0;
		String file;
		const Home home = material_home(material, file);
		PackedStringArray notes;
		if (glows && bool(result["changes"])) {
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
		const bool editable = home != HOME_IMPORTED;

		TreeItem *item = tree->create_item(root);
		item->set_text(COLUMN_MATERIAL, material_label(material));
		item->set_tooltip_text(COLUMN_MATERIAL, material->get_path());
		item->set_metadata(COLUMN_MATERIAL, material);
		if (needs_glow) {
			item->set_metadata(COLUMN_GLOW, result["glow"]);
			item->set_cell_mode(COLUMN_GLOW, TreeItem::CELL_MODE_RANGE);
			item->set_range_config(COLUMN_GLOW, 0.0, 1.0e9, 0.001);
			item->set_range(COLUMN_GLOW, luminance);
			item->set_editable(COLUMN_GLOW, true);
		}
		set_tag_cell(item, COLUMN_TAG_GLOW, needs_glow, editable, glows && editable);
		if (needs_albedo) {
			// The color as sRGB (as a color picker shows it), on a swatch of it.
			const Color albedo = result["albedo"];
			const Color shown = albedo.linear_to_srgb();
			item->set_metadata(COLUMN_ALBEDO, albedo);
			item->set_text(COLUMN_ALBEDO, "#" + shown.to_html(false));
			item->set_custom_bg_color(COLUMN_ALBEDO, shown);
			item->set_custom_color(COLUMN_ALBEDO, shown.get_luminance() > 0.5f ? Color(0, 0, 0) : Color(1, 1, 1));
		}
		set_tag_cell(item, COLUMN_TAG_ALBEDO, needs_albedo, editable, editable);
		Array meshes = glow_users.get(material, Array());
		if (meshes.is_empty()) {
			meshes = albedo_users.get(material, Array());
		}
		item->set_text(COLUMN_MESHES, itos(meshes.size()));
		item->set_tooltip_text(COLUMN_MESHES, "Double-click the row to select one and move the 3D view to it.");
		item->set_text(COLUMN_NOTES, String("; ").join(notes));
		item->set_tooltip_text(COLUMN_NOTES, String("\n").join(notes));
	}
	status->set_text(vformat("Off screen, PXTGI can't read a ShaderMaterial's glow or colors, nor the colors of a material's albedo texture: it bounces light off them as gray (or the material's albedo color) and gets no light from their glow. "
							 "%d of the untagged materials glow, and %d have colors it can't read. The ticked ones get a pxtgi_emission tag of the glow shown (which you can edit: it should be what the material draws) and a pxtgi_albedo of the color shown. "
							 "Tags are measured once: measure again after changing a material's textures or glow.",
			glowing, albedos));
	get_ok_button()->set_disabled(false);
}

void PXTGIEmissionDialog::_apply() {
	EditorUndoRedoManager *ur = EditorUndoRedoManager::get_singleton();
	const StringName meta_emission = "pxtgi_emission";
	const StringName meta_albedo = "pxtgi_albedo";
	TreeItem *root = tree->get_root();
	// One action holds them all, in one history: a material saved in the scene belongs to the scene's, one in a file of
	// its own to the global one, and an action can't span both (Godot reports a history mismatch for each call). The
	// scene's is used when any tagged material is in it (tagging it changes the scene, to be saved), else the global.
	int history = EditorUndoRedoManager::INVALID_HISTORY;
	for (TreeItem *item = root ? root->get_first_child() : nullptr; item; item = item->get_next()) {
		const bool tagged = (item->get_cell_mode(COLUMN_TAG_GLOW) == TreeItem::CELL_MODE_CHECK && item->is_checked(COLUMN_TAG_GLOW)) || (item->get_cell_mode(COLUMN_TAG_ALBEDO) == TreeItem::CELL_MODE_CHECK && item->is_checked(COLUMN_TAG_ALBEDO));
		const Ref<Material> material = item->get_metadata(COLUMN_MATERIAL);
		if (!tagged || material.is_null()) {
			continue;
		}
		const int id = ur->get_history_id_for_object(material.ptr());
		if (history == EditorUndoRedoManager::INVALID_HISTORY || history == EditorUndoRedoManager::GLOBAL_HISTORY) {
			history = id;
		}
	}
	if (history == EditorUndoRedoManager::INVALID_HISTORY) {
		return;
	}
	ur->create_action_for_history("Tag Materials for PXTGI", history);
	ur->force_fixed_history();
	for (TreeItem *item = root ? root->get_first_child() : nullptr; item; item = item->get_next()) {
		const bool tag_glow = item->get_cell_mode(COLUMN_TAG_GLOW) == TreeItem::CELL_MODE_CHECK && item->is_checked(COLUMN_TAG_GLOW);
		const bool tag_albedo = item->get_cell_mode(COLUMN_TAG_ALBEDO) == TreeItem::CELL_MODE_CHECK && item->is_checked(COLUMN_TAG_ALBEDO);
		if (!tag_glow && !tag_albedo) {
			continue;
		}
		const Ref<Material> material = item->get_metadata(COLUMN_MATERIAL);
		if (material.is_null()) {
			continue;
		}
		if (tag_glow) {
			// The measured color, scaled to the brightness in the list (edited or not).
			const Color measured = item->get_metadata(COLUMN_GLOW);
			const float wanted = item->get_range(COLUMN_GLOW);
			const float measured_luminance = measured.get_luminance();
			const Color glow = measured_luminance > 1.0e-6f ? measured * (wanted / measured_luminance) : Color(wanted, wanted, wanted);
			ur->add_do_method(material.ptr(), "set_meta", meta_emission, PXTGIEmissionMeter::emission_tag(glow));
			if (material->has_meta(meta_emission)) {
				ur->add_undo_method(material.ptr(), "set_meta", meta_emission, material->get_meta(meta_emission));
			} else {
				ur->add_undo_method(material.ptr(), "remove_meta", meta_emission);
			}
		}
		if (tag_albedo) {
			ur->add_do_method(material.ptr(), "set_meta", meta_albedo, PXTGIEmissionMeter::albedo_tag(item->get_metadata(COLUMN_ALBEDO)));
			if (material->has_meta(meta_albedo)) {
				ur->add_undo_method(material.ptr(), "set_meta", meta_albedo, material->get_meta(meta_albedo));
			} else {
				ur->add_undo_method(material.ptr(), "remove_meta", meta_albedo);
			}
		}
		ur->add_do_method(this, "save_material", material);
		ur->add_undo_method(this, "save_material", material);
	}
	ur->commit_action();
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
	const Variant material = item->get_metadata(COLUMN_MATERIAL);
	Array meshes = glow_users.get(material, Array());
	if (meshes.is_empty()) {
		meshes = albedo_users.get(material, Array());
	}
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
	find->set_text("Measure Materials...");
	find->set_tooltip_text("PXTGI can't read a ShaderMaterial's glow or colors, nor a texture's colors: measure the scene's untagged materials and tag them with a pxtgi_emission and pxtgi_albedo of what they draw.");
	PXTGIEmissionDialog *target = dialog;
	find->connect(SceneStringName(pressed), callable_mp(target, &PXTGIEmissionDialog::popup_for_scene));
	add_custom_control(find);
}

/**************************************************************************/
/*  physx_shader_optimizer_dialog.cpp                                     */
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

#include "physx_shader_optimizer_dialog.h"

#include "../nodes/physx_shader_optimizer.h"

#include "core/io/resource_saver.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "editor/editor_interface.h"
#include "editor/editor_undo_redo_manager.h"
#include "scene/gui/box_container.h"
#include "scene/gui/label.h"
#include "scene/gui/tree.h"
#include "scene/resources/material.h"
#include "scene/resources/shader.h"

namespace {

enum Column {
	COLUMN_SHADER,
	COLUMN_SHADERS,
	COLUMN_MATERIALS,
	COLUMN_NOTES,
};

String file_label(const Ref<Resource> &p_resource) {
	const String path = p_resource->get_path();
	if (path.is_empty()) {
		return p_resource->get_name().is_empty() ? String("(built-in)") : p_resource->get_name();
	}
	return path.contains("::") ? path.get_slice("::", 0).get_file() + " (built-in)" : path.get_file();
}

} // namespace

PhysXShaderOptimizerDialog::PhysXShaderOptimizerDialog() {
	set_title("Optimize Shaders");
	set_ok_button_text("Merge Selected");
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
	tree->set_column_title(COLUMN_SHADER, "Shader");
	tree->set_column_title(COLUMN_SHADERS, "Shaders");
	tree->set_column_title(COLUMN_MATERIALS, "Materials");
	tree->set_column_title(COLUMN_NOTES, "Notes");
	tree->set_column_title_tooltip_text(COLUMN_SHADER, "Each group's first row is the shader kept; the rows under it, the shaders whose materials move onto it.");
	tree->set_column_title_tooltip_text(COLUMN_MATERIALS, "The material files moved onto the kept shader.");
	tree->set_column_title_tooltip_text(COLUMN_NOTES, "Whether the shaders are the same code, or differ only in their parameters' default values (the defaults a material relied on are set on it, so it looks the same), and any shader left as it is.");
	tree->set_column_expand(COLUMN_SHADER, true);
	tree->set_column_expand(COLUMN_SHADERS, false);
	tree->set_column_custom_minimum_width(COLUMN_SHADERS, 70);
	tree->set_column_expand(COLUMN_MATERIALS, false);
	tree->set_column_custom_minimum_width(COLUMN_MATERIALS, 80);
	tree->set_column_expand(COLUMN_NOTES, true);
	tree->set_hide_root(true);
	tree->set_v_size_flags(Control::SIZE_EXPAND_FILL);
	tree->set_custom_minimum_size(Size2(860, 300));
	tree->connect("item_activated", callable_mp(this, &PhysXShaderOptimizerDialog::_item_activated));
	box->add_child(tree);
	connect(SceneStringName(confirmed), callable_mp(this, &PhysXShaderOptimizerDialog::_apply));
}

void PhysXShaderOptimizerDialog::popup_for_project() {
	tree->clear();
	groups.clear();
	status->set_text("Reading the project's materials...");
	get_ok_button()->set_disabled(true);
	popup_centered_clamped(Size2(920, 560), 0.7);
	// After the dialog has drawn: reading every material file takes a while in a big project.
	callable_mp(this, &PhysXShaderOptimizerDialog::_scan).call_deferred();
}

void PhysXShaderOptimizerDialog::_scan() {
	const TypedArray<ShaderMaterial> materials = PhysXShaderOptimizer::find_materials("res://");
	groups = PhysXShaderOptimizer::find_groups(materials);
	HashSet<ObjectID> used;
	for (int i = 0; i < materials.size(); i++) {
		const Ref<ShaderMaterial> material = materials[i];
		if (material.is_valid() && material->get_shader().is_valid()) {
			used.insert(material->get_shader()->get_instance_id());
		}
	}

	tree->clear();
	TreeItem *root = tree->create_item();
	int merged = 0;
	for (int g = 0; g < groups.size(); g++) {
		const Dictionary group = groups[g];
		const Ref<Shader> kept = group["shader"];
		const Array shaders = group["shaders"];
		const Array group_materials = group["materials"];
		const Array parameters = group["parameters"];
		const Array skipped = group["skipped"];
		TreeItem *item = tree->create_item(root);
		item->set_metadata(COLUMN_SHADER, g);
		item->set_cell_mode(COLUMN_SHADER, TreeItem::CELL_MODE_CHECK);
		item->set_text(COLUMN_SHADER, kept.is_valid() ? file_label(kept) : file_label(Ref<Resource>(shaders[0])));
		item->set_editable(COLUMN_SHADER, kept.is_valid());
		item->set_checked(COLUMN_SHADER, kept.is_valid());
		item->set_text(COLUMN_SHADERS, itos(shaders.size()));
		item->set_text(COLUMN_MATERIALS, itos(group_materials.size()));
		String notes;
		if (kept.is_null()) {
			notes = "None of these shaders is in its own file, so none can be shared: save one as a .tres to merge them.";
		} else {
			int values = 0;
			for (int i = 0; i < parameters.size(); i++) {
				values += Dictionary(parameters[i]).size();
			}
			notes = bool(group["identical"]) ? String("The same code.") : String("Differ only in parameter defaults.");
			if (values > 0) {
				notes += vformat(" %d default value%s set on materials.", values, values == 1 ? "" : "s");
			}
			if (!skipped.is_empty()) {
				notes += vformat(" %d left as they are (a default texture the kept shader has and they don't).", skipped.size());
			}
			merged += shaders.size() - 1 - skipped.size();
		}
		item->set_text(COLUMN_NOTES, notes);
		item->set_tooltip_text(COLUMN_NOTES, notes);
		item->set_collapsed(true);
		// The shaders whose materials move: each with how many.
		for (int s = 0; s < shaders.size(); s++) {
			const Ref<Shader> shader = shaders[s];
			if (shader == kept) {
				continue;
			}
			int count = 0;
			for (int i = 0; i < group_materials.size(); i++) {
				const Ref<ShaderMaterial> material = group_materials[i];
				count += material.is_valid() && material->get_shader() == shader ? 1 : 0;
			}
			TreeItem *child = tree->create_item(item);
			child->set_text(COLUMN_SHADER, file_label(shader));
			child->set_metadata(COLUMN_SHADER, shader);
			child->set_text(COLUMN_MATERIALS, itos(count));
			if (skipped.has(shader)) {
				child->set_text(COLUMN_NOTES, "Left as it is.");
			}
		}
	}
	if (groups.is_empty()) {
		status->set_text(vformat("%d materials use %d shaders, and none could be merged.", materials.size(), used.size()));
	} else {
		status->set_text(vformat("%d materials use %d shaders; merging every group leaves %d. Each shader Godot compiles costs system memory (tens of MB for a big one) and time to compile and to free, and two shaders with the same code are compiled twice. The materials look the same afterward; they're saved, so commit your project first. Double-click a shader to find it in the FileSystem dock.", materials.size(), used.size(), used.size() - merged));
	}
	get_ok_button()->set_disabled(groups.is_empty());
}

void PhysXShaderOptimizerDialog::_apply() {
	EditorUndoRedoManager *ur = EditorUndoRedoManager::get_singleton();
	int count = 0;
	TreeItem *root = tree->get_root();
	for (TreeItem *item = root ? root->get_first_child() : nullptr; item; item = item->get_next()) {
		if (!item->is_checked(COLUMN_SHADER)) {
			continue;
		}
		const Dictionary group = groups[int(item->get_metadata(COLUMN_SHADER))];
		const Ref<Shader> kept = group["shader"];
		if (kept.is_null()) {
			continue;
		}
		const Array materials = group["materials"];
		const Array parameters = group["parameters"];
		for (int i = 0; i < materials.size(); i++) {
			const Ref<ShaderMaterial> material = materials[i];
			if (material.is_null()) {
				continue;
			}
			if (count == 0) {
				ur->create_action("Merge Shaders");
			}
			count++;
			ur->add_do_method(material.ptr(), "set_shader", kept);
			ur->add_undo_method(material.ptr(), "set_shader", material->get_shader());
			const Dictionary set = parameters[i];
			for (const KeyValue<Variant, Variant> &E : set) {
				ur->add_do_method(material.ptr(), "set_shader_parameter", E.key, E.value);
				ur->add_undo_method(material.ptr(), "set_shader_parameter", E.key, material->get_shader_parameter(E.key));
			}
			ur->add_do_method(this, "save_material", material);
			ur->add_undo_method(this, "save_material", material);
		}
	}
	if (count > 0) {
		ur->commit_action();
	}
}

void PhysXShaderOptimizerDialog::save_material(const Ref<Resource> &p_material) {
	if (p_material.is_valid() && !p_material->get_path().is_empty() && !p_material->get_path().contains("::")) {
		ResourceSaver::save(p_material);
	}
}

void PhysXShaderOptimizerDialog::_item_activated() {
	TreeItem *item = tree->get_selected();
	if (item == nullptr) {
		return;
	}
	Ref<Resource> shader = item->get_metadata(COLUMN_SHADER);
	if (shader.is_null() && item->get_parent() != nullptr && item->get_parent() != tree->get_root()) {
		return;
	}
	if (shader.is_null()) {
		const Dictionary group = groups[int(item->get_metadata(COLUMN_SHADER))];
		shader = group["shader"];
	}
	if (shader.is_valid() && !shader->get_path().is_empty()) {
		EditorInterface::get_singleton()->select_file(shader->get_path().get_slice("::", 0));
	}
}

void PhysXShaderOptimizerDialog::_bind_methods() {
	ClassDB::bind_method(D_METHOD("save_material", "material"), &PhysXShaderOptimizerDialog::save_material);
}

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

#include "core/config/project_settings.h"
#include "core/io/file_access.h"
#include "core/io/resource_loader.h"
#include "core/io/resource_saver.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "core/os/os.h"
#include "editor/editor_interface.h"
#include "editor/editor_node.h"
#include "editor/editor_undo_redo_manager.h"
#include "editor/file_system/editor_file_system.h"
#include "scene/gui/box_container.h"
#include "scene/gui/check_box.h"
#include "scene/gui/item_list.h"
#include "scene/gui/label.h"
#include "scene/gui/tree.h"
#include "scene/main/scene_tree.h"
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

// The shaders of p_candidates that a file other than the candidates themselves and the materials just saved
// (p_ignore: the editor's record of them may predate the save) depends on.
void find_used(EditorFileSystemDirectory *p_dir, const HashSet<String> &p_candidates, const HashSet<String> &p_ignore, HashSet<String> &r_used) {
	if (p_dir == nullptr) {
		return;
	}
	for (int i = 0; i < p_dir->get_subdir_count(); i++) {
		find_used(p_dir->get_subdir(i), p_candidates, p_ignore, r_used);
	}
	for (int i = 0; i < p_dir->get_file_count(); i++) {
		const String path = p_dir->get_file_path(i);
		if (p_candidates.has(path) || p_ignore.has(path)) {
			continue;
		}
		for (const String &dep : p_dir->get_file_deps(i)) {
			if (p_candidates.has(dep)) {
				r_used.insert(dep);
			}
		}
	}
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
	delete_unused = memnew(CheckBox);
	delete_unused->set_text("Delete the shaders left unused");
	delete_unused->set_tooltip_text("After merging, list the shaders the merge moved materials away from that nothing else in the project uses, to move them to the system trash (asked first).");
	box->add_child(delete_unused);
	connect(SceneStringName(confirmed), callable_mp(this, &PhysXShaderOptimizerDialog::_apply));

	delete_confirm = memnew(ConfirmationDialog);
	delete_confirm->set_title("Delete Unused Shaders");
	delete_confirm->set_ok_button_text("Delete");
	delete_confirm->set_cancel_button_text("Keep Unused");
	VBoxContainer *delete_box = memnew(VBoxContainer);
	delete_confirm->add_child(delete_box);
	delete_label = memnew(Label);
	delete_label->set_autowrap_mode(TextServer::AUTOWRAP_WORD_SMART);
	// A real width: wrapping at 1 pixel, the dialog would size itself to that text's height (screen tall).
	delete_label->set_custom_minimum_size(Size2(640, 0));
	delete_box->add_child(delete_label);
	delete_list = memnew(ItemList);
	delete_list->set_v_size_flags(Control::SIZE_EXPAND_FILL);
	delete_list->set_custom_minimum_size(Size2(640, 260));
	delete_box->add_child(delete_list);
	delete_confirm->connect(SceneStringName(confirmed), callable_mp(this, &PhysXShaderOptimizerDialog::_delete_unused));
	// Beside this dialog, not in it: it opens once this one has closed, and a window inside a hidden one isn't drawn.
	EditorNode::get_singleton()->get_gui_base()->add_child(delete_confirm);
}

void PhysXShaderOptimizerDialog::popup_for_project() {
	tree->clear();
	groups.clear();
	status->set_text("Reading the project's materials...");
	get_ok_button()->set_disabled(true);
	popup_centered_clamped(Size2(920, 560), 0.7);
	// After the dialog has drawn: reading every material file takes a while in a big project. On the next frame, not
	// deferred: the editor's progress window can't be opened from a deferred call.
	get_tree()->connect(SNAME("process_frame"), callable_mp(this, &PhysXShaderOptimizerDialog::_scan), CONNECT_ONE_SHOT);
}

void PhysXShaderOptimizerDialog::_scan() {
	// Reading the materials (their textures with them) and the shaders' code (a VisualShader generates its own) takes a
	// while in a big project: shown, and can be canceled.
	const PackedStringArray paths = PhysXShaderOptimizer::find_material_paths("res://");
	TypedArray<ShaderMaterial> materials;
	{
		EditorProgress progress("physx_optimize_shaders", "Reading Materials", paths.size(), true);
		for (int i = 0; i < paths.size(); i++) {
			if (progress.step(paths[i].get_file(), i)) {
				hide();
				return;
			}
			Ref<ShaderMaterial> material = ResourceLoader::load(paths[i]);
			if (material.is_valid()) {
				materials.push_back(material);
			}
		}
	}
	{
		LocalVector<Ref<Shader>> shaders;
		HashSet<ObjectID> seen;
		for (int i = 0; i < materials.size(); i++) {
			const Ref<ShaderMaterial> material = materials[i];
			if (material->get_shader().is_valid() && !seen.has(material->get_shader()->get_instance_id())) {
				seen.insert(material->get_shader()->get_instance_id());
				shaders.push_back(material->get_shader());
			}
		}
		EditorProgress progress("physx_optimize_shaders_code", "Reading Shaders", shaders.size(), true);
		for (uint32_t i = 0; i < shaders.size(); i++) {
			if (progress.step(shaders[i]->get_path().get_file(), i)) {
				hide();
				return;
			}
			(void)shaders[i]->get_code(); // a VisualShader's generated now, kept for find_groups
			List<PropertyInfo> params;
			shaders[i]->get_shader_uniform_list(&params);
		}
	}
	groups = PhysXShaderOptimizer::find_groups(materials);
	HashSet<ObjectID> used;
	// The shaders that differ only in textures built into their graphs (texture nodes, not parameters): not merged, as
	// a material can't be given its own for those; counted so the user knows making them parameters would help.
	HashMap<String, HashSet<String>> textures_by_code;
	for (int i = 0; i < materials.size(); i++) {
		const Ref<ShaderMaterial> material = materials[i];
		if (material.is_valid() && material->get_shader().is_valid() && !used.has(material->get_shader()->get_instance_id())) {
			const Ref<Shader> shader = material->get_shader();
			used.insert(shader->get_instance_id());
			const String code = PhysXShaderOptimizer::code_without_defaults(shader->get_code());
			if (!textures_by_code.has(code)) {
				textures_by_code.insert(code, HashSet<String>());
			}
			textures_by_code[code].insert(PhysXShaderOptimizer::default_textures_key(shader));
		}
	}
	int texture_only = 0;
	for (const KeyValue<String, HashSet<String>> &E : textures_by_code) {
		texture_only += E.value.size() - 1;
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
				notes += vformat(" %d left as they are (a texture parameter with no texture where the kept shader has one, which a material can't hold).", skipped.size());
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
	String text;
	if (groups.is_empty()) {
		text = vformat("%d materials use %d shaders, and none could be merged.", materials.size(), used.size());
	} else {
		text = vformat("%d materials use %d shaders; merging every group leaves %d.\nThe shaders in each group are duplicates, but Godot compiles and keeps every copy, which costs memory and loading time. Merging points their materials at one shared shader; nothing looks different.\nChanges are written to disk, so back up your project or make sure it's tracked in version control (such as git) first. Double-click a shader to find it in the FileSystem dock.", materials.size(), used.size(), used.size() - merged);
	}
	if (texture_only > 0) {
		text += vformat("\n%d more shader%s from others only in the textures picked inside their graphs (Texture nodes). Those can't be merged, as a material can't be given its own texture for them; changing them to Texture2D Parameter nodes would let them merge.", texture_only, texture_only == 1 ? " differs" : "s differ");
	}
	status->set_text(text);
	get_ok_button()->set_disabled(groups.is_empty());
}

void PhysXShaderOptimizerDialog::_apply() {
	EditorUndoRedoManager *ur = EditorUndoRedoManager::get_singleton();
	int count = 0;
	PackedStringArray left; // the shaders materials were moved away from
	PackedStringArray moved; // the materials moved
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
		const Array shaders = group["shaders"];
		const Array skipped = group["skipped"];
		for (int i = 0; i < shaders.size(); i++) {
			const Ref<Shader> shader = shaders[i];
			if (shader.is_valid() && shader != kept && !skipped.has(shader) && !shader->get_path().is_empty() && !shader->get_path().contains("::")) {
				left.push_back(shader->get_path());
			}
		}
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
			moved.push_back(material->get_path());
		}
	}
	if (count > 0) {
		ur->commit_action();
	}
	if (count > 0 && delete_unused->is_pressed() && !left.is_empty()) {
		// After this dialog has closed.
		callable_mp(this, &PhysXShaderOptimizerDialog::_offer_delete).call_deferred(left, moved);
	}
}

void PhysXShaderOptimizerDialog::_offer_delete(const PackedStringArray &p_left, const PackedStringArray &p_moved) {
	HashSet<String> left;
	for (const String &path : p_left) {
		left.insert(path);
	}
	HashSet<String> moved;
	for (const String &path : p_moved) {
		moved.insert(path);
	}
	HashSet<String> used;
	find_used(EditorFileSystem::get_singleton()->get_filesystem(), left, moved, used);
	LocalVector<String> unused;
	for (const String &path : left) {
		if (!used.has(path)) {
			unused.push_back(path);
		}
	}
	unused.sort();
	delete_list->clear();
	for (const String &path : unused) {
		delete_list->add_item(path);
	}
	if (unused.is_empty()) {
		delete_label->set_text("Every shader the merge moved materials away from is still used by another file: nothing to delete.");
		delete_confirm->get_ok_button()->set_disabled(true);
	} else {
		String text = vformat("These %d shaders are no longer used by any file in the project. Delete moves them to the system trash; the merge can't be undone afterward without restoring them from there.", int(unused.size()));
		if (!used.is_empty()) {
			text += vformat(" (%d others are still used by other files and are kept.)", used.size());
		}
		delete_label->set_text(text);
		delete_confirm->get_ok_button()->set_disabled(false);
	}
	delete_confirm->popup_centered_clamped(Size2(680, 420), 0.7);
}

void PhysXShaderOptimizerDialog::_delete_unused() {
	PackedStringArray removed;
	for (int i = 0; i < delete_list->get_item_count(); i++) {
		const String file = delete_list->get_item_text(i);
		// As the FileSystem dock deletes: a loaded copy forgets its path, the file and its .uid go to the trash.
		if (ResourceCache::has(file)) {
			Ref<Resource> res = ResourceCache::get_ref(file);
			res->set_path("");
		}
		const String path = ProjectSettings::get_singleton()->globalize_path(file);
		if (OS::get_singleton()->move_to_trash(path) != OK) {
			EditorNode::get_singleton()->add_io_error("Cannot remove:\n" + file + "\n");
			continue;
		}
		if (FileAccess::exists(file + ".uid")) {
			OS::get_singleton()->move_to_trash(path + ".uid");
		}
		removed.push_back(file);
	}
	for (const String &file : removed) {
		EditorFileSystem::get_singleton()->update_file(file);
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

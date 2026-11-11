/**************************************************************************/
/*  physx_blast_asset_inspector_plugin.cpp                                */
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

#include "physx_blast_asset_inspector_plugin.h"

#ifdef GODOT_PHYSX_BLAST

#include "../blast/physx_blast_asset.h"
#include "physx_blast_preview.h"

#include "core/object/callable_mp.h"
#include "editor/editor_string_names.h"
#include "editor/editor_undo_redo_manager.h"
#include "editor/scene/3d/mesh_editor_plugin.h"
#include "scene/gui/button.h"
#include "scene/gui/label.h"

PhysXBlastHullControl::PhysXBlastHullControl() {
	status = memnew(Label);
	status->set_autowrap_mode(TextServer::AUTOWRAP_WORD_SMART);
	status->set_custom_minimum_size(Size2(1, 0)); // wraps to the inspector's width
	add_child(status);
	cook = memnew(Button);
	cook->set_text("Cook Hulls");
	cook->set_tooltip_text("Cook every chunk's collision hull with this build and store it in the asset, so a break doesn't cook it. The pieces and how they break stay the same.");
	cook->connect(SceneStringName(pressed), callable_mp(this, &PhysXBlastHullControl::_cook));
	add_child(cook);
}

void PhysXBlastHullControl::edit(const Ref<PhysXBlastAsset> &p_asset) {
	const Callable update = callable_mp(this, &PhysXBlastHullControl::_update);
	if (asset.is_valid() && asset->is_connected(CoreStringName(changed), update)) {
		asset->disconnect(CoreStringName(changed), update);
	}
	asset = p_asset;
	if (asset.is_valid()) {
		asset->connect(CoreStringName(changed), update);
	}
	_update();
}

void PhysXBlastHullControl::_notification(int p_what) {
	if (p_what == NOTIFICATION_THEME_CHANGED) {
		_update();
	}
}

void PhysXBlastHullControl::_update() {
	if (asset.is_null()) {
		return;
	}
	const Array hulls = asset->get_chunk_hulls();
	switch (asset->get_hull_status()) {
		case PhysXBlastAsset::HULLS_NONE:
			status->set_text("Collision hulls: not cooked. Each piece's first break cooks its hull on the main thread.");
			status->add_theme_color_override(SceneStringName(font_color), get_theme_color(SNAME("warning_color"), EditorStringName(Editor)));
			cook->set_text("Cook Hulls");
			break;
		case PhysXBlastAsset::HULLS_OUT_OF_DATE:
			status->set_text(vformat("Collision hulls: out of date (cooked by %s). They're cooked at runtime until they're cooked again.", asset->get_hull_format().is_empty() ? String("another build") : asset->get_hull_format()));
			status->add_theme_color_override(SceneStringName(font_color), get_theme_color(SNAME("warning_color"), EditorStringName(Editor)));
			cook->set_text("Cook Hulls");
			break;
		case PhysXBlastAsset::HULLS_COOKED: {
			int64_t bytes = 0;
			for (int i = 0; i < hulls.size(); i++) {
				bytes += PackedByteArray(hulls[i]).size();
			}
			status->set_text(vformat("Collision hulls: cooked (%d chunks, %s, %s).", hulls.size(), String::humanize_size(bytes), asset->get_hull_format()));
			status->remove_theme_color_override(SceneStringName(font_color));
			cook->set_text("Cook Hulls Again");
		} break;
	}
}

void PhysXBlastHullControl::_cook() {
	if (asset.is_null()) {
		return;
	}
	const Array old_hulls = asset->get_chunk_hulls();
	const String old_format = asset->get_hull_format();
	if (asset->cook_hulls() != OK) {
		return;
	}
	// Already done; recorded so it can be undone, and so the asset counts as changed and saves.
	EditorUndoRedoManager *ur = EditorUndoRedoManager::get_singleton();
	ur->create_action("Cook Blast Hulls", UndoRedo::MERGE_DISABLE, asset.ptr());
	ur->add_do_method(asset.ptr(), "set_chunk_hulls", asset->get_chunk_hulls());
	ur->add_do_method(asset.ptr(), "set_hull_format", asset->get_hull_format());
	ur->add_do_method(asset.ptr(), "emit_changed");
	ur->add_undo_method(asset.ptr(), "set_chunk_hulls", old_hulls);
	ur->add_undo_method(asset.ptr(), "set_hull_format", old_format);
	ur->add_undo_method(asset.ptr(), "emit_changed");
	ur->commit_action(false);
}

bool EditorInspectorPluginPhysXBlastAsset::can_handle(Object *p_object) {
	return Object::cast_to<PhysXBlastAsset>(p_object) != nullptr;
}

void EditorInspectorPluginPhysXBlastAsset::parse_begin(Object *p_object) {
	PhysXBlastAsset *asset = Object::cast_to<PhysXBlastAsset>(p_object);
	if (!asset) {
		return;
	}
	Ref<PhysXBlastAsset> a(asset);

	MeshEditor *editor = memnew(MeshEditor);
	editor->edit(physx_blast_build_preview_mesh(a));
	add_custom_control(editor);

	PhysXBlastHullControl *hulls = memnew(PhysXBlastHullControl);
	hulls->edit(a);
	add_custom_control(hulls);
}

#endif // GODOT_PHYSX_BLAST

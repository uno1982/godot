/**************************************************************************/
/*  physx_shader_optimizer_dialog.h                                       */
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

#pragma once

#include "scene/gui/dialogs.h"

class CheckBox;
class ItemList;
class Label;
class Tree;

// Optimize Shaders (Project > Tools): the project's material files' shaders that could be one shader
// (PhysXShaderOptimizer), listed by group; the ticked groups' materials are moved onto the group's kept shader, as one
// undoable action, and saved. With "Delete the shaders left unused" ticked, the shaders the merge moved materials away
// from that nothing else in the project uses are then listed, and moved to the system trash if confirmed.
class PhysXShaderOptimizerDialog : public ConfirmationDialog {
	GDCLASS(PhysXShaderOptimizerDialog, ConfirmationDialog);

	Label *status = nullptr;
	Tree *tree = nullptr;
	CheckBox *delete_unused = nullptr;
	ConfirmationDialog *delete_confirm = nullptr;
	Label *delete_label = nullptr;
	ItemList *delete_list = nullptr;
	Array groups;

	void _scan();
	void _apply();
	void _item_activated();
	void _offer_delete(const PackedStringArray &p_left, const PackedStringArray &p_moved);
	void _delete_unused();

protected:
	static void _bind_methods();

public:
	void save_material(const Ref<Resource> &p_material);
	void popup_for_project();
	PhysXShaderOptimizerDialog();
};

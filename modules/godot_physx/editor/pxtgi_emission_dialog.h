/**************************************************************************/
/*  pxtgi_emission_dialog.h                                               */
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

#include "../nodes/pxtgi_emission_meter.h"

#include "editor/inspector/editor_inspector.h"
#include "scene/gui/dialogs.h"

class Button;
class Label;
class Tree;

// Find Emissive Materials: the edited scene's ShaderMaterials that PXTGI can't read a glow from, measured
// (PXTGIEmissionMeter) and listed; the ones ticked get a pxtgi_emission tag of their measured (or edited) glow, as
// one undoable action, and those in their own files are saved.
class PXTGIEmissionDialog : public ConfirmationDialog {
	GDCLASS(PXTGIEmissionDialog, ConfirmationDialog);

	Label *status = nullptr;
	Tree *tree = nullptr;
	Button *rescan = nullptr;
	Ref<PXTGIEmissionMeter> meter;
	Dictionary users; // material -> the MeshInstance3Ds using it

	void _scan();
	void _measured(const Array &p_results);
	void _apply();
	void _item_activated();

protected:
	static void _bind_methods();

public:
	// Saves a material that has its own file (a .tres or .res); one inside a scene goes with that scene.
	void save_material(const Ref<Material> &p_material);
	void popup_for_scene();
	PXTGIEmissionDialog();
};

// The Find Emissive Materials button on a PXTGI3D's inspector.
class EditorInspectorPluginPXTGI : public EditorInspectorPlugin {
	GDCLASS(EditorInspectorPluginPXTGI, EditorInspectorPlugin);

	PXTGIEmissionDialog *dialog = nullptr;

public:
	virtual bool can_handle(Object *p_object) override;
	virtual void parse_begin(Object *p_object) override;
	void set_dialog(PXTGIEmissionDialog *p_dialog) { dialog = p_dialog; }
};

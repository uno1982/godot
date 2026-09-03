/**************************************************************************/
/*  register_types.cpp                                                    */
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

#include "register_types.h"

#include "godot_physx_project_settings.h"
#include "godot_physx_server_3d.h"
#include "nodes/physx_cloth_3d.h"
#include "nodes/physx_particle_fluid_3d.h"

#include "core/config/project_settings.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "servers/physics_3d/physics_server_3d_wrap_mt.h"

#ifdef TOOLS_ENABLED
#include "editor/physx_editor_plugin.h"
#include "editor/plugins/editor_plugin.h"
#endif

static PhysicsServer3D *create_physx_physics_server() {
#ifdef THREADS_ENABLED
	bool run_on_separate_thread = GLOBAL_GET("physics/3d/run_on_separate_thread");
#else
	bool run_on_separate_thread = false;
#endif

	GodotPhysXServer3D *physics_server = memnew(GodotPhysXServer3D);

	return memnew(PhysicsServer3DWrapMT(physics_server, run_on_separate_thread));
}

void initialize_godot_physx_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SERVERS) {
		GodotPhysXProjectSettings::register_settings();
		PhysicsServer3DManager::get_singleton()->register_server("PhysX", callable_mp_static(&create_physx_physics_server));
	}

	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(PhysXParticleFluid3D);
		GDREGISTER_CLASS(PhysXCloth3D);
	}

#ifdef TOOLS_ENABLED
	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		EditorPlugins::add_by_type<PhysXEditorPlugin>();
	}
#endif
}

void uninitialize_godot_physx_module(ModuleInitializationLevel p_level) {
}

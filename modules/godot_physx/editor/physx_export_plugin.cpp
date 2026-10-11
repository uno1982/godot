/**************************************************************************/
/*  physx_export_plugin.cpp                                               */
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

#include "physx_export_plugin.h"

#include "physx_license.gen.h"

#include "core/io/dir_access.h"
#include "core/io/file_access.h"
#include "core/version.h"
#include "editor/export/editor_export_platform.h"
#include "editor/file_system/editor_paths.h"

namespace {

struct ModuleLibrary {
	const char *windows;
	const char *linuxbsd;
	// What the game loses without it -- shown in the export warning.
	const char *missing;
};

} //namespace

void PhysXExportPlugin::_export_begin(const HashSet<String> &p_features, bool p_debug, const String &p_path, int p_flags) {
	const bool windows = p_features.has("windows");
	const bool linuxbsd = p_features.has("linux");
	if (!windows && !linuxbsd) {
		return; // The module's libraries are only built for Windows and Linux.
	}

	LocalVector<ModuleLibrary> libraries;
#ifdef GODOT_PHYSX_BLAST
	const char *blast_missing = "the exported game will not start (Blast is linked at load time)";
	libraries.push_back({ "NvBlast.dll", "libNvBlast.so", blast_missing });
	libraries.push_back({ "NvBlastGlobals.dll", "libNvBlastGlobals.so", blast_missing });
	libraries.push_back({ "NvBlastExtAuthoring.dll", "libNvBlastExtAuthoring.so", blast_missing });
	libraries.push_back({ "NvBlastExtShaders.dll", "libNvBlastExtShaders.so", blast_missing });
#endif
#ifdef GODOT_PHYSX_GPU
	libraries.push_back({ "PhysXGpu_64.dll", "libPhysXGpu_64.so", "the game will run PhysX on the CPU, without GPU dynamics" });
#endif
#ifdef GODOT_PHYSX_FLOW
	libraries.push_back({ "nvflow.dll", "libnvflow.so", "NVIDIA Flow (PhysXFlow3D) will stay inert" });
	libraries.push_back({ "nvflowext.dll", "libnvflowext.so", "NVIDIA Flow (PhysXFlow3D) will stay inert" });
#endif

	// The folder of the export template this export uses: the libraries must
	// come from the same build as the template, for the target platform.
	String template_dir;
	const Ref<EditorExportPreset> preset = get_export_preset();
	if (preset.is_valid()) {
		const String custom = preset->get(p_debug ? "custom_template/debug" : "custom_template/release");
		if (!custom.is_empty()) {
			template_dir = custom.get_base_dir();
		}
	}
	if (template_dir.is_empty()) {
		template_dir = EditorPaths::get_singleton()->get_export_templates_dir().path_join(GODOT_VERSION_FULL_CONFIG);
	}

	// One warning per feature (Blast has four libraries), listing what's missing.
	const Ref<EditorExportPlatform> platform = get_export_platform();
	LocalVector<String> missing_effects;
	LocalVector<PackedStringArray> missing_files;
	for (const ModuleLibrary &library : libraries) {
		const String path = template_dir.path_join(windows ? library.windows : library.linuxbsd);
		if (FileAccess::exists(path)) {
			add_shared_object(path, Vector<String>(), String());
			continue;
		}
		const String effect = library.missing;
		int64_t i = missing_effects.find(effect);
		if (i < 0) {
			missing_effects.push_back(effect);
			missing_files.push_back(PackedStringArray());
			i = missing_effects.size() - 1;
		}
		missing_files[i].push_back(path.get_file());
	}
	if (platform.is_valid()) {
		for (uint32_t i = 0; i < missing_effects.size(); i++) {
			platform->add_message(EditorExportPlatform::EXPORT_MESSAGE_WARNING, "PhysX",
					vformat("Not found next to the export template, so not exported: %s. Without %s, %s. Copy %s into %s from the build that made the template (see the PhysX module README, \"Exporting a game\").",
							String(", ").join(missing_files[i]), missing_files[i].size() > 1 ? "them" : "it", missing_effects[i], missing_files[i].size() > 1 ? "them" : "it", template_dir));
		}
	}

	// The PhysX SDK is compiled into the template and NvBlast / PhysXGpu /
	// Flow ship as binaries: BSD-3-Clause requires the notice to go with them.
	const String license_path = EditorPaths::get_singleton()->get_cache_dir().path_join("godot_physx").path_join("PHYSX-LICENSE.txt");
	DirAccess::make_dir_recursive_absolute(license_path.get_base_dir());
	Ref<FileAccess> f = FileAccess::open(license_path, FileAccess::WRITE);
	if (f.is_valid()) {
		f->store_string(String::utf8(GODOT_PHYSX_LICENSE_TEXT));
		f.unref();
		add_shared_object(license_path, Vector<String>(), String());
	}
}

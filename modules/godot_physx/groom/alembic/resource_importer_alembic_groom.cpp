/**************************************************************************/
/*  resource_importer_alembic_groom.cpp                                   */
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

#include "resource_importer_alembic_groom.h"

#include "../physx_groom.h"
#include "alembic_groom_reader.h"

#include "core/config/project_settings.h"
#include "core/io/resource_saver.h"
#include "core/math/random_pcg.h"

// The strands in a random order, the same every import: PhysXGroom3D draws fewer of them at a distance by drawing the
// first ones, which are then spread over the whole groom (Indiana Jones and the Great Circle's hair LOD, after
// Taillandier 2020).
static void _shuffle_strands(AlembicGroomData &r_data) {
	const int count = r_data.strand_offsets.size() - 1;
	LocalVector<int> order;
	order.resize(count);
	for (int i = 0; i < count; i++) {
		order[i] = i;
	}
	RandomPCG rng(0x9e3779b9);
	for (int i = count - 1; i > 0; i--) {
		SWAP(order[i], order[rng.rand(i + 1)]);
	}
	PackedVector3Array points;
	PackedFloat32Array widths;
	PackedInt32Array offsets;
	points.resize(r_data.points.size());
	widths.resize(r_data.widths.size());
	offsets.resize(count + 1);
	int at = 0;
	offsets.write[0] = 0;
	for (int i = 0; i < count; i++) {
		const int from = r_data.strand_offsets[order[i]];
		const int to = r_data.strand_offsets[order[i] + 1];
		for (int p = from; p < to; p++, at++) {
			points.write[at] = r_data.points[p];
			widths.write[at] = r_data.widths[p];
		}
		offsets.write[i + 1] = at;
	}
	r_data.points = points;
	r_data.widths = widths;
	r_data.strand_offsets = offsets;
}

void ResourceImporterAlembicGroom::get_import_options(const String &p_path, List<ImportOption> *r_options, int p_preset) const {
	// Blender writes meters; a file in centimeters (Maya, Houdini set up for it) wants 0.01.
	r_options->push_back(ImportOption(PropertyInfo(Variant::FLOAT, "scale", PROPERTY_HINT_RANGE, "0.0001,100,0.0001,or_greater"), 1.0));
	// For strands the file gives no width: about a human hair's.
	r_options->push_back(ImportOption(PropertyInfo(Variant::FLOAT, "default_width", PROPERTY_HINT_RANGE, "0.00001,0.01,0.00001,suffix:m"), 0.00008));
}

Error ResourceImporterAlembicGroom::import(ResourceUID::ID p_source_id, const String &p_source_file, const String &p_save_path, const HashMap<StringName, Variant> &p_options, List<String> *r_platform_variants, List<String> *r_gen_files, Variant *r_metadata) {
	AlembicGroomData data;
	alembic_read_groom(ProjectSettings::get_singleton()->globalize_path(p_source_file), p_options["scale"], p_options["default_width"], data);
	for (const String &warning : data.warnings) {
		WARN_PRINT(vformat("%s: %s", p_source_file, warning));
	}
	ERR_FAIL_COND_V_MSG(!data.error.is_empty(), ERR_FILE_CORRUPT, vformat("Can't import '%s' as a groom: %s", p_source_file, data.error));
	_shuffle_strands(data);

	Ref<PhysXGroom> groom;
	groom.instantiate();
	groom->set_points(data.points);
	groom->set_strand_offsets(data.strand_offsets);
	groom->set_widths(data.widths);
	return ResourceSaver::save(groom, p_save_path + ".res");
}

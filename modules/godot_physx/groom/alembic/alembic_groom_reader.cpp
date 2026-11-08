/**************************************************************************/
/*  alembic_groom_reader.cpp                                              */
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

// The Alembic library's headers before Godot's: Godot's macros (MIN, MAX, ...) would otherwise reach into them.
#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcGeom/All.h>

#include <exception>

#include "alembic_groom_reader.h"

namespace {

namespace AbcGeom = Alembic::AbcGeom;

struct GroomWalk {
	float scale = 1.0f;
	float default_width = 0.0f;
	AlembicGroomData *data = nullptr;
	bool warned_basis = false;
	bool warned_widths = false;
	int closed_skipped = 0;

	void walk(const Alembic::Abc::IObject &p_object, const Imath::M44d &p_parent) {
		Imath::M44d world = p_parent;
		const Alembic::Abc::MetaData &md = p_object.getMetaData();
		if (AbcGeom::IXform::matches(md)) {
			AbcGeom::IXform xform(p_object, Alembic::Abc::kWrapExisting);
			AbcGeom::XformSample sample;
			xform.getSchema().get(sample);
			// Imath multiplies row vectors: the child's matrix first, then its parent's.
			world = sample.getMatrix() * p_parent;
		} else if (AbcGeom::ICurves::matches(md)) {
			read_curves(AbcGeom::ICurves(p_object, Alembic::Abc::kWrapExisting), world);
		}
		for (size_t i = 0; i < p_object.getNumChildren(); i++) {
			walk(p_object.getChild(i), world);
		}
	}

	void read_curves(const AbcGeom::ICurves &p_curves, const Imath::M44d &p_world) {
		const AbcGeom::ICurvesSchema &schema = p_curves.getSchema();
		const AbcGeom::ICurvesSchema::Sample sample = schema.getValue();
		const Alembic::Abc::P3fArraySamplePtr positions = sample.getPositions();
		const Alembic::Abc::Int32ArraySamplePtr counts = sample.getCurvesNumVertices();
		if (!positions || !counts) {
			return;
		}
		// Hair is never a closed loop: closed curves are something else in the groom (Blender's braids demo ties
		// its braids with rings), and drawn as strands they'd be loops of hair.
		if (sample.getWrap() == AbcGeom::kPeriodic) {
			closed_skipped += (int)counts->size();
			return;
		}
		// A cubic Bezier or B-spline curve doesn't pass through its points: drawn through them, it's close, not exact.
		// Catmull-Rom does, and linear is them.
		if (!warned_basis && sample.getType() != AbcGeom::kLinear && (sample.getBasis() == AbcGeom::kBezierBasis || sample.getBasis() == AbcGeom::kBsplineBasis)) {
			data->warnings.push_back(String("'") + p_curves.getName().c_str() + "': its curves are Bezier or B-spline, drawn through their control points.");
			warned_basis = true;
		}

		const size_t point_count = positions->size();
		const size_t curve_count = counts->size();
		std::vector<float> widths;
		AbcGeom::IFloatGeomParam width_param = schema.getWidthsParam();
		if (width_param.valid()) {
			const Alembic::Abc::FloatArraySamplePtr values = width_param.getExpandedValue().getVals();
			if (values) {
				widths.assign(values->get(), values->get() + values->size());
			}
		}
		// One per point, one per curve, or one for all.
		if (!warned_widths && !widths.empty() && widths.size() != point_count && widths.size() != curve_count && widths.size() != 1) {
			data->warnings.push_back(String("'") + p_curves.getName().c_str() + "': its widths match neither its points nor its curves; the default width is used.");
			warned_widths = true;
		}

		size_t at = 0;
		for (size_t c = 0; c < curve_count; c++) {
			const int32_t n = (*counts)[c];
			if (n <= 0 || at + (size_t)n > point_count) {
				data->warnings.push_back(String("'") + p_curves.getName().c_str() + "': a curve's points run past the end of its points; the rest of its curves are left out.");
				return;
			}
			for (int32_t j = 0; j < n; j++) {
				const size_t p = at + (size_t)j;
				Imath::V3d world_point;
				p_world.multVecMatrix(Imath::V3d((*positions)[p]), world_point);
				data->points.push_back(Vector3(world_point.x, world_point.y, world_point.z) * scale);
				float width = default_width;
				if (widths.size() == point_count) {
					width = widths[p] * scale;
				} else if (widths.size() == curve_count) {
					width = widths[c] * scale;
				} else if (widths.size() == 1) {
					width = widths[0] * scale;
				}
				data->widths.push_back(width);
			}
			data->strand_offsets.push_back(data->points.size());
			at += (size_t)n;
		}
	}
};

} // namespace

void alembic_read_groom(const String &p_path, float p_scale, float p_default_width, AlembicGroomData &r_data) {
	r_data = AlembicGroomData();
	r_data.strand_offsets.push_back(0);
	try {
		Alembic::AbcCoreFactory::IFactory factory;
		Alembic::AbcCoreFactory::IFactory::CoreType core;
		const Alembic::Abc::IArchive archive = factory.getArchive(p_path.utf8().get_data(), core);
		if (!archive.valid()) {
			r_data.error = "not an Alembic file, or an HDF5 one (only Ogawa, Alembic's format since 2013, is read).";
			return;
		}
		GroomWalk walk;
		walk.scale = p_scale;
		walk.default_width = p_default_width;
		walk.data = &r_data;
		Imath::M44d identity;
		identity.makeIdentity();
		walk.walk(archive.getTop(), identity);
		if (walk.closed_skipped > 0) {
			r_data.warnings.push_back(vformat("%d closed curves left out: hair strands are open curves.", walk.closed_skipped));
		}
	} catch (const std::exception &e) {
		r_data.error = String("the Alembic library failed: ") + e.what();
	} catch (...) {
		r_data.error = "the Alembic library failed.";
	}
	if (r_data.error.is_empty() && r_data.strand_offsets.size() < 2) {
		r_data.error = "the file has no curves.";
	}
}

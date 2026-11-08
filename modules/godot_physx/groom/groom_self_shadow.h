/**************************************************************************/
/*  groom_self_shadow.h                                                   */
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

#include "core/variant/variant.h"

// How much of the groom's own hair lies in each direction from each of its points, for self-shadowing (the precomputed
// self-shadowing of Indiana Jones and the Great Circle, SIGGRAPH 2025): rays from each point count the other strands
// they pass, up to GROOM_MAX_SCATTER_EVENTS, within p_reach of it; a ray that passes none counts as
// -GROOM_MAX_SCATTER_EVENTS, so a strand on the edge stays sharply lit from outside. Four numbers per point, the counts as
// an L1 spherical harmonic (Y00, Y1-1 (y), Y10 (z), Y11 (x), in the groom's space): the strands between a point and a
// light are max(0, the harmonic in the light's direction).
static constexpr int GROOM_MAX_SCATTER_EVENTS = 5;

// p_points, p_strand_offsets, p_widths as a PhysXGroom holds them. p_rays per point, p_reach in the groom's units.
PackedFloat32Array groom_bake_self_shadow(const PackedVector3Array &p_points, const PackedInt32Array &p_strand_offsets, const PackedFloat32Array &p_widths, int p_rays, float p_reach);

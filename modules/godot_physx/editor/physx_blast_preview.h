/**************************************************************************/
/*  physx_blast_preview.h                                                 */
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

#ifdef GODOT_PHYSX_BLAST

#include "core/object/ref_counted.h"

class ArrayMesh;
class PhysXBlastAsset;

// Builds one surface per leaf chunk of p_asset (index 0 is the unfractured
// root chunk -- never rendered on its own, same convention
// GodotPhysXBlastProbe/PhysXDestructible3D already use), each a flat,
// distinct color so the fracture cells read clearly regardless of scene
// lighting. Shared by PhysXBlastFractureDialog's live preview and the
// Inspector's PhysXBlastAsset preview (EditorInspectorPluginPhysXBlastAsset)
// -- both want the exact same "here is what actually broke" picture.
Ref<ArrayMesh> physx_blast_build_preview_mesh(const Ref<PhysXBlastAsset> &p_asset);

// Rebuilds p_mesh in place, so a preview showing it keeps its rotation, with each piece pushed out from the whole mesh's
// center of volume by p_explode times its own center's distance from there (0 = the whole mesh, 1 = twice as far out).
void physx_blast_update_preview_mesh(const Ref<ArrayMesh> &p_mesh, const Ref<PhysXBlastAsset> &p_asset, real_t p_explode);

#endif // GODOT_PHYSX_BLAST

/**************************************************************************/
/*  physx_shader_optimizer.h                                              */
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

#include "core/object/ref_counted.h"
#include "core/variant/typed_array.h"
#include "scene/resources/material.h"

// Optimize Shaders: finds the shaders a project's materials use that could be one shader -- the same code, or code that
// differs only in its parameters' default values -- and moves their materials onto one of them. A material that relied
// on a default the shared shader doesn't have gets that value set on itself, so nothing looks different. Each shader
// Godot compiles is a set of render pipelines kept by the graphics driver (tens of MB of system memory each for a big
// one, and slow to free), and two Shader resources with the same code are compiled twice: exporters that write a shader
// per material (Unreal's material instances baked into their own VisualShaders, say) leave hundreds.
class PhysXShaderOptimizer : public RefCounted {
	GDCLASS(PhysXShaderOptimizer, RefCounted);

protected:
	static void _bind_methods();

public:
	// The ShaderMaterials in their own files (.tres, .res) under p_dir, its subfolders included (not .godot, nor
	// folders with a .gdignore).
	static TypedArray<ShaderMaterial> find_materials(const String &p_dir);
	// Their paths, without loading them (a file's type is read from its header).
	static PackedStringArray find_material_paths(const String &p_dir);
	// The code with its parameters' default values left out (a "uniform ... = value;" becomes "uniform ...;"); instance
	// and global uniforms are kept whole (their defaults can't be set on a material).
	static String code_without_defaults(const String &p_code);
	// The shader's default textures (a VisualShader's texture nodes become them) as text, the same for the same
	// textures. Not bound: the editor dialog uses it to count shaders that differ only in those.
	static String default_textures_key(const Ref<Shader> &p_shader);
	// The shaders p_materials use, in groups that could be one shader (same code without defaults, same default
	// textures; each group two or more shaders), as
	// dictionaries: "shader" (the one kept: the one most materials use among those in their own files; null when none
	// is, so nothing can be merged), "shaders" (all of them, that one first), "identical" (the same code, defaults too),
	// "materials" (those to move onto "shader") and "parameters" (for each of those, the values to set on it: the
	// defaults it relied on that "shader" doesn't share); "skipped": shaders left as they are (their materials would
	// need a value a material can't hold: a texture parameter's "no texture" where "shader" has a default).
	static Array find_groups(const TypedArray<ShaderMaterial> &p_materials);
	// Moves a group's materials onto its shader and sets their parameters (saves nothing).
	static void merge_group(const Dictionary &p_group);
};

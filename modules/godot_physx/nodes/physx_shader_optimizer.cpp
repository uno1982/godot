/**************************************************************************/
/*  physx_shader_optimizer.cpp                                            */
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

#include "physx_shader_optimizer.h"

#include "core/io/dir_access.h"
#include "core/io/resource_loader.h"
#include "core/object/class_db.h"
#include "scene/resources/shader.h"
#include "servers/rendering/rendering_server.h"

namespace {

void collect_materials(const String &p_dir, TypedArray<ShaderMaterial> &r_materials) {
	Ref<DirAccess> dir = DirAccess::open(p_dir);
	if (dir.is_null() || dir->file_exists(".gdignore")) {
		return;
	}
	for (const String &sub : dir->get_directories()) {
		if (!sub.begins_with(".")) {
			collect_materials(p_dir.path_join(sub), r_materials);
		}
	}
	for (const String &file : dir->get_files()) {
		const String ext = file.get_extension().to_lower();
		if (ext != "tres" && ext != "res") {
			continue;
		}
		const String path = p_dir.path_join(file);
		if (ResourceLoader::get_resource_type(path) != "ShaderMaterial") {
			continue;
		}
		Ref<ShaderMaterial> material = ResourceLoader::load(path);
		if (material.is_valid()) {
			r_materials.push_back(material);
		}
	}
}

bool is_word_char(char32_t p_char) {
	return is_ascii_identifier_char(p_char);
}

// The word ending just before p_at (skipping whitespace), or empty.
String word_before(const String &p_code, int p_at) {
	int end = p_at;
	while (end > 0 && is_whitespace(p_code[end - 1])) {
		end--;
	}
	int start = end;
	while (start > 0 && is_word_char(p_code[start - 1])) {
		start--;
	}
	return p_code.substr(start, end - start);
}

bool own_file(const Ref<Resource> &p_resource) {
	const String path = p_resource->get_path();
	return !path.is_empty() && !path.contains("::");
}

// A parameter's value as the shader gives it when a material sets none: its default, else its type's zero value
// (nil for a texture, which a material can't be given in its place).
Variant default_of(const Ref<Shader> &p_shader, const PropertyInfo &p_param) {
	const Variant value = RS::get_singleton()->shader_get_parameter_default(p_shader->get_rid(), p_param.name);
	if (value.get_type() != Variant::NIL || p_param.type == Variant::OBJECT) {
		return value;
	}
	Variant zero;
	Callable::CallError error;
	Variant::construct(p_param.type, zero, nullptr, 0, error);
	return zero;
}

} // namespace

TypedArray<ShaderMaterial> PhysXShaderOptimizer::find_materials(const String &p_dir) {
	TypedArray<ShaderMaterial> materials;
	collect_materials(p_dir, materials);
	return materials;
}

String PhysXShaderOptimizer::code_without_defaults(const String &p_code) {
	String out;
	const int n = p_code.length();
	int copied = 0;
	int i = 0;
	while (i < n) {
		const bool at_word = p_code[i] == 'u' && (i == 0 || !is_word_char(p_code[i - 1])) && p_code.substr(i, 7) == "uniform" && (i + 7 >= n || !is_word_char(p_code[i + 7]));
		if (!at_word) {
			i++;
			continue;
		}
		const int end = p_code.find_char(';', i);
		if (end < 0) {
			break;
		}
		const String before = word_before(p_code, i);
		if (before == "instance" || before == "global") {
			i = end + 1;
			continue;
		}
		// The "=" outside any parentheses (a hint's arguments are inside them).
		int depth = 0;
		int equals = -1;
		for (int k = i + 7; k < end; k++) {
			const char32_t c = p_code[k];
			if (c == '(') {
				depth++;
			} else if (c == ')') {
				depth--;
			} else if (c == '=' && depth == 0) {
				equals = k;
				break;
			}
		}
		if (equals >= 0) {
			int keep = equals;
			while (keep > i && is_whitespace(p_code[keep - 1])) {
				keep--;
			}
			out += p_code.substr(copied, keep - copied);
			copied = end;
		}
		i = end + 1;
	}
	out += p_code.substr(copied);
	return out;
}

Array PhysXShaderOptimizer::find_groups(const TypedArray<ShaderMaterial> &p_materials) {
	// The shaders, each with the materials using it, in the order first met.
	LocalVector<Ref<Shader>> shaders;
	HashMap<ObjectID, LocalVector<Ref<ShaderMaterial>>> users;
	for (int i = 0; i < p_materials.size(); i++) {
		Ref<ShaderMaterial> material = p_materials[i];
		if (material.is_null() || material->get_shader().is_null()) {
			continue;
		}
		const Ref<Shader> shader = material->get_shader();
		LocalVector<Ref<ShaderMaterial>> *list = users.getptr(shader->get_instance_id());
		if (list == nullptr) {
			shaders.push_back(shader);
			list = &users.insert(shader->get_instance_id(), LocalVector<Ref<ShaderMaterial>>())->value;
		}
		list->push_back(material);
	}

	// Grouped by their code without defaults.
	HashMap<String, LocalVector<Ref<Shader>>> by_code;
	LocalVector<String> order;
	for (const Ref<Shader> &shader : shaders) {
		const String key = code_without_defaults(shader->get_code());
		LocalVector<Ref<Shader>> *group = by_code.getptr(key);
		if (group == nullptr) {
			order.push_back(key);
			group = &by_code.insert(key, LocalVector<Ref<Shader>>())->value;
		}
		group->push_back(shader);
	}

	Array groups;
	for (const String &key : order) {
		LocalVector<Ref<Shader>> &group = by_code[key];
		if (group.size() < 2) {
			continue;
		}
		// The one kept: in its own file (a shader built into one file can't be used from another), used by the most.
		Ref<Shader> kept;
		for (const Ref<Shader> &shader : group) {
			if (own_file(shader) && (kept.is_null() || users[shader->get_instance_id()].size() > users[kept->get_instance_id()].size())) {
				kept = shader;
			}
		}
		Array all;
		if (kept.is_valid()) {
			all.push_back(kept);
		}
		bool identical = true;
		for (const Ref<Shader> &shader : group) {
			if (shader != kept) {
				all.push_back(shader);
			}
			identical = identical && shader->get_code() == group[0]->get_code();
		}
		Dictionary entry;
		entry["shader"] = kept;
		entry["shaders"] = all;
		entry["identical"] = identical;
		Array materials;
		Array parameters;
		Array skipped;
		if (kept.is_valid()) {
			List<PropertyInfo> params;
			kept->get_shader_uniform_list(&params);
			List<StringName> kept_textures;
			kept->get_default_texture_parameter_list(&kept_textures);
			for (const Ref<Shader> &shader : group) {
				if (shader == kept) {
					continue;
				}
				// The values a material of this shader gets when it sets none, where the kept shader's differ.
				Dictionary differ;
				for (const PropertyInfo &param : params) {
					const Variant value = default_of(shader, param);
					if (value != default_of(kept, param)) {
						differ[param.name] = value;
					}
				}
				List<StringName> textures;
				shader->get_default_texture_parameter_list(&textures);
				for (const StringName &name : kept_textures) {
					if (!textures.find(name)) {
						textures.push_back(name);
					}
				}
				// A default texture the kept shader has and this one hasn't: its materials would need "no texture" set,
				// which a material can't hold. Left as it is.
				bool blocked = false;
				for (const StringName &name : textures) {
					const Ref<Texture> texture = shader->get_default_texture_parameter(name);
					if (texture != kept->get_default_texture_parameter(name)) {
						blocked = blocked || texture.is_null();
						differ[name] = texture;
					}
				}
				for (const KeyValue<Variant, Variant> &E : differ) {
					blocked = blocked || E.value.get_type() == Variant::NIL;
				}
				if (blocked) {
					skipped.push_back(shader);
					continue;
				}
				for (const Ref<ShaderMaterial> &material : users[shader->get_instance_id()]) {
					Dictionary set;
					for (const KeyValue<Variant, Variant> &E : differ) {
						if (material->get_shader_parameter(E.key).get_type() == Variant::NIL) {
							set[E.key] = E.value;
						}
					}
					materials.push_back(material);
					parameters.push_back(set);
				}
			}
		}
		entry["materials"] = materials;
		entry["parameters"] = parameters;
		entry["skipped"] = skipped;
		groups.push_back(entry);
	}
	return groups;
}

void PhysXShaderOptimizer::merge_group(const Dictionary &p_group) {
	const Ref<Shader> shader = p_group.get("shader", Variant());
	ERR_FAIL_COND_MSG(shader.is_null(), "PhysXShaderOptimizer: this group has no shader in its own file to keep.");
	const Array materials = p_group.get("materials", Array());
	const Array parameters = p_group.get("parameters", Array());
	for (int i = 0; i < materials.size(); i++) {
		Ref<ShaderMaterial> material = materials[i];
		if (material.is_null()) {
			continue;
		}
		material->set_shader(shader);
		const Dictionary set = i < parameters.size() ? Dictionary(parameters[i]) : Dictionary();
		for (const KeyValue<Variant, Variant> &E : set) {
			material->set_shader_parameter(E.key, E.value);
		}
	}
}

void PhysXShaderOptimizer::_bind_methods() {
	ClassDB::bind_static_method("PhysXShaderOptimizer", D_METHOD("find_materials", "dir"), &PhysXShaderOptimizer::find_materials);
	ClassDB::bind_static_method("PhysXShaderOptimizer", D_METHOD("code_without_defaults", "code"), &PhysXShaderOptimizer::code_without_defaults);
	ClassDB::bind_static_method("PhysXShaderOptimizer", D_METHOD("find_groups", "materials"), &PhysXShaderOptimizer::find_groups);
	ClassDB::bind_static_method("PhysXShaderOptimizer", D_METHOD("merge_group", "group"), &PhysXShaderOptimizer::merge_group);
}

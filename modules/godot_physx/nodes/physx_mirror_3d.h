/**************************************************************************/
/*  physx_mirror_3d.h                                                     */
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

#include "scene/3d/node_3d.h"
#include "scene/resources/compositor.h"
#include "scene/resources/environment.h"
#include "scene/resources/material.h"

class Camera3D;
class MeshInstance3D;
class SubViewport;
class ViewportTexture;

// A flat mirror drawn by a scene capture (as Unreal's SceneCaptureComponent2D draws one): a second camera renders the
// world into a texture its glass shows. The glass is a rectangle of `size` in the node's XY plane, facing its +Z. Each
// frame the capture camera sits at the viewing camera's eye mirrored through the glass, in frustum mode with its near
// plane on the glass: the exact mirror view, clipped at the glass (Godot's cameras have no clip plane or custom
// projection; for a rectangle this off-axis frustum is the same thing). Rendered in linear HDR and shown unshaded,
// flipped left to right, so the viewing camera tone maps it with the rest of its frame. With PXTGI in the world it can
// be lit by PXTGI too (gi). Rendered only while the glass can be seen.
class PhysXMirror3D : public Node3D {
	GDCLASS(PhysXMirror3D, Node3D);

public:
	enum UpdateMode {
		UPDATE_WHEN_VISIBLE,
		UPDATE_ALWAYS,
	};

private:
	Vector2 size = Vector2(2, 2);
	float resolution_scale = 0.5f;
	int max_resolution = 1024;
	UpdateMode update_mode = UPDATE_WHEN_VISIBLE;
	int update_interval = 1;
	uint32_t cull_mask = 0xFFFFF;
	uint32_t glass_layers = 1 << 19;
	float far = 0.0f;
	float lod_threshold = 1.0f;
	bool gi = true;
	Ref<Material> material;

	SubViewport *viewport = nullptr;
	Camera3D *camera = nullptr;
	MeshInstance3D *glass = nullptr;
	Ref<ShaderMaterial> default_material;
	Ref<Compositor> compositor;
	Ref<Environment> environment; // the viewer's, tone mapping off
	ObjectID environment_source;
	bool environment_gi = false;
	uint64_t frames = 0;
	bool rendered = false; // the capture rendered at least once since it was last hidden or skipped

	void _create();
	void _update_glass();
	void _update_material();
	Camera3D *_viewer() const;
	Ref<Environment> _viewer_environment(Camera3D *p_viewer) const;
	void _pre_draw();

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void set_size(const Vector2 &p_size);
	Vector2 get_size() const { return size; }
	void set_resolution_scale(float p_scale) { resolution_scale = CLAMP(p_scale, 0.05f, 2.0f); }
	float get_resolution_scale() const { return resolution_scale; }
	void set_max_resolution(int p_height) { max_resolution = CLAMP(p_height, 16, 8192); }
	int get_max_resolution() const { return max_resolution; }
	void set_update_mode(UpdateMode p_mode) { update_mode = p_mode; }
	UpdateMode get_update_mode() const { return update_mode; }
	void set_update_interval(int p_frames) { update_interval = CLAMP(p_frames, 1, 60); }
	int get_update_interval() const { return update_interval; }
	void set_cull_mask(uint32_t p_mask);
	uint32_t get_cull_mask() const { return cull_mask; }
	void set_glass_layers(uint32_t p_layers);
	uint32_t get_glass_layers() const { return glass_layers; }
	void set_far(float p_far) { far = MAX(p_far, 0.0f); }
	float get_far() const { return far; }
	void set_lod_threshold(float p_threshold);
	float get_lod_threshold() const { return lod_threshold; }
	void set_gi(bool p_gi);
	bool get_gi() const { return gi; }
	void set_material(const Ref<Material> &p_material);
	Ref<Material> get_material() const { return material; }

	Ref<ViewportTexture> get_mirror_texture() const;

	PackedStringArray get_configuration_warnings() const override;
};

VARIANT_ENUM_CAST(PhysXMirror3D::UpdateMode);

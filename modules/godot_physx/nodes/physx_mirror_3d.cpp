/**************************************************************************/
/*  physx_mirror_3d.cpp                                                   */
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

#include "physx_mirror_3d.h"

#include "physx_gi_effect.h"

#include "core/config/engine.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/main/viewport.h"
#include "scene/resources/3d/primitive_meshes.h"
#include "scene/resources/3d/world_3d.h"
#include "servers/rendering/rendering_server.h"

namespace {

// The glass's default look: the capture, unshaded, flipped left to right (the capture looks back through the glass,
// its left the mirror's right).
const char *GLASS_SHADER = R"(
shader_type spatial;
render_mode unshaded, cull_back, shadows_disabled;
uniform sampler2D mirror_texture : filter_linear, repeat_disable, hint_default_black;
void fragment() {
	ALBEDO = texture(mirror_texture, vec2(1.0 - UV.x, UV.y)).rgb;
}
)";

const StringName &texture_param() {
	static const StringName name = "mirror_texture";
	return name;
}

} // namespace

void PhysXMirror3D::_create() {
	if (glass != nullptr) {
		return;
	}
	glass = memnew(MeshInstance3D);
	glass->set_cast_shadows_setting(GeometryInstance3D::SHADOW_CASTING_SETTING_OFF);
	glass->set_gi_mode(GeometryInstance3D::GI_MODE_DISABLED);
	glass->set_layer_mask(glass_layers);
	add_child(glass, false, INTERNAL_MODE_FRONT);

	if (Engine::get_singleton()->is_editor_hint()) {
		// In the editor the glass is a plain mirror-like surface: the capture follows the game's camera, not the
		// editor's view.
		Ref<StandardMaterial3D> placeholder;
		placeholder.instantiate();
		placeholder->set_metallic(1.0f);
		placeholder->set_roughness(0.05f);
		default_material.unref();
		glass->set_material_override(placeholder);
		_update_glass();
		return;
	}

	viewport = memnew(SubViewport);
	viewport->set_use_hdr_2d(true);
	viewport->set_update_mode(SubViewport::UPDATE_DISABLED);
	viewport->set_mesh_lod_threshold(lod_threshold);
	add_child(viewport, false, INTERNAL_MODE_FRONT);
	camera = memnew(Camera3D);
	camera->set_projection(Camera3D::PROJECTION_FRUSTUM);
	camera->set_cull_mask(cull_mask & ~glass_layers);
	viewport->add_child(camera);
	// The capture's own compositor (a camera's replaces the world's): PXTGI's effect when it lights the capture, none
	// of the lens effects (a mirror shows the room, not the viewer's lens).
	compositor.instantiate();
	camera->set_compositor(compositor);

	Ref<Shader> shader;
	shader.instantiate();
	shader->set_code(GLASS_SHADER);
	default_material.instantiate();
	default_material->set_shader(shader);
	_update_glass();
	_update_material();
}

void PhysXMirror3D::_update_glass() {
	if (glass == nullptr) {
		return;
	}
	Ref<QuadMesh> quad = glass->get_mesh();
	if (quad.is_null()) {
		quad.instantiate();
		glass->set_mesh(quad);
	}
	quad->set_size(size);
}

void PhysXMirror3D::_update_material() {
	if (glass == nullptr || viewport == nullptr) {
		return;
	}
	Ref<Material> used = material.is_valid() ? material : Ref<Material>(default_material);
	Ref<ShaderMaterial> shader_material = used;
	if (shader_material.is_valid()) {
		shader_material->set_shader_parameter(texture_param(), viewport->get_texture());
	}
	glass->set_material_override(used);
}

Camera3D *PhysXMirror3D::_viewer() const {
	Viewport *vp = get_viewport();
	return vp ? vp->get_camera_3d() : nullptr;
}

Ref<Environment> PhysXMirror3D::_viewer_environment(Camera3D *p_viewer) const {
	if (p_viewer->get_environment().is_valid()) {
		return p_viewer->get_environment();
	}
	Ref<World3D> world = get_world_3d();
	if (world.is_valid()) {
		if (world->get_environment().is_valid()) {
			return world->get_environment();
		}
		return world->get_fallback_environment();
	}
	return Ref<Environment>();
}

// Each frame just before drawing (everything has moved by then): where the capture camera is, what it sees, whether
// it renders this frame.
void PhysXMirror3D::_pre_draw() {
	if (viewport == nullptr || !is_inside_tree()) {
		return;
	}
	Camera3D *viewer = _viewer();
	const Transform3D glass_frame = get_global_transform();
	const Transform3D to_local = glass_frame.affine_inverse();
	bool render = viewer != nullptr && is_visible_in_tree();
	Vector3 eye;
	if (render) {
		eye = to_local.xform(viewer->get_global_position());
		// From behind (or edge on) the glass shows nothing of the room.
		render = eye.z > 0.01f;
	}
	if (render && update_mode == UPDATE_WHEN_VISIBLE) {
		// Out of the viewer's frustum: all four corners outside one of its planes.
		const Vector<Plane> planes = viewer->get_frustum();
		const Vector2 half = size * 0.5f;
		const Vector3 corners[4] = {
			glass_frame.xform(Vector3(-half.x, -half.y, 0)),
			glass_frame.xform(Vector3(half.x, -half.y, 0)),
			glass_frame.xform(Vector3(half.x, half.y, 0)),
			glass_frame.xform(Vector3(-half.x, half.y, 0)),
		};
		for (const Plane &plane : planes) {
			int outside = 0;
			for (const Vector3 &c : corners) {
				outside += plane.is_point_over(c) ? 1 : 0;
			}
			if (outside == 4) {
				render = false;
				break;
			}
		}
	}
	frames++;
	if (render && rendered && update_interval > 1 && frames % update_interval != 0) {
		render = false; // between captures the glass keeps the last one
	} else if (!render) {
		rendered = false;
	}
	if (!render) {
		viewport->set_update_mode(SubViewport::UPDATE_DISABLED);
		return;
	}

	// Its size: a share of the viewer's height, capped, shaped as the glass.
	const float view_height = viewer->get_viewport() ? viewer->get_viewport()->get_visible_rect().size.y : 720.0f;
	const int height = CLAMP(int(view_height * resolution_scale), 16, max_resolution);
	const Size2i target(MAX(int(height * size.x / MAX(size.y, 0.001f)), 16), height);
	if (viewport->get_size() != target) {
		viewport->set_size(target);
	}

	// The viewer's environment, tone mapping off (the viewer tone maps the glass with its frame); PXTGI's GI switch on
	// when it lights the capture.
	const Ref<Environment> source = _viewer_environment(viewer);
	const bool lit = gi && PhysXGIEffect::has_world(get_world_3d().is_valid() ? get_world_3d()->get_instance_id() : ObjectID());
	if (source.is_valid() && (environment.is_null() || environment_source != source->get_instance_id() || environment_gi != lit)) {
		environment = source->duplicate();
		environment->set_tonemapper(Environment::TONE_MAPPER_LINEAR);
		environment->set_tonemap_exposure(1.0f);
		environment->set_glow_enabled(false);
		environment->set_sdfgi_enabled(lit);
		if (lit) {
			environment->set_sdfgi_cascades(1);
		}
		environment_source = source->get_instance_id();
		environment_gi = lit;
		camera->set_environment(environment);
		TypedArray<CompositorEffect> effects;
		if (lit) {
			effects.push_back(PhysXGIEffect::get_shared());
		}
		compositor->set_compositor_effects(effects);
	}
	camera->set_attributes(viewer->get_attributes());

	// The eye mirrored through the glass, looking back through it (its own -z the glass's +z); the near plane on the
	// glass, its middle offset from the camera's axis (the camera's right is the glass's -x).
	const Basis basis = glass_frame.basis.orthonormalized() * Basis(Vector3(0, 1, 0), Math::PI);
	const Vector3 scale = glass_frame.basis.get_scale();
	const Vector3 eye_m(eye.x * scale.x, eye.y * scale.y, eye.z * scale.z); // in meters, the glass's axes
	camera->set_global_transform(Transform3D(basis, glass_frame.xform(Vector3(eye.x, eye.y, -eye.z))));
	const Vector2 glass_m(size.x * scale.x, size.y * scale.y);
	camera->set_keep_aspect_mode(Camera3D::KEEP_HEIGHT);
	camera->set_frustum(glass_m.y, Vector2(eye_m.x, -eye_m.y), eye_m.z, far > 0.0f ? far : viewer->get_far());
	viewport->set_update_mode(SubViewport::UPDATE_ONCE);
	rendered = true;
}

void PhysXMirror3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			_create();
			if (!Engine::get_singleton()->is_editor_hint()) {
				RenderingServer::get_singleton()->connect(SNAME("frame_pre_draw"), callable_mp(this, &PhysXMirror3D::_pre_draw));
			}
		} break;
		case NOTIFICATION_EXIT_TREE: {
			if (!Engine::get_singleton()->is_editor_hint()) {
				RenderingServer::get_singleton()->disconnect(SNAME("frame_pre_draw"), callable_mp(this, &PhysXMirror3D::_pre_draw));
			}
			rendered = false;
		} break;
	}
}

void PhysXMirror3D::set_size(const Vector2 &p_size) {
	size = p_size.max(Vector2(0.01f, 0.01f));
	_update_glass();
	update_gizmos();
}

void PhysXMirror3D::set_cull_mask(uint32_t p_mask) {
	cull_mask = p_mask;
	if (camera) {
		camera->set_cull_mask(cull_mask & ~glass_layers);
	}
}

void PhysXMirror3D::set_glass_layers(uint32_t p_layers) {
	glass_layers = p_layers;
	if (glass) {
		glass->set_layer_mask(glass_layers);
	}
	if (camera) {
		camera->set_cull_mask(cull_mask & ~glass_layers);
	}
	update_configuration_warnings();
}

void PhysXMirror3D::set_lod_threshold(float p_threshold) {
	lod_threshold = MAX(p_threshold, 0.0f);
	if (viewport) {
		viewport->set_mesh_lod_threshold(lod_threshold);
	}
}

void PhysXMirror3D::set_gi(bool p_gi) {
	gi = p_gi;
	environment.unref(); // made again with the right switch
}

void PhysXMirror3D::set_material(const Ref<Material> &p_material) {
	material = p_material;
	_update_material();
}

Ref<ViewportTexture> PhysXMirror3D::get_mirror_texture() const {
	return viewport ? viewport->get_texture() : Ref<ViewportTexture>();
}

PackedStringArray PhysXMirror3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();
	if (glass_layers == 0) {
		warnings.push_back(RTR("The glass needs a render layer of its own (glass_layers): the capture leaves that layer out, or it would see itself."));
	}
	return warnings;
}

void PhysXMirror3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_size", "size"), &PhysXMirror3D::set_size);
	ClassDB::bind_method(D_METHOD("get_size"), &PhysXMirror3D::get_size);
	ClassDB::bind_method(D_METHOD("set_resolution_scale", "scale"), &PhysXMirror3D::set_resolution_scale);
	ClassDB::bind_method(D_METHOD("get_resolution_scale"), &PhysXMirror3D::get_resolution_scale);
	ClassDB::bind_method(D_METHOD("set_max_resolution", "height"), &PhysXMirror3D::set_max_resolution);
	ClassDB::bind_method(D_METHOD("get_max_resolution"), &PhysXMirror3D::get_max_resolution);
	ClassDB::bind_method(D_METHOD("set_update_mode", "mode"), &PhysXMirror3D::set_update_mode);
	ClassDB::bind_method(D_METHOD("get_update_mode"), &PhysXMirror3D::get_update_mode);
	ClassDB::bind_method(D_METHOD("set_update_interval", "frames"), &PhysXMirror3D::set_update_interval);
	ClassDB::bind_method(D_METHOD("get_update_interval"), &PhysXMirror3D::get_update_interval);
	ClassDB::bind_method(D_METHOD("set_cull_mask", "mask"), &PhysXMirror3D::set_cull_mask);
	ClassDB::bind_method(D_METHOD("get_cull_mask"), &PhysXMirror3D::get_cull_mask);
	ClassDB::bind_method(D_METHOD("set_glass_layers", "layers"), &PhysXMirror3D::set_glass_layers);
	ClassDB::bind_method(D_METHOD("get_glass_layers"), &PhysXMirror3D::get_glass_layers);
	ClassDB::bind_method(D_METHOD("set_far", "far"), &PhysXMirror3D::set_far);
	ClassDB::bind_method(D_METHOD("get_far"), &PhysXMirror3D::get_far);
	ClassDB::bind_method(D_METHOD("set_lod_threshold", "threshold"), &PhysXMirror3D::set_lod_threshold);
	ClassDB::bind_method(D_METHOD("get_lod_threshold"), &PhysXMirror3D::get_lod_threshold);
	ClassDB::bind_method(D_METHOD("set_gi", "enabled"), &PhysXMirror3D::set_gi);
	ClassDB::bind_method(D_METHOD("get_gi"), &PhysXMirror3D::get_gi);
	ClassDB::bind_method(D_METHOD("set_material", "material"), &PhysXMirror3D::set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &PhysXMirror3D::get_material);
	ClassDB::bind_method(D_METHOD("get_mirror_texture"), &PhysXMirror3D::get_mirror_texture);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "size", PROPERTY_HINT_NONE, "suffix:m"), "set_size", "get_size");
	ADD_GROUP("Capture", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "resolution_scale", PROPERTY_HINT_RANGE, "0.05,2,0.01"), "set_resolution_scale", "get_resolution_scale");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_resolution", PROPERTY_HINT_RANGE, "16,8192,1,suffix:px"), "set_max_resolution", "get_max_resolution");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "update_mode", PROPERTY_HINT_ENUM, "When Visible,Always"), "set_update_mode", "get_update_mode");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "update_interval", PROPERTY_HINT_RANGE, "1,60,1,suffix:frames"), "set_update_interval", "get_update_interval");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "cull_mask", PROPERTY_HINT_LAYERS_3D_RENDER), "set_cull_mask", "get_cull_mask");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "far", PROPERTY_HINT_RANGE, "0,10000,0.1,or_greater,suffix:m"), "set_far", "get_far");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lod_threshold", PROPERTY_HINT_RANGE, "0,1024,0.1"), "set_lod_threshold", "get_lod_threshold");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "gi"), "set_gi", "get_gi");
	ADD_GROUP("Glass", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "glass_layers", PROPERTY_HINT_LAYERS_3D_RENDER), "set_glass_layers", "get_glass_layers");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material", PROPERTY_HINT_RESOURCE_TYPE, "ShaderMaterial,StandardMaterial3D,ORMMaterial3D"), "set_material", "get_material");

	BIND_ENUM_CONSTANT(UPDATE_WHEN_VISIBLE);
	BIND_ENUM_CONSTANT(UPDATE_ALWAYS);
}

/**************************************************************************/
/*  physx_water_spray_3d.cpp                                              */
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

#include "physx_water_spray_3d.h"

#include "physx_boat_3d.h"
#include "physx_buoyancy_3d.h"
#include "physx_water_surface_3d.h"
#include "physx_water_wake_3d.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "scene/3d/gpu_particles_3d.h"
#include "scene/3d/gpu_particles_collision_3d.h"
#include "scene/3d/physics/rigid_body_3d.h"
#include "scene/resources/particle_process_material.h"

void PhysXWaterSpray3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_water_surface_path", "path"), &PhysXWaterSpray3D::set_water_surface_path);
	ClassDB::bind_method(D_METHOD("get_water_surface_path"), &PhysXWaterSpray3D::get_water_surface_path);
	ClassDB::bind_method(D_METHOD("set_bow_emitters", "paths"), &PhysXWaterSpray3D::set_bow_emitters);
	ClassDB::bind_method(D_METHOD("get_bow_emitters"), &PhysXWaterSpray3D::get_bow_emitters);
	ClassDB::bind_method(D_METHOD("set_stern_emitters", "paths"), &PhysXWaterSpray3D::set_stern_emitters);
	ClassDB::bind_method(D_METHOD("get_stern_emitters"), &PhysXWaterSpray3D::get_stern_emitters);
	ClassDB::bind_method(D_METHOD("set_slam_emitters", "paths"), &PhysXWaterSpray3D::set_slam_emitters);
	ClassDB::bind_method(D_METHOD("get_slam_emitters"), &PhysXWaterSpray3D::get_slam_emitters);
	ClassDB::bind_method(D_METHOD("set_start_speed", "speed"), &PhysXWaterSpray3D::set_start_speed);
	ClassDB::bind_method(D_METHOD("get_start_speed"), &PhysXWaterSpray3D::get_start_speed);
	ClassDB::bind_method(D_METHOD("set_reference_speed", "speed"), &PhysXWaterSpray3D::set_reference_speed);
	ClassDB::bind_method(D_METHOD("get_reference_speed"), &PhysXWaterSpray3D::get_reference_speed);
	ClassDB::bind_method(D_METHOD("set_slow_velocity_scale", "scale"), &PhysXWaterSpray3D::set_slow_velocity_scale);
	ClassDB::bind_method(D_METHOD("get_slow_velocity_scale"), &PhysXWaterSpray3D::get_slow_velocity_scale);
	ClassDB::bind_method(D_METHOD("set_submerge_depth", "depth"), &PhysXWaterSpray3D::set_submerge_depth);
	ClassDB::bind_method(D_METHOD("get_submerge_depth"), &PhysXWaterSpray3D::get_submerge_depth);
	ClassDB::bind_method(D_METHOD("set_slam_speed", "speed"), &PhysXWaterSpray3D::set_slam_speed);
	ClassDB::bind_method(D_METHOD("get_slam_speed"), &PhysXWaterSpray3D::get_slam_speed);
	ClassDB::bind_method(D_METHOD("set_slam_full_speed", "speed"), &PhysXWaterSpray3D::set_slam_full_speed);
	ClassDB::bind_method(D_METHOD("get_slam_full_speed"), &PhysXWaterSpray3D::get_slam_full_speed);
	ClassDB::bind_method(D_METHOD("set_slam_cooldown", "seconds"), &PhysXWaterSpray3D::set_slam_cooldown);
	ClassDB::bind_method(D_METHOD("get_slam_cooldown"), &PhysXWaterSpray3D::get_slam_cooldown);
	ClassDB::bind_method(D_METHOD("set_water_collision", "enabled"), &PhysXWaterSpray3D::set_water_collision);
	ClassDB::bind_method(D_METHOD("get_water_collision"), &PhysXWaterSpray3D::get_water_collision);
	ClassDB::bind_method(D_METHOD("set_collision_size", "size"), &PhysXWaterSpray3D::set_collision_size);
	ClassDB::bind_method(D_METHOD("get_collision_size"), &PhysXWaterSpray3D::get_collision_size);
	ClassDB::bind_method(D_METHOD("set_collision_fit_radius", "radius"), &PhysXWaterSpray3D::set_collision_fit_radius);
	ClassDB::bind_method(D_METHOD("get_collision_fit_radius"), &PhysXWaterSpray3D::get_collision_fit_radius);
	ClassDB::bind_method(D_METHOD("set_collision_margin", "margin"), &PhysXWaterSpray3D::set_collision_margin);
	ClassDB::bind_method(D_METHOD("get_collision_margin"), &PhysXWaterSpray3D::get_collision_margin);

	const String emitter_list_hint = vformat("%d/%d:%s", Variant::NODE_PATH, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "GPUParticles3D");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "water_surface_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "PhysXWaterSurface3D"), "set_water_surface_path", "get_water_surface_path");
	ADD_GROUP("Emitters", "");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "bow_emitters", PROPERTY_HINT_TYPE_STRING, emitter_list_hint), "set_bow_emitters", "get_bow_emitters");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "stern_emitters", PROPERTY_HINT_TYPE_STRING, emitter_list_hint), "set_stern_emitters", "get_stern_emitters");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "slam_emitters", PROPERTY_HINT_TYPE_STRING, emitter_list_hint), "set_slam_emitters", "get_slam_emitters");
	ADD_GROUP("Speed", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "start_speed", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s"), "set_start_speed", "get_start_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "reference_speed", PROPERTY_HINT_RANGE, "0.1,50,0.1,or_greater,suffix:m/s"), "set_reference_speed", "get_reference_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "slow_velocity_scale", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_slow_velocity_scale", "get_slow_velocity_scale");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "submerge_depth", PROPERTY_HINT_RANGE, "0.01,2,0.01,suffix:m"), "set_submerge_depth", "get_submerge_depth");
	ADD_GROUP("Slam", "slam_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "slam_speed", PROPERTY_HINT_RANGE, "0,20,0.01,suffix:m/s"), "set_slam_speed", "get_slam_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "slam_full_speed", PROPERTY_HINT_RANGE, "0.1,30,0.01,suffix:m/s"), "set_slam_full_speed", "get_slam_full_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "slam_cooldown", PROPERTY_HINT_RANGE, "0,5,0.01,suffix:s"), "set_slam_cooldown", "get_slam_cooldown");
	ADD_GROUP("Water Collision", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "water_collision"), "set_water_collision", "get_water_collision");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collision_size", PROPERTY_HINT_RANGE, "0.5,100,0.1,or_greater,suffix:m"), "set_collision_size", "get_collision_size");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collision_fit_radius", PROPERTY_HINT_RANGE, "0.1,20,0.01,suffix:m"), "set_collision_fit_radius", "get_collision_fit_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collision_margin", PROPERTY_HINT_RANGE, "0,1,0.001,suffix:m"), "set_collision_margin", "get_collision_margin");

	ADD_SIGNAL(MethodInfo("slammed", PropertyInfo(Variant::VECTOR3, "position"), PropertyInfo(Variant::FLOAT, "speed")));
}

RigidBody3D *PhysXWaterSpray3D::get_body() const {
	return Object::cast_to<RigidBody3D>(get_parent());
}

PhysXWaterSurface3D *PhysXWaterSpray3D::get_water_surface() const {
	return ObjectDB::get_instance<PhysXWaterSurface3D>(water_id);
}

void PhysXWaterSpray3D::_clear() {
	emitters.clear();
	water_id = ObjectID();
	wake_id = ObjectID();
	built = false;
}

void PhysXWaterSpray3D::_add_emitters(const TypedArray<NodePath> &p_paths, Role p_role) {
	const Transform3D to_body = get_body()->get_global_transform().affine_inverse();
	for (int i = 0; i < p_paths.size(); i++) {
		GPUParticles3D *particles = Object::cast_to<GPUParticles3D>(get_node_or_null(p_paths[i]));
		if (particles == nullptr) {
			continue;
		}
		Emitter e;
		e.id = particles->get_instance_id();
		e.role = p_role;
		e.rest = to_body * particles->get_global_transform();
		e.base_ratio = particles->get_amount_ratio();
		// Our own copy of its process material: its launch speed is scaled
		// per emitter, and the resource may be shared.
		Ref<ParticleProcessMaterial> material = particles->get_process_material();
		if (material.is_valid()) {
			material = material->duplicate();
			particles->set_process_material(material);
			e.material = material;
			e.base_velocity_min = material->get_param_min(ParticleProcessMaterial::PARAM_INITIAL_LINEAR_VELOCITY);
			e.base_velocity_max = material->get_param_max(ParticleProcessMaterial::PARAM_INITIAL_LINEAR_VELOCITY);
		}
		if (p_role == ROLE_SLAM) {
			particles->set_emitting(false);
		} else {
			particles->set_amount_ratio(0.0f);
			particles->set_emitting(true);
		}
		emitters.push_back(e);
	}
}

void PhysXWaterSpray3D::_build() {
	_clear();
	built = true; // tried: not again every tick when there's no water
	RigidBody3D *body = get_body();
	if (body == nullptr) {
		return;
	}
	// The water: our own path, else the sibling buoyancy's.
	PhysXWaterSurface3D *water = Object::cast_to<PhysXWaterSurface3D>(get_node_or_null(water_surface_path));
	for (int i = 0; i < body->get_child_count(); i++) {
		Node *sibling = body->get_child(i);
		PhysXBuoyancy3D *buoyancy = Object::cast_to<PhysXBuoyancy3D>(sibling);
		if (water == nullptr && buoyancy != nullptr) {
			water = Object::cast_to<PhysXWaterSurface3D>(buoyancy->get_node_or_null(buoyancy->get_water_surface_path()));
		}
		if (Object::cast_to<PhysXWaterWake3D>(sibling) != nullptr) {
			wake_id = sibling->get_instance_id();
		}
	}
	if (water == nullptr) {
		return;
	}
	water_id = water->get_instance_id();
	_add_emitters(bow_emitters, ROLE_BOW);
	_add_emitters(stern_emitters, ROLE_STERN);
	_add_emitters(slam_emitters, ROLE_SLAM);
}

void PhysXWaterSpray3D::_update_collider(const PhysXWaterSurface3D *p_water, const Vector3 &p_center, const Basis &p_level) {
	have_plane = false;
	if (collider == nullptr || !water_collision) {
		return;
	}
	// A plane fitted to the water around the hull: its height under the
	// hull and its slope across collision_fit_radius each way (the waves,
	// without the hull's own wake).
	const Vector3 ax = p_level.get_column(0) * collision_fit_radius;
	const Vector3 az = p_level.get_column(2) * collision_fit_radius;
	const float h = p_water->sample_height_excluding_wake(p_center, wake_id);
	const float hxp = p_water->sample_height_excluding_wake(p_center + ax, wake_id);
	const float hxn = p_water->sample_height_excluding_wake(p_center - ax, wake_id);
	const float hzp = p_water->sample_height_excluding_wake(p_center + az, wake_id);
	const float hzn = p_water->sample_height_excluding_wake(p_center - az, wake_id);
	if (!Math::is_finite(h) || !Math::is_finite(hxp) || !Math::is_finite(hxn) || !Math::is_finite(hzp) || !Math::is_finite(hzn)) {
		// Near dry land: no plane to fit.
		collider->set_visible(false);
		return;
	}
	collider->set_visible(true);
	const Vector3 up(0, 1, 0);
	const Vector3 tx = (p_level.get_column(0) + up * ((hxp - hxn) / (2.0f * collision_fit_radius))).normalized();
	const Vector3 tz = p_level.get_column(2) + up * ((hzp - hzn) / (2.0f * collision_fit_radius));
	const Vector3 normal = tz.cross(tx).normalized();
	const Vector3 z = tx.cross(normal).normalized();
	const float depth = 2.0f;
	collider->set_size(Vector3(collision_size, depth, collision_size));
	// The box's top face on the plane.
	plane_point = Vector3(p_center.x, h, p_center.z);
	plane_normal = normal;
	have_plane = true;
	collider->set_global_transform(Transform3D(Basis(normal.cross(z), normal, z), plane_point - normal * (depth * 0.5f)));
}

void PhysXWaterSpray3D::_physics_step(double p_delta) {
	RigidBody3D *body = get_body();
	PhysXWaterSurface3D *water = get_water_surface();
	if (body == nullptr || water == nullptr || emitters.is_empty() || p_delta <= 0.0) {
		return;
	}
	const Transform3D xf = body->get_global_transform();
	const Vector3 lin = body->get_linear_velocity();
	const Vector3 ang = body->get_angular_velocity();
	// Level, turned with the hull's heading: spray leaves the water upward
	// whatever the hull's pitch and roll.
	Vector3 fwd = xf.basis.get_column(2);
	fwd.y = 0.0f;
	fwd = fwd.length_squared() > 1e-6f ? fwd.normalized() : Vector3(0, 0, 1);
	const Vector3 up(0, 1, 0);
	const Basis level(up.cross(fwd).normalized(), up, fwd);
	const float forward_speed = MAX(lin.dot(fwd), 0.0f);
	const float bow_rate = Math::smoothstep(start_speed, reference_speed, forward_speed);
	const float bow_velocity = Math::lerp(slow_velocity_scale, 1.0f, MIN(forward_speed / reference_speed, 1.0f));
	_update_collider(water, xf.origin, level);

	// Propeller churn, from a sibling PhysXBoat3D.
	float churn = 0.0f;
	for (int i = 0; i < body->get_child_count(); i++) {
		const PhysXBoat3D *boat = Object::cast_to<PhysXBoat3D>(body->get_child(i));
		if (boat != nullptr && boat->get_max_thrust() > 0.0f) {
			churn = MIN(Math::abs(boat->get_applied_thrust()) / boat->get_max_thrust(), 1.0f);
			break;
		}
	}
	const float stern_velocity = Math::lerp(slow_velocity_scale, 1.0f, churn);

	for (Emitter &e : emitters) {
		GPUParticles3D *particles = ObjectDB::get_instance<GPUParticles3D>(e.id);
		if (particles == nullptr) {
			continue;
		}
		e.cooldown = MAX(e.cooldown - (float)p_delta, 0.0f);
		const Vector3 hull = xf.xform(e.rest.origin);
		// How far that hull point sits above the water (as the buoyancy sees
		// it, without the hull's own wake), and the drawn surface it sprays
		// from.
		const float water_level = water->sample_height_excluding_wake(hull, wake_id);
		if (!Math::is_finite(water_level)) {
			// Over dry land: nothing to spray.
			if (e.role != ROLE_SLAM) {
				particles->set_amount_ratio(0.0f);
			}
			e.have_prev = false;
			continue;
		}
		const float surface = water->sample_height(hull);
		const float gap = hull.y - water_level;
		const float wet = CLAMP(1.0f - gap / submerge_depth, 0.0f, 1.0f);
		float spawn_y = surface;
		if (have_plane && plane_normal.y > 0.1f) {
			// Clear of the collider's top, which can sit above the drawn
			// surface in the hull's own wake.
			const float plane_y = plane_point.y - (plane_normal.x * (hull.x - plane_point.x) + plane_normal.z * (hull.z - plane_point.z)) / plane_normal.y;
			spawn_y = MAX(surface, plane_y) + collision_margin;
		}
		particles->set_global_transform(Transform3D(level * e.rest.basis, Vector3(hull.x, spawn_y, hull.z)));

		float velocity_scale = 1.0f;
		switch (e.role) {
			case ROLE_BOW: {
				particles->set_amount_ratio(e.base_ratio * wet * bow_rate);
				velocity_scale = bow_velocity;
			} break;
			case ROLE_STERN: {
				particles->set_amount_ratio(e.base_ratio * wet * churn);
				velocity_scale = stern_velocity;
			} break;
			case ROLE_SLAM: {
				// Dropped back onto the water: the hull point was clear of it
				// last tick and is in it now, closing faster than slam_speed.
				const float hull_vy = (lin + ang.cross(hull - xf.origin)).y;
				const float water_vy = e.have_prev ? (water_level - e.prev_water) / (float)p_delta : 0.0f;
				const float closing = water_vy - hull_vy;
				if (e.have_prev && e.prev_gap > 0.0f && gap <= 0.0f && closing > slam_speed && e.cooldown <= 0.0f) {
					const float strength = CLAMP(closing / slam_full_speed, 0.2f, 1.0f);
					particles->set_amount_ratio(e.base_ratio * strength);
					if (e.material.is_valid()) {
						const float scale = Math::lerp(slow_velocity_scale, 1.0f, strength);
						e.material->set_param_min(ParticleProcessMaterial::PARAM_INITIAL_LINEAR_VELOCITY, e.base_velocity_min * scale);
						e.material->set_param_max(ParticleProcessMaterial::PARAM_INITIAL_LINEAR_VELOCITY, e.base_velocity_max * scale);
					}
					particles->restart();
					e.cooldown = slam_cooldown;
					emit_signal(SNAME("slammed"), Vector3(hull.x, surface, hull.z), closing);
				}
				e.prev_gap = gap;
				e.prev_water = water_level;
				e.have_prev = true;
				continue;
			}
		}
		if (e.material.is_valid()) {
			const float vmax = e.base_velocity_max * velocity_scale;
			// Only on a real change: each set reaches the material's shader.
			if (Math::abs(e.material->get_param_max(ParticleProcessMaterial::PARAM_INITIAL_LINEAR_VELOCITY) - vmax) > 0.01f) {
				e.material->set_param_min(ParticleProcessMaterial::PARAM_INITIAL_LINEAR_VELOCITY, e.base_velocity_min * velocity_scale);
				e.material->set_param_max(ParticleProcessMaterial::PARAM_INITIAL_LINEAR_VELOCITY, vmax);
			}
		}
	}
}

void PhysXWaterSpray3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			if (!Engine::get_singleton()->is_editor_hint()) {
				set_physics_process_internal(true);
				// The water for the spray to land on; placed every tick.
				collider = memnew(GPUParticlesCollisionBox3D);
				collider->set_as_top_level(true);
				collider->set_visible(false);
				add_child(collider, false, INTERNAL_MODE_BACK);
			}
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			// Set up on the first tick, once the water and emitters are ready.
			if (!built) {
				_build();
			}
			_physics_step(get_physics_process_delta_time());
		} break;
		case NOTIFICATION_EXIT_TREE: {
			_clear();
		} break;
	}
}

void PhysXWaterSpray3D::set_water_surface_path(const NodePath &p_path) {
	water_surface_path = p_path;
	built = false;
	update_configuration_warnings();
}

void PhysXWaterSpray3D::set_water_collision(bool p_enabled) {
	water_collision = p_enabled;
	if (collider != nullptr && !water_collision) {
		collider->set_visible(false);
	}
}

void PhysXWaterSpray3D::set_bow_emitters(const TypedArray<NodePath> &p_paths) {
	bow_emitters = p_paths;
	built = false;
	update_configuration_warnings();
}

void PhysXWaterSpray3D::set_stern_emitters(const TypedArray<NodePath> &p_paths) {
	stern_emitters = p_paths;
	built = false;
	update_configuration_warnings();
}

void PhysXWaterSpray3D::set_slam_emitters(const TypedArray<NodePath> &p_paths) {
	slam_emitters = p_paths;
	built = false;
	update_configuration_warnings();
}

PackedStringArray PhysXWaterSpray3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();
	if (Object::cast_to<RigidBody3D>(get_parent()) == nullptr) {
		warnings.push_back(RTR("PhysXWaterSpray3D follows a RigidBody3D: make it a child of one (a boat)."));
	}
	if (bow_emitters.is_empty() && stern_emitters.is_empty() && slam_emitters.is_empty()) {
		warnings.push_back(RTR("No emitters: add GPUParticles3D nodes to bow_emitters, stern_emitters or slam_emitters."));
	}
	for (int i = 0; i < slam_emitters.size(); i++) {
		const GPUParticles3D *particles = Object::cast_to<GPUParticles3D>(get_node_or_null(slam_emitters[i]));
		if (particles != nullptr && !particles->get_one_shot()) {
			warnings.push_back(RTR("Slam emitters fire a burst on each slam: turn on one_shot on them."));
			break;
		}
	}
	return warnings;
}

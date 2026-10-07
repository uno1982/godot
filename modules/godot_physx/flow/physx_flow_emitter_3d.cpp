/**************************************************************************/
/*  physx_flow_emitter_3d.cpp                                             */
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

#include "physx_flow_emitter_3d.h"

#include "physx_flow_3d.h"

#include "core/object/class_db.h"
#include "scene/main/scene_tree.h"

void PhysXFlowEmitter3D::set_shape(Shape p_shape) {
	shape = p_shape;
	notify_property_list_changed();
	update_gizmos();
}

void PhysXFlowEmitter3D::set_radius(float p_radius) {
	radius = MAX(p_radius, 0.01f);
	update_gizmos();
}

void PhysXFlowEmitter3D::set_size(const Vector3 &p_size) {
	size = p_size.maxf(0.01f);
	update_gizmos();
}

void PhysXFlowEmitter3D::set_collision(bool p_collision) {
	collision = p_collision;
	notify_property_list_changed();
}

void PhysXFlowEmitter3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			add_to_group(GROUP);
			update_configuration_warnings();
			has_last_position = false;
			motion_velocity = Vector3();
			set_physics_process_internal(true);
		} break;
		case NOTIFICATION_EXIT_TREE: {
			remove_from_group(GROUP);
			set_physics_process_internal(false);
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			const Vector3 p = get_global_position();
			const double dt = get_physics_process_delta_time();
			motion_velocity = has_last_position && dt > 0.0 ? (p - last_position) / dt : Vector3();
			last_position = p;
			has_last_position = true;
		} break;
	}
}

void PhysXFlowEmitter3D::set_flow_path(const NodePath &p_path) {
	flow_path = p_path;
	update_configuration_warnings();
}

PhysXFlow3D *PhysXFlowEmitter3D::get_flow() const {
	if (!is_inside_tree()) {
		return nullptr;
	}
	if (!flow_path.is_empty()) {
		return Object::cast_to<PhysXFlow3D>(get_node_or_null(flow_path));
	}
	const Vector<Node *> flows = get_tree()->get_nodes_in_group(PhysXFlow3D::GROUP);
	return flows.size() == 1 ? Object::cast_to<PhysXFlow3D>(flows[0]) : nullptr;
}

PackedStringArray PhysXFlowEmitter3D::get_configuration_warnings() const {
	PackedStringArray warnings = Node3D::get_configuration_warnings();
	if (is_inside_tree() && get_flow() == nullptr) {
		const int flows = get_tree()->get_nodes_in_group(PhysXFlow3D::GROUP).size();
		if (!flow_path.is_empty()) {
			warnings.push_back(RTR("\"Flow\" doesn't point at a PhysXFlow3D: this emitter feeds nothing."));
		} else if (flows == 0) {
			warnings.push_back(RTR("No PhysXFlow3D in the scene: this emitter has nothing to feed."));
		} else {
			warnings.push_back(RTR("There's more than one PhysXFlow3D: set \"Flow\" to the one this emitter should feed."));
		}
	}
	return warnings;
}

void PhysXFlowEmitter3D::_validate_property(PropertyInfo &p_property) const {
	if (p_property.name == "radius" && shape != SHAPE_SPHERE) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	} else if (p_property.name == "size" && shape != SHAPE_BOX) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	} else if (collision && (p_property.name == "velocity" || p_property.name == "temperature" || p_property.name == "fuel" || p_property.name == "burn" || p_property.name == "smoke" || p_property.name == "divergence" || p_property.name == "couple_rate")) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void PhysXFlowEmitter3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &PhysXFlowEmitter3D::set_enabled);
	ClassDB::bind_method(D_METHOD("is_enabled"), &PhysXFlowEmitter3D::is_enabled);
	ClassDB::bind_method(D_METHOD("set_shape", "shape"), &PhysXFlowEmitter3D::set_shape);
	ClassDB::bind_method(D_METHOD("get_shape"), &PhysXFlowEmitter3D::get_shape);
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &PhysXFlowEmitter3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &PhysXFlowEmitter3D::get_radius);
	ClassDB::bind_method(D_METHOD("set_size", "size"), &PhysXFlowEmitter3D::set_size);
	ClassDB::bind_method(D_METHOD("get_size"), &PhysXFlowEmitter3D::get_size);
	ClassDB::bind_method(D_METHOD("set_velocity", "velocity"), &PhysXFlowEmitter3D::set_velocity);
	ClassDB::bind_method(D_METHOD("get_velocity"), &PhysXFlowEmitter3D::get_velocity);
	ClassDB::bind_method(D_METHOD("set_temperature", "temperature"), &PhysXFlowEmitter3D::set_temperature);
	ClassDB::bind_method(D_METHOD("get_temperature"), &PhysXFlowEmitter3D::get_temperature);
	ClassDB::bind_method(D_METHOD("set_fuel", "fuel"), &PhysXFlowEmitter3D::set_fuel);
	ClassDB::bind_method(D_METHOD("get_fuel"), &PhysXFlowEmitter3D::get_fuel);
	ClassDB::bind_method(D_METHOD("set_burn", "burn"), &PhysXFlowEmitter3D::set_burn);
	ClassDB::bind_method(D_METHOD("get_burn"), &PhysXFlowEmitter3D::get_burn);
	ClassDB::bind_method(D_METHOD("set_smoke", "smoke"), &PhysXFlowEmitter3D::set_smoke);
	ClassDB::bind_method(D_METHOD("get_smoke"), &PhysXFlowEmitter3D::get_smoke);
	ClassDB::bind_method(D_METHOD("set_divergence", "divergence"), &PhysXFlowEmitter3D::set_divergence);
	ClassDB::bind_method(D_METHOD("get_divergence"), &PhysXFlowEmitter3D::get_divergence);
	ClassDB::bind_method(D_METHOD("set_couple_rate", "rate"), &PhysXFlowEmitter3D::set_couple_rate);
	ClassDB::bind_method(D_METHOD("get_couple_rate"), &PhysXFlowEmitter3D::get_couple_rate);
	ClassDB::bind_method(D_METHOD("set_collision", "collision"), &PhysXFlowEmitter3D::set_collision);
	ClassDB::bind_method(D_METHOD("is_collision"), &PhysXFlowEmitter3D::is_collision);
	ClassDB::bind_method(D_METHOD("get_motion_velocity"), &PhysXFlowEmitter3D::get_motion_velocity);
	ClassDB::bind_method(D_METHOD("set_flow_path", "path"), &PhysXFlowEmitter3D::set_flow_path);
	ClassDB::bind_method(D_METHOD("get_flow_path"), &PhysXFlowEmitter3D::get_flow_path);
	ClassDB::bind_method(D_METHOD("get_flow"), &PhysXFlowEmitter3D::get_flow);

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "flow", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "PhysXFlow3D"), "set_flow_path", "get_flow_path");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "is_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "shape", PROPERTY_HINT_ENUM, "Sphere,Box"), "set_shape", "get_shape");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.01,20,0.01,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "size", PROPERTY_HINT_NONE, "suffix:m"), "set_size", "get_size");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collision"), "set_collision", "is_collision");
	ADD_GROUP("Emission", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "velocity", PROPERTY_HINT_NONE, "suffix:m/s"), "set_velocity", "get_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "temperature", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_temperature", "get_temperature");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fuel", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_fuel", "get_fuel");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "burn", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_burn", "get_burn");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "smoke", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_smoke", "get_smoke");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "divergence", PROPERTY_HINT_RANGE, "-10,10,0.01"), "set_divergence", "get_divergence");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "couple_rate", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater"), "set_couple_rate", "get_couple_rate");

	BIND_ENUM_CONSTANT(SHAPE_SPHERE);
	BIND_ENUM_CONSTANT(SHAPE_BOX);
}

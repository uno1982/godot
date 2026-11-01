/**************************************************************************/
/*  physx_vehicle_engine_settings.h                                       */
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

// The engine drive's node side, shared by PhysXVehicle3D and PhysXMotorcycle3D
// the way godot_physx_vehicle_control.h shares the body controls: the
// drivetrain choice, the engine and gearbox settings, the gearbox controls and
// their read-outs. A node using it puts PHYSX_VEHICLE_ENGINE_DECLARE() in its
// class and VARIANT_ENUM_CAST(m_class::Drivetrain) after it, and
// PHYSX_VEHICLE_ENGINE_DEFINE(m_class) in its .cpp, where impl->vehicle derives
// from VehicleEngineDrive (godot_physx_vehicle_engine.h) and _rebuild_if_live()
// rebuilds it. It binds them with PHYSX_VEHICLE_ENGINE_BIND_DRIVETRAIN() (in
// its Drivetrain group), PHYSX_VEHICLE_ENGINE_BIND_SETTINGS() (the Engine and
// Gearbox groups) and PHYSX_VEHICLE_ENGINE_BIND_CONTROLS() (in its Controls
// group); fills its config's engine with _engine_config() when the drivetrain is
// DRIVETRAIN_ENGINE; calls _engine_controls() before each step; and hides the
// settings its drivetrain doesn't use with _engine_validate_property().

#include "core/object/class_db.h"
#include "core/variant/variant.h"
#include "scene/resources/curve.h"

struct VehicleEngineConfig;

#define PHYSX_VEHICLE_ENGINE_DECLARE() \
public: \
	/* How throttle reaches the wheels. DIRECT: straight to wheel torque (max_engine_torque), forward or reverse. */ \
	/* ENGINE: PhysX's engine drive -- an engine with a torque curve, clutch and gearbox (automatic or manual), */ \
	/* tuned with the engine_*, gear_*, autobox_* and clutch_* properties. */ \
	enum Drivetrain { \
		DRIVETRAIN_DIRECT, \
		DRIVETRAIN_ENGINE, \
	}; \
	void set_drivetrain(Drivetrain p_drivetrain); \
	Drivetrain get_drivetrain() const { \
		return drivetrain; \
	} \
	/* Engine drive (DRIVETRAIN_ENGINE). Engine speeds are RPM; gear ratios are positive, first gear to top. */ \
	void set_engine_peak_torque(real_t p_v); \
	real_t get_engine_peak_torque() const { \
		return engine_peak_torque; \
	} \
	void set_engine_max_rpm(real_t p_v); \
	real_t get_engine_max_rpm() const { \
		return engine_max_rpm; \
	} \
	void set_engine_idle_rpm(real_t p_v); \
	real_t get_engine_idle_rpm() const { \
		return engine_idle_rpm; \
	} \
	/* Torque (of engine_peak_torque) against engine speed (of engine_max_rpm), both 0..1. Null = flat. */ \
	void set_engine_torque_curve(const Ref<Curve> &p_curve); \
	Ref<Curve> get_engine_torque_curve() const { \
		return engine_torque_curve; \
	} \
	void set_engine_moment_of_inertia(real_t p_v); \
	real_t get_engine_moment_of_inertia() const { \
		return engine_moment_of_inertia; \
	} \
	void set_engine_damping_full_throttle(real_t p_v); \
	real_t get_engine_damping_full_throttle() const { \
		return engine_damping_full_throttle; \
	} \
	void set_engine_damping_zero_throttle_clutch_engaged(real_t p_v); \
	real_t get_engine_damping_zero_throttle_clutch_engaged() const { \
		return engine_damping_zero_throttle_clutch_engaged; \
	} \
	void set_engine_damping_zero_throttle_clutch_disengaged(real_t p_v); \
	real_t get_engine_damping_zero_throttle_clutch_disengaged() const { \
		return engine_damping_zero_throttle_clutch_disengaged; \
	} \
	void set_gear_ratios(const PackedFloat32Array &p_ratios); \
	PackedFloat32Array get_gear_ratios() const { \
		return gear_ratios; \
	} \
	void set_reverse_gear_ratio(real_t p_v); \
	real_t get_reverse_gear_ratio() const { \
		return reverse_gear_ratio; \
	} \
	void set_final_drive_ratio(real_t p_v); \
	real_t get_final_drive_ratio() const { \
		return final_drive_ratio; \
	} \
	void set_gear_switch_time(real_t p_v); \
	real_t get_gear_switch_time() const { \
		return gear_switch_time; \
	} \
	void set_autobox_up_ratio(real_t p_v); \
	real_t get_autobox_up_ratio() const { \
		return autobox_up_ratio; \
	} \
	void set_autobox_down_ratio(real_t p_v); \
	real_t get_autobox_down_ratio() const { \
		return autobox_down_ratio; \
	} \
	void set_autobox_latency(real_t p_v); \
	real_t get_autobox_latency() const { \
		return autobox_latency; \
	} \
	void set_clutch_strength(real_t p_v); \
	real_t get_clutch_strength() const { \
		return clutch_strength; \
	} \
	/* Engine drive controls, read every tick. automatic_transmission: the autobox picks forward gears (reverse */ \
	/* still selects the reverse gear); off, the gearbox goes to `gear` (-1 reverse, 0 neutral, 1.. forward), and */ \
	/* reverse = true still means the reverse gear. clutch: 0 engaged .. 1 pressed. clutch_assist off: a manual */ \
	/* gearbox changes gear (reverse too) only while the clutch is pressed past halfway, and otherwise holds its */ \
	/* gear. The automatic holds its gear while the clutch is pressed past halfway (revving for a launch). */ \
	void set_automatic_transmission(bool p_v) { \
		automatic_transmission = p_v; \
	} \
	bool is_automatic_transmission() const { \
		return automatic_transmission; \
	} \
	void set_gear(int p_gear) { \
		gear = p_gear; \
	} \
	int get_gear() const { \
		return gear; \
	} \
	void set_clutch(real_t p_v) { \
		clutch = p_v; \
	} \
	real_t get_clutch() const { \
		return clutch; \
	} \
	void set_clutch_assist(bool p_v) { \
		clutch_assist = p_v; \
	} \
	bool is_clutch_assist() const { \
		return clutch_assist; \
	} \
	/* The gear engaged now, numbered as `gear` (0 while a shift is under way). Direct drive: 1 or -1. */ \
	int get_current_gear() const; \
	/* The gear the gearbox is shifting to (the current one when not shifting), numbered as `gear`. */ \
	int get_target_gear() const; \
	/* Engine speed in RPM; 0 with direct drive. */ \
	real_t get_engine_rpm() const; \
\
private: \
	Drivetrain drivetrain = DRIVETRAIN_DIRECT; \
	/* Defaults: see VehicleEngineConfig. */ \
	real_t engine_peak_torque = 500.0f; \
	real_t engine_max_rpm = 6000.0f; \
	real_t engine_idle_rpm = 0.0f; \
	Ref<Curve> engine_torque_curve; \
	real_t engine_moment_of_inertia = 1.0f; \
	real_t engine_damping_full_throttle = 0.15f; \
	real_t engine_damping_zero_throttle_clutch_engaged = 2.0f; \
	real_t engine_damping_zero_throttle_clutch_disengaged = 0.35f; \
	PackedFloat32Array gear_ratios = { 4.0f, 2.0f, 1.5f, 1.1f, 1.0f }; \
	real_t reverse_gear_ratio = 4.0f; \
	real_t final_drive_ratio = 4.0f; \
	real_t gear_switch_time = 0.5f; \
	real_t autobox_up_ratio = 0.65f; \
	real_t autobox_down_ratio = 0.5f; \
	real_t autobox_latency = 2.0f; \
	real_t clutch_strength = 10.0f; \
	bool automatic_transmission = true; \
	int gear = 1; \
	real_t clutch = 0.0f; \
	bool clutch_assist = true; \
	void _engine_config(VehicleEngineConfig &r_config) const; \
	void _engine_controls(); \
	void _engine_validate_property(PropertyInfo &p_property) const; \
\
public:

#define PHYSX_VEHICLE_ENGINE_SETTER(m_class, m_name) \
	void m_class::set_##m_name(real_t p_v) { \
		m_name = p_v; \
		_rebuild_if_live(); \
	}

#define PHYSX_VEHICLE_ENGINE_DEFINE(m_class) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, engine_peak_torque) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, engine_max_rpm) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, engine_idle_rpm) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, engine_moment_of_inertia) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, engine_damping_full_throttle) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, engine_damping_zero_throttle_clutch_engaged) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, engine_damping_zero_throttle_clutch_disengaged) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, reverse_gear_ratio) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, final_drive_ratio) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, gear_switch_time) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, autobox_up_ratio) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, autobox_down_ratio) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, autobox_latency) \
	PHYSX_VEHICLE_ENGINE_SETTER(m_class, clutch_strength) \
	void m_class::set_drivetrain(Drivetrain p_drivetrain) { \
		if (drivetrain == p_drivetrain) { \
			return; \
		} \
		drivetrain = p_drivetrain; \
		notify_property_list_changed(); \
		_rebuild_if_live(); \
	} \
	void m_class::set_gear_ratios(const PackedFloat32Array &p_ratios) { \
		gear_ratios = p_ratios; \
		_rebuild_if_live(); \
	} \
	void m_class::set_engine_torque_curve(const Ref<Curve> &p_curve) { \
		if (engine_torque_curve == p_curve) { \
			return; \
		} \
		if (engine_torque_curve.is_valid()) { \
			engine_torque_curve->disconnect_changed(callable_mp(this, &m_class::_rebuild_if_live)); \
		} \
		engine_torque_curve = p_curve; \
		if (engine_torque_curve.is_valid()) { \
			engine_torque_curve->connect_changed(callable_mp(this, &m_class::_rebuild_if_live)); \
		} \
		_rebuild_if_live(); \
	} \
	int m_class::get_current_gear() const { \
		if (!impl->built || !impl->vehicle.engine_drive) { \
			return reverse ? -1 : 1; \
		} \
		return impl->vehicle.get_current_gear(); \
	} \
	int m_class::get_target_gear() const { \
		if (!impl->built || !impl->vehicle.engine_drive) { \
			return reverse ? -1 : 1; \
		} \
		return impl->vehicle.get_target_gear(); \
	} \
	real_t m_class::get_engine_rpm() const { \
		if (!impl->built || !impl->vehicle.engine_drive) { \
			return 0.0; \
		} \
		return impl->vehicle.get_engine_rpm(); \
	} \
	void m_class::_engine_config(VehicleEngineConfig &e) const { \
		e.enabled = true; \
		e.peak_torque = engine_peak_torque; \
		e.max_rpm = engine_max_rpm; \
		e.idle_rpm = engine_idle_rpm; \
		e.torque_curve.clear(); \
		if (engine_torque_curve.is_valid() && engine_torque_curve->get_point_count() > 0) { \
			/* PhysX's table holds 8 points: a curve with more is sampled at 8 even speeds. */ \
			const Ref<Curve> &curve = engine_torque_curve; \
			const real_t from = curve->get_min_domain(); \
			const real_t span = MAX(curve->get_max_domain() - from, (real_t)CMP_EPSILON); \
			const int count = curve->get_point_count(); \
			if (count <= 8) { \
				for (int i = 0; i < count; i++) { \
					const Vector2 point = curve->get_point_position(i); \
					e.torque_curve.push_back(Vector2((point.x - from) / span, point.y)); \
				} \
			} else { \
				for (int i = 0; i < 8; i++) { \
					const real_t x = i / 7.0; \
					e.torque_curve.push_back(Vector2(x, curve->sample(from + x * span))); \
				} \
			} \
		} \
		e.moment_of_inertia = engine_moment_of_inertia; \
		e.damping_full_throttle = engine_damping_full_throttle; \
		e.damping_zero_throttle_clutch_engaged = engine_damping_zero_throttle_clutch_engaged; \
		e.damping_zero_throttle_clutch_disengaged = engine_damping_zero_throttle_clutch_disengaged; \
		e.gear_ratios.clear(); \
		for (float ratio : gear_ratios) { \
			e.gear_ratios.push_back(ratio); \
		} \
		e.reverse_gear_ratio = reverse_gear_ratio; \
		e.final_drive_ratio = final_drive_ratio; \
		e.gear_switch_time = gear_switch_time; \
		e.autobox_up_ratio = autobox_up_ratio; \
		e.autobox_down_ratio = autobox_down_ratio; \
		e.autobox_latency = autobox_latency; \
		e.clutch_strength = clutch_strength; \
	} \
	void m_class::_engine_controls() { \
		if (impl->vehicle.engine_drive) { \
			impl->vehicle.set_engine_controls(automatic_transmission, gear, clutch, clutch_assist, reverse); \
		} \
	} \
	void m_class::_engine_validate_property(PropertyInfo &p_property) const { \
		/* Each drivetrain shows only its own settings. */ \
		const String name = p_property.name; \
		const bool engine_setting = name.begins_with("engine_") || name.begins_with("gear_") || name.begins_with("autobox_") || \
				name == "reverse_gear_ratio" || name == "final_drive_ratio" || name == "clutch_strength" || \
				name == "automatic_transmission" || name == "gear" || name == "clutch" || name == "clutch_assist"; \
		if ((drivetrain == DRIVETRAIN_ENGINE && name == "max_engine_torque") || (drivetrain != DRIVETRAIN_ENGINE && engine_setting)) { \
			p_property.usage = PROPERTY_USAGE_NO_EDITOR; \
		} \
	}

#define PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, m_name, m_hint) \
	ClassDB::bind_method(D_METHOD("set_" #m_name, "value"), &m_class::set_##m_name); \
	ClassDB::bind_method(D_METHOD("get_" #m_name), &m_class::get_##m_name); \
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, #m_name, PROPERTY_HINT_RANGE, m_hint), "set_" #m_name, "get_" #m_name);

#define PHYSX_VEHICLE_ENGINE_BIND_DRIVETRAIN(m_class) \
	BIND_ENUM_CONSTANT(DRIVETRAIN_DIRECT); \
	BIND_ENUM_CONSTANT(DRIVETRAIN_ENGINE); \
	ClassDB::bind_method(D_METHOD("set_drivetrain", "drivetrain"), &m_class::set_drivetrain); \
	ClassDB::bind_method(D_METHOD("get_drivetrain"), &m_class::get_drivetrain); \
	ADD_PROPERTY(PropertyInfo(Variant::INT, "drivetrain", PROPERTY_HINT_ENUM, "Direct,Engine"), "set_drivetrain", "get_drivetrain");

#define PHYSX_VEHICLE_ENGINE_BIND_SETTINGS(m_class) \
	ADD_GROUP("Engine", "engine_"); \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, engine_peak_torque, "0,5000,1,or_greater,suffix:N·m") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, engine_max_rpm, "100,20000,10,or_greater") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, engine_idle_rpm, "0,5000,10,or_greater") \
	ClassDB::bind_method(D_METHOD("set_engine_torque_curve", "curve"), &m_class::set_engine_torque_curve); \
	ClassDB::bind_method(D_METHOD("get_engine_torque_curve"), &m_class::get_engine_torque_curve); \
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "engine_torque_curve", PROPERTY_HINT_RESOURCE_TYPE, Curve::get_class_static()), "set_engine_torque_curve", "get_engine_torque_curve"); \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, engine_moment_of_inertia, "0.01,20,0.01,or_greater,suffix:kg·m²") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, engine_damping_full_throttle, "0,10,0.01,or_greater") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, engine_damping_zero_throttle_clutch_engaged, "0,10,0.01,or_greater") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, engine_damping_zero_throttle_clutch_disengaged, "0,10,0.01,or_greater") \
	ADD_GROUP("Gearbox", ""); \
	ClassDB::bind_method(D_METHOD("set_gear_ratios", "ratios"), &m_class::set_gear_ratios); \
	ClassDB::bind_method(D_METHOD("get_gear_ratios"), &m_class::get_gear_ratios); \
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "gear_ratios"), "set_gear_ratios", "get_gear_ratios"); \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, reverse_gear_ratio, "0.1,20,0.01,or_greater") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, final_drive_ratio, "0.1,20,0.01,or_greater") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, gear_switch_time, "0,5,0.01,or_greater,suffix:s") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, autobox_up_ratio, "0,1,0.01") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, autobox_down_ratio, "0,1,0.01") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, autobox_latency, "0,10,0.01,or_greater,suffix:s") \
	PHYSX_VEHICLE_ENGINE_BIND_FLOAT(m_class, clutch_strength, "0,100,0.1,or_greater")

#define PHYSX_VEHICLE_ENGINE_BIND_CONTROLS(m_class) \
	ClassDB::bind_method(D_METHOD("set_automatic_transmission", "value"), &m_class::set_automatic_transmission); \
	ClassDB::bind_method(D_METHOD("is_automatic_transmission"), &m_class::is_automatic_transmission); \
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "automatic_transmission"), "set_automatic_transmission", "is_automatic_transmission"); \
	ClassDB::bind_method(D_METHOD("set_gear", "gear"), &m_class::set_gear); \
	ClassDB::bind_method(D_METHOD("get_gear"), &m_class::get_gear); \
	ADD_PROPERTY(PropertyInfo(Variant::INT, "gear", PROPERTY_HINT_RANGE, "-1,30,1"), "set_gear", "get_gear"); \
	ClassDB::bind_method(D_METHOD("set_clutch", "value"), &m_class::set_clutch); \
	ClassDB::bind_method(D_METHOD("get_clutch"), &m_class::get_clutch); \
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "clutch", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_clutch", "get_clutch"); \
	ClassDB::bind_method(D_METHOD("set_clutch_assist", "value"), &m_class::set_clutch_assist); \
	ClassDB::bind_method(D_METHOD("is_clutch_assist"), &m_class::is_clutch_assist); \
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "clutch_assist"), "set_clutch_assist", "is_clutch_assist"); \
	ClassDB::bind_method(D_METHOD("get_current_gear"), &m_class::get_current_gear); \
	ClassDB::bind_method(D_METHOD("get_target_gear"), &m_class::get_target_gear); \
	ClassDB::bind_method(D_METHOD("get_engine_rpm"), &m_class::get_engine_rpm);

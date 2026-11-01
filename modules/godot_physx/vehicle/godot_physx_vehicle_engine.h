/**************************************************************************/
/*  godot_physx_vehicle_engine.h                                          */
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

#include "core/error/error_macros.h"
#include "core/math/math_funcs.h"
#include "core/math/vector2.h"
#include "core/string/ustring.h"
#include "core/templates/vector.h"
#include "core/typedefs.h"

#include <PxPhysicsAPI.h>
#include <vehicle/PxVehicleAPI.h>

using namespace physx;

// The engine drive's tuning, in Godot-friendly units: engine speeds in RPM, gears as positive ratios. Defaults are
// the PhysX vehicle snippets' own (EngineDrive.json), with the speeds rounded to RPM.
struct VehicleEngineConfig {
	bool enabled = false;
	real_t peak_torque = 500.0f; // N m
	real_t max_rpm = 6000.0f;
	real_t idle_rpm = 0.0f;
	// Normalized torque (of peak_torque) against normalized engine speed (of max_rpm), at most 8 points with
	// ascending x in [0, 1]. Empty = flat.
	Vector<Vector2> torque_curve;
	real_t moment_of_inertia = 1.0f;
	real_t damping_full_throttle = 0.15f;
	real_t damping_zero_throttle_clutch_engaged = 2.0f;
	real_t damping_zero_throttle_clutch_disengaged = 0.35f;
	// Forward gears, first to top: positive and descending. One reverse gear, then neutral.
	Vector<real_t> gear_ratios = { 4.0f, 2.0f, 1.5f, 1.1f, 1.0f };
	real_t reverse_gear_ratio = 4.0f;
	real_t final_drive_ratio = 4.0f;
	real_t gear_switch_time = 0.5f;
	// The autobox shifts up above up_ratio * max_rpm and down below down_ratio * max_rpm, at most once per latency.
	real_t autobox_up_ratio = 0.65f;
	real_t autobox_down_ratio = 0.5f;
	real_t autobox_latency = 2.0f;
	real_t clutch_strength = 10.0f;
};

// PhysX's engine drive (snippetvehiclecommon's EngineDrivetrain), for any of this module's wheeled vehicles: V is
// the vehicle class deriving from this (it holds the wheels' params and states the components read), N its wheel
// count. A vehicle drives one of two ways, picked when it's configured (engine_drive): direct drive (throttle
// straight to wheel torque) or this one (engine torque curve, clutch, gearbox, autobox, differential). The SDK's
// reference keeps those as two vehicle classes; here both component sets live on the one vehicle and its
// initComponentSequence() adds only the chosen one's (add_engine_drive_components() and add_engine_drivetrain()).
// V also provides get_engine_drive_ackermann(), its Ackermann params (or none).
template <class V, PxU32 N>
class VehicleEngineDrive : public PxVehicleEngineDriveCommandResponseComponent,
						   public PxVehicleMultiWheelDriveDifferentialStateComponent,
						   public PxVehicleFourWheelDriveDifferentialStateComponent,
						   public PxVehicleEngineDriveActuationStateComponent,
						   public PxVehicleEngineDrivetrainComponent {
	V &self() { return static_cast<V &>(*this); }

public:
	bool engine_drive = false;
	bool limited_slip = false; // four-wheel-drive differential instead of the open multi-wheel one (4 wheels only)
	PxVehicleAutoboxParams autoboxParams;
	PxVehicleClutchCommandResponseParams clutchCommandResponseParams;
	PxVehicleEngineParams engineParams;
	PxVehicleGearboxParams gearboxParams;
	PxVehicleMultiWheelDriveDifferentialParams multiWheelDifferentialParams;
	PxVehicleFourWheelDriveDifferentialParams fourWheelDifferentialParams;
	PxVehicleClutchParams clutchParams;
	PxVehicleEngineDriveThrottleCommandResponseState engineThrottleResponseState;
	PxVehicleAutoboxState autoboxState;
	PxVehicleClutchCommandResponseState clutchCommandResponseState;
	PxVehicleDifferentialState differentialState;
	PxVehicleWheelConstraintGroupState wheelConstraintGroupState;
	PxVehicleEngineState engineState;
	PxVehicleGearboxState gearboxState;
	PxVehicleClutchSlipState clutchSlipState;
	PxVehicleEngineDriveTransmissionCommandState engineTransmissionCommandState;

	// The engine idles and the drivetrain lets go (a teleport that stops the vehicle). The gear stays.
	void stop_drivetrain() {
		engineThrottleResponseState.setToDefault();
		autoboxState.setToDefault();
		clutchCommandResponseState.setToDefault();
		differentialState.setToDefault();
		wheelConstraintGroupState.setToDefault();
		clutchSlipState.setToDefault();
		engineState.setToDefault();
		engineState.rotationSpeed = engine_drive ? engineParams.idleOmega : 0.0f;
	}

	void reset_engine_drive() {
		stop_drivetrain();
		gearboxState.setToDefault();
		engineTransmissionCommandState.setToDefault();
	}

	// The nodes' gearbox controls, every tick. `gear` counts as the nodes do: -1 reverse, 0 neutral, 1.. forward.
	// automatic: the autobox picks the forward gears, holding its gear while the clutch is pressed past halfway
	// (it shifts on engine speed alone, so a vehicle revved against the clutch would climb through the gears
	// standing still). Manual: the gearbox goes to `gear`; without clutch_assist, only while the clutch is pressed
	// past halfway. reverse selects the reverse gear either way.
	void set_engine_controls(bool p_automatic, int p_gear, real_t p_clutch, bool p_clutch_assist, bool p_reverse) {
		// Gearbox gears: 0 reverse, 1 neutral, 2.. forward -- `gear` + 1.
		PxU32 target = PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR;
		if (p_reverse) {
			target = 0;
		} else if (!p_automatic) {
			target = (PxU32)CLAMP(p_gear + 1, 0, (int)gearboxParams.nbRatios - 1);
		}
		if (!p_clutch_assist && !p_automatic && p_clutch < 0.5f) {
			// No clutch, no shift: the gearbox stays in (or keeps shifting to) the gear it has.
			target = gearboxState.targetGear;
		} else if (p_automatic && !p_reverse && p_clutch >= 0.5f) {
			target = gearboxState.targetGear;
		}
		engineTransmissionCommandState.targetGear = target;
		engineTransmissionCommandState.clutch = (PxReal)CLAMP(p_clutch, (real_t)0.0, (real_t)1.0);
	}

	// Numbered as `gear`; 0 while a shift is under way.
	int get_current_gear() const { return (int)gearboxState.currentGear - 1; }
	int get_target_gear() const { return (int)gearboxState.targetGear - 1; }
	real_t get_engine_rpm() const { return engineState.rotationSpeed * (real_t)(60.0 / Math::TAU); }

	// Before the road queries: the autobox may start a shift here (overriding the target gear and cutting the
	// throttle meanwhile).
	void add_engine_drive_components(PxVehicleComponentSequence &r_sequence) {
		r_sequence.add(static_cast<PxVehicleEngineDriveCommandResponseComponent *>(this));
		if (limited_slip) {
			r_sequence.add(static_cast<PxVehicleFourWheelDriveDifferentialStateComponent *>(this));
		} else {
			r_sequence.add(static_cast<PxVehicleMultiWheelDriveDifferentialStateComponent *>(this));
		}
		r_sequence.add(static_cast<PxVehicleEngineDriveActuationStateComponent *>(this));
	}

	// In the substep group, where the direct drivetrain would go.
	void add_engine_drivetrain(PxVehicleComponentSequence &r_sequence) {
		r_sequence.add(static_cast<PxVehicleEngineDrivetrainComponent *>(this));
	}

	// getDataForEngineDriveCommandResponseComponent (PxVehicleEngineDriveCommandResponseComponent)
	virtual void getDataForEngineDriveCommandResponseComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			PxVehicleSizedArrayData<const PxVehicleBrakeCommandResponseParams> &outBrakeResponseParams,
			const PxVehicleSteerCommandResponseParams *&outSteerResponseParams,
			PxVehicleSizedArrayData<const PxVehicleAckermannParams> &outAckermannParams,
			const PxVehicleGearboxParams *&outGearboxParams,
			const PxVehicleClutchCommandResponseParams *&outClutchResponseParams,
			const PxVehicleEngineParams *&outEngineParams,
			const PxVehicleRigidBodyState *&outRigidBodyState,
			const PxVehicleEngineState *&outEngineState,
			const PxVehicleAutoboxParams *&outAutoboxParams,
			const PxVehicleCommandState *&outCommands,
			const PxVehicleEngineDriveTransmissionCommandState *&outTransmissionCommands,
			PxVehicleArrayData<PxReal> &outBrakeResponseStates,
			PxVehicleEngineDriveThrottleCommandResponseState *&outThrottleResponseState,
			PxVehicleArrayData<PxReal> &outSteerResponseStates,
			PxVehicleGearboxState *&outGearboxResponseState,
			PxVehicleClutchCommandResponseState *&outClutchResponseState,
			PxVehicleAutoboxState *&outAutoboxState) override {
		V &v = self();
		outAxleDescription = &v.axleDescription;
		outBrakeResponseParams.setDataAndCount(v.brakeResponseParams, 2);
		outSteerResponseParams = &v.steerResponseParams;
		v.get_engine_drive_ackermann(outAckermannParams);
		outGearboxParams = &gearboxParams;
		outClutchResponseParams = &clutchCommandResponseParams;
		outEngineParams = &engineParams;
		outRigidBodyState = &v.rigidBodyState;
		outEngineState = &engineState;
		outAutoboxParams = &autoboxParams;
		outCommands = &v.commandState;
		outTransmissionCommands = &engineTransmissionCommandState;
		outBrakeResponseStates.setData(v.brakeCommandResponseStates);
		outThrottleResponseState = &engineThrottleResponseState;
		outSteerResponseStates.setData(v.steerCommandResponseStates);
		outGearboxResponseState = &gearboxState;
		outClutchResponseState = &clutchCommandResponseState;
		outAutoboxState = &autoboxState;
	}

	// getDataForMultiWheelDriveDifferentialStateComponent (PxVehicleMultiWheelDriveDifferentialStateComponent)
	virtual void getDataForMultiWheelDriveDifferentialStateComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehicleMultiWheelDriveDifferentialParams *&outDifferentialParams,
			PxVehicleDifferentialState *&outDifferentialState) override {
		outAxleDescription = &self().axleDescription;
		outDifferentialParams = &multiWheelDifferentialParams;
		outDifferentialState = &differentialState;
	}

	// getDataForFourWheelDriveDifferentialStateComponent (PxVehicleFourWheelDriveDifferentialStateComponent)
	virtual void getDataForFourWheelDriveDifferentialStateComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehicleFourWheelDriveDifferentialParams *&outDifferentialParams,
			PxVehicleArrayData<const PxVehicleWheelRigidBody1dState> &outWheelRigidbody1dStates,
			PxVehicleDifferentialState *&outDifferentialState,
			PxVehicleWheelConstraintGroupState *&outWheelConstraintGroups) override {
		outAxleDescription = &self().axleDescription;
		outDifferentialParams = &fourWheelDifferentialParams;
		outWheelRigidbody1dStates.setData(self().wheelRigidBody1dStates);
		outDifferentialState = &differentialState;
		outWheelConstraintGroups = &wheelConstraintGroupState;
	}

	// getDataForEngineDriveActuationStateComponent (PxVehicleEngineDriveActuationStateComponent)
	virtual void getDataForEngineDriveActuationStateComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehicleGearboxParams *&outGearboxParams,
			PxVehicleArrayData<const PxReal> &outBrakeResponseStates,
			const PxVehicleEngineDriveThrottleCommandResponseState *&outThrottleResponseState,
			const PxVehicleGearboxState *&outGearboxState,
			const PxVehicleDifferentialState *&outDifferentialState,
			const PxVehicleClutchCommandResponseState *&outClutchResponseState,
			PxVehicleArrayData<PxVehicleWheelActuationState> &outActuationStates) override {
		V &v = self();
		outAxleDescription = &v.axleDescription;
		outGearboxParams = &gearboxParams;
		outBrakeResponseStates.setData(v.brakeCommandResponseStates);
		outThrottleResponseState = &engineThrottleResponseState;
		outGearboxState = &gearboxState;
		outDifferentialState = &differentialState;
		outClutchResponseState = &clutchCommandResponseState;
		outActuationStates.setData(v.actuationStates);
	}

	// getDataForEngineDrivetrainComponent (PxVehicleEngineDrivetrainComponent)
	virtual void getDataForEngineDrivetrainComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			PxVehicleArrayData<const PxVehicleWheelParams> &outWheelParams,
			const PxVehicleEngineParams *&outEngineParams,
			const PxVehicleClutchParams *&outClutchParams,
			const PxVehicleGearboxParams *&outGearboxParams,
			PxVehicleArrayData<const PxReal> &outBrakeResponseStates,
			PxVehicleArrayData<const PxVehicleWheelActuationState> &outActuationStates,
			PxVehicleArrayData<const PxVehicleTireForce> &outTireForces,
			const PxVehicleEngineDriveThrottleCommandResponseState *&outThrottleResponseState,
			const PxVehicleClutchCommandResponseState *&outClutchResponseState,
			const PxVehicleDifferentialState *&outDifferentialState,
			const PxVehicleWheelConstraintGroupState *&outConstraintGroupState,
			PxVehicleArrayData<PxVehicleWheelRigidBody1dState> &outWheelRigidBody1dStates,
			PxVehicleEngineState *&outEngineState,
			PxVehicleGearboxState *&outGearboxState,
			PxVehicleClutchSlipState *&outClutchState) override {
		V &v = self();
		outAxleDescription = &v.axleDescription;
		outWheelParams.setData(v.wheelParams);
		outEngineParams = &engineParams;
		outClutchParams = &clutchParams;
		outGearboxParams = &gearboxParams;
		outBrakeResponseStates.setData(v.brakeCommandResponseStates);
		outActuationStates.setData(v.actuationStates);
		outTireForces.setData(v.tireForces);
		outThrottleResponseState = &engineThrottleResponseState;
		outClutchResponseState = &clutchCommandResponseState;
		outDifferentialState = &differentialState;
		// The limited slip differential works through these wheel groups. (The SDK's own EngineDriveVehicle passes
		// them only for a tank, so its four-wheel-drive differential never limits slip.)
		outConstraintGroupState = limited_slip ? &wheelConstraintGroupState : nullptr;
		outWheelRigidBody1dStates.setData(v.wheelRigidBody1dStates);
		outEngineState = &engineState;
		outGearboxState = &gearboxState;
		outClutchState = &clutchSlipState;
	}

	// Fills the engine drive params from e, the torque shared evenly between the wheels with traction[i]. The
	// differential is the open one; a 4-wheel vehicle sets up limited slip on top. PhysX checks these only in its debug
	// builds, so what a scene can get wrong is checked here.
	bool configure_engine(const VehicleEngineConfig &e, const bool traction[N]) {
		engine_drive = true;
		limited_slip = false;
		const PxReal rpm_to_omega = (PxReal)(Math::TAU / 60.0);

		if (e.peak_torque < 0.0 || e.idle_rpm < 0.0 || e.max_rpm <= e.idle_rpm || e.moment_of_inertia <= 0.0) {
			ERR_PRINT("PhysX vehicle: the engine needs peak_torque >= 0, 0 <= idle_rpm < max_rpm and a moment of inertia > 0.");
			return false;
		}
		PxVehicleEngineParams &engine = engineParams;
		engine.torqueCurve.clear();
		if (e.torque_curve.is_empty()) {
			engine.torqueCurve.addPair(0.0f, 1.0f);
			engine.torqueCurve.addPair(0.33f, 1.0f);
			engine.torqueCurve.addPair(1.0f, 1.0f);
		} else {
			if (e.torque_curve.size() > PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES) {
				ERR_PRINT(vformat("PhysX vehicle: the engine torque curve has %d points; PhysX takes at most %d.", e.torque_curve.size(), (int)PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES));
				return false;
			}
			for (int i = 0; i < e.torque_curve.size(); i++) {
				if (i > 0 && e.torque_curve[i].x <= e.torque_curve[i - 1].x) {
					ERR_PRINT("PhysX vehicle: the engine torque curve's points need ascending engine speeds.");
					return false;
				}
				engine.torqueCurve.addPair((PxReal)e.torque_curve[i].x, (PxReal)MAX(e.torque_curve[i].y, (real_t)0.0));
			}
		}
		engine.moi = (PxReal)e.moment_of_inertia;
		engine.peakTorque = (PxReal)e.peak_torque;
		engine.idleOmega = (PxReal)e.idle_rpm * rpm_to_omega;
		engine.maxOmega = (PxReal)e.max_rpm * rpm_to_omega;
		engine.dampingRateFullThrottle = (PxReal)MAX(e.damping_full_throttle, (real_t)0.0);
		engine.dampingRateZeroThrottleClutchEngaged = (PxReal)MAX(e.damping_zero_throttle_clutch_engaged, (real_t)0.0);
		engine.dampingRateZeroThrottleClutchDisengaged = (PxReal)MAX(e.damping_zero_throttle_clutch_disengaged, (real_t)0.0);

		// Gears: [0] reverse, [1] neutral, then the forward gears.
		const int forward = e.gear_ratios.size();
		if (forward < 1 || forward + 2 > (int)PxVehicleGearboxParams::eMAX_NB_GEARS) {
			ERR_PRINT(vformat("PhysX vehicle: the gearbox needs 1 to %d forward gear ratios (has %d).", (int)PxVehicleGearboxParams::eMAX_NB_GEARS - 2, forward));
			return false;
		}
		if (e.reverse_gear_ratio <= 0.0 || e.final_drive_ratio <= 0.0 || e.gear_switch_time < 0.0) {
			ERR_PRINT("PhysX vehicle: the reverse gear and final drive ratios must be > 0 and the gear switch time >= 0.");
			return false;
		}
		PxVehicleGearboxParams &gears = gearboxParams;
		gears.neutralGear = 1;
		gears.nbRatios = (PxU32)forward + 2;
		gears.ratios[0] = -(PxReal)e.reverse_gear_ratio;
		gears.ratios[1] = 0.0f;
		for (int i = 0; i < forward; i++) {
			if (e.gear_ratios[i] <= 0.0 || (i > 0 && e.gear_ratios[i] >= e.gear_ratios[i - 1])) {
				ERR_PRINT("PhysX vehicle: forward gear ratios must be > 0 and descending, first gear to top.");
				return false;
			}
			gears.ratios[i + 2] = (PxReal)e.gear_ratios[i];
		}
		gears.finalRatio = (PxReal)e.final_drive_ratio;
		gears.switchTime = (PxReal)e.gear_switch_time;

		for (PxU32 i = 0; i < PxVehicleGearboxParams::eMAX_NB_GEARS; i++) {
			autoboxParams.upRatios[i] = (PxReal)MAX(e.autobox_up_ratio, (real_t)0.0);
			autoboxParams.downRatios[i] = (PxReal)MAX(e.autobox_down_ratio, (real_t)0.0);
		}
		autoboxParams.latency = (PxReal)MAX(e.autobox_latency, (real_t)0.0);

		clutchCommandResponseParams.maxResponse = (PxReal)MAX(e.clutch_strength, (real_t)0.0);
		clutchParams.accuracyMode = PxVehicleClutchAccuracyMode::eBEST_POSSIBLE;
		clutchParams.estimateIterations = 5;

		// The engine's torque is shared evenly between the traction wheels.
		int driven = 0;
		for (PxU32 i = 0; i < N; i++) {
			driven += traction[i] ? 1 : 0;
		}
		if (driven == 0) {
			ERR_PRINT("PhysX vehicle: the engine drive needs at least one wheel with use_as_traction.");
			return false;
		}
		PxVehicleMultiWheelDriveDifferentialParams &multi = multiWheelDifferentialParams;
		multi.setToDefault();
		for (PxU32 i = 0; i < N; i++) {
			multi.torqueRatios[i] = traction[i] ? 1.0f / driven : 0.0f;
			multi.aveWheelSpeedRatios[i] = multi.torqueRatios[i];
		}

		// Start idling in first gear (the PhysX snippets' start), the gearbox in automatic until told otherwise.
		gearboxState.setToDefault();
		gearboxState.currentGear = gears.neutralGear + 1;
		gearboxState.targetGear = gears.neutralGear + 1;
		engineTransmissionCommandState.setToDefault();
		engineTransmissionCommandState.targetGear = PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR;
		stop_drivetrain();
		return true;
	}
};

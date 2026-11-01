/**************************************************************************/
/*  godot_physx_vehicle4w.h                                               */
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

#include "../godot_physx_conversions.h"
#include "../objects/godot_physx_material.h"
#include "godot_physx_wheel_query_filter.h"
#include "../spaces/godot_physx_space_3d.h"

#include "core/error/error_macros.h"
#include "core/math/basis.h"
#include "core/math/vector3.h"
#include "core/templates/vector.h"
#include "core/typedefs.h"

#include <PxPhysicsAPI.h>
#include <vehicle/PxVehicleAPI.h>

using namespace physx;

// A single 4-wheel vehicle, composed exactly the way PhysX's own
// snippetvehiclecommon (Base/DirectDrivetrain/EngineDrivetrain/PhysXIntegration) composes one --
// ported here rather than linked, since that layer is snippet source, not part
// of the installed PhysXVehicle_static_64 lib (which only supplies the real
// per-component math these getDataForXComponent overrides hand pointers into).
// Wheel index convention: 0=FL, 1=FR, 2=RL, 3=RR. Shared verbatim between
// GodotPhysXVehicleProbe (headless regression) and PhysXVehicle3D (the real
// node) -- this class and configure_vehicle4w() below are pure PxVehicle2
// composition with no project-specific decisions in them, so duplicating them
// would just be ~500 lines of copy-pasted boilerplate for no benefit.
// It drives one of two ways, picked when it's configured (engine_drive): direct
// drive (throttle straight to wheel torque, the default) or the SDK's engine
// drive (engine torque curve, clutch, gearbox, autobox, differential). The SDK's
// reference keeps those as two vehicle classes; here both component sets live
// on the one class and initComponentSequence() adds only the chosen one's.
class Vehicle4W : public PxVehicleRigidBodyComponent,
				  public PxVehicleSuspensionComponent,
				  public PxVehicleTireComponent,
				  public PxVehicleWheelComponent,
				  public PxVehiclePhysXActorBeginComponent,
				  public PxVehiclePhysXActorEndComponent,
				  public PxVehiclePhysXConstraintComponent,
				  public PxVehiclePhysXRoadGeometrySceneQueryComponent,
				  public PxVehicleDirectDriveCommandResponseComponent,
				  public PxVehicleDirectDriveActuationStateComponent,
				  public PxVehicleDirectDrivetrainComponent,
				  public PxVehicleEngineDriveCommandResponseComponent,
				  public PxVehicleMultiWheelDriveDifferentialStateComponent,
				  public PxVehicleFourWheelDriveDifferentialStateComponent,
				  public PxVehicleEngineDriveActuationStateComponent,
				  public PxVehicleEngineDrivetrainComponent {
public:
	static constexpr PxU32 WHEEL_FL = 0;
	static constexpr PxU32 WHEEL_FR = 1;
	static constexpr PxU32 WHEEL_RL = 2;
	static constexpr PxU32 WHEEL_RR = 3;

	// --- Base (BaseVehicleParams/State) ------------------------------------
	PxVehicleAxleDescription axleDescription;
	PxVehicleFrame frame;
	PxVehicleScale scale;
	PxVehicleSuspensionStateCalculationParams suspensionStateCalculationParams;
	PxVehicleBrakeCommandResponseParams brakeResponseParams[2];
	PxVehicleSteerCommandResponseParams steerResponseParams;
	PxVehicleAckermannParams ackermannParams[1];
	PxVehicleSuspensionParams suspensionParams[4];
	PxVehicleSuspensionComplianceParams suspensionComplianceParams[4];
	PxVehicleSuspensionForceParams suspensionForceParams[4];
	PxVehicleTireForceParams tireForceParams[4];
	PxVehicleWheelParams wheelParams[4];
	PxVehicleRigidBodyParams rigidBodyParams;

	PxReal brakeCommandResponseStates[4] = {};
	PxReal steerCommandResponseStates[4] = {};
	PxVehicleWheelActuationState actuationStates[4];
	PxVehicleRoadGeometryState roadGeomStates[4];
	PxVehicleSuspensionState suspensionStates[4];
	PxVehicleSuspensionComplianceState suspensionComplianceStates[4];
	PxVehicleSuspensionForce suspensionForces[4];
	PxVehicleTireGripState tireGripStates[4];
	PxVehicleTireDirectionState tireDirectionStates[4];
	PxVehicleTireSpeedState tireSpeedStates[4];
	PxVehicleTireSlipState tireSlipStates[4];
	PxVehicleTireCamberAngleState tireCamberAngleStates[4];
	PxVehicleTireStickyState tireStickyStates[4];
	PxVehicleTireForce tireForces[4];
	PxVehicleWheelRigidBody1dState wheelRigidBody1dStates[4];
	PxVehicleWheelLocalPose wheelLocalPoses[4];
	PxVehicleRigidBodyState rigidBodyState;

	// Anti-roll bars: [0] connects WHEEL_FL/WHEEL_FR (front axle), [1]
	// connects WHEEL_RL/WHEEL_RR (rear axle). Always submitted (even at
	// stiffness=0) so the wiring in getDataForSuspensionComponent/
	// getDataForRigidBodyComponent stays unconditional -- see
	// PxVehicleAntiRollForceParams's own isValid(), zero stiffness is valid
	// and just contributes zero torque.
	PxVehicleAntiRollForceParams antiRollForceParams[2];
	PxVehicleAntiRollTorque antiRollTorque;

	// --- Direct drive (DirectDrivetrainParams/State) -----------------------
	PxVehicleDirectDriveThrottleCommandResponseParams directDriveThrottleResponseParams;
	PxReal directDriveThrottleResponseStates[4] = {};
	PxVehicleCommandState commandState;
	PxVehicleDirectDriveTransmissionCommandState transmissionCommandState;

	// --- Engine drive (EngineDrivetrainParams/State) -----------------------
	bool engine_drive = false;
	bool limited_slip = false; // four-wheel-drive differential instead of the open multi-wheel one
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

	// --- PhysX integration (PhysXIntegrationParams/State) ------------------
	PxVehiclePhysXRoadGeometryQueryParams physxRoadGeometryQueryParams;
	GodotPhysXWheelQueryFilter wheelQueryFilter; // physxRoadGeometryQueryParams.filterCallback points here
	PxVehiclePhysXMaterialFrictionParams physxMaterialFrictionParams[4];
	PxVehiclePhysXSuspensionLimitConstraintParams physxSuspensionLimitConstraintParams[4];
	PxTransform physxActorCMassLocalPose;
	PxVec3 physxActorBoxShapeHalfExtents;
	PxTransform physxActorBoxShapeLocalPose;
	PxTransform physxWheelShapeLocalPoses[4];
	PxVehiclePhysXActor physxActor;
	PxVehiclePhysXSteerState physxSteerState;
	PxVehiclePhysXConstraints physxConstraints;

	PxVehicleComponentSequence componentSequence;
	PxU8 componentSequenceSubstepGroupHandle = 0;

	void setToDefault() {
		for (PxU32 i = 0; i < 4; i++) {
			actuationStates[i].setToDefault();
			roadGeomStates[i].setToDefault();
			suspensionStates[i].setToDefault();
			suspensionComplianceStates[i].setToDefault();
			suspensionForces[i].setToDefault();
			tireGripStates[i].setToDefault();
			tireDirectionStates[i].setToDefault();
			tireSpeedStates[i].setToDefault();
			tireSlipStates[i].setToDefault();
			tireCamberAngleStates[i].setToDefault();
			tireStickyStates[i].setToDefault();
			tireForces[i].setToDefault();
			wheelRigidBody1dStates[i].setToDefault();
			wheelLocalPoses[i].setToDefault();
		}
		rigidBodyState.setToDefault();
		antiRollTorque.setToDefault();
		commandState.setToDefault();
		transmissionCommandState.gear = PxVehicleDirectDriveTransmissionCommandState::eNEUTRAL;
		stop_drivetrain();
		gearboxState.setToDefault();
		engineTransmissionCommandState.setToDefault();
		physxActor.setToDefault();
		physxSteerState.setToDefault();
		physxConstraints.setToDefault();
	}

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

	// getDataForRigidBodyComponent (PxVehicleRigidBodyComponent)
	virtual void getDataForRigidBodyComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehicleRigidBodyParams *&outRigidBodyParams,
			PxVehicleArrayData<const PxVehicleSuspensionForce> &outSuspensionForces,
			PxVehicleArrayData<const PxVehicleTireForce> &outTireForces,
			const PxVehicleAntiRollTorque *&outAntiRollTorque,
			PxVehicleRigidBodyState *&outRigidBodyState) override {
		outAxleDescription = &axleDescription;
		outRigidBodyParams = &rigidBodyParams;
		outSuspensionForces.setData(suspensionForces);
		outTireForces.setData(tireForces);
		outAntiRollTorque = &antiRollTorque;
		outRigidBodyState = &rigidBodyState;
	}

	// getDataForSuspensionComponent (PxVehicleSuspensionComponent)
	virtual void getDataForSuspensionComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehicleRigidBodyParams *&outRigidBodyParams,
			const PxVehicleSuspensionStateCalculationParams *&outSuspensionStateCalculationParams,
			PxVehicleArrayData<const PxReal> &outSteerResponseStates,
			const PxVehicleRigidBodyState *&outRigidBodyState,
			PxVehicleArrayData<const PxVehicleWheelParams> &outWheelParams,
			PxVehicleArrayData<const PxVehicleSuspensionParams> &outSuspensionParams,
			PxVehicleArrayData<const PxVehicleSuspensionComplianceParams> &outSuspensionComplianceParams,
			PxVehicleArrayData<const PxVehicleSuspensionForceParams> &outSuspensionForceParams,
			PxVehicleSizedArrayData<const PxVehicleAntiRollForceParams> &outAntiRollForceParams,
			PxVehicleArrayData<const PxVehicleRoadGeometryState> &outWheelRoadGeomStates,
			PxVehicleArrayData<PxVehicleSuspensionState> &outSuspensionStates,
			PxVehicleArrayData<PxVehicleSuspensionComplianceState> &outSuspensionComplianceStates,
			PxVehicleArrayData<PxVehicleSuspensionForce> &outSuspensionForces,
			PxVehicleAntiRollTorque *&outAntiRollTorque) override {
		outAxleDescription = &axleDescription;
		outRigidBodyParams = &rigidBodyParams;
		outSuspensionStateCalculationParams = &suspensionStateCalculationParams;
		outSteerResponseStates.setData(steerCommandResponseStates);
		outRigidBodyState = &rigidBodyState;
		outWheelParams.setData(wheelParams);
		outSuspensionParams.setData(suspensionParams);
		outSuspensionComplianceParams.setData(suspensionComplianceParams);
		outSuspensionForceParams.setData(suspensionForceParams);
		outAntiRollForceParams.setDataAndCount(antiRollForceParams, 2);
		outWheelRoadGeomStates.setData(roadGeomStates);
		outSuspensionStates.setData(suspensionStates);
		outSuspensionComplianceStates.setData(suspensionComplianceStates);
		outSuspensionForces.setData(suspensionForces);
		outAntiRollTorque = &antiRollTorque;
	}

	// getDataForTireComponent (PxVehicleTireComponent)
	virtual void getDataForTireComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			PxVehicleArrayData<const PxReal> &outSteerResponseStates,
			const PxVehicleRigidBodyState *&outRigidBodyState,
			PxVehicleArrayData<const PxVehicleWheelActuationState> &outActuationStates,
			PxVehicleArrayData<const PxVehicleWheelParams> &outWheelParams,
			PxVehicleArrayData<const PxVehicleSuspensionParams> &outSuspensionParams,
			PxVehicleArrayData<const PxVehicleTireForceParams> &outTireForceParams,
			PxVehicleArrayData<const PxVehicleRoadGeometryState> &outRoadGeomStates,
			PxVehicleArrayData<const PxVehicleSuspensionState> &outSuspensionStates,
			PxVehicleArrayData<const PxVehicleSuspensionComplianceState> &outSuspensionComplianceStates,
			PxVehicleArrayData<const PxVehicleSuspensionForce> &outSuspensionForces,
			PxVehicleArrayData<const PxVehicleWheelRigidBody1dState> &outWheelRigidBody1DStates,
			PxVehicleArrayData<PxVehicleTireGripState> &outTireGripStates,
			PxVehicleArrayData<PxVehicleTireDirectionState> &outTireDirectionStates,
			PxVehicleArrayData<PxVehicleTireSpeedState> &outTireSpeedStates,
			PxVehicleArrayData<PxVehicleTireSlipState> &outTireSlipStates,
			PxVehicleArrayData<PxVehicleTireCamberAngleState> &outTireCamberAngleStates,
			PxVehicleArrayData<PxVehicleTireStickyState> &outTireStickyStates,
			PxVehicleArrayData<PxVehicleTireForce> &outTireForces) override {
		outAxleDescription = &axleDescription;
		outSteerResponseStates.setData(steerCommandResponseStates);
		outRigidBodyState = &rigidBodyState;
		outActuationStates.setData(actuationStates);
		outWheelParams.setData(wheelParams);
		outSuspensionParams.setData(suspensionParams);
		outTireForceParams.setData(tireForceParams);
		outRoadGeomStates.setData(roadGeomStates);
		outSuspensionStates.setData(suspensionStates);
		outSuspensionComplianceStates.setData(suspensionComplianceStates);
		outSuspensionForces.setData(suspensionForces);
		outWheelRigidBody1DStates.setData(wheelRigidBody1dStates);
		outTireGripStates.setData(tireGripStates);
		outTireDirectionStates.setData(tireDirectionStates);
		outTireSpeedStates.setData(tireSpeedStates);
		outTireSlipStates.setData(tireSlipStates);
		outTireCamberAngleStates.setData(tireCamberAngleStates);
		outTireStickyStates.setData(tireStickyStates);
		outTireForces.setData(tireForces);
	}

	// getDataForWheelComponent (PxVehicleWheelComponent)
	virtual void getDataForWheelComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			PxVehicleArrayData<const PxReal> &outSteerResponseStates,
			PxVehicleArrayData<const PxVehicleWheelParams> &outWheelParams,
			PxVehicleArrayData<const PxVehicleSuspensionParams> &outSuspensionParams,
			PxVehicleArrayData<const PxVehicleWheelActuationState> &outActuationStates,
			PxVehicleArrayData<const PxVehicleSuspensionState> &outSuspensionStates,
			PxVehicleArrayData<const PxVehicleSuspensionComplianceState> &outSuspensionComplianceStates,
			PxVehicleArrayData<const PxVehicleTireSpeedState> &outTireSpeedStates,
			PxVehicleArrayData<PxVehicleWheelRigidBody1dState> &outWheelRigidBody1dStates,
			PxVehicleArrayData<PxVehicleWheelLocalPose> &outWheelLocalPoses) override {
		outAxleDescription = &axleDescription;
		outSteerResponseStates.setData(steerCommandResponseStates);
		outWheelParams.setData(wheelParams);
		outSuspensionParams.setData(suspensionParams);
		outActuationStates.setData(actuationStates);
		outSuspensionStates.setData(suspensionStates);
		outSuspensionComplianceStates.setData(suspensionComplianceStates);
		outTireSpeedStates.setData(tireSpeedStates);
		outWheelRigidBody1dStates.setData(wheelRigidBody1dStates);
		outWheelLocalPoses.setData(wheelLocalPoses);
	}

	// getDataForPhysXActorBeginComponent (PxVehiclePhysXActorBeginComponent)
	virtual void getDataForPhysXActorBeginComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehicleCommandState *&outCommands,
			const PxVehicleEngineDriveTransmissionCommandState *&outTransmissionCommands,
			const PxVehicleGearboxParams *&outGearParams,
			const PxVehicleGearboxState *&outGearState,
			const PxVehicleEngineParams *&outEngineParams,
			PxVehiclePhysXActor *&outPhysxActor,
			PxVehiclePhysXSteerState *&outPhysxSteerState,
			PxVehiclePhysXConstraints *&outPhysxConstraints,
			PxVehicleRigidBodyState *&outRigidBodyState,
			PxVehicleArrayData<PxVehicleWheelRigidBody1dState> &outWheelRigidBody1dStates,
			PxVehicleEngineState *&outEngineState) override {
		outAxleDescription = &axleDescription;
		outCommands = &commandState;
		outPhysxActor = &physxActor;
		outPhysxSteerState = &physxSteerState;
		outPhysxConstraints = &physxConstraints;
		outRigidBodyState = &rigidBodyState;
		outWheelRigidBody1dStates.setData(wheelRigidBody1dStates);
		outTransmissionCommands = engine_drive ? &engineTransmissionCommandState : nullptr;
		outGearParams = engine_drive ? &gearboxParams : nullptr;
		outGearState = engine_drive ? &gearboxState : nullptr;
		outEngineParams = engine_drive ? &engineParams : nullptr;
		outEngineState = engine_drive ? &engineState : nullptr;
	}

	// getDataForPhysXActorEndComponent (PxVehiclePhysXActorEndComponent)
	virtual void getDataForPhysXActorEndComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehicleRigidBodyState *&outRigidBodyState,
			PxVehicleArrayData<const PxVehicleWheelParams> &outWheelParams,
			PxVehicleArrayData<const PxTransform> &outWheelShapeLocalPoses,
			PxVehicleArrayData<const PxVehicleWheelRigidBody1dState> &outWheelRigidBody1dStates,
			PxVehicleArrayData<const PxVehicleWheelLocalPose> &outWheelLocalPoses,
			const PxVehicleGearboxState *&outGearState,
			const PxReal *&outThrottle,
			PxVehiclePhysXActor *&outPhysxActor) override {
		outAxleDescription = &axleDescription;
		outRigidBodyState = &rigidBodyState;
		outWheelParams.setData(wheelParams);
		outWheelShapeLocalPoses.setData(physxWheelShapeLocalPoses);
		outWheelRigidBody1dStates.setData(wheelRigidBody1dStates);
		outWheelLocalPoses.setData(wheelLocalPoses);
		outPhysxActor = &physxActor;
		outGearState = engine_drive ? &gearboxState : nullptr;
		outThrottle = &commandState.throttle;
	}

	// getDataForPhysXConstraintComponent (PxVehiclePhysXConstraintComponent)
	virtual void getDataForPhysXConstraintComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehicleRigidBodyState *&outRigidBodyState,
			PxVehicleArrayData<const PxVehicleSuspensionParams> &outSuspensionParams,
			PxVehicleArrayData<const PxVehiclePhysXSuspensionLimitConstraintParams> &outSuspensionLimitParams,
			PxVehicleArrayData<const PxVehicleSuspensionState> &outSuspensionStates,
			PxVehicleArrayData<const PxVehicleSuspensionComplianceState> &outSuspensionComplianceStates,
			PxVehicleArrayData<const PxVehicleRoadGeometryState> &outWheelRoadGeomStates,
			PxVehicleArrayData<const PxVehicleTireDirectionState> &outTireDirectionStates,
			PxVehicleArrayData<const PxVehicleTireStickyState> &outTireStickyStates,
			PxVehiclePhysXConstraints *&outConstraints) override {
		outAxleDescription = &axleDescription;
		outRigidBodyState = &rigidBodyState;
		outSuspensionParams.setData(suspensionParams);
		outSuspensionLimitParams.setData(physxSuspensionLimitConstraintParams);
		outSuspensionStates.setData(suspensionStates);
		outSuspensionComplianceStates.setData(suspensionComplianceStates);
		outWheelRoadGeomStates.setData(roadGeomStates);
		outTireDirectionStates.setData(tireDirectionStates);
		outTireStickyStates.setData(tireStickyStates);
		outConstraints = &physxConstraints;
	}

	// getDataForPhysXRoadGeometrySceneQueryComponent (PxVehiclePhysXRoadGeometrySceneQueryComponent)
	virtual void getDataForPhysXRoadGeometrySceneQueryComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehiclePhysXRoadGeometryQueryParams *&outRoadGeomParams,
			PxVehicleArrayData<const PxReal> &outSteerResponseStates,
			const PxVehicleRigidBodyState *&outRigidBodyState,
			PxVehicleArrayData<const PxVehicleWheelParams> &outWheelParams,
			PxVehicleArrayData<const PxVehicleSuspensionParams> &outSuspensionParams,
			PxVehicleArrayData<const PxVehiclePhysXMaterialFrictionParams> &outMaterialFrictionParams,
			PxVehicleArrayData<PxVehicleRoadGeometryState> &outRoadGeometryStates,
			PxVehicleArrayData<PxVehiclePhysXRoadGeometryQueryState> &outPhysxRoadGeometryStates) override {
		outAxleDescription = &axleDescription;
		outRoadGeomParams = &physxRoadGeometryQueryParams;
		outSteerResponseStates.setData(steerCommandResponseStates);
		outRigidBodyState = &rigidBodyState;
		outWheelParams.setData(wheelParams);
		outSuspensionParams.setData(suspensionParams);
		outMaterialFrictionParams.setData(physxMaterialFrictionParams);
		outRoadGeometryStates.setData(roadGeomStates);
		outPhysxRoadGeometryStates.setEmpty();
	}

	// getDataForDirectDriveCommandResponseComponent (PxVehicleDirectDriveCommandResponseComponent)
	virtual void getDataForDirectDriveCommandResponseComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			PxVehicleSizedArrayData<const PxVehicleBrakeCommandResponseParams> &outBrakeResponseParams,
			const PxVehicleDirectDriveThrottleCommandResponseParams *&outThrottleResponseParams,
			const PxVehicleSteerCommandResponseParams *&outSteerResponseParams,
			PxVehicleSizedArrayData<const PxVehicleAckermannParams> &outAckermannParams,
			const PxVehicleCommandState *&outCommands,
			const PxVehicleDirectDriveTransmissionCommandState *&outTransmissionCommands,
			const PxVehicleRigidBodyState *&outRigidBodyState,
			PxVehicleArrayData<PxReal> &outBrakeResponseStates,
			PxVehicleArrayData<PxReal> &outThrottleResponseStates,
			PxVehicleArrayData<PxReal> &outSteerResponseStates) override {
		outAxleDescription = &axleDescription;
		outBrakeResponseParams.setDataAndCount(brakeResponseParams, 2);
		outThrottleResponseParams = &directDriveThrottleResponseParams;
		outSteerResponseParams = &steerResponseParams;
		outAckermannParams.setDataAndCount(ackermannParams, 1);
		outCommands = &commandState;
		outTransmissionCommands = &transmissionCommandState;
		outRigidBodyState = &rigidBodyState;
		outBrakeResponseStates.setData(brakeCommandResponseStates);
		outThrottleResponseStates.setData(directDriveThrottleResponseStates);
		outSteerResponseStates.setData(steerCommandResponseStates);
	}

	// getDataForDirectDriveActuationStateComponent (PxVehicleDirectDriveActuationStateComponent)
	virtual void getDataForDirectDriveActuationStateComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			PxVehicleArrayData<const PxReal> &outBrakeResponseStates,
			PxVehicleArrayData<const PxReal> &outThrottleResponseStates,
			PxVehicleArrayData<PxVehicleWheelActuationState> &outActuationStates) override {
		outAxleDescription = &axleDescription;
		outBrakeResponseStates.setData(brakeCommandResponseStates);
		outThrottleResponseStates.setData(directDriveThrottleResponseStates);
		outActuationStates.setData(actuationStates);
	}

	// getDataForDirectDrivetrainComponent (PxVehicleDirectDrivetrainComponent)
	virtual void getDataForDirectDrivetrainComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			PxVehicleArrayData<const PxReal> &outBrakeResponseStates,
			PxVehicleArrayData<const PxReal> &outThrottleResponseStates,
			PxVehicleArrayData<const PxVehicleWheelParams> &outWheelParams,
			PxVehicleArrayData<const PxVehicleWheelActuationState> &outActuationStates,
			PxVehicleArrayData<const PxVehicleTireForce> &outTireForces,
			PxVehicleArrayData<PxVehicleWheelRigidBody1dState> &outWheelRigidBody1dStates) override {
		outAxleDescription = &axleDescription;
		outBrakeResponseStates.setData(brakeCommandResponseStates);
		outThrottleResponseStates.setData(directDriveThrottleResponseStates);
		outWheelParams.setData(wheelParams);
		outActuationStates.setData(actuationStates);
		outTireForces.setData(tireForces);
		outWheelRigidBody1dStates.setData(wheelRigidBody1dStates);
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
		outAxleDescription = &axleDescription;
		outBrakeResponseParams.setDataAndCount(brakeResponseParams, 2);
		outSteerResponseParams = &steerResponseParams;
		outAckermannParams.setDataAndCount(ackermannParams, 1);
		outGearboxParams = &gearboxParams;
		outClutchResponseParams = &clutchCommandResponseParams;
		outEngineParams = &engineParams;
		outRigidBodyState = &rigidBodyState;
		outEngineState = &engineState;
		outAutoboxParams = &autoboxParams;
		outCommands = &commandState;
		outTransmissionCommands = &engineTransmissionCommandState;
		outBrakeResponseStates.setData(brakeCommandResponseStates);
		outThrottleResponseState = &engineThrottleResponseState;
		outSteerResponseStates.setData(steerCommandResponseStates);
		outGearboxResponseState = &gearboxState;
		outClutchResponseState = &clutchCommandResponseState;
		outAutoboxState = &autoboxState;
	}

	// getDataForMultiWheelDriveDifferentialStateComponent (PxVehicleMultiWheelDriveDifferentialStateComponent)
	virtual void getDataForMultiWheelDriveDifferentialStateComponent(
			const PxVehicleAxleDescription *&outAxleDescription,
			const PxVehicleMultiWheelDriveDifferentialParams *&outDifferentialParams,
			PxVehicleDifferentialState *&outDifferentialState) override {
		outAxleDescription = &axleDescription;
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
		outAxleDescription = &axleDescription;
		outDifferentialParams = &fourWheelDifferentialParams;
		outWheelRigidbody1dStates.setData(wheelRigidBody1dStates);
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
		outAxleDescription = &axleDescription;
		outGearboxParams = &gearboxParams;
		outBrakeResponseStates.setData(brakeCommandResponseStates);
		outThrottleResponseState = &engineThrottleResponseState;
		outGearboxState = &gearboxState;
		outDifferentialState = &differentialState;
		outClutchResponseState = &clutchCommandResponseState;
		outActuationStates.setData(actuationStates);
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
		outAxleDescription = &axleDescription;
		outWheelParams.setData(wheelParams);
		outEngineParams = &engineParams;
		outClutchParams = &clutchParams;
		outGearboxParams = &gearboxParams;
		outBrakeResponseStates.setData(brakeCommandResponseStates);
		outActuationStates.setData(actuationStates);
		outTireForces.setData(tireForces);
		outThrottleResponseState = &engineThrottleResponseState;
		outClutchResponseState = &clutchCommandResponseState;
		outDifferentialState = &differentialState;
		// The limited slip differential works through these wheel groups. (The SDK's own EngineDriveVehicle passes
		// them only for a tank, so its four-wheel-drive differential never limits slip.)
		outConstraintGroupState = limited_slip ? &wheelConstraintGroupState : nullptr;
		outWheelRigidBody1dStates.setData(wheelRigidBody1dStates);
		outEngineState = &engineState;
		outGearboxState = &gearboxState;
		outClutchState = &clutchSlipState;
	}

	void initComponentSequence() {
		// From empty: a live rebuild configures the same object again, and adding to the old sequence ran every
		// component once more per step for each rebuild.
		componentSequence = PxVehicleComponentSequence();
		componentSequence.add(static_cast<PxVehiclePhysXActorBeginComponent *>(this));
		if (engine_drive) {
			// The autobox may start a shift here (overriding the target gear and cutting the throttle meanwhile).
			componentSequence.add(static_cast<PxVehicleEngineDriveCommandResponseComponent *>(this));
			if (limited_slip) {
				componentSequence.add(static_cast<PxVehicleFourWheelDriveDifferentialStateComponent *>(this));
			} else {
				componentSequence.add(static_cast<PxVehicleMultiWheelDriveDifferentialStateComponent *>(this));
			}
			componentSequence.add(static_cast<PxVehicleEngineDriveActuationStateComponent *>(this));
		} else {
			componentSequence.add(static_cast<PxVehicleDirectDriveCommandResponseComponent *>(this));
			componentSequence.add(static_cast<PxVehicleDirectDriveActuationStateComponent *>(this));
		}
		componentSequence.add(static_cast<PxVehiclePhysXRoadGeometrySceneQueryComponent *>(this));

		componentSequenceSubstepGroupHandle = componentSequence.beginSubstepGroup(3);
		componentSequence.add(static_cast<PxVehicleSuspensionComponent *>(this));
		componentSequence.add(static_cast<PxVehicleTireComponent *>(this));
		componentSequence.add(static_cast<PxVehiclePhysXConstraintComponent *>(this));
		if (engine_drive) {
			componentSequence.add(static_cast<PxVehicleEngineDrivetrainComponent *>(this));
		} else {
			componentSequence.add(static_cast<PxVehicleDirectDrivetrainComponent *>(this));
		}
		componentSequence.add(static_cast<PxVehicleRigidBodyComponent *>(this));
		componentSequence.endSubstepGroup();

		componentSequence.add(static_cast<PxVehicleWheelComponent *>(this));
		componentSequence.add(static_cast<PxVehiclePhysXActorEndComponent *>(this));
	}

	void step(PxReal dt, const PxVehicleSimulationContext &context) {
		componentSequence.update(dt, context);
	}

	void destroy() {
		PxVehicleConstraintsDestroy(physxConstraints);
		GodotPhysXSpace3D::hold_actor_materials(physxActor.rigidBody); // (no material handle reused before a step)
		PxVehiclePhysXActorDestroy(physxActor);
	}
};

// Everything a caller can tune about ONE wheel of a direct-drive 4-wheel
// vehicle, in Godot units/conventions. `position` is the suspension
// attachment point in chassis-local space -- same meaning as VehicleWheel3D's
// own `position` (the hardpoint), so a PhysXVehicleWheel3D child node's
// transform origin maps straight onto it with no separate axle/track-width
// config needed.
struct Vehicle4WWheelConfig {
	Vector3 position;
	// The wheel node's own local orientation (relative to the chassis) --
	// identity (the default) means straight-down suspension travel, matching
	// every existing scene/test. A non-identity basis angles the suspension
	// travel direction and gives the wheel real camber -- e.g. a dune buggy's
	// angled A-arms. Same convention as Vehicle2WWheelConfig::basis; see that
	// struct's own doc comment for the stock-Godot precedent this mirrors.
	Basis basis;
	real_t radius = 0.35f;
	real_t half_width = 0.15f;
	real_t wheel_mass = 20.0f;
	real_t wheel_moment_of_inertia = 1.2f;
	real_t damping_rate = 0.25f;

	real_t suspension_travel = 0.15f;
	// 35000 (the snippet reference's own value, scaled for this car's mass)
	// left the suspension sitting at ~68% of suspension_travel just holding
	// static weight at rest (measured directly via PxVehicleSuspensionState
	// .jounce: 0.102/0.15) -- almost no margin before bottoming out under any
	// real driving load (braking, cornering weight transfer, a bump), same
	// failure mode VehicleWheel3D's own default (5.88) had. Returned so static
	// jounce sits around 30% of travel instead, leaving real margin.
	real_t suspension_stiffness = 90000.0f;
	real_t suspension_damping = 4500.0f;

	real_t tire_lateral_stiffness = 20000.0f;
	real_t tire_longitudinal_stiffness = 20000.0f;
	real_t tire_camber_stiffness = 0.0f;
	real_t tire_friction = 1.0f;
	// frictionVsSlip curve, as fractions of tire_friction: grip ramps up to
	// peak (1.0x) at 10% slip, then falls off to tire_slide_grip once fully
	// sliding/locked (100% slip). tire_rest_grip is the 0%-slip value.
	real_t tire_rest_grip = 0.9f;
	real_t tire_slide_grip = 0.7f;

	// Front axle (steering) vs. rear -- also which axle Ackermann correction
	// and the steer response multiplier apply to. Exactly 2 of the 4 wheels
	// must have this true.
	bool use_as_steering = false;
	// Whether this wheel receives engine torque. All 4 true = AWD direct
	// drive (the only drivetrain this MVP composition supports).
	bool use_as_traction = true;
};

// The engine drive's tuning (Vehicle4WConfig::engine), in Godot-friendly units: engine speeds in RPM, gears as
// positive ratios. Defaults are the PhysX vehicle snippets' own (EngineDrive.json), with the speeds rounded to RPM.
struct Vehicle4WEngineConfig {
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
	// Limited slip: PhysX's four-wheel-drive differential. A bias (> 1) is the largest wheel speed ratio across an
	// axle (front, rear) or between the axles (center) before torque moves to the slower side, until the ratio is
	// down to the target (1 < target < bias), at `rate`. A bias of 0 leaves that one open.
	bool limited_slip = false;
	real_t front_bias = 1.3f;
	real_t front_target = 1.29f;
	real_t rear_bias = 1.3f;
	real_t rear_target = 1.29f;
	real_t center_bias = 1.3f;
	real_t center_target = 1.29f;
	real_t rate = 10.0f;
};

// Everything a caller can tune about a 4-wheel vehicle, in Godot
// units/conventions (meters, kg, radians, Godot's +Z-forward/+X-left/+Y-up vehicle convention).
// Shared between the probe (hardcoded sedan-like defaults) and PhysXVehicle3D
// (these become real exported properties -- chassis_half_extents/
// chassis_half_extents/chassis_box_center_local come from a real CollisionShape3D child, wheels[] from
// real PhysXVehicleWheel3D children, matching VehicleBody3D/VehicleWheel3D's
// own node structure instead of flat scalar properties on the body).
struct Vehicle4WConfig {
	real_t mass = 1500.0f;
	Vector3 moment_of_inertia = Vector3(2000.0f, 2200.0f, 1000.0f);

	Vector3 chassis_half_extents = Vector3(0.95f, 0.4f, 1.85f);
	Vector3 chassis_box_center_local = Vector3(0.0f, 0.4f, 0.0f);
	// Center of mass, separate from the box's own visual/collision center --
	// a lower CoM than the box's geometric center is what keeps the chassis
	// planted through hard cornering instead of rolling.
	Vector3 chassis_com_local = Vector3(0.0f, 0.35f, 0.0f);

	// Caller order is arbitrary -- configure_vehicle4w() classifies each
	// entry into front/rear by use_as_steering and left/right by
	// position.x's sign, so a scene author can add 4 PhysXVehicleWheel3D
	// children in any order (matching how VehicleWheel3D children work
	// today: only position and use_as_steering matter, not insertion order).
	Vehicle4WWheelConfig wheels[4];

	real_t max_engine_torque = 700.0f; // direct drive only
	Vehicle4WEngineConfig engine;
	real_t max_brake_torque = 6000.0f;
	real_t max_steer_angle = 0.6f; // radians
	real_t ackermann_strength = 1.0f;

	// Anti-roll bar stiffness, front axle (FL/FR) and rear axle (RL/RR).
	// Positive values are a real anti-roll bar: they reduce the jounce
	// difference between the two wheels, i.e. resist body roll during
	// cornering. Negative values do the opposite -- they amplify the jounce
	// difference, exaggerating roll instead of resisting it. 0 = no anti-roll
	// bar on that axle (PxVehicleAntiRollForceParams is still submitted with
	// stiffness=0, a harmless no-op, rather than omitted, to keep the wiring
	// unconditional).
	real_t front_anti_roll_stiffness = 0.0f;
	real_t rear_anti_roll_stiffness = 0.0f;

	// Where the tires push the chassis: 0 at the suspension mount (the wheel's top of travel), 1 at the tire's
	// contact patch on the ground, following the suspension. Low in the car, as a real tire does, drive and
	// braking forces pitch the body (squat, dive, a wheelie with enough grip) and cornering rolls it.
	real_t tire_force_point = 0.0f;

	// Same meaning as RigidBody3D/VehicleBody3D's own collision_layer/mask --
	// applies to the chassis box shape only (see configure_vehicle4w()'s own
	// note on why the wheel shapes stay non-simulating).
	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;
	// Chassis box contact material, signed like PhysicsMaterial::computed_friction() / computed_bounce() (the
	// node's physics_material_override, or these defaults).
	float chassis_friction = 0.5f;
	float chassis_bounce = 0.1f;
};

// Fills v's engine drive params from cfg.engine (wheels already in canonical FL/FR/RL/RR slots; traction[i] = that
// slot's use_as_traction). PhysX checks these only in its debug builds, so what a scene can get wrong is checked here.
inline bool configure_vehicle4w_engine(Vehicle4W &v, const Vehicle4WEngineConfig &e, const bool traction[4]) {
	v.engine_drive = true;
	v.limited_slip = e.limited_slip;
	const PxReal rpm_to_omega = (PxReal)(Math::TAU / 60.0);

	if (e.peak_torque < 0.0 || e.idle_rpm < 0.0 || e.max_rpm <= e.idle_rpm || e.moment_of_inertia <= 0.0) {
		ERR_PRINT("PhysX vehicle: the engine needs peak_torque >= 0, 0 <= idle_rpm < max_rpm and a moment of inertia > 0.");
		return false;
	}
	PxVehicleEngineParams &engine = v.engineParams;
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
	PxVehicleGearboxParams &gears = v.gearboxParams;
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
		v.autoboxParams.upRatios[i] = (PxReal)MAX(e.autobox_up_ratio, (real_t)0.0);
		v.autoboxParams.downRatios[i] = (PxReal)MAX(e.autobox_down_ratio, (real_t)0.0);
	}
	v.autoboxParams.latency = (PxReal)MAX(e.autobox_latency, (real_t)0.0);

	v.clutchCommandResponseParams.maxResponse = (PxReal)MAX(e.clutch_strength, (real_t)0.0);
	v.clutchParams.accuracyMode = PxVehicleClutchAccuracyMode::eBEST_POSSIBLE;
	v.clutchParams.estimateIterations = 5;

	// The engine's torque is shared evenly between the traction wheels.
	int driven = 0;
	for (PxU32 i = 0; i < 4; i++) {
		driven += traction[i] ? 1 : 0;
	}
	if (driven == 0) {
		ERR_PRINT("PhysX vehicle: the engine drive needs at least one wheel with use_as_traction.");
		return false;
	}
	PxVehicleMultiWheelDriveDifferentialParams &multi = v.multiWheelDifferentialParams;
	multi.setToDefault();
	for (PxU32 i = 0; i < 4; i++) {
		multi.torqueRatios[i] = traction[i] ? 1.0f / driven : 0.0f;
		multi.aveWheelSpeedRatios[i] = multi.torqueRatios[i];
	}
	PxVehicleFourWheelDriveDifferentialParams &four = v.fourWheelDifferentialParams;
	four.setToDefault();
	for (PxU32 i = 0; i < 4; i++) {
		four.torqueRatios[i] = multi.torqueRatios[i];
		four.aveWheelSpeedRatios[i] = multi.aveWheelSpeedRatios[i];
	}
	four.frontWheelIds[0] = Vehicle4W::WHEEL_FL;
	four.frontWheelIds[1] = Vehicle4W::WHEEL_FR;
	four.rearWheelIds[0] = Vehicle4W::WHEEL_RL;
	four.rearWheelIds[1] = Vehicle4W::WHEEL_RR;
	if (e.limited_slip) {
		// A bias only on an axle (or, center, between axles) that is driven on both sides.
		const bool front = traction[Vehicle4W::WHEEL_FL] && traction[Vehicle4W::WHEEL_FR];
		const bool rear = traction[Vehicle4W::WHEEL_RL] && traction[Vehicle4W::WHEEL_RR];
		auto valid = [](real_t p_bias, real_t p_target) {
			return p_bias == 0.0 || (p_bias == 1.0 && p_target == 1.0) || (p_target > 1.0 && p_target < p_bias);
		};
		if (!valid(e.front_bias, e.front_target) || !valid(e.rear_bias, e.rear_target) || !valid(e.center_bias, e.center_target) || e.rate < 0.0) {
			ERR_PRINT("PhysX vehicle: each limited slip bias must be 0 (open) or > 1 with 1 < target < bias, and the rate >= 0.");
			return false;
		}
		four.frontBias = front ? (PxReal)e.front_bias : 0.0f;
		four.frontTarget = front ? (PxReal)e.front_target : 0.0f;
		four.rearBias = rear ? (PxReal)e.rear_bias : 0.0f;
		four.rearTarget = rear ? (PxReal)e.rear_target : 0.0f;
		four.centerBias = front && rear ? (PxReal)e.center_bias : 0.0f;
		four.centerTarget = front && rear ? (PxReal)e.center_target : 0.0f;
		four.rate = (PxReal)e.rate;
	}

	// Start idling in first gear (the PhysX snippets' start), the gearbox in automatic until told otherwise.
	v.gearboxState.setToDefault();
	v.gearboxState.currentGear = gears.neutralGear + 1;
	v.gearboxState.targetGear = gears.neutralGear + 1;
	v.engineTransmissionCommandState.setToDefault();
	v.engineTransmissionCommandState.targetGear = PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR;
	v.stop_drivetrain();
	return true;
}

// Fills every param struct on v from cfg, builds the real PxRigidDynamic
// chassis+wheel shapes (PxVehiclePhysXActorCreate), the suspension-limit/
// sticky-tire constraints (PxVehicleConstraintsCreate), the component
// sequence, and out_context -- everything GodotPhysXVehicleProbe::initialize()
// used to do inline. Does NOT add the actor to the scene or set its start
// pose -- the caller does that (probe/node have different start-pose
// conventions: a bare position vs. a node's own global_transform).
// Returns false (with an ERR_PRINT) on any real validation failure --
// including cfg.wheels not containing exactly 2 use_as_steering==true and
// 2 ==false entries, which this fixed direct-drive/Ackermann composition
// requires.
// out_wheel_order[Vehicle4W::WHEEL_FL/FR/RL/RR] = the index into cfg.wheels[]
// that ended up in that canonical slot -- callers that keep their own
// per-wheel objects in cfg.wheels[] order (e.g. PhysXVehicle3D's own
// PhysXVehicleWheel3D children) need this to know which of their own
// objects corresponds to v.wheelLocalPoses[WHEEL_FL] etc. each tick.
inline bool configure_vehicle4w(Vehicle4W &v, const Vehicle4WConfig &cfg, PxPhysics &physics, PxScene &scene, PxVehiclePhysXSimulationContext &out_context, PxU32 out_wheel_order[4]) {
	v.setToDefault();

	// Godot's vehicle convention: forward = +Z (VehicleBody3D: "the local forward
	// for this node is Vector3.MODEL_FRONT"), left = +X, up = +Y -- also PhysX's own
	// default frame, and a 1:1 component mapping (godot_physx_conversions.h), so no
	// axis remap reading the chassis pose back out. (Was declared -Z while the
	// vehicles drove +Z anyway: only get_forward_speed() read the declaration.)
	v.frame.lngAxis = PxVehicleAxes::ePosZ;
	v.frame.latAxis = PxVehicleAxes::ePosX;
	v.frame.vrtAxis = PxVehicleAxes::ePosY;
	v.scale.scale = 1.0f;

	// Classify cfg.wheels[0..3] (arbitrary caller order) into canonical
	// FL/FR/RL/RR slots by (use_as_steering, position.x sign).
	PxU32 front[2], rear[2];
	PxU32 nb_front = 0, nb_rear = 0;
	for (PxU32 i = 0; i < 4; i++) {
		if (cfg.wheels[i].use_as_steering) {
			if (nb_front < 2) {
				front[nb_front++] = i;
			}
		} else {
			if (nb_rear < 2) {
				rear[nb_rear++] = i;
			}
		}
	}
	if (nb_front != 2 || nb_rear != 2) {
		ERR_PRINT("PhysX vehicle: need exactly 2 steering (front) and 2 non-steering (rear) wheels.");
		return false;
	}
	if (cfg.wheels[front[0]].position.x > cfg.wheels[front[1]].position.x) {
		SWAP(front[0], front[1]);
	}
	if (cfg.wheels[rear[0]].position.x > cfg.wheels[rear[1]].position.x) {
		SWAP(rear[0], rear[1]);
	}
	// slot[Vehicle4W::WHEEL_FL] = index into cfg.wheels[] for that slot.
	PxU32 slot[4];
	slot[Vehicle4W::WHEEL_FL] = front[0]; // negative (left) x
	slot[Vehicle4W::WHEEL_FR] = front[1]; // positive (right) x
	slot[Vehicle4W::WHEEL_RL] = rear[0];
	slot[Vehicle4W::WHEEL_RR] = rear[1];
	for (PxU32 i = 0; i < 4; i++) {
		out_wheel_order[i] = slot[i];
	}

	const PxU32 frontWheels[2] = { Vehicle4W::WHEEL_FL, Vehicle4W::WHEEL_FR };
	const PxU32 rearWheels[2] = { Vehicle4W::WHEEL_RL, Vehicle4W::WHEEL_RR };
	v.axleDescription.setToDefault();
	v.axleDescription.addAxle(2, frontWheels);
	v.axleDescription.addAxle(2, rearWheels);

	v.suspensionStateCalculationParams.suspensionJounceCalculationType = PxVehicleSuspensionJounceCalculationType::eRAYCAST;
	v.suspensionStateCalculationParams.limitSuspensionExpansionVelocity = false;

	// Regular brake: all 4 wheels; handbrake: rear only.
	v.brakeResponseParams[0].maxResponse = (PxReal)cfg.max_brake_torque;
	v.brakeResponseParams[1].maxResponse = (PxReal)cfg.max_brake_torque;
	for (PxU32 i = 0; i < 4; i++) {
		v.brakeResponseParams[0].wheelResponseMultipliers[i] = 1.0f;
		v.brakeResponseParams[1].wheelResponseMultipliers[i] = (i == Vehicle4W::WHEEL_RL || i == Vehicle4W::WHEEL_RR) ? 1.0f : 0.0f;
	}
	v.steerResponseParams.maxResponse = (PxReal)cfg.max_steer_angle;
	for (PxU32 i = 0; i < 4; i++) {
		v.steerResponseParams.wheelResponseMultipliers[i] = (i == Vehicle4W::WHEEL_FL || i == Vehicle4W::WHEEL_FR) ? 1.0f : 0.0f;
	}
	const Vector3 &fl_pos = cfg.wheels[slot[Vehicle4W::WHEEL_FL]].position;
	const Vector3 &fr_pos = cfg.wheels[slot[Vehicle4W::WHEEL_FR]].position;
	const Vector3 &rl_pos = cfg.wheels[slot[Vehicle4W::WHEEL_RL]].position;
	v.ackermannParams[0].wheelIds[0] = Vehicle4W::WHEEL_FL;
	v.ackermannParams[0].wheelIds[1] = Vehicle4W::WHEEL_FR;
	v.ackermannParams[0].wheelBase = (PxReal)Math::abs(fl_pos.z - rl_pos.z);
	v.ackermannParams[0].trackWidth = (PxReal)Math::abs(fr_pos.x - fl_pos.x);
	v.ackermannParams[0].strength = (PxReal)cfg.ackermann_strength;

	v.antiRollForceParams[0].wheel0 = Vehicle4W::WHEEL_FL;
	v.antiRollForceParams[0].wheel1 = Vehicle4W::WHEEL_FR;
	v.antiRollForceParams[0].stiffness = (PxReal)cfg.front_anti_roll_stiffness;
	v.antiRollForceParams[1].wheel0 = Vehicle4W::WHEEL_RL;
	v.antiRollForceParams[1].wheel1 = Vehicle4W::WHEEL_RR;
	v.antiRollForceParams[1].stiffness = (PxReal)cfg.rear_anti_roll_stiffness;

	v.directDriveThrottleResponseParams.maxResponse = (PxReal)cfg.max_engine_torque;
	bool traction[4];
	for (PxU32 i = 0; i < 4; i++) {
		traction[i] = cfg.wheels[slot[i]].use_as_traction;
		v.directDriveThrottleResponseParams.wheelResponseMultipliers[i] = traction[i] ? 1.0f : 0.0f;
	}
	v.engine_drive = false;
	v.limited_slip = false;
	if (cfg.engine.enabled && !configure_vehicle4w_engine(v, cfg.engine, traction)) {
		return false;
	}

	v.rigidBodyParams.mass = (PxReal)cfg.mass;
	v.rigidBodyParams.moi = to_px(cfg.moment_of_inertia);

	for (PxU32 i = 0; i < 4; i++) {
		const Vehicle4WWheelConfig &w = cfg.wheels[slot[i]];

		v.wheelParams[i].radius = (PxReal)w.radius;
		v.wheelParams[i].halfWidth = (PxReal)w.half_width;
		v.wheelParams[i].mass = (PxReal)w.wheel_mass;
		v.wheelParams[i].moi = (PxReal)w.wheel_moment_of_inertia;
		v.wheelParams[i].dampingRate = (PxReal)w.damping_rate;

		v.suspensionForceParams[i].stiffness = (PxReal)w.suspension_stiffness;
		v.suspensionForceParams[i].damping = (PxReal)w.suspension_damping;
		v.suspensionForceParams[i].sprungMass = v.rigidBodyParams.mass * 0.25f;

		// suspensionAttachment is PxVehicle2's own "wheel pose at maximum
		// compression" (PxVehicleSuspensionParams.h), specified in "the
		// frame of the rigid body" -- CoM-composed
		// (actorGlobalPose * actorCMassLocalPose, see PxVehicleWheelHelpers.h's
		// rigidBodyPose), not the actor's raw origin. Neither of those is
		// what a scene author expects w.position to mean: PxVehicle2 has no
		// notion of a rest length independent of travel (unlike Bullet's
		// VehicleWheel3D, whose spring force is stiffness*(restLength -
		// currentLength) -- restLength and travel are two separate
		// numbers there), so its natural zero-force point always sits
		// pinned to full droop, `travel` away from the attachment. That
		// means changing travel alone shifts where the suspension settles,
		// which is surprising and makes w.position mean "attachment", not
		// "resting position" -- the opposite of VehicleWheel3D's own
		// convention. Backing out the attachment from an estimated static
		// jounce makes w.position mean the same thing VehicleWheel3D's own
		// position means: where the wheel actually sits at rest. Travel
		// direction is the wheel node's own local -Y (see
		// Vehicle4WWheelConfig::basis's own doc comment) instead of a
		// hardcoded (0,-1,0), so the backing-out math below projects along
		// that direction rather than assuming +Y is "up" relative to the
		// suspension.
		const Vector3 travel_dir_local = w.basis.xform(Vector3(0.0f, -1.0f, 0.0f)).normalized();
		const PxReal restLoadEstimate = v.suspensionForceParams[i].sprungMass * 9.81f;
		const PxReal jounceAtRest = (w.suspension_stiffness > 0.0) ? PxClamp(restLoadEstimate / (PxReal)w.suspension_stiffness, 0.0f, (PxReal)w.suspension_travel) : 0.0f;
		const Vector3 attachment_local = (w.position - cfg.chassis_com_local) - travel_dir_local * (real_t)((PxReal)w.suspension_travel - jounceAtRest);
		v.suspensionParams[i].suspensionAttachment = PxTransform(to_px(attachment_local), to_px(w.basis.get_rotation_quaternion()));
		v.suspensionParams[i].suspensionTravelDir = to_px(travel_dir_local);
		v.suspensionParams[i].suspensionTravelDist = (PxReal)w.suspension_travel;
		v.suspensionParams[i].wheelAttachment = PxTransform(PxIdentity);

		v.suspensionComplianceParams[i] = PxVehicleSuspensionComplianceParams(); // no toe/camber curves
		if (cfg.tire_force_point > 0.0) {
			// The contact patch from the mount (the attachment, in its own frame): travel_dist - jounce + radius
			// down the travel direction, i.e. travel + radius fully extended (0) and radius fully compressed (1).
			const PxVec3 down = v.suspensionParams[i].suspensionAttachment.q.rotateInv(to_px(travel_dir_local)) * (PxReal)CLAMP(cfg.tire_force_point, (real_t)0.0, (real_t)1.0);
			v.suspensionComplianceParams[i].tireForceAppPoint.addPair(0.0f, down * (PxReal)(w.suspension_travel + w.radius));
			v.suspensionComplianceParams[i].tireForceAppPoint.addPair(1.0f, down * (PxReal)w.radius);
		}

		v.tireForceParams[i].latStiffX = 0.01f;
		v.tireForceParams[i].latStiffY = (PxReal)w.tire_lateral_stiffness;
		v.tireForceParams[i].longStiff = (PxReal)w.tire_longitudinal_stiffness;
		v.tireForceParams[i].camberStiff = (PxReal)w.tire_camber_stiffness;
		v.tireForceParams[i].restLoad = v.suspensionForceParams[i].sprungMass * 9.81f;
		// Peak grip at ~10% slip, falling off once fully locked/sliding --
		// a flat curve here would mean a locked tire grips as well as a
		// rolling one, which never produces a real skid.
		v.tireForceParams[i].frictionVsSlip[0][0] = 0.0f;
		v.tireForceParams[i].frictionVsSlip[0][1] = (PxReal)w.tire_friction * (PxReal)w.tire_rest_grip;
		v.tireForceParams[i].frictionVsSlip[1][0] = 0.1f;
		v.tireForceParams[i].frictionVsSlip[1][1] = (PxReal)w.tire_friction;
		v.tireForceParams[i].frictionVsSlip[2][0] = 1.0f;
		v.tireForceParams[i].frictionVsSlip[2][1] = (PxReal)w.tire_friction * (PxReal)w.tire_slide_grip;
		v.tireForceParams[i].loadFilter[0][0] = 0.0f;
		v.tireForceParams[i].loadFilter[0][1] = 0.23f;
		v.tireForceParams[i].loadFilter[1][0] = 3.0f;
		v.tireForceParams[i].loadFilter[1][1] = 3.0f;

		v.physxMaterialFrictionParams[i].defaultFriction = (PxReal)w.tire_friction;
		v.physxMaterialFrictionParams[i].materialFrictions = nullptr;
		v.physxMaterialFrictionParams[i].nbMaterialFrictions = 0;

		v.physxSuspensionLimitConstraintParams[i].restitution = 0.0f;
		v.physxSuspensionLimitConstraintParams[i].directionForSuspensionLimitConstraint = PxVehiclePhysXSuspensionLimitConstraintParams::eROAD_GEOMETRY_NORMAL;

		v.physxWheelShapeLocalPoses[i] = PxTransform(PxIdentity);
	}

	if (!v.axleDescription.isValid()) {
		ERR_PRINT("PhysX vehicle: invalid axle description.");
		return false;
	}
	if (!v.rigidBodyParams.isValid()) {
		ERR_PRINT("PhysX vehicle: invalid rigid body params.");
		return false;
	}

	v.physxRoadGeometryQueryParams.roadGeometryQueryType = PxVehiclePhysXRoadGeometryQueryType::eRAYCAST;
	v.physxRoadGeometryQueryParams.defaultFilterData = PxQueryFilterData(PxFilterData(0, 0, 0, 0), PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER);
	v.wheelQueryFilter.collision_mask = (PxU32)cfg.collision_mask;
	v.physxRoadGeometryQueryParams.filterCallback = &v.wheelQueryFilter;
	v.physxRoadGeometryQueryParams.filterDataEntries = nullptr;

	v.physxActorCMassLocalPose = PxTransform(to_px(cfg.chassis_com_local));
	v.physxActorBoxShapeHalfExtents = to_px(cfg.chassis_half_extents);
	v.physxActorBoxShapeLocalPose = PxTransform(to_px(cfg.chassis_box_center_local));

	// Wheel shapes stay non-simulating (flags(0) below), so this material's
	// friction/restitution never actually gets consumed by them -- it only
	// exists because PxVehiclePhysXWheelShapeParams requires one.
	PxMaterial *wheel_material = physics.createMaterial((PxReal)cfg.wheels[0].tire_friction, (PxReal)cfg.wheels[0].tire_friction, 0.1f);
	// Chassis gets its own, deliberately low-friction material -- it's a
	// REAL simulation shape now (see below), and if suspension settling ever
	// lets the box graze the ground even slightly, a high-friction contact
	// there fights the drivetrain directly (found via a real regression:
	// reusing the wheel's friction=1.0 material on the chassis froze the car
	// in place, throttle doing nothing, the instant this shape went from
	// PxShapeFlags(0) to a real simulation shape). The chassis's role is
	// just physical bulk for other objects to bump into, not a friction
	// surface -- the wheels' own tire model is the only thing that should
	// ever resist the car's own motion.
	// Not 0, though: bodies combine friction by MIN (Godot's rule, see godot_physx_material.h), so a 0 here made a
	// flipped vehicle slide on its roof forever. The default 0.5 is what the roof used to get against a default
	// (1.0) ground under PhysX's AVERAGE; the node's physics_material_override replaces it. Combined the same
	// way as every other body. Created per vehicle, so the node can edit it in place.
	PxMaterial *chassis_material = physics.createMaterial(Math::abs(cfg.chassis_friction), Math::abs(cfg.chassis_friction), CLAMP(cfg.chassis_bounce, 0.0f, 1.0f));
	if (chassis_material) {
		godot_physx_apply_material(chassis_material, cfg.chassis_friction, cfg.chassis_bounce);
	}
	if (!wheel_material || !chassis_material) {
		ERR_PRINT("PhysX vehicle: failed to create material.");
		return false;
	}
	PxCookingParams cookingParams(physics.getTolerancesScale());

	{
		// Chassis box: a REAL simulation shape (unlike the reference vehicle's
		// own PxShapeFlags(0)) so the car's body can actually be hit by other
		// rigid bodies in the scene, using Godot's own layer/mask convention
		// (word0=layer, word1=mask -- see godot_physx_filter_shader).
		// Also a scene-query shape, so Godot raycasts, shape queries and
		// CharacterBody3D motion hit the car (they see it as the node's
		// stand-in body, see GodotPhysXBody3D::set_vehicle_actor). The road-
		// geometry raycast each wheel casts starts inside this box, so the
		// wheel filter skips the car's own actor (self_actor below): without
		// that every wheel hit its own chassis instead of the ground (jounce
		// pinned at 0, zero tire load, the car settling flush on the ground).
		// Wheels stay at flags(0): their ground contact is entirely the
		// road-geometry raycast + suspension, not real shape collision, and
		// making them real simulation shapes too would double-apply ground
		// reaction forces on top of that.
		const PxFilterData chassisFilterData((PxU32)cfg.collision_layer, (PxU32)cfg.collision_mask, 0, 0);
		const PxShapeFlags chassisShapeFlags(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eVISUALIZATION);
		const PxVehiclePhysXRigidActorParams actorParams(v.rigidBodyParams, nullptr);
		const PxBoxGeometry boxGeom(v.physxActorBoxShapeHalfExtents);
		const PxVehiclePhysXRigidActorShapeParams actorShapeParams(boxGeom, v.physxActorBoxShapeLocalPose, *chassis_material, chassisShapeFlags, chassisFilterData, chassisFilterData);
		const PxVehiclePhysXWheelParams wheelParams(v.axleDescription, v.wheelParams);
		const PxVehiclePhysXWheelShapeParams wheelShapeParams(*wheel_material, PxShapeFlags(0), PxFilterData(), PxFilterData());

		PxVehiclePhysXActorCreate(
				v.frame,
				actorParams, v.physxActorCMassLocalPose, actorShapeParams,
				wheelParams, wheelShapeParams,
				physics, cookingParams,
				v.physxActor);
	}
	wheel_material->release();
	chassis_material->release();

	if (!v.physxActor.rigidBody) {
		ERR_PRINT("PhysX vehicle: PxVehiclePhysXActorCreate failed.");
		return false;
	}
	v.wheelQueryFilter.self_actor = v.physxActor.rigidBody;

	PxVehicleConstraintsCreate(v.axleDescription, physics, *v.physxActor.rigidBody, v.physxConstraints);

	v.initComponentSequence();
	v.transmissionCommandState.gear = PxVehicleDirectDriveTransmissionCommandState::eFORWARD;

	out_context.setToDefault();
	out_context.frame = v.frame;
	out_context.scale = v.scale;
	out_context.gravity = v.frame.getVrtAxis() * -9.81f;
	out_context.physxScene = &scene;
	out_context.physxUnitCylinderSweepMesh = nullptr;

	return true;
}

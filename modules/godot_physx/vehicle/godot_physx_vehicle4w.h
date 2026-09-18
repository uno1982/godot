/**************************************************************************/
/*  godot_physx_vehicle4w.h                                               */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
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
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "../godot_physx_conversions.h"

#include "core/error/error_macros.h"
#include "core/math/vector3.h"
#include "core/typedefs.h"

#include <PxPhysicsAPI.h>
#include <vehicle/PxVehicleAPI.h>

using namespace physx;

// A single direct-drive 4-wheel vehicle, composed exactly the way PhysX's own
// snippetvehiclecommon (Base/DirectDrivetrain/PhysXIntegration) composes one --
// ported here rather than linked, since that layer is snippet source, not part
// of the installed PhysXVehicle_static_64 lib (which only supplies the real
// per-component math these getDataForXComponent overrides hand pointers into).
// Wheel index convention: 0=FL, 1=FR, 2=RL, 3=RR. Shared verbatim between
// GodotPhysXVehicleProbe (headless regression) and PhysXVehicle3D (the real
// node) -- this class and configure_vehicle4w() below are pure PxVehicle2
// composition with no project-specific decisions in them, so duplicating them
// would just be ~500 lines of copy-pasted boilerplate for no benefit.
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
				   public PxVehicleDirectDrivetrainComponent {
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

	// --- Direct drive (DirectDrivetrainParams/State) -----------------------
	PxVehicleDirectDriveThrottleCommandResponseParams directDriveThrottleResponseParams;
	PxReal directDriveThrottleResponseStates[4] = {};
	PxVehicleCommandState commandState;
	PxVehicleDirectDriveTransmissionCommandState transmissionCommandState;

	// --- PhysX integration (PhysXIntegrationParams/State) ------------------
	PxVehiclePhysXRoadGeometryQueryParams physxRoadGeometryQueryParams;
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
		commandState.setToDefault();
		transmissionCommandState.gear = PxVehicleDirectDriveTransmissionCommandState::eNEUTRAL;
		physxActor.setToDefault();
		physxSteerState.setToDefault();
		physxConstraints.setToDefault();
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
		outAntiRollTorque = nullptr;
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
		outAntiRollForceParams.setEmpty();
		outWheelRoadGeomStates.setData(roadGeomStates);
		outSuspensionStates.setData(suspensionStates);
		outSuspensionComplianceStates.setData(suspensionComplianceStates);
		outSuspensionForces.setData(suspensionForces);
		outAntiRollTorque = nullptr;
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
		outTransmissionCommands = nullptr;
		outGearParams = nullptr;
		outGearState = nullptr;
		outEngineParams = nullptr;
		outEngineState = nullptr;
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
		outGearState = nullptr;
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

	void initComponentSequence() {
		componentSequence.add(static_cast<PxVehiclePhysXActorBeginComponent *>(this));
		componentSequence.add(static_cast<PxVehicleDirectDriveCommandResponseComponent *>(this));
		componentSequence.add(static_cast<PxVehicleDirectDriveActuationStateComponent *>(this));
		componentSequence.add(static_cast<PxVehiclePhysXRoadGeometrySceneQueryComponent *>(this));

		componentSequenceSubstepGroupHandle = componentSequence.beginSubstepGroup(3);
		componentSequence.add(static_cast<PxVehicleSuspensionComponent *>(this));
		componentSequence.add(static_cast<PxVehicleTireComponent *>(this));
		componentSequence.add(static_cast<PxVehiclePhysXConstraintComponent *>(this));
		componentSequence.add(static_cast<PxVehicleDirectDrivetrainComponent *>(this));
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
	// failure mode VehicleWheel3D's own default (5.88) had. Retuned so static
	// jounce sits around 30% of travel instead, leaving real margin.
	real_t suspension_stiffness = 90000.0f;
	real_t suspension_damping = 4500.0f;

	real_t tire_lateral_stiffness = 20000.0f;
	real_t tire_longitudinal_stiffness = 20000.0f;
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

// Everything a caller can tune about a direct-drive 4-wheel vehicle, in Godot
// units/conventions (meters, kg, radians, Godot's -Z-forward/+X-right/+Y-up).
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

	real_t max_engine_torque = 700.0f;
	real_t max_brake_torque = 6000.0f;
	real_t max_steer_angle = 0.6f; // radians
	real_t ackermann_strength = 1.0f;

	// Same meaning as RigidBody3D/VehicleBody3D's own collision_layer/mask --
	// applies to the chassis box shape only (see configure_vehicle4w()'s own
	// note on why the wheel shapes stay non-simulating).
	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;
};

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

	// Godot convention: forward = -Z, right = +X, up = +Y (matches
	// godot_physx_conversions.h's 1:1 PhysX<->Godot component mapping, so no
	// axis remap is needed when reading the chassis pose back out).
	v.frame.lngAxis = PxVehicleAxes::eNegZ;
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

	v.directDriveThrottleResponseParams.maxResponse = (PxReal)cfg.max_engine_torque;
	for (PxU32 i = 0; i < 4; i++) {
		v.directDriveThrottleResponseParams.wheelResponseMultipliers[i] = cfg.wheels[slot[i]].use_as_traction ? 1.0f : 0.0f;
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
		// position means: where the wheel actually sits at rest.
		const PxReal restLoadEstimate = v.suspensionForceParams[i].sprungMass * 9.81f;
		const PxReal jounceAtRest = (w.suspension_stiffness > 0.0) ? PxClamp(restLoadEstimate / (PxReal)w.suspension_stiffness, 0.0f, (PxReal)w.suspension_travel) : 0.0f;
		const Vector3 attachment_local = w.position - cfg.chassis_com_local + Vector3(0.0f, (real_t)((PxReal)w.suspension_travel - jounceAtRest), 0.0f);
		v.suspensionParams[i].suspensionAttachment = PxTransform(to_px(attachment_local));
		v.suspensionParams[i].suspensionTravelDir = PxVec3(0.0f, -1.0f, 0.0f);
		v.suspensionParams[i].suspensionTravelDist = (PxReal)w.suspension_travel;
		v.suspensionParams[i].wheelAttachment = PxTransform(PxIdentity);

		v.suspensionComplianceParams[i] = PxVehicleSuspensionComplianceParams(); // no toe/camber/force-offset curves

		v.tireForceParams[i].latStiffX = 0.01f;
		v.tireForceParams[i].latStiffY = (PxReal)w.tire_lateral_stiffness;
		v.tireForceParams[i].longStiff = (PxReal)w.tire_longitudinal_stiffness;
		v.tireForceParams[i].camberStiff = 0.0f;
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
	v.physxRoadGeometryQueryParams.defaultFilterData = PxQueryFilterData(PxFilterData(0, 0, 0, 0), PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC);
	v.physxRoadGeometryQueryParams.filterCallback = nullptr;
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
	PxMaterial *chassis_material = physics.createMaterial(0.0f, 0.0f, 0.1f);
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
		// Deliberately NOT eSCENE_QUERY_SHAPE: the road-geometry raycast each
		// wheel casts starts near its own suspension attachment point, which
		// sits inside this box's own vertical extent -- with the box a valid
		// query target, every wheel's raycast hit its own car's chassis
		// instead of the ground (jounce pinned at 0, zero tire load, the
		// actor's real PxRigidDynamic origin settling flush on the ground
		// instead of the wheels ever bearing the car's weight -- a real
		// regression caught by comparing rigidBodyState.pose, which is
		// CoM-relative, against a raw getGlobalPose() read). Simulation
		// contacts (blocking other bodies) and scene queries (raycasts
		// hitting it) are independent PhysX flags; only the former is
		// wanted here. Trade-off: this also means an ordinary Godot-side
		// raycast query (e.g. a gameplay "aim" raycast) can't hit this car's
		// body either -- acceptable for the immediate ask (the car should
		// physically block/be blocked by other objects), revisit if
		// raycast-pickability is ever needed too.
		// Wheels stay at flags(0): their ground contact is entirely the
		// road-geometry raycast + suspension, not real shape collision, and
		// making them real simulation shapes too would double-apply ground
		// reaction forces on top of that.
		const PxFilterData chassisFilterData((PxU32)cfg.collision_layer, (PxU32)cfg.collision_mask, 0, 0);
		const PxShapeFlags chassisShapeFlags(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eVISUALIZATION);
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

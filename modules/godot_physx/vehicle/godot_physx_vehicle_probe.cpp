/**************************************************************************/
/*  godot_physx_vehicle_probe.cpp                                         */
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

#include "godot_physx_vehicle_probe.h"

#include "../godot_physx_conversions.h"
#include "../godot_physx_server_3d.h"
#include "../spaces/godot_physx_space_3d.h"

#include "core/object/class_db.h"

#include <PxPhysicsAPI.h>
#include <vehicle/PxVehicleAPI.h>

using namespace physx;

namespace {

// A single direct-drive 4-wheel vehicle, composed exactly the way PhysX's own
// snippetvehiclecommon (Base/DirectDrivetrain/PhysXIntegration) composes one --
// ported here rather than linked, since that layer is snippet source, not part
// of the installed PhysXVehicle_static_64 lib (which only supplies the real
// per-component math these getDataForXComponent overrides hand pointers into).
// Wheel index convention: 0=FL, 1=FR, 2=RL, 3=RR.
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

} //namespace

struct GodotPhysXVehicleProbe::Impl {
	Vehicle4W vehicle;
	PxVehiclePhysXSimulationContext simulationContext;
	PxScene *scene = nullptr;
	bool initialized = false;
};

GodotPhysXVehicleProbe::GodotPhysXVehicleProbe() {
	impl = memnew(Impl);
}

GodotPhysXVehicleProbe::~GodotPhysXVehicleProbe() {
	if (impl) {
		if (impl->initialized && impl->scene) {
			impl->scene->removeActor(*impl->vehicle.physxActor.rigidBody);
			impl->vehicle.destroy();
		}
		memdelete(impl);
	}
}

bool GodotPhysXVehicleProbe::initialize(RID p_space, const Vector3 &p_position) {
	ERR_FAIL_COND_V(impl->initialized, false);

	GodotPhysXServer3D *server = GodotPhysXServer3D::get_singleton();
	ERR_FAIL_NULL_V(server, false);
	GodotPhysXSpace3D *space = server->get_space(p_space);
	ERR_FAIL_NULL_V(space, false);
	PxPhysics *physics = space->get_px_physics();
	PxScene *scene = space->get_px_scene();
	ERR_FAIL_NULL_V(physics, false);
	ERR_FAIL_NULL_V(scene, false);

	Vehicle4W &v = impl->vehicle;
	v.setToDefault();

	// Godot convention: forward = -Z, right = +X, up = +Y (matches
	// godot_physx_conversions.h's 1:1 PhysX<->Godot component mapping, so no
	// axis remap is needed when reading the chassis pose back out).
	v.frame.lngAxis = PxVehicleAxes::eNegZ;
	v.frame.latAxis = PxVehicleAxes::ePosX;
	v.frame.vrtAxis = PxVehicleAxes::ePosY;
	v.scale.scale = 1.0f;

	const PxU32 frontWheels[2] = { Vehicle4W::WHEEL_FL, Vehicle4W::WHEEL_FR };
	const PxU32 rearWheels[2] = { Vehicle4W::WHEEL_RL, Vehicle4W::WHEEL_RR };
	v.axleDescription.setToDefault();
	v.axleDescription.addAxle(2, frontWheels);
	v.axleDescription.addAxle(2, rearWheels);

	v.suspensionStateCalculationParams.suspensionJounceCalculationType = PxVehicleSuspensionJounceCalculationType::eRAYCAST;
	v.suspensionStateCalculationParams.limitSuspensionExpansionVelocity = false;

	// Regular brake: all 4 wheels; handbrake: rear only.
	v.brakeResponseParams[0].maxResponse = 6000.0f;
	v.brakeResponseParams[1].maxResponse = 6000.0f;
	for (PxU32 i = 0; i < 4; i++) {
		v.brakeResponseParams[0].wheelResponseMultipliers[i] = 1.0f;
		v.brakeResponseParams[1].wheelResponseMultipliers[i] = (i == Vehicle4W::WHEEL_RL || i == Vehicle4W::WHEEL_RR) ? 1.0f : 0.0f;
	}
	v.steerResponseParams.maxResponse = 0.6f;
	for (PxU32 i = 0; i < 4; i++) {
		v.steerResponseParams.wheelResponseMultipliers[i] = (i == Vehicle4W::WHEEL_FL || i == Vehicle4W::WHEEL_FR) ? 1.0f : 0.0f;
	}
	v.ackermannParams[0].wheelIds[0] = Vehicle4W::WHEEL_FL;
	v.ackermannParams[0].wheelIds[1] = Vehicle4W::WHEEL_FR;
	v.ackermannParams[0].wheelBase = 2.7f;
	v.ackermannParams[0].trackWidth = 1.5f;
	v.ackermannParams[0].strength = 1.0f;

	v.directDriveThrottleResponseParams.maxResponse = 700.0f;
	for (PxU32 i = 0; i < 4; i++) {
		v.directDriveThrottleResponseParams.wheelResponseMultipliers[i] = 1.0f;
	}

	// A ~1500kg sedan: half-track 0.75m, wheelbase 2.7m (front axle +1.35, rear -1.35).
	const PxReal halfTrack = 0.75f;
	const PxReal frontZ = 1.35f;
	const PxReal rearZ = -1.35f;
	const PxReal wheelRadius = 0.35f;
	const PxReal wheelHalfWidth = 0.15f;
	const PxReal suspensionTravel = 0.15f;
	const PxReal hubDropZ = 0.05f; // small drop from chassis-frame origin to hub attach point

	struct WheelLayout {
		PxReal x, z;
	};
	const WheelLayout layout[4] = {
		{ -halfTrack, frontZ }, // FL
		{ halfTrack, frontZ }, // FR
		{ -halfTrack, rearZ }, // RL
		{ halfTrack, rearZ }, // RR
	};

	v.rigidBodyParams.mass = 1500.0f;
	v.rigidBodyParams.moi = PxVec3(2000.0f, 2200.0f, 1000.0f);

	for (PxU32 i = 0; i < 4; i++) {
		v.wheelParams[i].radius = wheelRadius;
		v.wheelParams[i].halfWidth = wheelHalfWidth;
		v.wheelParams[i].mass = 20.0f;
		v.wheelParams[i].moi = 1.2f;
		v.wheelParams[i].dampingRate = 0.25f;

		v.suspensionParams[i].suspensionAttachment = PxTransform(PxVec3(layout[i].x, hubDropZ, layout[i].z));
		v.suspensionParams[i].suspensionTravelDir = PxVec3(0.0f, -1.0f, 0.0f);
		v.suspensionParams[i].suspensionTravelDist = suspensionTravel;
		v.suspensionParams[i].wheelAttachment = PxTransform(PxIdentity);

		v.suspensionComplianceParams[i] = PxVehicleSuspensionComplianceParams(); // no toe/camber/force-offset curves

		v.suspensionForceParams[i].stiffness = 35000.0f;
		v.suspensionForceParams[i].damping = 4500.0f;
		v.suspensionForceParams[i].sprungMass = v.rigidBodyParams.mass * 0.25f;

		v.tireForceParams[i].latStiffX = 0.01f;
		v.tireForceParams[i].latStiffY = 20000.0f;
		v.tireForceParams[i].longStiff = 20000.0f;
		v.tireForceParams[i].camberStiff = 0.0f;
		v.tireForceParams[i].restLoad = v.suspensionForceParams[i].sprungMass * 9.81f;
		v.tireForceParams[i].frictionVsSlip[0][0] = 0.0f;
		v.tireForceParams[i].frictionVsSlip[0][1] = 1.0f;
		v.tireForceParams[i].frictionVsSlip[1][0] = 0.1f;
		v.tireForceParams[i].frictionVsSlip[1][1] = 1.0f;
		v.tireForceParams[i].frictionVsSlip[2][0] = 1.0f;
		v.tireForceParams[i].frictionVsSlip[2][1] = 1.0f;
		v.tireForceParams[i].loadFilter[0][0] = 0.0f;
		v.tireForceParams[i].loadFilter[0][1] = 0.23f;
		v.tireForceParams[i].loadFilter[1][0] = 3.0f;
		v.tireForceParams[i].loadFilter[1][1] = 3.0f;

		v.physxMaterialFrictionParams[i].defaultFriction = 1.0f;
		v.physxMaterialFrictionParams[i].materialFrictions = nullptr;
		v.physxMaterialFrictionParams[i].nbMaterialFrictions = 0;

		v.physxSuspensionLimitConstraintParams[i].restitution = 0.0f;
		v.physxSuspensionLimitConstraintParams[i].directionForSuspensionLimitConstraint = PxVehiclePhysXSuspensionLimitConstraintParams::eROAD_GEOMETRY_NORMAL;

		v.physxWheelShapeLocalPoses[i] = PxTransform(PxIdentity);
	}

	if (!v.axleDescription.isValid()) {
		ERR_PRINT("PhysX vehicle probe: invalid axle description.");
		return false;
	}
	if (!v.rigidBodyParams.isValid()) {
		ERR_PRINT("PhysX vehicle probe: invalid rigid body params.");
		return false;
	}

	v.physxRoadGeometryQueryParams.roadGeometryQueryType = PxVehiclePhysXRoadGeometryQueryType::eRAYCAST;
	v.physxRoadGeometryQueryParams.defaultFilterData = PxQueryFilterData(PxFilterData(0, 0, 0, 0), PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC);
	v.physxRoadGeometryQueryParams.filterCallback = nullptr;
	v.physxRoadGeometryQueryParams.filterDataEntries = nullptr;

	v.physxActorCMassLocalPose = PxTransform(PxVec3(0.0f, 0.35f, 0.0f));
	v.physxActorBoxShapeHalfExtents = PxVec3(halfTrack + wheelHalfWidth + 0.05f, 0.4f, frontZ + 0.5f);
	v.physxActorBoxShapeLocalPose = PxTransform(PxVec3(0.0f, 0.4f, 0.0f));

	PxMaterial *material = physics->createMaterial(1.0f, 1.0f, 0.1f);
	ERR_FAIL_NULL_V(material, false);
	PxCookingParams cookingParams(physics->getTolerancesScale());

	{
		// PxShapeFlags(0) matches PhysX's own reference vehicle (VhPhysXActorHelpers.cpp):
		// neither eSIMULATION_SHAPE nor eSCENE_QUERY_SHAPE is set, so these shapes never
		// generate normal contacts or get hit by other queries -- weight support comes
		// entirely from the road-geometry raycast (a separate query against the ground,
		// which has real shape flags) plus direct rigid-body force integration, not from
		// these shapes colliding. A real PhysXVehicle3D node would want real flags + Godot-
		// convention filter data here so the car's body can be hit by other objects.
		const PxVehiclePhysXRigidActorParams actorParams(v.rigidBodyParams, nullptr);
		const PxBoxGeometry boxGeom(v.physxActorBoxShapeHalfExtents);
		const PxVehiclePhysXRigidActorShapeParams actorShapeParams(boxGeom, v.physxActorBoxShapeLocalPose, *material, PxShapeFlags(0), PxFilterData(), PxFilterData());
		const PxVehiclePhysXWheelParams wheelParams(v.axleDescription, v.wheelParams);
		const PxVehiclePhysXWheelShapeParams wheelShapeParams(*material, PxShapeFlags(0), PxFilterData(), PxFilterData());

		PxVehiclePhysXActorCreate(
				v.frame,
				actorParams, v.physxActorCMassLocalPose, actorShapeParams,
				wheelParams, wheelShapeParams,
				*physics, cookingParams,
				v.physxActor);
	}
	material->release();

	if (!v.physxActor.rigidBody) {
		ERR_PRINT("PhysX vehicle probe: PxVehiclePhysXActorCreate failed.");
		return false;
	}

	PxVehicleConstraintsCreate(v.axleDescription, *physics, *v.physxActor.rigidBody, v.physxConstraints);

	v.initComponentSequence();

	v.transmissionCommandState.gear = PxVehicleDirectDriveTransmissionCommandState::eFORWARD;

	const PxTransform startPose(to_px(p_position), PxQuat(PxIdentity));
	v.physxActor.rigidBody->setGlobalPose(startPose);
	scene->addActor(*v.physxActor.rigidBody);
	v.physxActor.rigidBody->setName("GodotPhysXVehicleProbe");

	impl->simulationContext.setToDefault();
	impl->simulationContext.frame = v.frame;
	impl->simulationContext.scale = v.scale;
	impl->simulationContext.gravity = v.frame.getVrtAxis() * -9.81f;
	impl->simulationContext.physxScene = scene;
	impl->simulationContext.physxUnitCylinderSweepMesh = nullptr;

	impl->scene = scene;
	impl->initialized = true;
	return true;
}

void GodotPhysXVehicleProbe::step(real_t p_dt, real_t p_throttle, real_t p_brake, real_t p_steer) {
	ERR_FAIL_COND(!impl->initialized);
	Vehicle4W &v = impl->vehicle;
	v.commandState.throttle = (PxReal)p_throttle;
	v.commandState.brakes[0] = (PxReal)p_brake;
	v.commandState.nbBrakes = 1;
	v.commandState.steer = (PxReal)p_steer;
	v.step((PxReal)p_dt, impl->simulationContext);
}

Vector3 GodotPhysXVehicleProbe::get_position() const {
	ERR_FAIL_COND_V(!impl->initialized, Vector3());
	return to_godot(impl->vehicle.rigidBodyState.pose.p);
}

Vector3 GodotPhysXVehicleProbe::get_linear_velocity() const {
	ERR_FAIL_COND_V(!impl->initialized, Vector3());
	return to_godot(impl->vehicle.rigidBodyState.linearVelocity);
}

real_t GodotPhysXVehicleProbe::get_forward_speed() const {
	ERR_FAIL_COND_V(!impl->initialized, 0.0);
	const PxVec3 fwd = impl->vehicle.frame.getLngAxis();
	return (real_t)impl->vehicle.rigidBodyState.linearVelocity.dot(fwd);
}

void GodotPhysXVehicleProbe::_bind_methods() {
	ClassDB::bind_method(D_METHOD("initialize", "space", "position"), &GodotPhysXVehicleProbe::initialize);
	ClassDB::bind_method(D_METHOD("step", "dt", "throttle", "brake", "steer"), &GodotPhysXVehicleProbe::step);
	ClassDB::bind_method(D_METHOD("get_position"), &GodotPhysXVehicleProbe::get_position);
	ClassDB::bind_method(D_METHOD("get_linear_velocity"), &GodotPhysXVehicleProbe::get_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_forward_speed"), &GodotPhysXVehicleProbe::get_forward_speed);
}

/**************************************************************************/
/*  godot_physx_wheel_query_filter.h                                      */
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

#include "../objects/godot_physx_area_3d.h"
#include "../objects/godot_physx_body_3d.h"

#include <PxPhysicsAPI.h>

// Filter for the wheels' road-geometry raycasts (PxVehiclePhysXRoadGeometryQueryParams::filterCallback). Without one,
// a wheel hit every scene-query shape: Area3D triggers, and bodies on layers outside the vehicle's collision_mask
// (characters, debris, the driver). Like a Godot raycast, a wheel now hits only a non-trigger shape whose collision
// layer (query filter word0) is in the vehicle's mask. One per vehicle: the query params point at it, so it lives
// in the vehicle object itself.
// The chassis is a scene-query shape too (Godot raycasts hit vehicles), so a wheel skips its own vehicle's actor --
// its ray starts inside the chassis box -- and, like the chassis, anything the vehicle has a collision exception with.
class GodotPhysXWheelQueryFilter : public physx::PxQueryFilterCallback {
public:
	physx::PxU32 collision_mask = 1;
	const physx::PxRigidActor *self_actor = nullptr;
	const GodotPhysXBody3D *self_body = nullptr; // the vehicle's identity (exceptions), once the node binds it

	virtual physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData &, const physx::PxShape *p_shape, const physx::PxRigidActor *p_actor, physx::PxHitFlags &) override {
		if (p_shape == nullptr || p_actor == self_actor || GodotPhysXArea3D::is_area_actor(p_actor)) {
			return physx::PxQueryHitType::eNONE;
		}
		if (!(p_shape->getQueryFilterData().word0 & collision_mask)) {
			return physx::PxQueryHitType::eNONE;
		}
		// Every other shape belongs to a body (or a vehicle's stand-in), which userData points at.
		const GodotPhysXBody3D *other = p_actor ? static_cast<const GodotPhysXBody3D *>(p_actor->userData) : nullptr;
		if (self_body && other && (self_body->has_collision_exception(other->get_self()) || other->has_collision_exception(self_body->get_self()))) {
			return physx::PxQueryHitType::eNONE;
		}
		return physx::PxQueryHitType::eBLOCK;
	}
	virtual physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData &, const physx::PxQueryHit &, const physx::PxShape *, const physx::PxRigidActor *) override {
		return physx::PxQueryHitType::eBLOCK;
	}
};

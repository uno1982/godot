/**************************************************************************/
/*  godot_physx_material.h                                                */
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

#include "core/math/math_funcs.h"
#include "core/typedefs.h"

#include <PxPhysicsAPI.h>

// Godot's contact-material rule (Godot Physics and Jolt alike): friction = |min(a, b)| -- a `rough`
// PhysicsMaterial arrives NEGATIVE, so it wins -- and bounce = clamp(a + b, 0, 1). PhysX instead combines per
// material by a mode (default AVERAGE), the higher-priority mode of the two winning (AVERAGE < MIN < MULTIPLY <
// MAX), and rejects a negative friction. So: MIN for an ordinary friction and MAX for a rough one (exact unless
// the other side's friction is the higher), MAX for bounce (exact while one side's is 0, the usual case; two
// bouncy bodies get the larger, not the sum; an `absorbent` one counts as 0).
// p_friction / p_bounce are signed the way PhysicsMaterial::computed_friction() / computed_bounce() return them.
inline void godot_physx_apply_material(physx::PxMaterial *p_material, real_t p_friction, real_t p_bounce) {
	const physx::PxReal f = (physx::PxReal)Math::abs(p_friction);
	p_material->setStaticFriction(f);
	p_material->setDynamicFriction(f);
	p_material->setFrictionCombineMode(p_friction < 0.0 ? physx::PxCombineMode::eMAX : physx::PxCombineMode::eMIN);
	p_material->setRestitution((physx::PxReal)CLAMP(p_bounce, (real_t)0.0, (real_t)1.0));
	p_material->setRestitutionCombineMode(physx::PxCombineMode::eMAX);
}

// The same, onto the material of every simulation shape of a live actor -- a PxVehicle2 actor's chassis box
// (its wheel shapes are not simulation shapes). The material must be the actor's own, not shared.
inline void godot_physx_apply_material_to_simulation_shapes(physx::PxRigidActor *p_actor, real_t p_friction, real_t p_bounce) {
	if (p_actor == nullptr) {
		return;
	}
	const physx::PxU32 n = p_actor->getNbShapes();
	for (physx::PxU32 i = 0; i < n; i++) {
		physx::PxShape *shape = nullptr;
		p_actor->getShapes(&shape, 1, i);
		if (shape == nullptr || !(shape->getFlags() & physx::PxShapeFlag::eSIMULATION_SHAPE)) {
			continue;
		}
		physx::PxMaterial *material = nullptr;
		if (shape->getMaterials(&material, 1, 0) == 1 && material != nullptr) {
			godot_physx_apply_material(material, p_friction, p_bounce);
		}
	}
}

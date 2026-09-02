/**************************************************************************/
/*  godot_physx_conversions.h                                             */
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

#include "core/math/basis.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"

#include <foundation/PxQuat.h>
#include <foundation/PxTransform.h>
#include <foundation/PxVec3.h>

_FORCE_INLINE_ physx::PxVec3 to_px(const Vector3 &p_v) {
	return physx::PxVec3((physx::PxReal)p_v.x, (physx::PxReal)p_v.y, (physx::PxReal)p_v.z);
}

_FORCE_INLINE_ Vector3 to_godot(const physx::PxVec3 &p_v) {
	return Vector3((real_t)p_v.x, (real_t)p_v.y, (real_t)p_v.z);
}

_FORCE_INLINE_ physx::PxQuat to_px(const Quaternion &p_q) {
	return physx::PxQuat((physx::PxReal)p_q.x, (physx::PxReal)p_q.y, (physx::PxReal)p_q.z, (physx::PxReal)p_q.w);
}

_FORCE_INLINE_ Quaternion to_godot(const physx::PxQuat &p_q) {
	return Quaternion((real_t)p_q.x, (real_t)p_q.y, (real_t)p_q.z, (real_t)p_q.w);
}

_FORCE_INLINE_ physx::PxTransform to_px(const Transform3D &p_xform) {
	// PhysX transforms are rigid (rotation + translation only); scale is applied
	// at the geometry level, not here.
	physx::PxVec3 p = to_px(p_xform.origin);
	physx::PxQuat q = to_px(p_xform.basis.get_rotation_quaternion());
	// A degenerate/NaN transform from script would assert or crash PhysX; clamp
	// it to identity instead.
	if (!p.isFinite() || !q.isFinite() || q.magnitudeSquared() < 1e-6f) {
		return physx::PxTransform(p.isFinite() ? p : physx::PxVec3(0.0f), physx::PxQuat(physx::PxIdentity));
	}
	return physx::PxTransform(p, q.getNormalized());
}

_FORCE_INLINE_ Transform3D to_godot(const physx::PxTransform &p_xform) {
	return Transform3D(Basis(to_godot(p_xform.q)), to_godot(p_xform.p));
}

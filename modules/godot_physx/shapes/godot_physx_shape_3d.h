/**************************************************************************/
/*  godot_physx_shape_3d.h                                                */
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

#include "core/templates/rid.h"
#include "core/templates/rid_owner.h"
#include "core/variant/variant.h"
#include "servers/physics_3d/physics_server_3d.h"

#include <foundation/PxTransform.h>
#include <geometry/PxBoxGeometry.h>
#include <geometry/PxCapsuleGeometry.h>
#include <geometry/PxConvexMeshGeometry.h>
#include <geometry/PxPlaneGeometry.h>
#include <geometry/PxSphereGeometry.h>
#include <geometry/PxTriangleMeshGeometry.h>

class GodotPhysXShape3D;

// Local pose offset that a shape wants applied when attached to an actor
// (PhysX capsules/planes are X-axis aligned; Godot expects Y / plane-at-origin).
struct GodotPhysXShapeGeometry {
	// Only one of these is valid, per `type`.
	physx::PxBoxGeometry box{ physx::PxVec3(0.5f) };
	physx::PxSphereGeometry sphere{ 0.5f };
	physx::PxCapsuleGeometry capsule{ 0.5f, 0.5f };
	physx::PxPlaneGeometry plane{};
	physx::PxConvexMeshGeometry convex;
	physx::PxTriangleMeshGeometry trimesh;

	physx::PxGeometryType::Enum type = physx::PxGeometryType::eINVALID;
	physx::PxTransform local_pose{ physx::PxIdentity };

	const physx::PxGeometry &geometry() const;
};

class GodotPhysXShape3D {
	RID self;
	PhysicsServer3D::ShapeType type = PhysicsServer3D::SHAPE_CUSTOM;
	Variant data;
	real_t margin = 0.04;

	GodotPhysXShapeGeometry geom;
	bool geom_valid = false;

	// Owned cooked meshes (reference-counted by PhysX; released on re-cook/destroy).
	physx::PxConvexMesh *convex_mesh = nullptr;
	physx::PxTriangleMesh *triangle_mesh = nullptr;
	void _release_meshes();

public:
	~GodotPhysXShape3D();

	void set_self(const RID &p_self) { self = p_self; }
	RID get_self() const { return self; }

	void set_type(PhysicsServer3D::ShapeType p_type) { type = p_type; }
	PhysicsServer3D::ShapeType get_type() const { return type; }

	void set_data(const Variant &p_data);
	Variant get_data() const { return data; }

	void set_margin(real_t p_margin) { margin = p_margin; }
	real_t get_margin() const { return margin; }

	bool is_valid() const { return geom_valid; }
	bool is_trimesh() const { return type == PhysicsServer3D::SHAPE_CONCAVE_POLYGON; }
	const GodotPhysXShapeGeometry &get_geometry() const { return geom; }
};

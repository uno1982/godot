/**************************************************************************/
/*  godot_physx_server_3d.h                                               */
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

#include "core/templates/hash_map.h"
#include "core/templates/hash_set.h"
#include "core/templates/rid_owner.h"
#include "servers/physics_3d/physics_server_3d_dummy.h"

namespace physx {
class PxFoundation;
class PxPhysics;
class PxDefaultCpuDispatcher;
class PxCudaContextManager;
} //namespace physx

class GodotPhysXSpace3D;
class GodotPhysXBody3D;
class GodotPhysXShape3D;
class GodotPhysXArea3D;
class GodotPhysXJoint3D;
class GodotPhysXParticleFluid3D;
class GodotPhysXCloth3D;

// PhysX 5 implementation of PhysicsServer3D, built on top of PhysicsServer3DDummy
// so that only the methods the backend actually implements need to be overridden;
// everything else stays inherited no-op. See README.md for the supported feature
// set and known limitations.
class GodotPhysXServer3D : public PhysicsServer3DDummy {
	GDCLASS(GodotPhysXServer3D, PhysicsServer3DDummy);

	static GodotPhysXServer3D *singleton;

	physx::PxFoundation *px_foundation = nullptr;
	physx::PxPhysics *px_physics = nullptr;
	physx::PxDefaultCpuDispatcher *px_dispatcher = nullptr;
	physx::PxCudaContextManager *px_cuda = nullptr; // null unless a GPU build with a usable CUDA device

	mutable RID_PtrOwner<GodotPhysXSpace3D, true> space_owner;
	mutable RID_PtrOwner<GodotPhysXArea3D, true> area_owner;
	mutable RID_PtrOwner<GodotPhysXBody3D, true> body_owner;
	mutable RID_PtrOwner<GodotPhysXShape3D, true> shape_owner;
	mutable RID_PtrOwner<GodotPhysXJoint3D, true> joint_owner;
	mutable RID_PtrOwner<GodotPhysXParticleFluid3D, true> fluid_owner;
	mutable RID_PtrOwner<GodotPhysXCloth3D, true> cloth_owner;

	HashSet<GodotPhysXSpace3D *> active_spaces;

	bool active = true;
	bool flushing_queries = false;
	int active_objects = 0;

	RID _shape_create(PhysicsServer3D::ShapeType p_type);

public:
	static GodotPhysXServer3D *get_singleton() { return singleton; }

	// Lookups for the direct space state (scene queries).
	GodotPhysXShape3D *get_shape(RID p_rid) const { return shape_owner.get_or_null(p_rid); }
	GodotPhysXBody3D *get_body(RID p_rid) const { return body_owner.get_or_null(p_rid); }

	physx::PxPhysics *get_px_physics() const { return px_physics; }

	/* SHAPE API */
	virtual RID world_boundary_shape_create() override;
	virtual RID sphere_shape_create() override;
	virtual RID box_shape_create() override;
	virtual RID capsule_shape_create() override;
	virtual RID convex_polygon_shape_create() override;
	virtual RID concave_polygon_shape_create() override;
	virtual RID cylinder_shape_create() override;
	virtual RID separation_ray_shape_create() override;
	virtual RID heightmap_shape_create() override;
	virtual RID custom_shape_create() override;

	virtual void shape_set_data(RID p_shape, const Variant &p_data) override;
	virtual void shape_set_margin(RID p_shape, real_t p_margin) override;
	virtual real_t shape_get_margin(RID p_shape) const override;
	virtual ShapeType shape_get_type(RID p_shape) const override;
	virtual Variant shape_get_data(RID p_shape) const override;

	/* SPACE API */
	virtual RID space_create() override;
	virtual void space_set_active(RID p_space, bool p_active) override;
	virtual bool space_is_active(RID p_space) const override;
	virtual void space_set_param(RID p_space, SpaceParameter p_param, real_t p_value) override;
	virtual real_t space_get_param(RID p_space, SpaceParameter p_param) const override;
	virtual PhysicsDirectSpaceState3D *space_get_direct_state(RID p_space) override;

	/* AREA API (minimal: the space RID doubles as its default area for gravity) */
	virtual RID area_create() override;
	virtual void area_set_space(RID p_area, RID p_space) override;
	virtual RID area_get_space(RID p_area) const override;
	virtual void area_add_shape(RID p_area, RID p_shape, const Transform3D &p_transform = Transform3D(), bool p_disabled = false) override;
	virtual void area_set_shape(RID p_area, int p_shape_idx, RID p_shape) override;
	virtual void area_set_shape_transform(RID p_area, int p_shape_idx, const Transform3D &p_transform) override;
	virtual int area_get_shape_count(RID p_area) const override;
	virtual RID area_get_shape(RID p_area, int p_shape_idx) const override;
	virtual Transform3D area_get_shape_transform(RID p_area, int p_shape_idx) const override;
	virtual void area_remove_shape(RID p_area, int p_shape_idx) override;
	virtual void area_clear_shapes(RID p_area) override;
	virtual void area_set_shape_disabled(RID p_area, int p_shape_idx, bool p_disabled) override;
	virtual void area_attach_object_instance_id(RID p_area, ObjectID p_id) override;
	virtual ObjectID area_get_object_instance_id(RID p_area) const override;
	virtual void area_set_param(RID p_area, AreaParameter p_param, const Variant &p_value) override;
	virtual void area_set_transform(RID p_area, const Transform3D &p_transform) override;
	virtual Variant area_get_param(RID p_area, AreaParameter p_param) const override;
	virtual Transform3D area_get_transform(RID p_area) const override;
	virtual void area_set_collision_layer(RID p_area, uint32_t p_layer) override;
	virtual uint32_t area_get_collision_layer(RID p_area) const override;
	virtual void area_set_collision_mask(RID p_area, uint32_t p_mask) override;
	virtual uint32_t area_get_collision_mask(RID p_area) const override;
	virtual void area_set_monitorable(RID p_area, bool p_monitorable) override;
	virtual void area_set_monitor_callback(RID p_area, const Callable &p_callback) override;

	/* BODY API */
	virtual RID body_create() override;
	virtual void body_set_space(RID p_body, RID p_space) override;
	virtual RID body_get_space(RID p_body) const override;
	virtual void body_set_mode(RID p_body, BodyMode p_mode) override;
	virtual BodyMode body_get_mode(RID p_body) const override;

	virtual void body_add_shape(RID p_body, RID p_shape, const Transform3D &p_transform = Transform3D(), bool p_disabled = false) override;
	virtual void body_set_shape(RID p_body, int p_shape_idx, RID p_shape) override;
	virtual void body_set_shape_transform(RID p_body, int p_shape_idx, const Transform3D &p_transform) override;
	virtual int body_get_shape_count(RID p_body) const override;
	virtual RID body_get_shape(RID p_body, int p_shape_idx) const override;
	virtual Transform3D body_get_shape_transform(RID p_body, int p_shape_idx) const override;
	virtual void body_remove_shape(RID p_body, int p_shape_idx) override;
	virtual void body_clear_shapes(RID p_body) override;
	virtual void body_set_shape_disabled(RID p_body, int p_shape_idx, bool p_disabled) override;

	virtual void body_attach_object_instance_id(RID p_body, ObjectID p_id) override;
	virtual ObjectID body_get_object_instance_id(RID p_body) const override;

	virtual void body_set_collision_layer(RID p_body, uint32_t p_layer) override;
	virtual uint32_t body_get_collision_layer(RID p_body) const override;
	virtual void body_set_collision_mask(RID p_body, uint32_t p_mask) override;
	virtual uint32_t body_get_collision_mask(RID p_body) const override;

	virtual void body_set_enable_continuous_collision_detection(RID p_body, bool p_enable) override;
	virtual bool body_is_continuous_collision_detection_enabled(RID p_body) const override;

	virtual void body_set_param(RID p_body, BodyParameter p_param, const Variant &p_value) override;
	virtual Variant body_get_param(RID p_body, BodyParameter p_param) const override;

	virtual void body_set_state(RID p_body, BodyState p_state, const Variant &p_variant) override;
	virtual Variant body_get_state(RID p_body, BodyState p_state) const override;

	virtual void body_apply_central_impulse(RID p_body, const Vector3 &p_impulse) override;
	virtual void body_apply_impulse(RID p_body, const Vector3 &p_impulse, const Vector3 &p_position = Vector3()) override;
	virtual void body_apply_torque_impulse(RID p_body, const Vector3 &p_impulse) override;
	virtual void body_apply_central_force(RID p_body, const Vector3 &p_force) override;

	virtual void body_set_axis_lock(RID p_body, BodyAxis p_axis, bool p_lock) override;
	virtual bool body_is_axis_locked(RID p_body, BodyAxis p_axis) const override;

	virtual void body_set_max_contacts_reported(RID p_body, int p_contacts) override;
	virtual int body_get_max_contacts_reported(RID p_body) const override;

	virtual void body_set_state_sync_callback(RID p_body, const Callable &p_callable) override;
	virtual PhysicsDirectBodyState3D *body_get_direct_state(RID p_body) override;

	virtual bool body_test_motion(RID p_body, const MotionParameters &p_parameters, MotionResult *r_result = nullptr) override;

	/* JOINT API */
	virtual RID joint_create() override;
	virtual void joint_clear(RID p_joint) override;
	virtual JointType joint_get_type(RID p_joint) const override;
	virtual void joint_set_solver_priority(RID p_joint, int p_priority) override;
	virtual int joint_get_solver_priority(RID p_joint) const override;
	virtual void joint_disable_collisions_between_bodies(RID p_joint, bool p_disable) override;
	virtual bool joint_is_disabled_collisions_between_bodies(RID p_joint) const override;

	virtual void joint_make_pin(RID p_joint, RID p_body_A, const Vector3 &p_local_A, RID p_body_B, const Vector3 &p_local_B) override;
	virtual void pin_joint_set_param(RID p_joint, PinJointParam p_param, real_t p_value) override;
	virtual real_t pin_joint_get_param(RID p_joint, PinJointParam p_param) const override;
	virtual void pin_joint_set_local_a(RID p_joint, const Vector3 &p_A) override;
	virtual Vector3 pin_joint_get_local_a(RID p_joint) const override;
	virtual void pin_joint_set_local_b(RID p_joint, const Vector3 &p_B) override;
	virtual Vector3 pin_joint_get_local_b(RID p_joint) const override;

	virtual void joint_make_hinge(RID p_joint, RID p_body_A, const Transform3D &p_hinge_A, RID p_body_B, const Transform3D &p_hinge_B) override;
	virtual void joint_make_hinge_simple(RID p_joint, RID p_body_A, const Vector3 &p_pivot_A, const Vector3 &p_axis_A, RID p_body_B, const Vector3 &p_pivot_B, const Vector3 &p_axis_B) override;
	virtual void hinge_joint_set_param(RID p_joint, HingeJointParam p_param, real_t p_value) override;
	virtual real_t hinge_joint_get_param(RID p_joint, HingeJointParam p_param) const override;
	virtual void hinge_joint_set_flag(RID p_joint, HingeJointFlag p_flag, bool p_enabled) override;
	virtual bool hinge_joint_get_flag(RID p_joint, HingeJointFlag p_flag) const override;

	virtual void joint_make_slider(RID p_joint, RID p_body_A, const Transform3D &p_local_frame_A, RID p_body_B, const Transform3D &p_local_frame_B) override;
	virtual void slider_joint_set_param(RID p_joint, SliderJointParam p_param, real_t p_value) override;
	virtual real_t slider_joint_get_param(RID p_joint, SliderJointParam p_param) const override;

	virtual void joint_make_cone_twist(RID p_joint, RID p_body_A, const Transform3D &p_local_frame_A, RID p_body_B, const Transform3D &p_local_frame_B) override;
	virtual void cone_twist_joint_set_param(RID p_joint, ConeTwistJointParam p_param, real_t p_value) override;
	virtual real_t cone_twist_joint_get_param(RID p_joint, ConeTwistJointParam p_param) const override;

	virtual void joint_make_generic_6dof(RID p_joint, RID p_body_A, const Transform3D &p_local_frame_A, RID p_body_B, const Transform3D &p_local_frame_B) override;
	virtual void generic_6dof_joint_set_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param, real_t p_value) override;
	virtual real_t generic_6dof_joint_get_param(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisParam p_param) const override;
	virtual void generic_6dof_joint_set_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag, bool p_enable) override;
	virtual bool generic_6dof_joint_get_flag(RID p_joint, Vector3::Axis p_axis, G6DOFJointAxisFlag p_flag) const override;

	/* PARTICLE FLUID (module extension -- not part of PhysicsServer3D) */
	// Called by PhysXParticleFluid3D via GodotPhysXServer3D::get_singleton().
	RID particle_fluid_create();
	void particle_fluid_set_space(RID p_fluid, RID p_space);
	void particle_fluid_set_param(RID p_fluid, int p_param, real_t p_value);
	void particle_fluid_set_capacity(RID p_fluid, int p_max);
	void particle_fluid_set_particles(RID p_fluid, const Vector<Vector3> &p_positions, const Vector3 &p_initial_velocity);
	void particle_fluid_emit(RID p_fluid, const Vector<Vector3> &p_positions, const Vector3 &p_velocity);
	void particle_fluid_clear(RID p_fluid);
	void particle_fluid_set_foam(RID p_fluid, bool p_enabled, int p_capacity, real_t p_lifetime, real_t p_threshold, real_t p_buoyancy, real_t p_size);
	Vector<Vector3> particle_fluid_get_positions(RID p_fluid) const;
	Vector<Vector3> particle_fluid_get_foam_positions(RID p_fluid) const;
	int particle_fluid_get_particle_count(RID p_fluid) const;
	int particle_fluid_get_foam_count(RID p_fluid) const;

	void particle_fluid_set_surface_mesh(RID p_fluid, bool p_enabled);
	void particle_fluid_set_surface_anisotropy(RID p_fluid, bool p_enabled);
	// Fills Godot arrays with the latest GPU isosurface; returns the triangle count.
	// r_version: pass the last-seen version; returns -1 (leaving r_* untouched) if
	// the mesh is unchanged, else the triangle count and the new version.
	int particle_fluid_get_surface_mesh(RID p_fluid, PackedVector3Array &r_vertices, PackedVector3Array &r_normals, PackedInt32Array &r_indices, uint32_t &r_version) const;
	// Same, for the coarse foam isosurface layer (diffuse particles).
	int particle_fluid_get_foam_mesh(RID p_fluid, PackedVector3Array &r_vertices, PackedVector3Array &r_normals, PackedInt32Array &r_indices, uint32_t &r_version) const;
	real_t particle_fluid_get_submersion(RID p_fluid, const AABB &p_world_aabb) const;

	// GPU cloth (PxDeformableSurface). Returns RID() when no CUDA device is
	// available -- the caller then uses its CPU fallback.
	RID cloth_create();
	void cloth_set_space(RID p_cloth, RID p_space);
	void cloth_set_params(RID p_cloth, real_t p_thickness, real_t p_density, real_t p_stretch, real_t p_bend, real_t p_damping, uint32_t p_collision_mask);
	void cloth_build(RID p_cloth, const Vector<Vector3> &p_positions, const Vector<int32_t> &p_indices, const Transform3D &p_xform);
	void cloth_set_pinned(RID p_cloth, const Vector<int32_t> &p_pinned);
	void cloth_set_pin_targets(RID p_cloth, const Vector<Vector3> &p_world_targets);
	void cloth_apply_wind(RID p_cloth, const Vector3 &p_wind, real_t p_drag, real_t p_lift, real_t p_dt);
	bool cloth_is_ready(RID p_cloth) const;
	int cloth_get_mesh(RID p_cloth, PackedVector3Array &r_positions, PackedInt32Array &r_indices, uint32_t &r_version) const;
	bool has_gpu() const { return px_cuda != nullptr; }

	/* MISC */
	virtual void free_rid(RID p_rid) override;
	virtual void set_active(bool p_active) override;
	virtual void init() override;
	virtual void step(real_t p_step) override;
	virtual void flush_queries() override;
	virtual void finish() override;
	virtual bool is_flushing_queries() const override { return flushing_queries; }
	virtual int get_process_info(ProcessInfo p_info) override;

	GodotPhysXServer3D();
	virtual ~GodotPhysXServer3D() override;
};

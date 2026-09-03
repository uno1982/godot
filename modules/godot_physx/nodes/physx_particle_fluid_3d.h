/**************************************************************************/
/*  physx_particle_fluid_3d.h                                             */
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

#include "scene/3d/visual_instance_3d.h"
#include "scene/resources/mesh.h"

// A GPU PBD fluid volume simulated by the PhysX backend (PhysX 5
// PxPBDParticleSystem). It only does anything when the active 3D physics engine
// is "PhysX" and the engine was built with GPU support (physx_gpu=yes) on a
// machine with a CUDA device; otherwise it is inert.
class PhysXParticleFluid3D : public GeometryInstance3D {
	GDCLASS(PhysXParticleFluid3D, GeometryInstance3D);

	RID fluid; // GodotPhysXServer3D particle-fluid RID
	RID multimesh;
	Ref<Mesh> particle_mesh;

	// Foam/spray/bubble particles, drawn in their own MultiMesh + RS instance.
	RID foam_multimesh;
	RID foam_instance;
	Ref<Mesh> foam_mesh;

	int particle_count = 4096;
	float particle_size = 0.1;
	float viscosity = 0.01;
	float surface_tension = 0.006;
	float cohesion = 0.02;
	float vorticity = 8.0;
	Vector3 spawn_region_size = Vector3(1, 1, 1);
	bool spawn_on_ready = true;

	bool emitting = false;
	float emission_rate = 2000.0; // particles per second
	float emission_radius = 0.1; // spawn disc/sphere radius at the node origin
	Vector3 emission_velocity = Vector3(0, -3, 0); // local space; length is speed

	bool foam_enabled = false;
	int foam_particle_count = 16384;
	float foam_lifetime = 1.5;
	float foam_threshold = 300.0;
	float foam_buoyancy = 0.9;

	bool spawned = false;
	double emit_accum = 0.0;

	void _make_fluid();
	void _free_fluid();
	void _apply_params();
	void _apply_foam();
	void _update_render();
	void _emit_step(double p_delta);

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void set_particle_count(int p_count);
	int get_particle_count() const { return particle_count; }
	void set_particle_size(float p_size);
	float get_particle_size() const { return particle_size; }
	void set_viscosity(float p_v);
	float get_viscosity() const { return viscosity; }
	void set_surface_tension(float p_v);
	float get_surface_tension() const { return surface_tension; }
	void set_cohesion(float p_v);
	float get_cohesion() const { return cohesion; }
	void set_vorticity(float p_v);
	float get_vorticity() const { return vorticity; }
	void set_spawn_region_size(const Vector3 &p_size);
	Vector3 get_spawn_region_size() const { return spawn_region_size; }
	void set_spawn_on_ready(bool p_enable);
	bool get_spawn_on_ready() const { return spawn_on_ready; }

	void set_emitting(bool p_emitting);
	bool is_emitting() const { return emitting; }
	void set_emission_rate(float p_rate);
	float get_emission_rate() const { return emission_rate; }
	void set_emission_radius(float p_radius);
	float get_emission_radius() const { return emission_radius; }
	void set_emission_velocity(const Vector3 &p_velocity);
	Vector3 get_emission_velocity() const { return emission_velocity; }

	void set_foam_enabled(bool p_enabled);
	bool is_foam_enabled() const { return foam_enabled; }
	void set_foam_particle_count(int p_count);
	int get_foam_particle_count() const { return foam_particle_count; }
	void set_foam_lifetime(float p_v);
	float get_foam_lifetime() const { return foam_lifetime; }
	void set_foam_threshold(float p_v);
	float get_foam_threshold() const { return foam_threshold; }
	void set_foam_buoyancy(float p_v);
	float get_foam_buoyancy() const { return foam_buoyancy; }
	int get_live_foam_count() const;

	// Fill the region (centered on this node) with a jittered grid of particles.
	void spawn();
	// Remove all particles.
	void clear();
	int get_live_particle_count() const;

	PackedVector3Array get_particle_positions() const;

	// 0..1 fraction of the world-space box currently filled with fluid.
	float get_submersion(const AABB &p_world_aabb) const;

	AABB get_aabb() const override;

	PhysXParticleFluid3D();
	~PhysXParticleFluid3D();
};

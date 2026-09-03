/**************************************************************************/
/*  godot_physx_particle_fluid_3d.cpp                                     */
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

#include "godot_physx_particle_fluid_3d.h"

#include "../godot_physx_conversions.h"
#include "../spaces/godot_physx_space_3d.h"

#include "core/error/error_macros.h"
#include "core/math/math_defs.h"

#include <PxPhysicsAPI.h>
#include <extensions/PxCudaHelpersExt.h>
#include <extensions/PxParticleExt.h>

using namespace physx;

// PhysX's fluid density in kg/m^3; particle mass follows from the spacing.
static constexpr float FLUID_DENSITY = 1000.0f;

static PxDiffuseParticleParams _diffuse_params(float p_lifetime, float p_threshold, float p_buoyancy) {
	PxDiffuseParticleParams params;
	params.threshold = p_threshold;
	params.lifetime = p_lifetime;
	params.buoyancy = p_buoyancy;
	params.airDrag = 0.0f;
	params.bubbleDrag = 0.9f;
	params.kineticEnergyWeight = 0.01f;
	params.pressureWeight = 1.0f;
	params.divergenceWeight = 10.0f;
	params.collisionDecay = 0.5f;
	params.useAccurateVelocity = false;
	return params;
}

GodotPhysXParticleFluid3D::~GodotPhysXParticleFluid3D() {
	set_space(nullptr);
}

void GodotPhysXParticleFluid3D::_destroy() {
	if (px_buffer && px_system) {
		px_system->removeParticleBuffer(px_buffer);
	}
	if (px_buffer) {
		px_buffer->release();
		px_buffer = nullptr;
	}
	if (px_system) {
		if (space && space->get_px_scene()) {
			space->get_px_scene()->removeActor(*px_system);
		}
		px_system->release();
		px_system = nullptr;
	}
	if (px_material) {
		px_material->release();
		px_material = nullptr;
	}
	active_count = 0;
	foam_count = 0;
	read_scratch.clear();
	read_positions.clear();
	foam_scratch.clear();
	foam_positions.clear();
	dirty_material = true;
	dirty_foam = true;
}

void GodotPhysXParticleFluid3D::set_space(GodotPhysXSpace3D *p_space) {
	if (space == p_space) {
		return;
	}
	_destroy();
	if (space) {
		space->unregister_fluid(this);
	}
	space = p_space;
	if (space) {
		space->register_fluid(this);
	}
}

void GodotPhysXParticleFluid3D::_ensure_system() {
	if (px_system) {
		return;
	}
	if (!space) {
		return;
	}
	PxCudaContextManager *cuda = space->get_px_cuda();
	if (!cuda) {
		WARN_PRINT_ONCE("PhysXParticleFluid3D needs GPU dynamics (a physx_gpu build with a CUDA device); the fluid stays inert.");
		return;
	}
	PxPhysics *physics = space->get_px_physics();
	PxScene *scene = space->get_px_scene();
	ERR_FAIL_NULL(physics);
	ERR_FAIL_NULL(scene);

	px_material = physics->createPBDMaterial(
			0.05f, // friction
			0.0f, // damping
			(PxReal)adhesion,
			(PxReal)viscosity,
			(PxReal)vorticity,
			(PxReal)surface_tension,
			(PxReal)cohesion,
			0.0f, // lift (deprecated)
			0.0f); // drag (deprecated)
	ERR_FAIL_NULL(px_material);
	px_material->setGravityScale((PxReal)gravity_scale);

	px_system = physics->createPBDParticleSystem(*cuda, 96);
	ERR_FAIL_NULL_MSG(px_system, "PhysX: createPBDParticleSystem failed.");

	// Rest/contact offsets derived from the particle spacing, matching PhysX's
	// PBF snippet.
	const PxReal spacing = MAX((PxReal)particle_size, 0.001f);
	const PxReal rest_offset = 0.5f * spacing / 0.6f;
	const PxReal fluid_rest_offset = rest_offset * 0.6f;
	px_system->setRestOffset(rest_offset);
	px_system->setContactOffset(rest_offset + 0.01f);
	px_system->setParticleContactOffset(fluid_rest_offset / 0.6f);
	px_system->setSolidRestOffset(rest_offset);
	px_system->setFluidRestOffset(fluid_rest_offset);
	px_system->setMaxLinearVelocity(rest_offset * 100.0f);
	// Without this, speculative contacts let dense bodies rest on the fluid
	// surface instead of sinking through it (PhysX's PBF snippet also disables it).
	px_system->setParticleFlag(PxParticleFlag::eENABLE_SPECULATIVE_CCD, false);

	fluid_phase = px_system->createPhase(px_material,
			PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseFluid | PxParticlePhaseFlag::eParticlePhaseSelfCollide));

	// Our scene filter shader suppresses any pair whose layer/mask cross-check is
	// zero; a particle system's filter data defaults to all-zero, so give it
	// layer bit 0 and a full mask so it collides with the standard body layers.
	px_system->setSimulationFilterData(PxFilterData(1, 0xFFFFFFFF, 0, 0));

	scene->addActor(*px_system);
	dirty_material = false;
}

void GodotPhysXParticleFluid3D::_apply_material() {
	if (!px_material || !dirty_material) {
		return;
	}
	px_material->setViscosity((PxReal)viscosity);
	px_material->setSurfaceTension((PxReal)surface_tension);
	px_material->setCohesion((PxReal)cohesion);
	px_material->setAdhesion((PxReal)adhesion);
	px_material->setVorticityConfinement((PxReal)vorticity);
	px_material->setGravityScale((PxReal)gravity_scale);
	dirty_material = false;
}

void GodotPhysXParticleFluid3D::_apply_foam_params() {
	if (!px_buffer || !dirty_foam) {
		return;
	}
	px_buffer->setDiffuseParticleParams(_diffuse_params((float)foam_lifetime, (float)foam_threshold, (float)foam_buoyancy));
	px_buffer->raiseFlags(PxParticleBufferFlag::eUPDATE_DIFFUSE_PARAM);
	dirty_foam = false;
}

void GodotPhysXParticleFluid3D::set_param(Param p_param, real_t p_value) {
	switch (p_param) {
		case PARAM_VISCOSITY:
			viscosity = p_value;
			break;
		case PARAM_SURFACE_TENSION:
			surface_tension = p_value;
			break;
		case PARAM_COHESION:
			cohesion = p_value;
			break;
		case PARAM_ADHESION:
			adhesion = p_value;
			break;
		case PARAM_VORTICITY:
			vorticity = p_value;
			break;
		case PARAM_GRAVITY_SCALE:
			gravity_scale = p_value;
			break;
		case PARAM_PARTICLE_SIZE:
			particle_size = MAX(p_value, (real_t)0.001);
			break;
		default:
			return;
	}
	dirty_material = true;
	_apply_material();
}

real_t GodotPhysXParticleFluid3D::get_param(Param p_param) const {
	switch (p_param) {
		case PARAM_VISCOSITY:
			return viscosity;
		case PARAM_SURFACE_TENSION:
			return surface_tension;
		case PARAM_COHESION:
			return cohesion;
		case PARAM_ADHESION:
			return adhesion;
		case PARAM_VORTICITY:
			return vorticity;
		case PARAM_GRAVITY_SCALE:
			return gravity_scale;
		case PARAM_PARTICLE_SIZE:
			return particle_size;
		default:
			return 0.0;
	}
}

void GodotPhysXParticleFluid3D::set_capacity(uint32_t p_capacity) {
	p_capacity = MAX(p_capacity, 1u);
	if (p_capacity == capacity) {
		return;
	}
	capacity = p_capacity;
	// The buffer is fixed size; drop it so the next spawn/emit rebuilds it.
	clear();
}

void GodotPhysXParticleFluid3D::clear() {
	if (px_buffer && px_system) {
		px_system->removeParticleBuffer(px_buffer);
	}
	if (px_buffer) {
		px_buffer->release();
		px_buffer = nullptr;
	}
	active_count = 0;
	write_head = 0;
	foam_count = 0;
	read_scratch.clear();
	read_positions.clear();
	foam_scratch.clear();
	foam_positions.clear();
}

void GodotPhysXParticleFluid3D::set_foam_enabled(bool p_enabled) {
	if (foam_enabled == p_enabled) {
		return;
	}
	foam_enabled = p_enabled;
	// The diffuse capacity is baked into the buffer; rebuild it on the next fill.
	clear();
}

void GodotPhysXParticleFluid3D::set_foam_capacity(uint32_t p_capacity) {
	p_capacity = MAX(p_capacity, 1u);
	if (p_capacity == foam_capacity) {
		return;
	}
	foam_capacity = p_capacity;
	if (foam_enabled) {
		clear();
	}
}

void GodotPhysXParticleFluid3D::set_foam_lifetime(real_t p_v) {
	foam_lifetime = MAX(p_v, (real_t)0.01);
	dirty_foam = true;
	_apply_foam_params();
}

void GodotPhysXParticleFluid3D::set_foam_threshold(real_t p_v) {
	foam_threshold = MAX(p_v, (real_t)0.0);
	dirty_foam = true;
	_apply_foam_params();
}

void GodotPhysXParticleFluid3D::set_foam_buoyancy(real_t p_v) {
	foam_buoyancy = CLAMP(p_v, (real_t)0.0, (real_t)1.0);
	dirty_foam = true;
	_apply_foam_params();
}

void GodotPhysXParticleFluid3D::_ensure_buffer() {
	if (px_buffer) {
		return;
	}
	_ensure_system();
	if (!px_system) {
		return;
	}
	PxCudaContextManager *cuda = space->get_px_cuda();

	// Create an empty buffer at full capacity; particles are added later.
	PxVec4 seed_pos(0.0f);
	PxVec4 seed_vel(0.0f);
	PxU32 seed_phase = fluid_phase;
	const PxU32 max_diffuse = foam_enabled ? MAX(foam_capacity, 1u) : 0u;
	ExtGpu::PxParticleAndDiffuseBufferDesc desc;
	desc.maxParticles = capacity;
	desc.numActiveParticles = 0;
	desc.positions = &seed_pos;
	desc.velocities = &seed_vel;
	desc.phases = &seed_phase;
	desc.maxDiffuseParticles = max_diffuse;
	desc.maxActiveDiffuseParticles = max_diffuse;
	desc.diffuseParams = _diffuse_params((float)foam_lifetime, (float)foam_threshold, (float)foam_buoyancy);

	px_buffer = ExtGpu::PxCreateAndPopulateParticleAndDiffuseBuffer(desc, cuda);
	ERR_FAIL_NULL_MSG(px_buffer, "PhysX: PxCreateAndPopulateParticleAndDiffuseBuffer failed.");
	px_system->addParticleBuffer(px_buffer);
	active_count = 0;
	write_head = 0;
	foam_count = 0;
	dirty_foam = false;
}

void GodotPhysXParticleFluid3D::_write_particles(uint32_t p_at, const Vector<Vector3> &p_positions, const Vector3 &p_velocity) {
	const uint32_t n = MIN((uint32_t)p_positions.size(), capacity);
	if (n == 0 || !px_buffer) {
		return;
	}
	PxCudaContextManager *cuda = space->get_px_cuda();
	const PxReal spacing = MAX((PxReal)particle_size, 0.001f);
	// Each particle stands in for a spacing^3 cell of fluid, so its mass is the
	// rest density times that volume -- this keeps the effective fluid density
	// near FLUID_DENSITY.
	const PxReal particle_mass = FLUID_DENSITY * spacing * spacing * spacing;
	const PxReal inv_mass = particle_mass > 0.0f ? 1.0f / particle_mass : 1.0f;
	const PxVec4 vel(to_px(p_velocity), 0.0f);

	LocalVector<PxVec4> positions;
	LocalVector<PxVec4> velocities;
	LocalVector<PxU32> phases;
	positions.resize(n);
	velocities.resize(n);
	phases.resize(n);
	const Vector3 *src = p_positions.ptr();
	for (uint32_t i = 0; i < n; i++) {
		positions[i] = PxVec4(to_px(src[i]), inv_mass);
		velocities[i] = vel;
		phases[i] = fluid_phase;
	}

	PxVec4 *dev_pos = px_buffer->getPositionInvMasses();
	PxVec4 *dev_vel = px_buffer->getVelocities();
	PxU32 *dev_phase = px_buffer->getPhases();

	// Copy in up to two runs so a write that crosses the end of the ring wraps.
	uint32_t done = 0;
	while (done < n) {
		const uint32_t slot = (p_at + done) % capacity;
		const uint32_t run = MIN(n - done, capacity - slot);
		Ext::PxCudaHelpersExt::copyHToD(*cuda, dev_pos + slot, positions.ptr() + done, run);
		Ext::PxCudaHelpersExt::copyHToD(*cuda, dev_vel + slot, velocities.ptr() + done, run);
		Ext::PxCudaHelpersExt::copyHToD(*cuda, dev_phase + slot, phases.ptr() + done, run);
		done += run;
	}
	px_buffer->raiseFlags(PxParticleBufferFlag::eUPDATE_POSITION);
	px_buffer->raiseFlags(PxParticleBufferFlag::eUPDATE_VELOCITY);
	px_buffer->raiseFlags(PxParticleBufferFlag::eUPDATE_PHASE);
}

void GodotPhysXParticleFluid3D::set_particles(const Vector<Vector3> &p_positions, const Vector3 &p_initial_velocity) {
	clear();
	_ensure_buffer();
	if (!px_buffer) {
		return;
	}
	const uint32_t n = MIN((uint32_t)p_positions.size(), capacity);
	_write_particles(0, p_positions, p_initial_velocity);
	active_count = n;
	write_head = n % capacity;
	px_buffer->setNbActiveParticles(active_count);
}

void GodotPhysXParticleFluid3D::emit(const Vector<Vector3> &p_positions, const Vector3 &p_velocity) {
	_ensure_buffer();
	if (!px_buffer) {
		return;
	}
	const uint32_t n = MIN((uint32_t)p_positions.size(), capacity);
	if (n == 0) {
		return;
	}
	_write_particles(write_head, p_positions, p_velocity);
	write_head = (write_head + n) % capacity;
	active_count = MIN(active_count + n, capacity);
	px_buffer->setNbActiveParticles(active_count);
}

void GodotPhysXParticleFluid3D::read_back() {
	if (!px_buffer || !space) {
		return;
	}
	PxCudaContextManager *cuda = space->get_px_cuda();
	if (!cuda) {
		return;
	}
	active_count = px_buffer->getNbActiveParticles();
	if (active_count == 0) {
		read_positions.clear();
		return;
	}
	read_scratch.resize(active_count);
	Ext::PxCudaHelpersExt::copyDToH(*cuda,
			reinterpret_cast<PxVec4 *>(read_scratch.ptr()),
			px_buffer->getPositionInvMasses(),
			active_count);

	read_positions.resize(active_count);
	for (uint32_t i = 0; i < active_count; i++) {
		const Vector4 &p = read_scratch[i];
		read_positions[i] = Vector3(p.x, p.y, p.z);
	}

	foam_count = foam_enabled ? px_buffer->getNbActiveDiffuseParticles() : 0;
	if (foam_count == 0) {
		foam_positions.clear();
		return;
	}
	foam_scratch.resize(foam_count);
	Ext::PxCudaHelpersExt::copyDToH(*cuda,
			reinterpret_cast<PxVec4 *>(foam_scratch.ptr()),
			px_buffer->getDiffusePositionLifeTime(),
			foam_count);
	foam_positions.resize(foam_count);
	for (uint32_t i = 0; i < foam_count; i++) {
		const Vector4 &p = foam_scratch[i];
		foam_positions[i] = Vector3(p.x, p.y, p.z);
	}
}

real_t GodotPhysXParticleFluid3D::get_submersion(const AABB &p_world_aabb) const {
	const real_t box_vol = p_world_aabb.get_volume();
	if (box_vol <= 0.0 || read_positions.is_empty()) {
		return 0.0;
	}
	uint32_t inside = 0;
	const Vector3 *p = read_positions.ptr();
	for (uint32_t i = 0; i < read_positions.size(); i++) {
		if (p_world_aabb.has_point(p[i])) {
			inside++;
		}
	}
	// Each particle stands in for a particle_size^3 cell of fluid.
	const real_t s = MAX(particle_size, (real_t)0.001);
	const real_t filled = (real_t)inside * s * s * s;
	return CLAMP(filled / box_vol, (real_t)0.0, (real_t)1.0);
}

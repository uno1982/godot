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

#include <PxAnisotropy.h>
#include <PxIsosurfaceExtraction.h>
#include <PxPhysicsAPI.h>
#include <PxSmoothing.h>
#include <extensions/PxCudaHelpersExt.h>
#include <extensions/PxParticleExt.h>
#include <gpu/PxGpu.h>
#include <gpu/PxPhysicsGpu.h>

using namespace physx;

// PhysX's fluid density in kg/m^3; particle mass follows from the spacing.
static constexpr float FLUID_DENSITY = 1000.0f;

// GPU isosurface extraction: PhysX smooths the particle positions and
// marching-cubes a triangle mesh, all on the GPU
// via a particle-system callback. The result is read to host arrays each solve
// for the node to render as an ArrayMesh.
struct GodotPhysXFluidIsosurface : public PxParticleSystemCallback {
	GodotPhysXParticleFluid3D *owner = nullptr;
	PxCudaContextManager *cuda = nullptr;

	PxSparseGridIsosurfaceExtractor *extractor = nullptr;
	PxSmoothedPositionGenerator *smoothing = nullptr;
	PxAnisotropyGenerator *anisotropy = nullptr; // used only when owner->surface_anisotropy_enabled

	PxVec4 *dev_smoothed = nullptr;
	PxVec4 *dev_aniso1 = nullptr;
	PxVec4 *dev_aniso2 = nullptr;
	PxVec4 *dev_aniso3 = nullptr;

	uint32_t max_vertices = 0;
	uint32_t max_triangles = 0;
	LocalVector<PxVec4> host_vertices;
	LocalVector<PxU32> host_indices;
	LocalVector<PxVec4> host_normals;
	LocalVector<PxVec4> host_positions; // scratch for outlier clamping
	float clamp_reach = 3.5f; // max meters a particle may sit from the fluid's median before it is pinned
	uint32_t frame = 0; // isosurface is re-extracted every other solve to halve the cost
	// Extractions are kicked async on the stream and their results read one
	// 30 Hz tick later, so the GPU is never stalled waiting on marching cubes.
	bool surface_pending = false;
	bool foam_pending = false;

	// Second extractor over the diffuse (foam) particles. Coarser grid: foam is
	// meant to read as froth, not a smooth skin, and the particle count is low.
	PxSparseGridIsosurfaceExtractor *foam_extractor = nullptr;
	PxVec4 *dev_foam = nullptr;
	static constexpr uint32_t FOAM_MAX_PARTICLES = 65536;
	uint32_t foam_max_vertices = 0;
	uint32_t foam_max_triangles = 0;
	LocalVector<PxVec4> foam_host_vertices;
	LocalVector<PxU32> foam_host_indices;
	LocalVector<PxVec4> foam_host_normals;
	LocalVector<PxVec4> foam_host_positions;

	void init(PxCudaContextManager *p_cuda, uint32_t p_max_particles, float p_spacing, float p_foam_spacing) {
		cuda = p_cuda;
		const float rest_offset = 0.5f * p_spacing;
		const float foam_rest = 0.5f * p_foam_spacing;

		PxPhysicsGpu *gpu = PxGetPhysicsGpu();
		ERR_FAIL_NULL(gpu);

		smoothing = gpu->createSmoothedPositionGenerator(cuda, p_max_particles, 0.5f);
		dev_smoothed = PX_EXT_DEVICE_MEMORY_ALLOC(PxVec4, *cuda, p_max_particles);
		smoothing->setResultBufferDevice(dev_smoothed);

		// min 1.0 keeps every ellipsoid at least a grid cell wide (smaller ones
		// flicker); max 1.6 is well below PhysX's default 2.0 to limit needling.
		anisotropy = gpu->createAnisotropyGenerator(cuda, p_max_particles, 5.0f, 1.0f, 1.6f);
		dev_aniso1 = PX_EXT_DEVICE_MEMORY_ALLOC(PxVec4, *cuda, p_max_particles);
		dev_aniso2 = PX_EXT_DEVICE_MEMORY_ALLOC(PxVec4, *cuda, p_max_particles);
		dev_aniso3 = PX_EXT_DEVICE_MEMORY_ALLOC(PxVec4, *cuda, p_max_particles);
		anisotropy->setResultBufferDevice(dev_aniso1, dev_aniso2, dev_aniso3);

		max_vertices = 512 * 1024;
		max_triangles = 1024 * 1024;
		host_vertices.resize(max_vertices);
		host_indices.resize(3 * max_triangles);
		host_normals.resize(max_vertices);
		host_positions.resize(p_max_particles);

		PxSparseGridParams sgp;
		sgp.subgridSizeX = 16;
		sgp.subgridSizeY = 16;
		sgp.subgridSizeZ = 16;
		sgp.haloSize = 0;
		sgp.maxNumSubgrids = 2048;
		// Grid cell ~= 1.5x the particle diameter. Finer than this multiplies the
		// marching-cubes cost (and can crater the frame rate for a fluid spread
		// over a wide area) with little visible gain once mesh smoothing runs.
		sgp.gridSpacing = 3.5f * rest_offset;

		PxIsosurfaceParams ip;
		ip.particleCenterToIsosurfaceDistance = 2.4f * rest_offset;
		ip.clearFilteringPasses();
		ip.numMeshSmoothingPasses = 6;
		ip.numMeshNormalSmoothingPasses = 4;

		extractor = gpu->createSparseGridIsosurfaceExtractor(cuda, sgp, ip, p_max_particles, max_vertices, max_triangles);
		if (extractor) {
			extractor->setResultBufferHost(host_vertices.ptr(), host_indices.ptr(), host_normals.ptr());
		}

		// Foam layer: a coarser grid over its own (small) position buffer.
		dev_foam = PX_EXT_DEVICE_MEMORY_ALLOC(PxVec4, *cuda, FOAM_MAX_PARTICLES);
		foam_max_vertices = 128 * 1024;
		foam_max_triangles = 256 * 1024;
		foam_host_vertices.resize(foam_max_vertices);
		foam_host_indices.resize(3 * foam_max_triangles);
		foam_host_normals.resize(foam_max_vertices);
		foam_host_positions.resize(FOAM_MAX_PARTICLES);

		PxSparseGridParams fsgp = sgp;
		fsgp.gridSpacing = 4.5f * foam_rest;
		PxIsosurfaceParams fip;
		// A wide reach so the sparse, scattered foam particles still blob together
		// into visible clumps of froth rather than isolated specks.
		fip.particleCenterToIsosurfaceDistance = 4.5f * foam_rest;
		fip.clearFilteringPasses();
		fip.numMeshSmoothingPasses = 4;
		fip.numMeshNormalSmoothingPasses = 4;
		foam_extractor = gpu->createSparseGridIsosurfaceExtractor(cuda, fsgp, fip, FOAM_MAX_PARTICLES, foam_max_vertices, foam_max_triangles);
		if (foam_extractor) {
			foam_extractor->setResultBufferHost(foam_host_vertices.ptr(), foam_host_indices.ptr(), foam_host_normals.ptr());
		}
	}

	void destroy() {
		if (extractor) {
			extractor->release();
			extractor = nullptr;
		}
		if (foam_extractor) {
			foam_extractor->release();
			foam_extractor = nullptr;
			PX_EXT_DEVICE_MEMORY_FREE(*cuda, dev_foam);
		}
		foam_host_vertices.reset();
		foam_host_indices.reset();
		foam_host_normals.reset();
		foam_host_positions.reset();
		if (smoothing) {
			smoothing->release();
			smoothing = nullptr;
			PX_EXT_DEVICE_MEMORY_FREE(*cuda, dev_smoothed);
		}
		if (anisotropy) {
			anisotropy->release();
			anisotropy = nullptr;
			PX_EXT_DEVICE_MEMORY_FREE(*cuda, dev_aniso1);
			PX_EXT_DEVICE_MEMORY_FREE(*cuda, dev_aniso2);
			PX_EXT_DEVICE_MEMORY_FREE(*cuda, dev_aniso3);
		}
		host_vertices.reset();
		host_indices.reset();
		host_normals.reset();
		host_positions.reset();
	}

	void onBegin(const PxGpuMirroredPointer<PxGpuParticleSystem> &, CUstream) override {}
	void onAdvance(const PxGpuMirroredPointer<PxGpuParticleSystem> &, CUstream) override {}

	void onPostSolve(const PxGpuMirroredPointer<PxGpuParticleSystem> &p_gps, CUstream p_stream) override {
		if (!extractor || !owner) {
			return;
		}
		// Re-extract on every other solve -- 30 Hz surface updates are plenty for
		// water and this halves the cost.
		if ((frame++ & 1u) != 0u) {
			return;
		}

		// Publish the extractions kicked last cycle. Two full solves have run on
		// the stream since, so the host result buffers and counts are settled --
		// no synchronize needed. The mesh is one 30 Hz tick behind, invisible for
		// a liquid, and the GPU never stalls waiting on marching cubes.
		if (surface_pending) {
			publish_surface();
			surface_pending = false;
		}
		if (foam_pending) {
			publish_foam();
			foam_pending = false;
		}

		PxGpuParticleSystem &gps = *p_gps.mHostPtr;
		const PxU32 n = gps.mCommonData.mNumParticles;
		if (n == 0) {
			MutexLock lock(owner->mesh_mutex);
			owner->mesh_vertices.clear();
			owner->mesh_normals.clear();
			owner->mesh_indices.clear();
			return;
		}

		const bool use_aniso = owner->surface_anisotropy_enabled;

		smoothing->generateSmoothedPositions(p_gps.mDevicePtr, gps.mCommonData.mMaxParticles, p_stream);
		if (use_aniso) {
			anisotropy->generateAnisotropy(p_gps.mDevicePtr, gps.mCommonData.mMaxParticles, p_stream);
		}
		// The one unavoidable sync: the outlier clamp below reads the smoothed
		// positions back to the host.
		cuda->getCudaContext()->streamSynchronize(p_stream);

		// Pin outliers: a particle that escapes the container drags the sparse
		// grid (and the mesh bounds) out to it -- the surface appears to stretch
		// to infinity. Clamp every particle to within clamp_reach meters of the
		// mean before feeding the extractor.
		Ext::PxCudaHelpersExt::copyDToH(*cuda, host_positions.ptr(), dev_smoothed, n);
		PxVec3 center(0.0f);
		for (uint32_t i = 0; i < n; i++) {
			center += host_positions[i].getXYZ();
		}
		center *= 1.0f / (float)n;
		bool clamped_any = false;
		for (uint32_t i = 0; i < n; i++) {
			PxVec3 p = host_positions[i].getXYZ();
			const PxVec3 d = p - center;
			if (d.x < -clamp_reach || d.x > clamp_reach || d.y < -clamp_reach || d.y > clamp_reach || d.z < -clamp_reach || d.z > clamp_reach) {
				p.x = center.x + PxClamp(d.x, -clamp_reach, clamp_reach);
				p.y = center.y + PxClamp(d.y, -clamp_reach, clamp_reach);
				p.z = center.z + PxClamp(d.z, -clamp_reach, clamp_reach);
				host_positions[i] = PxVec4(p, host_positions[i].w);
				clamped_any = true;
			}
		}
		if (clamped_any) {
			Ext::PxCudaHelpersExt::copyHToD(*cuda, dev_smoothed, host_positions.ptr(), n);
		}

		// Anisotropy is passed only when the owner opts in (settled pools). For
		// emitting fluid it is left off -- fast particles along the emission
		// column stretch into ellipsoids that marching cubes meshes as needles.
		extractor->extractIsosurface(dev_smoothed, n, p_stream, gps.mUnsortedPhaseArray,
				PxParticlePhaseFlag::eParticlePhaseFluid, nullptr,
				use_aniso ? dev_aniso1 : nullptr,
				use_aniso ? dev_aniso2 : nullptr,
				use_aniso ? dev_aniso3 : nullptr,
				use_aniso ? gps.mCommonData.mParticleContactDistance : 1.0f);
		surface_pending = true;

		kick_foam(center, p_stream);
	}

	// Read the completed surface extraction (host buffers filled, counts settled)
	// into the owner's mesh arrays.
	void publish_surface() {
		uint32_t nv = MIN(extractor->getNumVertices(), max_vertices);
		uint32_t nt = MIN(extractor->getNumTriangles(), max_triangles);
		for (uint32_t i = 0; i < nt * 3; i++) {
			if (host_indices[i] >= nv) {
				nt = 0;
				break;
			}
		}
		MutexLock lock(owner->mesh_mutex);
		owner->mesh_vertices.resize(nv);
		owner->mesh_normals.resize(nv);
		for (uint32_t i = 0; i < nv; i++) {
			const PxVec4 &v = host_vertices[i];
			const PxVec4 &nrm = host_normals[i];
			owner->mesh_vertices[i] = Vector3(v.x, v.y, v.z);
			owner->mesh_normals[i] = Vector3(nrm.x, nrm.y, nrm.z);
		}
		owner->mesh_indices.resize(nt * 3);
		for (uint32_t i = 0; i < nt * 3; i++) {
			owner->mesh_indices[i] = (int32_t)host_indices[i];
		}
		owner->mesh_version++;
	}

	// Kick the coarse diffuse-particle isosurface async on the stream.
	// p_fluid_center is the fluid median, used to pin foam spray that has flung
	// far from the body of water. Diffuse positions are final after the solve, so
	// the readback for the clamp needs no synchronize.
	void kick_foam(const PxVec3 &p_fluid_center, CUstream p_stream) {
		if (!foam_extractor) {
			return;
		}
		const PxU32 fn = (owner->foam_enabled && owner->px_buffer)
				? MIN(owner->px_buffer->getNbActiveDiffuseParticles(), FOAM_MAX_PARTICLES)
				: 0;
		PxVec4 *diffuse = (fn > 0) ? owner->px_buffer->getDiffusePositionLifeTime() : nullptr;
		if (!diffuse) {
			// No foam: clear the layer once (version stamp lets the node skip
			// while it stays empty).
			MutexLock lock(owner->foam_mesh_mutex);
			if (!owner->foam_mesh_indices.is_empty()) {
				owner->foam_mesh_vertices.clear();
				owner->foam_mesh_normals.clear();
				owner->foam_mesh_indices.clear();
				owner->foam_mesh_version++;
			}
			return;
		}
		Ext::PxCudaHelpersExt::copyDToH(*cuda, foam_host_positions.ptr(), diffuse, fn);
		const float reach = clamp_reach + 1.0f;
		for (uint32_t i = 0; i < fn; i++) {
			PxVec3 p = foam_host_positions[i].getXYZ();
			const PxVec3 d = p - p_fluid_center;
			p.x = p_fluid_center.x + PxClamp(d.x, -reach, reach);
			p.y = p_fluid_center.y + PxClamp(d.y, -reach, reach);
			p.z = p_fluid_center.z + PxClamp(d.z, -reach, reach);
			foam_host_positions[i] = PxVec4(p, 0.0f);
		}
		Ext::PxCudaHelpersExt::copyHToD(*cuda, dev_foam, foam_host_positions.ptr(), fn);

		foam_extractor->extractIsosurface(dev_foam, fn, p_stream, nullptr, 0, nullptr,
				nullptr, nullptr, nullptr, 1.0f);
		foam_pending = true;
	}

	void publish_foam() {
		uint32_t nv = MIN(foam_extractor->getNumVertices(), foam_max_vertices);
		uint32_t nt = MIN(foam_extractor->getNumTriangles(), foam_max_triangles);
		for (uint32_t i = 0; i < nt * 3; i++) {
			if (foam_host_indices[i] >= nv) {
				nt = 0;
				break;
			}
		}
		MutexLock lock(owner->foam_mesh_mutex);
		owner->foam_mesh_vertices.resize(nv);
		owner->foam_mesh_normals.resize(nv);
		for (uint32_t i = 0; i < nv; i++) {
			const PxVec4 &v = foam_host_vertices[i];
			const PxVec4 &nrm = foam_host_normals[i];
			owner->foam_mesh_vertices[i] = Vector3(v.x, v.y, v.z);
			owner->foam_mesh_normals[i] = Vector3(nrm.x, nrm.y, nrm.z);
		}
		owner->foam_mesh_indices.resize(nt * 3);
		for (uint32_t i = 0; i < nt * 3; i++) {
			owner->foam_mesh_indices[i] = (int32_t)foam_host_indices[i];
		}
		owner->foam_mesh_version++;
	}
};

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

void GodotPhysXParticleFluid3D::_destroy_isosurface() {
	if (!iso) {
		return;
	}
	if (px_system) {
		px_system->setParticleSystemCallback(nullptr);
	}
	iso->destroy();
	memdelete(iso);
	iso = nullptr;
	{
		MutexLock lock(mesh_mutex);
		mesh_vertices.clear();
		mesh_normals.clear();
		mesh_indices.clear();
	}
	{
		MutexLock lock(foam_mesh_mutex);
		foam_mesh_vertices.clear();
		foam_mesh_normals.clear();
		foam_mesh_indices.clear();
	}
}

void GodotPhysXParticleFluid3D::_ensure_isosurface() {
	if (iso || !surface_mesh_enabled || !px_system || !space) {
		return;
	}
	PxCudaContextManager *cuda = space->get_px_cuda();
	if (!cuda) {
		return;
	}
	iso = memnew(GodotPhysXFluidIsosurface);
	iso->owner = this;
	iso->init(cuda, capacity, MAX((float)particle_size, 0.001f), MAX((float)foam_size, 0.001f));
	if (!iso->extractor) {
		_destroy_isosurface();
		return;
	}
	px_system->setParticleSystemCallback(iso);
}

void GodotPhysXParticleFluid3D::set_surface_mesh_enabled(bool p_enabled) {
	if (surface_mesh_enabled == p_enabled) {
		return;
	}
	surface_mesh_enabled = p_enabled;
	if (p_enabled) {
		_ensure_isosurface();
	} else {
		_destroy_isosurface();
	}
}

uint32_t GodotPhysXParticleFluid3D::copy_surface_mesh(LocalVector<Vector3> &r_vertices, LocalVector<Vector3> &r_normals, LocalVector<int32_t> &r_indices, uint32_t &p_have_version) const {
	MutexLock lock(mesh_mutex);
	if (p_have_version == mesh_version) {
		return UINT32_MAX; // unchanged since the caller last asked -- skip the rebuild
	}
	p_have_version = mesh_version;
	r_vertices = mesh_vertices;
	r_normals = mesh_normals;
	r_indices = mesh_indices;
	return mesh_indices.size() / 3;
}

uint32_t GodotPhysXParticleFluid3D::copy_foam_mesh(LocalVector<Vector3> &r_vertices, LocalVector<Vector3> &r_normals, LocalVector<int32_t> &r_indices, uint32_t &p_have_version) const {
	MutexLock lock(foam_mesh_mutex);
	if (p_have_version == foam_mesh_version) {
		return UINT32_MAX;
	}
	p_have_version = foam_mesh_version;
	r_vertices = foam_mesh_vertices;
	r_normals = foam_mesh_normals;
	r_indices = foam_mesh_indices;
	return foam_mesh_indices.size() / 3;
}

void GodotPhysXParticleFluid3D::_destroy() {
	_destroy_isosurface();
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

void GodotPhysXParticleFluid3D::set_foam_size(real_t p_v) {
	p_v = MAX(p_v, (real_t)0.001);
	if (p_v == foam_size) {
		return;
	}
	foam_size = p_v;
	// The foam grid spacing is baked into the extractor -- rebuild it.
	if (iso) {
		_destroy_isosurface();
		_ensure_isosurface();
	}
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

	_ensure_isosurface();
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

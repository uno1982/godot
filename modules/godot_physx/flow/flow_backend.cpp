/**************************************************************************/
/*  flow_backend.cpp                                                      */
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

#include "flow_backend.h"

#include "NvFlowLoader.h"
#include "flow_context_rd.h"

#define PNANOVDB_C
#include "nvflow/nanovdb/PNanoVDB.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

namespace FlowBackend {

namespace {

// Flow's libraries, shared by every grid.
struct Library {
	NvFlowLoader loader = {};
	int refs = 0;
};
Library *library = nullptr;
char load_error[512] = {};
// Flow's grids and contexts aren't thread-safe.
std::mutex device_mutex;

void on_load_error(const char *p_str, void *) {
	snprintf(load_error, sizeof(load_error), "%s", p_str);
}

// Godot's column-vector transform (scaled to Flow units) as Flow's
// row-vector matrix: rows 0-2 are the basis columns, row 3 the origin.
NvFlowFloat4x4 to_flow_matrix(const float p_basis[3][3], const float p_origin[3], bool p_with_origin) {
	const float s = UNITS_PER_METER;
	NvFlowFloat4x4 m;
	m.x = { p_basis[0][0] * s, p_basis[0][1] * s, p_basis[0][2] * s, 0.0f };
	m.y = { p_basis[1][0] * s, p_basis[1][1] * s, p_basis[1][2] * s, 0.0f };
	m.z = { p_basis[2][0] * s, p_basis[2][1] * s, p_basis[2][2] * s, 0.0f };
	if (p_with_origin) {
		m.w = { p_origin[0] * s, p_origin[1] * s, p_origin[2] * s, 1.0f };
	} else {
		m.w = { 0.0f, 0.0f, 0.0f, 1.0f };
	}
	return m;
}

template <typename T>
void set_channels(T &r_params, const Emitter &p_emitter) {
	const float s = UNITS_PER_METER;
	r_params.localToWorld = to_flow_matrix(p_emitter.basis, p_emitter.origin, true);
	if (p_emitter.collision) {
		// A solid: the gas inside moves with the body and holds nothing.
		r_params.velocityIsWorldSpace = NV_FLOW_TRUE;
		r_params.velocity = { p_emitter.body_velocity[0] * s, p_emitter.body_velocity[1] * s, p_emitter.body_velocity[2] * s };
		r_params.temperature = 0.0f;
		r_params.fuel = 0.0f;
		r_params.burn = 0.0f;
		r_params.smoke = 0.0f;
		r_params.divergence = 0.0f;
		r_params.coupleRateVelocity = 100.0f;
		r_params.coupleRateDivergence = 100.0f;
		r_params.coupleRateTemperature = 100.0f;
		r_params.coupleRateFuel = 100.0f;
		r_params.coupleRateBurn = 100.0f;
		r_params.coupleRateSmoke = 100.0f;
		r_params.applyPostPressure = NV_FLOW_TRUE;
		// Acts only where gas already is: a floor or wall allocates nothing.
		r_params.allocationScale = 0.0f;
		return;
	}
	// Velocity in the emitter's own frame (a nozzle), rotated -- not scaled
	// twice -- by the rotation part alone.
	r_params.velocityIsWorldSpace = NV_FLOW_FALSE;
	const float no_origin[3] = { 0, 0, 0 };
	r_params.localToWorldVelocity = to_flow_matrix(p_emitter.basis, no_origin, false);
	r_params.velocity = { p_emitter.velocity[0], p_emitter.velocity[1], p_emitter.velocity[2] };
	r_params.temperature = p_emitter.temperature;
	r_params.fuel = p_emitter.fuel;
	r_params.burn = p_emitter.burn;
	r_params.smoke = p_emitter.smoke;
	r_params.divergence = p_emitter.divergence;
	const float c = p_emitter.couple_rate;
	r_params.coupleRateVelocity = c;
	r_params.coupleRateTemperature = c;
	r_params.coupleRateFuel = p_emitter.fuel > 0.0f ? c : 0.0f;
	r_params.coupleRateSmoke = p_emitter.smoke > 0.0f ? c : 0.0f;
	r_params.coupleRateBurn = p_emitter.burn > 0.0f ? c : 0.0f;
	r_params.coupleRateDivergence = p_emitter.divergence != 0.0f ? c : 0.0f;
}

} // namespace

struct Grid {
	// The libraries, and the RenderingDevice context the grid runs on.
	NvFlowLoader *loader = nullptr;
	NvFlowContextInterface *context_interface = nullptr;
	NvFlowContext *context = nullptr;
	FlowContextRD::Context *rd_context = nullptr;
	NvFlowGrid *grid = nullptr;
	NvFlowGridParamsNamed *params_named = nullptr;
	NvFlowGridParams *params = nullptr;
	NvFlowUint64 version = 1;
	// Kept alive across the commit -> simulate of one step.
	NvFlowGridSimulateLayerParams simulate = NvFlowGridSimulateLayerParams_default;
	NvFlowGridOffscreenLayerParams offscreen = NvFlowGridOffscreenLayerParams_default;
	NvFlowGridRenderLayerParams render = NvFlowGridRenderLayerParams_default;
	std::vector<NvFlowEmitterSphereParams> spheres;
	std::vector<NvFlowEmitterBoxParams> boxes;
	std::vector<NvFlowUint8 *> sphere_ptrs;
	std::vector<NvFlowUint8 *> box_ptrs;
	NvFlowGridRenderData render_data = {};
	const NvFlowGridRenderDataNanoVdbReadback *readback = nullptr;
	double last_time = 0.0; // params snapshot the renderer maps
	// Flow keeps pointers into the committed snapshot (the renderer maps it
	// again later), so everything it points at lives here, not on the stack.
	NvFlowUint8 *sim_ptr = nullptr;
	NvFlowUint8 *offscreen_ptr = nullptr;
	NvFlowUint8 *render_ptr = nullptr;
	NvFlowDatabaseTypeSnapshot types[5] = {};
	NvFlowGridParamsDescSnapshot snapshot = {};
	// The colormap the offscreen params point at.
	float colormap_x[Settings::COLORMAP_POINTS] = {};
	NvFlowFloat4 colormap_rgba[Settings::COLORMAP_POINTS] = {};
	float colormap_scale[Settings::COLORMAP_POINTS] = {};
};

void grid_destroy(Grid *p_grid) {
	std::lock_guard<std::mutex> lock(device_mutex);
	if (p_grid == nullptr) {
		return;
	}
	p_grid->loader->gridInterface.destroyGrid(p_grid->context, p_grid->grid);
	p_grid->loader->gridParamsInterface.destroyGridParamsNamed(p_grid->params_named);
	FlowContextRD::destroy(p_grid->rd_context);
	delete p_grid;
}

bool acquire_library(char *r_error, int p_error_size) {
	std::lock_guard<std::mutex> lock(device_mutex);
	if (library != nullptr) {
		library->refs++;
		return true;
	}
	Library *l = new Library();
	load_error[0] = '\0';
	NvFlowLoaderInit(&l->loader, on_load_error, nullptr);
	if (l->loader.module_nvflow == nullptr || l->loader.module_nvflowext == nullptr) {
#ifdef _WIN32
		snprintf(r_error, p_error_size, "nvflow.dll / nvflowext.dll not found next to the executable (%s)", load_error);
#else
		snprintf(r_error, p_error_size, "libnvflow.so / libnvflowext.so not found next to the executable (%s)", load_error);
#endif
		NvFlowLoaderDestroy(&l->loader);
		delete l;
		return false;
	}
	l->refs = 1;
	library = l;
	return true;
}

void release_library() {
	std::lock_guard<std::mutex> lock(device_mutex);
	if (library == nullptr || --library->refs > 0) {
		return;
	}
	NvFlowLoaderDestroy(&library->loader);
	delete library;
	library = nullptr;
}

Grid *grid_create_rd(RenderingDevice *p_rd, uint32_t p_max_blocks) {
	std::lock_guard<std::mutex> lock(device_mutex);
	if (library == nullptr || p_rd == nullptr) {
		return nullptr;
	}
	Grid *g = new Grid();
	g->loader = &library->loader;
	g->rd_context = FlowContextRD::create(p_rd);
	g->context_interface = FlowContextRD::get_interface();
	g->context = FlowContextRD::get_flow_context(g->rd_context);
	NvFlowGridDesc desc = NvFlowGridDesc_default;
	desc.maxLocations = p_max_blocks;
	desc.maxLocationsIsosurface = p_max_blocks;
	g->grid = library->loader.gridInterface.createGrid(g->context_interface, g->context, library->loader.opList_orig, library->loader.extOpList_orig, &desc);
	char name[64];
	snprintf(name, sizeof(name), "godot_physx_flow_%p", (void *)g);
	g->params_named = library->loader.gridParamsInterface.createGridParamsNamed(name);
	g->params = library->loader.gridParamsInterface.mapGridParamsNamed(g->params_named);
	return g;
}

uint64_t grid_render_rd(Grid *p_grid, const float p_view[16], const float p_projection[16], uint32_t p_width, uint32_t p_height, uint64_t p_depth_rid, uint32_t p_depth_width, uint32_t p_depth_height, uint64_t p_color_rid) {
	std::unique_lock<std::mutex> lock(device_mutex, std::try_to_lock);
	if (!lock.owns_lock() || p_grid == nullptr || p_grid->rd_context == nullptr || p_width == 0 || p_height == 0) {
		return 0;
	}
	NvFlowFloat4x4 view;
	NvFlowFloat4x4 projection;
	memcpy(&view, p_view, sizeof(view));
	memcpy(&projection, p_projection, sizeof(projection));

	NvFlowGridParamsInterface &params_if = p_grid->loader->gridParamsInterface;
	NvFlowGridParamsSnapshot *params_snapshot = params_if.getParamsSnapshot(p_grid->params, p_grid->last_time, 0llu);
	NvFlowGridParamsDesc desc = {};
	if (!params_if.mapParamsDesc(p_grid->params, params_snapshot, &desc)) {
		return 0;
	}
	NvFlowTextureTransient *depth = (NvFlowTextureTransient *)FlowContextRD::import_texture(p_grid->rd_context, p_depth_rid, p_depth_width, p_depth_height, eNvFlowFormat_r32_float);
	NvFlowTextureTransient *color = (NvFlowTextureTransient *)FlowContextRD::import_texture(p_grid->rd_context, p_color_rid, p_width, p_height, eNvFlowFormat_r16g16b16a16_float);
	NvFlowTextureTransient *out = nullptr;
	p_grid->loader->gridInterface.offscreen(p_grid->context, p_grid->grid, &desc);
	p_grid->loader->gridInterface.render(p_grid->context, p_grid->grid, &desc, &view, &projection, &projection, p_width, p_height, p_depth_width, p_depth_height, 1.0f, depth, eNvFlowFormat_r16g16b16a16_float, color, &out);
	params_if.unmapParamsDesc(p_grid->params, params_snapshot);
	return out != nullptr ? FlowContextRD::transient_texture_rid(out) : 0;
}

void grid_render_rd_finish(Grid *p_grid) {
	std::lock_guard<std::mutex> lock(device_mutex);
	if (p_grid != nullptr && p_grid->rd_context != nullptr) {
		FlowContextRD::flush(p_grid->rd_context);
	}
}

uint32_t grid_active_blocks(Grid *p_grid) {
	std::lock_guard<std::mutex> lock(device_mutex);
	if (p_grid == nullptr || p_grid->grid == nullptr) {
		return 0;
	}
	return p_grid->loader->gridInterface.getActiveBlockCount(p_grid->grid);
}

bool grid_rd_stats(Grid *p_grid, int &r_pipelines, int &r_pipeline_failures, int &r_passes, int &r_buffers, int &r_textures, uint64_t &r_frame, uint64_t &r_completed) {
	if (p_grid == nullptr) {
		return false;
	}
	const FlowContextRD::Stats st = FlowContextRD::get_stats(p_grid->rd_context);
	r_pipelines = st.pipelines;
	r_pipeline_failures = st.pipeline_failures;
	r_passes = st.passes;
	r_buffers = st.buffers;
	r_textures = st.textures;
	r_frame = st.current_frame;
	r_completed = st.last_completed_frame;
	return true;
}

bool grid_step(Grid *p_grid, double p_time, float p_delta, const Settings &p_settings, const Emitter *p_emitters, int p_emitter_count) {
	// Busy (another grid still being created): skip this step rather than
	// stall the frame.
	std::unique_lock<std::mutex> lock(device_mutex, std::try_to_lock);
	if (!lock.owns_lock()) {
		return false;
	}
	if (p_grid == nullptr || p_grid->loader == nullptr) {
		return false;
	}
	const float s = UNITS_PER_METER;

	NvFlowGridSimulateLayerParams &sim = p_grid->simulate;
	sim = NvFlowGridSimulateLayerParams_default;
	sim.densityCellSize = p_settings.cell_size * s;
	sim.physicsCollisionEnabled = NV_FLOW_TRUE;
	sim.advection.combustionEnabled = p_settings.combustion ? NV_FLOW_TRUE : NV_FLOW_FALSE;
	sim.advection.gravity = { p_settings.gravity[0] * s, p_settings.gravity[1] * s, p_settings.gravity[2] * s };
	sim.advection.buoyancyPerTemp = p_settings.buoyancy;
	sim.advection.coolingRate = p_settings.cooling_rate;
	sim.advection.smoke.fade = p_settings.smoke_fade;
	sim.advection.ignitionTemp = p_settings.ignition_temperature;
	sim.vorticity.forceScale = p_settings.vorticity;
	sim.nanoVdbExport.enabled = NV_FLOW_TRUE;
	sim.nanoVdbExport.readbackEnabled = p_settings.readback ? NV_FLOW_TRUE : NV_FLOW_FALSE;
	sim.nanoVdbExport.temperatureEnabled = NV_FLOW_TRUE;
	sim.nanoVdbExport.smokeEnabled = NV_FLOW_TRUE;

	p_grid->render = NvFlowGridRenderLayerParams_default;
	p_grid->render.rayMarch.attenuation = p_settings.attenuation / s;
	p_grid->render.rayMarch.colormapXMin = 0.0f;
	p_grid->render.rayMarch.colormapXMax = p_settings.temperature_range;

	p_grid->offscreen = NvFlowGridOffscreenLayerParams_default;
	NvFlowShadowParams &shadow = p_grid->offscreen.shadow;
	shadow.lightDirection = { p_settings.light_direction[0], p_settings.light_direction[1], p_settings.light_direction[2] };
	shadow.attenuation = 0.9f * p_settings.attenuation / s;
	shadow.minIntensity = p_settings.shadow_min_intensity;
	shadow.numSteps = (NvFlowUint)(p_settings.shadow_steps > 0 ? p_settings.shadow_steps : 1);
	shadow.enabled = p_settings.shadow_steps > 0 ? NV_FLOW_TRUE : NV_FLOW_FALSE;
	shadow.colormapXMin = 0.0f;
	shadow.colormapXMax = p_settings.temperature_range;
	for (int i = 0; i < Settings::COLORMAP_POINTS; i++) {
		p_grid->colormap_x[i] = (float)i / (float)(Settings::COLORMAP_POINTS - 1);
		p_grid->colormap_rgba[i] = { p_settings.colormap_rgb[i][0], p_settings.colormap_rgb[i][1], p_settings.colormap_rgb[i][2], 1.0f };
		p_grid->colormap_scale[i] = 1.0f;
	}
	NvFlowRayMarchColormapParams &colormap = p_grid->offscreen.colormap;
	colormap.xPoints = p_grid->colormap_x;
	colormap.xPointCount = Settings::COLORMAP_POINTS;
	colormap.rgbaPoints = p_grid->colormap_rgba;
	colormap.rgbaPointCount = Settings::COLORMAP_POINTS;
	colormap.colorScalePoints = p_grid->colormap_scale;
	colormap.colorScalePointCount = Settings::COLORMAP_POINTS;
	colormap.colorScale = 1.0f;

	p_grid->spheres.clear();
	p_grid->boxes.clear();
	for (int i = 0; i < p_emitter_count; i++) {
		const Emitter &e = p_emitters[i];
		if (e.shape == Emitter::SHAPE_BOX) {
			NvFlowEmitterBoxParams b = NvFlowEmitterBoxParams_default;
			b.position = { 0.0f, 0.0f, 0.0f };
			b.halfSize = { e.half_size[0], e.half_size[1], e.half_size[2] };
			set_channels(b, e);
			b.isPhysicsCollision = e.collision ? NV_FLOW_TRUE : NV_FLOW_FALSE;
			p_grid->boxes.push_back(b);
		} else {
			NvFlowEmitterSphereParams sp = NvFlowEmitterSphereParams_default;
			sp.position = { 0.0f, 0.0f, 0.0f };
			sp.radius = e.radius;
			sp.radiusIsWorldSpace = NV_FLOW_FALSE;
			set_channels(sp, e);
			p_grid->spheres.push_back(sp);
		}
	}
	p_grid->sphere_ptrs.resize(p_grid->spheres.size());
	for (size_t i = 0; i < p_grid->spheres.size(); i++) {
		p_grid->sphere_ptrs[i] = (NvFlowUint8 *)&p_grid->spheres[i];
	}
	p_grid->box_ptrs.resize(p_grid->boxes.size());
	for (size_t i = 0; i < p_grid->boxes.size(); i++) {
		p_grid->box_ptrs[i] = (NvFlowUint8 *)&p_grid->boxes[i];
	}

	p_grid->sim_ptr = (NvFlowUint8 *)&p_grid->simulate;
	p_grid->offscreen_ptr = (NvFlowUint8 *)&p_grid->offscreen;
	p_grid->render_ptr = (NvFlowUint8 *)&p_grid->render;
	const NvFlowUint64 v = ++p_grid->version;
	p_grid->types[0] = { v, &NvFlowGridSimulateLayerParams_NvFlowReflectDataType, &p_grid->sim_ptr, 1u };
	p_grid->types[1] = { v, &NvFlowGridEmitterSphereParams_NvFlowReflectDataType, p_grid->sphere_ptrs.data(), p_grid->sphere_ptrs.size() };
	p_grid->types[2] = { v, &NvFlowGridEmitterBoxParams_NvFlowReflectDataType, p_grid->box_ptrs.data(), p_grid->box_ptrs.size() };
	p_grid->types[3] = { v, &NvFlowGridOffscreenLayerParams_NvFlowReflectDataType, &p_grid->offscreen_ptr, 1u };
	p_grid->types[4] = { v, &NvFlowGridRenderLayerParams_NvFlowReflectDataType, &p_grid->render_ptr, 1u };
	p_grid->snapshot = { { v, p_grid->types, 5u }, p_time, p_delta, NV_FLOW_FALSE, nullptr, 0u };
	NvFlowGridParamsInterface &params_if = p_grid->loader->gridParamsInterface;
	params_if.commitParams(p_grid->params, &p_grid->snapshot);

	NvFlowGridParamsSnapshot *params_snapshot = params_if.getParamsSnapshot(p_grid->params, p_time, 0llu);
	p_grid->last_time = p_time;
	NvFlowGridParamsDesc desc = {};
	p_grid->render_data = {};
	if (params_if.mapParamsDesc(p_grid->params, params_snapshot, &desc)) {
		p_grid->loader->gridInterface.simulate(p_grid->context, p_grid->grid, &desc, NV_FLOW_FALSE);
		p_grid->loader->gridInterface.getRenderData(p_grid->context, p_grid->grid, &p_grid->render_data);
		params_if.unmapParamsDesc(p_grid->params, params_snapshot);
	}

	FlowContextRD::flush(p_grid->rd_context);

	// Newest readback the GPU has finished writing (no CPU wait: it lags a
	// few frames behind). Readbacks are stamped with the GLOBAL frame -- the
	// context's own frame counter is a different one, and comparing against
	// it picks buffers the GPU is still copying into.
	const NvFlowUint64 completed = p_grid->context_interface->getLastGlobalFrameCompleted(p_grid->context);
	p_grid->readback = nullptr;
	const NvFlowGridRenderDataNanoVdb &nano = p_grid->render_data.nanoVdb;
	for (NvFlowUint64 i = nano.readbackCount; i > 0; i--) {
		const NvFlowGridRenderDataNanoVdbReadback *rb = nano.readbacks + (i - 1);
		if (completed >= rb->globalFrameCompleted) {
			p_grid->readback = rb;
			break;
		}
	}
	return true;
}

bool grid_get_readback(Grid *p_grid, const uint8_t **r_temperature, uint64_t *r_temperature_size, const uint8_t **r_smoke, uint64_t *r_smoke_size) {
	if (p_grid == nullptr || p_grid->readback == nullptr) {
		return false;
	}
	*r_temperature = p_grid->readback->temperatureNanoVdbReadback;
	*r_temperature_size = p_grid->readback->temperatureNanoVdbReadbackSize;
	*r_smoke = p_grid->readback->smokeNanoVdbReadback;
	*r_smoke_size = p_grid->readback->smokeNanoVdbReadbackSize;
	return *r_temperature != nullptr || *r_smoke != nullptr;
}

bool nanovdb_bounds(const uint8_t *p_nanovdb, uint64_t p_size, float r_min[3], float r_max[3]) {
	if (p_nanovdb == nullptr || p_size < PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE) {
		return false;
	}
	pnanovdb_buf_t buf = pnanovdb_make_buf((uint32_t *)p_nanovdb, p_size / 4u);
	pnanovdb_grid_handle_t grid = { pnanovdb_address_null() };
	pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, pnanovdb_grid_get_tree(buf, grid));
	const pnanovdb_coord_t lo = pnanovdb_root_get_bbox_min(buf, root);
	const pnanovdb_coord_t hi = pnanovdb_root_get_bbox_max(buf, root);
	if (lo.x > hi.x || lo.y > hi.y || lo.z > hi.z) {
		return false; // empty
	}
	pnanovdb_vec3_t a = { (float)lo.x - 0.5f, (float)lo.y - 0.5f, (float)lo.z - 0.5f };
	pnanovdb_vec3_t b = { (float)hi.x + 0.5f, (float)hi.y + 0.5f, (float)hi.z + 0.5f };
	const pnanovdb_vec3_t wa = pnanovdb_grid_index_to_worldf(buf, grid, &a);
	const pnanovdb_vec3_t wb = pnanovdb_grid_index_to_worldf(buf, grid, &b);
	const float inv = 1.0f / UNITS_PER_METER;
	r_min[0] = fminf(wa.x, wb.x) * inv;
	r_min[1] = fminf(wa.y, wb.y) * inv;
	r_min[2] = fminf(wa.z, wb.z) * inv;
	r_max[0] = fmaxf(wa.x, wb.x) * inv;
	r_max[1] = fmaxf(wa.y, wb.y) * inv;
	r_max[2] = fmaxf(wa.z, wb.z) * inv;
	return true;
}

void sample_dense(const uint8_t *p_nanovdb, uint64_t p_size, const float p_origin[3], float p_cell, const int p_dims[3], int p_z_begin, int p_z_end, float *r_out, int p_stride) {
	const int64_t slice = (int64_t)p_dims[0] * p_dims[1];
	if (p_nanovdb == nullptr || p_size < PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE) {
		for (int64_t i = p_z_begin * slice; i < p_z_end * slice; i++) {
			r_out[i * p_stride] = 0.0f;
		}
		return;
	}
	pnanovdb_buf_t buf = pnanovdb_make_buf((uint32_t *)p_nanovdb, p_size / 4u);
	pnanovdb_grid_handle_t grid = { pnanovdb_address_null() };
	const pnanovdb_grid_type_t type = pnanovdb_grid_get_grid_type(buf, grid);
	pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, pnanovdb_grid_get_tree(buf, grid));
	// World meters -> index space is a scale + offset (Flow grids carry no
	// rotation): map the first cell and one step per axis, then walk.
	const float s = UNITS_PER_METER;
	pnanovdb_vec3_t w0 = { (p_origin[0] + 0.5f * p_cell) * s, (p_origin[1] + 0.5f * p_cell) * s, (p_origin[2] + 0.5f * p_cell) * s };
	pnanovdb_vec3_t w1 = { w0.x + p_cell * s, w0.y + p_cell * s, w0.z + p_cell * s };
	const pnanovdb_vec3_t i0 = pnanovdb_grid_world_to_indexf(buf, grid, &w0);
	const pnanovdb_vec3_t i1 = pnanovdb_grid_world_to_indexf(buf, grid, &w1);
	const float step[3] = { i1.x - i0.x, i1.y - i0.y, i1.z - i0.z };
	// Voxel i spans [i, i + 1) in index space: the one holding a point is
	// its floor. Lookups go through the root, uncached: PNanoVDB's read
	// accessor returned stale (empty) results for negative coordinates under
	// some read orders.
	for (int z = p_z_begin; z < p_z_end; z++) {
		const int iz = (int)floorf(i0.z + step[2] * z);
		for (int y = 0; y < p_dims[1]; y++) {
			const int iy = (int)floorf(i0.y + step[1] * y);
			float *row = r_out + ((int64_t)z * slice + (int64_t)y * p_dims[0]) * p_stride;
			for (int x = 0; x < p_dims[0]; x++) {
				pnanovdb_coord_t ijk = { (int)floorf(i0.x + step[0] * x), iy, iz };
				float v = 0.0f;
				if (type == PNANOVDB_GRID_TYPE_FLOAT) {
					v = pnanovdb_read_float(buf, pnanovdb_root_get_value_address(type, buf, root, &ijk));
				}
				row[x * p_stride] = v;
			}
		}
	}
}

} // namespace FlowBackend

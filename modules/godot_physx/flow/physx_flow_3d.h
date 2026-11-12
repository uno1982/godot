/**************************************************************************/
/*  physx_flow_3d.h                                                       */
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

#include "flow_scatter_rd.h"

#include "core/os/mutex.h"
#include "core/templates/local_vector.h"
#include "core/templates/paged_array.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/gradient.h"

class RenderingDevice;
class PhysXFlowRenderEffect;

namespace FlowBackend {
struct Grid;
struct Emitter;
struct Settings;
} //namespace FlowBackend

// NVIDIA Flow smoke / fire / dust: a sparse, unbounded grid (it grows
// wherever the gas goes) fed by the PhysXFlowEmitter3D nodes that point at
// it, with nearby physics bodies as solids.
//
// Flow runs on Godot's own RenderingDevice, and PhysXFlowRenderEffect draws
// it into the frame with Flow's ray marcher (self-shadowed by the scene's
// first DirectionalLight3D) -- nothing goes through the CPU unless
// cpu_readback asks for it.
class PhysXFlow3D : public Node3D {
	GDCLASS(PhysXFlow3D, Node3D);

public:
	static constexpr const char *GROUP = "physx_flows";

private:
	RenderingDevice *rd = nullptr;
	bool library_acquired = false;
	FlowBackend::Grid *grid = nullptr;
	String device_error;
	double sim_time = 0.0;

	// Simulation.
	float cell_size = 0.05f;
	// Sparse blocks the gas may take up. Flow sizes its GPU textures for all
	// of them when the grid is made (~0.45 MB a block), so this is a memory
	// budget: Flow's own default of 4096 is ~1.9 GB per flow.
	int max_blocks = 1024;
	bool max_blocks_warned = false;
	float buoyancy = 2.0f;
	float cooling_rate = 1.5f;
	float smoke_fade = 0.65f;
	float vorticity = 0.6f;
	float ignition_temperature = 0.05f;
	bool combustion = true;

	// Collision: physics bodies within collision_range of this flow's
	// emitters become solids for the gas.
	bool collide_with_bodies = true;
	uint32_t collision_mask = 1;
	float collision_range = 10.0f;
	// A FLOOR: with floor_enabled, gas never goes below floor_height — a wide solid under it (no blocks reserved there).
	// Colliders can't describe terrain (heightmaps / trimeshes), so a blast's or a fire's gas sank into the ground and
	// spent the grid's blocks there.
	bool floor_enabled = false;
	float floor_height = 0.0f;
	RID query_shape;

	// Look.
	float smoke_density = 4.0f; // per meter, per unit of smoke
	Color smoke_color = Color(0.8, 0.8, 0.8);
	// Smoke color by temperature (0..temperature_range); empty: smoke_color.
	Ref<Gradient> smoke_ramp;
	// Empty: a built-in fire ramp (_default_heat_at).
	Ref<Gradient> heat_ramp;
	static Color _default_heat_at(float p_t);
	float temperature_range = 2.0f;
	float emission_strength = 4.0f;
	// Self-shadowing reach, in steps of 3/4 of a cell: 48 covers ~1.8 m at
	// the default 5 cm cells -- fewer leaves wide gas lit almost evenly.
	int shadow_steps = 48;

	// Optional copy of the gas on the CPU, for sample_smoke(),
	// sample_temperature() and stats:
	// every READBACK_INTERVAL steps (a full field each time).
	bool cpu_readback = false;
	static constexpr int READBACK_INTERVAL = 4;
	uint64_t step_count = 0;
	// Flow's GPU work -- making the grid, each step (simulate + flush +
	// readbacks), destroying it -- runs on the RENDER thread
	// (RenderingServer::call_on_render_thread: in single-threaded rendering
	// that is this thread, at once). The main thread gathers emitters and
	// queues each step's inputs; the render thread steps in order and leaves
	// the newest readback here, copied. step_mutex guards the queue and all
	// of these.
	struct StepQueue;
	StepQueue *steps = nullptr;
	Mutex step_mutex;
	Vector<uint8_t> last_smoke; // copies of the newest readback (NanoVDB)
	Vector<uint8_t> last_temperature;
	const uint8_t *last_smoke_source = nullptr; // the mapped readback they were copied from
	AABB gas_bounds;
	bool blocks_full = false; // seen on the render thread, warned about on this one

	// Stats.
	double stat_start_ms = 0.0;
	double stat_step_ms = 0.0;
	uint64_t stat_render_calls = 0;
	uint64_t stat_render_outputs = 0;
	int64_t stat_readback_bytes = 0;
	// Light scattering: lights picked last step, passes run, frames Flow had no volume to hand over.
	int stat_scatter_lights = 0;
	uint64_t stat_scatter_passes = 0;
	uint64_t stat_scatter_no_source = 0;

	// Where the gas is, roughly (its emitters' middle), for drawing flows
	// back to front. Written by _step, read on the render thread.
	Vector3 draw_center;

	// The first visible DirectionalLight3D, looked up every so often.
	ObjectID light_id;
	int light_search_countdown = 0;

	// Light scattering: point / spot (and directional) lights glowing in the gas (a pass of its
	// own after Flow's, see FlowScatterRD). Off by default: it marches every
	// pixel the gas covers once more, with a short march toward each light.
	bool light_scattering = false;
	int scattering_max_lights = 4;
	float scattering_strength = 1.0f;
	float scattering_anisotropy = 0.4f;
	// The scene's directional lights scatter too (off: only omni and spot lights; Flow's own sun lighting either way).
	bool scattering_directional = false;
	int scattering_shadow_steps = 4;
	float scattering_shadow_reach = 4.0f;

public:
	// Which lights' shadows the glow receives (a checkbox list). Directional
	// will be a third flag (4) when the sun's shadow reaches the gas.
	enum ScatteringShadowFlags {
		SCATTERING_SHADOW_OMNI = 1,
		SCATTERING_SHADOW_SPOT = 2,
	};

	enum ScatteringResolution {
		SCATTERING_RESOLUTION_FULL,
		SCATTERING_RESOLUTION_HALF,
		SCATTERING_RESOLUTION_QUARTER,
	};

private:
	ScatteringResolution scattering_resolution = SCATTERING_RESOLUTION_QUARTER;
	uint32_t scattering_receive_shadows = SCATTERING_SHADOW_OMNI | SCATTERING_SHADOW_SPOT;
	// Every OmniLight3D / SpotLight3D in the scene, looked up every so often;
	// the ones lighting the gas this step, for the render thread.
	LocalVector<ObjectID> scatter_light_ids;
	int scatter_search_countdown = 0;
	Mutex scatter_mutex;
	LocalVector<FlowScatterRD::Light> scatter_lights;
	void _gather_scatter_lights();
	void _fill_scatter_shadows(LocalVector<FlowScatterRD::Light> &r_lights, const PagedArray<RID> *p_instances, const RID &p_shadow_atlas, uint32_t p_flags) const;

	void _start();
	void _stop();
	ObjectID attached_world;
	void _step(double p_delta);
	void _gather_emitters(LocalVector<FlowBackend::Emitter> &r_emitters, AABB &r_emitter_bounds);
	void _gather_colliders(const AABB &p_region, LocalVector<FlowBackend::Emitter> &r_emitters);
	void _gather_light(Vector3 &r_direction, Color &r_color, Color &r_ambient);
	void _fill_render_settings(FlowBackend::Settings &r_settings);
	void _read_back();
	float _sample_field(const Vector<uint8_t> &p_field, const Vector3 &p_position) const;
	void _post(const Callable &p_call);
	void _rt_create(int p_max_blocks);
	void _rt_destroy();
	void _rt_step();

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void set_cell_size(float p_v) { cell_size = MAX(p_v, 0.005f); }
	float get_cell_size() const { return cell_size; }
	void set_max_blocks(int p_v);
	int get_max_blocks() const { return max_blocks; }
	void set_buoyancy(float p_v) { buoyancy = p_v; }
	float get_buoyancy() const { return buoyancy; }
	void set_cooling_rate(float p_v) { cooling_rate = MAX(p_v, 0.0f); }
	float get_cooling_rate() const { return cooling_rate; }
	void set_smoke_fade(float p_v) { smoke_fade = MAX(p_v, 0.0f); }
	float get_smoke_fade() const { return smoke_fade; }
	void set_vorticity(float p_v) { vorticity = MAX(p_v, 0.0f); }
	float get_vorticity() const { return vorticity; }
	void set_ignition_temperature(float p_v) { ignition_temperature = p_v; }
	float get_ignition_temperature() const { return ignition_temperature; }
	void set_combustion(bool p_v) { combustion = p_v; }
	bool is_combustion() const { return combustion; }

	void set_collide_with_bodies(bool p_v) { collide_with_bodies = p_v; }
	bool is_collide_with_bodies() const { return collide_with_bodies; }
	void set_collision_mask(uint32_t p_mask) { collision_mask = p_mask; }
	uint32_t get_collision_mask() const { return collision_mask; }
	void set_collision_range(float p_v) { collision_range = MAX(p_v, 0.0f); }
	float get_collision_range() const { return collision_range; }
	void set_floor_enabled(bool p_v) { floor_enabled = p_v; }
	bool is_floor_enabled() const { return floor_enabled; }
	void set_floor_height(float p_v) { floor_height = p_v; }
	float get_floor_height() const { return floor_height; }

	void set_smoke_density(float p_v) { smoke_density = MAX(p_v, 0.0f); }
	float get_smoke_density() const { return smoke_density; }
	void set_smoke_color(const Color &p_v) { smoke_color = p_v; }
	Color get_smoke_color() const { return smoke_color; }
	void set_smoke_ramp(const Ref<Gradient> &p_ramp) { smoke_ramp = p_ramp; }
	Ref<Gradient> get_smoke_ramp() const { return smoke_ramp; }
	void set_heat_ramp(const Ref<Gradient> &p_ramp) { heat_ramp = p_ramp; }
	Ref<Gradient> get_heat_ramp() const { return heat_ramp; }
	void set_temperature_range(float p_v) { temperature_range = MAX(p_v, 0.001f); }
	float get_temperature_range() const { return temperature_range; }
	void set_emission_strength(float p_v) { emission_strength = MAX(p_v, 0.0f); }
	float get_emission_strength() const { return emission_strength; }
	void set_shadow_steps(int p_v) { shadow_steps = CLAMP(p_v, 0, 128); }
	int get_shadow_steps() const { return shadow_steps; }

	void set_light_scattering(bool p_v) { light_scattering = p_v; }
	bool is_light_scattering() const { return light_scattering; }
	void set_scattering_max_lights(int p_v) { scattering_max_lights = CLAMP(p_v, 1, FlowScatterRD::MAX_LIGHTS); }
	int get_scattering_max_lights() const { return scattering_max_lights; }
	void set_scattering_strength(float p_v) { scattering_strength = MAX(p_v, 0.0f); }
	float get_scattering_strength() const { return scattering_strength; }
	void set_scattering_anisotropy(float p_v) { scattering_anisotropy = CLAMP(p_v, -0.95f, 0.95f); }
	float get_scattering_anisotropy() const { return scattering_anisotropy; }
	void set_scattering_directional(bool p_v) {
		scattering_directional = p_v;
		scatter_search_countdown = 0; // find the lights again now
	}
	bool is_scattering_directional() const { return scattering_directional; }
	void set_scattering_shadow_steps(int p_v) { scattering_shadow_steps = CLAMP(p_v, 0, 64); }
	int get_scattering_shadow_steps() const { return scattering_shadow_steps; }
	void set_scattering_shadow_reach(float p_v) { scattering_shadow_reach = MAX(p_v, 0.05f); }
	float get_scattering_shadow_reach() const { return scattering_shadow_reach; }
	void set_scattering_receive_shadows(BitField<ScatteringShadowFlags> p_flags) { scattering_receive_shadows = uint32_t(p_flags) & (SCATTERING_SHADOW_OMNI | SCATTERING_SHADOW_SPOT); }
	BitField<ScatteringShadowFlags> get_scattering_receive_shadows() const { return BitField<ScatteringShadowFlags>(scattering_receive_shadows); }
	void set_scattering_resolution(ScatteringResolution p_v) { scattering_resolution = p_v; }
	ScatteringResolution get_scattering_resolution() const { return scattering_resolution; }

	void set_cpu_readback(bool p_v);
	bool is_cpu_readback() const { return cpu_readback; }
	// Smoke at a world position, from the last CPU readback (needs
	// cpu_readback; -1 without it).
	float sample_smoke(const Vector3 &p_position) const;
	// Temperature at a world position, the same way.
	float sample_temperature(const Vector3 &p_position) const;

	bool is_running() const { return grid != nullptr; }
	Dictionary get_stats() const;

	// From PhysXFlowRenderEffect, on the render thread.
	Vector3 get_draw_center() const { return draw_center; }
	// p_lights / p_shadow_atlas: the frame's light instances and shadow atlas
	// (RenderDataRD), for the scattering pass's shadows. An XR camera's eyes are
	// drawn one after another in one Flow frame: p_first_view / p_last_view.
	void render_rd(PhysXFlowRenderEffect *p_effect, RenderingDevice *p_rd, const float p_view[16], const float p_projection[16], const Size2i &p_size, const RID &p_depth, const RID &p_color, const Projection &p_inv_view_projection, const Vector3 &p_camera, const PagedArray<RID> *p_lights, const RID &p_shadow_atlas, bool p_first_view = true, bool p_last_view = true);

	PackedStringArray get_configuration_warnings() const override;

	PhysXFlow3D();
	~PhysXFlow3D();
};

VARIANT_ENUM_CAST(PhysXFlow3D::ScatteringResolution);
VARIANT_BITFIELD_CAST(PhysXFlow3D::ScatteringShadowFlags);

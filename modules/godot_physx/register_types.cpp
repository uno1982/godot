/**************************************************************************/
/*  register_types.cpp                                                    */
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

#include "register_types.h"

#include "godot_physx_project_settings.h"
#include "godot_physx_server_3d.h"
#include "physx_info.h"
#ifdef GODOT_PHYSX_BLAST
#include "blast/godot_physx_blast_probe.h"
#include "blast/physx_blast_asset.h"
#include "blast/physx_blast_authoring.h"
#include "blast/physx_destructible_3d.h"
#endif
#include "cloth/physx_skinned_cloth_3d.h"
#include "groom/physx_groom.h"
#include "groom/physx_groom_3d.h"
#ifdef GODOT_PHYSX_FLOW
#include "flow/physx_flow_3d.h"
#include "flow/physx_flow_emitter_3d.h"
#include "flow/physx_flow_render_effect.h"
#endif
#include "nodes/physx_chunk_emitter_3d.h"
#include "nodes/physx_cloth_3d.h"
#include "nodes/physx_fluid_render_effect.h"
#include "nodes/physx_gi_effect.h"
#include "nodes/physx_granular_3d.h"
#include "nodes/physx_lens_dirt_3d.h"
#include "nodes/physx_lens_dirt_effect.h"
#include "nodes/physx_lens_flare_3d.h"
#include "nodes/physx_lens_flare_effect.h"
#include "nodes/physx_lens_flare_settings.h"
#include "nodes/physx_light_shafts_3d.h"
#include "nodes/physx_light_shafts_effect.h"
#include "nodes/physx_particle_fluid_3d.h"
#include "nodes/pxtgi_3d.h"
#include "vehicle/godot_physx_motorcycle_probe.h"
#include "vehicle/godot_physx_tank_probe.h"
#include "vehicle/godot_physx_vehicle_probe.h"
#include "vehicle/physx_motorcycle_3d.h"
#include "vehicle/physx_tank_3d.h"
#include "vehicle/physx_vehicle_3d.h"
#include "vehicle/physx_vehicle_wheel_3d.h"
#include "water/physx_boat_3d.h"
#include "water/physx_buoyancy_3d.h"
#include "water/physx_water_spray_3d.h"
#include "water/physx_water_surface_3d.h"
#include "water/physx_water_wake_3d.h"
#include "water/water_ripple_probe.h"

#include "core/config/project_settings.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "servers/physics_3d/physics_server_3d_wrap_mt.h"

#ifdef TOOLS_ENABLED
#include "editor/editor_node.h"
#include "editor/export/editor_export.h"
#include "editor/physx_cloth_paint_plugin.h"
#include "editor/physx_editor_plugin.h"
#include "editor/physx_export_plugin.h"
#ifdef GODOT_PHYSX_ALEMBIC
#include "groom/alembic/resource_importer_alembic_groom.h"
#endif
#include "editor/plugins/editor_plugin.h"
#endif

static PhysicsServer3D *create_physx_physics_server() {
#ifdef THREADS_ENABLED
	bool run_on_separate_thread = GLOBAL_GET("physics/3d/run_on_separate_thread");
#else
	bool run_on_separate_thread = false;
#endif

	GodotPhysXServer3D *physics_server = memnew(GodotPhysXServer3D);

	return memnew(PhysicsServer3DWrapMT(physics_server, run_on_separate_thread));
}

#ifdef TOOLS_ENABLED
static void _physx_editor_init() {
	Ref<PhysXExportPlugin> export_plugin;
	export_plugin.instantiate();
	EditorExport::get_singleton()->add_export_plugin(export_plugin);
#ifdef GODOT_PHYSX_ALEMBIC
	Ref<ResourceImporterAlembicGroom> groom_importer;
	groom_importer.instantiate();
	ResourceFormatImporter::get_singleton()->add_importer(groom_importer);
#endif
}
#endif

void initialize_godot_physx_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SERVERS) {
		GodotPhysXProjectSettings::register_settings();
		PhysicsServer3DManager::get_singleton()->register_server("PhysX", callable_mp_static(&create_physx_physics_server));
	}

	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_ABSTRACT_CLASS(PhysX);
		GDREGISTER_CLASS(PhysXParticleFluid3D);
		GDREGISTER_INTERNAL_CLASS(PhysXFluidRenderEffect);
		GDREGISTER_CLASS(PhysXGranular3D);
		GDREGISTER_CLASS(PhysXCloth3D);
		GDREGISTER_CLASS(PhysXSkinnedCloth3D);
		GDREGISTER_CLASS(PhysXChunkEmitter3D);
		GDREGISTER_CLASS(PhysXGroom);
		GDREGISTER_CLASS(PhysXGroom3D);
		GDREGISTER_CLASS(PhysXLightShafts3D);
		GDREGISTER_INTERNAL_CLASS(PhysXLightShaftsEffect);
		GDREGISTER_CLASS(PXTGI3D);
		GDREGISTER_INTERNAL_CLASS(PhysXGIEffect);
		GDREGISTER_CLASS(PhysXLensDirt3D);
		GDREGISTER_INTERNAL_CLASS(PhysXLensDirtEffect);
		GDREGISTER_CLASS(PhysXLensFlareSettings);
		GDREGISTER_CLASS(PhysXLensFlare3D);
		GDREGISTER_INTERNAL_CLASS(PhysXLensFlareEffect);
		// Runtime-bridge MVP probe, not public API yet -- see water/water_ripple_probe.h.
		GDREGISTER_CLASS(WaterRippleProbe);
		GDREGISTER_CLASS(PhysXWaterSurface3D);
		GDREGISTER_CLASS(PhysXBuoyancy3D);
		GDREGISTER_CLASS(PhysXBoat3D);
		GDREGISTER_CLASS(PhysXWaterWake3D);
		GDREGISTER_CLASS(PhysXWaterSpray3D);
		// Runtime-bridge MVP probe, not public API yet -- see vehicle/godot_physx_vehicle_probe.h.
		GDREGISTER_CLASS(GodotPhysXVehicleProbe);
		GDREGISTER_CLASS(PhysXVehicle3D);
		GDREGISTER_CLASS(PhysXVehicleWheel3D);
		// Runtime-bridge MVP probe, not public API yet -- see vehicle/godot_physx_motorcycle_probe.h.
		GDREGISTER_CLASS(GodotPhysXMotorcycleProbe);
		GDREGISTER_CLASS(PhysXMotorcycle3D);
		// Runtime-bridge MVP probe, not public API yet -- see vehicle/godot_physx_tank_probe.h.
		GDREGISTER_CLASS(GodotPhysXTankProbe);
		GDREGISTER_CLASS(PhysXTank3D);
#ifdef GODOT_PHYSX_BLAST
		// Runtime-bridge MVP probe, not public API yet -- see blast/godot_physx_blast_probe.h.
		GDREGISTER_CLASS(GodotPhysXBlastProbe);
		GDREGISTER_CLASS(PhysXDestructible3D);
		GDREGISTER_CLASS(PhysXBlastAsset);
		GDREGISTER_CLASS(PhysXBlastAuthoring);
#endif
#ifdef GODOT_PHYSX_FLOW
		GDREGISTER_CLASS(PhysXFlow3D);
		GDREGISTER_CLASS(PhysXFlowEmitter3D);
		GDREGISTER_CLASS(PhysXFlowRenderEffect);
#endif
	}

#ifdef TOOLS_ENABLED
	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		EditorPlugins::add_by_type<PhysXEditorPlugin>();
		EditorPlugins::add_by_type<PhysXClothPaintPlugin>();
		EditorNode::add_init_callback(_physx_editor_init);
	}
#endif
}

void uninitialize_godot_physx_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		PhysXFluidRenderEffect::free_shared();
		PhysXSkinnedCloth3D::free_shared();
		PhysXLightShaftsEffect::free_shared();
		PhysXLensDirtEffect::free_shared();
		PhysXLensFlareEffect::free_shared();
		PhysXGIEffect::free_shared();
		PXTGI3D::free_shared();
		PhysXGroom3D::free_shared();
	}
#ifdef GODOT_PHYSX_FLOW
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		PhysXFlowRenderEffect::free_shared();
	}
#endif
}

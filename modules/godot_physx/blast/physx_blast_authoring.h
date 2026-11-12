/**************************************************************************/
/*  physx_blast_authoring.h                                               */
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

#include "physx_blast_asset.h"

#include "core/object/ref_counted.h"
#include "core/object/worker_thread_pool.h"
#include "core/os/mutex.h"
#include "core/templates/local_vector.h"
#include "core/templates/safe_refcount.h"
#include "core/variant/typed_array.h"

class Material;
class Mesh;
class Texture2D;
class PhysXBlastFractureTask;

// What fracture_mesh() reads from the Mesh (and the cutout texture) before any of the work: the rest runs on these
// alone, so it can run on another thread.
struct PhysXBlastFractureInput {
	PackedVector3Array verts;
	PackedVector3Array normals; // empty if no surface had any
	PackedVector2Array uvs;
	LocalVector<uint32_t> indices;
	LocalVector<int32_t> triangle_surfaces;
	int surface_count = 0;
	TypedArray<Material> surface_materials;
	int site_count = 0;
	int seed = 0;
	int pattern = 0;
	// PATTERN_CUTOUT: the pattern as one gray byte per pixel, or why there's none (reported when fracturing starts).
	PackedByteArray cutout_gray;
	uint32_t cutout_width = 0;
	uint32_t cutout_height = 0;
	String cutout_error;
};

// What the work produces, or why it failed.
struct PhysXBlastFractureOutput {
	String error;
	PackedStringArray warnings;
	bool cancelled = false;
	bool done = false;
	PackedByteArray asset_bytes;
	Array chunk_points;
	Array chunk_normals;
	Array chunk_uvs;
	Array chunk_surface_starts;
	PackedInt32Array chunk_interior_starts;
	Array chunk_hulls;
	String hull_format;
};

// In-engine mesh fracturing -- the real replacement for the standalone
// throwaway blast_test_gen.cpp tool this whole Blast effort prototyped with.
// Ports that tool's proven-correct authoring call sequence
// (NvBlastExtAuthoringCreateFractureTool -> VoronoiSitesGenerator ->
// voronoiFracturing -> NvBlastExtAuthoringProcessFracture) into real module
// code, using the module's own PxPhysics (GodotPhysXServer3D) to cook
// collision hulls instead of a standalone PxCreatePhysics call.
//
// A RefCounted rather than a set of static functions so it's directly
// callable from GDScript (ClassDB.instantiate("PhysXBlastAuthoring")) for a
// headless test today, and so a future editor dock has an obvious object to
// hold onto without inventing a second calling convention.
class PhysXBlastAuthoring : public RefCounted {
	GDCLASS(PhysXBlastAuthoring, RefCounted);

public:
	// NvBlastExtAuthoring's FractureTool supports more than one fracture
	// algorithm -- PATTERN_VORONOI (organic/rock-like chunks, the original
	// only option), PATTERN_SLICING (regular, brick-like pieces from cutting
	// planes -- much better suited to man-made/architectural shapes like
	// crates or walls than Voronoi's random cells), and PATTERN_CUTOUT (a 2D
	// pattern extruded through the mesh -- glass-pane spiderweb cracks, brick
	// walls, anything with a real authored break-up shape). Unlike the other
	// two, NvBlast doesn't generate the Cutout pattern itself -- same as
	// Unreal's own Blast integration (BlastFractureSettingsCutout::Pattern,
	// a plain UTexture2D artists import), it's a grayscale image the caller
	// supplies via p_cutout_pattern; PATTERN_CUTOUT with no pattern set fails
	// the same way Unreal's does with no Pattern texture assigned.
	//
	// Pattern authoring rule, found the hard way: every crack line must
	// reach the image boundary. A pattern whose lines stop short (e.g. a
	// spiderweb with cracks that fade out before the edge) leaves one
	// single connected "background" region wrapping the whole pattern with
	// a hole in the middle -- unlike a real broken-glass pane, where every
	// shard is a closed polygon. That topology crashed two different ways
	// deep in the SDK's own Cutout code (FractureToolImpl::cutout's
	// SweepingAccelerator construction), not this module's integration --
	// confirmed by testing the exact same pattern with lines extended to
	// the edge, which fractures and fully verifies clean. A straight grid/
	// brick pattern is naturally edge-to-edge already and was never
	// affected; only curved/radial patterns are at risk of this if authored
	// with cracks that stop short.
	enum FracturePattern {
		PATTERN_VORONOI,
		PATTERN_SLICING,
		PATTERN_CUTOUT,
	};

protected:
	static void _bind_methods();

public:
	// Fractures p_mesh's surface 0 into roughly p_site_count pieces (plus the
	// implicit root chunk = the whole unfractured mesh, chunk 0 -- same
	// convention PhysXDestructible3D/GodotPhysXBlastProbe already assume),
	// using p_pattern's algorithm. For PATTERN_SLICING, p_site_count is only
	// approximate -- it's converted to a roughly-cube-root split across the
	// 3 slicing axes, since slicing works in slice-counts-per-axis, not a
	// single site count. p_site_count is ignored entirely for PATTERN_CUTOUT
	// -- piece count there comes from however many distinct regions
	// p_cutout_pattern actually contains, and p_cutout_pattern is required
	// (a null Ref fails, same as PATTERN_VORONOI/PATTERN_SLICING would fail
	// with a null p_mesh). Returns a null Ref on failure (mesh has no
	// surfaces, cooking failed, etc).
	Ref<PhysXBlastAsset> fracture_mesh(const Ref<Mesh> &p_mesh, int p_site_count, int p_seed, FracturePattern p_pattern = PATTERN_VORONOI, const Ref<Texture2D> &p_cutout_pattern = Ref<Texture2D>());

	// The same fracture on a worker thread: the mesh is read here, the rest runs in the background. The task reports
	// each stage and finishes with the asset (on the main thread), and can be cancelled.
	Ref<PhysXBlastFractureTask> fracture_mesh_async(const Ref<Mesh> &p_mesh, int p_site_count, int p_seed, FracturePattern p_pattern = PATTERN_VORONOI, const Ref<Texture2D> &p_cutout_pattern = Ref<Texture2D>());

	// Why the last fracture_mesh() failed (empty if it didn't), and what it warned about -- the same text it printed,
	// for a UI to show (the fracture dialog does).
	String get_last_error() const { return last_error; }
	PackedStringArray get_last_warnings() const { return last_warnings; }

	// The stages the work goes through, in order (fracture_mesh_async() reports them).
	enum Stage {
		STAGE_CHECKING = 1,
		STAGE_CLEANING,
		STAGE_FRACTURING,
		STAGE_FINALIZING,
		STAGE_BONDS,
		STAGE_HULLS,
		STAGE_COUNT = STAGE_HULLS,
	};
	static String stage_name(int p_stage);

	// Reads the mesh and the cutout texture: false (with r_output.error) if there's nothing usable to fracture.
	static bool read_input(const Ref<Mesh> &p_mesh, int p_site_count, int p_seed, FracturePattern p_pattern, const Ref<Texture2D> &p_cutout_pattern, PhysXBlastFractureInput &r_input, PhysXBlastFractureOutput &r_output);
	// The work itself, on any thread. p_task (may be null) is told each stage, and is asked between stages whether to stop.
	static void run(const PhysXBlastFractureInput &p_input, PhysXBlastFractureOutput &r_output, PhysXBlastFractureTask *p_task);
	// The asset from a finished run, or null if it failed.
	static Ref<PhysXBlastAsset> make_asset(const PhysXBlastFractureInput &p_input, const PhysXBlastFractureOutput &p_output);

private:
	String last_error;
	PackedStringArray last_warnings;
};

// A fracture running in the background (PhysXBlastAuthoring::fracture_mesh_async()). Its signals come on the main thread.
// Blast can't stop partway through a stage, so cancel() takes effect when the running stage ends; finished is emitted
// then, with no asset.
class PhysXBlastFractureTask : public RefCounted {
	GDCLASS(PhysXBlastFractureTask, RefCounted);

	friend class PhysXBlastAuthoring;

	PhysXBlastFractureInput input;
	PhysXBlastFractureOutput output;
	Ref<PhysXBlastFractureTask> keep_alive; // while the worker runs
	WorkerThreadPool::TaskID task_id = WorkerThreadPool::INVALID_TASK_ID;
	SafeFlag cancel_requested;
	SafeFlag running;
	SafeNumeric<int> stage;
	Mutex description_mutex;
	String description;
	// Copied from output on the main thread when finished.
	Ref<PhysXBlastAsset> asset;
	String last_error;
	PackedStringArray last_warnings;
	bool cancelled = false;

	void _work();
	void _emit_stage(int p_stage, const String &p_description);
	void _finish();

protected:
	static void _bind_methods();

public:
	// From the worker: the stage it's starting (p_detail: progress within it, e.g. "12 of 64"). False if it should stop.
	bool report_stage(int p_stage, const String &p_detail = String());

	void cancel();
	bool is_running() const { return running.is_set(); }
	bool is_cancelled() const { return cancelled; }
	int get_stage() const { return stage.get(); }
	int get_stage_count() const { return PhysXBlastAuthoring::STAGE_COUNT; }
	String get_stage_description();
	Ref<PhysXBlastAsset> get_asset() const { return asset; }
	String get_last_error() const { return last_error; }
	PackedStringArray get_last_warnings() const { return last_warnings; }
};

VARIANT_ENUM_CAST(PhysXBlastAuthoring::FracturePattern);

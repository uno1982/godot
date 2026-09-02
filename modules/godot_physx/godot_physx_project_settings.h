/**************************************************************************/
/*  godot_physx_project_settings.h                                        */
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

class GodotPhysXProjectSettings {
public:
	// physics/physx_3d/simulation/enhanced_determinism
	//
	// Sets PxSceneFlag::eENABLE_ENHANCED_DETERMINISM: the CPU simulation then
	// produces identical results across runs on the same binary/platform,
	// independent of the CPU worker count and API call order (it is NOT
	// cross-platform deterministic). Has a performance cost.
	//
	// GPU dynamics is never deterministic, so enabling this forces the CPU
	// solver even when a CUDA device is available.
	inline static bool enhanced_determinism = false;

	// physics/physx_3d/simulation/solver_type
	//
	// 0 = PGS (Projected Gauss-Seidel, default) -- PhysX's classic solver.
	// 1 = TGS (Temporal Gauss-Seidel) -- steadier for joint chains under
	// sustained external forces (wind, thrusters), but can be looser on joints
	// in large mixed rigid-body scenes.
	inline static int solver_type = 0;

	// physics/physx_3d/simulation/allow_sleep
	//
	// When false, no rigid body ever sleeps (equivalent to RigidBody3D.can_sleep
	// off on every body). Useful for debugging and for setups that need every
	// body integrated every step.
	inline static bool allow_sleep = true;

	// physics/physx_3d/simulation/stabilization
	//
	// PxSceneFlag::eENABLE_STABILIZATION. Damps low-mass stacked/piled bodies
	// toward rest so they settle and can sleep instead of jittering; the same
	// mechanism Unity and Unreal call "stabilization". Turn it off if it causes
	// visible positional drift on very light bodies.
	inline static bool stabilization = true;

	// physics/physx_3d/simulation/cpu_worker_threads
	//
	// 0 = auto (most of the machine for the CPU solver, a small pool for the
	// GPU path). A fixed value is useful for reproducible profiling.
	inline static int cpu_worker_threads = 0;

	static void register_settings();
	static void read_settings();
};

/**************************************************************************/
/*  godot_physx_project_settings.cpp                                      */
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

#include "godot_physx_project_settings.h"

#include "core/config/project_settings.h"

void GodotPhysXProjectSettings::register_settings() {
	GLOBAL_DEF(PropertyInfo(Variant::BOOL, "physics/physx_3d/simulation/enhanced_determinism"), false);
	GLOBAL_DEF(PropertyInfo(Variant::INT, "physics/physx_3d/simulation/solver_type", PROPERTY_HINT_ENUM, "PGS,TGS"), 0);
	GLOBAL_DEF(PropertyInfo(Variant::BOOL, "physics/physx_3d/simulation/allow_sleep"), true);
	GLOBAL_DEF(PropertyInfo(Variant::BOOL, "physics/physx_3d/simulation/stabilization"), true);
	GLOBAL_DEF(PropertyInfo(Variant::INT, "physics/physx_3d/simulation/cpu_worker_threads", PROPERTY_HINT_RANGE, U"0,32,1"), 0);
}

void GodotPhysXProjectSettings::read_settings() {
	enhanced_determinism = GLOBAL_GET("physics/physx_3d/simulation/enhanced_determinism");
	solver_type = GLOBAL_GET("physics/physx_3d/simulation/solver_type");
	allow_sleep = GLOBAL_GET("physics/physx_3d/simulation/allow_sleep");
	stabilization = GLOBAL_GET("physics/physx_3d/simulation/stabilization");
	cpu_worker_threads = GLOBAL_GET("physics/physx_3d/simulation/cpu_worker_threads");
}

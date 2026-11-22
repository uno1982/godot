/**************************************************************************/
/*  physx_lens_flare_3d.h                                                 */
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

#include "physx_lens_flare_settings.h"

#include "scene/3d/node_3d.h"

// A camera lens flare: the frame's bright light -- the sun, fire, headlights -- reflected back over the screen as a
// row of tinted ghosts through the screen's center and a halo ring, the way light bounces between a real lens's
// elements. How it looks is a PhysXLensFlareSettings resource, so a look can be saved as a preset and shared; without
// one the node draws the default look. Put a PhysXLensDirt3D in the scene too and the flare lights the dirt.
class PhysXLensFlare3D : public Node3D {
	GDCLASS(PhysXLensFlare3D, Node3D);

	Ref<PhysXLensFlareSettings> settings;
	Ref<PhysXLensFlareSettings> fallback; // drawn while settings is null

	ObjectID attached_world;

	void _attach();
	void _detach();
	void _publish();

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void set_settings(const Ref<PhysXLensFlareSettings> &p_settings);
	Ref<PhysXLensFlareSettings> get_settings() const { return settings; }
};

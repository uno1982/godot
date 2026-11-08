/**************************************************************************/
/*  groom_sim.h                                                           */
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

#include "core/math/transform_3d.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "core/templates/rid.h"
#include "core/variant/variant.h"

class PhysXGroom;

// Hair that moves, on the RenderingDevice (compute, any GPU the renderer supports): one thread per strand, similar to
// AMD's TressFX. Each substep the root and the point after it follow the groom's transform, the rest of the strand
// falls and flies (gravity, wind, inertia, damped), is pulled back toward the groomed shape (most at the root), keeps
// its bends (each point its distance to the points two and three before it: curls stay curls) and its segment
// lengths, with corrections that move both ends, and is pushed out of the colliders. The points land in world space in an RGBA32F texture, one
// texel a point, that the strand mesh's vertex shader and the screen-space passes read.
class GroomSim {
public:
	static constexpr int MAX_COLLIDERS = 16;

	// A capsule (a sphere when a == b), world space: where it is this frame and where it was last, swept between them
	// through the substeps (a fast shoulder moves with the hair instead of jumping into it once a frame).
	struct Collider {
		Vector3 a;
		Vector3 b;
		Vector3 a_prev;
		Vector3 b_prev;
		float radius = 0.0f;
	};

	struct Settings {
		// Fractions per 1/60 s: how far the strands go back toward the groomed shape (at the root; none at the tip), and
		// how far each segment goes back to its bend from the one before it.
		float stiffness = 0.3f;
		float bend_stiffness = 0.5f;
		// Velocity lost per 1/60 s (as TressFX's damping).
		float damping = 0.05f;
		Vector3 gravity;
		// Air velocity (m/s) and how strongly the strands are dragged along with it.
		Vector3 wind;
		float drag = 1.5f;
		// How far the strands stay off the colliders.
		float margin = 0.002f;
	};

	// Uploads the groom's strands (in its own space) and makes the texture: on the render thread, waited for.
	void build(const Ref<PhysXGroom> &p_groom);
	// One frame: the groom's transform this frame and the one it last stepped to, substeps between them. Queued on the
	// render thread. False when the frame was shorter than a substep (its time carries over): nothing ran, so the next
	// step starts from p_from again.
	bool step(const Transform3D &p_from, const Transform3D &p_to, double p_delta, const Settings &p_settings, const LocalVector<Collider> &p_colliders);
	// The next step starts from the groomed shape, at rest.
	void reset();
	void clear();
	bool is_built() const;

	// The points' texture (RenderingDevice RID) and its width in texels.
	RID get_texture() const;
	int get_width() const;

	static void free_shared();

	GroomSim();
	~GroomSim();

private:
	class GPU;
	Ref<GPU> gpu;
	// Time not stepped yet, less than a substep.
	double carried = 0.0;
};

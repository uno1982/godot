# PhysX 3D physics module

A [PhysX 5](https://github.com/NVIDIA-Omniverse/PhysX) implementation of Godot's
`PhysicsServer3D`, in the same shape as the built-in Jolt and Godot Physics
backends. It is selected per project and every 3D physics node
(`RigidBody3D`, `CharacterBody3D`, `Area3D`, the joints, `RayCast3D`, …) works
against it unchanged.

The reason to use it over Jolt is **GPU rigid-body dynamics** on NVIDIA
hardware, and the room that a built-in backend leaves for GPU particle,
destruction and fluid effects that have no representation in the stock physics
interface. On the CPU alone, Jolt is the better choice.

## Building

The PhysX 5 SDK is **not vendored** — it is built out of tree and linked in. The
module is skipped (with a one-line notice) unless an SDK is configured, so a
stock Godot build is unaffected.

### Prerequisites

On top of everything a normal Godot Windows editor build needs (Python, SCons,
Visual Studio with the C++ workload and Windows SDK, the D3D12 Agility SDK):

- **CMake** and **git** — the PhysX SDK builds with its own CMake.
- **CUDA Toolkit** — only for a GPU build (`--gpu`); built and tested against
  12.8. On Windows: `winget install Nvidia.CUDA --version 12.8` (installs to
  `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8` and sets
  `CUDA_PATH`; open a fresh terminal afterwards). The end-user machine only
  needs an NVIDIA driver (`nvcuda.dll`), not the toolkit.

Currently only the Win64 / MSVC build presets ship (`misc/physx_presets/`).
Other platforms need an equivalent preset and PhysX's `generate_projects.sh`.

### Step 1 — build the PhysX SDK

```
python modules/godot_physx/misc/build_physx.py            # CPU only
python modules/godot_physx/misc/build_physx.py --gpu      # + GPU dynamics / fluid
```

This clones NVIDIA's PhysX repo (pinned) into a `physx-sdk/` folder next to the
Godot repo, applies the Godot-tuned preset, builds and installs it, then prints
the exact `scons` command for step 2. Pass `--src <dir>` to reuse an existing
PhysX checkout instead of cloning.

To build the SDK by hand instead: static libraries, static CRT
(`NV_USE_STATIC_WINCRT=True`, i.e. `/MT` on Windows), `release` config. Then
point scons at the install directory (the one containing `include/` and `bin/`)
with `physx_sdk=<path>` or the `PHYSX_SDK` environment variable.

### Step 2 — build the editor

```
scons platform=windows target=editor physx_sdk=<path from step 1>            # CPU
scons platform=windows target=editor physx_sdk=<path> physx_gpu=yes          # GPU
```

For a **GPU** build, also copy the CUDA runtime next to the built binary:

```
copy <sdk>\bin\win.x86_64.vc143.mt\release\PhysXGpu_64.dll bin\
```

At startup a GPU build logs `PhysX: CUDA context ready on device '...'`; if no
usable CUDA device is found it warns and falls back to CPU simulation.

To confirm a stock build is unaffected, build with
`module_godot_physx_enabled=no` (or simply without an SDK configured).

## Selecting the backend

**Project Settings → Physics → 3D → Physics Engine → `PhysX`**, or in
`project.godot`:

```
[physics]
3d/physics_engine="PhysX"
```

The choice is project-wide and applied at startup; it cannot be changed per
scene or at runtime.

## Project settings

Under `physics/physx_3d/simulation/`:

| Setting | Default | Meaning |
| --- | --- | --- |
| `solver_type` | `PGS` | `PGS` is PhysX's classic solver and matches the other backends' feel in large rigid-body scenes. `TGS` is steadier for joint chains under sustained external forces (wind, thrusters) but can be looser on joints in big mixed piles. GPU soft bodies need `TGS` for firm soft-vs-soft contact. |
| `enhanced_determinism` | `false` | Makes the CPU simulation reproducible across runs on the same binary and platform, independent of worker-thread count and API call order. It is **not** cross-platform deterministic and has a performance cost. Enabling it forces the CPU solver even when a CUDA device is present, because the GPU solver is never deterministic. |
| `allow_sleep` | `true` | When off, no rigid body ever sleeps — the same as turning `RigidBody3D.can_sleep` off on every body. Useful for debugging or setups that need every body integrated every step. |
| `stabilization` | `true` | `PxSceneFlag::eENABLE_STABILIZATION` — damps low-mass stacked bodies toward rest so piles settle and sleep instead of jittering. Turn it off if it causes visible drift on very light bodies. |
| `cpu_worker_threads` | `0` (auto) | Size of the PhysX CPU task pool. `0` picks a value based on the active path: a small pool (2–4) when GPU dynamics is running, since the CPU mostly waits on the GPU each step; most of the machine otherwise. A fixed value overrides this in both cases — setting it high while on the GPU path will usually cost performance, not gain it. |

And `physics/physx_3d/soft_body/mode` — `Auto` / `CPU` / `GPU` for the stock
`SoftBody3D` node (see [Soft bodies](#soft-bodies--stock-softbody3d)).

The standard `physics/3d/sleep_threshold_linear`, `sleep_threshold_angular` and
`time_before_sleep` project settings also apply — they are mapped onto PhysX's
sleep energy threshold and wake counter.

## Area3D overrides

`Area3D` gravity, damping and wind overrides are applied to the rigid bodies
that overlap the area, each physics step, folded across overlapping areas in
`priority` order (`COMBINE` adds, `REPLACE` replaces):

- **Gravity** — directional or point (`gravity_point`), with the optional
  `gravity_point_unit_distance` falloff.
- **Linear / angular damp** — applied as a velocity-proportional drag on top of
  each body's own damping.
- **Wind** — `wind_force_magnitude` along the `wind_source_path` node's −Z, with
  `wind_attenuation_factor` falloff over downwind distance. Note that stock Godot
  and Jolt apply area wind only to `SoftBody3D`; this backend also applies it to
  rigid bodies.

## GPU fluid — `PhysXParticleFluid3D`

A `GeometryInstance3D`-derived node that fills a box region with GPU position-
based fluid particles (PhysX 5 `PxPBDParticleSystem`). The particles collide with
the rigid bodies in the same space and are drawn as a `MultiMesh` of spheres
(`material_override` applies). Tunables: `particle_count`, `particle_size`,
`viscosity`, `surface_tension`, `cohesion`, `vorticity`.

Set `emitting` to stream particles in over time instead of spawning them all at
once — a faucet or hose. Particles spawn near the node origin at `emission_rate`
per second within `emission_radius` and with `emission_velocity` (node-local),
and once `particle_count` is reached the oldest particles are recycled.

Set `foam_enabled` to have the solver spawn diffuse particles — foam, spray and
bubbles — where the fluid is agitated (`PxParticleAndDiffuseBuffer`). They render
as a separate sphere cloud, do not affect the fluid, and are tuned with
`foam_particle_count`, `foam_lifetime`, `foam_threshold`, `foam_buoyancy`.

`get_submersion(world_aabb)` returns the 0..1 fraction of a box currently filled
with fluid — a primitive for script-side buoyancy.

Rigid-body interaction is collision only: bodies splash and displace the fluid,
but PhysX PBD does not produce accurate density-based buoyancy (a light body
does not cleanly float, a dense one does not cleanly sink through). Use
`get_submersion()` to apply your own buoyant force where that matters.

GPU-only: the node is inert unless the active physics engine is PhysX, the build
has GPU support, and a CUDA device is present. See the class reference for
details.

### Editor

`PhysXParticleFluid3D` has a viewport gizmo: a wireframe box for `spawn_region_size`
(with drag handles), a ring for `emission_radius` and an arrow for
`emission_velocity`. The node shows configuration warnings when the 3D physics
engine is not PhysX, when `surface_anisotropy` is set without `surface_mesh` or
while emitting, or at very high particle counts. The GPU sim does not run in the
editor — press Play to see the fluid.

### Surface rendering

Set `surface_mesh` on the node to draw the fluid as a smooth liquid surface
instead of spheres: PhysX smooths the particle positions and marching-cubes a
triangle mesh on the GPU (`PxIsosurfaceExtractor`), which the node renders as an
`ArrayMesh`. Give it a water look with `material_override` (transparency +
refraction). It uses the normal Godot material pipeline and needs no compositor
setup. `surface_anisotropy` optionally feeds PhysX per-particle anisotropy to the
extractor for sharper crests — off by default; leave it off while emitting, where
fast particles along the stream can mesh as spikes.

## Cloth — `PhysXCloth3D`

A cloth patch: a generated grid or a supplied triangle mesh, simulated and drawn
as an `ArrayMesh` with a standard `material_override`. `simulation_mode` picks the
solver:

- **GPU** — a PhysX `PxDeformableSurface` (its own XPBD solver on CUDA). High
  vertex counts, proper draping and two-way rigid-body contact. Needs a
  `physx_gpu=yes` build and a CUDA device.
- **CPU** — a built-in extended position-based-dynamics solver (`cloth/`). No
  PhysX dependency, so it runs on any platform. Collision goes through
  `PhysicsDirectSpaceState3D` queries and works regardless of the active engine.

`Auto` (the default) uses the GPU path when it can and falls back to the CPU one
otherwise, transparently — the same node either way. `is_gpu_accelerated()`
reports which ran.

Pin an edge, the corners or explicit vertex indices with `pin_mode` /
`pinned_vertices`, or attach the pins to a moving `Node3D` with `anchor_path`.
Wind comes from an assigned `wind_area` (`Area3D`) plus a constant `wind` vector,
with `drag` / `lift` / `wind_turbulence` shaping the response. The node has a
viewport gizmo: the rest-grid outline with size handles, a marker on each pinned
vertex and a wind arrow.

## Soft bodies — stock `SoftBody3D`

The stock `SoftBody3D` node works on this backend (the `soft_body_*`
`PhysicsServer3D` API is implemented) — its gizmo for painting pinned vertices
and its inspector (`total_mass`, `pressure_coefficient`, `linear_stiffness`,
`simulation_precision`, `damping_coefficient`, `drag_coefficient`,
`shrinking_factor`) all apply, no module-specific node.

Each soft body resolves independently to one of two paths:

- **GPU** — a PhysX `PxDeformableVolume` (tetrahedral FEM on CUDA). Cooked from
  the render mesh with a conforming tet mesh, so the collision surface lines up
  with the render vertices and reads straight back each step. GPU volumes also
  collide with **each other** (firmly under the TGS solver; on PGS the contact
  is soft and transient — a stack slowly compacts).
- **CPU** — the same XPBD solver as CPU cloth (`cloth/`), over the welded render
  mesh. Edge constraints hold the shape; `pressure_coefficient > 0` adds a
  volume constraint that keeps a closed mesh from collapsing. Collision against
  rigid bodies is a per-vertex `PhysicsDirectSpaceState3D` query. Soft bodies do
  **not** collide with each other on this path (same as Jolt and Godot Physics).

`physics/physx_3d/soft_body/mode` picks the path: `Auto` (default — GPU when the
mesh tetrahedralizes and CUDA is present, else CPU, decided per body), `CPU`, or
`GPU`. A per-body override is the node metadata `physx_soft_mode` = `"cpu"` or
`"gpu"`. `enhanced_determinism` forces every soft body to CPU (no CUDA context).

## Chunk bursts — `PhysXChunkEmitter3D`

Call `spawn_at(position, direction)` — typically from a raycast hit — and a burst
of small rigid-body chunks flies out, bounces and settles. Real `PhysicsServer3D`
bodies, not a particle effect, so they land on slopes and pile up convincingly;
drawn as one `MultiMesh`. General-purpose: impact debris (the "shoot the ground
and chunks fly everywhere" effect from PhysX-sponsored titles of the GameWorks
era — Borderlands 2's debris system, for one), an exploding crate (`spread_degrees
= 180` scatters a burst in every direction instead of a cone), a rockslide or
falling debris (`emitting` + `emission_rate` for a continuous stream instead of a
one-off burst), confetti that actually collides — anything that wants many small
solid things flying and settling for real. `chunk_shape` picks box or sphere
chunks; `chunk_mesh` overrides the default box/sphere with any mesh.

Needs no PhysX-specific code (it talks to `PhysicsServer3D` generically, so it
works on any backend), but on this module with a `physx_gpu=yes` build and a
CUDA device it automatically rides the same GPU rigid-body dynamics as the rest
of the scene — the whole space is GPU-accelerated, not individual actors — which
is what makes a high `chunk_count` / `max_active` affordable. Lower them on the
CPU path, the same way PhysX-era games scaled debris down without a supporting
GPU.

`max_active` is a hard budget shared across every chunk this emitter has spawned,
burst or continuous: past it, the oldest chunks are freed to make room. `lifetime`
additionally recycles a chunk after it's been alive that long even under budget,
so chunks never linger forever.

## Determinism and multiplayer

- **GPU dynamics is never deterministic** — GPU solver scheduling varies run to
  run. Not usable for lockstep netcode or replays.
- **CPU simulation is deterministic only with `enhanced_determinism`** enabled,
  and only for the same binary on the same platform.

For deterministic lockstep multiplayer, use the Jolt backend.

## Known limitations

- **Joint chains under sustained force.** With the default `PGS` solver a chain
  of roughly three or more pin/6DOF joints will drift apart under a continuous
  external force such as area wind. Short chains, ragdolls and pendulums are
  fine; for longer wind-loaded chains switch `solver_type` to `TGS`, or add a
  `Generic6DOFJoint3D` linear/angular spring on each link to pull it back toward
  its rest pose (PhysX 5 removed joint projection, so a spring is the closest
  substitute).
- **Not yet implemented:** heightmap and separation-ray shapes; area-to-area
  detection (`Area3D` monitoring another `Area3D`); center-of-mass and inertia
  tensor overrides; 6DOF angular motors; joint softness / bias / restitution
  parameters. 6DOF linear and angular springs are supported (mapped onto PhysX
  joint drives).
  Unsupported shapes are treated as having no collision and log a warning once.
- **`PhysicalBone3D`** (physics-driven skeleton bones / ragdolls) simulates —
  bodies, joints and the per-step transform sync all work — but the joint
  softness / bias / relaxation parameters and `omit_force_integration` are not
  mapped, so joint stiffness can't be tuned and the animated-to-simulated blend
  is approximate. `SkeletonModifier3D` spring bones (`SpringBoneSimulator3D`,
  for hair and clothing) are engine-side and unaffected — they work identically
  on any backend.
- **Cylinder shapes** are approximated by a 16-sided convex prism.
- **Concave (trimesh) shapes** are supported on static and kinematic bodies
  only, as in most engines.
- **Cloth self-collision** is disabled; a cloth can pass through itself. Cloth
  tearing is not implemented.
- Windows x86-64 is the only platform wired up in the build script so far.

## Layout

| Path | Contents |
| --- | --- |
| `godot_physx_server_3d.*` | `PhysicsServer3D` implementation; owns the PhysX foundation, physics, CPU dispatcher and CUDA context |
| `godot_physx_project_settings.*` | registers and reads the `physics/physx_3d/*` settings |
| `godot_physx_conversions.h` | `Vector3` / `Quaternion` / `Transform3D` ↔ PhysX conversions |
| `objects/` | rigid bodies, areas, the GPU fluid and the GPU cloth surface |
| `shapes/` | collision shape wrappers and mesh cooking |
| `spaces/` | the `PxScene` wrapper, direct space/body state, area-override application |
| `joints/` | all `Joint3D` types |
| `cloth/` | the CPU (XPBD) cloth solver — no PhysX dependency |
| `nodes/` | `PhysXParticleFluid3D`, `PhysXCloth3D`, `PhysXChunkEmitter3D` |
| `editor/` | viewport gizmos for the fluid and cloth nodes |

## License

The module's own source is under the same MIT license as Godot Engine (see the
header of each file).

It links **NVIDIA PhysX 5** (<https://github.com/NVIDIA-Omniverse/PhysX>),
which is distributed under the BSD-3-Clause license — a full copy is in
[`PHYSX-LICENSE.md`](PHYSX-LICENSE.md). The PhysX SDK is *not* vendored here;
`SCsub` links it from an out-of-tree build pointed at by `physx_sdk=` /
`PHYSX_SDK`. The static PhysX libraries are compiled into the Godot binary, so
any binary you distribute must carry the PhysX copyright notice and disclaimer
(e.g. by shipping `PHYSX-LICENSE.md` alongside it or adding a stanza to the
engine's `COPYRIGHT.txt`).

With `physx_gpu=yes` the build also depends on `PhysXGpu_64.dll` (same PhysX
SDK, same BSD-3-Clause license, built from its GPU source) and, at runtime, on
an NVIDIA driver's CUDA library (`nvcuda.dll`) — the CUDA toolkit is only
needed to *build* the SDK, not to ship it.

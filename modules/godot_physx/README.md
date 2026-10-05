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

On **Linux**, on top of a normal Godot Linux editor build: **CMake**, **git**
and **clang** (the PhysX SDK is built with clang even when Godot itself is built
with GCC; the two link together fine). Tested with clang 22.

Win64 / MSVC is built and tested, CPU and GPU. Linux x86-64 is built and
tested for the CPU build only. The Linux **GPU** build (`--gpu` /
`physx_gpu=yes`) has a preset (`misc/physx_presets/linux64-godot-gpu.xml`) but
has not been run yet: whether Linux links `libPhysXGpu_64.so` like the Windows
import lib, needs an rpath/`LD_LIBRARY_PATH` entry instead, or nothing at link
time at all (pure `dlopen`) hasn't been checked, and `SCsub` doesn't copy the
`.so` next to the binary. The same applies to the Blast `.so` files
(`blast_sdk=`). The bundled `misc/physx_patches/` apply on Linux too; the two
GPU ones (heightfield GPU boundary crash, Turing `sm_75` SASS) are
platform-generic, but their effect is untested there for the same reason.

### Step 1 — build the PhysX SDK

```
python modules/godot_physx/misc/build_physx.py            # CPU only
python modules/godot_physx/misc/build_physx.py --gpu      # + GPU dynamics / fluid
```

`--platform` defaults to the host OS (`windows` on Windows, `linuxbsd`
elsewhere) — pass it explicitly to be sure, e.g. `--platform linuxbsd --gpu`.
Run it on the machine you're building for.

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
scons platform=linuxbsd target=editor physx_sdk=<path from step 1>           # Linux (CPU)
```

For a **GPU** build, `SCsub` also copies `PhysXGpu_64.dll` next to the built
binary automatically (a real SCons dependency, keyed on `physx_sdk=`'s own
DLL — it only re-copies when that changes, not on every build).

At startup a GPU build logs `PhysX: CUDA context ready on device '...'`; if no
usable CUDA device is found it warns and falls back to CPU simulation.

`build_physx.py` compiles native GPU code (SASS) for `sm_75` (Turing) through
`sm_120` (Blackwell), plus a PTX fallback the driver JITs for anything newer.
Older cards (Pascal `sm_61` and below) aren't targeted; widen the SASS list in
`misc/physx_patches/0002-gpu-turing-sm75-arch.patch` if you need one. A card
with no matching SASS still runs, but every kernel is JIT-compiled at load and
GPU dynamics is several times slower.

To confirm a stock build is unaffected, build with
`module_godot_physx_enabled=no` (or simply without an SDK configured).

### Optional — Blast SDK (`PhysXDestructible3D`)

Runtime mesh fracture/destruction (`PhysXDestructible3D`, the in-editor
fracture dialog) needs a second, separate SDK on top of PhysX: NVIDIA's
[NvBlast](https://github.com/NVIDIA-Omniverse/PhysX/tree/main/blast). It lives
in the *same* checkout `build_physx.py` clones for PhysX itself (a `blast/`
subdirectory of that monorepo, version-locked to the same release), builds
with its own premake5-based build system, and is entirely optional — the
module is skipped (again with a one-line notice) unless `blast_sdk=` is also
set, so a PhysX build without it is unaffected.

```
python modules/godot_physx/misc/build_physx.py --blast          # + CPU-only PhysX
python modules/godot_physx/misc/build_physx.py --gpu --blast    # + GPU PhysX
```

This builds PhysX as usual, then also runs Blast's own `build.bat`/`build.sh`
and prints the resulting SDK path plus the exact `scons` command for step 2.
Unlike PhysX's static libraries, Blast ships as DLLs (`NvBlast`,
`NvBlastGlobals`, `NvBlastExtAuthoring`, `NvBlastExtShaders`) — `SCsub` copies
all four next to the built binary automatically when `blast_sdk=` is set,
the same way it does for `PhysXGpu_64.dll` above.

```
scons platform=windows target=editor physx_sdk=<path> blast_sdk=<path from --blast>            # CPU
scons platform=windows target=editor physx_sdk=<path> physx_gpu=yes blast_sdk=<path from --blast>  # GPU
```

To build the SDK by hand instead: point scons at the Blast install directory
(the one containing `include/` and `bin/`) with `blast_sdk=<path>` or the
`BLAST_SDK` environment variable.

To confirm a Blast-less build still works, build with `physx_sdk=` set but no
`blast_sdk=` — `PhysXDestructible3D` and the fracture dialog simply won't be
registered.

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

`solver` picks the backend: `PBD (CUDA)` is the above; `MPM (compute)` is a
cross-vendor MLS-MPM fluid on plain `RenderingDevice` compute (no CUDA);
`Auto` uses PBD when a CUDA device is present, else MPM. The MPM path runs on
the engine's main render device and reads back asynchronously, so visuals and
the collider reaction land a few frames later.

The MPM fluid grid is **block-sparse and boundless** — it allocates only the
4³-cell blocks the fluid touches and follows the fluid as it flows, so
`mpm_domain_size` no longer confines it (it's now just the `spawn()` box, the
`mpm_auto_colliders` scan volume, and — via its Y — the implicit floor height).
Fluid runs off ledges and spreads freely; add a `WorldBoundaryShape3D` to
`mpm_colliders` or lower the domain for a true drop. The grid cell size is
`2 × particle_size` (`mpm_grid_resolution` is `PhysXGranular3D`-only — its dense
box grid still uses it). Cost is at parity with the old fixed-box grid.

Static and kinematic colliders couple cleanly; a **dynamic `RigidBody3D` in MPM
fluid has soft buoyancy** and may sink slowly or over-bounce — feed it through
`mpm_colliders` for the splash and drive real buoyancy from `get_submersion()`.
`PhysXGranular3D` (MPM sand/snow) runs the original dense box grid, unchanged.

GPU-only: the node is inert unless the active physics engine is PhysX and the
build has GPU/compute support (`PBD` additionally needs a CUDA device). See the
class reference for details.

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

## Granular — `PhysXGranular3D`

A `PhysXParticleFluid3D` subclass whose particles behave as grains — sand, gravel,
snow — rather than a liquid: they pile up, hold a slope up to `friction` (the
angle of repose, in degrees), and get plowed and cratered by bodies moving
through them. `density`, `hardness` and `grain_cohesion` tune the material;
emission, colliders, the domain and the sphere `MultiMesh` render are all
inherited. There is no isosurface.

Only the MPM compute solver holds a real angle of repose (PhysX PBD particle
friction cannot pile), so `solver = Auto` always resolves to MPM here — which
also means it runs on any GPU, no CUDA needed. Pick `PBD (CUDA)` explicitly only
for a pour or cascade that never has to hold a shape. Granular uses the dense
box grid, so unlike the fluid it *is* confined to `mpm_domain_size`.

## Smoke and gas — `PhysXGas3D`

A volumetric smoke/gas solver: a persistent velocity + density field on a
block-sparse grid, advected semi-Lagrangian, driven by `buoyancy` and vorticity
confinement, and made divergence-free with a Jacobi pressure projection. It runs
on plain compute shaders (no CUDA), independent of the PhysX GPU path. It is not
a particle system.

Injection points are `PhysXGasEmitter3D` nodes placed anywhere in the scene and
listed in `emitters`; each has its own shape (sphere or axis-aligned box),
`velocity` (rotated by the emitter's orientation), `density`, `divergence` and
`swirl`. `turbulence_strength` / `turbulence_scale` add divergence-free curl-noise
detail. Up to 4 `colliders` are voxelized as solids.

It renders through a stock `FogVolume` + `FogMaterial` (the density grid is baked
into an `ImageTexture3D` each step), so **`Environment.volumetric_fog_enabled`
must be on** or nothing draws — the node shows a configuration warning when it
isn't. `fire_look` swaps in an emissive hot-core-to-smoke colour ramp
(`fire_color_ramp`, `fire_emission_strength`) over the same density field.
`debug_point_cloud` is a fog-free fallback view.

Scope: the domain's X/Z footprint and floor are fixed where the node was when it
configured. Moving the node re-configures (and clears) the grid once it has been
still for 0.2 s, so it can be repositioned while authoring, but it is not meant
to follow a continuously moving object. The top of the box grows as a plume
rises, up to 3× `domain_size.y`.

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

## Destruction — `PhysXDestructible3D`

Real runtime mesh fracture via NVIDIA's [NvBlast](https://github.com/NVIDIA-Omniverse/PhysX/tree/main/blast)
(needs the separate `blast_sdk=` build — see Building above). Select a
`MeshInstance3D` and use **Mesh → Fracture with Blast** (or right-click a
`Mesh` resource in the FileSystem dock) to open a live in-viewport authoring
dialog: pick a fracture pattern, tune it, and Accept replaces the node with a
`PhysXDestructible3D` using the result.

Three fracture patterns: **Voronoi** (random cells, the default), **Slicing**
(brick-like, evenly spaced planes per axis), and **Cutout** (a caller-supplied
grayscale pattern texture extruded through the mesh — glass/tile-style
breaks; needs cracks that reach the image edge, like a real broken pane, or
the fracture is rejected before ever touching the native SDK).

While intact, a `PhysXDestructible3D` renders and collides as the whole
unfractured mesh. `apply_radial_damage(world_position, damage, min_radius,
max_radius)` breaks it explicitly (an explosion, a weapon hit) — each newly
detached chunk becomes its own rigid body with a cooked convex hull and a
`shatter_speed`-scaled outward kick from the damage origin. `dynamic = true`
additionally makes the intact object fall/collide like any other rigid body
and auto-fractures it from a hard enough physical impact
(`impact_strength`/`impact_damage_scale`), the way stacked destructible props
behave in Unreal's own Blast integration; `dynamic = false` (the default)
keeps it a fixed prop that only ever breaks from an explicit
`apply_radial_damage()` call. `mass` auto-computes from the mesh's volume
(uncheck `auto_mass` for a true override) and is distributed across split
pieces proportional to each one's own volume. `gi_mode` controls VoxelGI
static/dynamic baking per piece (defaults to Static, matching a plain
`MeshInstance3D`'s default — opt into Dynamic per-node, since a single
fracture can spawn many pieces at once and a VoxelGI's dynamic-object
tracking cost scales with how many it has to follow).

## Vehicles — `PhysXVehicle3D`, `PhysXMotorcycle3D`, `PhysXTank3D`

Vehicle nodes backed by PhysX's `PxVehicle2`. The stock `VehicleBody3D` /
`VehicleWheel3D` already work on this backend and remain the right choice for a
project that may switch engines; these nodes exist for what that model can't
offer: an engine torque response, Ackermann steering, a slip-based tire friction
curve (grip peaks at small slip and falls off when sliding), and per-wheel drive.

The node layout mirrors `VehicleBody3D`: a `CollisionShape3D` child with a
`BoxShape3D` for the chassis, plus `PhysXVehicleWheel3D` children. A wheel's own
`position` is where it rests under static load, so a mesh parented under it
follows the live suspension, steering and roll with no syncing code, and already
sits at the right height in the editor. Wheels carry the suspension
(`suspension_stiffness` / `_damping` / `_travel`) and tire
(`tire_rest_grip`, `tire_slide_grip`, stiffnesses) parameters.

- **`PhysXVehicle3D`** — 4 wheels, exactly 2 with `use_as_steering` (the front
  axle). Drive with `throttle` / `brake` (0..1), `steer` (−1..1) and `reverse`.
  Front/rear anti-roll bars, `ackermann_strength`, custom `center_of_mass`.
- **`PhysXMotorcycle3D`** — 2 wheels, front steering, rear traction. `PxVehicle2`
  has no balance mechanism, so staying upright is the script's job: read
  `get_roll_angle()` / `get_angular_velocity()` each tick and correct with
  `apply_torque_impulse()`, like a rider would.
- **`PhysXTank3D`** — 2–16 wheels, skid-steer. Each wheel is assigned to the left
  or right track from the sign of its local X. Drive with signed `left_ratio` /
  `right_ratio` (−1..1): the sign is the direction, so opposite signs pivot in
  place without a gear change.

All three sleep when parked (`can_sleep`) and only simulate in a running game,
never in the editor.

## Water — `PhysXWaterSurface3D`

An animated water surface with two GPU compute layers summed into one displaced
mesh (created automatically as a child): a Tessendorf FFT ocean with a
fetch-limited JONSWAP wind spectrum (`wind_speed`, `wind_direction`, `fetch`,
over `ocean_domain_size`; heights in meters at `wave_amplitude` 1) and a local
ripple layer — a damped shallow-water wave equation over `domain_size` whose
wave speed follows `depth` — that bodies disturb.

The water is a square by default. Set `surface_mesh` to any flat mesh (a disc
baked from `CSGCylinder3D`, a kidney bean, a lake with an island) and its X/Z
footprint becomes the water: the ripple grid fits itself to the outline, waves
reflect off the shoreline, and the rendered surface is the footprint resampled
into an even grid, so a coarse or fan-triangulated mesh still animates
everywhere. With `fetch` left at 0 the footprint also sets the fetch, so an
enclosed pool only gets small, short wind ripples while an open square is a
developed sea. The simulation is centered on the node's position; keep the node
unrotated and unscaled.

`choppiness` adds the ocean waves' horizontal displacement, so crests come
to a point and troughs flatten. Where it squeezes the surface past
`foam_threshold` whitecap foam forms; it's kept with the water, not the wave,
and fades over `foam_persistence` seconds, so crests leave streaks that break
into lace. A strong wind gets whitecaps and a light one doesn't. `normal_mode` picks how normals and foam are shaded: Per Pixel
(default) uses exact slopes from the FFT, so crests stay smooth at any mesh
density; Per Vertex follows the mesh, for a softer or faceted, stylized look.

Set `seabed_from_floor` for water over uneven ground, like a beach or a lake
bed: each ripple cell's depth is measured once from the floor (static bodies in
`seabed_collision_mask`). Waves then travel at the local shallow-water speed,
slowing over shallows, the shoreline falls wherever the ground rises above the
water, and the ocean chop fades out over the last `shallow_fade_depth` meters.
Without it, the constant `depth` applies everywhere. On a beach, each arriving
crest also leaves a thin foam sheet at the waterline (`shore_foam_band`) that
surges in with the wave and slides back out with the backwash and undertow
(`shore_undertow`), the shallows turn milky, and the surface feathers out onto
the sand along a waterline that scallops with the arriving waves rather than
following the depth contour. That edge is measured in meters of water
(`swash_reach`, `shore_edge_softness` in the material), so it behaves the same
whatever `shallow_fade_depth` is. The bigger waves also run up past the
still-water line as a thin film on the sand (`swash_run_up`,
`swash_drain_speed`), and the sand they cover stays dark and glossy while it
dries (`wet_sand_dry_time`). The material's `shore_detail_scale` sets how fine
the swash edge, the rim lace and the wet-sand line are. The shore patterns
mirror-repeat past the simulated square, so the beach keeps its variation all
along the shore.

For open water, `render_extent` renders the surface out to the horizon: the FFT
ocean tiles seamlessly past the simulated square and ripples settle to still
water there. The surface draws only the side facing the camera (the top from
above, the underside from below), and its shaders take their clock from the
node's `water_time` parameter rather than `TIME`, so an open scene doesn't
keep the editor redrawing (and competing with Play Scene for the GPU).

`sample_height(position)` returns the same combined surface height on the CPU,
for buoyancy scripts. Over dry cells (outside a `surface_mesh` footprint, or
where the seabed is above the water) it returns `-INF` and `is_wet()` is false,
so floaters on dry land don't float. Call `submit_sphere()` once per physics tick per floating
body so it actually pushes the water (`ripple_amplitude` scales the wake), or
`submit_impulse()` for one-off splashes.

`caustics_enabled` renders a light-space caustic map with a direct
`RenderingDevice` draw pass along `caustics_sun_direction`. Any material — the
floor, walls, submerged props — can sample it by projecting its world position
with `get_caustics_texture()`, `get_caustics_origin()`,
`get_caustics_light_right()` / `_up()` and `get_caustics_half_extent()`, so the
light pattern lands on anything under the water rather than only a flat floor.
The map's green channel marks where sunlight actually came through the water
(multiply by it, or a pool wall's outside picks up caustics), and the pattern
repeats every `ocean_domain_size` (`get_caustics_tile_size()`), so a receiver
can fold its position into one tile and light a whole seabed.

Runs on any GPU with compute support, no CUDA needed. Without compute (e.g.
headless) the surface stays flat and `sample_height()` returns `water_level`.

## Boats — `PhysXBuoyancy3D`, `PhysXBoat3D`, `PhysXWaterWake3D`, `PhysXWaterSpray3D`

Four nodes, each a child of a `RigidBody3D`, that float and drive it on a
`PhysXWaterSurface3D`. Buoyancy and the boat only apply `RigidBody3D` forces,
so they work with whichever physics engine is selected, Jolt included.

- **`PhysXBuoyancy3D`** floats the body: sample points on the hull (six from
  its collision shapes' bounds by default) each carry a share of `hull_area`
  and are pushed up by the water they displace, so the body settles at its
  real draft, pitches and rolls with the waves and rights itself. It reads the
  water with `sample_height()`, refreshed every physics tick, waves and other
  bodies' wakes included.
- **`PhysXBoat3D`** drives it: thrust from a propeller at `propeller_position`
  (only while it's in the water) that `steering` turns like an outboard, the
  water across the propeller's leg (it steers with flow and holds the stern on
  course), and hull drag in the boat's frame — low along the keel, high
  sideways and vertically, at the bow and stern so the hull's length resists
  turning and pitching. `hull_drag.z` with `max_thrust` sets the top speed.
- **`PhysXWaterWake3D`** leaves a wake: a small GPU ripple grid that slides
  with the boat (fixed to the water, so the wake stays where it was made) and
  that the hull pushes down, plus foam — two bands peeling off the hull's
  sides and a propeller band down the middle, fading over `foam_persistence`.
  The water material draws it and `sample_height()` includes it, so other
  floaters ride it. A grid big enough to hold a turning circle (`size`, with
  `cells` for about 0.25 m cells) and a `foam_persistence` of about a lap let
  the wake close into a ring.
- **`PhysXWaterSpray3D`** drives ordinary `GPUParticles3D` nodes authored in
  the editor: bow emitters with speed, stern emitters with thrust, one-shot
  slam emitters when the hull drops back onto the water. Emitters are held at
  the water under their place on the hull. The spray leaves foam where it
  lands (worked out from the emitters' launch geometry, no particle collision
  needed); `water_collision` adds a collider on the water for spray that
  should land and skid. Save the emitters with `emitting` off — the node
  starts them — or the editor redraws every frame for them.

The demo boat is `demo/common/boat/boat.tscn`, on the beach.

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
- **Not yet implemented:** 6DOF angular motors; joint softness / bias /
  restitution parameters.
- **`SeparationRayShape3D`** has no PhysX geometry: it's cast as a ray. In
  `move_and_slide` it lifts a character until its tip sits on what it hits
  (stairs), as on Godot Physics and Jolt. On a rigid body it's applied before
  each step as a frictional contact at the tip, not inside the PhysX solver,
  so a body on rays settles a little differently from Jolt (it loses more
  energy crossing bumps). 6DOF linear and angular springs are supported (mapped onto PhysX
  joint drives). Unsupported shapes are treated as having no collision and log
  a warning once.
- **Area-to-area detection** (`Area3D` monitoring another `Area3D`) works, but
  unlike every other collision pair in this module it costs real per-step
  work: PhysX never reports trigger-trigger pairs (only trigger-vs-rigid), so
  `GodotPhysXSpace3D` polls it manually every step with a naive O(n²) pass
  over every `(monitoring, monitorable)` area pair, each a real shape-vs-shape
  `PxGeometryQuery::overlap()` test. Both `monitoring` and `monitorable`
  default to `true` on every `Area3D`, so this runs for every area pair by
  default unless a scene explicitly opts areas out with `monitorable = false`.
  Fine for the handful of areas a scene typically wants this on; revisit with
  a broad-phase pre-filter if a scene ever has many mutually-monitoring areas.
- **`HeightMapShape3D`** works — a `PxHeightField` quantized to 16 bits over the
  map's height range (so vertical resolution is `(max_height − min_height) /
  65535`). Like concave (trimesh) shapes, it is static/kinematic only. The GPU
  build requires the bundled PhysX patch
  (`misc/physx_patches/0001-heightfield-gpu-boundary-crash.patch`, upstream
  [PR #503](https://github.com/NVIDIA-Omniverse/PhysX/pull/503)) — without it a
  body straddling the height-field boundary faults the GPU narrowphase and kills
  the CUDA context. `build_physx.py` applies it automatically.
- **Node scale** is baked into the collision geometry when the actor is built
  (PhysX actor poses carry no scale): box per-axis, convex and trimesh via
  `PxMeshScale`, height field via row/column/height scale — full non-uniform
  scale on all of those, like Jolt. **Sphere and capsule are uniform-only**;
  non-uniform scale on them collapses to the mean axis and logs a warning once
  (a PhysX capsule/sphere can't be an ellipsoid), again matching Jolt. Changing
  a body's scale at runtime re-cooks its shapes.
- **`PhysicalBone3D`** (physics-driven skeleton bones / ragdolls) simulates —
  bodies, joints, `omit_force_integration` (Custom Integrator bones / active
  ragdolls) and the per-step transform sync all work. The Bullet-era joint
  softness / bias / relaxation / ERP parameters are ignored (as they are on
  Jolt), so joint stiffness itself can't be dialed in; PhysX's `PxD6` cone
  defaults sit a little softer than Jolt's. `SkeletonModifier3D` spring bones
  (`SpringBoneSimulator3D`,
  for hair and clothing) are engine-side and unaffected — they work identically
  on any backend.
- **Cylinder shapes** are approximated by a 16-sided convex prism.
- **Concave (trimesh) shapes** are supported on static and kinematic bodies
  only, as in most engines. They collide on their front faces only unless
  `backface_collision` is on, as on Jolt; with it on, the mesh is cooked with a
  flipped copy of every triangle (PhysX has no two-sided contact option), so
  it takes twice the memory and triangle tests. One difference from Jolt:
  overlap queries (`intersect_shape`, `collide_shape`, `get_rest_info`) see a
  one-sided mesh from behind too.
- **Cloth self-collision** is disabled; a cloth can pass through itself. Cloth
  tearing is not implemented. `PhysXCloth3D` pins follow a single shared
  `anchor_path`, so there is no per-vertex bone attachment yet.
- Windows and Linux x86-64 are the only platforms built and tested, and on
  Linux only the CPU build — see [Building](#building) for what's unverified
  about a Linux GPU build.

## Layout

| Path | Contents |
| --- | --- |
| `godot_physx_server_3d.*` | `PhysicsServer3D` implementation; owns the PhysX foundation, physics, CPU dispatcher and CUDA context |
| `godot_physx_project_settings.*` | registers and reads the `physics/physx_3d/*` settings |
| `godot_physx_conversions.h` | `Vector3` / `Quaternion` / `Transform3D` ↔ PhysX conversions |
| `objects/` | rigid bodies, areas, the GPU fluid, the GPU cloth surface and GPU soft bodies |
| `shapes/` | collision shape wrappers and mesh cooking |
| `spaces/` | the `PxScene` wrapper, direct space/body state, area-override application |
| `joints/` | all `Joint3D` types |
| `cloth/` | the CPU (XPBD) cloth solver — no PhysX dependency |
| `nodes/` | `PhysXParticleFluid3D`, `PhysXGranular3D`, `PhysXGas3D`, `PhysXGasEmitter3D`, `PhysXCloth3D`, `PhysXChunkEmitter3D` |
| `particles/` | the MPM fluid/granular and gas compute solvers and their GLSL shaders |
| `vehicle/` | `PhysXVehicle3D`, `PhysXMotorcycle3D`, `PhysXTank3D`, `PhysXVehicleWheel3D` and the `PxVehicle2` glue |
| `water/` | `PhysXWaterSurface3D` and its ripple / FFT ocean / caustics compute and draw passes; `PhysXBuoyancy3D`, `PhysXBoat3D`, `PhysXWaterWake3D` (and its wake compute pass) and `PhysXWaterSpray3D` |
| `blast/` | `PhysXDestructible3D`, `PhysXBlastAsset`, and the NvBlast fracture-authoring bridge — optional, gated on `blast_sdk=` (see Building above) |
| `editor/` | viewport gizmos for the fluid and cloth nodes; the Blast fracture dialog and its FileSystem/Inspector plugins |

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

With `blast_sdk=` set, the build also links **NvBlast**
(<https://github.com/NVIDIA-Omniverse/PhysX/tree/main/blast>) — the same
`NVIDIA-Omniverse/PhysX` repository as PhysX itself, so the same
BSD-3-Clause license in `PHYSX-LICENSE.md` covers it too. Unlike PhysX's
static libraries, Blast ships as DLLs (`NvBlast`, `NvBlastGlobals`,
`NvBlastExtAuthoring`, `NvBlastExtShaders`) that must ship next to the Godot
binary — see Building above.

# PhysX source patches

`build_physx.py` applies every `*.patch` here (sorted by name) to the PhysX
checkout after clone/locate and before project generation. Each is a `-p1` diff
rooted at the repo (`a/physx/...`), and is skipped cleanly if already applied.

These are fixes the pinned `PHYSX_REF` (`ovphysx-0.5.11`) doesn't have, either
upstream fixes that landed later or our own build fixes. Drop a patch once the
ref is bumped past it.

| Patch | Upstream | Fixes |
| --- | --- | --- |
| `0001-heightfield-gpu-boundary-crash.patch` | [PR #503](https://github.com/NVIDIA-Omniverse/PhysX/pull/503) / [issue #502](https://github.com/NVIDIA-Omniverse/PhysX/issues/502) | GPU sphere–height-field narrowphase reads ~8 GB out of bounds at a height-field rim triangle (the `BOUNDARY` adjacency sentinel is passed to `getTriangle()` unchecked) → `CUDA_ERROR_ILLEGAL_ADDRESS`, dead CUDA context. Latent until the bad address is unmapped, which a renderer sharing the GPU makes reliable. |
| `0002-gpu-turing-sm75-arch.patch` | none (build config) | The GPU arch list has no SASS for `sm_75` (Turing: RTX 20-series / GTX 16-series). Those cards JIT every PhysX CUDA kernel from PTX at load and run GPU dynamics ~8× slower. Adds `sm_75` to the SASS lists. |
| `0003-linux-clang-no-werror.patch` | none (build config) | The Linux clang build uses `-Werror -Weverything`, so every clang release that adds a warning breaks the SDK build (clang 22: `-Wmissing-include-dirs`, `-Wnrvo`, `-Wformat-signedness`). Drops `-Werror`; the warnings are still printed. |
| `0004-cuda13-toolkit.patch` | none (build config) | Builds and loads with the CUDA 13 toolkit (new `cuCtxCreate`, plus stubs for two new launch functions). Only kicks in on CUDA 13, 12.x builds don't change. |
| `0005-blast-linux-host-gcc.patch` | none (build config) | Builds Blast on Linux with a normal GCC instead of NVIDIA's GCC 9 container. No `-Werror`, three missing includes, `-fnew-inheriting-ctors` for GCC 13, `-no-pie` for the tests, and `-fno-strict-aliasing` since Blast casts between its vector types and optimized builds hang in fracture authoring without it. |
| `0006-blast-cutout-double-free.patch` | none (not reported yet) | Blast's cutout code runs a vector's destructor by hand and then deletes it, so it's freed twice. MSVC gets away with it, on Linux cutout fracturing crashes. |
| `0007-flow-linux-arch.patch` | none (build config) | Flow's `build.sh` uses the `arch` command, which some distros (Arch) don't have. `uname -m` gives the same thing everywhere. |

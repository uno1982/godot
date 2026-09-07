# PhysX source patches

`build_physx.py` applies every `*.patch` here (sorted by name) to the PhysX
checkout after clone/locate and before project generation. Each is a `-p1` diff
rooted at the repo (`a/physx/...`), and is skipped cleanly if already applied.

These are fixes that landed upstream *after* the pinned `PHYSX_REF`
(`ovphysx-0.5.11`). Drop a patch once the ref is bumped past it.

| Patch | Upstream | Fixes |
| --- | --- | --- |
| `0001-heightfield-gpu-boundary-crash.patch` | [PR #503](https://github.com/NVIDIA-Omniverse/PhysX/pull/503) / [issue #502](https://github.com/NVIDIA-Omniverse/PhysX/issues/502) | GPU sphere–height-field narrowphase reads ~8 GB out of bounds at a height-field rim triangle (the `BOUNDARY` adjacency sentinel is passed to `getTriangle()` unchecked) → `CUDA_ERROR_ILLEGAL_ADDRESS`, dead CUDA context. Latent until the bad address is unmapped, which a renderer sharing the GPU makes reliable. |

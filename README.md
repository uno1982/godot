# Godot Engine

<p align="center">
  <a href="https://godotengine.org">
    <img src="misc/logo/logo_outlined.svg" width="400" alt="Godot Engine logo">
  </a>
</p>

---

## ⚡ NVIDIA Path Tracing + DLSS Branch

This branch (`nvidia-pt-dlss`) is based on [NVIDIA-RTX/godot](https://github.com/NVIDIA-RTX/godot/tree/nvidia-pt-dlss) and adds **hardware-accelerated path tracing** and **DLSS** support for NVIDIA RTX GPUs.

### Requirements

- **NVIDIA RTX GPU** (RTX 20 series or newer)
- **Latest NVIDIA drivers** (recommend 550+)
- **Vulkan SDK** installed
- **Streamline SDK** (required for DLSS features)

### Setup Instructions

1. **Build Godot** from this branch:
   ```bash
   scons platform=windows target=editor -j12
   ```

2. **Download the Streamline SDK** from:  
   https://github.com/NVIDIAGameWorks/Streamline/releases

3. **Copy these DLLs** from Streamline SDK `bin/x64/` to your Godot `bin/` folder:
   ```
   sl.interposer.dll
   sl.common.dll
   sl.dlss.dll
   sl.dlss_d.dll
   sl.dlss_g.dll
   nvngx_dlss.dll
   nvngx_dlssd.dll
   nvngx_dlssg.dll
   ```

4. **Enable Path Tracing** in your project:
   - Use **Forward+** rendering method (default)
   - Add a **WorldEnvironment** node with an **Environment** resource
   - In the Environment, scroll to **Path Tracing** section and enable it

### Upstream

- Original NVIDIA branch: https://github.com/NVIDIA-RTX/godot/tree/nvidia-pt-dlss
- Official Godot: https://github.com/godotengine/godot

---

## 2D and 3D cross-platform game engine

**[Godot Engine](https://godotengine.org) is a feature-packed, cross-platform
game engine to create 2D and 3D games from a unified interface.** It provides a
comprehensive set of [common tools](https://godotengine.org/features), so that
users can focus on making games without having to reinvent the wheel. Games can
be exported with one click to a number of platforms, including the major desktop
platforms (Linux, macOS, Windows), mobile platforms (Android, iOS), as well as
Web-based platforms and [consoles](https://godotengine.org/consoles).

## Free, open source and community-driven

Godot is completely free and open source under the very permissive [MIT license](https://godotengine.org/license).
No strings attached, no royalties, nothing. The users' games are theirs, down
to the last line of engine code. Godot's development is fully independent and
community-driven, empowering users to help shape their engine to match their
expectations. It is supported by the [Godot Foundation](https://godot.foundation/)
not-for-profit.

Before being open sourced in [February 2014](https://github.com/godotengine/godot/commit/0b806ee0fc9097fa7bda7ac0109191c9c5e0a1ac),
Godot had been developed by [Juan Linietsky](https://github.com/reduz) and
[Ariel Manzur](https://github.com/punto-) for several years as an in-house
engine, used to publish several work-for-hire titles.

![Screenshot of a 3D scene in the Godot Engine editor](https://raw.githubusercontent.com/godotengine/godot-design/master/screenshots/editor_tps_demo_1920x1080.jpg)

## Getting the engine

### Binary downloads

Official binaries for the Godot editor and the export templates can be found
[on the Godot website](https://godotengine.org/download).

### Compiling from source

[See the official docs](https://docs.godotengine.org/en/latest/engine_details/development/compiling)
for compilation instructions for every supported platform.

## Community and contributing

Godot is not only an engine but an ever-growing community of users and engine
developers. The main community channels are listed [on the homepage](https://godotengine.org/community).

The best way to get in touch with the core engine developers is to join the
[Godot Contributors Chat](https://chat.godotengine.org).

To get started contributing to the project, see the [contributing guide](CONTRIBUTING.md).
This document also includes guidelines for reporting bugs.

## Documentation and demos

The official documentation is hosted on [Read the Docs](https://docs.godotengine.org).
It is maintained by the Godot community in its own [GitHub repository](https://github.com/godotengine/godot-docs).

The [class reference](https://docs.godotengine.org/en/latest/classes/)
is also accessible from the Godot editor.

We also maintain official demos in their own [GitHub repository](https://github.com/godotengine/godot-demo-projects)
as well as a list of [awesome Godot community resources](https://github.com/godotengine/awesome-godot).

There are also a number of other
[learning resources](https://docs.godotengine.org/en/latest/community/tutorials.html)
provided by the community, such as text and video tutorials, demos, etc.
Consult the [community channels](https://godotengine.org/community)
for more information.

[![Code Triagers Badge](https://www.codetriage.com/godotengine/godot/badges/users.svg)](https://www.codetriage.com/godotengine/godot)
[![Translate on Weblate](https://hosted.weblate.org/widgets/godot-engine/-/godot/svg-badge.svg)](https://hosted.weblate.org/engage/godot-engine/?utm_source=widget)

#!/usr/bin/env python3
"""Build the PhysX 5 SDK for linking into the godot_physx module.

The module links against a PhysX SDK built out of tree with the static CRT.
This script clones NVIDIA's PhysX repository at a pinned revision, drops in a
build preset tuned to match Godot, runs PhysX's own project generation and
CMake build, and prints the path to pass to scons as physx_sdk=.

    python modules/godot_physx/misc/build_physx.py [--gpu]
    scons platform=windows target=editor physx_sdk=<printed path> [physx_gpu=yes]

The GPU build additionally needs the CUDA Toolkit installed (CUDA_PATH set) and
copies nothing automatically -- PhysXGpu_64.dll from the install's bin/ must sit
next to the Godot binary at runtime.

Run this script on the machine you're building the module for -- --platform
defaults to the host OS (windows / linuxbsd). The Linux presets
(linux64-godot[-gpu].xml) are UNVERIFIED: nobody has run this script or built
the module on Linux yet, only prepared it. If generate_projects.sh, the CMake
build, or the later scons link step fails, see each preset file's header
comment and README.md's Linux section for the specific unknowns to check
first (preset platform/compiler naming, install bin/ layout, GPU link
mechanism) before assuming something else is wrong.
"""

import argparse
import os
import shutil
import subprocess
import sys

# Pinned so a given godot_physx revision always builds against the same SDK.
PHYSX_REPO = "https://github.com/NVIDIA-Omniverse/PhysX.git"
PHYSX_REF = "ovphysx-0.5.11"  # PhysX SDK 5.10.0

HERE = os.path.dirname(os.path.abspath(__file__))
PRESET_DIR = os.path.join(HERE, "physx_presets")
PATCH_DIR = os.path.join(HERE, "physx_patches")


def run(cmd, cwd):
    print("+ " + " ".join(cmd) + "  (in %s)" % cwd)
    subprocess.check_call(cmd, cwd=cwd)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gpu", action="store_true", help="also build the CUDA GPU projects (needs the CUDA Toolkit)")
    ap.add_argument(
        "--platform",
        choices=["windows", "linuxbsd"],
        default="windows" if os.name == "nt" else "linuxbsd",
        help="target platform / scons platform= value (default: autodetected from the host)",
    )
    ap.add_argument(
        "--src",
        metavar="DIR",
        help="PhysX checkout to build in; cloned here if absent (default: a 'physx-sdk' directory beside the Godot repo)",
    )
    ap.add_argument("--ref", default=PHYSX_REF, help="git ref to check out when cloning (default: %(default)s)")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4, help="parallel build jobs")
    ap.add_argument("--config", default="release", choices=["release", "checked", "profile", "debug"])
    args = ap.parse_args()

    preset_base = "vc17win64-godot" if args.platform == "windows" else "linux64-godot"
    preset = preset_base + "-gpu" if args.gpu else preset_base
    preset_file = os.path.join(PRESET_DIR, preset + ".xml")
    if not os.path.isfile(preset_file):
        sys.exit("missing preset: " + preset_file)
    if args.platform == "linuxbsd":
        print(
            "NOTE: the %s preset is unverified -- no Linux build of this module has "
            "been run yet. See its header comment and README.md's Linux section for "
            "the specific unknowns (preset platform/compiler naming, install bin/ "
            "layout, GPU link mechanism) if generate_projects.sh or the CMake build "
            "fails here." % preset
        )

    if args.gpu and not (os.environ.get("CUDA_PATH") or shutil.which("nvcc")):
        sys.exit("--gpu needs the CUDA Toolkit (set CUDA_PATH or put nvcc on PATH)")

    # Locate / create the PhysX source tree.
    src = args.src
    if not src:
        # Beside the Godot repo, not inside it -- SDK source is not part of the tree.
        repo_root = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
        src = os.path.join(os.path.dirname(repo_root), "physx-sdk")
    src = os.path.abspath(src)

    if not os.path.isdir(os.path.join(src, ".git")):
        os.makedirs(os.path.dirname(src), exist_ok=True)
        run(["git", "clone", "--depth", "1", "--branch", args.ref, PHYSX_REPO, src], cwd=os.path.dirname(src) or ".")
    else:
        print("using existing PhysX checkout: " + src)

    physx = os.path.join(src, "physx")
    if not os.path.isdir(physx):
        sys.exit("no physx/ directory in " + src)

    # Apply bundled patches (fixes not yet in the pinned PhysX ref). Each is a
    # -p1 diff rooted at the checkout; skipped cleanly if already applied.
    if os.path.isdir(PATCH_DIR):
        for name in sorted(f for f in os.listdir(PATCH_DIR) if f.endswith(".patch")):
            patch = os.path.join(PATCH_DIR, name)
            if subprocess.call(["git", "apply", "--reverse", "--check", patch], cwd=src,
                               stderr=subprocess.DEVNULL) == 0:
                print("patch already applied: " + name)
                continue
            if subprocess.call(["git", "apply", "--check", patch], cwd=src,
                               stderr=subprocess.DEVNULL) != 0:
                sys.exit("bundled patch does not apply (wrong PhysX ref?): " + name)
            run(["git", "apply", patch], cwd=src)
            print("applied patch: " + name)

    # Install the Godot-tuned preset.
    shutil.copy2(preset_file, os.path.join(physx, "buildtools", "presets", "public", preset + ".xml"))

    is_windows = os.name == "nt"
    gen = "generate_projects.bat" if is_windows else "generate_projects.sh"
    gen_path = os.path.join(physx, gen)
    run([gen_path, preset] if is_windows else ["bash", gen_path, preset], cwd=physx)

    build_dir = os.path.join(physx, "compiler", preset)
    if not os.path.isdir(build_dir):
        sys.exit("project generation did not produce " + build_dir)
    run(
        ["cmake", "--build", build_dir, "--config", args.config, "--target", "install", "--parallel", str(args.jobs)],
        cwd=physx,
    )

    sdk = os.path.join(physx, "install", preset, "PhysX")
    if not os.path.isdir(os.path.join(sdk, "include")):
        sys.exit("build finished but no SDK at " + sdk)

    print()
    print("PhysX SDK ready:")
    print("    " + sdk)
    print()
    print("Build the module with:")
    print(
        "    scons platform=%s target=editor physx_sdk=%s%s"
        % (args.platform, sdk.replace("\\", "/"), " physx_gpu=yes" if args.gpu else "")
    )
    if args.gpu:
        if args.platform == "windows":
            dll = os.path.join(sdk, "bin", "win.x86_64.vc143.mt", args.config, "PhysXGpu_64.dll")
            print()
            print("Then copy the GPU runtime next to the Godot binary:")
            print("    copy %s bin\\" % dll)
        else:
            # UNVERIFIED bin/ subdirectory name -- see linux64-godot-gpu.xml's
            # header comment; check the actual install output and adjust.
            so = os.path.join(sdk, "bin", "linux.clang.x86_64", args.config, "libPhysXGpu_64.so")
            print()
            print("Then copy the GPU runtime next to the Godot binary (path above is a guess -- verify it):")
            print("    cp %s bin/" % so)


if __name__ == "__main__":
    main()

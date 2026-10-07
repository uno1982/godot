#!/usr/bin/env python3
"""Build the PhysX 5 SDK for linking into the godot_physx module.

The module links against a PhysX SDK built out of tree with the static CRT.
This script clones NVIDIA's PhysX repository at a pinned revision, drops in a
build preset tuned to match Godot, runs PhysX's own project generation and
CMake build, and prints the path to pass to scons as physx_sdk=.

    python modules/godot_physx/misc/build_physx.py [--gpu] [--blast] [--flow]
    scons platform=windows target=editor physx_sdk=<printed path> [physx_gpu=yes] [blast_sdk=<printed path>] [flow_sdk=<printed path>]

The GPU build additionally needs the CUDA Toolkit installed (CUDA_PATH set).
PhysXGpu_64.dll must sit next to the Godot binary at runtime -- on Windows,
SCsub copies it there automatically as part of the scons build printed below
(a real SCons dependency, not a one-time thing you have to remember); on
Linux this is still unverified/manual, see README.md's Linux section.

--blast additionally builds the Blast SDK (runtime mesh fracture/destruction,
NvBlast + extensions, backing PhysXDestructible3D and its in-editor fracture
dialog) from this same checkout's blast/ subdirectory -- it's version-locked
to the same NVIDIA-Omniverse/PhysX release train as PhysX itself, so no
separate clone or ref pin is needed. Unlike PhysX, Blast ships as DLLs, not
static libs; on Windows, SCsub copies all four next to the Godot binary
automatically too, same mechanism as PhysXGpu_64.dll -- on Linux this is
also still unverified/manual.

--flow additionally builds NVIDIA Flow (sparse-grid smoke / fire / dust,
backing PhysXFlow3D) from the same checkout's flow/ subdirectory, with its own
build.bat / build.sh. flow_sdk= is that flow/ directory itself. Nothing is
linked -- nvflow / nvflowext are loaded at run time; on Windows SCsub copies
both DLLs next to the Godot binary, on Linux it doesn't yet.

The last thing printed is the scons command for everything built.

Run this script on the machine you're building the module for -- --platform
defaults to the host OS (windows / linuxbsd). The Linux GPU preset
(linux64-godot-gpu.xml) is UNVERIFIED -- see its header comment and
README.md's Linux section.
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
        "--blast",
        action="store_true",
        help="also build the Blast SDK (NvBlast + extensions) from this same checkout's blast/ subdirectory",
    )
    ap.add_argument(
        "--flow",
        action="store_true",
        help="also build NVIDIA Flow (nvflow + nvflowext) from this same checkout's flow/ subdirectory",
    )
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
    if args.platform == "linuxbsd" and args.gpu:
        print(
            "NOTE: the %s preset is unverified -- no Linux GPU build of this module has "
            "been run yet. See its header comment and README.md's Linux section." % preset
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
            if (
                subprocess.call(["git", "apply", "--reverse", "--check", patch], cwd=src, stderr=subprocess.DEVNULL)
                == 0
            ):
                print("patch already applied: " + name)
                continue
            if subprocess.call(["git", "apply", "--check", patch], cwd=src, stderr=subprocess.DEVNULL) != 0:
                sys.exit("bundled patch does not apply (wrong PhysX ref?): " + name)
            run(["git", "apply", patch], cwd=src)
            print("applied patch: " + name)

    # Install the Godot-tuned preset.
    shutil.copy2(preset_file, os.path.join(physx, "buildtools", "presets", "public", preset + ".xml"))

    is_windows = os.name == "nt"
    gen = "generate_projects.bat" if is_windows else "generate_projects.sh"
    gen_path = os.path.join(physx, gen)
    run([gen_path, preset] if is_windows else ["bash", gen_path, preset], cwd=physx)

    # Visual Studio is a multi-config generator (one build dir, config picked at
    # build time); on Linux PhysX generates one Makefile build dir per config.
    build_dir = os.path.join(physx, "compiler", preset if is_windows else preset + "-" + args.config)
    if not os.path.isdir(build_dir):
        sys.exit("project generation did not produce " + build_dir)
    run(
        ["cmake", "--build", build_dir, "--config", args.config, "--target", "install", "--parallel", str(args.jobs)],
        cwd=physx,
    )

    sdk = os.path.join(physx, "install", preset, "PhysX")
    if not os.path.isdir(os.path.join(sdk, "include")):
        sys.exit("build finished but no SDK at " + sdk)

    # Extra scons options from each optional SDK, printed as one command at the end.
    scons_args = ["physx_sdk=" + sdk.replace("\\", "/")]
    if args.gpu:
        scons_args.append("physx_gpu=yes")

    print()
    print("PhysX SDK ready:")
    print("    " + sdk)
    if args.gpu:
        if args.platform == "windows":
            print()
            print("SCsub copies PhysXGpu_64.dll next to the Godot binary automatically -- nothing more to do.")
        else:
            # UNVERIFIED whether Linux even needs a copied .so the way Windows
            # needs the DLL (vs. an rpath/LD_LIBRARY_PATH entry, or a pure
            # dlopen with no on-disk convention to match) -- SCsub does not
            # auto-copy anything here yet.
            so = os.path.join(sdk, "bin", "linux.x86_64", args.config, "libPhysXGpu_64.so")
            print()
            print("Then copy the GPU runtime next to the Godot binary (unverified -- see README.md's Linux section):")
            print("    cp %s bin/" % so)

    if args.blast:
        # Blast lives in the SAME checkout as PhysX (NVIDIA-Omniverse/PhysX's
        # own blast/ subdirectory), version-locked to the same release train --
        # no separate clone or ref to manage. It has its own build system
        # (premake5 via a bundled `repo` tool), independent of PhysX's CMake
        # build above, and always builds its "release" config by default (no
        # attempt is made to map PhysX's release/checked/profile/debug config
        # enum onto Blast's own, simpler config flag).
        blast_dir = os.path.join(src, "blast")
        if not os.path.isdir(blast_dir):
            sys.exit("--blast needs a blast/ directory in the checkout at " + src)
        blast_script = os.path.join(blast_dir, "build.bat" if is_windows else "build.sh")
        if not os.path.isfile(blast_script):
            sys.exit("no Blast build script at " + blast_script)
        run([blast_script] if is_windows else ["bash", blast_script], cwd=blast_dir)

        blast_platform = "windows-x86_64" if is_windows else "linux-x86_64"
        blast_sdk = os.path.join(blast_dir, "_build", blast_platform, "release", "blast-sdk")
        if not os.path.isdir(os.path.join(blast_sdk, "include")):
            sys.exit("Blast build finished but no SDK at " + blast_sdk)

        scons_args.append("blast_sdk=" + blast_sdk.replace("\\", "/"))

        print()
        print("Blast SDK ready:")
        print("    " + blast_sdk)
        if is_windows:
            print("Blast ships as DLLs (not static libs, unlike PhysX) -- SCsub copies all four next to")
            print("the Godot binary automatically (NvBlast, NvBlastGlobals, NvBlastExtAuthoring,")
            print("NvBlastExtShaders) -- nothing more to do.")
        else:
            # UNVERIFIED, same as the GPU .so case above -- SCsub does not
            # auto-copy anything on Linux yet.
            print("Blast ships as DLLs (not static libs, unlike PhysX) -- copy these next to the Godot binary too:")
            blast_bin = os.path.join(blast_sdk, "bin")
            for name in ("NvBlast", "NvBlastGlobals", "NvBlastExtAuthoring", "NvBlastExtShaders"):
                runtime = os.path.join(blast_bin, name + ".so")
                print("    cp %s bin/" % runtime)

    if args.flow:
        # Flow lives in the same checkout too (its flow/ subdirectory), with its
        # own premake5-based build. Its scripts must run from the flow/
        # directory. MSVC 14.44 turns warning C4756 (overflow in constant
        # arithmetic, in Flow's float-limit constants) into an error under
        # Flow's warnings-as-errors; CL=/wd4756 silences just that one.
        flow_dir = os.path.join(src, "flow")
        if not os.path.isdir(flow_dir):
            sys.exit("--flow needs a flow/ directory in the checkout at " + src)
        flow_script = os.path.join(flow_dir, "build.bat" if is_windows else "build.sh")
        if not os.path.isfile(flow_script):
            sys.exit("no Flow build script at " + flow_script)
        flow_env = os.environ.copy()
        if is_windows:
            flow_env["CL"] = (flow_env.get("CL", "") + " /wd4756").strip()
        print("+ %s  (in %s)" % (flow_script, flow_dir))
        subprocess.check_call([flow_script] if is_windows else ["bash", flow_script], cwd=flow_dir, env=flow_env)

        flow_platform = "windows-x86_64" if is_windows else "linux-x86_64"
        flow_bin = os.path.join(flow_dir, "_build", flow_platform, "release")
        if is_windows:
            flow_libs = ("nvflow.dll", "nvflowext.dll")
            missing = [name for name in flow_libs if not os.path.isfile(os.path.join(flow_bin, name))]
            if missing:
                sys.exit("Flow build finished but %s missing from %s" % (", ".join(missing), flow_bin))
        else:
            # The Linux library names aren't verified yet: take whatever
            # nvflow / nvflowext shared libraries the build produced.
            flow_libs = sorted(
                name
                for name in (os.listdir(flow_bin) if os.path.isdir(flow_bin) else [])
                if "nvflow" in name and ".so" in name and "rtx" not in name
            )
            if not flow_libs:
                sys.exit("Flow build finished but no nvflow libraries in " + flow_bin)
        scons_args.append("flow_sdk=" + flow_dir.replace("\\", "/"))

        print()
        print("Flow ready:")
        print("    " + flow_dir)
        if is_windows:
            print("SCsub copies nvflow.dll and nvflowext.dll next to the Godot binary automatically.")
        else:
            # SCsub only copies Flow's Windows DLLs so far.
            print("Copy Flow's libraries next to the Godot binary (SCsub doesn't on Linux yet):")
            for name in flow_libs:
                print("    cp %s bin/" % os.path.join(flow_bin, name))

    print()
    print("Build the module with:")
    print("    scons platform=%s target=editor %s" % (args.platform, " ".join(scons_args)))


if __name__ == "__main__":
    main()

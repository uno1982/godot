#!/usr/bin/env python3
"""Build the PhysX 5 SDK for linking into the godot_physx module.

The module links against a PhysX SDK built out of tree with the static CRT.
This script clones NVIDIA's PhysX repository at a pinned revision, drops in a
build preset tuned to match Godot, runs PhysX's own project generation and
CMake build, and prints the path to pass to scons as physx_sdk=.

    python modules/godot_physx/misc/build_physx.py [--gpu] [--blast] [--flow] [--alembic]
    scons platform=windows target=editor physx_sdk=<printed path> [physx_gpu=yes] [blast_sdk=<printed path>] [flow_sdk=<printed path>] [alembic_sdk=<printed path>]

The GPU build also needs the CUDA Toolkit (CUDA_PATH set or nvcc on PATH).
SCsub copies the GPU library next to the Godot binary for you.

--blast additionally builds the Blast SDK (runtime mesh fracture/destruction,
NvBlast + extensions, backing PhysXDestructible3D and its in-editor fracture
dialog) from this same checkout's blast/ subdirectory -- it's version-locked
to the same NVIDIA-Omniverse/PhysX release train as PhysX itself, so no
separate clone or ref pin is needed. SCsub copies its libraries next to the
Godot binary too.

--flow additionally builds NVIDIA Flow (sparse-grid smoke / fire / dust,
backing PhysXFlow3D) from the same checkout's flow/ subdirectory, with its own
build.bat / build.sh. flow_sdk= is that flow/ directory itself. Nothing is
linked -- nvflow / nvflowext are loaded at run time, and SCsub copies both
next to the Godot binary.

--alembic additionally builds Alembic and Imath (static, Ogawa only, the
static CRT on Windows as Godot links it), cloned at pinned tags into an
'alembic-sdk' directory beside the PhysX checkout. Only the editor links them,
for PhysXGroom's .abc importer; alembic_sdk= is their install directory.

The last thing printed is the scons command for everything built.

Run this script on the machine you're building the module for -- --platform
defaults to the host OS (windows / linuxbsd).

--platform android cross-builds the CPU SDK for Android arm64 with the NDK
Godot uses (ANDROID_HOME, or ANDROID_NDK_ROOT for another NDK; needs cmake and
ninja on PATH); with --blast, also Blast as static libraries, and with
--flow, Flow's two libraries (after Flow's own build for this machine, which
compiles its shaders). GPU and Alembic aren't available there. Use a separate
checkout (--src) from the desktop SDK's.
"""

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys

# Pinned so a given godot_physx revision always builds against the same SDK.
PHYSX_REPO = "https://github.com/NVIDIA-Omniverse/PhysX.git"
PHYSX_REF = "ovphysx-0.5.11"  # PhysX SDK 5.10.0

# Alembic and Imath for the editor's groom importer (--alembic), pinned the same way.
ALEMBIC_REPO = "https://github.com/alembic/alembic.git"
ALEMBIC_REF = "1.8.12"
IMATH_REPO = "https://github.com/AcademySoftwareFoundation/Imath.git"
IMATH_REF = "v3.2.3"

HERE = os.path.dirname(os.path.abspath(__file__))
PRESET_DIR = os.path.join(HERE, "physx_presets")
PATCH_DIR = os.path.join(HERE, "physx_patches")


def run(cmd, cwd):
    print("+ " + " ".join(cmd) + "  (in %s)" % cwd)
    subprocess.check_call(cmd, cwd=cwd)


def build_alembic(base, jobs):
    """Clones and builds static Imath + Alembic under base, installed to base/install. Returns the install path."""
    install = os.path.join(base, "install")
    os.makedirs(base, exist_ok=True)
    for name, repo, ref in (("imath", IMATH_REPO, IMATH_REF), ("alembic", ALEMBIC_REPO, ALEMBIC_REF)):
        if not os.path.isdir(os.path.join(base, name, ".git")):
            run(["git", "clone", "--depth", "1", "--branch", ref, repo, name], cwd=base)
        else:
            print("using existing %s checkout: %s" % (name, os.path.join(base, name)))
    common = [
        "-DCMAKE_BUILD_TYPE=Release",
        "-DCMAKE_INSTALL_PREFIX=" + install,
        "-DCMAKE_PREFIX_PATH=" + install,
        # Godot links the static CRT on Windows, and builds position-independent code on Linux.
        "-DCMAKE_POLICY_DEFAULT_CMP0091=NEW",
        "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded",
        "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
    ]
    projects = (
        ("imath", ["-DBUILD_SHARED_LIBS=OFF", "-DBUILD_TESTING=OFF", "-DPYTHON=OFF"]),
        (
            "alembic",
            [
                "-DALEMBIC_SHARED_LIBS=OFF",
                "-DUSE_HDF5=OFF",
                "-DUSE_TESTS=OFF",
                "-DUSE_BINARIES=OFF",
                "-DUSE_ARNOLD=OFF",
                "-DUSE_MAYA=OFF",
                "-DUSE_PRMAN=OFF",
                "-DUSE_PYALEMBIC=OFF",
                "-DImath_DIR=" + os.path.join(install, "lib", "cmake", "Imath"),
            ],
        ),
    )
    for name, options in projects:
        build = os.path.join(base, "build", name)
        run(["cmake", "-S", os.path.join(base, name), "-B", build] + common + options, cwd=base)
        run(
            ["cmake", "--build", build, "--config", "Release", "--target", "install", "--parallel", str(jobs)], cwd=base
        )
    if not os.path.isdir(os.path.join(install, "include", "Alembic")):
        sys.exit("Alembic build finished but no SDK at " + install)
    return install


def android_ndk():
    """The NDK Godot's Android build uses (platform/android/detect.py), or ANDROID_NDK_ROOT if set."""
    ndk = os.environ.get("ANDROID_NDK_ROOT", "")
    if not ndk:
        home = os.environ.get("ANDROID_HOME") or os.environ.get("ANDROID_SDK_ROOT", "")
        if not home:
            sys.exit("--platform android needs ANDROID_HOME (the Android SDK) or ANDROID_NDK_ROOT")
        # Read, not imported: detect.py needs SCons' build environment.
        detect = os.path.join(HERE, "..", "..", "..", "platform", "android", "detect.py")
        with open(detect, encoding="utf-8") as f:
            m = re.search(r'def get_ndk_version\(\):\s*return "([^"]+)"', f.read())
        if not m:
            sys.exit("could not read the NDK version from " + detect)
        ndk = os.path.join(home, "ndk", m.group(1))
    toolchain = os.path.join(ndk, "build", "cmake", "android.toolchain.cmake")
    if not os.path.isfile(toolchain):
        sys.exit("no Android NDK at %s (install it with the Android SDK manager)" % ndk)
    return ndk


def build_android(physx, ndk, config, jobs):
    """CPU PhysX for Android arm64, as PhysX's own Linux arm64 target built with the NDK toolchain. PhysX's
    generate_projects needs NVIDIA's packman tools, so CMake is configured directly with what it would pass (the
    switches match linux64-godot.xml). Returns the installed SDK path."""
    if not shutil.which("ninja"):
        sys.exit("--platform android needs ninja on PATH (the Android SDK's cmake/<version>/bin has one)")
    root = physx.replace("\\", "/")
    build = os.path.join(physx, "compiler", "android-arm64-godot-" + config)
    install = os.path.join(physx, "install", "android-arm64-godot", "PhysX")
    run(
        [
            "cmake",
            "-S",
            os.path.join(physx, "compiler", "public"),
            "-B",
            build,
            "-G",
            "Ninja",
            "--no-warn-unused-cli",
            "-DCMAKE_TOOLCHAIN_FILE=" + os.path.join(ndk, "build", "cmake", "android.toolchain.cmake"),
            "-DANDROID_ABI=arm64-v8a",
            "-DANDROID_PLATFORM=android-24",  # Godot's minimum (get_min_target_api)
            "-DANDROID_STL=c++_static",  # as Godot links it
            "-DTARGET_BUILD_PLATFORM=linux",
            "-DPX_OUTPUT_ARCH=arm",
            "-DCMAKE_BUILD_TYPE=" + config,
            "-DPHYSX_ROOT_DIR=" + root,
            "-DPX_OUTPUT_LIB_DIR=" + root,
            "-DPX_OUTPUT_BIN_DIR=" + root,
            "-DPX_BUILDSNIPPETS=FALSE",
            "-DPX_BUILDPVDRUNTIME=FALSE",
            "-DPX_GENERATE_STATIC_LIBRARIES=TRUE",
            "-DPX_CMAKE_SUPPRESS_REGENERATION=TRUE",
            "-DPX_GENERATE_GPU_PROJECTS=FALSE",
            "-DPX_FLOAT_POINT_PRECISE_MATH=FALSE",
            "-DCMAKE_INSTALL_PREFIX=" + install.replace("\\", "/"),
        ],
        cwd=physx,
    )
    run(["cmake", "--build", build, "--target", "install", "--parallel", str(jobs)], cwd=physx)
    if not os.path.isdir(os.path.join(install, "bin", "linux.aarch64", config)):
        sys.exit("build finished but no Android libraries under " + install)
    return install


def build_host_flow(src):
    """Flow's own build for this machine (it lives in the checkout's flow/ subdirectory, with a premake5 build whose
    scripts must run from there). Returns (flow dir, its platform folder name). MSVC 14.44 turns warning C4756
    (overflow in constant arithmetic, in Flow's float-limit constants) into an error under Flow's
    warnings-as-errors; CL=/wd4756 silences just that one."""
    is_windows = os.name == "nt"
    flow_dir = os.path.join(src, "flow")
    if not os.path.isdir(flow_dir):
        sys.exit("--flow needs a flow/ directory in the checkout at " + src)
    if is_windows:
        flow_script = os.path.join(flow_dir, "build.bat")
    else:
        # ARM Linux has its own script.
        flow_script = os.path.join(flow_dir, "build_aarch64.sh" if platform.machine() == "aarch64" else "build.sh")
    if not os.path.isfile(flow_script):
        sys.exit("no Flow build script at " + flow_script)
    flow_env = os.environ.copy()
    if is_windows:
        flow_env["CL"] = (flow_env.get("CL", "") + " /wd4756").strip()
    print("+ %s  (in %s)" % (flow_script, flow_dir))
    subprocess.check_call([flow_script] if is_windows else ["bash", flow_script], cwd=flow_dir, env=flow_env)
    return flow_dir, "windows-x86_64" if is_windows else "linux-" + platform.machine()


def build_android_flow(src, ndk, jobs):
    """libnvflow.so and libnvflowext.so for Android arm64, with misc/flow_android/CMakeLists.txt (Flow's premake
    build has no Android target). Their shaders come precompiled from Flow's own build for this machine, run first:
    the module uses only the SPIR-V variants, which don't depend on the platform. Returns the flow/ directory
    (scons' flow_sdk=); SCsub picks the libraries up from its _build/android-arm64/install/lib."""
    flow_dir, host_platform = build_host_flow(src)
    generated = os.path.join(flow_dir, "_generated", host_platform, "release")
    if not os.path.isdir(os.path.join(generated, "nvflow", "shaders")):
        sys.exit("Flow's build finished but no generated shaders at " + generated)
    build = os.path.join(flow_dir, "_build", "android-arm64")
    run(
        [
            "cmake",
            "-S",
            os.path.join(HERE, "flow_android"),
            "-B",
            build,
            "-G",
            "Ninja",
            "-DCMAKE_TOOLCHAIN_FILE=" + os.path.join(ndk, "build", "cmake", "android.toolchain.cmake"),
            "-DANDROID_ABI=arm64-v8a",
            "-DANDROID_PLATFORM=android-24",
            "-DANDROID_STL=c++_static",
            "-DCMAKE_BUILD_TYPE=Release",
            "-DFLOW_ROOT=" + flow_dir.replace("\\", "/"),
            "-DFLOW_GENERATED=" + generated.replace("\\", "/"),
            "-DCMAKE_INSTALL_PREFIX=" + os.path.join(build, "install").replace("\\", "/"),
        ],
        cwd=flow_dir,
    )
    run(["cmake", "--build", build, "--target", "install", "--parallel", str(jobs)], cwd=flow_dir)
    for name in ("libnvflow.so", "libnvflowext.so"):
        if not os.path.isfile(os.path.join(build, "install", "lib", name)):
            sys.exit("Flow build for Android finished but no " + name)
    return flow_dir


def build_android_blast(src, ndk, jobs):
    """Blast's NvBlast, NvBlastGlobals, NvBlastExtShaders and NvBlastExtAuthoring for Android arm64, as static
    libraries, with misc/blast_android/CMakeLists.txt (Blast's premake build has no Android target). Its one
    dependency, Boost.Multiprecision (header only), comes from Blast's own packman. Returns the installed SDK path."""
    blast = os.path.join(src, "blast")
    if not os.path.isdir(os.path.join(blast, "source", "sdk", "lowlevel")):
        sys.exit("--blast needs a blast/ directory in the checkout at " + src)
    boost = os.path.join(blast, "_build", "target-deps", "BoostMultiprecision")
    if not os.path.isdir(os.path.join(boost, "boost")):
        packman = os.path.join(blast, "tools", "packman", "packman.cmd" if os.name == "nt" else "packman")
        run([packman, "install", "BoostMultiprecision", "1.64.0.1", "-l", boost], cwd=blast)
    build = os.path.join(blast, "_build", "android-arm64")
    install = os.path.join(build, "blast-sdk")
    run(
        [
            "cmake",
            "-S",
            os.path.join(HERE, "blast_android"),
            "-B",
            build,
            "-G",
            "Ninja",
            "-DCMAKE_TOOLCHAIN_FILE=" + os.path.join(ndk, "build", "cmake", "android.toolchain.cmake"),
            "-DANDROID_ABI=arm64-v8a",
            "-DANDROID_PLATFORM=android-24",
            "-DANDROID_STL=c++_static",
            "-DCMAKE_BUILD_TYPE=Release",
            "-DBLAST_ROOT=" + blast.replace("\\", "/"),
            "-DBOOST_MULTIPRECISION=" + boost.replace("\\", "/"),
            "-DCMAKE_INSTALL_PREFIX=" + install.replace("\\", "/"),
        ],
        cwd=blast,
    )
    run(["cmake", "--build", build, "--target", "install", "--parallel", str(jobs)], cwd=blast)
    if not os.path.isfile(os.path.join(install, "bin", "libNvBlast.a")):
        sys.exit("Blast build finished but no libraries under " + install)
    return install


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
        "--alembic",
        action="store_true",
        help="also build Alembic + Imath (static) for the editor's .abc groom importer",
    )
    ap.add_argument(
        "--platform",
        choices=["windows", "linuxbsd", "android"],
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

    android = args.platform == "android"
    if android:
        if args.gpu or args.alembic:
            sys.exit("--platform android builds the CPU PhysX SDK, Blast and Flow (no --gpu / --alembic)")
        ndk = android_ndk()
    else:
        preset_base = "vc17win64-godot" if args.platform == "windows" else "linux64-godot"
        preset = preset_base + "-gpu" if args.gpu else preset_base
        preset_file = os.path.join(PRESET_DIR, preset + ".xml")
        if not os.path.isfile(preset_file):
            sys.exit("missing preset: " + preset_file)
    if args.gpu and not (os.environ.get("CUDA_PATH") or shutil.which("nvcc")):
        sys.exit("--gpu needs the CUDA Toolkit (set CUDA_PATH or put nvcc on PATH)")

    # Locate / create the PhysX source tree.
    src = args.src
    if not src:
        # Beside the Godot repo, not inside it -- SDK source is not part of the tree.
        repo_root = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
        src = os.path.join(os.path.dirname(repo_root), "physx-sdk")
    src = os.path.abspath(src)

    if not os.path.exists(os.path.join(src, ".git")):  # (a file in a git worktree)
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

    if android:
        sdk = build_android(physx, ndk, args.config, args.jobs)
        scons_args = ["physx_sdk=" + sdk.replace("\\", "/")]
        print()
        print("PhysX SDK ready:")
        print("    " + sdk)
        if args.blast:
            blast_sdk = build_android_blast(src, ndk, args.jobs)
            scons_args.append("blast_sdk=" + blast_sdk.replace("\\", "/"))
            print()
            print("Blast SDK ready (static, linked into the game's library):")
            print("    " + blast_sdk)
        if args.flow:
            flow_dir = build_android_flow(src, ndk, args.jobs)
            scons_args.append("flow_sdk=" + flow_dir.replace("\\", "/"))
            print()
            print("Flow ready (SCsub packs its libraries into the Android template):")
            print("    " + flow_dir)
        print()
        print("Build the Android export templates with:")
        print("    scons platform=android target=template_release arch=arm64 " + " ".join(scons_args))
        return

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
        print()
        print(
            "SCsub copies %s next to the Godot binary, nothing else to do."
            % ("PhysXGpu_64.dll" if args.platform == "windows" else "libPhysXGpu_64.so")
        )

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
        if is_windows:
            run([blast_script], cwd=blast_dir)
        else:
            # Use our own compiler instead of NVIDIA's build container (patch 0005).
            blast_cmd = ["bash", blast_script, "--no-docker"]
            # Parallel codegen sometimes leaves a half written header, a rerun fixes it.
            if subprocess.call(blast_cmd, cwd=blast_dir) != 0:
                print("Blast build failed, retrying once")
                run(blast_cmd, cwd=blast_dir)

        blast_platform = "windows-x86_64" if is_windows else "linux-x86_64"
        blast_sdk = os.path.join(blast_dir, "_build", blast_platform, "release", "blast-sdk")
        if not os.path.isdir(os.path.join(blast_sdk, "include")):
            sys.exit("Blast build finished but no SDK at " + blast_sdk)

        scons_args.append("blast_sdk=" + blast_sdk.replace("\\", "/"))

        print()
        print("Blast SDK ready:")
        print("    " + blast_sdk)
        print("SCsub copies Blast's libraries next to the Godot binary.")

    if args.flow:
        flow_dir, flow_platform = build_host_flow(src)
        flow_bin = os.path.join(flow_dir, "_build", flow_platform, "release")
        flow_libs = ("nvflow.dll", "nvflowext.dll") if is_windows else ("libnvflow.so", "libnvflowext.so")
        missing = [name for name in flow_libs if not os.path.isfile(os.path.join(flow_bin, name))]
        if missing:
            sys.exit("Flow build finished but %s missing from %s" % (", ".join(missing), flow_bin))
        scons_args.append("flow_sdk=" + flow_dir.replace("\\", "/"))

        print()
        print("Flow ready:")
        print("    " + flow_dir)
        print("SCsub copies %s and %s next to the Godot binary automatically." % flow_libs)

    if args.alembic:
        # Beside the PhysX checkout: its own small tree, independent of PhysX's build.
        alembic_sdk = build_alembic(os.path.join(os.path.dirname(src), "alembic-sdk"), args.jobs)
        scons_args.append("alembic_sdk=" + alembic_sdk.replace("\\", "/"))

        print()
        print("Alembic + Imath ready (linked by editor builds only):")
        print("    " + alembic_sdk)
        print("SCsub copies their license notice, ALEMBIC-LICENSE.md, next to the editor.")

    print()
    print("Build the module with:")
    print("    scons platform=%s target=editor %s" % (args.platform, " ".join(scons_args)))


if __name__ == "__main__":
    main()

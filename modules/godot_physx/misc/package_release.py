#!/usr/bin/env python3
"""Package a release of the PhysX module build: an editor zip and an export templates .tpz per platform.

Run it after building, with the same options, at the commit being released:

    scons platform=windows target=editor   ... (and template_debug, template_release)
    scons platform=linuxbsd target=editor  ... (and template_debug, template_release)
    python modules/godot_physx/misc/package_release.py --version 0.1.0-beta \\
        --windows-bin bin --linux-bin <the Linux build's bin/>

It writes, straight from the bin/ folders (nothing is staged or copied first), into
bin/release/<version>/ (or --out):

    godot-<engine>-physx-<version>-windows-x86_64-editor.zip            the editor + its DLLs + licenses
    godot-<engine>-physx-<version>-windows-x86_64-export-templates.tpz  debug/release templates + their DLLs
    godot-<engine>-physx-<version>-linux-x86_64-editor.zip              the editor + its .so files + licenses
    godot-<engine>-physx-<version>-linux-x86_64-export-templates.tpz    debug/release templates + their .so files
    SHA256SUMS.txt

<engine> is the version in version.py (e.g. 4.7.3). A .tpz installs with Editor -> Manage Export
Templates -> Install from File; installing both adds them to the same version folder, so one editor can
export to both platforms. The export plugin copies the module's libraries for the target platform from
that folder next to the exported game.

Every binary must carry the commit being released (--commit, default HEAD): Godot embeds its build's
commit hash, so a stale binary left in bin/ by an earlier build stops the packaging instead of being
shipped. Leave out --windows-bin or --linux-bin to package one platform only.
"""

import argparse
import hashlib
import os
import subprocess
import sys
import zipfile

MODULE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENGINE_DIR = os.path.dirname(os.path.dirname(MODULE_DIR))

# The module's runtime libraries next to the binary (SCsub copies them into bin/).
WINDOWS_LIBS = [
    "PhysXGpu_64.dll",
    "NvBlast.dll",
    "NvBlastGlobals.dll",
    "NvBlastExtAuthoring.dll",
    "NvBlastExtShaders.dll",
    "nvflow.dll",
    "nvflowext.dll",
]
LINUX_LIBS = [
    "libPhysXGpu_64.so",
    "libNvBlast.so",
    "libNvBlastGlobals.so",
    "libNvBlastExtAuthoring.so",
    "libNvBlastExtShaders.so",
    "libnvflow.so",
    "libnvflowext.so",
]
# The D3D12 Agility SDK, when the build has one: next to the editor as is, and in the templates folder with the
# arch in the name, which is what the Windows exporter looks for.
D3D12_LIBS = ["D3D12Core.dll", "d3d12SDKLayers.dll"]

# (path in the archive, source relative to the engine checkout)
LICENSES = [
    ("LICENSE.txt", "LICENSE.txt"),
    ("COPYRIGHT.txt", "COPYRIGHT.txt"),
    ("godot_physx-LICENSE.txt", "modules/godot_physx/LICENSE"),
    ("PHYSX-LICENSE.md", "modules/godot_physx/PHYSX-LICENSE.md"),
    ("ALEMBIC-LICENSE.md", "modules/godot_physx/ALEMBIC-LICENSE.md"),
]


def fail(msg):
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(1)


def engine_version():
    values = {}
    with open(os.path.join(ENGINE_DIR, "version.py"), encoding="utf-8") as f:
        exec(f.read(), values)
    return values["major"], values["minor"], values["patch"], values["status"]


def resolve_commit(ref):
    try:
        return subprocess.check_output(["git", "-C", ENGINE_DIR, "rev-parse", ref], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        fail(f"can't resolve commit {ref!r} in {ENGINE_DIR}")


def require(path):
    if not os.path.isfile(path):
        fail(f"missing {path}")
    return path


def check_commit(path, commit):
    with open(path, "rb") as f:
        data = f.read()
    if commit.encode("ascii") not in data:
        fail(f"{path} wasn't built from {commit[:10]} (rebuild it at that commit)")


class Archive:
    def __init__(self, path):
        self.path = path
        self.zip = zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED, compresslevel=6)

    def add(self, src, name, executable=False):
        info = zipfile.ZipInfo.from_file(src, name)
        # Unix permissions, so the Linux binaries unzip executable. unzip only reads them from an entry made on
        # Unix, and zipfile marks entries written on Windows as MS-DOS ones.
        info.create_system = 3
        info.external_attr = (0o100755 if executable else 0o100644) << 16
        info.compress_type = zipfile.ZIP_DEFLATED
        with open(src, "rb") as f, self.zip.open(info, "w") as out:
            while chunk := f.read(1 << 24):
                out.write(chunk)
        print(f"  {name}")

    def add_text(self, name, text):
        info = zipfile.ZipInfo(name)
        info.create_system = 3
        info.external_attr = 0o100644 << 16
        info.compress_type = zipfile.ZIP_DEFLATED
        self.zip.writestr(info, text)
        print(f"  {name}")

    def add_licenses(self, prefix=""):
        for name, src in LICENSES:
            self.add(require(os.path.join(ENGINE_DIR, src)), prefix + name)

    def close(self):
        self.zip.close()


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(1 << 24):
            h.update(chunk)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--version", required=True, help="module release version, e.g. 0.1.0-beta")
    parser.add_argument("--windows-bin", help="bin/ of the Windows build")
    parser.add_argument("--linux-bin", help="bin/ of the Linux build")
    parser.add_argument("--commit", default="HEAD", help="the commit every binary must be built from (default HEAD)")
    parser.add_argument("--out", help="output folder (default bin/release/<version>)")
    args = parser.parse_args()
    if not args.windows_bin and not args.linux_bin:
        fail("give --windows-bin and/or --linux-bin")

    major, minor, patch, status = engine_version()
    engine = f"{major}.{minor}.{patch}" if patch else f"{major}.{minor}"
    templates_version = f"{engine}.{status}"  # the export templates folder the editor looks in, e.g. 4.7.3.rc
    commit = resolve_commit(args.commit)
    prefix = f"godot-{engine}-physx-{args.version}"
    out = args.out or os.path.join(ENGINE_DIR, "bin", "release", args.version)
    os.makedirs(out, exist_ok=True)
    print(f"engine {templates_version}, commit {commit[:10]}, into {out}")

    win = args.windows_bin
    lin = args.linux_bin
    win_editor = ["godot.windows.editor.x86_64.exe", "godot.windows.editor.x86_64.console.exe"]
    # (source in bin/, name in the templates folder)
    win_templates = [
        ("godot.windows.template_release.x86_64.exe", "windows_release_x86_64.exe"),
        ("godot.windows.template_release.x86_64.console.exe", "windows_release_x86_64_console.exe"),
        ("godot.windows.template_debug.x86_64.exe", "windows_debug_x86_64.exe"),
        ("godot.windows.template_debug.x86_64.console.exe", "windows_debug_x86_64_console.exe"),
    ]
    lin_templates = [
        ("godot.linuxbsd.template_release.x86_64", "linux_release.x86_64"),
        ("godot.linuxbsd.template_debug.x86_64", "linux_debug.x86_64"),
    ]

    # Check everything before writing anything.
    if win:
        for name in win_editor + WINDOWS_LIBS + [src for src, _ in win_templates]:
            require(os.path.join(win, name))
        for name in [win_editor[0]] + [src for src, _ in win_templates if not src.endswith(".console.exe")]:
            check_commit(os.path.join(win, name), commit)
    if lin:
        for name in ["godot.linuxbsd.editor.x86_64"] + LINUX_LIBS + [src for src, _ in lin_templates]:
            require(os.path.join(lin, name))
        for name in ["godot.linuxbsd.editor.x86_64"] + [src for src, _ in lin_templates]:
            check_commit(os.path.join(lin, name), commit)
    d3d12 = [name for name in D3D12_LIBS if win and os.path.isfile(os.path.join(win, name))]

    written = []
    if win:
        path = os.path.join(out, f"{prefix}-windows-x86_64-editor.zip")
        print(path)
        a = Archive(path)
        for name in win_editor + WINDOWS_LIBS + d3d12:
            a.add(os.path.join(win, name), name)
        a.add_licenses()
        a.close()
        written.append(path)
    if lin:
        path = os.path.join(out, f"{prefix}-linux-x86_64-editor.zip")
        print(path)
        a = Archive(path)
        a.add(os.path.join(lin, "godot.linuxbsd.editor.x86_64"), "godot.linuxbsd.editor.x86_64", executable=True)
        for name in LINUX_LIBS:
            a.add(os.path.join(lin, name), name, executable=True)
        a.add_licenses()
        a.close()
        written.append(path)

    # One .tpz per platform: installing a second one adds to the same version folder (the editor doesn't clear it).
    if win:
        path = os.path.join(out, f"{prefix}-windows-x86_64-export-templates.tpz")
        print(path)
        a = Archive(path)
        a.add_text("templates/version.txt", templates_version)
        for src, name in win_templates:
            a.add(os.path.join(win, src), "templates/" + name)
        for name in WINDOWS_LIBS:
            a.add(os.path.join(win, name), "templates/" + name)
        for name in d3d12:
            base, ext = os.path.splitext(name)
            a.add(os.path.join(win, name), f"templates/{base}.x86_64{ext}")
        a.add_licenses("templates/")
        a.close()
        written.append(path)
    if lin:
        path = os.path.join(out, f"{prefix}-linux-x86_64-export-templates.tpz")
        print(path)
        a = Archive(path)
        a.add_text("templates/version.txt", templates_version)
        for src, name in lin_templates:
            a.add(os.path.join(lin, src), "templates/" + name, executable=True)
        for name in LINUX_LIBS:
            a.add(os.path.join(lin, name), "templates/" + name, executable=True)
        a.add_licenses("templates/")
        a.close()
        written.append(path)

    sums = os.path.join(out, "SHA256SUMS.txt")
    with open(sums, "w", encoding="utf-8", newline="\n") as f:
        for p in written:
            line = f"{sha256(p)}  {os.path.basename(p)}"
            f.write(line + "\n")
            print(f"{line}  ({os.path.getsize(p) / (1 << 20):.0f} MB)")
    print(f"done: {len(written)} archives + {os.path.basename(sums)}")


if __name__ == "__main__":
    main()

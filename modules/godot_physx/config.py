import os

from SCons.Script import ARGUMENTS

_notified = False


def _physx_sdk_path():
    return ARGUMENTS.get("physx_sdk", os.environ.get("PHYSX_SDK", ""))


def can_build(env, platform):
    global _notified

    if env["disable_physics_3d"]:
        return False

    # The module links against an out-of-tree PhysX 5 SDK (it is not vendored).
    # With no SDK configured, quietly skip it so a stock build still succeeds.
    if not _physx_sdk_path():
        if not _notified:
            print(
                "godot_physx: no PhysX SDK configured, module disabled. "
                "Pass physx_sdk=<path> or set PHYSX_SDK, or run "
                "modules/godot_physx/misc/build_physx.py. See modules/godot_physx/README.md."
            )
            _notified = True
        return False

    return True


def configure(env):
    pass

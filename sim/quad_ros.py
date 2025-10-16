#!/usr/bin/env python3
"""Backward-compatible wrapper that forwards to py_sim package."""

import os
import runpy
import sys


def main() -> None:
    try:
        import rospkg
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("rospkg is required to launch py_sim.quad_ros") from exc

    rospack = rospkg.RosPack()
    script_path = os.path.join(rospack.get_path("py_sim"), "scripts", "quad_ros.py")
    if not os.path.isfile(script_path):
        raise RuntimeError("Unable to locate py_sim/scripts/quad_ros.py")
    runpy.run_path(script_path, run_name="__main__")


if __name__ == "__main__":
    main()

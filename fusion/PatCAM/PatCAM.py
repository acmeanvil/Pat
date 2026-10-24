"""Pat CAM add-in for Autodesk Fusion.

Adds commands to the Manufacture workspace that export a CAM setup (part,
stock, fixtures, tools, work coordinate system) to Pat, generate verified
toolpaths, preview them in the canvas and write G-code.

The engine is the compiled ``pat`` module in ``lib/`` (built by
``fusion/build_addin.py``). The Fusion-free logic lives in ``pat_fusion``;
``patcam_glue`` holds everything that talks to the Fusion API.
"""

import os
import sys
import traceback

import adsk.core

HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(HERE, "lib"), HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def run(context):
    try:
        import patcam_glue

        patcam_glue.start()
    except Exception:
        adsk.core.Application.get().userInterface.messageBox("Pat add-in failed to start:\n" + traceback.format_exc())


def stop(context):
    try:
        import patcam_glue

        patcam_glue.stop()
    except Exception:
        adsk.core.Application.get().userInterface.messageBox("Pat add-in failed to stop:\n" + traceback.format_exc())

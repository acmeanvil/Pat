"""Fusion-independent parts of the Pat add-in.

Nothing in this package imports ``adsk``, so it can be unit-tested outside
Fusion. The Fusion glue (``fusion_glue.py``) extracts plain data from the
Fusion API and hands it to these modules.
"""

__all__ = ["tools", "stl", "scene", "geometry", "engine"]

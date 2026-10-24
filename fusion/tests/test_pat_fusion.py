"""Tests for the Fusion-independent add-in code (run outside Fusion).

    python -m unittest discover -s fusion/tests

Needs the ``pat`` module installed (``maturin develop`` in crates/pat-py).
"""

import json
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "fusion", "PatCAM"))

from pat_fusion import geometry, scene, stl, tools  # noqa: E402
from pat_fusion.engine import Job, verification_text  # noqa: E402

BALL_MM = {
    "type": "ball end mill",
    "unit": "millimeters",
    "description": "6mm ball",
    "geometry": {"DC": 6, "LCF": 15, "LB": 40, "SFDM": 6, "RE": 3},
    "holder": {"segments": [
        {"height": 15, "lower-diameter": 20, "upper-diameter": 20},
        {"height": 25, "lower-diameter": 20, "upper-diameter": 32},
    ]},
    "post-process": {"number": 3},
}

BULL_INCH = {
    "type": "bull nose end mill",
    "unit": "inches",
    "geometry": {"DC": 0.5, "LCF": 1.0, "LB": 1.5, "RE": 0.03},
    "post-process": {"number": 7},
}


class ToolConversion(unittest.TestCase):
    def test_ball_with_holder(self):
        t, notes = tools.fusion_tool_to_pat(BALL_MM)
        self.assertEqual(t["number"], 3)
        cutter, shank, h1, h2 = t["segments"]
        self.assertEqual(cutter, {"kind": "cutter", "length": 15.0, "diameter": 6.0, "corner_radius": 3.0})
        self.assertEqual(shank["length"], 25.0)
        self.assertEqual(h1["diameter"], 20.0)
        self.assertEqual((h2["diameter_bottom"], h2["diameter_top"]), (20.0, 32.0))
        self.assertAlmostEqual(t["gauge_length"], 80.0)
        self.assertTrue(any("gauge length" in n for n in notes))

    def test_inch_bull_without_holder(self):
        t, notes = tools.fusion_tool_to_pat(BULL_INCH, gauge_length=100.0)
        cutter = t["segments"][0]
        self.assertAlmostEqual(cutter["diameter"], 12.7)
        self.assertAlmostEqual(cutter["corner_radius"], 0.762)
        self.assertAlmostEqual(cutter["length"], 25.4)
        self.assertEqual(t["gauge_length"], 100.0)
        self.assertTrue(any("no holder" in n for n in notes))

    def test_missing_diameter(self):
        with self.assertRaises(ValueError):
            tools.fusion_tool_to_pat({"type": "flat end mill", "geometry": {}})


class Geometry(unittest.TestCase):
    def test_wcs_round_trip(self):
        # WCS rotated 90° about Z and shifted (10, 0, 5) cm in design space.
        m = [0.0, -1.0, 0.0, 10.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 5.0, 0, 0, 0, 1]
        p = (12.0, 3.0, 6.0)
        w = geometry.design_to_wcs(m, p)
        self.assertEqual(tuple(round(v, 9) for v in w), (30.0, -20.0, 10.0))
        back = geometry.wcs_to_design(m, w)
        self.assertEqual(tuple(round(v, 9) for v in back), p)

    def test_preview_strips(self):
        moves = [
            {"type": "rapid", "tip": [0, 0, 10], "axis": [0, 0, 1]},
            {"type": "rapid", "tip": [0, 0, 1], "axis": [0, 0, 1]},
            {"type": "feed", "tip": [10, 0, 0], "axis": [0, 0, 1], "feed": 100},
            {"type": "feed", "tip": [20, 0, 0], "axis": [0, 0, 1], "feed": 100},
            {"type": "rapid", "tip": [20, 0, 10], "axis": [0, 0, 1]},
        ]
        lines = geometry.preview_polylines(moves, geometry.IDENTITY, axis_every=1)
        kinds = [k for k, _ in lines]
        self.assertEqual(kinds.count("axis"), 2)
        feed = [c for k, c in lines if k == "feed"][0]
        # Feed strip starts where the rapid ended, in cm.
        self.assertEqual(feed[:3], [0.0, 0.0, 0.1])
        self.assertEqual(feed[-3:], [2.0, 0.0, 0.0])


class EndToEnd(unittest.TestCase):
    """Export-like flow without Fusion: tool JSON, STL bodies, scene, then
    rough and verify with the engine."""

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="pat-fusion-")

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_rough_a_block(self):
        job = Job(self.dir)
        job.write_machine(os.path.join(REPO, "machines", "vmc-3axis.toml"))
        for name, lo, hi in [("part", (-20, -15, -10), (20, 15, 0)), ("stock", (-22, -17, -10), (22, 17, 2))]:
            v, t = stl.box_mesh(lo, hi)
            stl.write_binary_stl(os.path.join(self.dir, name + ".stl"), v, t)
        ball, _ = tools.fusion_tool_to_pat(dict(BALL_MM, type="flat end mill", geometry={"DC": 6, "LCF": 15, "LB": 40}))
        job.write_scene(scene.scene_toml([ball], [
            {"name": "part", "role": "part", "stl": "part.stl"},
            {"name": "stock", "role": "stock", "stl": "stock.stl"},
        ]))
        desc = job.load()
        self.assertEqual(desc["setup"]["tools"][0]["number"], 3)
        r = job.call("rough", tool=3, params={"stepdown": 2.0, "resolution": 0.5})
        self.assertTrue(r["verified"], json.dumps(r["verification"], indent=1))
        text = verification_text(r)
        self.assertIn("VERIFIED", text)
        prog = job.call("get_program", id=r["program_id"], gcode_lines=5)
        self.assertEqual(len(prog["gcode_head"]), 5)
        saved = job.call("save_program", id=r["program_id"], path="rough.nc")
        self.assertTrue(os.path.exists(os.path.join(self.dir, "rough.nc")))
        self.assertIn("verified", saved["note"])


if __name__ == "__main__":
    unittest.main()

# Pat

Open-source 3-, 4- and 5-axis CAM for Autodesk Fusion and beyond.

The goal is a toolpath planner with these features:

- collision avoidance, including automatic tool tilting;
- high-speed milling strategies;
- AI-agent integration to help program and verify parts.

The core is a Rust library with Python bindings. Fusion is the first host, and
the same engine works from the command line or any Python environment.

**Status: all 7 roadmap milestones are done.** The Fusion add-in still needs testing inside Fusion.

- **Milestone 1:** machine models, forward and inverse kinematics, a G-code
  simulator and a multi-axis post. The simulator covers work offsets, canned
  cycles, subroutines, G68.2 tilted planes and acceleration-aware cycle time.
- **Milestone 2:** tool, holder, fixture and machine collision checking, plus
  explainable tilt-to-avoid and reachability maps.
- **Milestone 3:** automatic 3+2 orientation selection, adaptive
  (constant-engagement) roughing with rest machining, and material-removal
  simulation with gouge detection.
- **Milestone 4:** continuous 4-axis rotary roughing and finishing, wrapping
  2D paths onto cylinders, and indexed 4-axis.
- **Milestone 5:** simultaneous 5-axis finishing:
  - surface finishing with tool-axis control (normal with lead and tilt,
    point and line modes);
  - automatic collision avoidance, singularity locking and smoothing;
  - swarf milling.
- **Milestone 6:** AI agent integration. The `pat-mcp` MCP server lets
  assistants such as Claude plan, generate and verify toolpaths. Every program
  is verified by simulation, and failing ones are not saved. See
  [docs/agent.md](docs/agent.md).
- **Milestone 7:** a Fusion add-in (`fusion/PatCAM`). It exports a setup,
  generates and verifies toolpaths with Pat, previews them and writes G-code.
  See [fusion/README.md](fusion/README.md). See [docs/architecture.md](docs/architecture.md) for the
design and roadmap, and
[reports/CAM software landscape survey.md](reports/CAM%20software%20landscape%20survey.md)
for the market research behind it.

## Layout

| Path | What |
|---|---|
| `crates/pat-core` | Kinematics, G-code interpreter, simulator, post |
| `crates/pat-cli` | `pat` command-line tool |
| `crates/pat-py` | Python module `pat` (for the Fusion add-in and scripting) |
| `crates/pat-mcp` | MCP server for AI agents (`pat-mcp`) |
| `fusion/` | Fusion add-in (`PatCAM`), its tests and build script |
| `machines/` | Example machine definitions (table-table, head-table, head-head, 4th axis, 3-axis) |
| `examples/` | Sample CL path and G-code |

## Quick start

```sh
cargo test
cargo run -- machine machines/trunnion-ac.toml

# Post a 5-axis cutter-location path, then verify the result
cargo run -- post examples/cone-sweep.cl.json -m machines/trunnion-ac.toml -o /tmp/cone.ngc
cargo run -- check /tmp/cone.ngc -m machines/trunnion-ac.toml --tool 1=100

# See what the checker catches
cargo run -- check examples/bad-trunnion.ngc -m machines/trunnion-ac.toml --tool 1=100

# Kinematics
cargo run -- ik -m machines/trunnion-ac.toml --tip 0,0,0 --axis 0,0.5,0.866 --tool-length 100

# Collisions: tools, vise and part from a setup file, plus the machine's own geometry
cargo run -- check examples/vise-crash.ngc -m machines/trunnion-ac.toml --scene examples/vise-setup.toml

# Smallest tilt that clears the clamp, with an explanation
cargo run -- tilt -m machines/trunnion-ac.toml --scene examples/vise-setup.toml --tool 1 --tip 0,15,0

# Automatic 3+2 roughing, then verify with collision and stock simulation
cargo run --release -- orient -m machines/trunnion-ac.toml --scene examples/overhang-setup.toml --tool 1
cargo run --release -- rough -m machines/trunnion-ac.toml --scene examples/overhang-setup.toml --tool 1 --auto --gcode /tmp/overhang.ngc
cargo run --release -- check /tmp/overhang.ngc -m machines/trunnion-ac.toml --scene examples/overhang-setup.toml --stock

# Continuous 4-axis: rough a square bar from round stock, engrave text on a cylinder
cargo run --release -- rotary rough -m machines/vmc-4th-a.toml --scene examples/square-bar-4th.toml --tool 1 --gcode /tmp/bar.ngc
cargo run --release -- check /tmp/bar.ngc -m machines/vmc-4th-a.toml --scene examples/square-bar-4th.toml --stock
cargo run --release -- wrap examples/wrap-text.json -m machines/vmc-4th-a.toml --radius 14 --depth 0.3 --gcode /tmp/engrave.ngc

# Simultaneous 5-axis: finish a bump with the axis following the surface (avoiding a clamp), then swarf a sloped wall
cargo run --release -- surface -m machines/trunnion-ac.toml --scene examples/bump-5axis.toml --tool 1 --stepover 1 --step 1 --gcode /tmp/bump.ngc
cargo run --release -- check /tmp/bump.ngc -m machines/trunnion-ac.toml --scene examples/bump-5axis.toml --stock
cargo run --release -- swarf examples/wall-rails.json -m machines/trunnion-ac.toml --scene examples/bump-5axis.toml --tool 2 --side right --gcode /tmp/swarf.ngc
```

Add `--json` to `check` or `machine` for machine-readable output. Use
`--offset G55=x,y,z` for work offsets not in the machine file, and
`--dwell-ms` for Fanuc-style millisecond `P` dwells.

### Python

```sh
pip install maturin
cd crates/pat-py && maturin develop
```

```python
import json, pat
machine = open("machines/trunnion-ac.toml").read()
report = json.loads(pat.check(machine, open("part.ngc").read(), json.dumps({"tool_lengths": {"1": 100}})))
print(report["ok"], [i["code"] for i in report["issues"]])
```

## Machine files

A machine has two kinematic chains: the **head** carries the spindle and the
**table** carries the part. Axes are listed from the base outward. They are
described with every axis at zero, in the machine coordinate frame. The
conventions are documented in
[crates/pat-core/src/machine.rs](crates/pat-core/src/machine.rs).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <https://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in this work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

GPL-licensed CAM code may be read for reference only. Do not copy it in. See the
licensing rules in [docs/architecture.md](docs/architecture.md).

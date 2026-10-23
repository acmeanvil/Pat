# Pat

Open-source 3-, 4- and 5-axis CAM for Autodesk Fusion and beyond.

The goal is a toolpath planner with these features:

- collision avoidance, including automatic tool tilting;
- high-speed milling strategies;
- AI-agent integration to help program and verify parts.

The core is a Rust library with Python bindings. Fusion is the first host, and
the same engine works from the command line or any Python environment.

**Status: milestones 1 and 2 of 6 are done.**

- **Milestone 1:** machine models, forward and inverse kinematics, a G-code
  simulator and a multi-axis post. The simulator covers work offsets, canned
  cycles, subroutines, G68.2 tilted planes and acceleration-aware cycle time.
- **Milestone 2:** tool, holder, fixture and machine collision checking, plus
  explainable tilt-to-avoid and reachability maps. See [docs/architecture.md](docs/architecture.md) for the
design and roadmap, and
[reports/CAM software landscape survey.md](reports/CAM%20software%20landscape%20survey.md)
for the market research behind it.

## Layout

| Path | What |
|---|---|
| `crates/pat-core` | Kinematics, G-code interpreter, simulator, post |
| `crates/pat-cli` | `pat` command-line tool |
| `crates/pat-py` | Python module `pat` (for the Fusion add-in and scripting) |
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

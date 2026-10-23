# Pat architecture

Pat is an open-source (MIT OR Apache-2.0) 3-, 4- and 5-axis toolpath planner.
Its goal is collision-aware multi-axis toolpaths, high-speed milling
strategies, and an AI agent that can drive and explain all of it. Fusion is
the first host, not the only one.

## Why a host-agnostic core

The [CAM landscape survey](../reports/CAM%20software%20landscape%20survey.md)
found that Fusion add-ins cannot register new toolpath strategies or inject
external toolpaths as native operations. Pat therefore computes toolpaths,
simulates them and posts them itself (the survey's Option C/D). It uses
Fusion only for:

- geometry,
- machine definitions,
- user interface.

The same core can serve a CLI, FreeCAD, a web app or an AI agent.

## Layers

```
            Fusion add-in (Python)   CLI (pat)   AI agent tools (MCP, planned)
                       \                |              /
                        pat-py (Python bindings, JSON in/out)
                                        |
  pat-core ─────────────────────────────────────────────────────────────
   planning (planned)   strategies: adaptive/HSM, 3+2, 4-axis, 5-axis
   collision (planned)  tool / holder / fixture / machine, auto-tilt
   post                 CL (tip + axis) → G-code, linearisation, G93
   sim                  replay G-code on the machine, report problems
   interp / gcode       G-code parsing and modal state
   kinematics           FK / analytic IK for any XYZ + ≤2 rotary layout
   machine              TOML/JSON machine definitions
   math                 vectors, rotations, rigid transforms
```

Every layer exchanges plain serialisable data (TOML machines, CL JSON, G-code
text, JSON reports). That is what makes the AI-agent integration cheap: each
capability is already a function with structured input and output.

## Milestone 1: kinematics and verification foundation (done)

- **Machine model.** Head and table chains, axes defined at home in the base
  frame (product of exponentials), soft limits, continuous rotaries.
  Examples are in `machines/`: table-table AC, head-table BC, head-head CA,
  4th-axis VMC and 3-axis VMC.
- **Kinematics.**
  - Forward kinematics for any arrangement.
  - Analytic inverse kinematics for rotaries (Paden–Kahan subproblems 1 and 2)
    plus an affine solve for the linear axes.
  - All branches and ±360° wraps are enumerated and ranked by limits and
    continuity.
  - Singular poses hold the primary rotary.
- **Simulator** (`pat check`). Replays non-TCP, TCP (G43.4 / G234) and G53
  moves, sampling rotary motion. It reports:
  - axis-limit violations, including overtravel mid-move;
  - near-singularity swings and rotary flips;
  - tool-tip deviation on moves that were not linearised;
  - feed errors;
  - unsupported codes.

  It also estimates cycle time and can emit a per-move trace.
- **Post** (`pat post`).
  - LinuxCNC joint-space output with adaptive linearisation to a tip and axis
    tolerance and G93 inverse-time feeds.
  - Fanuc-style G43.4 TCP output.

The posted output is verified by re-simulating it in the tests.

### Milestone 1b: control coverage (done)

- **Work offsets.** G54–G59, G59.1–G59.3, G54.1 Pn and Haas G110–G129, from
  the machine file's `[work_offsets]` or `--offset NAME=x,y,z`.
- **Canned cycles.** G73, G81–G86, G88 and G89, with G98/G99 retract,
  G90/G91 and L/K repeats. Cycles drill along the active plane's Z, so they
  also work inside G68.2.
- **Subroutines.** LinuxCNC `oNNN sub/endsub/call/return` (no arguments) and
  Fanuc/Haas `M98 P… L…` / `M99`, flattened before interpretation. Issues
  point at lines inside the subroutine.
- **Tilted planes.** G68.2 P0 (ZXZ Euler) plus G53.1 alignment and G69. A
  move in the plane without aligning the tool is flagged.
- **Cycle time.** A look-ahead planner uses per-axis `max_accel` and
  `max_rate` with GRBL-style junction deviation and trapezoidal velocity
  profiles. It honours G61/G64/G9 exact stop and falls back to feed-rate
  arithmetic when acceleration data is missing.

Still not covered:
- O-word control flow, parameters and expressions;
- G10 offset setting;
- G68.3, and G68.2 with P1–P3;
- G87 back boring and G74 left-hand tapping;
- jerk limits and servo lag in the time estimate.

## Milestone 2: collision layer (done)

- **Geometry** (`geom`):
  - triangle meshes from binary/ASCII STL, boxes and cylinders;
  - an AABB tree;
  - an exact finite-cylinder-vs-triangle test (slab clip plus
    axis-to-polygon distance);
  - triangle-triangle and point-in-mesh (ray parity) tests.
- **Scene** (`collision`):
  - **Tools** are stacks of cutter, shank and holder sections. Cones are
    stepped with the larger radius, so checks err on the safe side.
  - **Part, stock and fixture bodies** are given in WCS coordinates and ride
    on the table.
  - **Machine components** are declared in the machine file and attach to
    any link (base, head, table or a specific axis).
- **Checks.** Each sampled position, at a 1 mm step along linear motion and
  0.5° of rotary motion, is checked for:
  - holder or shank hitting anything;
  - the cutter hitting a fixture or machine component;
  - the cutter inside the part during a rapid;
  - machine bodies colliding across head, table and base. This includes
    bodies nested wholly inside one another.

  Each collision is reported with the line, both bodies and the tool-tip
  position.
- **Planning** (`access`):
  - `clearance`: how far the tool assembly clears everything;
  - `tilt_to_avoid`: the smallest tilt from a preferred axis that is
    reachable within limits and collision-free with a margin. Among
    directions at that tilt it picks the one with the most clearance and
    explains the choice, e.g. *"preferred axis blocked (T1 holder vs tall
    clamp); tilted 5.0° toward −Y for 1.4 mm clearance"*;
  - `reachable_directions`: an accessibility map over a cone of tool axes.
- **Front ends.** `pat check --scene`, `pat tilt`, `pat reach` and the
  Python functions `check(..., scene_toml)`, `tilt` and `reach`.

Known limits:
- Stock is not updated by cutting yet (no material-removal simulation), so
  the cutter is not checked against stock, and rapids are checked against the
  finished part.
- Feed moves into the part are not yet reported as gouges.
- Meshes must be closed.
- Exactly coplanar contact is not reported.
- Machine-to-machine checks have no clearance margin.

## Roadmap

1. **3+2 and HSM roughing.**
   - Automatic 3+2 orientation selection from the reachability map.
   - Adaptive (constant-engagement) clearing in 2.5D and 3D (z-level), with
     engagement-based feed control.
   - Rest machining.
   - A material-removal model (dexel or voxel stock) for in-process stock,
     gouge checks and rapids through remaining stock.

   All GPL CAM code is used for reference only: algorithms are reimplemented
   from published papers.
2. **4-axis.** Continuous rotary roughing and finishing, plus wrap and unwrap.
3. **5-axis finishing.**
   - Swarf, multi-axis flowline and surface-normal strategies with lead/lag and
     tilt control.
   - Tool-axis smoothing.
   - One application pack: port machining is the first candidate.
4. **AI agent integration.**
   - An MCP server exposing `machine`, `check`, `post`, `ik`, and later `plan`,
     `collide` and `reach` as tools.
   - The agent proposes setups and orientations, and explains simulator
     findings in shop terms.
   - The agent never bypasses verification: every agent-produced program goes
     through `check`.
5. **Fusion adapter.**
   - The add-in reads the machine definition (`adsk.cam.Machine`), BRep/mesh,
     stock and tools.
   - It runs `pat` through the Python wheel and previews toolpaths with custom
     graphics.
   - It writes G-code with the Pat post.
   - Open questions from the survey (reading CL data, Manual NC payloads,
     Personal-tier terms) must be settled before this ships.

## Licensing rules for contributions

- MIT/Apache code may be vendored with its notices kept.
- LGPL libraries (OpenCAMLib, OCCT) may only be used as separate, replaceable
  runtime dependencies and are never copied into this tree.
- GPL code is reference reading only. Clean-room reimplementation must work
  from papers, not from GPL source.
- Code with no license (e.g. OpenMill) must not be read for implementation
  detail.

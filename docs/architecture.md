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

Not yet covered: G68.2 tilted planes, canned cycles, multiple work offsets,
O-word subroutines, and the controller's own look-ahead and acceleration
model (cycle times are feed-rate estimates).

## Roadmap

1. **Collision layer.**
   - Tool, holder, fixture, stock and machine-component solids attached to the
     kinematic chains.
   - BVH/SDF distance queries, plus collision checks in `sim` during G-code
     replay.
   - Tilt-to-avoid: find the minimum tilt away from a preferred axis that
     clears the holder, with an explanation of why each tilt was chosen.
   - An accessibility and reachability map per surface.
2. **3+2 and HSM roughing.**
   - Automatic 3+2 orientation selection from reachability.
   - Adaptive (constant-engagement) clearing in 2.5D and 3D (z-level), with
     engagement-based feed control.
   - Rest machining.

   All GPL CAM code is used for reference only: algorithms are reimplemented
   from published papers.
3. **4-axis.** Continuous rotary roughing and finishing, plus wrap and unwrap.
4. **5-axis finishing.**
   - Swarf, multi-axis flowline and surface-normal strategies with lead/lag and
     tilt control.
   - Tool-axis smoothing.
   - One application pack: port machining is the first candidate.
5. **AI agent integration.**
   - An MCP server exposing `machine`, `check`, `post`, `ik`, and later `plan`,
     `collide` and `reach` as tools.
   - The agent proposes setups and orientations, and explains simulator
     findings in shop terms.
   - The agent never bypasses verification: every agent-produced program goes
     through `check`.
6. **Fusion adapter.**
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

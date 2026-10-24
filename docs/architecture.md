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

## Milestone 3: 3+2 orientation, adaptive roughing, material removal (done)

- **Material model** (`stock`): a voxel stock with per-block occupancy, cut
  by flat, ball and bull-nose cutter shapes.
  - Fixed-axis moves are swept exactly in one pass (an analytic swept-cylinder
    test); only rounded tips and axis-changing moves are sampled.
  - `pat check --stock` uses it to report gouges into the part (beyond a
    tolerance), rapids into stock, shanks or holders rubbing remaining stock,
    and removed and remaining volume.
- **Rasters** (`raster`): part height fields in any orientation and a raster
  drop-cutter for the whole tool assembly:
  - the shaped cutter;
  - shank, holder and the spindle housing taken from the machine file;
  - a fast disk-max filter for large radii;
  - a 1.5-cell safety margin, so the stock to leave is a guaranteed minimum.
- **Adaptive clearing** (`adaptive`): constant-engagement steering toward a
  target cut width.
  - Material is kept on the climb side.
  - It travels through cleared space (keep-down links) found with one
    distance-map search, and enters through open air, a helix or a plunge.
  - A low-load cleanup phase removes slivers.
- **Roughing** (`rough`): z-levels plus levels at flat floors, in any fixed
  orientation.
  - Material comes from the in-process stock, so rest machining is automatic.
  - The tool assembly keeps clear of fixtures and of the table-side machine
    bodies placed at the setup's rotary angles.
  - Non-cutting sections keep clear of the stock still standing above the cut.
- **Orientation selection** (`orient`): area-weighted surface samples and
  visibility. Walls count only where the flute can reach them.
  - Each direction is checked for machine reachability and collisions at the
    part corners, then chosen by a greedy area cover that prefers head-on
    views.
  - Each setup comes with rotary angles, G68.2 angles and an explanation.
- **Pipeline** (`plan::auto_rough`): orient, then rough each setup on the
  stock left by the previous ones, joined by transitions along a safe sphere.
- **Front ends.** `pat orient`, `pat rough [--axis | --auto] [--gcode]` and
  `pat check --stock`; Python `orient` and `rough`.

Verified end to end in the tests. Generated programs are posted and replayed
with collision and stock simulation on the trunnion model, including its
spindle housing, cradle and platter, with zero gouges, rapids into stock,
rubs or collisions. Rest machining cuts less than 60% of what a fresh run with
the small tool would.

Known limits:
- Raster accuracy is about one cell (0.25 to 0.5 mm typical), so walls end up
  oversized by up to 1.5 cells on top of the stock to leave.
- Roughing does not yet adjust feed to engagement.
- No finishing strategies yet.
- 3+2 transitions assume the safe sphere is free of fixtures.
- 4- and 5-axis simultaneous roughing are not covered.

## Milestone 4: continuous 4-axis (done)

- **Cylindrical planning** (`rotary`), around the machine's table rotary axis
  (found automatically):
  - the part's radius at each (x, θ);
  - the outer radius of the remaining stock, from the voxels;
  - a rotary drop-cutter giving the lowest safe tip radius for a radial
    cutter (flat, ball or bull nose, grown by the stock to leave), plus
    shank, holder and spindle housing.
- **Rotary roughing.** Radial levels are cut with continuous helical passes,
  X advancing while A spins.
  - Levels are added at cylindrical areas, and the direction alternates per
    level.
  - The tool retracts radially around places it may not go.
  - The stock is updated as it cuts, so rest machining works.
  - It never cuts outside the part's axial range (or a given `x_range`), so
    the stock held by the chuck and tailstock is never parted off.
- **Rotary finishing.** A fine helix rides the safe surface.
- **Wrap / unwrap.** 2D paths (u axial, v arc length) become 4-axis cuts on a
  cylinder, and back again for previews.
- **Indexed 4-axis (4+1).** This works through `auto_rough`: orientation
  selection keeps only directions the machine can reach (radial ones on a 4th
  axis). Downward faces count as real surface whenever the machine can point
  at them.
- **Front ends.** `pat rotary rough|finish`, `pat wrap` and Python `rotary`
  and `wrap_paths`. Examples: `examples/square-bar-4th.toml` and
  `examples/wrap-text.json`.

Verified by posting (LinuxCNC joint dialect with adaptive linearisation and
G93) and replaying on `machines/vmc-4th-a.toml` with collision and stock
simulation: zero gouges, rapids into stock, rubs or collisions.

Known limits:
- Continuous helices wind the A axis through many turns (about 99 turns in
  the bar example). Controls need rotary rollover configured, or LinuxCNC
  wrapped-rotary mode; the post does not unwind.
- Roughing passes are helical at full step-down rather than
  constant-engagement.
- Finishing follows the conservative raster surface, so accuracy is about
  one grid cell.
- With 3+2 or indexed roughing on a 4th axis, model the chuck and tailstock
  as fixtures to protect the held stock.

## Milestone 5: simultaneous 5-axis finishing (done)

- **Exact ball drop-cutter** (`geom::sphere_drop_triangle`): face, edge and
  vertex contact along any direction. The ball centre is gouge-free by
  construction, so the tool axis is free to choose.
- **Surface finishing** (`fiveaxis::surface_finish`): a zig-zag pattern in any
  projection frame and pass angle, dropped onto the part.
  - **Tool-axis control** (`AxisControl`): fixed, surface normal with lead
    and tilt, toward or from a point, toward or from a line.
  - **Feasibility** of each axis:
    - the machine reaches it within limits, continuing from the previous
      point, with rotary flips rejected;
    - with a scene, the shank, holder, head and the cutter's side above the
      ball (new `MoveClass::Finish`) clear the part, fixtures and machine.
  - **Avoidance.** When an axis fails, rings of increasing tilt are searched
    around the preferred axis, keeping the ball centre fixed and preferring
    continuity.
  - **Singularity lock.** Near the singular direction, the primary rotary is
    held and only the other rotary is solved (`Kinematics::inverse_locked`).
  - **Branch changes.** Where limits force another rotary branch, the pass
    breaks for a retract and rotate instead of dropping points.
  - **Smoothing and densifying.** Axes are smoothed and re-checked, and
    midpoints between points are checked. Points are inserted wherever the
    axis turns more than 5°, so the straight-line tip motion between points
    cannot sag the ball into the part.
  - **Transitions.** The tool backs out along its axis, travels upright above
    the part *and* fixtures, turns to the next start axis and comes back down
    along it.
- **Swarf** (`fiveaxis::swarf`): flank milling of a ruled wall from two rails.
  - The axis follows the rulings and the tool is offset by its radius toward
    the chosen side.
  - It supports depth passes, leave, tip offset, lead in and out, and station
    reachability and collision checks.
  - It reports the deviation between stations (zero for developable walls).
- **Front ends.** `pat surface` and `pat swarf`; Python `surface` and
  `swarf_wall`. Examples: `examples/bump-5axis.toml` and
  `examples/wall-rails.json`.

Verified by posting (G43.4 TCP) and replaying on the trunnion with its
spindle housing, cradle and platter, with collision and stock simulation.
Normal-plus-lead finishing over a bump and avoidance around a tall clamp
replay with zero gouges or collisions, and swarf on a sloped wall has zero
deviation and never enters the part.

Known limits:
- Surface finishing is ball-end only.
- Patterns are zig-zag only (no flowline / UV patterns yet; meshes carry no
  surface parameterisation).
- Swarf needs its rails supplied. Detecting ruled walls from meshes is left
  for the Fusion adapter, which has the B-rep edges.
- Passing close to the singular direction between points can still swing C
  quickly; the simulator flags these moves.

## Roadmap

1. **AI agent integration.**
   - An MCP server exposing `machine`, `check`, `post`, `ik`, and later `plan`,
     `collide` and `reach` as tools.
   - The agent proposes setups and orientations, and explains simulator
     findings in shop terms.
   - The agent never bypasses verification: every agent-produced program goes
     through `check`.
2. **Fusion adapter.**
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

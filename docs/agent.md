# Using Pat from an AI agent

`pat-mcp` is a [Model Context Protocol](https://modelcontextprotocol.io)
server. It lets an AI assistant plan, generate and verify toolpaths with Pat.
It speaks MCP over stdio and has no dependencies beyond the Pat core.

## Connect

**Claude Code, in this repository:** the checked-in `.mcp.json` registers the
server. Build it once with `cargo build --release -p pat-mcp`, then approve
the `pat` server when Claude Code asks.

**Claude Code, anywhere:**

```sh
cargo build --release -p pat-mcp
claude mcp add pat -- /path/to/Pat/target/release/pat-mcp --root /path/to/your/jobs
```

**Claude Desktop and other MCP clients:** run the same command
(`pat-mcp --root DIR [--machine FILE] [--scene FILE]`) as a stdio server.

`--root` is the workspace. The server only reads and writes files inside it.
`--machine` and `--scene` preload a setup.

## Tools

| Tool | What it does |
|---|---|
| `load_machine`, `load_scene` | Load a machine file, and a setup with tools (cutter, shank and holder), part, stock and fixtures. |
| `describe_setup` | Axes, limits, machine geometry, tools, bodies, programs so far. |
| `check_gcode` | Simulate any G-code: limits, collisions, gouges, singularities, feeds, cycle time. |
| `plan_orientations` | 3+2 setups that reach the part, with rotary and G68.2 angles and explanations. |
| `rough` | Adaptive roughing (feed adjusted to load, smoothed corners) along an axis, or automatic 3+2. |
| `trochoidal_slot` | High-speed trochoidal slotting along a centre line, with measured engagement. |
| `rotary` | Continuous 4-axis roughing (constant-engagement adaptive or helical) or finishing. |
| `surface_finish` | Simultaneous 5-axis finishing with tool-axis control and collision avoidance. |
| `swarf` | Flank milling of a ruled wall from two rails. |
| `tilt` | Smallest collision-free tilt at a point, and what blocks the preferred axis. |
| `inverse_kinematics` | All joint solutions for a tool pose. |
| `get_program`, `save_program` | Inspect a generated program; write its G-code. |

## Safety rules

The server enforces these rules; it does not rely on the agent to follow
them:

1. **Every generated program is verified.** It is posted and replayed on the
   machine model with collision checks, and with material-removal simulation
   when the setup has stock. The response carries `verified` and a report:
   counts per issue kind, the first issues with line numbers, a plain-language
   remedy for each kind, the cycle time and the volume removed.
2. **Unverified G-code is not written.** `save_program` refuses a program
   that failed verification. `force: true` overrides that, and the server's
   instructions tell the agent to use it only when the user explicitly accepts
   the risk after it has been explained.
3. **File access is confined** to the workspace root.

## Example session

> **User:** Rough the overhang part on the trunnion and tell me if it is safe
> to run.
>
> The agent:
> 1. Calls `load_machine {path: "machines/trunnion-ac.toml"}`.
> 2. Calls `load_scene {path: "examples/overhang-setup.toml"}`.
> 3. Calls `plan_orientations {tool: 1}`. Answer: 7 setups, top first, then
>    tilts of about 63°. The cradle limits the tilt.
> 4. Calls `rough {tool: 1, auto: true}`. Answer: program 1, verified, about
>    24k blocks, cycle time about 12 minutes, 10.5% of the surface out of
>    reach (the walls under the overhang need a longer tool).
> 5. Calls `save_program {id: 1, path: "out/overhang.ngc"}`.

//! Model Context Protocol server for Pat.
//!
//! A small JSON-RPC 2.0 implementation of the MCP stdio transport
//! (newline-delimited messages): `initialize`, `ping`, `tools/list` and
//! `tools/call`. Every tool is a thin wrapper around
//! [`pat_core::session::Session`], which owns the safety rules: generated
//! programs are always verified by simulation, unverified ones are not
//! written out, and file access stays in the workspace.

use pat_core::math::Vec3;
use pat_core::session::Session;
use serde_json::{json, Value};

pub const PROTOCOL_VERSIONS: [&str; 3] = ["2025-06-18", "2025-03-26", "2024-11-05"];

pub const INSTRUCTIONS: &str = "\
Pat is an open-source 3/4/5-axis CAM engine. Typical workflow:
1. load_machine (e.g. machines/trunnion-ac.toml) and load_scene (tools, part, stock, fixtures).
2. describe_setup to see axes, limits, tools and bodies.
3. Plan: plan_orientations for 3+2 setups; tilt to explain tool access at a point.
4. Generate: rough (adaptive, axis or auto 3+2), trochoidal_slot, rotary (4th axis), surface_finish (5-axis,
   tool-axis control), swarf. Roughing adjusts feed to the measured load and smooths corners; posts fit arcs.
5. Every generated program comes back with `verified` and a `verification` report (collisions, gouges,
   axis limits, singularities, cycle time) plus a remedy per issue kind. Explain the issues to the user in
   shop terms and change parameters to fix them. Do not hide or downplay errors.
6. save_program writes G-code into the workspace. It refuses programs that failed verification; only
   pass force=true when the user explicitly accepts the risk after you explained it.
Units are mm and degrees; points and axes are in the work coordinate system (part zero).";

fn vec3(v: &Value) -> Option<Vec3> {
    let a = v.as_array()?;
    (a.len() == 3).then(|| {
        Vec3::new(
            a[0].as_f64().unwrap_or(0.0),
            a[1].as_f64().unwrap_or(0.0),
            a[2].as_f64().unwrap_or(0.0),
        )
    })
}

fn points(v: &Value) -> Option<Vec<Vec3>> {
    v.as_array()?.iter().map(vec3).collect()
}

fn schema(props: Value, required: &[&str]) -> Value {
    json!({ "type": "object", "properties": props, "required": required })
}

const XYZ: &str = "[x, y, z] in mm (WCS)";

/// Tool definitions advertised by `tools/list`.
pub fn tool_list() -> Value {
    let p3 = json!({"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3, "description": XYZ});
    let dir = json!({"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3, "description": "unit direction [i, j, k] (WCS)"});
    let dialect = json!({"type": "string", "enum": ["fanuc-tcp", "linuxcnc-joint"], "description": "post dialect (default: TCP for 5-axis machines, joint space otherwise)"});
    let params = |what: &str| json!({"type": "object", "description": format!("optional overrides of {what} fields")});
    json!([
        {"name": "describe_setup", "description": "Show the loaded machine (axes, limits, geometry), setup (tools, bodies) and generated programs.",
         "inputSchema": schema(json!({}), &[])},
        {"name": "load_machine", "description": "Load a machine definition (TOML) from a workspace file or text.",
         "inputSchema": schema(json!({"path": {"type": "string"}, "toml": {"type": "string"}}), &[])},
        {"name": "load_scene", "description": "Load the job setup (tools with cutter/shank/holder sections, part, stock and fixture bodies) from a workspace file or TOML text.",
         "inputSchema": schema(json!({"path": {"type": "string"}, "toml": {"type": "string"}}), &[])},
        {"name": "check_gcode", "description": "Simulate G-code on the machine and setup: axis limits, collisions, gouges (with stock simulation), singularities, feeds, cycle time. Returns issues with remedies.",
         "inputSchema": schema(json!({"path": {"type": "string"}, "gcode": {"type": "string"}, "simulate_stock": {"type": "boolean", "default": true}}), &[])},
        {"name": "plan_orientations", "description": "Choose 3+2 tool orientations that reach the part, with machine reachability and collision checks; each comes with rotary angles, G68.2 angles and an explanation.",
         "inputSchema": schema(json!({"tool": {"type": "integer"}, "options": params("OrientOptions (max_tilt_deg, max_wall_depth, target_coverage, ...)")}), &[])},
        {"name": "rough", "description": "Adaptive (constant-engagement) roughing of the stock to the part, along a fixed axis or with automatic 3+2 setups. The result is posted and verified.",
         "inputSchema": schema(json!({"tool": {"type": "integer"}, "axis": dir, "auto": {"type": "boolean", "default": false},
            "params": params("RoughParams (stepdown, stepover, leave, resolution, feed, ...)"), "dialect": dialect}), &["tool"])},
        {"name": "rotary", "description": "Continuous 4-axis roughing or finishing around the machine's table rotary axis (helical passes). Posted and verified.",
         "inputSchema": schema(json!({"tool": {"type": "integer"}, "mode": {"type": "string", "enum": ["rough", "finish"]},
            "params": params("RotaryParams (stepdown, stepover = helix pitch, leave, dx, dtheta_deg, x_range, ...)"), "dialect": dialect}), &["tool", "mode"])},
        {"name": "surface_finish", "description": "Simultaneous 5-axis surface finishing with a ball end mill. params.axis.control picks the tool axis: {\"type\":\"normal\",\"lead_deg\":10,\"tilt_deg\":0}, {\"type\":\"fixed\",\"axis\":[0,0,1]}, {\"type\":\"toward_point\"|\"from_point\",\"point\":[...]}, {\"type\":\"toward_line\"|\"from_line\",\"point\":[...],\"direction\":[...]}. Collisions are avoided by tilting. Posted and verified.",
         "inputSchema": schema(json!({"tool": {"type": "integer"},
            "params": params("SurfaceParams (stepover, step, angle_deg, projection, leave, region, axis {control, avoid, margin, max_tilt_deg, smooth, singularity_lock_deg})"), "dialect": dialect}), &["tool"])},
        {"name": "swarf", "description": "5-axis swarf (flank) milling of a ruled wall given bottom and top rails. Reports the deviation on twisted walls. Posted and verified.",
         "inputSchema": schema(json!({"tool": {"type": "integer"}, "bottom": {"type": "array", "items": p3}, "top": {"type": "array", "items": p3},
            "params": params("SwarfParams (side left/right, passes, depth_step, leave, tip_offset, step, feed)"), "dialect": dialect}), &["tool", "bottom", "top"])},
        {"name": "trochoidal_slot", "description": "High-speed trochoidal slotting: small climb-milling loops advancing along a slot centre line, full depth at a light radial load. Reports the measured engagement. Posted and verified (loops become G2/G3 arcs).",
         "inputSchema": schema(json!({"tool": {"type": "integer"},
            "centerline": {"type": "array", "items": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2}, "description": "slot centre line, [x, y] points (WCS)"},
            "width": {"type": "number"}, "z_top": {"type": "number"}, "z_bottom": {"type": "number"},
            "params": params("SlotParams (pitch = advance per loop, stepdown, leave, step, feed, return_feed, open_start)"), "dialect": dialect}),
            &["tool", "centerline", "width", "z_top", "z_bottom"])},
        {"name": "tilt", "description": "Smallest tilt from a preferred tool axis that reaches a point without collision, with an explanation of what blocks the preferred axis.",
         "inputSchema": schema(json!({"tool": {"type": "integer"}, "tip": p3, "axis": dir, "options": params("TiltOptions (margin, max_tilt_deg)")}), &["tool", "tip"])},
        {"name": "inverse_kinematics", "description": "All machine joint solutions for a tool tip and axis (WCS), with limit and singularity flags.",
         "inputSchema": schema(json!({"tip": p3, "axis": dir, "tool": {"type": "integer"}}), &["tip", "axis"])},
        {"name": "get_program", "description": "Details of a generated program: verification report, generator summary and the first lines of G-code.",
         "inputSchema": schema(json!({"id": {"type": "integer"}, "gcode_lines": {"type": "integer", "default": 40},
            "include_cl": {"type": "boolean", "default": false, "description": "also return the cutter-location moves (large)"}}), &["id"])},
        {"name": "save_program", "description": "Write a generated program's G-code to a workspace file. Refused if it failed verification unless force=true (only with the user's explicit consent).",
         "inputSchema": schema(json!({"id": {"type": "integer"}, "path": {"type": "string"}, "force": {"type": "boolean", "default": false}}), &["id", "path"])},
    ])
}

pub struct Server {
    pub session: Session,
}

impl Server {
    pub fn new(session: Session) -> Server {
        Server { session }
    }

    /// Run one tool; errors become tool results with `isError`.
    pub fn call(&mut self, name: &str, a: &Value) -> Result<Value, String> {
        let s = &mut self.session;
        let str_arg = |k: &str| a.get(k).and_then(Value::as_str);
        let tool = || {
            a.get("tool")
                .and_then(Value::as_u64)
                .map(|t| t as u32)
                .ok_or("missing 'tool' (tool number)".to_string())
        };
        let dialect = a.get("dialect").and_then(Value::as_str);
        match name {
            "describe_setup" => Ok(s.describe()),
            "load_machine" => s.load_machine(str_arg("path"), str_arg("toml")),
            "load_scene" => s.load_scene(str_arg("path"), str_arg("toml")),
            "check_gcode" => s.check_gcode(
                str_arg("gcode"),
                str_arg("path"),
                a.get("simulate_stock")
                    .and_then(Value::as_bool)
                    .unwrap_or(true),
            ),
            "plan_orientations" => s.plan_orientations(
                a.get("tool").and_then(Value::as_u64).map(|t| t as u32),
                a.get("options"),
            ),
            "rough" => s.rough(
                tool()?,
                a.get("axis").and_then(vec3),
                a.get("auto").and_then(Value::as_bool).unwrap_or(false),
                a.get("params"),
                dialect,
            ),
            "rotary" => {
                let finish = match str_arg("mode") {
                    Some("rough") => false,
                    Some("finish") => true,
                    _ => return Err("mode must be \"rough\" or \"finish\"".into()),
                };
                s.rotary(tool()?, finish, a.get("params"), dialect)
            }
            "surface_finish" => s.surface_finish(tool()?, a.get("params"), dialect),
            "swarf" => {
                let bottom = a
                    .get("bottom")
                    .and_then(points)
                    .ok_or("bottom must be a list of [x, y, z]")?;
                let top = a
                    .get("top")
                    .and_then(points)
                    .ok_or("top must be a list of [x, y, z]")?;
                s.swarf(tool()?, bottom, top, a.get("params"), dialect)
            }
            "trochoidal_slot" => {
                let line: Vec<[f64; 2]> = a
                    .get("centerline")
                    .and_then(Value::as_array)
                    .ok_or("centerline must be a list of [x, y]")?
                    .iter()
                    .map(|p| {
                        let v = p
                            .as_array()
                            .filter(|v| v.len() == 2)
                            .ok_or("centerline points must be [x, y]")?;
                        Ok([v[0].as_f64().unwrap_or(0.0), v[1].as_f64().unwrap_or(0.0)])
                    })
                    .collect::<Result<_, String>>()?;
                let num = |k: &str| {
                    a.get(k)
                        .and_then(Value::as_f64)
                        .ok_or(format!("missing '{k}'"))
                };
                s.trochoidal_slot(
                    tool()?,
                    line,
                    num("width")?,
                    num("z_top")?,
                    num("z_bottom")?,
                    a.get("params"),
                    dialect,
                )
            }
            "tilt" => {
                let tip = a.get("tip").and_then(vec3).ok_or("tip must be [x, y, z]")?;
                let axis = a.get("axis").and_then(vec3).unwrap_or(Vec3::Z);
                s.tilt(tool()?, tip, axis, a.get("options"))
            }
            "inverse_kinematics" => {
                let tip = a.get("tip").and_then(vec3).ok_or("tip must be [x, y, z]")?;
                let axis = a
                    .get("axis")
                    .and_then(vec3)
                    .ok_or("axis must be [i, j, k]")?;
                s.inverse_kinematics(
                    tip,
                    axis,
                    a.get("tool").and_then(Value::as_u64).map(|t| t as u32),
                )
            }
            "get_program" => s.get_program(
                a.get("id").and_then(Value::as_u64).ok_or("missing 'id'")? as usize,
                a.get("gcode_lines").and_then(Value::as_u64).unwrap_or(40) as usize,
                a.get("include_cl")
                    .and_then(Value::as_bool)
                    .unwrap_or(false),
            ),
            "save_program" => s.save_program(
                a.get("id").and_then(Value::as_u64).ok_or("missing 'id'")? as usize,
                str_arg("path").ok_or("missing 'path'")?,
                a.get("force").and_then(Value::as_bool).unwrap_or(false),
            ),
            other => Err(format!("unknown tool '{other}'")),
        }
    }

    /// Handle one JSON-RPC message; returns the response (None for
    /// notifications).
    pub fn handle(&mut self, msg: &Value) -> Option<Value> {
        let id = msg.get("id").cloned();
        let method = msg.get("method").and_then(Value::as_str).unwrap_or("");
        let params = msg.get("params").cloned().unwrap_or(Value::Null);
        let id = id?; // notifications (no id) get no response
        let result = match method {
            "initialize" => {
                let asked = params
                    .get("protocolVersion")
                    .and_then(Value::as_str)
                    .unwrap_or(PROTOCOL_VERSIONS[0]);
                let version = if PROTOCOL_VERSIONS.contains(&asked) {
                    asked
                } else {
                    PROTOCOL_VERSIONS[0]
                };
                Ok(json!({
                    "protocolVersion": version,
                    "capabilities": {"tools": {"listChanged": false}},
                    "serverInfo": {"name": "pat", "version": env!("CARGO_PKG_VERSION")},
                    "instructions": INSTRUCTIONS,
                }))
            }
            "ping" => Ok(json!({})),
            "tools/list" => Ok(json!({"tools": tool_list()})),
            "tools/call" => {
                let name = params.get("name").and_then(Value::as_str).unwrap_or("");
                let args = params.get("arguments").cloned().unwrap_or(json!({}));
                let (text, structured, is_error) = match self.call(name, &args) {
                    Ok(v) => (
                        serde_json::to_string_pretty(&v).unwrap_or_default(),
                        Some(v),
                        false,
                    ),
                    Err(e) => (e, None, true),
                };
                let mut r =
                    json!({"content": [{"type": "text", "text": text}], "isError": is_error});
                if let Some(v) = structured.filter(Value::is_object) {
                    r["structuredContent"] = v;
                }
                Ok(r)
            }
            _ => Err(json!({"code": -32601, "message": format!("method not found: {method}")})),
        };
        Some(match result {
            Ok(r) => json!({"jsonrpc": "2.0", "id": id, "result": r}),
            Err(e) => json!({"jsonrpc": "2.0", "id": id, "error": e}),
        })
    }
}

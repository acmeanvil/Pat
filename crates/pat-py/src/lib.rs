//! Python module `pat`.
//!
//! The interface is deliberately JSON-in / JSON-out: the same payloads work
//! from the Fusion add-in, plain Python scripts and AI agent tools, and the
//! binding layer stays thin.
//!
//! ```python
//! import json, pat
//! machine = open("machines/trunnion-ac.toml").read()
//! report = json.loads(pat.check(machine, open("part.ngc").read(), json.dumps({"tool_lengths": {"1": 100}})))
//! gcode = json.loads(pat.post(machine, cl_json, json.dumps({"dialect": "linuxcnc-joint"})))["gcode"]
//!
//! # Collision checks: pass a setup (tools, part, fixtures) as TOML.
//! scene = open("examples/vise-setup.toml").read()
//! report = json.loads(pat.check(machine, program, None, scene, "examples", "machines"))
//! tilt = json.loads(pat.tilt(machine, scene, 1, [0, 15, 0], [0, 0, 1], scene_dir="examples"))
//! print(tilt["explanation"])
//! ```

use pat_core::access::{cap_directions, reachable_directions, tilt_to_avoid, TiltOptions};
use pat_core::collision::{Role, SceneDef, World};
use pat_core::geom::Mesh;
use pat_core::math::Vec3;
use pat_core::orient::{select_orientations, OrientOptions};
use pat_core::plan::{auto_rough, stock_from_world, AutoRoughParams};
use pat_core::post::ClTool;
use pat_core::post::{post as post_cl, ClProgram, PostOptions};
use pat_core::rotary::{rotary_finish, rotary_rough, wrap, RotaryFrame, RotaryParams};
use pat_core::rough::{machining_frame, rough_with, RoughParams};
use pat_core::sim::{simulate, SimOptions};
use pat_core::{Kinematics, MachineDef, Pose};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use std::path::Path;
use std::sync::Arc;

fn err<E: std::fmt::Display>(e: E) -> PyErr {
    PyValueError::new_err(e.to_string())
}

fn machine(src: &str) -> PyResult<MachineDef> {
    let t = src.trim_start();
    if t.starts_with('{') {
        MachineDef::from_json(src)
    } else {
        MachineDef::from_toml(src)
    }
    .map_err(err)
}

fn opts<T: serde::de::DeserializeOwned + Default>(json: Option<&str>) -> PyResult<T> {
    match json {
        Some(s) if !s.trim().is_empty() => serde_json::from_str(s).map_err(err),
        _ => Ok(T::default()),
    }
}

/// Validate a machine definition (TOML or JSON); returns it as JSON.
#[pyfunction]
fn load_machine(machine_src: &str) -> PyResult<String> {
    serde_json::to_string(&machine(machine_src)?).map_err(err)
}

fn world(
    m: &MachineDef,
    scene_toml: Option<&str>,
    scene_dir: Option<&str>,
    machine_dir: Option<&str>,
) -> PyResult<World> {
    let scene = match scene_toml {
        Some(s) => SceneDef::from_toml(s).map_err(err)?,
        None => SceneDef::default(),
    };
    let k = Kinematics::new(m.clone(), m.default_tool_length).map_err(err)?;
    World::build(
        &k,
        &scene,
        scene_dir.map(Path::new),
        machine_dir.map(Path::new),
    )
    .map_err(err)
}

/// Simulate G-code; returns the report as JSON. With a scene (TOML) or a
/// machine that has components, moves are also checked for collisions.
/// STL paths resolve against `scene_dir` / `machine_dir`.
#[pyfunction]
#[pyo3(signature = (machine_src, program, options_json=None, scene_toml=None, scene_dir=None, machine_dir=None))]
fn check(
    machine_src: &str,
    program: &str,
    options_json: Option<&str>,
    scene_toml: Option<&str>,
    scene_dir: Option<&str>,
    machine_dir: Option<&str>,
) -> PyResult<String> {
    let m = machine(machine_src)?;
    let mut o: SimOptions = opts(options_json)?;
    if scene_toml.is_some() || !m.components.is_empty() {
        o.world = Some(Arc::new(world(&m, scene_toml, scene_dir, machine_dir)?));
    }
    serde_json::to_string(&simulate(&m, program, &o)).map_err(err)
}

fn kin_for_tool(m: &MachineDef, w: &World, tool: u32) -> PyResult<Kinematics> {
    let len = w
        .tool(tool)
        .map_or(m.default_tool_length, |t| t.def.gauge_length);
    Kinematics::new(m.clone(), len).map_err(err)
}

/// Smallest tilt from `axis` that reaches `tip` without collision; JSON
/// result with the chosen axis, joints, clearance and an explanation.
#[pyfunction]
#[pyo3(signature = (machine_src, scene_toml, tool, tip, axis, options_json=None, prev=None, scene_dir=None, machine_dir=None))]
#[allow(clippy::too_many_arguments)]
fn tilt(
    machine_src: &str,
    scene_toml: &str,
    tool: u32,
    tip: [f64; 3],
    axis: [f64; 3],
    options_json: Option<&str>,
    prev: Option<Vec<f64>>,
    scene_dir: Option<&str>,
    machine_dir: Option<&str>,
) -> PyResult<String> {
    let m = machine(machine_src)?;
    let w = world(&m, Some(scene_toml), scene_dir, machine_dir)?;
    let k = kin_for_tool(&m, &w, tool)?;
    let o: TiltOptions = opts(options_json)?;
    let r = tilt_to_avoid(&k, &w, tool, tip.into(), axis.into(), prev.as_deref(), &o);
    serde_json::to_string(&r).map_err(err)
}

/// Choose 3+2 orientations for the scene's part; JSON with each
/// orientation's axis, joints, G68.2 angles and explanation.
#[pyfunction]
#[pyo3(signature = (machine_src, scene_toml, options_json=None, scene_dir=None, machine_dir=None))]
fn orient(
    machine_src: &str,
    scene_toml: &str,
    options_json: Option<&str>,
    scene_dir: Option<&str>,
    machine_dir: Option<&str>,
) -> PyResult<String> {
    let m = machine(machine_src)?;
    let w = world(&m, Some(scene_toml), scene_dir, machine_dir)?;
    let o: OrientOptions = opts(options_json)?;
    let k = match o.tool {
        Some(t) => kin_for_tool(&m, &w, t)?,
        None => Kinematics::new(m.clone(), m.default_tool_length).map_err(err)?,
    };
    let parts: Vec<&Mesh> = w
        .table_meshes(Role::Part)
        .into_iter()
        .map(|(_, p)| p)
        .collect();
    serde_json::to_string(&select_orientations(&parts, &k, Some(&w), m.wcs_origin, &o)).map_err(err)
}

/// Adaptive roughing of the scene's stock to its part with tool `tool`.
/// `params_json` overrides RoughParams fields (the cutter comes from the
/// tool). With `auto`, 3+2 orientations are chosen automatically; otherwise
/// the tool stays along `axis`. Returns {"cl": CL program, "summary": ...}.
#[pyfunction]
#[pyo3(signature = (machine_src, scene_toml, tool, params_json=None, axis=None, auto=false, scene_dir=None, machine_dir=None))]
#[allow(clippy::too_many_arguments)]
fn rough(
    machine_src: &str,
    scene_toml: &str,
    tool: u32,
    params_json: Option<&str>,
    axis: Option<[f64; 3]>,
    auto: bool,
    scene_dir: Option<&str>,
    machine_dir: Option<&str>,
) -> PyResult<String> {
    let m = machine(machine_src)?;
    let w = world(&m, Some(scene_toml), scene_dir, machine_dir)?;
    let asm = w
        .tool(tool)
        .ok_or_else(|| err(format!("tool T{tool} is not in the scene")))?;
    let cutter = asm
        .cutter_shape()
        .ok_or_else(|| err(format!("tool T{tool} has no cutter section")))?;
    let k = kin_for_tool(&m, &w, tool)?;
    let mut rp: RoughParams = opts(params_json)?;
    rp.cutter = cutter;
    let parts: Vec<&Mesh> = w
        .table_meshes(Role::Part)
        .into_iter()
        .map(|(_, p)| p)
        .collect();
    let mut stock =
        stock_from_world(&w, rp.resolution).ok_or_else(|| err("the scene has no stock body"))?;
    let (moves, summary) = if auto {
        let mut ap = AutoRoughParams {
            rough: rp,
            ..Default::default()
        };
        ap.orient.tool = Some(tool);
        let r = auto_rough(&parts, &mut stock, &k, Some(&w), m.wcs_origin, &ap);
        let summary = serde_json::json!({ "summary": r.summary, "setups": r.setups, "removed_mm3": r.removed_mm3, "remaining_mm3": r.remaining_mm3 });
        (r.moves, summary)
    } else {
        let axis: Vec3 = axis.unwrap_or([0.0, 0.0, 1.0]).into();
        let sol = k
            .inverse(
                &Pose {
                    tip: Vec3::ZERO,
                    axis,
                },
                None,
            )
            .map_err(err)?;
        let obstacles = w.setup_obstacles(&k, &sol.joints);
        let obs: Vec<&Mesh> = obstacles.iter().collect();
        let sections = w.head_sections(&k, Some(tool));
        let f = machining_frame(m.wcs_origin, axis);
        let r = rough_with(&parts, &obs, &sections, &mut stock, &f, m.wcs_origin, &rp);
        let summary = serde_json::json!({ "levels": r.levels, "removed_mm3": r.removed_mm3, "remaining_mm3": r.remaining_mm3, "warnings": r.warnings });
        (r.moves, summary)
    };
    let cl = ClProgram {
        name: format!("rough T{tool}"),
        tool: ClTool {
            number: tool,
            length: asm.def.gauge_length,
            diameter: Some(cutter.radius * 2.0),
        },
        spindle_rpm: None,
        coolant: true,
        moves,
    };
    serde_json::to_string(&serde_json::json!({ "cl": cl, "summary": summary })).map_err(err)
}

/// Continuous 4-axis roughing (`mode` = "rough") or finishing ("finish")
/// around the machine's rotary table axis. Returns {"cl", "summary"}.
#[pyfunction]
#[pyo3(signature = (machine_src, scene_toml, tool, mode="rough", params_json=None, scene_dir=None, machine_dir=None))]
#[allow(clippy::too_many_arguments)]
fn rotary(
    machine_src: &str,
    scene_toml: &str,
    tool: u32,
    mode: &str,
    params_json: Option<&str>,
    scene_dir: Option<&str>,
    machine_dir: Option<&str>,
) -> PyResult<String> {
    let m = machine(machine_src)?;
    let f = RotaryFrame::from_machine(&m)
        .ok_or_else(|| err("the machine needs exactly one rotary axis on the table"))?;
    let w = world(&m, Some(scene_toml), scene_dir, machine_dir)?;
    let asm = w
        .tool(tool)
        .ok_or_else(|| err(format!("tool T{tool} is not in the scene")))?;
    let cutter = asm
        .cutter_shape()
        .ok_or_else(|| err(format!("tool T{tool} has no cutter section")))?;
    let k = kin_for_tool(&m, &w, tool)?;
    let sections = w.head_sections(&k, Some(tool));
    let mut rp: RotaryParams = opts(params_json)?;
    rp.cutter = cutter;
    let parts: Vec<&Mesh> = w
        .table_meshes(Role::Part)
        .into_iter()
        .map(|(_, p)| p)
        .collect();
    let r = match mode {
        "rough" => {
            let mut stock =
                stock_from_world(&w, rp.dx).ok_or_else(|| err("the scene has no stock body"))?;
            rotary_rough(&parts, &sections, &mut stock, &f, m.wcs_origin, &rp)
        }
        "finish" => rotary_finish(&parts, &sections, &f, m.wcs_origin, &rp),
        other => {
            return Err(err(format!(
                "mode must be \"rough\" or \"finish\", not {other:?}"
            )))
        }
    };
    let cl = ClProgram {
        name: format!("rotary {mode} T{tool}"),
        tool: ClTool {
            number: tool,
            length: asm.def.gauge_length,
            diameter: Some(cutter.radius * 2.0),
        },
        spindle_rpm: None,
        coolant: true,
        moves: r.moves.clone(),
    };
    let summary = serde_json::json!({
        "levels": r.levels, "cut_length_mm": r.cut_length_mm, "removed_mm3": r.removed_mm3,
        "remaining_mm3": r.remaining_mm3, "warnings": r.warnings,
    });
    serde_json::to_string(&serde_json::json!({ "cl": cl, "summary": summary })).map_err(err)
}

/// Wrap 2D paths ([[u, v], ...] lists; u axial, v arc length) onto a
/// cylinder of `radius`, cutting `depth` deep. Returns CL moves as JSON.
#[pyfunction]
#[pyo3(signature = (machine_src, paths_json, radius, depth, feed=600.0))]
fn wrap_paths(
    machine_src: &str,
    paths_json: &str,
    radius: f64,
    depth: f64,
    feed: f64,
) -> PyResult<String> {
    let m = machine(machine_src)?;
    let f = RotaryFrame::from_machine(&m)
        .ok_or_else(|| err("the machine needs exactly one rotary axis on the table"))?;
    let paths: Vec<Vec<[f64; 2]>> = serde_json::from_str(paths_json).map_err(err)?;
    serde_json::to_string(&wrap(
        &paths,
        &f,
        m.wcs_origin,
        radius,
        depth,
        0.5,
        feed,
        3.0,
    ))
    .map_err(err)
}

/// Reachability map: directions within `cone_deg` of `axis` and whether
/// each reaches `tip` without collision.
#[pyfunction]
#[pyo3(signature = (machine_src, scene_toml, tool, tip, axis, cone_deg=60.0, samples=200, margin=1.0, scene_dir=None, machine_dir=None))]
#[allow(clippy::too_many_arguments)]
fn reach(
    machine_src: &str,
    scene_toml: &str,
    tool: u32,
    tip: [f64; 3],
    axis: [f64; 3],
    cone_deg: f64,
    samples: usize,
    margin: f64,
    scene_dir: Option<&str>,
    machine_dir: Option<&str>,
) -> PyResult<String> {
    let m = machine(machine_src)?;
    let w = world(&m, Some(scene_toml), scene_dir, machine_dir)?;
    let k = kin_for_tool(&m, &w, tool)?;
    let dirs = cap_directions(axis.into(), cone_deg, samples);
    serde_json::to_string(&reachable_directions(
        &k,
        &w,
        tool,
        tip.into(),
        &dirs,
        margin,
        25.0,
    ))
    .map_err(err)
}

/// Post a CL program (JSON); returns {"gcode", "blocks", "warnings"} as JSON.
#[pyfunction]
#[pyo3(signature = (machine_src, cl_json, options_json=None))]
fn post(machine_src: &str, cl_json: &str, options_json: Option<&str>) -> PyResult<String> {
    let m = machine(machine_src)?;
    let cl: ClProgram = serde_json::from_str(cl_json).map_err(err)?;
    let o: PostOptions = opts(options_json)?;
    serde_json::to_string(&post_cl(&m, &cl, &o).map_err(err)?).map_err(err)
}

/// Forward kinematics for machine joint values; returns {"tip", "axis"}.
#[pyfunction]
fn forward(machine_src: &str, tool_length: f64, joints: Vec<f64>) -> PyResult<String> {
    let k = Kinematics::new(machine(machine_src)?, tool_length).map_err(err)?;
    if joints.len() != k.axis_count() {
        return Err(err(format!("expected {} joint values", k.axis_count())));
    }
    serde_json::to_string(&k.forward(&joints)).map_err(err)
}

/// All inverse-kinematics solutions, best first, as JSON.
#[pyfunction]
#[pyo3(signature = (machine_src, tool_length, tip, axis, prev=None))]
fn inverse(
    machine_src: &str,
    tool_length: f64,
    tip: [f64; 3],
    axis: [f64; 3],
    prev: Option<Vec<f64>>,
) -> PyResult<String> {
    let k = Kinematics::new(machine(machine_src)?, tool_length).map_err(err)?;
    let pose = Pose {
        tip: tip.into(),
        axis: axis.into(),
    };
    serde_json::to_string(&k.inverse_all(&pose, prev.as_deref()).map_err(err)?).map_err(err)
}

#[pymodule(name = "pat")]
fn pat_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(load_machine, m)?)?;
    m.add_function(wrap_pyfunction!(check, m)?)?;
    m.add_function(wrap_pyfunction!(post, m)?)?;
    m.add_function(wrap_pyfunction!(tilt, m)?)?;
    m.add_function(wrap_pyfunction!(reach, m)?)?;
    m.add_function(wrap_pyfunction!(orient, m)?)?;
    m.add_function(wrap_pyfunction!(rough, m)?)?;
    m.add_function(wrap_pyfunction!(rotary, m)?)?;
    m.add_function(wrap_pyfunction!(wrap_paths, m)?)?;
    m.add_function(wrap_pyfunction!(forward, m)?)?;
    m.add_function(wrap_pyfunction!(inverse, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

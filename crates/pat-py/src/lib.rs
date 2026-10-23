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
//! ```

use pat_core::post::{post as post_cl, ClProgram, PostOptions};
use pat_core::sim::{simulate, SimOptions};
use pat_core::{Kinematics, MachineDef, Pose};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

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

/// Simulate G-code; returns the report as JSON.
#[pyfunction]
#[pyo3(signature = (machine_src, program, options_json=None))]
fn check(machine_src: &str, program: &str, options_json: Option<&str>) -> PyResult<String> {
    let m = machine(machine_src)?;
    let o: SimOptions = opts(options_json)?;
    serde_json::to_string(&simulate(&m, program, &o)).map_err(err)
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
    m.add_function(wrap_pyfunction!(forward, m)?)?;
    m.add_function(wrap_pyfunction!(inverse, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

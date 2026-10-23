//! `pat` command-line tool.
//!
//! ```text
//! pat machine machines/trunnion-ac.toml
//! pat check program.ngc --machine machines/trunnion-ac.toml --tool 1=100 --json
//! pat post path.json --machine machines/trunnion-ac.toml --dialect linuxcnc-joint -o out.ngc
//! pat fk --machine m.toml --joints X=0,Y=0,Z=-300,A=30,C=45 --tool-length 100
//! pat ik --machine m.toml --tip 10,0,5 --axis 0,0.5,0.866 --tool-length 100
//! pat check program.ngc -m machines/trunnion-ac.toml --scene examples/vise-setup.toml
//! pat tilt -m machines/trunnion-ac.toml --scene examples/vise-setup.toml --tool 1 --tip 0,0,0
//! ```
//!
//! Every subcommand can print JSON (`--json`) so scripts and AI agents can
//! consume the results directly.

use clap::{Parser, Subcommand, ValueEnum};
use pat_core::access::{cap_directions, reachable_directions, tilt_to_avoid, TiltOptions};
use pat_core::collision::{SceneDef, World};
use pat_core::interp::Severity;
use pat_core::math::Vec3;
use pat_core::post::{post, ClProgram, Dialect, PostOptions};
use pat_core::sim::{simulate, SimOptions};
use pat_core::{Kinematics, MachineDef, Pose};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;

#[derive(Parser)]
#[command(
    name = "pat",
    version,
    about = "Open-source 3/4/5-axis CAM core: kinematics, simulation, posting"
)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Validate a machine file and print a summary.
    Machine {
        file: PathBuf,
        #[arg(long)]
        json: bool,
    },
    /// Simulate a G-code program and report problems.
    Check {
        program: PathBuf,
        #[arg(short, long)]
        machine: PathBuf,
        /// Tool length as NUMBER=LENGTH (repeatable), e.g. --tool 1=100
        #[arg(long = "tool", value_parser = parse_tool)]
        tools: Vec<(u32, f64)>,
        /// Tool length before the first tool change / for unknown tools.
        #[arg(long)]
        tool_length: Option<f64>,
        /// Allowed tool-tip deviation on non-TCP rotary moves (mm).
        #[arg(long, default_value_t = 0.01)]
        tip_tolerance: f64,
        /// Work offset as NAME=X,Y,Z (part zero in the machine base frame),
        /// e.g. --offset G55=150,0,0 (repeatable).
        #[arg(long = "offset", value_parser = parse_offset, allow_hyphen_values = true)]
        offsets: Vec<(String, Vec3)>,
        /// Read P dwell words as milliseconds (Fanuc / Haas).
        #[arg(long)]
        dwell_ms: bool,
        /// Setup file (tools, part, stock, fixtures) for collision checks.
        #[arg(long)]
        scene: Option<PathBuf>,
        /// Tool already in the spindle at program start.
        #[arg(long)]
        initial_tool: Option<u32>,
        /// Include the machine state after every move.
        #[arg(long)]
        trace: bool,
        #[arg(long)]
        json: bool,
    },
    /// Smallest tilt from a preferred tool axis that reaches a point without
    /// collision (prints JSON with an explanation).
    Tilt {
        #[arg(short, long)]
        machine: PathBuf,
        #[arg(long)]
        scene: PathBuf,
        #[arg(long)]
        tool: u32,
        /// Tool tip in WCS coordinates.
        #[arg(long, value_parser = parse_vec3, allow_hyphen_values = true)]
        tip: Vec3,
        /// Preferred tool axis (default straight up).
        #[arg(long, value_parser = parse_vec3, allow_hyphen_values = true, default_value = "0,0,1")]
        axis: Vec3,
        /// Required clearance (mm).
        #[arg(long, default_value_t = 1.0)]
        margin: f64,
        /// Largest tilt to try (degrees).
        #[arg(long, default_value_t = 60.0)]
        max_tilt: f64,
    },
    /// Reachability map: which tool directions around an axis reach a point.
    Reach {
        #[arg(short, long)]
        machine: PathBuf,
        #[arg(long)]
        scene: PathBuf,
        #[arg(long)]
        tool: u32,
        #[arg(long, value_parser = parse_vec3, allow_hyphen_values = true)]
        tip: Vec3,
        #[arg(long, value_parser = parse_vec3, allow_hyphen_values = true, default_value = "0,0,1")]
        axis: Vec3,
        /// Cone half-angle to sample (degrees).
        #[arg(long, default_value_t = 60.0)]
        cone: f64,
        #[arg(long, default_value_t = 200)]
        samples: usize,
        #[arg(long, default_value_t = 1.0)]
        margin: f64,
    },
    /// Post a CL path (JSON) to G-code.
    Post {
        cl: PathBuf,
        #[arg(short, long)]
        machine: PathBuf,
        #[arg(long, value_enum, default_value_t = DialectArg::LinuxcncJoint)]
        dialect: DialectArg,
        /// Tool-tip tolerance for linearisation (mm).
        #[arg(long, default_value_t = 0.005)]
        tolerance: f64,
        #[arg(short, long)]
        output: Option<PathBuf>,
    },
    /// Forward kinematics: joint values → tool tip and axis.
    Fk {
        #[arg(short, long)]
        machine: PathBuf,
        /// Machine joint values, e.g. X=0,Y=0,Z=-300,A=30,C=45
        #[arg(long)]
        joints: String,
        #[arg(long)]
        tool_length: Option<f64>,
    },
    /// Inverse kinematics: tool tip and axis → all joint solutions.
    Ik {
        #[arg(short, long)]
        machine: PathBuf,
        #[arg(long, value_parser = parse_vec3, allow_hyphen_values = true)]
        tip: Vec3,
        #[arg(long, value_parser = parse_vec3, allow_hyphen_values = true)]
        axis: Vec3,
        #[arg(long)]
        tool_length: Option<f64>,
    },
}

#[derive(Clone, Copy, ValueEnum)]
enum DialectArg {
    LinuxcncJoint,
    FanucTcp,
}

fn parse_tool(s: &str) -> Result<(u32, f64), String> {
    let (n, l) = s.split_once('=').ok_or("expected NUMBER=LENGTH")?;
    Ok((
        n.trim().parse().map_err(|_| "bad tool number")?,
        l.trim().parse().map_err(|_| "bad length")?,
    ))
}

fn parse_offset(s: &str) -> Result<(String, Vec3), String> {
    let (n, v) = s.split_once('=').ok_or("expected NAME=X,Y,Z")?;
    Ok((n.trim().to_ascii_uppercase(), parse_vec3(v)?))
}

fn parse_vec3(s: &str) -> Result<Vec3, String> {
    let v: Vec<f64> = s
        .split(',')
        .map(|p| p.trim().parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|e| e.to_string())?;
    if v.len() != 3 {
        return Err("expected x,y,z".into());
    }
    Ok(Vec3::new(v[0], v[1], v[2]))
}

/// Build the collision world for a machine file and an optional scene file.
fn load_world(
    machine_path: &Path,
    m: &MachineDef,
    scene: Option<&Path>,
) -> Result<pat_core::collision::World, String> {
    let scene_def = match scene {
        Some(p) => SceneDef::from_toml(&read(p)?).map_err(|e| format!("{}: {e}", p.display()))?,
        None => SceneDef::default(),
    };
    let kin = Kinematics::new(m.clone(), m.default_tool_length).map_err(|e| e.to_string())?;
    World::build(
        &kin,
        &scene_def,
        scene.and_then(Path::parent),
        machine_path.parent(),
    )
}

fn tool_length_for(world: &World, m: &MachineDef, tool: u32) -> f64 {
    world
        .tool(tool)
        .map_or(m.default_tool_length, |t| t.def.gauge_length)
}

fn read(path: &Path) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))
}

fn load_machine(path: &Path) -> Result<MachineDef, String> {
    let src = read(path)?;
    let res = if path.extension().is_some_and(|e| e == "json") {
        MachineDef::from_json(&src)
    } else {
        MachineDef::from_toml(&src)
    };
    res.map_err(|e| e.to_string())
}

fn to_json<T: serde::Serialize>(v: &T) -> String {
    serde_json::to_string_pretty(v).expect("serialisable")
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(2)
        }
    }
}

fn run(cli: Cli) -> Result<ExitCode, String> {
    match cli.cmd {
        Cmd::Machine { file, json } => {
            let m = load_machine(&file)?;
            if json {
                println!("{}", to_json(&m));
            } else {
                println!("{} — {}", m.name, m.description);
                for a in &m.axes {
                    let lim = match (a.min, a.max) {
                        (None, None) => "continuous".to_string(),
                        (lo, hi) => format!(
                            "{} .. {}",
                            lo.map_or("-inf".into(), |v| v.to_string()),
                            hi.map_or("+inf".into(), |v| v.to_string())
                        ),
                    };
                    println!(
                        "  {:<2} {:?} on {:?}, dir {:?}, limits {lim}",
                        a.name,
                        a.kind,
                        a.chain,
                        <[f64; 3]>::from(a.direction)
                    );
                }
                let k =
                    Kinematics::new(m.clone(), m.default_tool_length).map_err(|e| e.to_string())?;
                println!(
                    "  program zero at machine joints {:?}",
                    k.to_machine(&vec![0.0; m.axes.len()])
                );
            }
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Check {
            program,
            machine,
            tools,
            tool_length,
            tip_tolerance,
            offsets,
            dwell_ms,
            scene,
            initial_tool,
            trace,
            json,
        } => {
            let m = load_machine(&machine)?;
            let src = read(&program)?;
            // Collision checks run when a scene is given or the machine has geometry.
            let world = if scene.is_some() || !m.components.is_empty() {
                Some(Arc::new(load_world(&machine, &m, scene.as_deref())?))
            } else {
                None
            };
            let opts = SimOptions {
                world,
                initial_tool,
                tool_lengths: tools.into_iter().collect(),
                tool_length,
                tip_tolerance,
                record_trace: trace,
                work_offsets: offsets.into_iter().collect(),
                dwell_p_milliseconds: dwell_ms,
                ..Default::default()
            };
            let r = simulate(&m, &src, &opts);
            if json {
                println!("{}", to_json(&r));
            } else {
                for i in &r.issues {
                    let sev = match i.severity {
                        Severity::Error => "error",
                        Severity::Warning => "warning",
                        Severity::Info => "info",
                    };
                    println!(
                        "{}:{}: {sev} [{}] {}",
                        program.display(),
                        i.line,
                        i.code,
                        i.message
                    );
                }
                println!(
                    "{}: {} errors, {} warnings, {} moves, cutting {:.1}s, rapid {:.1}s, cut length {:.1} mm",
                    r.machine, r.errors, r.warnings, r.moves, r.cutting_time_s, r.rapid_time_s, r.cutting_length_mm
                );
                println!(
                    "  cycle time {:.1}s ({} model, dwell {:.1}s)",
                    r.cycle_time_s, r.cycle_time_model, r.dwell_time_s
                );
                for a in &r.axis_ranges {
                    println!("  {:<2} {:>10.3} .. {:<10.3}", a.axis, a.min, a.max);
                }
            }
            Ok(if r.ok {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(1)
            })
        }
        Cmd::Post {
            cl,
            machine,
            dialect,
            tolerance,
            output,
        } => {
            let m = load_machine(&machine)?;
            let prog: ClProgram =
                serde_json::from_str(&read(&cl)?).map_err(|e| format!("{}: {e}", cl.display()))?;
            let opts = PostOptions {
                dialect: match dialect {
                    DialectArg::LinuxcncJoint => Dialect::LinuxcncJoint,
                    DialectArg::FanucTcp => Dialect::FanucTcp,
                },
                tolerance,
                ..Default::default()
            };
            let out = post(&m, &prog, &opts).map_err(|e| e.to_string())?;
            for w in &out.warnings {
                eprintln!("warning: {w}");
            }
            match output {
                Some(p) => {
                    std::fs::write(&p, &out.gcode).map_err(|e| format!("{}: {e}", p.display()))?;
                    eprintln!("wrote {} blocks to {}", out.blocks, p.display());
                }
                None => print!("{}", out.gcode),
            }
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Fk {
            machine,
            joints,
            tool_length,
        } => {
            let m = load_machine(&machine)?;
            let k = Kinematics::new(m.clone(), tool_length.unwrap_or(m.default_tool_length))
                .map_err(|e| e.to_string())?;
            let mut q = vec![0.0; m.axes.len()];
            for part in joints.split(',').filter(|p| !p.trim().is_empty()) {
                let (n, v) = part
                    .split_once('=')
                    .ok_or("joints must look like X=1,A=30")?;
                let i = m
                    .axis_index(n.trim())
                    .ok_or_else(|| format!("no axis {n}"))?;
                q[i] = v.trim().parse().map_err(|_| format!("bad value for {n}"))?;
            }
            let pose = k.forward(&q);
            println!(
                "{}",
                to_json(&serde_json::json!({
                    "tip": pose.tip, "axis": pose.axis,
                    "program": k.to_program(&q),
                    "limit_violations": k.limit_violations(&q).iter().map(|(i, v)| (m.axes[*i].name.clone(), *v)).collect::<Vec<_>>(),
                }))
            );
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Tilt {
            machine,
            scene,
            tool,
            tip,
            axis,
            margin,
            max_tilt,
        } => {
            let m = load_machine(&machine)?;
            let world = load_world(&machine, &m, Some(&scene))?;
            let k = Kinematics::new(m.clone(), tool_length_for(&world, &m, tool))
                .map_err(|e| e.to_string())?;
            let opts = TiltOptions {
                margin,
                max_tilt_deg: max_tilt,
                ..Default::default()
            };
            let r = tilt_to_avoid(&k, &world, tool, tip, axis, None, &opts);
            println!("{}", to_json(&r));
            Ok(if r.axis.is_some() {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(1)
            })
        }
        Cmd::Reach {
            machine,
            scene,
            tool,
            tip,
            axis,
            cone,
            samples,
            margin,
        } => {
            let m = load_machine(&machine)?;
            let world = load_world(&machine, &m, Some(&scene))?;
            let k = Kinematics::new(m.clone(), tool_length_for(&world, &m, tool))
                .map_err(|e| e.to_string())?;
            let dirs = cap_directions(axis, cone, samples);
            let map = reachable_directions(&k, &world, tool, tip, &dirs, margin, 25.0);
            let reachable = map.iter().filter(|d| d.reachable).count();
            println!(
                "{}",
                to_json(&serde_json::json!({
                    "reachable": reachable,
                    "total": map.len(),
                    "directions": map,
                }))
            );
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Ik {
            machine,
            tip,
            axis,
            tool_length,
        } => {
            let m = load_machine(&machine)?;
            let k = Kinematics::new(m.clone(), tool_length.unwrap_or(m.default_tool_length))
                .map_err(|e| e.to_string())?;
            let sols = k
                .inverse_all(&Pose { tip, axis }, None)
                .map_err(|e| e.to_string())?;
            println!("{}", to_json(&sols));
            Ok(ExitCode::SUCCESS)
        }
    }
}

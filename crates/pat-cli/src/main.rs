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
use pat_core::collision::{Role, SceneDef, World};
use pat_core::interp::Severity;
use pat_core::math::Vec3;
use pat_core::orient::{select_orientations, OrientOptions};
use pat_core::plan::{auto_rough, stock_from_world, AutoRoughParams};
use pat_core::post::{post, ClProgram, Dialect, PostOptions};
use pat_core::rough::{machining_frame, rough_with, RoughParams};
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
        /// Simulate material removal on the scene's stock (gouges, rapids
        /// into stock, holders rubbing, removed volume).
        #[arg(long)]
        stock: bool,
        /// Voxel size for stock simulation (mm).
        #[arg(long, default_value_t = 0.5)]
        stock_resolution: f64,
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
    /// Choose 3+2 orientations that reach the scene's part (JSON).
    Orient {
        #[arg(short, long)]
        machine: PathBuf,
        #[arg(long)]
        scene: PathBuf,
        /// Tool for the clearance check.
        #[arg(long)]
        tool: Option<u32>,
        /// Usable flute length for flank-cut walls (mm).
        #[arg(long, default_value_t = 25.0)]
        max_wall_depth: f64,
        /// Largest tilt from vertical to consider (degrees).
        #[arg(long, default_value_t = 120.0)]
        max_tilt: f64,
    },
    /// Adaptive roughing of the scene's stock down to its part. Writes CL
    /// data (JSON) for `pat post`, or G-code directly with --gcode.
    Rough {
        #[arg(short, long)]
        machine: PathBuf,
        #[arg(long)]
        scene: PathBuf,
        #[arg(long)]
        tool: u32,
        /// Fixed tool axis (WCS); default straight down.
        #[arg(long, value_parser = parse_vec3, allow_hyphen_values = true)]
        axis: Option<Vec3>,
        /// Choose 3+2 orientations automatically and rough from each.
        #[arg(long)]
        auto: bool,
        #[arg(long, default_value_t = 3.0)]
        stepdown: f64,
        /// Radial width of cut (mm); default 15% of the cutter diameter.
        #[arg(long)]
        stepover: Option<f64>,
        #[arg(long, default_value_t = 0.3)]
        leave: f64,
        #[arg(long, default_value_t = 0.5)]
        resolution: f64,
        #[arg(long, default_value_t = 2000.0)]
        feed: f64,
        #[arg(long, default_value_t = 10000.0)]
        rpm: f64,
        /// Output CL JSON path (default: stdout).
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Also post straight to G-code at this path.
        #[arg(long)]
        gcode: Option<PathBuf>,
        #[arg(long, value_enum, default_value_t = DialectArg::FanucTcp)]
        dialect: DialectArg,
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
            stock,
            stock_resolution,
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
                simulate_stock: stock,
                stock_resolution,
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
                if let Some(st) = &r.stock {
                    println!(
                        "  stock: removed {:.0} mm³ of {:.0} mm³, {:.0} mm³ left ({} mm voxels)",
                        st.removed_mm3, st.initial_mm3, st.remaining_mm3, st.resolution_mm
                    );
                }
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
        Cmd::Orient {
            machine,
            scene,
            tool,
            max_wall_depth,
            max_tilt,
        } => {
            let m = load_machine(&machine)?;
            let world = load_world(&machine, &m, Some(&scene))?;
            let len = tool.map_or(m.default_tool_length, |t| tool_length_for(&world, &m, t));
            let k = Kinematics::new(m.clone(), len).map_err(|e| e.to_string())?;
            let parts: Vec<&pat_core::geom::Mesh> = world
                .table_meshes(Role::Part)
                .into_iter()
                .map(|(_, mesh)| mesh)
                .collect();
            if parts.is_empty() {
                return Err("the scene has no body with role = \"part\"".into());
            }
            let opts = OrientOptions {
                tool,
                max_wall_depth,
                max_tilt_deg: max_tilt,
                ..Default::default()
            };
            let r = select_orientations(&parts, &k, Some(&world), m.wcs_origin, &opts);
            for o in &r.orientations {
                eprintln!("{}", o.explanation);
            }
            println!("{}", to_json(&r));
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Rough {
            machine,
            scene,
            tool,
            axis,
            auto,
            stepdown,
            stepover,
            leave,
            resolution,
            feed,
            rpm,
            output,
            gcode,
            dialect,
        } => {
            let m = load_machine(&machine)?;
            let world = load_world(&machine, &m, Some(&scene))?;
            let asm = world
                .tool(tool)
                .ok_or_else(|| format!("tool T{tool} is not in the scene"))?;
            let cutter = asm
                .cutter_shape()
                .ok_or_else(|| format!("tool T{tool} has no cutter section"))?;
            let k = Kinematics::new(m.clone(), asm.def.gauge_length).map_err(|e| e.to_string())?;
            let parts: Vec<&pat_core::geom::Mesh> = world
                .table_meshes(Role::Part)
                .into_iter()
                .map(|(_, mesh)| mesh)
                .collect();
            let mut stock = stock_from_world(&world, resolution)
                .ok_or("the scene has no body with role = \"stock\"")?;
            let rp = RoughParams {
                cutter,
                stepdown,
                stepover: stepover.unwrap_or(cutter.radius * 0.3),
                leave,
                resolution,
                feed,
                plunge_feed: feed * 0.3,
                link_feed: feed * 1.5,
                ..Default::default()
            };
            let moves = if auto {
                let mut ap = AutoRoughParams {
                    rough: rp,
                    ..Default::default()
                };
                ap.orient.tool = Some(tool);
                let r = auto_rough(&parts, &mut stock, &k, Some(&world), m.wcs_origin, &ap);
                for s in &r.setups {
                    eprintln!("{}", s.orientation.explanation);
                    for w in &s.warnings {
                        eprintln!("  warning: {w}");
                    }
                }
                eprintln!("{}", r.summary);
                r.moves
            } else {
                let axis = axis.unwrap_or(Vec3::Z);
                let sol = k
                    .inverse(
                        &Pose {
                            tip: Vec3::ZERO,
                            axis,
                        },
                        None,
                    )
                    .map_err(|e| format!("axis {axis:?} is not reachable: {e}"))?;
                let obstacles = world.setup_obstacles(&k, &sol.joints);
                let obs: Vec<&pat_core::geom::Mesh> = obstacles.iter().collect();
                let sections = world.head_sections(&k, Some(tool));
                let f = machining_frame(m.wcs_origin, axis);
                let r = rough_with(&parts, &obs, &sections, &mut stock, &f, m.wcs_origin, &rp);
                for w in &r.warnings {
                    eprintln!("warning: {w}");
                }
                eprintln!(
                    "{} levels, removed {:.0} mm³, {:.0} mm³ of stock left",
                    r.levels.len(),
                    r.removed_mm3,
                    r.remaining_mm3
                );
                r.moves
            };
            let cl = ClProgram {
                name: format!("rough T{tool}"),
                tool: pat_core::post::ClTool {
                    number: tool,
                    length: asm.def.gauge_length,
                    diameter: Some(cutter.radius * 2.0),
                },
                spindle_rpm: Some(rpm),
                coolant: true,
                moves,
            };
            let json = to_json(&cl);
            match output {
                Some(p) => std::fs::write(&p, json).map_err(|e| format!("{}: {e}", p.display()))?,
                None if gcode.is_none() => println!("{json}"),
                None => {}
            }
            if let Some(g) = gcode {
                let opts = PostOptions {
                    dialect: match dialect {
                        DialectArg::LinuxcncJoint => Dialect::LinuxcncJoint,
                        DialectArg::FanucTcp => Dialect::FanucTcp,
                    },
                    ..Default::default()
                };
                let out = post(&m, &cl, &opts).map_err(|e| e.to_string())?;
                std::fs::write(&g, &out.gcode).map_err(|e| format!("{}: {e}", g.display()))?;
                eprintln!("wrote {} blocks to {}", out.blocks, g.display());
            }
            Ok(ExitCode::SUCCESS)
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

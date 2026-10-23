//! `pat` command-line tool.
//!
//! ```text
//! pat machine machines/trunnion-ac.toml
//! pat check program.ngc --machine machines/trunnion-ac.toml --tool 1=100 --json
//! pat post path.json --machine machines/trunnion-ac.toml --dialect linuxcnc-joint -o out.ngc
//! pat fk --machine m.toml --joints X=0,Y=0,Z=-300,A=30,C=45 --tool-length 100
//! pat ik --machine m.toml --tip 10,0,5 --axis 0,0.5,0.866 --tool-length 100
//! ```
//!
//! Every subcommand can print JSON (`--json`) so scripts and AI agents can
//! consume the results directly.

use clap::{Parser, Subcommand, ValueEnum};
use pat_core::interp::Severity;
use pat_core::math::Vec3;
use pat_core::post::{post, ClProgram, Dialect, PostOptions};
use pat_core::sim::{simulate, SimOptions};
use pat_core::{Kinematics, MachineDef, Pose};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

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
        /// Include the machine state after every move.
        #[arg(long)]
        trace: bool,
        #[arg(long)]
        json: bool,
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
            trace,
            json,
        } => {
            let m = load_machine(&machine)?;
            let src = read(&program)?;
            let opts = SimOptions {
                tool_lengths: tools.into_iter().collect(),
                tool_length,
                tip_tolerance,
                record_trace: trace,
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

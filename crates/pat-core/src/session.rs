//! A stateful workspace for agents and other front ends.
//!
//! A [`Session`] holds the loaded machine, the job scene (tools, part, stock,
//! fixtures) and the programs generated so far. Every capability is a method
//! taking and returning JSON, so an MCP server, the Fusion add-in or a script
//! can drive it the same way.
//!
//! Safety rules built into the API, not left to the caller:
//!
//! - **Every generated program is verified.** It is posted and replayed with
//!   collision checks (and stock simulation when the scene has stock). The
//!   result is stored with the program, with a plain-language remedy for each
//!   kind of issue.
//! - **Unverified programs are not written out.** [`Session::save_program`]
//!   refuses programs whose verification found errors unless `force` is set,
//!   and the response says so.
//! - **File access stays inside the workspace root** given at construction.

use crate::collision::{Role, SceneDef, World};
use crate::fiveaxis::{surface_finish, swarf, SurfaceParams, SwarfParams};
use crate::geom::Mesh;
use crate::kinematics::{Kinematics, Pose};
use crate::machine::MachineDef;
use crate::math::Vec3;
use crate::orient::{select_orientations, OrientOptions};
use crate::plan::{auto_rough, stock_from_world, AutoRoughParams};
use crate::post::{post, ClProgram, ClTool, Dialect, PostOptions};
use crate::rotary::{rotary_finish, rotary_rough, RotaryFrame, RotaryParams};
use crate::rough::{machining_frame, rough_with, RoughParams};
use crate::sim::{simulate, Report, SimOptions};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

pub type Result<T> = std::result::Result<T, String>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Program {
    pub id: usize,
    pub name: String,
    pub kind: String,
    pub cl: ClProgram,
    pub dialect: Dialect,
    pub gcode: String,
    pub verified: bool,
    pub verification: Value,
    /// Generator summary (levels, setups, adjustments, warnings).
    pub summary: Value,
}

pub struct Session {
    root: PathBuf,
    machine: Option<(MachineDef, PathBuf)>,
    scene: Option<(SceneDef, Option<PathBuf>)>,
    world: Option<Arc<World>>,
    programs: Vec<Program>,
    /// Voxel size for stock simulation during verification (mm).
    pub stock_resolution: f64,
}

/// Plain-language remedy for a simulator issue code.
pub fn remedy(code: &str) -> &'static str {
    match code {
        "collision" => "Something in the tool assembly or machine touches another body. Change the tool axis or orientation, use a longer or slimmer tool or holder, or check the fixture positions.",
        "rapid_into_part" | "rapid_into_stock" => "A rapid move goes through material. Add clearance, raise the safe height, or turn the move into a feed move.",
        "gouge" => "The cutter goes into the finished part. Increase the stock to leave, use a finer resolution, or check the part geometry and tool definition.",
        "holder_rubs_stock" => "The shank or holder touches remaining material. Use a longer tool, rough in more steps, or tilt the tool.",
        "axis_limit" => "An axis goes past its travel limit. Pick another orientation, move part zero, or use a shorter tool.",
        "near_singularity" => "The tool axis passes the machine's singular direction while a rotary swings. Add lead or tilt, change the pass direction, or turn on the singularity lock.",
        "rotary_flip" => "A rotary axis swings a long way during a cut. Change the pass direction or orientation so the rotaries stay on one side.",
        "tip_deviation" => "Between points the tool tip strays from the straight path. Post with tool-centre-point control or tighter linearisation.",
        "missing_feed" => "A feed move has no feed rate. Set F before cutting (every block in G93 mode).",
        "work_offset" => "The program uses a work offset the machine file does not define. Add it to [work_offsets].",
        "tool_length" | "tool_geometry" => "A tool in the program is missing from the setup. Add it with its geometry and gauge length.",
        "unsupported" | "unknown_gcode" | "canned_cycle" => "The program uses codes the simulator does not model, so check that part by other means.",
        _ => "See the issue message.",
    }
}

fn dialect_from(s: Option<&str>) -> Result<Option<Dialect>> {
    match s {
        None => Ok(None),
        Some("linuxcnc-joint") => Ok(Some(Dialect::LinuxcncJoint)),
        Some("fanuc-tcp") => Ok(Some(Dialect::FanucTcp)),
        Some(other) => Err(format!(
            "unknown dialect '{other}' (use linuxcnc-joint or fanuc-tcp)"
        )),
    }
}

fn from_json<T: for<'de> Deserialize<'de> + Default>(v: Option<&Value>) -> Result<T> {
    match v {
        None | Some(Value::Null) => Ok(T::default()),
        Some(v) => serde_json::from_value(v.clone()).map_err(|e| format!("bad parameters: {e}")),
    }
}

impl Session {
    pub fn new(root: impl Into<PathBuf>) -> Session {
        let root = root.into();
        let root = root.canonicalize().unwrap_or(root);
        Session {
            root,
            machine: None,
            scene: None,
            world: None,
            programs: Vec::new(),
            stock_resolution: 0.5,
        }
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Resolve a path inside the workspace root (absolute or relative).
    fn resolve(&self, p: &str, must_exist: bool) -> Result<PathBuf> {
        let raw = Path::new(p);
        let joined = if raw.is_absolute() {
            raw.to_path_buf()
        } else {
            self.root.join(raw)
        };
        let check = if must_exist {
            joined.canonicalize().map_err(|e| format!("{p}: {e}"))?
        } else {
            let parent = joined
                .parent()
                .ok_or_else(|| format!("{p}: no parent directory"))?;
            let parent = parent.canonicalize().map_err(|e| format!("{p}: {e}"))?;
            parent.join(
                joined
                    .file_name()
                    .ok_or_else(|| format!("{p}: not a file path"))?,
            )
        };
        if !check.starts_with(&self.root) {
            return Err(format!(
                "{p} is outside the workspace root {}",
                self.root.display()
            ));
        }
        Ok(check)
    }

    fn machine(&self) -> Result<&MachineDef> {
        self.machine
            .as_ref()
            .map(|(m, _)| m)
            .ok_or_else(|| "no machine loaded; call load_machine first".into())
    }

    fn world(&self) -> Result<Arc<World>> {
        self.world
            .clone()
            .ok_or_else(|| "no setup loaded; call load_scene first".into())
    }

    fn rebuild_world(&mut self) -> Result<()> {
        let Some((m, mdir)) = &self.machine else {
            return Ok(());
        };
        let empty = (SceneDef::default(), None);
        let (scene, sdir) = self.scene.as_ref().unwrap_or(&empty);
        let kin = Kinematics::new(m.clone(), m.default_tool_length).map_err(|e| e.to_string())?;
        let w = World::build(&kin, scene, sdir.as_deref(), Some(mdir))?;
        self.world = Some(Arc::new(w));
        Ok(())
    }

    /// Load a machine from a file in the workspace, or from TOML text.
    pub fn load_machine(&mut self, path: Option<&str>, toml: Option<&str>) -> Result<Value> {
        let (def, dir) = match (path, toml) {
            (Some(p), _) => {
                let full = self.resolve(p, true)?;
                let src = std::fs::read_to_string(&full).map_err(|e| format!("{p}: {e}"))?;
                let def = MachineDef::from_toml(&src).map_err(|e| e.to_string())?;
                (
                    def,
                    full.parent()
                        .map(Path::to_path_buf)
                        .unwrap_or_else(|| self.root.clone()),
                )
            }
            (None, Some(t)) => (
                MachineDef::from_toml(t).map_err(|e| e.to_string())?,
                self.root.clone(),
            ),
            _ => return Err("give a machine file path or TOML text".into()),
        };
        self.machine = Some((def, dir));
        self.rebuild_world()?;
        Ok(self.describe())
    }

    /// Load the job setup (tools, part, stock, fixtures) from a file or TOML.
    pub fn load_scene(&mut self, path: Option<&str>, toml: Option<&str>) -> Result<Value> {
        let (scene, dir) = match (path, toml) {
            (Some(p), _) => {
                let full = self.resolve(p, true)?;
                let src = std::fs::read_to_string(&full).map_err(|e| format!("{p}: {e}"))?;
                (
                    SceneDef::from_toml(&src)?,
                    full.parent().map(Path::to_path_buf),
                )
            }
            (None, Some(t)) => (SceneDef::from_toml(t)?, Some(self.root.clone())),
            _ => return Err("give a scene file path or TOML text".into()),
        };
        self.scene = Some((scene, dir));
        self.rebuild_world()?;
        Ok(self.describe())
    }

    /// What is loaded: machine axes and limits, tools, bodies, programs.
    pub fn describe(&self) -> Value {
        let machine = self.machine.as_ref().map(|(m, _)| {
            json!({
                "name": m.name,
                "description": m.description,
                "axes": m.axes.iter().map(|a| json!({
                    "name": a.name, "kind": a.kind, "chain": a.chain,
                    "min": a.min, "max": a.max,
                    "continuous": a.is_continuous(),
                })).collect::<Vec<_>>(),
                "rotary_axes": m.rotary_count(),
                "has_table_rotary_axis": RotaryFrame::from_machine(m).is_some(),
                "machine_bodies": m.components.iter().map(|c| c.name.clone()).collect::<Vec<_>>(),
            })
        });
        let scene = self.scene.as_ref().map(|(s, _)| {
            json!({
                "tools": s.tools.iter().map(|t| json!({
                    "number": t.number, "name": t.name, "gauge_length": t.gauge_length,
                    "segments": t.segments,
                })).collect::<Vec<_>>(),
                "bodies": s.bodies.iter().map(|b| json!({"name": b.name, "role": b.role})).collect::<Vec<_>>(),
            })
        });
        json!({
            "workspace": self.root.display().to_string(),
            "machine": machine,
            "setup": scene,
            "programs": self.programs.iter().map(|p| json!({
                "id": p.id, "name": p.name, "kind": p.kind, "moves": p.cl.moves.len(),
                "verified": p.verified,
            })).collect::<Vec<_>>(),
        })
    }

    fn sim_options(&self, stock: bool) -> SimOptions {
        let mut tool_lengths = BTreeMap::new();
        if let Some((s, _)) = &self.scene {
            for t in &s.tools {
                tool_lengths.insert(t.number, t.gauge_length);
            }
        }
        let has_stock = self
            .world
            .as_ref()
            .is_some_and(|w| !w.table_meshes(Role::Stock).is_empty());
        SimOptions {
            world: self.world.clone(),
            tool_lengths,
            simulate_stock: stock && has_stock,
            stock_resolution: self.stock_resolution,
            gouge_tolerance: 0.1,
            ..Default::default()
        }
    }

    /// Condense a simulator report for an agent: counts, the first issues,
    /// one remedy per issue kind.
    pub fn summarize_report(r: &Report, max_issues: usize) -> Value {
        let mut counts: BTreeMap<String, usize> = BTreeMap::new();
        for i in &r.issues {
            *counts.entry(i.code.clone()).or_default() += 1;
        }
        let remedies: BTreeMap<String, &str> =
            counts.keys().map(|c| (c.clone(), remedy(c))).collect();
        json!({
            "ok": r.ok,
            "errors": r.errors,
            "warnings": r.warnings,
            "issue_counts": counts,
            "first_issues": r.issues.iter().take(max_issues).collect::<Vec<_>>(),
            "remedies": remedies,
            "cycle_time_s": (r.cycle_time_s * 10.0).round() / 10.0,
            "cycle_time_model": r.cycle_time_model,
            "cutting_length_mm": r.cutting_length_mm.round(),
            "stock": r.stock,
            "axis_ranges": r.axis_ranges,
        })
    }

    /// Simulate G-code text (or a workspace file) against the loaded machine
    /// and setup.
    pub fn check_gcode(
        &self,
        text: Option<&str>,
        path: Option<&str>,
        simulate_stock: bool,
    ) -> Result<Value> {
        let m = self.machine()?;
        let src = match (text, path) {
            (Some(t), _) => t.to_string(),
            (None, Some(p)) => {
                std::fs::read_to_string(self.resolve(p, true)?).map_err(|e| format!("{p}: {e}"))?
            }
            _ => return Err("give G-code text or a file path".into()),
        };
        let r = simulate(m, &src, &self.sim_options(simulate_stock));
        Ok(Self::summarize_report(&r, 20))
    }

    fn tool_kin(&self, tool: u32) -> Result<(Kinematics, crate::stock::CutterShape, f64)> {
        let m = self.machine()?;
        let w = self.world()?;
        let asm = w
            .tool(tool)
            .ok_or_else(|| format!("tool T{tool} is not in the setup"))?;
        let cutter = asm
            .cutter_shape()
            .ok_or_else(|| format!("tool T{tool} has no cutter section"))?;
        let len = asm.def.gauge_length;
        Ok((
            Kinematics::new(m.clone(), len).map_err(|e| e.to_string())?,
            cutter,
            len,
        ))
    }

    fn parts(&self) -> Result<(Arc<World>, Vec<Mesh>)> {
        let w = self.world()?;
        let parts: Vec<Mesh> = w
            .table_meshes(Role::Part)
            .into_iter()
            .map(|(_, m)| m.clone())
            .collect();
        if parts.is_empty() {
            return Err("the setup has no body with role = \"part\"".into());
        }
        Ok((w, parts))
    }

    /// Post and verify a CL program, store it, and return its summary.
    fn add_program(
        &mut self,
        kind: &str,
        cl: ClProgram,
        dialect: Dialect,
        summary: Value,
    ) -> Result<Value> {
        let m = self.machine()?.clone();
        let out = post(
            &m,
            &cl,
            &PostOptions {
                dialect,
                ..Default::default()
            },
        )
        .map_err(|e| e.to_string())?;
        let report = simulate(&m, &out.gcode, &self.sim_options(true));
        let verification = Self::summarize_report(&report, 12);
        let id = self.programs.len() + 1;
        let verified = report.ok;
        self.programs.push(Program {
            id,
            name: cl.name.clone(),
            kind: kind.into(),
            cl,
            dialect,
            gcode: out.gcode,
            verified,
            verification: verification.clone(),
            summary: summary.clone(),
        });
        Ok(json!({
            "program_id": id,
            "kind": kind,
            "blocks": out.blocks,
            "post_warnings": out.warnings.iter().take(5).collect::<Vec<_>>(),
            "summary": summary,
            "verified": verified,
            "verification": verification,
        }))
    }

    fn default_dialect(&self) -> Dialect {
        // Tool-centre-point output for 5-axis heads/tables, joint space
        // otherwise (LinuxCNC-style 4th axes).
        match self.machine.as_ref().map(|(m, _)| m.rotary_count()) {
            Some(2) => Dialect::FanucTcp,
            _ => Dialect::LinuxcncJoint,
        }
    }

    fn cl(
        &self,
        name: String,
        tool: u32,
        len: f64,
        d: f64,
        moves: Vec<crate::post::ClMove>,
    ) -> ClProgram {
        ClProgram {
            name,
            tool: ClTool {
                number: tool,
                length: len,
                diameter: Some(d),
            },
            spindle_rpm: Some(10000.0),
            coolant: true,
            moves,
        }
    }

    /// Choose 3+2 orientations for the part.
    pub fn plan_orientations(&self, tool: Option<u32>, options: Option<&Value>) -> Result<Value> {
        let (w, parts) = self.parts()?;
        let m = self.machine()?;
        let mut o: OrientOptions = from_json(options)?;
        if tool.is_some() {
            o.tool = tool;
        }
        let kin = match o.tool {
            Some(t) => self.tool_kin(t)?.0,
            None => Kinematics::new(m.clone(), m.default_tool_length).map_err(|e| e.to_string())?,
        };
        let refs: Vec<&Mesh> = parts.iter().collect();
        let r = select_orientations(&refs, &kin, Some(&w), m.wcs_origin, &o);
        Ok(json!({
            "orientations": r.orientations,
            "uncovered_fraction": r.uncovered,
            "surface_area_mm2": r.surface_area_mm2,
            "ignored_area_mm2": r.ignored_area_mm2,
        }))
    }

    /// Adaptive roughing: along `axis` (default straight down) or with
    /// automatic 3+2 orientations. Parameters override RoughParams.
    pub fn rough(
        &mut self,
        tool: u32,
        axis: Option<Vec3>,
        auto: bool,
        params: Option<&Value>,
        dialect: Option<&str>,
    ) -> Result<Value> {
        let (w, parts) = self.parts()?;
        let (kin, cutter, len) = self.tool_kin(tool)?;
        let m = self.machine()?.clone();
        let mut rp: RoughParams = from_json(params)?;
        rp.cutter = cutter;
        if params.and_then(|p| p.get("stepover")).is_none() {
            rp.stepover = cutter.radius * 0.3;
        }
        let refs: Vec<&Mesh> = parts.iter().collect();
        let mut stock = stock_from_world(&w, rp.resolution)
            .ok_or("the setup has no body with role = \"stock\"")?;
        let (moves, summary) = if auto {
            let mut ap = AutoRoughParams {
                rough: rp,
                ..Default::default()
            };
            ap.orient.tool = Some(tool);
            let r = auto_rough(&refs, &mut stock, &kin, Some(&w), m.wcs_origin, &ap);
            let setups: Vec<Value> = r
                .setups
                .iter()
                .map(|s| json!({"explanation": s.orientation.explanation, "removed_mm3": s.removed_mm3.round(), "warnings": s.warnings}))
                .collect();
            (
                r.moves,
                json!({"summary": r.summary, "setups": setups, "remaining_mm3": r.remaining_mm3.round()}),
            )
        } else {
            let axis = axis.unwrap_or(Vec3::Z);
            let sol = kin
                .inverse(
                    &Pose {
                        tip: Vec3::ZERO,
                        axis,
                    },
                    None,
                )
                .map_err(|e| format!("axis {axis:?} is not reachable: {e}"))?;
            let obstacles = w.setup_obstacles(&kin, &sol.joints);
            let obs: Vec<&Mesh> = obstacles.iter().collect();
            let sections = w.head_sections(&kin, Some(tool));
            let f = machining_frame(m.wcs_origin, axis);
            let r = rough_with(&refs, &obs, &sections, &mut stock, &f, m.wcs_origin, &rp);
            let levels: Vec<f64> = r
                .levels
                .iter()
                .map(|l| (l.z * 100.0).round() / 100.0)
                .collect();
            (
                r.moves,
                json!({"levels": levels, "removed_mm3": r.removed_mm3.round(), "remaining_mm3": r.remaining_mm3.round(), "warnings": r.warnings}),
            )
        };
        let d = dialect_from(dialect)?.unwrap_or(self.default_dialect());
        let cl = self.cl(
            format!("rough T{tool}"),
            tool,
            len,
            cutter.radius * 2.0,
            moves,
        );
        self.add_program(if auto { "rough-3+2" } else { "rough" }, cl, d, summary)
    }

    /// Continuous 4-axis roughing or finishing.
    pub fn rotary(
        &mut self,
        tool: u32,
        finish: bool,
        params: Option<&Value>,
        dialect: Option<&str>,
    ) -> Result<Value> {
        let (w, parts) = self.parts()?;
        let (kin, cutter, len) = self.tool_kin(tool)?;
        let m = self.machine()?.clone();
        let f = RotaryFrame::from_machine(&m)
            .ok_or("the machine needs exactly one rotary axis on the table")?;
        let mut rp: RotaryParams = from_json(params)?;
        rp.cutter = cutter;
        let sections = w.head_sections(&kin, Some(tool));
        let refs: Vec<&Mesh> = parts.iter().collect();
        let r = if finish {
            rotary_finish(&refs, &sections, &f, m.wcs_origin, &rp)
        } else {
            let mut stock =
                stock_from_world(&w, rp.dx).ok_or("the setup has no body with role = \"stock\"")?;
            rotary_rough(&refs, &sections, &mut stock, &f, m.wcs_origin, &rp)
        };
        let summary = json!({"levels": r.levels, "cut_length_mm": r.cut_length_mm.round(), "removed_mm3": r.removed_mm3.round(), "warnings": r.warnings});
        let d = dialect_from(dialect)?.unwrap_or(Dialect::LinuxcncJoint);
        let cl = self.cl(
            format!("rotary {} T{tool}", if finish { "finish" } else { "rough" }),
            tool,
            len,
            cutter.radius * 2.0,
            r.moves,
        );
        self.add_program(
            if finish {
                "rotary-finish"
            } else {
                "rotary-rough"
            },
            cl,
            d,
            summary,
        )
    }

    /// 5-axis surface finishing with tool-axis control.
    pub fn surface_finish(
        &mut self,
        tool: u32,
        params: Option<&Value>,
        dialect: Option<&str>,
    ) -> Result<Value> {
        let (w, parts) = self.parts()?;
        let (kin, cutter, len) = self.tool_kin(tool)?;
        let m = self.machine()?.clone();
        let mut sp: SurfaceParams = from_json(params)?;
        sp.cutter = cutter;
        let refs: Vec<&Mesh> = parts.iter().collect();
        let r = surface_finish(&refs, &kin, Some(&w), Some(tool), m.wcs_origin, &sp);
        let summary = json!({
            "summary": r.summary, "points": r.points, "adjusted": r.adjusted, "skipped": r.skipped,
            "max_axis_step_deg": (r.max_axis_step_deg * 10.0).round() / 10.0, "warnings": r.warnings,
        });
        let d = dialect_from(dialect)?.unwrap_or(self.default_dialect());
        let cl = self.cl(
            format!("5-axis surface T{tool}"),
            tool,
            len,
            cutter.radius * 2.0,
            r.moves,
        );
        self.add_program("surface-5x", cl, d, summary)
    }

    /// Swarf milling between two rails (WCS points).
    pub fn swarf(
        &mut self,
        tool: u32,
        bottom: Vec<Vec3>,
        top: Vec<Vec3>,
        params: Option<&Value>,
        dialect: Option<&str>,
    ) -> Result<Value> {
        let (kin, cutter, len) = self.tool_kin(tool)?;
        let w = self.world()?;
        let m = self.machine()?.clone();
        let mut sp: SwarfParams = from_json(params)?;
        sp.cutter = cutter;
        let r = swarf(&bottom, &top, &kin, Some(&w), Some(tool), m.wcs_origin, &sp);
        let summary = json!({"stations": r.stations, "max_deviation_mm": r.max_deviation, "warnings": r.warnings});
        let d = dialect_from(dialect)?.unwrap_or(self.default_dialect());
        let cl = self.cl(
            format!("swarf T{tool}"),
            tool,
            len,
            cutter.radius * 2.0,
            r.moves,
        );
        self.add_program("swarf", cl, d, summary)
    }

    /// Trochoidal slot along a centre line (WCS XY points).
    #[allow(clippy::too_many_arguments)]
    pub fn trochoidal_slot(
        &mut self,
        tool: u32,
        centerline: Vec<[f64; 2]>,
        width: f64,
        z_top: f64,
        z_bottom: f64,
        params: Option<&Value>,
        dialect: Option<&str>,
    ) -> Result<Value> {
        let (_, cutter, len) = self.tool_kin(tool)?;
        let mut sp: crate::slot::SlotParams = from_json(params)?;
        sp.cutter = cutter;
        sp.width = width;
        sp.z_top = z_top;
        sp.z_bottom = z_bottom;
        let r = crate::slot::trochoidal_slot(&centerline, &sp);
        if r.moves.is_empty() {
            return Err(r.warnings.join("; "));
        }
        let summary = json!({
            "levels": r.levels, "loops": r.loops, "loop_radius_mm": r.loop_radius,
            "max_engagement_mm": (r.max_engagement_mm * 1000.0).round() / 1000.0,
            "max_engagement_ratio": (r.max_engagement_ratio * 1000.0).round() / 1000.0,
            "warnings": r.warnings,
        });
        let d = dialect_from(dialect)?.unwrap_or(self.default_dialect());
        let cl = self.cl(
            format!("trochoidal slot T{tool}"),
            tool,
            len,
            cutter.radius * 2.0,
            r.moves,
        );
        self.add_program("trochoidal-slot", cl, d, summary)
    }

    /// Smallest tilt from `axis` that reaches `tip` without collision.
    pub fn tilt(&self, tool: u32, tip: Vec3, axis: Vec3, options: Option<&Value>) -> Result<Value> {
        let (kin, _, _) = self.tool_kin(tool)?;
        let w = self.world()?;
        let o: crate::access::TiltOptions = from_json(options)?;
        let r = crate::access::tilt_to_avoid(&kin, &w, tool, tip, axis, None, &o);
        serde_json::to_value(r).map_err(|e| e.to_string())
    }

    /// Inverse kinematics: all joint solutions for a tool pose (WCS).
    pub fn inverse_kinematics(&self, tip: Vec3, axis: Vec3, tool: Option<u32>) -> Result<Value> {
        let m = self.machine()?;
        let kin = match tool {
            Some(t) => self.tool_kin(t)?.0,
            None => Kinematics::new(m.clone(), m.default_tool_length).map_err(|e| e.to_string())?,
        };
        let sols = kin
            .inverse_all(&Pose { tip, axis }, None)
            .map_err(|e| e.to_string())?;
        let names: Vec<&str> = m.axes.iter().map(|a| a.name.as_str()).collect();
        Ok(json!({
            "axes": names,
            "solutions": sols.iter().map(|s| json!({
                "joints": s.joints.iter().map(|v| (v * 1000.0).round() / 1000.0).collect::<Vec<_>>(),
                "within_limits": s.within_limits, "singular": s.singular,
            })).collect::<Vec<_>>(),
        }))
    }

    /// Program details; with `include_cl`, also the cutter-location moves
    /// (tip, axis, feed) for previews.
    pub fn get_program(&self, id: usize, gcode_lines: usize, include_cl: bool) -> Result<Value> {
        let p = self
            .programs
            .iter()
            .find(|p| p.id == id)
            .ok_or_else(|| format!("no program {id}"))?;
        let head: Vec<&str> = p.gcode.lines().take(gcode_lines).collect();
        let mut v = json!({
            "id": p.id, "name": p.name, "kind": p.kind, "dialect": p.dialect,
            "moves": p.cl.moves.len(), "blocks": p.gcode.lines().count(),
            "verified": p.verified, "verification": p.verification, "summary": p.summary,
            "gcode_head": head,
        });
        if include_cl {
            v["cl_moves"] = serde_json::to_value(&p.cl.moves).map_err(|e| e.to_string())?;
        }
        Ok(v)
    }

    /// Write a program's G-code inside the workspace. Programs that failed
    /// verification are refused unless `force` is true.
    pub fn save_program(&self, id: usize, path: &str, force: bool) -> Result<Value> {
        let p = self
            .programs
            .iter()
            .find(|p| p.id == id)
            .ok_or_else(|| format!("no program {id}"))?;
        if !p.verified && !force {
            return Err(format!(
                "program {id} failed verification ({} errors); fix the issues, or pass force=true only if the user explicitly accepts the risk",
                p.verification["errors"]
            ));
        }
        let full = self.resolve(path, false)?;
        std::fs::write(&full, &p.gcode).map_err(|e| format!("{path}: {e}"))?;
        Ok(json!({
            "written": full.display().to_string(),
            "blocks": p.gcode.lines().count(),
            "verified": p.verified,
            "note": if p.verified { "verified by simulation" } else { "WRITTEN WITHOUT PASSING VERIFICATION (force)" },
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
    }

    #[test]
    fn rough_is_verified_and_saved() {
        let mut s = Session::new(repo());
        s.load_machine(Some("machines/trunnion-ac.toml"), None)
            .unwrap();
        s.load_scene(Some("examples/overhang-setup.toml"), None)
            .unwrap();
        let r = s
            .rough(
                1,
                None,
                false,
                Some(&json!({"stepdown": 4.0, "resolution": 0.5})),
                None,
            )
            .unwrap();
        assert_eq!(r["verified"], true, "{r:#}");
        let id = r["program_id"].as_u64().unwrap() as usize;
        let out = std::env::temp_dir().join("pat-session-test.ngc");
        // Outside the workspace: refused.
        assert!(s.save_program(id, out.to_str().unwrap(), false).is_err());
        let d = s.describe();
        assert_eq!(d["programs"][0]["verified"], true);
    }

    #[test]
    fn failing_programs_need_force() {
        let mut s = Session::new(repo());
        s.load_machine(Some("machines/trunnion-ac.toml"), None)
            .unwrap();
        s.load_scene(Some("examples/vise-setup.toml"), None)
            .unwrap();
        let v = s
            .check_gcode(None, Some("examples/vise-crash.ngc"), false)
            .unwrap();
        assert_eq!(v["ok"], false);
        assert!(v["remedies"]["collision"]
            .as_str()
            .unwrap()
            .contains("tool axis"));
        // A generated program with errors is stored but not saved without force.
        let p = Program {
            id: 99,
            name: "bad".into(),
            kind: "test".into(),
            cl: ClProgram {
                name: "bad".into(),
                tool: ClTool {
                    number: 1,
                    length: 100.0,
                    diameter: None,
                },
                spindle_rpm: None,
                coolant: false,
                moves: vec![],
            },
            dialect: Dialect::FanucTcp,
            gcode: "G0 X0\n".into(),
            verified: false,
            verification: v,
            summary: Value::Null,
        };
        s.programs.push(p);
        let e = s
            .save_program(99, "target/should-not-exist.ngc", false)
            .unwrap_err();
        assert!(e.contains("failed verification"), "{e}");
    }

    #[test]
    fn paths_stay_in_the_workspace() {
        let s = Session::new(repo());
        assert!(s.resolve("../outside.toml", false).is_err());
        assert!(s.resolve("machines/trunnion-ac.toml", true).is_ok());
    }
}

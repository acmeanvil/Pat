//! `pat-mcp`: MCP server over stdio.
//!
//! ```text
//! pat-mcp [--root DIR] [--machine FILE] [--scene FILE]
//! ```
//!
//! `--root` limits file access (default: the current directory). `--machine`
//! and `--scene` preload a setup. Logs go to stderr; stdout carries only
//! protocol messages.

use pat_core::session::Session;
use pat_mcp::Server;
use serde_json::{json, Value};
use std::io::{BufRead, Write};

fn main() {
    let mut root = std::env::current_dir().unwrap_or_else(|_| ".".into());
    let (mut machine, mut scene) = (None, None);
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--root" => root = args.next().map(Into::into).unwrap_or(root),
            "--machine" => machine = args.next(),
            "--scene" => scene = args.next(),
            "-h" | "--help" => {
                eprintln!("usage: pat-mcp [--root DIR] [--machine FILE] [--scene FILE]");
                return;
            }
            other => eprintln!("pat-mcp: ignoring unknown argument {other}"),
        }
    }
    let mut session = Session::new(root);
    if let Some(m) = machine {
        if let Err(e) = session.load_machine(Some(&m), None) {
            eprintln!("pat-mcp: {e}");
        }
    }
    if let Some(s) = scene {
        if let Err(e) = session.load_scene(Some(&s), None) {
            eprintln!("pat-mcp: {e}");
        }
    }
    eprintln!("pat-mcp ready (workspace {})", session.root().display());
    let mut server = Server::new(session);
    let stdin = std::io::stdin();
    let mut out = std::io::stdout().lock();
    for line in stdin.lock().lines() {
        let Ok(line) = line else { break };
        if line.trim().is_empty() {
            continue;
        }
        let response = match serde_json::from_str::<Value>(&line) {
            Ok(Value::Array(batch)) => {
                let rs: Vec<Value> = batch.iter().filter_map(|m| server.handle(m)).collect();
                (!rs.is_empty()).then_some(Value::Array(rs))
            }
            Ok(msg) => server.handle(&msg),
            Err(e) => Some(
                json!({"jsonrpc": "2.0", "id": null, "error": {"code": -32700, "message": format!("parse error: {e}")}}),
            ),
        };
        if let Some(r) = response {
            let _ = writeln!(out, "{r}");
            let _ = out.flush();
        }
    }
}

//! MCP protocol and end-to-end agent workflow tests.

use pat_core::session::Session;
use pat_mcp::Server;
use serde_json::{json, Value};
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Command, Stdio};

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}

fn call(s: &mut Server, id: u64, name: &str, args: Value) -> Value {
    let r = s.handle(&json!({"jsonrpc": "2.0", "id": id, "method": "tools/call", "params": {"name": name, "arguments": args}})).unwrap();
    r["result"].clone()
}

#[test]
fn handshake_and_tool_list() {
    let mut s = Server::new(Session::new(repo()));
    let r = s.handle(&json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "test", "version": "0"}}})).unwrap();
    assert_eq!(r["result"]["protocolVersion"], "2025-03-26");
    assert!(r["result"]["instructions"]
        .as_str()
        .unwrap()
        .contains("verified"));
    assert!(s
        .handle(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"}))
        .is_none());
    let r = s
        .handle(&json!({"jsonrpc": "2.0", "id": 2, "method": "tools/list"}))
        .unwrap();
    let tools = r["result"]["tools"].as_array().unwrap();
    assert_eq!(tools.len(), 13);
    for t in tools {
        assert!(
            t["description"].as_str().unwrap().len() > 20,
            "{}",
            t["name"]
        );
        assert_eq!(t["inputSchema"]["type"], "object");
    }
    let r = s
        .handle(&json!({"jsonrpc": "2.0", "id": 3, "method": "nope"}))
        .unwrap();
    assert_eq!(r["error"]["code"], -32601);
}

#[test]
fn errors_come_back_as_tool_errors() {
    let mut s = Server::new(Session::new(repo()));
    let r = call(&mut s, 1, "rough", json!({"tool": 1}));
    assert_eq!(r["isError"], true);
    assert!(
        r["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("load_scene")
            || r["content"][0]["text"]
                .as_str()
                .unwrap()
                .contains("load_machine")
    );
    let r = call(
        &mut s,
        2,
        "load_machine",
        json!({"path": "../../etc/passwd"}),
    );
    assert_eq!(r["isError"], true);
}

#[test]
fn agent_workflow_rough_check_and_save() {
    let mut s = Server::new(Session::new(repo()));
    assert_eq!(
        call(
            &mut s,
            1,
            "load_machine",
            json!({"path": "machines/trunnion-ac.toml"})
        )["isError"],
        false
    );
    assert_eq!(
        call(
            &mut s,
            2,
            "load_scene",
            json!({"path": "examples/overhang-setup.toml"})
        )["isError"],
        false
    );
    let d = call(&mut s, 3, "describe_setup", json!({}));
    assert_eq!(d["structuredContent"]["machine"]["rotary_axes"], 2);

    let o = call(&mut s, 4, "plan_orientations", json!({"tool": 1}));
    let first = &o["structuredContent"]["orientations"][0];
    assert!(first["explanation"].as_str().unwrap().contains("setup 1"));

    let r = call(
        &mut s,
        5,
        "rough",
        json!({"tool": 1, "auto": true, "params": {"stepdown": 4.0, "resolution": 0.5}}),
    );
    assert_eq!(r["isError"], false, "{}", r["content"][0]["text"]);
    let sc = &r["structuredContent"];
    assert_eq!(sc["verified"], true, "{:#}", sc["verification"]);
    let id = sc["program_id"].as_u64().unwrap();

    let p = call(
        &mut s,
        6,
        "get_program",
        json!({"id": id, "gcode_lines": 10}),
    );
    assert_eq!(
        p["structuredContent"]["gcode_head"]
            .as_array()
            .unwrap()
            .len(),
        10
    );

    let target = "target/pat-mcp-test-output.ngc";
    let w = call(&mut s, 7, "save_program", json!({"id": id, "path": target}));
    assert_eq!(w["isError"], false, "{}", w["content"][0]["text"]);
    let written = std::fs::read_to_string(repo().join(target)).unwrap();
    assert!(written.contains("G43.4"));

    // A crashing program checked through the agent API gets remedies.
    call(
        &mut s,
        8,
        "load_scene",
        json!({"path": "examples/vise-setup.toml"}),
    );
    let c = call(
        &mut s,
        9,
        "check_gcode",
        json!({"path": "examples/vise-crash.ngc", "simulate_stock": false}),
    );
    let v = &c["structuredContent"];
    assert_eq!(v["ok"], false);
    assert!(v["issue_counts"]["collision"].as_u64().unwrap() >= 1);
    assert!(v["remedies"]["rapid_into_part"].is_string());
}

#[test]
fn stdio_transport() {
    let mut child = Command::new(env!("CARGO_BIN_EXE_pat-mcp"))
        .args([
            "--root",
            repo().to_str().unwrap(),
            "--machine",
            "machines/vmc-4th-a.toml",
        ])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let mut lines = BufReader::new(child.stdout.take().unwrap()).lines();
    let mut send = |v: Value| writeln!(stdin, "{v}").unwrap();
    send(
        json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-06-18"}}),
    );
    send(json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));
    send(
        json!({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "inverse_kinematics", "arguments": {"tip": [0, 0, 10], "axis": [0, -1, 0]}}}),
    );
    let init: Value = serde_json::from_str(&lines.next().unwrap().unwrap()).unwrap();
    assert_eq!(init["id"], 1);
    let ik: Value = serde_json::from_str(&lines.next().unwrap().unwrap()).unwrap();
    assert_eq!(ik["id"], 2);
    let sols = &ik["result"]["structuredContent"]["solutions"];
    assert!(!sols.as_array().unwrap().is_empty());
    // A = 90 tips the tool toward -Y on this machine.
    let a = sols[0]["joints"][3].as_f64().unwrap();
    assert!((a.rem_euclid(360.0) - 90.0).abs() < 1e-6, "A = {a}");
    drop(stdin);
    child.wait().unwrap();
}

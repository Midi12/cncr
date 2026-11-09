use std::sync::{Arc, Mutex};

use cncr_core::autolevel::Point2;
use cncr_core::controller::{ControllerOptions, PinOptions, Registry};
use cncr_core::hub::{Frame, Hub};
use cncr_core::monitor::Monitor;
use cncr_core::taskrunner::TaskRunner;
use cncr_core::workflow::WorkflowState;
use serde_json::{json, Value};
use tokio::sync::{mpsc, oneshot};

use super::*;

#[derive(Clone, Default)]
struct Rec(Arc<Mutex<Vec<String>>>);

impl Port for Rec {
    fn write(&mut self, data: &str) {
        self.0.lock().unwrap().push(data.to_string());
    }
}

struct H {
    ctl: Controller,
    rec: Rec,
    frames: mpsc::UnboundedReceiver<Frame>,
    internal: mpsc::UnboundedReceiver<Internal>,
}

fn setup_with(config: Value, monitor: Monitor) -> H {
    let hub = Arc::new(Hub::default());
    let services = Services {
        config: Arc::new(ConfigStore::in_memory(config)),
        tasks: Arc::new(TaskRunner::new(hub.clone())),
        hub,
        monitor: Arc::new(monitor),
        registry: Arc::new(Registry::default()),
    };
    let options = ControllerOptions {
        port: "/dev/null".into(),
        baudrate: 115200,
        rtscts: false,
        pin: PinOptions::default(),
    };
    let rec = Rec::default();
    let (itx, internal) = mpsc::unbounded_channel();
    let mut ctl = Controller::new(services, options, Some(Box::new(rec.clone())), itx);
    let (socket, frames) = Socket::new();
    ctl.sockets.add(socket);
    H {
        ctl,
        rec,
        frames,
        internal,
    }
}

fn setup(config: Value) -> H {
    setup_with(config, Monitor::new(None))
}

impl H {
    fn writes(&self) -> Vec<String> {
        self.rec.0.lock().unwrap().clone()
    }

    fn cmd(&mut self, cmd: &str, args: Vec<Value>) {
        self.ctl.command(cmd, &args, None);
    }

    fn cmd_reply(&mut self, cmd: &str, args: Vec<Value>) -> Result<Value, String> {
        let (tx, mut rx) = oneshot::channel();
        self.ctl.command(cmd, &args, Some(tx));
        rx.try_recv().expect("reply")
    }

    fn line(&mut self, line: &str) {
        self.ctl.on_line(line);
    }

    /// Drains the emitted `(event, args)`.
    fn events(&mut self) -> Vec<(String, Vec<Value>)> {
        let mut out = vec![];
        while let Ok(f) = self.frames.try_recv() {
            let v: Value = serde_json::from_str(&f).unwrap();
            out.push((
                v["event"].as_str().unwrap().to_string(),
                v["args"].as_array().unwrap().clone(),
            ));
        }
        out
    }

    fn reads(&mut self) -> Vec<String> {
        self.events()
            .into_iter()
            .filter(|(e, _)| e == "serialport:read")
            .map(|(_, a)| a[0].as_str().unwrap().to_string())
            .collect()
    }

    fn write_sources(&mut self) -> Vec<String> {
        self.events()
            .into_iter()
            .filter(|(e, _)| e == "serialport:write")
            .map(|(_, a)| a[1]["source"].as_str().unwrap().to_string())
            .collect()
    }

    fn flush_feeder(&mut self) {
        while self.ctl.feeder.size() > 0 {
            self.ctl.feeder.unhold();
            self.line("ok");
        }
    }
}

const SHORT_PROGRAM: &str = "G0 X0\nG0 Y0\nM30";

fn long_program() -> String {
    vec!["G0 X10"; 20].join("\n")
}

fn repeat(line: &str, n: usize) -> Vec<String> {
    vec![line.to_string(); n]
}

#[tokio::test]
async fn realtime_and_simple_commands() {
    for (cmd, expected) in [
        ("feedhold", "!"),
        ("cyclestart", "~"),
        ("statusreport", "?"),
        ("homing", "$H\n"),
        ("sleep", "$SLP\n"),
        ("unlock", "$X\n"),
        ("reset", "\x18"),
        ("jogCancel", "\u{85}"),
    ] {
        let mut h = setup(json!({}));
        h.cmd(cmd, vec![]);
        assert_eq!(h.writes(), [expected], "{cmd}");
        assert_eq!(h.write_sources(), [WRITE_SOURCE_CLIENT], "{cmd}");
    }
}

#[tokio::test]
async fn reset_stops_the_workflow_and_clears_the_feeder() {
    let mut h = setup(json!({}));
    h.cmd("gcode", vec![json!(["G0 X0", "G0 Y0"])]);
    h.cmd("reset", vec![]);
    assert_eq!(h.writes(), ["G0 X0\n", "\x18"]);
    assert_eq!(h.ctl.feeder.size(), 0);
    assert!(h.ctl.workflow.is_idle());
}

#[tokio::test]
async fn override_bytes() {
    let cases: [(&str, i64, Option<&str>); 17] = [
        ("feedOverride", 0, Some("\u{90}")),
        ("feedOverride", 10, Some("\u{91}")),
        ("feedOverride", -10, Some("\u{92}")),
        ("feedOverride", 1, Some("\u{93}")),
        ("feedOverride", -1, Some("\u{94}")),
        ("feedOverride", 5, None),
        ("spindleOverride", 0, Some("\u{99}")),
        ("spindleOverride", 10, Some("\u{9a}")),
        ("spindleOverride", -10, Some("\u{9b}")),
        ("spindleOverride", 1, Some("\u{9c}")),
        ("spindleOverride", -1, Some("\u{9d}")),
        ("spindleOverride", 5, None),
        ("rapidOverride", 0, Some("\u{95}")),
        ("rapidOverride", 100, Some("\u{95}")),
        ("rapidOverride", 50, Some("\u{96}")),
        ("rapidOverride", 25, Some("\u{97}")),
        ("rapidOverride", 75, None),
    ];
    for (cmd, value, expected) in cases {
        let mut h = setup(json!({}));
        h.cmd(cmd, vec![json!(value)]);
        assert_eq!(
            h.writes(),
            expected.into_iter().collect::<Vec<_>>(),
            "{cmd} {value}"
        );
    }
}

#[tokio::test]
async fn laser_test() {
    let mut h = setup(json!({}));
    h.cmd("lasertest:on", vec![]);
    h.flush_feeder();
    assert_eq!(h.writes(), ["G1F1\n", "M3S0\n"]);
    assert_eq!(
        h.write_sources(),
        [WRITE_SOURCE_FEEDER, WRITE_SOURCE_FEEDER]
    );

    let mut h = setup(json!({}));
    h.cmd("lasertest:on", vec![json!(50), json!(100), json!(1000)]);
    h.flush_feeder();
    assert_eq!(h.writes(), ["G1F1\n", "M3S500\n", "G4P0.1\n", "M5S0\n"]);

    let mut h = setup(json!({}));
    h.cmd("lasertest:off", vec![]);
    assert_eq!(h.writes(), ["M5S0\n"]);
}

#[tokio::test]
async fn feeder_commands() {
    let mut h = setup(json!({}));
    h.cmd("feeder:feed", vec![json!(["G0 X0", "G0 Y0"])]);
    h.line("ok");
    assert_eq!(h.writes(), ["G0 X0\n", "G0 Y0\n"]);
    assert_eq!(
        h.write_sources(),
        [WRITE_SOURCE_FEEDER, WRITE_SOURCE_FEEDER]
    );

    // feeder:start writes cycle start, then the next queued line.
    let mut h = setup(json!({}));
    h.cmd("feeder:feed", vec![json!(["G0 X0", "G0 Y0"])]);
    h.cmd("feeder:start", vec![]);
    assert_eq!(h.writes(), ["G0 X0\n", "~", "G0 Y0\n"]);
    assert_eq!(
        h.write_sources(),
        [
            WRITE_SOURCE_FEEDER,
            WRITE_SOURCE_CLIENT,
            WRITE_SOURCE_FEEDER
        ]
    );

    // ...and nothing while a program is running.
    let mut h = setup(json!({}));
    h.cmd(
        "gcode:load",
        vec![json!("test.gcode"), json!(SHORT_PROGRAM)],
    );
    h.cmd("gcode:start", vec![]);
    let n = h.writes().len();
    h.cmd("feeder:start", vec![]);
    assert_eq!(h.writes().len(), n);

    let mut h = setup(json!({}));
    h.cmd("gcode", vec![json!(["G0 X0", "G0 Y0"])]);
    h.cmd("feeder:stop", vec![]);
    h.line("ok");
    assert_eq!(h.writes(), ["G0 X0\n"]);
    assert_eq!(h.ctl.feeder.size(), 0);
}

#[tokio::test]
async fn gcode_skips_blank_and_comment_lines() {
    let mut h = setup(json!({}));
    h.cmd(
        "gcode",
        vec![json!("G0 X0\n\n   \n; comment\n(parenthesized)\r\nG0 Y0")],
    );
    h.line("ok");
    assert_eq!(h.writes(), ["G0 X0\n", "G0 Y0\n"]);
    assert_eq!(h.ctl.feeder.size(), 0);
}

#[tokio::test]
async fn feeder_pauses_on_m0_and_stops_in_alarm() {
    let mut h = setup(json!({}));
    h.cmd("gcode", vec![json!("%msg Change bit\nM0\nG0 X1")]);
    assert_eq!(h.writes(), ["M0\n"]);
    assert!(h.ctl.feeder.hold);
    assert_eq!(
        h.ctl.feeder.hold_reason,
        Some(json!({"data": "M0", "msg": "Change bit"}))
    );
    h.line("ok");
    assert_eq!(h.writes().len(), 1, "held");

    h.line("ALARM:1");
    assert_eq!(
        h.reads().last().map(String::as_str),
        Some("ALARM:1 (Hard limit)")
    );
    h.ctl.feeder.unhold();
    h.line("ok");
    assert_eq!(h.writes().len(), 1, "nothing is sent in Alarm mode");
    assert_eq!(h.ctl.feeder.size(), 0);
}

#[tokio::test]
async fn gcode_load_appends_a_dwell() {
    let mut h = setup(json!({}));
    let json = h
        .cmd_reply(
            "gcode:load",
            vec![json!("test.gcode"), json!(SHORT_PROGRAM), json!({})],
        )
        .unwrap();
    assert_eq!(
        (
            json["name"].clone(),
            json["total"].clone(),
            json["sent"].clone()
        ),
        (json!("test.gcode"), json!(4), json!(0))
    );
    assert_eq!(
        h.ctl.sender.state.gcode,
        format!("{SHORT_PROGRAM}\n%wait ; Wait for the planner to empty")
    );
    assert!(h.writes().is_empty());
    assert!(h.events().iter().any(|(e, _)| e == "gcode:load"));

    // An empty or non-string program still loads because of the dwell.
    for (gcode, total) in [(json!(""), 1), (json!(42), 2)] {
        let json = h
            .cmd_reply("gcode:load", vec![json!("test.gcode"), gcode])
            .unwrap();
        assert_eq!(json["total"], json!(total));
    }
}

#[tokio::test]
async fn program_streaming_fills_the_character_counting_buffer() {
    let mut h = setup(json!({}));
    h.cmd(
        "gcode:load",
        vec![json!("test.gcode"), json!(SHORT_PROGRAM)],
    );
    h.cmd("gcode:start", vec![]);
    assert_eq!(h.writes(), ["G0 X0\n", "G0 Y0\n", "M30\n", "G4 P0.5\n"]);
    assert!(h.ctl.workflow.is_running());
    h.cmd("gcode:unload", vec![]);
    assert!(h.ctl.workflow.is_idle());
    assert!(h.ctl.sender.state.gcode.is_empty());
    assert!(h.events().iter().any(|(e, _)| e == "gcode:unload"));

    // 120 byte buffer: 17 lines of 7 bytes.
    let mut h = setup(json!({}));
    h.cmd(
        "gcode:load",
        vec![json!("test.gcode"), json!(long_program())],
    );
    h.cmd("gcode:start", vec![]);
    assert_eq!(h.writes(), repeat("G0 X10\n", 17));
    assert_eq!(h.ctl.sender.state.sent, 17);
    // Every ok frees one line.
    h.line("ok");
    assert_eq!(h.writes().len(), 18);
}

#[tokio::test]
async fn pause_resume_and_stop() {
    let mut h = setup(json!({}));
    h.cmd(
        "gcode:load",
        vec![json!("test.gcode"), json!(long_program())],
    );
    h.cmd("gcode:start", vec![]);
    h.cmd("gcode:pause", vec![]);
    let mut expected = repeat("G0 X10\n", 17);
    expected.push("!".into());
    assert_eq!(h.writes(), expected);
    assert!(h.ctl.workflow.is_paused() && h.ctl.sender.state.hold);

    // Acks still arrive while paused, nothing is sent.
    h.line("ok");
    assert_eq!(h.ctl.sender.state.received, 1);
    assert_eq!(h.writes().len(), 18);

    h.cmd("gcode:resume", vec![]);
    expected.extend(["~".to_string(), "G0 X10\n".into(), "G0 X10\n".into()]);
    assert_eq!(h.writes(), expected);
    assert!(h.ctl.workflow.is_running() && !h.ctl.sender.state.hold);

    h.cmd("gcode:stop", vec![]);
    assert_eq!(h.writes().len(), expected.len());
    assert!(h.ctl.workflow.is_idle());
    assert_eq!(h.ctl.sender.state.sent, 0);
}

#[tokio::test]
async fn deprecated_aliases_match_canonical_commands() {
    for (alias, canonical) in [
        ("start", "gcode:start"),
        ("pause", "gcode:pause"),
        ("resume", "gcode:resume"),
        ("stop", "gcode:stop"),
    ] {
        let run = |cmd: &str| {
            let mut h = setup(json!({}));
            h.cmd(
                "gcode:load",
                vec![json!("test.gcode"), json!(SHORT_PROGRAM)],
            );
            if canonical != "gcode:start" {
                h.cmd("gcode:start", vec![]);
            }
            if canonical == "gcode:resume" {
                h.cmd("gcode:pause", vec![]);
            }
            h.cmd(cmd, vec![]);
            (h.writes(), h.ctl.workflow.state)
        };
        assert_eq!(run(alias), run(canonical), "{alias}");
    }
}

#[tokio::test(start_paused = true)]
async fn forced_stop_holds_then_resets() {
    // Running: feed hold now, reset 500 ms later once on hold.
    let mut h = setup(json!({}));
    h.ctl.runner.state.status.active_state = "Run".into();
    h.cmd("gcode:stop", vec![json!({"force": true})]);
    h.ctl.runner.state.status.active_state = "Hold".into();
    assert_eq!(h.writes(), ["!"]);
    let msg = h.internal.recv().await.unwrap();
    h.ctl.handle_internal(msg);
    assert_eq!(h.writes(), ["!", "\x18"]);
    assert_eq!(
        h.write_sources(),
        [WRITE_SOURCE_CLIENT, WRITE_SOURCE_CLIENT]
    );

    for (state, expected) in [("Hold", vec!["\x18"]), ("Idle", vec![])] {
        let mut h = setup(json!({}));
        h.ctl.runner.state.status.active_state = state.into();
        h.cmd("gcode:stop", vec![json!({"force": true})]);
        let msg = h.internal.recv().await.unwrap();
        h.ctl.handle_internal(msg);
        assert_eq!(h.writes(), expected, "{state}");
    }
}

#[tokio::test]
async fn program_m0_pauses_the_workflow_until_resumed() {
    let mut h = setup(json!({}));
    h.cmd(
        "gcode:load",
        vec![json!("p.nc"), json!("G0 X1\n%msg Flip the part\nM0\nG0 X2")],
    );
    h.events();
    h.cmd("gcode:start", vec![]);
    assert_eq!(h.writes(), ["G0 X1\n", "M0\n"]);
    assert!(h.ctl.workflow.is_paused());
    assert_eq!(
        h.ctl.sender.state.hold_reason,
        Some(json!({"data": "M0", "msg": "Flip the part"}))
    );
    let states: Vec<Value> = h
        .events()
        .into_iter()
        .filter(|(e, _)| e == "workflow:state")
        .map(|(_, a)| a[0].clone())
        .collect();
    assert_eq!(states, [json!("running"), json!("paused")]);
    h.line("ok");
    h.line("ok");
    assert_eq!(h.writes().len(), 2);
    h.cmd("gcode:resume", vec![]);
    assert_eq!(h.writes(), ["G0 X1\n", "M0\n", "~", "G0 X2\n", "G4 P0.5\n"]);
}

#[tokio::test]
async fn program_wait_holds_until_the_buffer_drains() {
    let mut h = setup(json!({}));
    h.cmd(
        "gcode:load",
        vec![json!("p.nc"), json!("G0 X1\n%wait\nG0 X2")],
    );
    h.cmd("gcode:start", vec![]);
    assert_eq!(h.writes(), ["G0 X1\n", "G4 P0.5\n"]);
    assert!(h.ctl.sender.state.hold);
    h.line("ok");
    assert_eq!(h.writes().len(), 2, "one line still in flight");
    h.line("ok");
    assert_eq!(h.writes(), ["G0 X1\n", "G4 P0.5\n", "G0 X2\n", "G4 P0.5\n"]);
}

#[tokio::test]
async fn errors_while_running_pause_the_program() {
    let mut h = setup(json!({}));
    h.cmd(
        "gcode:load",
        vec![json!("p.nc"), json!("G0 X1\nG5 X2\nG0 X3")],
    );
    h.cmd("gcode:start", vec![]);
    h.events();
    h.line("ok");
    h.line("error:20");
    // As in cncjs: `ln` is the failing line number, the echoed text is `lines[received - 1]`.
    assert_eq!(
        h.reads(),
        ["> G0 X1 (ln=2)", "error:20 (Unsupported command)"]
    );
    assert!(h.ctl.workflow.is_paused());

    // Grbl v0.9 messages are shown verbatim; ignoreErrors keeps running.
    let mut h = setup(json!({"state": {"controller": {"exception": {"ignoreErrors": true}}}}));
    h.cmd("gcode:load", vec![json!("p.nc"), json!("G0 X1\nG5 X2")]);
    h.cmd("gcode:start", vec![]);
    h.line("ok");
    h.events();
    h.line("error: Unsupported statement");
    assert_eq!(
        h.reads(),
        ["> G0 X1 (ln=2)", "error: Unsupported statement"]
    );
    assert!(h.ctl.workflow.is_running());

    // Outside a program the error is reported and the feeder continues.
    let mut h = setup(json!({}));
    h.cmd("gcode", vec![json!("G5\nG0 X1")]);
    h.line("error:20");
    assert_eq!(h.reads(), ["error:20 (Unsupported command)"]);
    assert_eq!(h.writes(), ["G5\n", "G0 X1\n"]);
}

#[tokio::test]
async fn m6_follows_the_tool_change_policy() {
    // Ignore: commented out and paused.
    let mut h = setup(json!({"tool": {"toolChangePolicy": 0}}));
    h.cmd("gcode:load", vec![json!("p.nc"), json!("T1 M6\nG0 X1")]);
    h.cmd("gcode:start", vec![]);
    assert_eq!(h.writes(), ["T1 (M6)\n"]);
    assert!(h.ctl.workflow.is_paused());

    // Send: passed through.
    let mut h = setup(json!({"tool": {"toolChangePolicy": 1}}));
    h.cmd("gcode:load", vec![json!("p.nc"), json!("T1 M6\nG0 X1")]);
    h.cmd("gcode:start", vec![]);
    assert_eq!(h.writes(), ["T1 M6\n", "G0 X1\n", "G4 P0.5\n"]);

    // Manual: the tool change program starts through the feeder before the line.
    let mut h = setup(json!({"tool": {"toolChangePolicy": 2}}));
    h.cmd("gcode:load", vec![json!("p.nc"), json!("T1 M6\nG0 X1")]);
    h.cmd("gcode:start", vec![]);
    assert_eq!(h.writes(), ["G4 P0.5\n", "T1 (M6)\n"]);
    assert!(h.ctl.workflow.is_paused());
    assert_eq!(h.ctl.feeder.size(), 26);
}

const TOOL_CHANGE_PREFIX: [&str; 10] = [
    "G4 P0.5\n",
    "M5\n",
    "G90\n",
    "G53 G0 Z{0}\n",
    "G53 G0 X{0} Y{0}\n",
    "G4 P0.5\n",
    "M0\n",
    "G53 G0 X{0} Y{0}\n",
    "G53 G0 Z{0}\n",
    "G4 P0.5\n",
];
const TOOL_CHANGE_SUFFIX: [&str; 9] = [
    "G53 G0 Z{0}\n",
    "G53 G0 X{0} Y{0}\n",
    "G4 P0.5\n",
    "M0\n",
    "G90\n",
    "G0 X0 Y0\n",
    "G0 Z0\n",
    "M5\n",
    "G4 P5\n",
];

fn tool_change_writes(zero: &str, probing: &[&str]) -> Vec<String> {
    let fill = |l: &&str| l.replace("{0}", zero);
    TOOL_CHANGE_PREFIX
        .iter()
        .map(fill)
        .chain(probing.iter().map(|l| l.to_string()))
        .chain(TOOL_CHANGE_SUFFIX.iter().map(fill))
        .collect()
}

#[tokio::test]
async fn tool_change_sequences() {
    let cases: [(i64, Vec<&str>); 5] = [
        (0, vec![]),
        (1, vec![]),
        (2, vec!["G91 G38.2 F10 Z-1\n", "G10 L20 P1 Z0\n"]),
        (3, vec!["G91 G38.2 F10 Z-1\n", "G4 P1\n", "G43.1 Z0\n"]),
        (4, vec!["G38.2 Z-5\n", "G92 Z0\n"]),
    ];
    for (policy, probing) in cases {
        let mut h = setup(
            json!({"tool": {"toolChangePolicy": policy, "toolProbeCustomCommands": "G38.2 Z-5\nG92 Z0"}}),
        );
        h.cmd("tool:change", vec![]);
        h.flush_feeder();
        assert_eq!(
            h.writes(),
            tool_change_writes("0.000", &probing),
            "policy {policy}"
        );
    }

    let mut h = setup(json!({"tool": {"toolChangePolicy": 2}}));
    h.ctl
        .runner
        .state
        .parserstate
        .modal
        .insert("units".into(), json!("G20"));
    h.cmd("tool:change", vec![]);
    h.flush_feeder();
    assert_eq!(
        h.writes(),
        tool_change_writes(
            "0.0000",
            &["G91 G38.2 F0.3937 Z-0.0394\n", "G10 L20 P1 Z0\n"]
        )
    );
}

#[tokio::test]
async fn autolevel_probing() {
    let mut h = setup(json!({}));
    h.cmd(
        "autolevel:start",
        vec![json!({"mode": "test", "clearanceZ": 5, "startZ": 1, "endZ": -1, "feedrate": 100})],
    );
    h.flush_feeder();
    assert_eq!(
        h.writes(),
        ["G90\n", "G0 Z5\n", "G0 Z1\n", "G38.2 Z-1 F100\n", "G0 Z5\n"]
    );
    assert_eq!(h.ctl.probe_state.config, None);

    let mut h = setup(json!({}));
    let params = json!({"startX": 0, "endX": 10, "stepX": 10, "startY": 0, "endY": 10, "stepY": 10,
        "clearanceZ": 5, "startZ": 1, "endZ": -1, "feedrate": 100});
    h.cmd("autolevel:start", vec![params.clone()]);
    h.flush_feeder();
    let mut expected = vec![];
    for (i, (x, y)) in [(0, 0), (10, 0), (0, 10), (10, 10)].iter().enumerate() {
        let f = if i == 0 { 50 } else { 100 };
        expected.extend([
            "G90\n".to_string(),
            "G0 Z5\n".into(),
            format!("G0 X{x} Y{y}\n"),
            "G0 Z1\n".into(),
            format!("G38.2 Z-1 F{f}\n"),
            "G0 Z5\n".into(),
        ]);
    }
    assert_eq!(h.writes(), expected);
    assert_eq!(h.ctl.probe_state.probe_points.len(), 4);
    assert_eq!(h.ctl.probe_state.config, Some(params));

    h.cmd("autolevel:stop", vec![]);
    assert_eq!(h.writes().last().map(String::as_str), Some("\x18"));
    assert_eq!(h.ctl.probe_state, ProbeState::default());
}

#[tokio::test]
async fn probe_results_are_recorded_at_grid_nodes() {
    let mut h = setup(json!({}));
    h.ctl.probe_state.probe_points = vec![Point2 { x: 0.0, y: 0.0 }, Point2 { x: 10.0, y: 0.0 }];
    // Quantised reports of the commanded nodes (0,0) and (10,0).
    h.line("[PRB:0.001,0.001,-1.500:1]");
    h.line("[PRB:10.001,-0.001,-1.000:0]"); // failed probe: ignored
    h.line("[PRB:10.001,-0.001,-1.000:1]");
    let p: Vec<_> = h
        .ctl
        .probe_state
        .probed_positions
        .iter()
        .map(|p| (p.x, p.y, p.z))
        .collect();
    assert_eq!(p, [(0.0, 0.0, -1.5), (10.0, 0.0, -1.0)]);
    assert_eq!(
        (h.ctl.probe_state.min_z, h.ctl.probe_state.max_z),
        (Some(-1.5), Some(-1.0))
    );
    let events = h.events();
    assert_eq!(
        events
            .iter()
            .filter(|(e, _)| e == "autolevel:update")
            .count(),
        2
    );
    assert_eq!(events.last().unwrap().0, "autolevel:complete");

    // PRB is in machine coordinates and follows $13 (inches), not G20/G21.
    let mut h = setup(json!({}));
    h.ctl.probe_state.probe_points = vec![Point2 { x: 1.0, y: 0.0 }];
    h.line("<Idle|MPos:0.000,0.000,10.000|FS:0,0|WCO:0.000,0.000,10.000>");
    h.ctl.write("$13=1\n", Context::new());
    h.line("[PRB:0.000,0.000,9.900:1]");
    let p = h.ctl.probe_state.probed_positions[0];
    assert_eq!((p.x, p.y), (1.0, 0.0));
    assert!((p.z - -2.54).abs() < 1e-9, "{}", p.z);
}

#[tokio::test]
async fn macros_and_watch_directory() {
    let config = json!({"macros": [{"id": "m1", "name": "Square", "content": "G0 X0\nG0 Y0"}]});
    let mut h = setup(config.clone());
    assert_eq!(
        h.cmd_reply("macro:run", vec![json!("m1"), json!({})]),
        Ok(Value::Null)
    );
    h.flush_feeder();
    assert_eq!(h.writes(), ["G0 X0\n", "G0 Y0\n"]);

    let mut h = setup(config);
    h.cmd("macro:run", vec![json!("nope")]);
    assert!(h.writes().is_empty());
    let json = h.cmd_reply("macro:load", vec![json!("m1")]).unwrap();
    assert_eq!(
        (json["name"].clone(), json["total"].clone()),
        (json!("Square"), json!(3))
    );

    let dir = std::env::temp_dir().join(format!("cncr-grbl-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("part.nc"), "G0 X0\nG0 Y0").unwrap();
    let mut h = setup_with(json!({}), Monitor::new(Some(dir.clone())));
    for (file, ok) in [("part.nc", true), ("missing.nc", false)] {
        let (tx, rx) = oneshot::channel();
        h.ctl.command("watchdir:load", &[json!(file)], Some(tx));
        let msg = h.internal.recv().await.unwrap();
        h.ctl.handle_internal(msg);
        let result = rx.await.unwrap();
        assert_eq!(result.is_ok(), ok, "{file}");
        if ok {
            assert_eq!(result.unwrap()["total"], json!(3));
        }
    }
    std::fs::remove_dir_all(dir).unwrap();
    assert!(h.writes().is_empty());
}

#[tokio::test]
async fn nothing_is_written_when_closed_or_unknown() {
    let mut h = setup(json!({}));
    h.cmd("nonexistent", vec![]);
    assert!(h.writes().is_empty());
    h.ctl.close();
    h.cmd("feedhold", vec![]);
    h.cmd("homing", vec![]);
    h.cmd("gcode", vec![json!(["G0 X0"])]);
    h.cmd(
        "gcode:load",
        vec![json!("test.gcode"), json!(SHORT_PROGRAM)],
    );
    h.cmd("gcode:start", vec![]);
    assert!(h.writes().is_empty());
    assert!(h
        .events()
        .iter()
        .any(|(e, a)| e == "serialport:close"
            && a[0] == json!({"port": "/dev/null", "inuse": false})));
}

#[tokio::test]
async fn first_status_report_initializes_the_controller() {
    let mut h = setup(json!({}));
    h.line("<Idle|MPos:0.000,0.000,0.000|Bf:15,254|FS:0,0>");
    assert!(h.ctl.ready);
    assert_eq!(h.writes(), ["$$\n"]);
    // The RX buffer grows to the reported size (minus 8) while idle.
    assert_eq!(h.ctl.sender.sp.buffer_size(), Some(246));
    assert!(matches!(
        h.internal.recv().await,
        Some(Internal::ControllerReady)
    ));
}

#[tokio::test]
async fn parser_state_queries_are_hidden_unless_requested() {
    let mut h = setup(json!({}));
    h.line("Grbl 1.1f ['$' for help]");
    h.ctl.on_timer();
    assert_eq!(h.writes(), ["$$\n", "?", "$G\n"]);
    h.events();
    h.line("<Idle|MPos:0.000,0.000,0.000|FS:0,0>");
    h.line("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]");
    h.line("ok");
    assert!(
        h.reads().is_empty(),
        "automatic ? and $G replies are not shown"
    );

    // A user `$G` shows the parser state and its ok.
    h.cmd("statusreport", vec![]);
    h.ctl.write("$G\n", Context::new());
    h.line("<Idle|MPos:0.000,0.000,0.000|FS:0,0>");
    h.line("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]");
    h.line("ok");
    assert_eq!(
        h.reads(),
        [
            "<Idle|MPos:0.000,0.000,0.000|FS:0,0>",
            "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]",
            "ok"
        ]
    );
}

#[tokio::test]
async fn timer_emits_state_settings_and_detects_program_end() {
    let mut h = setup(json!({}));
    h.ctl.write("$13=1\n", Context::new());
    h.line("$13=1");
    assert_eq!(h.reads(), ["$13=1 (Report in inches, boolean)"]);
    h.ctl.on_timer();
    let events = h.events();
    let settings = events
        .iter()
        .find(|(e, _)| e == "controller:settings")
        .unwrap();
    assert_eq!(settings.1[0], json!("Grbl"));
    assert_eq!(settings.1[1]["settings"]["$13"], json!("1"));
    assert!(events
        .iter()
        .any(|(e, a)| e == "controller:state" && a[1]["status"]["mpos"]["x"] == json!("0.000")));
    h.ctl.on_timer();
    assert!(
        !h.events().iter().any(|(e, _)| e.starts_with("controller:")),
        "unchanged state is not re-emitted"
    );

    // Program sent and acknowledged, machine idle and still: the workflow stops.
    h.line("<Idle|MPos:0.000,0.000,0.000|FS:0,0>");
    h.cmd("gcode:load", vec![json!("p.nc"), json!("G0 X0")]);
    h.cmd("gcode:start", vec![]);
    h.line("ok");
    h.line("ok");
    assert!(h.ctl.time.sender_finish > 0);
    h.ctl.on_timer();
    h.ctl.time.sender_finish -= 600;
    h.ctl.on_timer();
    assert!(h.ctl.workflow.is_idle());
}

#[tokio::test]
async fn new_sockets_get_the_current_state() {
    let mut h = setup(json!({}));
    h.cmd("gcode:load", vec![json!("p.nc"), json!("G0 X0")]);
    h.ctl.on_timer();
    let (socket, mut rx) = Socket::new();
    h.ctl.add_socket(socket);
    let mut events = vec![];
    while let Ok(f) = rx.try_recv() {
        let v: Value = serde_json::from_str(&f).unwrap();
        events.push(v["event"].as_str().unwrap().to_string());
    }
    assert_eq!(
        events,
        [
            "serialport:open",
            "controller:settings",
            "controller:state",
            "feeder:status",
            "sender:status",
            "gcode:load",
            "workflow:state"
        ]
    );
    assert_eq!(h.ctl.status()["controller"]["type"], json!("Grbl"));
    assert_eq!(h.ctl.workflow.state, WorkflowState::Idle);
}

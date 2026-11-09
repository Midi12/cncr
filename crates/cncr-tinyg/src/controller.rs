//! The TinyG controller task (port of `TinyGController`): JSON init sequence, feeder
//! and send-response sender with queue-report flow control, commands and events.

use std::time::Duration;

use serde_json::{json, Map, Value};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio::time::{interval_at, Instant};
use tracing::{debug, error, info, warn};

use cncr_core::autolevel::{apply_probe_compensation_params, ProbeState};
use cncr_core::config::ConfigStore;
use cncr_core::controller::{reply, ControllerMsg, ControllerOptions, Reply, Services};
use cncr_core::feeder::Feeder;
use cncr_core::gcode::{in2mm, is_m0, is_m1, is_m6, replace_m6};
use cncr_core::hub::{Socket, Sockets};
use cncr_core::program::{
    populate_context, preprocess, sync_global, tool_change_policy, tool_change_program,
    tool_probe_custom_commands, Line, MachineSnapshot,
};
use cncr_core::sender::{FilterResult, Sender, StreamingProtocol};
use cncr_core::serial::{SerialConnection, SerialEvent};
use cncr_core::util::{js_num_to_string, now_ms, num_or_zero, round_to, MessageSlot};
use cncr_core::workflow::Workflow;
use cncr_core::{
    Context, TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS,
    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING,
    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO, TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS,
    WRITE_SOURCE_CLIENT, WRITE_SOURCE_FEEDER,
};

use crate::constants::*;
use crate::runner::{either, truthy, Report, Runner, State};

const AXES: [&str; 6] = ["x", "y", "z", "a", "b", "c"];

/// Status report fields requested at init, in the JS order. `spc`, `sps`, `com` and
/// `cof` are dropped when the firmware answers `null` for them.
const SR_FIELDS: [&str; 35] = [
    "stat", "line", "vel", "feed", "unit", "coor", "momo", "plan", "path", "dist", "admo", "frmo",
    "tool", "posx", "posy", "posz", "posa", "posb", "posc", "mpox", "mpoy", "mpoz", "mpoa", "mpob",
    "mpoc", "spc", "sps", "com", "cof", "tofx", "tofy", "tofz", "tofa", "tofb", "tofc",
];

const PLANNER_DWELL: &str = "%wait ; Wait for the planner to empty";
const TOOL_PROBE: &str =
    "G91 [tool_probe_command] F[tool_probe_feedrate] Z[tool_probe_z - mposz - tool_probe_distance]";

/// Serial output, injectable for tests.
pub trait Port: Send + 'static {
    fn write(&mut self, data: &str);
}

impl Port for SerialConnection {
    fn write(&mut self, data: &str) {
        SerialConnection::write(self, data);
    }
}

/// Deferred work delivered back to the controller task.
#[derive(Debug)]
pub(crate) enum Internal {
    /// One command of the init sequence (`initController` `send`).
    InitSend(&'static str),
    /// Status report fields, then the initial queries.
    InitStatusReport,
    ControllerReady,
    EnergizeMotors,
    WatchdirLoaded {
        file: String,
        data: Result<String, String>,
        reply: Option<Reply>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum SenderStatus {
    None,
    Next,
    Ack,
}

/// What the common front half of both data filters produced.
enum Filtered {
    Skip,
    Wait {
        dwell: String,
        reason: Value,
    },
    Gcode {
        line: String,
        pauses: Vec<Value>,
        tool_change: bool,
    },
}

/// JS `String(value)` (`undefined` for a missing argument).
fn js_string(v: Option<&Value>) -> String {
    match v {
        None => "undefined".into(),
        Some(Value::Null) => "null".into(),
        Some(Value::String(s)) => s.clone(),
        Some(Value::Number(n)) => js_num_to_string(n.as_f64().unwrap_or(f64::NAN)),
        Some(v) => v.to_string(),
    }
}

fn object_or_empty(v: Option<&Value>) -> Context {
    match v {
        Some(Value::Object(m)) => m.clone(),
        _ => Context::new(),
    }
}

/// `ensurePositiveNumber`.
fn positive(n: f64) -> f64 {
    if n.is_nan() {
        0.0
    } else {
        n.max(0.0)
    }
}

/// Feed/spindle override step: `value` percent added to `current` (a ratio), clamped to 5..200 %.
fn override_ratio(current: f64, value: f64) -> f64 {
    let pct = current * 100.0 + value;
    if value == 0.0 {
        1.0
    } else if pct > 200.0 {
        2.0
    } else if pct < 5.0 {
        0.05
    } else {
        pct / 100.0
    }
}

fn filter_line(
    raw: &str,
    ctx: &mut Context,
    snap: &MachineSnapshot,
    global: &mut Value,
    slot: &mut MessageSlot,
    config: &ConfigStore,
) -> Filtered {
    populate_context(ctx, snap, global);
    if let Some(Value::Object(m)) = ctx.get_mut("modal") {
        m.insert(
            "path".into(),
            snap.modal.get("path").cloned().unwrap_or(Value::Null),
        );
    }
    let parsed = preprocess(raw, ctx);
    sync_global(ctx, global);
    let mut reason = |data: &str| json!({ "data": data, "msg": slot.take_or(json!(raw)) });
    match parsed {
        Line::Msg(msg) => {
            debug!("%msg: line={raw:?}");
            slot.put(json!(msg));
            Filtered::Skip
        }
        Line::Assigned => Filtered::Skip,
        Line::Wait(dwell) => Filtered::Wait {
            dwell,
            reason: reason("%wait"),
        },
        Line::Gcode { mut line, words } => {
            let mut pauses = Vec::new();
            if words.iter().any(|w| is_m0(w)) {
                pauses.push(reason("M0"));
            }
            if words.iter().any(|w| is_m1(w)) {
                pauses.push(reason("M1"));
            }
            let mut tool_change = false;
            if words.iter().any(|w| is_m6(w)) {
                let policy = tool_change_policy(config);
                let manual = [
                    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS,
                    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO,
                    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING,
                ]
                .contains(&policy);
                if manual || policy == TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS {
                    line = replace_m6(&line, |x| format!("({x})"));
                    pauses.push(reason("M6"));
                    tool_change = manual;
                }
            }
            Filtered::Gcode {
                line,
                pauses,
                tool_change,
            }
        }
    }
}

pub(crate) struct Controller<P: Port> {
    services: Services,
    options: ControllerOptions,
    port: Option<P>,
    sockets: Sockets,
    pub(crate) runner: Runner,
    pub(crate) ready: bool,
    /// Last emitted state / settings (`None` until the first emission).
    state: Option<State>,
    settings: Option<Map<String, Value>>,
    /// Status report fields the firmware does not support.
    sr_unsupported: Vec<&'static str>,
    blocked: bool,
    sender_status: SenderStatus,
    energize_motors_time: i64,
    energize_motors_timer: Option<JoinHandle<()>>,
    sender_finish_time: i64,
    message_slot: MessageSlot,
    probe: ProbeState,
    feeder: Feeder,
    sender: Sender,
    shared_context: Value,
    workflow: Workflow,
    internal: mpsc::UnboundedSender<Internal>,
}

impl<P: Port> Controller<P> {
    pub(crate) fn new(
        services: Services,
        options: ControllerOptions,
        port: P,
        internal: mpsc::UnboundedSender<Internal>,
    ) -> Self {
        Self {
            services,
            options,
            port: Some(port),
            sockets: Sockets::default(),
            runner: Runner::default(),
            ready: false,
            state: None,
            settings: None,
            sr_unsupported: Vec::new(),
            blocked: false,
            sender_status: SenderStatus::None,
            energize_motors_time: 0,
            energize_motors_timer: None,
            sender_finish_time: 0,
            message_slot: MessageSlot::default(),
            probe: ProbeState::default(),
            feeder: Feeder::new(),
            sender: Sender::new(StreamingProtocol::SendResponse),
            shared_context: json!({}),
            workflow: Workflow::default(),
            internal,
        }
    }

    /// The controller task: owns all state until the port closes.
    pub(crate) async fn run(
        mut self,
        mut rx: mpsc::UnboundedReceiver<ControllerMsg>,
        mut serial: mpsc::UnboundedReceiver<SerialEvent>,
        mut internal: mpsc::UnboundedReceiver<Internal>,
    ) {
        let period = Duration::from_millis(250);
        let mut query = interval_at(Instant::now() + period, period);
        query.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            tokio::select! {
                msg = rx.recv() => match msg {
                    Some(ControllerMsg::Close(done)) => {
                        self.close();
                        let _ = done.send(());
                        return;
                    }
                    Some(msg) => self.handle(msg),
                    None => {
                        self.close();
                        return;
                    }
                },
                ev = serial.recv() => match ev {
                    Some(SerialEvent::Line(line)) => self.handle_line(&line),
                    Some(SerialEvent::Closed(err)) => {
                        if let Some(err) = err {
                            warn!("Disconnected from serial port {:?}: {err}", self.options.port);
                        }
                        self.close();
                        return;
                    }
                    None => {
                        self.close();
                        return;
                    }
                },
                Some(msg) = internal.recv() => self.handle_internal(msg),
                _ = query.tick() => self.on_query_timer(),
            }
        }
    }

    fn handle(&mut self, msg: ControllerMsg) {
        match msg {
            ControllerMsg::AddSocket(socket) => self.add_socket(socket),
            ControllerMsg::RemoveSocket(id) => self.sockets.remove(id),
            ControllerMsg::Command { cmd, args, reply } => self.command(&cmd, &args, reply),
            ControllerMsg::Write { data, context } => self.write(&data, context),
            ControllerMsg::Writeln { data, context } => self.write(&format!("{data}\n"), context),
            ControllerMsg::Status(tx) => {
                let _ = tx.send(self.status());
            }
            ControllerMsg::Gcode(tx) => {
                let s = &self.sender.state;
                let _ = tx.send((self.sender.to_json(), s.name.clone(), s.gcode.clone()));
            }
            ControllerMsg::Close(done) => {
                self.close();
                let _ = done.send(());
            }
        }
    }

    pub(crate) fn handle_internal(&mut self, msg: Internal) {
        match msg {
            Internal::InitSend(cmd) => self.init_send(cmd),
            Internal::InitStatusReport => {
                // Minified (relaxed JSON) so it stays within the serial buffer limit.
                let fields: Vec<String> = SR_FIELDS
                    .iter()
                    .filter(|f| !self.sr_unsupported.contains(f))
                    .map(|f| format!("{f}:t"))
                    .collect();
                self.init_send(&format!("{{sr:{{{}}}}}", fields.join(",")));
                for cmd in ["{sys:n}", "{mt:n}", "{pwr:n}", "{qr:n}", "{sr:n}"] {
                    self.init_send(cmd);
                }
            }
            Internal::ControllerReady => self.trigger("controller:ready"),
            Internal::EnergizeMotors => {
                let now = now_ms();
                if self.energize_motors_time <= 0 {
                    self.energize_motors_time = now;
                }
                // Energize motors up to 30 minutes.
                if (now - self.energize_motors_time).abs() > 30 * 60 * 1000 {
                    self.command("energizeMotors:off", &[], None);
                    return;
                }
                self.feed(&["{me:0}"], Context::new());
                self.feed(&["{pwr:n}"], Context::new());
            }
            Internal::WatchdirLoaded {
                file,
                data,
                reply: r,
            } => match data {
                Ok(data) => self.gcode_load(file, &data, Context::new(), r),
                Err(e) => reply(r, Err(e)),
            },
        }
    }

    /// Spawns the timed JSON init sequence (`initController`).
    /// https://github.com/synthetos/TinyG/wiki/TinyG-Configuration-for-Firmware-Version-0.97
    pub(crate) fn spawn_init(&self) {
        let tx = self.internal.clone();
        tokio::spawn(async move {
            let send = |m: Internal| tx.send(m).is_ok();
            let delay = |ms: u64| tokio::time::sleep(Duration::from_millis(ms));
            // JSON mode, JSON verbosity 4 (line numbers), filtered queue and status
            // reports, 200 ms status interval.
            for cmd in ["{ej:1}", "{jv:4}", "{qv:1}", "{sv:1}", "{si:200}"] {
                if !send(Internal::InitSend(cmd)) {
                    return;
                }
            }
            // Probe spindle and coolant support (unsupported keys answer null).
            for cmd in [
                "{spe:n}", "{spd:n}", "{spc:n}", "{sps:n}", "{com:n}", "{cof:n}",
            ] {
                delay(100).await;
                if !send(Internal::InitSend(cmd)) {
                    return;
                }
            }
            delay(200).await;
            if !send(Internal::InitStatusReport) {
                return;
            }
            delay(50).await;
            send(Internal::ControllerReady);
        });
    }

    fn init_send(&mut self, cmd: &str) {
        if self.port.is_none() {
            return;
        }
        if cmd.len() >= SERIAL_BUFFER_LIMIT {
            error!("Exceeded serial buffer limit ({SERIAL_BUFFER_LIMIT}): cmd={cmd}");
            return;
        }
        self.feed(&[cmd], Context::new());
    }

    fn emit(&self, event: &str, args: &[Value]) {
        self.sockets.emit(event, args);
    }

    fn emit_workflow_state(&self) {
        self.emit("workflow:state", &[json!(self.workflow.state.as_str())]);
    }

    fn add_socket(&mut self, socket: Socket) {
        debug!("Add socket connection: id={}", socket.id);
        if self.port.is_some() {
            socket.emit(
                "serialport:open",
                &[json!({
                    "port": self.options.port,
                    "baudrate": self.options.baudrate,
                    "controllerType": TYPE,
                    "inuse": true,
                })],
            );
        }
        if let Some(settings) = self.settings.as_ref().filter(|s| !s.is_empty()) {
            socket.emit(
                "controller:settings",
                &[json!(TYPE), Value::Object(settings.clone())],
            );
        }
        if let Some(state) = &self.state {
            socket.emit("controller:state", &[json!(TYPE), json!(state)]);
        }
        socket.emit("feeder:status", &[self.feeder.to_json()]);
        socket.emit("sender:status", &[self.sender.to_json()]);
        let s = &self.sender.state;
        if !s.gcode.is_empty() {
            socket.emit(
                "gcode:load",
                &[
                    json!(s.name),
                    json!(s.gcode),
                    Value::Object(s.context.clone()),
                ],
            );
        }
        socket.emit("workflow:state", &[json!(self.workflow.state.as_str())]);
        self.sockets.add(socket);
    }

    fn status(&self) -> Value {
        let ids: Vec<String> = self.sockets.ids().iter().map(u64::to_string).collect();
        json!({
            "port": self.options.port,
            "baudrate": self.options.baudrate,
            "rtscts": self.options.rtscts,
            "sockets": ids,
            "ready": self.ready,
            "controller": {
                "type": TYPE,
                "settings": self.settings.clone().unwrap_or_default(),
                "state": self.state.as_ref().map_or_else(|| json!({}), |s| json!(s)),
                "footer": self.runner.footer,
            },
            "feeder": self.feeder.to_json(),
            "sender": self.sender.to_json(),
            "workflow": { "state": self.workflow.state.as_str() },
        })
    }

    fn close(&mut self) {
        let port = self.options.port.clone();
        self.ready = false;
        self.emit(
            "serialport:close",
            &[json!({ "port": port, "inuse": false })],
        );
        self.services.hub.broadcast(
            "serialport:change",
            &[json!({ "port": port, "inuse": false })],
        );
        self.port = None;
        if let Some(t) = self.energize_motors_timer.take() {
            t.abort();
        }
        self.services.registry.remove(&port);
    }

    // Writes

    fn write(&mut self, data: &str, mut context: Context) {
        if self.port.is_none() {
            error!("Serial port {:?} is not accessible", self.options.port);
            return;
        }
        context.insert("source".into(), json!(WRITE_SOURCE_CLIENT));
        self.emit("serialport:write", &[json!(data), Value::Object(context)]);
        if let Some(p) = self.port.as_mut() {
            p.write(data);
        }
    }

    fn writeln(&mut self, data: &str) {
        self.write(&format!("{data}\n"), Context::new());
    }

    fn write_feeder_line(&mut self, line: &str, mut context: Context) {
        if self.port.is_none() {
            error!("Serial port {:?} is not accessible", self.options.port);
            return;
        }
        if self.runner.is_alarm() {
            self.feeder.reset();
            warn!("Stopped sending G-code commands in Alarm mode");
            return;
        }
        let line = line.trim();
        if line.is_empty() {
            return;
        }
        let data = format!("{line}\n");
        context.insert("source".into(), json!(WRITE_SOURCE_FEEDER));
        self.emit("serialport:write", &[json!(data), Value::Object(context)]);
        if let Some(p) = self.port.as_mut() {
            p.write(&data);
        }
    }

    fn write_sender_line(&mut self, line: &str) {
        if self.port.is_none() {
            error!("Serial port {:?} is not accessible", self.options.port);
            return;
        }
        if self.workflow.is_idle() {
            error!("Unexpected workflow state: idle");
            return;
        }
        // Remove blanks to reduce the amount of bandwidth.
        let compact: String = line.chars().filter(|c| !c.is_whitespace()).collect();
        if compact.is_empty() {
            warn!("Expected non-empty line: N={}", self.sender.state.sent);
            return;
        }
        // Replace line numbers with the number of lines sent.
        let body = compact.strip_prefix('N').map_or(compact.as_str(), |rest| {
            rest.trim_start_matches(|c: char| c.is_ascii_digit())
        });
        let data = format!("N{}{body}\n", self.sender.state.sent);
        if let Some(p) = self.port.as_mut() {
            p.write(&data);
        }
    }

    // Feeder and sender

    fn snapshot(&self) -> MachineSnapshot {
        let sr = &self.runner.state.sr;
        let axes = |m: &Map<String, Value>| AXES.map(|a| num_or_zero(m.get(a)));
        MachineSnapshot {
            mpos: axes(&sr.mpos),
            wpos: axes(&sr.wpos),
            modal: json!(sr.modal),
            tool: num_or_zero(Some(&sr.tool)),
            params: Value::Null,
        }
    }

    /// Feeds lines (each item may hold several) like `command('gcode', ...)`.
    fn feed<S: AsRef<str>>(&mut self, items: &[S], context: Context) {
        let lines: Vec<String> = items
            .iter()
            .flat_map(|s| s.as_ref().split('\n'))
            .map(|l| l.strip_suffix('\r').unwrap_or(l))
            .filter(|l| !l.trim().is_empty())
            .map(str::to_string)
            .collect();
        self.feeder.feed(lines, context);
        if !self.feeder.is_pending() {
            self.feeder_next();
        }
    }

    fn trigger(&mut self, event: &str) {
        for gcode in self.services.trigger(event) {
            self.feed(&[gcode], Context::new());
        }
    }

    pub(crate) fn feeder_next(&mut self) {
        let snap = self.snapshot();
        let mut tool_change = false;
        let config = &self.services.config;
        let global = &mut self.shared_context;
        let slot = &mut self.message_slot;
        let sender = &mut self.sender;
        let out = self.feeder.next(&mut |raw, ctx| match filter_line(
            raw, ctx, &snap, global, slot, config,
        ) {
            Filtered::Skip => FilterResult::skip(),
            // As in cncjs, `%wait` from the feeder holds the sender, not the feeder.
            Filtered::Wait { dwell, reason } => {
                sender.hold(Some(reason));
                FilterResult::line(dwell)
            }
            Filtered::Gcode {
                line,
                pauses,
                tool_change: tc,
            } => {
                tool_change |= tc;
                FilterResult {
                    line,
                    hold: pauses.into_iter().next(),
                }
            }
        });
        if let Some((line, context)) = out {
            self.write_feeder_line(&line, context);
        }
        if tool_change {
            self.command("tool:change", &[], None);
        }
    }

    fn sender_next(&mut self) {
        let snap = self.snapshot();
        let mut running = self.workflow.is_running();
        let mut paused = false;
        let mut pause_gcode = Vec::new();
        let mut tool_change = false;
        let services = &self.services;
        let global = &mut self.shared_context;
        let slot = &mut self.message_slot;
        let out = self.sender.next(&mut |raw, ctx| match filter_line(
            raw,
            ctx,
            &snap,
            global,
            slot,
            &services.config,
        ) {
            Filtered::Skip => FilterResult::skip(),
            Filtered::Wait { dwell, reason } => FilterResult {
                line: dwell,
                hold: Some(reason),
            },
            Filtered::Gcode {
                line,
                pauses,
                tool_change: tc,
            } => {
                let mut hold = None;
                for reason in pauses {
                    pause_gcode.extend(services.trigger("gcode:pause"));
                    // `workflow.pause(reason)` holds the sender only when it was running.
                    if running {
                        running = false;
                        paused = true;
                        hold = Some(reason);
                    }
                }
                tool_change |= tc;
                FilterResult { line, hold }
            }
        });
        if out.started.is_some() {
            self.sender_finish_time = 0;
        }
        for line in &out.lines {
            self.write_sender_line(line);
        }
        if let Some(t) = out.ended {
            self.sender_finish_time = t;
        }
        if paused && self.workflow.pause() {
            self.emit_workflow_state();
        }
        for gcode in pause_gcode {
            self.feed(&[gcode], Context::new());
        }
        if tool_change {
            self.command("tool:change", &[], None);
        }
    }

    fn sender_ack_next(&mut self) {
        self.sender_status = SenderStatus::Ack;
        self.sender.ack();
        self.sender_next();
        self.sender_status = SenderStatus::Next;
    }

    // Workflow transitions with the JS listener side effects.

    fn workflow_start(&mut self) {
        if self.workflow.start() {
            self.emit_workflow_state();
            self.blocked = false;
            self.sender_status = SenderStatus::None;
            self.sender.rewind();
        }
    }

    fn workflow_stop(&mut self) {
        if self.workflow.stop() {
            self.emit_workflow_state();
            self.blocked = false;
            self.sender_status = SenderStatus::None;
            self.sender.rewind();
        }
    }

    fn workflow_pause(&mut self, reason: Option<Value>) {
        if self.workflow.pause() {
            self.emit_workflow_state();
            self.sender.hold(reason);
        }
    }

    fn workflow_resume(&mut self) {
        if self.workflow.resume() {
            self.emit_workflow_state();
            // Reset the feeder prior to resuming program execution.
            self.feeder.reset();
            self.sender.unhold();
            self.sender_next();
        }
    }

    // Serial input

    pub(crate) fn handle_line(&mut self, line: &str) {
        let data = line.trim_end();
        if data.is_empty() {
            return;
        }
        if self.workflow.is_idle() {
            self.emit("serialport:read", &[json!(data)]);
        }
        let (report, footer) = self.runner.parse(data);
        match report {
            Report::R(r) => self.on_receive_report(&r),
            Report::Qr(qr) => self.on_queue_report(qr),
            Report::Other => {}
        }
        if let Some(f) = footer {
            self.on_footer(&f);
        }
    }

    /// https://github.com/synthetos/g2/wiki/g2core-Communications
    fn on_receive_report(&mut self, r: &Value) {
        for key in ["spc", "sps", "com", "cof"] {
            if r.get(key) == Some(&Value::Null) && !self.sr_unsupported.contains(&key) {
                self.sr_unsupported.push(key);
            }
        }

        // Probe result: {"r":{"prb":{"e":1,"x":0,"y":0,"z":-1.5}}}
        if let Some(prb) =
            either(r, "prb").filter(|p| p.get("e").and_then(Value::as_f64) == Some(1.0))
        {
            debug!("[autolevel] TinyG probe response received: {prb}");
            // prb and mpos are always mm; wpos follows G20/G21.
            let sr = &self.runner.state.sr;
            let imperial = sr.modal.units == "G20";
            let num = |v: Option<&Value>| match v {
                Some(Value::Number(n)) => n.as_f64().unwrap_or(f64::NAN),
                Some(Value::String(s)) => cncr_core::util::js_str_to_num(s),
                Some(Value::Null) => 0.0,
                _ => f64::NAN,
            };
            let posz = num(sr.wpos.get("z"));
            let wco_z = round_to(
                num(sr.mpos.get("z")) - if imperial { in2mm(posz) } else { posz },
                3,
            );
            let prb_z = prb
                .get("z")
                .and_then(Value::as_f64)
                .filter(|z| z.is_finite())
                .unwrap_or(0.0);
            if let Some((update, completed)) = self.probe.record(prb_z - wco_z, imperial) {
                self.emit("autolevel:update", &[update]);
                if completed {
                    self.emit("autolevel:complete", &[]);
                }
            }
        }

        let (hold, sent, received) = (
            self.sender.state.hold,
            self.sender.state.sent,
            self.sender.state.received,
        );
        let check_n = || {
            let n = either(r, "n");
            if n.and_then(Value::as_f64) != Some(sent as f64) {
                warn!("Expression: n ({}) === sent ({sent})", js_string(n));
            }
        };

        if self.workflow.is_running() {
            check_n();
            self.sender_status = SenderStatus::Ack;
            if !self.blocked {
                self.sender.ack();
                self.sender_next();
                self.sender_status = SenderStatus::Next;
            }
            return;
        }

        // Paused manually or by M0/M1/M6: acknowledge what is still in flight.
        if self.workflow.is_paused() && received < sent {
            if !hold {
                error!("The sender does not hold off during the paused state");
            }
            if received + 1 >= sent {
                debug!(
                    "Stop sending G-code: hold={hold}, sent={sent}, received={}",
                    received + 1
                );
            }
            check_n();
            self.sender_ack_next();
            return;
        }

        self.feeder_next();
    }

    fn on_queue_report(&mut self, qr: f64) {
        if qr <= PLANNER_BUFFER_LOW_WATER_MARK {
            self.blocked = true;
            return;
        }
        if qr >= PLANNER_BUFFER_HIGH_WATER_MARK {
            self.blocked = false;
        }
        let pool = self.runner.planner_buffer_pool_size;

        if self.workflow.is_running() {
            let s = &self.sender.state;
            match self.sender_status {
                SenderStatus::Next if s.hold && s.received >= s.sent && qr >= pool => {
                    debug!(
                        "Continue sending G-code: sent={}, received={}, qr={qr}",
                        s.sent, s.received
                    );
                    self.sender.unhold();
                    self.sender_next();
                    self.sender_status = SenderStatus::Next;
                }
                SenderStatus::Ack => self.sender_ack_next(),
                _ => {}
            }
            return;
        }

        if self.workflow.is_paused() && self.sender_status == SenderStatus::Ack {
            let s = &self.sender.state;
            if s.received >= s.sent {
                error!("Expression: received ({}) < sent ({})", s.received, s.sent);
            }
            if !s.hold {
                error!("The sender does not hold off during the paused state");
            }
            self.sender_ack_next();
            return;
        }

        if self.feeder.hold {
            let data = self.feeder.hold_reason.as_ref().and_then(|r| r.get("data"));
            if data == Some(&json!("%wait")) && qr >= pool {
                self.feeder.unhold();
            }
        }
        self.feeder_next();
    }

    /// https://github.com/synthetos/g2/wiki/Status-Codes
    fn on_footer(&mut self, f: &[Value]) {
        let code = num_or_zero(f.get(1));
        if code != 0.0 {
            let msg = Some(code)
                .filter(|c| c.fract() == 0.0 && (0.0..256.0).contains(c))
                .map_or("undefined", |c| STATUS_CODES[c as usize]);
            let code = js_num_to_string(code);
            if self.workflow.is_running() {
                let ignore_errors = self
                    .services
                    .config
                    .get("state.controller.exception.ignoreErrors")
                    .is_some_and(|v| truthy(&v));
                let received = self.sender.state.received;
                let line = received
                    .checked_sub(1)
                    .and_then(|i| self.sender.state.lines.get(i))
                    .map_or("", |l| l.trim());
                let echo = format!("> {line} (ln={})", received + 1);
                self.emit("serialport:read", &[json!(echo)]);
                self.emit("serialport:read", &[json!(format!("error:{code} ({msg})"))]);
                if !ignore_errors {
                    self.workflow_pause(Some(json!({ "err": true, "msg": msg })));
                }
                return;
            }
            self.emit("serialport:read", &[json!(format!("error:{code} ({msg})"))]);
        }
        if self.workflow.is_idle() {
            self.feeder_next();
        }
    }

    pub(crate) fn on_query_timer(&mut self) {
        if self.port.is_none() {
            return;
        }
        if self.feeder.peek() {
            self.emit("feeder:status", &[self.feeder.to_json()]);
        }
        if self.sender.peek() {
            self.emit("sender:status", &[self.sender.to_json()]);
        }
        let zero_offset = self
            .state
            .as_ref()
            .is_some_and(|s| s.sr.wpos == self.runner.state.sr.wpos);
        if self.runner.take_settings_changed() {
            let settings = self.runner.settings.clone();
            self.emit(
                "controller:settings",
                &[json!(TYPE), Value::Object(settings.clone())],
            );
            self.settings = Some(settings);
        }
        if self.runner.take_state_changed() {
            let state = self.runner.state.clone();
            self.emit("controller:state", &[json!(TYPE), json!(state)]);
            self.state = Some(state);
        }
        if !self.ready {
            return;
        }
        // Stop the workflow once the machine has been idle for a while after the last line.
        if self.sender_finish_time > 0 {
            let machine_idle = zero_offset && self.runner.is_idle();
            let now = now_ms();
            let timespan = (now - self.sender_finish_time).abs();
            if !machine_idle {
                self.sender_finish_time = now;
            } else if timespan > 500 {
                debug!("Finished sending G-code: timespan={timespan}");
                self.sender_finish_time = 0;
                self.command("gcode:stop", &[], None);
            }
        }
    }

    // Commands

    fn gcode_load(&mut self, name: String, gcode: &str, context: Context, r: Option<Reply>) {
        // G4 P0 (or a tiny P) empties the planner and answers once the dwell is done.
        let ok = self
            .sender
            .load(&name, &format!("{gcode}\n{PLANNER_DWELL}"), context.clone());
        if !ok {
            reply(r, Err(format!("Invalid G-code: name={name}")));
            return;
        }
        self.emit(
            "gcode:load",
            &[
                json!(name),
                json!(self.sender.state.gcode),
                Value::Object(context),
            ],
        );
        self.trigger("gcode:load");
        debug!(
            "Load G-code: name={:?}, size={}, total={}",
            self.sender.state.name,
            self.sender.state.gcode.len(),
            self.sender.state.total
        );
        self.workflow_stop();
        reply(r, Ok(self.sender.to_json()));
    }

    fn find_macro(&self, id: Option<&Value>) -> Option<Value> {
        let found = self
            .services
            .config
            .get_array("macros")
            .into_iter()
            .find(|m| m.get("id") == id);
        if found.is_none() {
            error!("Cannot find the macro: id={}", js_string(id));
        }
        found
    }

    /// JS `command(cmd, ...args)`; the trailing callback is `reply`.
    pub(crate) fn command(&mut self, cmd: &str, args: &[Value], r: Option<Reply>) {
        let arg = |i: usize| args.get(i);
        let num_arg = |i: usize, default: f64| match arg(i) {
            None => default,
            Some(v) => v.as_f64().unwrap_or(f64::NAN),
        };
        match cmd {
            "gcode:load" => {
                let context = object_or_empty(arg(2));
                self.gcode_load(js_string(arg(0)), &js_string(arg(1)), context, r);
            }
            "gcode:unload" => {
                self.workflow_stop();
                self.sender.unload();
                self.emit("gcode:unload", &[]);
                self.trigger("gcode:unload");
            }
            "start" | "stop" | "pause" | "resume" => {
                warn!("Warning: The {cmd:?} command is deprecated and will be removed in a future release.");
                self.command(&format!("gcode:{cmd}"), args, r);
            }
            "gcode:start" => {
                self.trigger("gcode:start");
                self.workflow_start();
                self.feeder.reset();
                self.sender_next();
            }
            "gcode:stop" => {
                self.trigger("gcode:stop");
                self.workflow_stop();
                if arg(0).and_then(|o| o.get("force")).is_some_and(truthy) {
                    let fb = num_or_zero(self.settings.as_ref().and_then(|s| s.get("fb")));
                    if fb >= 101.0 {
                        // g2core 101.02: explicit job kill ^d, same effect as M30.
                        self.writeln("\x04");
                    } else if fb >= 100.0 {
                        self.writeln("\x04");
                        self.writeln("M30");
                    } else {
                        // Feedhold, flush the planner queue, end the program.
                        self.writeln("!");
                        self.writeln("%");
                        self.writeln("M30");
                    }
                }
                self.writeln("{\"qr\":\"\"}");
            }
            "gcode:pause" => {
                self.trigger("gcode:pause");
                self.workflow_pause(None);
                self.writeln("!");
                self.writeln("{\"qr\":\"\"}");
            }
            "gcode:resume" => {
                self.trigger("gcode:resume");
                self.writeln("~");
                self.workflow_resume();
                self.writeln("{\"qr\":\"\"}");
            }
            "feeder:feed" => self.command("gcode", &args[..args.len().min(2)], None),
            "feeder:start" => {
                if self.workflow.is_running() {
                    return;
                }
                self.writeln("~");
                self.writeln("{\"qr\":\"\"}");
                self.feeder.unhold();
                self.feeder_next();
            }
            "feeder:stop" => self.feeder.reset(),
            "feedhold" => {
                self.trigger("feedhold");
                self.writeln("!");
                self.writeln("{\"qr\":\"\"}");
            }
            "cyclestart" => {
                self.trigger("cyclestart");
                self.writeln("~");
                self.writeln("{\"qr\":\"\"}");
            }
            "statusreport" => self.writeln("{\"sr\":null}"),
            "homing" => {
                self.trigger("homing");
                self.writeln("G28.2 X0 Y0 Z0");
            }
            // Not supported by TinyG.
            "sleep" => self.trigger("sleep"),
            "unlock" => self.writeln("{clear:null}"),
            "reset" => {
                self.workflow_stop();
                self.feeder.reset();
                self.write("\x18", Context::new());
            }
            "jogCancel" => {
                self.writeln("!");
                self.writeln("%");
            }
            "feedOverride" | "spindleOverride" => {
                let key = if cmd == "feedOverride" { "mfo" } else { "sso" };
                let current = num_or_zero(self.runner.settings.get(key));
                let ratio = override_ratio(current, num_arg(0, f64::NAN));
                self.feed(
                    &[format!("{{{key}:{}}}", js_num_to_string(ratio))],
                    Context::new(),
                );
            }
            "rapidOverride" => {
                let mto = match num_arg(0, f64::NAN) {
                    v if v == 0.0 || v == 100.0 => "{mto:1}",
                    50.0 => "{mto:0.5}",
                    25.0 => "{mto:0.25}",
                    _ => return,
                };
                self.feed(&[mto], Context::new());
            }
            "energizeMotors:on" => {
                let mt = self
                    .state
                    .as_ref()
                    .map_or(0.0, |s| num_or_zero(Some(&s.mt)));
                if self.energize_motors_timer.is_some() || mt == 0.0 {
                    return;
                }
                self.feed(&["{me:0}"], Context::new());
                self.feed(&["{pwr:n}"], Context::new());
                let period = Duration::from_millis((mt * 1000.0 - 500.0).max(1.0) as u64);
                let tx = self.internal.clone();
                self.energize_motors_timer = Some(tokio::spawn(async move {
                    let mut timer = interval_at(Instant::now() + period, period);
                    loop {
                        timer.tick().await;
                        if tx.send(Internal::EnergizeMotors).is_err() {
                            return;
                        }
                    }
                }));
            }
            "energizeMotors:off" => {
                if let Some(t) = self.energize_motors_timer.take() {
                    t.abort();
                }
                self.energize_motors_time = 0;
                self.feed(&["{md:0}"], Context::new());
                self.feed(&["{pwr:n}"], Context::new());
            }
            "lasertest:on" => {
                let (power, duration, max_s) =
                    (num_arg(0, 0.0), num_arg(1, 0.0), num_arg(2, 1000.0));
                let mut commands = vec![format!(
                    "M3S{}",
                    js_num_to_string(positive(max_s * (power / 100.0)))
                )];
                if duration > 0.0 {
                    commands.push(format!(
                        "G4P{}",
                        js_num_to_string(positive(duration / 1000.0))
                    ));
                    commands.push("M5S0".into());
                }
                self.feed(&commands, Context::new());
            }
            "lasertest:off" => self.feed(&["M5S0"], Context::new()),
            "gcode" => {
                // `ensureArray(commands).join('\n')`
                let items: Vec<String> = match arg(0) {
                    None | Some(Value::Null) => vec![],
                    Some(Value::Array(a)) => a
                        .iter()
                        .map(|v| {
                            if v.is_null() {
                                String::new()
                            } else {
                                js_string(Some(v))
                            }
                        })
                        .collect(),
                    Some(v) => vec![js_string(Some(v))],
                };
                self.feed(&items, object_or_empty(arg(1)));
            }
            "macro:run" => {
                let Some(m) = self.find_macro(arg(0)) else {
                    return;
                };
                self.trigger("macro:run");
                let content = m.get("content").cloned().unwrap_or(Value::Null);
                self.command(
                    "gcode",
                    &[content, Value::Object(object_or_empty(arg(1)))],
                    None,
                );
                reply(r, Ok(Value::Null));
            }
            "macro:load" => {
                let Some(m) = self.find_macro(arg(0)) else {
                    return;
                };
                self.trigger("macro:load");
                let context = object_or_empty(arg(1));
                self.gcode_load(
                    js_string(m.get("name")),
                    &js_string(m.get("content")),
                    context,
                    r,
                );
            }
            "watchdir:load" => {
                let file = arg(0)
                    .and_then(Value::as_str)
                    .unwrap_or_default()
                    .to_string();
                let monitor = self.services.monitor.clone();
                let tx = self.internal.clone();
                tokio::spawn(async move {
                    let path = file.clone();
                    let data = tokio::task::spawn_blocking(move || {
                        monitor.read_file(&path).map_err(|e| e.to_string())
                    })
                    .await
                    .unwrap_or_else(|e| Err(e.to_string()));
                    let _ = tx.send(Internal::WatchdirLoaded {
                        file,
                        data,
                        reply: r,
                    });
                });
            }
            "tool:change" => {
                let config = &self.services.config;
                let block: Vec<String> = match tool_change_policy(config) {
                    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS => {
                        vec![
                            TOOL_PROBE.into(),
                            "G10 L20 P[mapWCSToPValue(modal.wcs)] Z[touch_plate_height]".into(),
                        ]
                    }
                    // Probe, wait a second, then set the tool length offset.
                    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO => {
                        vec![
                            TOOL_PROBE.into(),
                            "%wait 1".into(),
                            "{tofz:[posz - touch_plate_height]}".into(),
                        ]
                    }
                    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING => {
                        tool_probe_custom_commands(config)
                    }
                    _ => vec![],
                };
                let (lines, context) =
                    tool_change_program(config, &self.runner.state.sr.modal.units, &block);
                self.feed(&lines, context);
            }
            "autolevel:start" => {
                let params = arg(0).cloned().unwrap_or_else(|| json!({}));
                let lines = self.probe.start(&params);
                self.feed(&lines, Context::new());
            }
            "autolevel:stop" => {
                // Reset the machine to cancel the probe cycle immediately.
                self.command("reset", &[], None);
                self.probe.clear();
                info!("[autolevel:stop] Probe stopped and state cleared");
            }
            "autolevel:getProbeState" => reply(r, Ok(json!({ "state": self.probe.to_json() }))),
            "autolevel:loadFromFile" => {
                let path = js_string(arg(0));
                let result = self.probe.load_from_file(&path);
                if let Err(e) = &result {
                    error!("[autolevel:load] Error loading probe data: {e}");
                }
                reply(r, result);
            }
            "autolevel:saveToFile" => {
                let path = js_string(arg(0));
                let result = self.probe.save_to_file(&path);
                if let Err(e) = &result {
                    error!("[autolevel:saveToFile] Error saving probe data: {e}");
                }
                reply(r, result);
            }
            "autolevel:applyProbeCompensation" => {
                let result = apply_probe_compensation_params(arg(0).unwrap_or(&Value::Null));
                info!("[autolevel:applyProbeCompensation] Probe compensation applied");
                reply(r, Ok(result));
            }
            _ => error!("Unknown command: {cmd}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use cncr_core::controller::Registry;
    use cncr_core::hub::{Frame, Hub};
    use cncr_core::monitor::Monitor;
    use cncr_core::taskrunner::TaskRunner;
    use tokio::sync::oneshot;

    use super::*;

    #[derive(Default)]
    struct Rec(Vec<String>);

    impl Port for Rec {
        fn write(&mut self, data: &str) {
            self.0.push(data.to_string());
        }
    }

    struct T {
        c: Controller<Rec>,
        frames: mpsc::UnboundedReceiver<Frame>,
        internal: mpsc::UnboundedReceiver<Internal>,
    }

    fn setup_with(config: Value, watch_root: Option<std::path::PathBuf>) -> T {
        let hub = Arc::new(Hub::default());
        let services = Services {
            config: Arc::new(ConfigStore::in_memory(config)),
            hub: hub.clone(),
            tasks: Arc::new(TaskRunner::new(hub)),
            monitor: Arc::new(Monitor::new(watch_root)),
            registry: Arc::new(Registry::default()),
        };
        let options = ControllerOptions {
            port: "/dev/null".into(),
            baudrate: 115_200,
            rtscts: false,
            pin: Default::default(),
        };
        let (tx, internal) = mpsc::unbounded_channel();
        let mut c = Controller::new(services, options, Rec::default(), tx);
        let (socket, frames) = Socket::new();
        c.sockets.add(socket);
        T {
            c,
            frames,
            internal,
        }
    }

    fn setup() -> T {
        setup_with(json!({}), None)
    }

    impl T {
        fn writes(&self) -> Vec<String> {
            self.c.port.as_ref().unwrap().0.clone()
        }

        fn clear_writes(&mut self) {
            self.c.port.as_mut().unwrap().0.clear();
        }

        fn cmd(&mut self, cmd: &str, args: Value) {
            let args = args.as_array().cloned().unwrap_or_default();
            self.c.command(cmd, &args, None);
        }

        fn cmd_reply(&mut self, cmd: &str, args: Value) -> Option<Result<Value, String>> {
            let (tx, mut rx) = oneshot::channel();
            self.c.command(cmd, args.as_array().unwrap(), Some(tx));
            rx.try_recv().ok()
        }

        /// Emitted `(event, args)` since the last call.
        fn events(&mut self) -> Vec<(String, Vec<Value>)> {
            let mut out = Vec::new();
            while let Ok(f) = self.frames.try_recv() {
                let v: Value = serde_json::from_str(&f).unwrap();
                out.push((
                    v["event"].as_str().unwrap().to_string(),
                    v["args"].as_array().unwrap().clone(),
                ));
            }
            out
        }

        fn write_sources(&mut self, data: &str) -> Vec<String> {
            self.events()
                .into_iter()
                .filter(|(e, a)| e == "serialport:write" && a[0] == data)
                .map(|(_, a)| a[1]["source"].as_str().unwrap().to_string())
                .collect()
        }

        /// One line per acknowledgement, unholding M0/%wait parks in between.
        fn drain_feeder(&mut self) {
            while self.c.feeder.size() > 0 {
                self.c.feeder.unhold();
                self.c.feeder_next();
            }
        }
    }

    fn lines(l: &[&str]) -> Vec<String> {
        l.iter().map(|s| format!("{s}\n")).collect()
    }

    const ACK: &str = "{\"r\":{},\"f\":[1,0,8]}";

    #[test]
    fn simple_commands_write_their_codes() {
        let cases: [(&str, &[&str]); 7] = [
            ("feedhold", &["!", "{\"qr\":\"\"}"]),
            ("cyclestart", &["~", "{\"qr\":\"\"}"]),
            ("statusreport", &["{\"sr\":null}"]),
            ("homing", &["G28.2 X0 Y0 Z0"]),
            ("sleep", &[]),
            ("unlock", &["{clear:null}"]),
            ("jogCancel", &["!", "%"]),
        ];
        for (cmd, expected) in cases {
            let mut t = setup();
            t.cmd(cmd, json!([]));
            assert_eq!(t.writes(), lines(expected), "{cmd}");
        }
        let mut t = setup();
        t.cmd("feedhold", json!([]));
        assert_eq!(t.write_sources("!\n"), ["client"]);
    }

    #[test]
    fn reset_writes_a_bare_control_character_and_clears_the_feeder() {
        let mut t = setup();
        t.cmd("feeder:feed", json!([["G21", "G90"]]));
        t.cmd("reset", json!([]));
        assert_eq!(t.writes(), ["G21\n", "\u{18}"]);
        assert_eq!(t.write_sources("\u{18}"), ["client"]);
        assert_eq!(t.c.feeder.size(), 0);
    }

    #[test]
    fn overrides_clamp_and_go_through_the_feeder() {
        for (cmd, key) in [("feedOverride", "mfo"), ("spindleOverride", "sso")] {
            for (value, expected) in [(0, "1"), (150, "2"), (-96, "0.05"), (10, "1.1")] {
                let mut t = setup();
                t.cmd(cmd, json!([value]));
                let data = format!("{{{key}:{expected}}}\n");
                assert_eq!(t.writes(), std::slice::from_ref(&data));
                assert_eq!(t.write_sources(&data), ["feeder"]);
            }
        }
        for (value, expected) in [
            (0, "{mto:1}"),
            (100, "{mto:1}"),
            (50, "{mto:0.5}"),
            (25, "{mto:0.25}"),
        ] {
            let mut t = setup();
            t.cmd("rapidOverride", json!([value]));
            assert_eq!(t.writes(), lines(&[expected]));
        }
        let mut t = setup();
        t.cmd("rapidOverride", json!([75]));
        assert!(t.writes().is_empty());
    }

    #[tokio::test(start_paused = true)]
    async fn energize_motors_needs_a_motor_timeout_and_repeats_until_off() {
        let mut t = setup();
        t.cmd("energizeMotors:on", json!([]));
        assert!(t.writes().is_empty() && t.c.energize_motors_timer.is_none());

        let mut state = t.c.runner.state.clone();
        state.mt = json!(1);
        t.c.state = Some(state);
        t.cmd("energizeMotors:on", json!([]));
        t.cmd("energizeMotors:on", json!([]));
        assert_eq!(t.writes(), lines(&["{me:0}", "{pwr:n}"]));

        // Re-energized every `mt * 1000 - 500` ms.
        tokio::time::advance(Duration::from_millis(500)).await;
        let msg = t.internal.recv().await.unwrap();
        assert!(matches!(msg, Internal::EnergizeMotors));
        t.c.handle_internal(msg);
        assert_eq!(
            t.writes(),
            lines(&["{me:0}", "{pwr:n}", "{me:0}", "{pwr:n}"])
        );

        t.cmd("energizeMotors:off", json!([]));
        assert!(t.c.energize_motors_timer.is_none());
        assert_eq!(t.writes()[4..], lines(&["{md:0}", "{pwr:n}"]));
    }

    #[test]
    fn laser_test() {
        let mut t = setup();
        t.cmd("lasertest:on", json!([]));
        assert_eq!(t.writes(), ["M3S0\n"]);
        let mut t = setup();
        t.cmd("lasertest:on", json!([50, 250, 2000]));
        t.drain_feeder();
        assert_eq!(t.writes(), lines(&["M3S1000", "G4P0.25", "M5S0"]));
        let mut t = setup();
        t.cmd("lasertest:off", json!([]));
        assert_eq!(t.writes(), ["M5S0\n"]);
    }

    #[test]
    fn gcode_feeds_non_blank_lines_with_its_context() {
        let mut t = setup();
        t.cmd("gcode", json!(["G21\n\n  \r\nG90", {"myContext": true}]));
        assert_eq!(t.writes(), ["G21\n"]);
        let ev = t
            .events()
            .into_iter()
            .find(|(e, a)| e == "serialport:write" && a[0] == "G21\n")
            .unwrap();
        assert_eq!(
            (ev.1[1]["myContext"].clone(), ev.1[1]["source"].clone()),
            (json!(true), json!("feeder"))
        );
        // Idle: the receive report and its footer each advance the feeder (as in cncjs).
        t.cmd("gcode", json!([["G0 X1", "G0 X2"]]));
        t.c.handle_line(ACK);
        assert_eq!(t.writes(), lines(&["G21", "G90", "G0 X1"]));
    }

    #[test]
    fn feeder_start_and_stop() {
        let mut t = setup();
        t.cmd("feeder:feed", json!([["G0 X0", "G0 X1"]]));
        t.cmd("feeder:start", json!([]));
        assert_eq!(t.writes(), lines(&["G0 X0", "~", "{\"qr\":\"\"}", "G0 X1"]));

        let mut t = setup();
        t.cmd("gcode:start", json!([]));
        t.cmd("feeder:start", json!([]));
        assert!(t.writes().is_empty(), "no-op while running");

        let mut t = setup();
        t.cmd("feeder:feed", json!([["G0 X0", "G0 X1"]]));
        t.cmd("feeder:stop", json!([]));
        t.drain_feeder();
        assert_eq!(t.writes(), ["G0 X0\n"]);
    }

    #[test]
    fn gcode_load_appends_the_planner_dwell() {
        let mut t = setup();
        let r = t
            .cmd_reply("gcode:load", json!(["test.nc", "G21\nG90"]))
            .unwrap()
            .unwrap();
        assert_eq!(
            (r["name"].clone(), r["total"].clone()),
            (json!("test.nc"), json!(3))
        );
        assert_eq!(
            t.c.sender.state.gcode,
            "G21\nG90\n%wait ; Wait for the planner to empty"
        );
        assert!(t.writes().is_empty());
        assert_eq!(
            t.events().iter().filter(|(e, _)| e == "gcode:load").count(),
            1
        );

        let r = t
            .cmd_reply("gcode:load", json!(["test.nc", ""]))
            .unwrap()
            .unwrap();
        assert_eq!(r["total"], json!(1));
        let r = t
            .cmd_reply("gcode:load", json!(["test.nc"]))
            .unwrap()
            .unwrap();
        assert_eq!(
            r["total"],
            json!(2),
            "missing program is the string \"undefined\""
        );

        t.cmd("gcode:unload", json!([]));
        assert!(t.c.sender.state.gcode.is_empty() && t.writes().is_empty());
    }

    #[test]
    fn start_pause_resume_stop() {
        let mut t = setup();
        t.cmd("gcode:load", json!(["test.nc", "G21\nN100 G0 X1"]));
        t.cmd("gcode:start", json!([]));
        assert_eq!(t.writes(), ["N1G21\n"]);
        assert!(t.c.workflow.is_running());
        t.cmd("gcode:pause", json!([]));
        assert!(t.c.workflow.is_paused() && t.c.sender.state.hold);
        t.cmd("gcode:resume", json!([]));
        assert_eq!(
            t.writes(),
            lines(&[
                "N1G21",
                "!",
                "{\"qr\":\"\"}",
                "~",
                "N2G0X1",
                "{\"qr\":\"\"}"
            ])
        );
        assert!(t.c.workflow.is_running() && !t.c.sender.state.hold);
        t.cmd("gcode:stop", json!([]));
        assert_eq!(t.writes().last().unwrap(), "{\"qr\":\"\"}\n");
        assert!(t.c.workflow.is_idle());
    }

    #[test]
    fn forced_stop_depends_on_the_firmware_build() {
        for (fb, expected) in [
            (None, &["!", "%", "M30", "{\"qr\":\"\"}"][..]),
            (Some(100.04), &["\u{4}", "M30", "{\"qr\":\"\"}"][..]),
            (Some(101.02), &["\u{4}", "{\"qr\":\"\"}"][..]),
        ] {
            let mut t = setup();
            if let Some(fb) = fb {
                t.c.settings = Some(serde_json::from_value(json!({ "fb": fb })).unwrap());
            }
            t.cmd("gcode:stop", json!([{"force": true}]));
            assert_eq!(t.writes(), lines(expected));
        }
    }

    #[test]
    fn queue_reports_block_and_release_acknowledgements() {
        let mut t = setup();
        t.cmd("gcode:load", json!(["p", "G21\nG90\nG0 X1"]));
        t.cmd("gcode:start", json!([]));
        t.c.handle_line("{\"r\":{\"n\":1},\"f\":[1,0,8]}");
        assert_eq!(t.writes(), lines(&["N1G21", "N2G90"]));

        // Planner nearly full: the next response is not acknowledged...
        t.c.handle_line("{\"qr\":4}");
        t.c.handle_line("{\"r\":{\"n\":2},\"f\":[1,0,8]}");
        assert_eq!(t.writes().len(), 2);
        assert_eq!(t.c.sender.state.received, 1);
        // ...until the queue drains past the high water mark.
        t.c.handle_line("{\"qr\":8}");
        assert_eq!(t.writes(), lines(&["N1G21", "N2G90", "N3G0X1"]));
        assert_eq!(t.c.sender.state.received, 2);
    }

    #[test]
    fn errors_while_running_pause_unless_ignored() {
        let mut t = setup();
        t.cmd("gcode:load", json!(["p", "G21\nG90\nG0 X1"]));
        t.cmd("gcode:start", json!([]));
        t.events();
        t.c.handle_line("{\"r\":{\"n\":1},\"f\":[1,100,8]}");
        let reads: Vec<Value> = t
            .events()
            .into_iter()
            .filter(|(e, _)| e == "serialport:read")
            .map(|(_, a)| a[0].clone())
            .collect();
        let msg = STATUS_CODES[100];
        assert_eq!(
            reads,
            [json!("> G21 (ln=2)"), json!(format!("error:100 ({msg})"))]
        );
        assert!(t.c.workflow.is_paused());
        assert_eq!(
            t.c.sender.state.hold_reason,
            Some(json!({"err": true, "msg": msg}))
        );

        let mut t = setup_with(
            json!({"state": {"controller": {"exception": {"ignoreErrors": true}}}}),
            None,
        );
        t.cmd("gcode:load", json!(["p", "G21\nG90"]));
        t.cmd("gcode:start", json!([]));
        t.c.handle_line("{\"r\":{\"n\":1},\"f\":[1,100,8]}");
        assert!(t.c.workflow.is_running());
    }

    #[test]
    fn m0_pauses_the_program() {
        let mut t = setup();
        t.cmd("gcode:load", json!(["p", "%msg Insert part\nG21\nM0\nG90"]));
        t.cmd("gcode:start", json!([]));
        t.c.handle_line(ACK);
        assert_eq!(t.writes(), lines(&["N2G21", "N3M0"]));
        assert!(t.c.workflow.is_paused());
        assert_eq!(
            t.c.sender.state.hold_reason,
            Some(json!({"data": "M0", "msg": "Insert part"}))
        );
        // The M0 acknowledgement does not send more while paused.
        t.c.handle_line(ACK);
        assert_eq!(t.writes().len(), 2);
        t.cmd("gcode:resume", json!([]));
        assert_eq!(t.writes()[2..], lines(&["~", "N4G90", "{\"qr\":\"\"}"]));
    }

    #[test]
    fn m6_with_manual_policy_pauses_and_runs_the_tool_change() {
        let mut t = setup_with(json!({"tool": {"toolChangePolicy": 2}}), None);
        t.cmd("gcode:load", json!(["p", "G21\nT1 M6\nG90"]));
        t.cmd("gcode:start", json!([]));
        t.c.handle_line(ACK);
        assert!(t.c.workflow.is_paused());
        assert_eq!(
            t.c.sender.state.hold_reason.as_ref().unwrap()["data"],
            json!("M6")
        );
        assert_eq!(t.writes(), lines(&["N1G21", "N2T1(M6)", "G4 P0.5"]));
        assert!(t.c.feeder.size() > 0, "tool change program queued");
    }

    fn tool_change_writes(
        change_z: &str,
        change_xy: &str,
        probe_xy: &str,
        probe_z: &str,
        policy: &[&str],
    ) -> Vec<String> {
        let mut l = vec![
            "G4 P0.5", "M5", "G90", change_z, change_xy, "G4 P0.5", "M0", probe_xy, probe_z,
            "G4 P0.5",
        ];
        l.extend_from_slice(policy);
        l.extend([
            change_z, change_xy, "G4 P0.5", "M0", "G90", "G0 X0 Y0", "G0 Z0", "M5", "G4 P5",
        ]);
        lines(&l)
    }

    fn run_tool_change(config: Value, units: &str) -> Vec<String> {
        let mut t = setup_with(json!({ "tool": config }), None);
        let modal = &mut t.c.runner.state.sr.modal;
        modal.spindle = "M5".into();
        modal.wcs = "G54".into();
        modal.units = units.into();
        t.cmd("tool:change", json!([]));
        t.drain_feeder();
        t.writes()
    }

    #[test]
    fn tool_change_sequences() {
        let metric = |policy: &[&str]| {
            tool_change_writes(
                "G53 G0 Z0.000",
                "G53 G0 X0.000 Y0.000",
                "G53 G0 X0.000 Y0.000",
                "G53 G0 Z0.000",
                policy,
            )
        };
        for policy in [0, 1] {
            assert_eq!(
                run_tool_change(json!({"toolChangePolicy": policy}), "G21"),
                metric(&[])
            );
        }
        assert_eq!(
            run_tool_change(json!({"toolChangePolicy": 2}), "G21"),
            metric(&["G91 G38.2 F10 Z-1", "G10 L20 P1 Z0"])
        );
        assert_eq!(
            run_tool_change(json!({"toolChangePolicy": 3}), "G21"),
            metric(&["G91 G38.2 F10 Z-1", "G4 P1", "{tofz:0}"])
        );
        assert_eq!(
            run_tool_change(
                json!({"toolChangePolicy": 4, "toolProbeCustomCommands": "G38.2 Z-5 F25\nG92 Z0"}),
                "G21"
            ),
            metric(&["G38.2 Z-5 F25", "G92 Z0"])
        );
        let imperial = json!({"toolChangePolicy": 2, "toolChangeX": 25.4, "toolChangeZ": 25.4, "toolProbeDistance": 25.4,
            "toolProbeFeedrate": 254, "touchPlateHeight": 25.4});
        assert_eq!(
            run_tool_change(imperial, "G20"),
            tool_change_writes(
                "G53 G0 Z1.0000",
                "G53 G0 X1.0000 Y0.0000",
                "G53 G0 X0.0000 Y0.0000",
                "G53 G0 Z0.0000",
                &["G91 G38.2 F10 Z-1", "G10 L20 P1 Z1"]
            )
        );
    }

    #[test]
    fn autolevel_probe_sequences() {
        let mut t = setup();
        t.cmd(
            "autolevel:start",
            json!([{"mode": "test", "clearanceZ": 5, "startZ": 1, "endZ": -1, "feedrate": 100}]),
        );
        t.drain_feeder();
        assert_eq!(
            t.writes(),
            lines(&["G90", "G0 Z5", "G0 Z1", "G38.2 Z-1 F100", "G0 Z5"])
        );

        let mut t = setup();
        t.cmd(
            "autolevel:start",
            json!([{"mode": "full", "startX": 0, "endX": 10, "stepX": 10, "startY": 0, "endY": 10, "stepY": 10,
                "clearanceZ": 5, "startZ": 1, "endZ": -1, "feedrate": 100}]),
        );
        t.drain_feeder();
        let mut expected = Vec::new();
        for (i, (x, y)) in [(0, 0), (10, 0), (0, 10), (10, 10)].into_iter().enumerate() {
            let f = if i == 0 { 50 } else { 100 };
            expected.extend(lines(&[
                "G90",
                "G0 Z5",
                &format!("G0 X{x} Y{y}"),
                "G0 Z1",
                &format!("G38.2 Z-1 F{f}"),
                "G0 Z5",
            ]));
        }
        assert_eq!(t.writes(), expected);

        t.cmd("autolevel:stop", json!([]));
        assert_eq!(t.writes().last().unwrap(), "\u{18}");
        assert_eq!(t.c.probe, ProbeState::default());
    }

    #[test]
    fn probe_results_are_stored_at_the_grid_node_in_work_coordinates() {
        let mut t = setup();
        t.cmd(
            "autolevel:start",
            json!([{"startX": 0, "endX": 10, "stepX": 10, "startY": 0, "endY": 0, "stepY": 10,
                "clearanceZ": 5, "startZ": 1, "endZ": -1, "feedrate": 100}]),
        );
        // WCO z = mpos - wpos = -10 - (-2) = -8 mm.
        t.c.runner.state.sr.mpos.insert("z".into(), json!(-10));
        t.c.runner.state.sr.wpos.insert("z".into(), json!(-2));
        t.events();
        t.c.handle_line(
            "{\"r\":{\"prb\":{\"e\":1,\"x\":0.001,\"y\":0.001,\"z\":-11.5}},\"f\":[1,0,8]}",
        );
        t.c.handle_line("{\"r\":{\"prb\":{\"e\":0,\"x\":5,\"y\":5,\"z\":-30}},\"f\":[1,0,8]}");
        t.c.handle_line(
            "{\"r\":{\"prb\":{\"e\":1,\"x\":10.001,\"y\":-0.001,\"z\":-11}},\"f\":[1,0,8]}",
        );
        let p: Vec<(f64, f64, f64)> =
            t.c.probe
                .probed_positions
                .iter()
                .map(|p| (p.x, p.y, p.z))
                .collect();
        assert_eq!(p, [(0.0, 0.0, -3.5), (10.0, 0.0, -3.0)]);
        let events: Vec<String> = t
            .events()
            .into_iter()
            .map(|(e, _)| e)
            .filter(|e| e.starts_with("autolevel"))
            .collect();
        assert_eq!(
            events,
            ["autolevel:update", "autolevel:update", "autolevel:complete"]
        );
    }

    #[test]
    fn status_report_fields_skip_unsupported_keys() {
        let mut t = setup();
        t.c.handle_line("{\"r\":{\"spc\":null},\"f\":[1,0,4]}");
        t.c.handle_line("{\"r\":{\"com\":null},\"f\":[1,0,4]}");
        t.c.handle_internal(Internal::InitStatusReport);
        let sr = &t.writes()[0];
        assert!(sr.len() < SERIAL_BUFFER_LIMIT);
        assert_eq!(
            sr,
            "{sr:{stat:t,line:t,vel:t,feed:t,unit:t,coor:t,momo:t,plan:t,path:t,dist:t,admo:t,frmo:t,tool:t,\
             posx:t,posy:t,posz:t,posa:t,posb:t,posc:t,mpox:t,mpoy:t,mpoz:t,mpoa:t,mpob:t,mpoc:t,sps:t,cof:t,\
             tofx:t,tofy:t,tofz:t,tofa:t,tofb:t,tofc:t}}\n"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn init_sequence_order() {
        let mut t = setup();
        t.c.spawn_init();
        let mut sent = Vec::new();
        loop {
            match t.internal.recv().await.unwrap() {
                Internal::InitSend(cmd) => sent.push(cmd.to_string()),
                Internal::InitStatusReport => sent.push("<sr>".into()),
                Internal::ControllerReady => break,
                m => panic!("unexpected {m:?}"),
            }
        }
        assert_eq!(
            sent,
            [
                "{ej:1}", "{jv:4}", "{qv:1}", "{sv:1}", "{si:200}", "{spe:n}", "{spd:n}",
                "{spc:n}", "{sps:n}", "{com:n}", "{cof:n}", "<sr>"
            ]
        );
    }

    #[test]
    fn query_timer_emits_changes_and_stops_finished_programs() {
        let mut t = setup();
        t.c.ready = true;
        t.c.on_query_timer();
        let names: Vec<String> = t.events().into_iter().map(|(e, _)| e).collect();
        assert!(
            names.contains(&"controller:state".to_string())
                && names.contains(&"controller:settings".to_string())
        );
        t.c.on_query_timer();
        assert!(t.events().is_empty(), "nothing changed");

        t.cmd("gcode:load", json!(["p", "G21"]));
        t.cmd("gcode:start", json!([]));
        t.c.handle_line("{\"sr\":{\"stat\":1}}");
        t.c.on_query_timer();
        t.clear_writes();
        t.c.sender_finish_time = now_ms() - 1000;
        t.c.on_query_timer();
        assert!(t.c.workflow.is_idle());
        assert_eq!(t.writes(), ["{\"qr\":\"\"}\n"]);
    }

    #[tokio::test]
    async fn watchdir_load_reads_through_the_monitor() {
        let dir = std::env::temp_dir().join(format!("cncr-tinyg-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("part.nc"), "G21\nG90").unwrap();
        let mut t = setup_with(json!({}), Some(dir.clone()));

        let (tx, rx) = oneshot::channel();
        t.c.command("watchdir:load", &[json!("part.nc")], Some(tx));
        let msg = t.internal.recv().await.unwrap();
        t.c.handle_internal(msg);
        let r = rx.await.unwrap().unwrap();
        assert_eq!(
            (r["name"].clone(), r["total"].clone()),
            (json!("part.nc"), json!(3))
        );

        let (tx, rx) = oneshot::channel();
        t.c.command("watchdir:load", &[json!("missing.nc")], Some(tx));
        let msg = t.internal.recv().await.unwrap();
        t.c.handle_internal(msg);
        assert!(rx.await.unwrap().is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }
}

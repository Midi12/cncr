//! Smoothieware controller logic (port of cncjs `SmoothieController`). All of it runs on
//! the controller task; serial output goes through [`Port`] so it is testable without
//! hardware.

use std::sync::Arc;
use std::time::Duration;

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
use cncr_core::serial::SerialConnection;
use cncr_core::util::{
    js_num_to_string, js_str_to_num, now_ms, num_or_zero, round_to, MessageSlot, Throttle,
};
use cncr_core::workflow::Workflow;
use cncr_core::{
    Context, TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS,
    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING,
    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO, TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS,
    WRITE_SOURCE_CLIENT, WRITE_SOURCE_FEEDER,
};
use serde_json::{json, Value};
use tokio::sync::mpsc::UnboundedSender;
use tracing::{debug, error, info, warn};

use crate::runner::{Axes, Event, Runner, Settings, State, ACTIVE_STATE_HOLD};
use crate::TYPE;

/// Written without a trailing newline: cycle start, feed hold, status report, reset.
const REALTIME_COMMANDS: [&str; 4] = ["~", "!", "?", "\x18"];

/// The 128-byte RX buffer minus 8 bytes to prevent buffer overrun.
const BUFFER_SIZE: usize = 128 - 8;

const PROBE_TOOL: &str =
    "G91 [tool_probe_command] F[tool_probe_feedrate] Z[tool_probe_z - mposz - tool_probe_distance]";

/// Serial output.
pub trait Port {
    fn write(&mut self, data: &str);
}

impl Port for SerialConnection {
    fn write(&mut self, data: &str) {
        SerialConnection::write(self, data);
    }
}

/// Work finished off the controller task (JS `await delay(..)` / async callbacks).
pub enum Deferred {
    ControllerReady,
    WatchdirLoaded {
        file: String,
        result: Result<String, String>,
        reply: Option<Reply>,
    },
}

#[derive(Default)]
struct ActionMask {
    /// `$G` sent, waiting for the parser state line.
    query_parser_state: bool,
    /// Parser state received, waiting for its `ok`.
    query_parser_state_reply: bool,
    query_status_report: bool,
    /// A client sent `$G` / `?` itself: forward the reply.
    reply_parser_state: bool,
    reply_status_report: bool,
}

#[derive(Default)]
struct ActionTime {
    query_parser_state: i64,
    query_status_report: i64,
    sender_finish_time: i64,
}

#[derive(Clone, Copy, PartialEq)]
enum Source {
    Feeder,
    Sender,
}

/// Side effects the JS filters ran re-entrantly; run once the queue call returns.
enum Effect {
    Trigger(&'static str),
    EmitWorkflowState,
    ToolChange,
}

/// The feeder and sender `dataFilter`.
struct Filter<'a> {
    source: Source,
    snap: MachineSnapshot,
    slot: &'a mut MessageSlot,
    global: &'a mut Value,
    config: &'a ConfigStore,
    workflow: &'a mut Workflow,
    effects: Vec<Effect>,
    /// `%wait` fed through the feeder holds the sender (as in JS).
    sender_hold: Option<Value>,
}

impl<'a> Filter<'a> {
    fn new(
        source: Source,
        snap: MachineSnapshot,
        slot: &'a mut MessageSlot,
        global: &'a mut Value,
        config: &'a ConfigStore,
        workflow: &'a mut Workflow,
    ) -> Self {
        Self {
            source,
            snap,
            slot,
            global,
            config,
            workflow,
            effects: Vec::new(),
            sender_hold: None,
        }
    }

    fn run(&mut self, raw: &str, ctx: &mut Context) -> FilterResult {
        populate_context(ctx, &self.snap, self.global);
        // Smoothie does not expose G-code parameters to macros.
        ctx.remove("params");
        let result = self.filter(raw, ctx);
        sync_global(ctx, self.global);
        result
    }

    fn reason(&mut self, data: &str, raw: &str) -> Value {
        json!({ "data": data, "msg": self.slot.take_or(json!(raw)) })
    }

    fn filter(&mut self, raw: &str, ctx: &mut Context) -> FilterResult {
        match preprocess(raw, ctx) {
            Line::Msg(msg) => {
                self.slot.put(Value::String(msg));
                FilterResult::skip()
            }
            Line::Wait(dwell) => {
                let reason = self.reason("%wait", raw);
                let hold = match self.source {
                    Source::Feeder => {
                        self.sender_hold.get_or_insert(reason);
                        None
                    }
                    Source::Sender => Some(reason),
                };
                // Smoothie dwells in seconds with S (P is milliseconds).
                FilterResult {
                    line: format!("G4 S{}", &dwell["G4 P".len()..]),
                    hold,
                }
            }
            Line::Assigned => FilterResult::skip(),
            Line::Gcode { mut line, words } => {
                let mut hold = None;
                if words.iter().any(|w| is_m0(w)) {
                    self.pause("M0", raw, &mut hold);
                }
                if words.iter().any(|w| is_m1(w)) {
                    self.pause("M1", raw, &mut hold);
                }
                if words.iter().any(|w| is_m6(w)) {
                    let policy = tool_change_policy(self.config);
                    let manual = [
                        TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS,
                        TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO,
                        TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING,
                    ]
                    .contains(&policy);
                    if policy == TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS || manual {
                        line = replace_m6(&line, |x| format!("({x})"));
                        self.pause("M6", raw, &mut hold);
                    }
                    if manual {
                        self.effects.push(Effect::ToolChange);
                    }
                }
                FilterResult { line, hold }
            }
        }
    }

    /// The feeder holds itself; the sender pauses the workflow, whose listener holds it.
    fn pause(&mut self, data: &str, raw: &str, hold: &mut Option<Value>) {
        let reason = self.reason(data, raw);
        match self.source {
            Source::Feeder => {
                hold.get_or_insert(reason);
            }
            Source::Sender => {
                self.effects.push(Effect::Trigger("gcode:pause"));
                if self.workflow.pause() {
                    self.effects.push(Effect::EmitWorkflowState);
                    hold.get_or_insert(reason);
                }
            }
        }
    }
}

pub struct Controller<P: Port> {
    services: Services,
    options: ControllerOptions,
    port: Option<P>,
    sockets: Sockets,
    ready: bool,
    runner: Runner,
    /// Last emitted settings / state (`None` until the first timer tick).
    settings: Option<Settings>,
    state: Option<State>,
    mask: ActionMask,
    time: ActionTime,
    parser_state_throttle: Throttle,
    message_slot: MessageSlot,
    probe: ProbeState,
    feeder: Feeder,
    sender: Sender,
    workflow: Workflow,
    shared_context: Value,
    deferred: UnboundedSender<Deferred>,
}

fn context_arg(v: &Value) -> Context {
    v.as_object().cloned().unwrap_or_default()
}

/// `ensureArray(commands).join('\n')`
fn commands_text(v: &Value) -> String {
    match v {
        Value::Array(a) => a.iter().map(commands_text).collect::<Vec<_>>().join("\n"),
        Value::String(s) => s.clone(),
        Value::Null => String::new(),
        v => v.to_string(),
    }
}

fn wpos(state: Option<&State>) -> &[String] {
    state
        .and_then(|s| s.status.wpos.as_ref())
        .map_or(&[], |a| &a.0)
}

impl<P: Port> Controller<P> {
    pub fn new(
        services: Services,
        options: ControllerOptions,
        port: P,
        deferred: UnboundedSender<Deferred>,
    ) -> Self {
        Self {
            services,
            options,
            port: Some(port),
            sockets: Sockets::default(),
            ready: false,
            runner: Runner::default(),
            settings: None,
            state: None,
            mask: ActionMask::default(),
            time: ActionTime::default(),
            parser_state_throttle: Throttle::new(Duration::from_millis(500), true),
            message_slot: MessageSlot::default(),
            probe: ProbeState::default(),
            feeder: Feeder::new(),
            sender: Sender::new(StreamingProtocol::char_counting(BUFFER_SIZE)),
            workflow: Workflow::default(),
            shared_context: json!({}),
            deferred,
        }
    }

    /// Post-open work: set the ready flag and initialize the controller.
    pub fn start(&mut self) {
        self.ready = true;
        // Check if it is Smoothieware
        self.gcode("version", Context::new());
        let tx = self.deferred.clone();
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(50)).await;
            let _ = tx.send(Deferred::ControllerReady);
        });
    }

    /// Returns false once the controller has closed.
    pub fn handle(&mut self, msg: ControllerMsg) -> bool {
        match msg {
            ControllerMsg::AddSocket(socket) => self.add_socket(socket),
            ControllerMsg::RemoveSocket(id) => self.sockets.remove(id),
            ControllerMsg::Command { cmd, args, reply } => self.command(&cmd, &args, reply),
            ControllerMsg::Write { data, context } => self.write(&data, context),
            ControllerMsg::Writeln { data, context } => self.writeln(&data, context),
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
                return false;
            }
        }
        true
    }

    pub fn deferred(&mut self, d: Deferred) {
        match d {
            Deferred::ControllerReady => self.trigger("controller:ready"),
            Deferred::WatchdirLoaded {
                file,
                result: Ok(data),
                reply,
            } => self.load(&file, &data, Context::new(), reply),
            Deferred::WatchdirLoaded {
                result: Err(e),
                reply: reply_to,
                ..
            } => reply(reply_to, Err(e)),
        }
    }

    pub fn close(&mut self) {
        self.ready = false;
        let change = json!({ "port": self.options.port, "inuse": false });
        self.emit("serialport:close", std::slice::from_ref(&change));
        self.services.hub.broadcast("serialport:change", &[change]);
        self.port = None;
        self.services.registry.remove(&self.options.port);
    }

    fn emit(&self, event: &str, args: &[Value]) {
        self.sockets.emit(event, args);
    }

    fn read(&self, raw: &str) {
        self.emit("serialport:read", &[json!(raw)]);
    }

    fn emit_workflow_state(&self) {
        self.emit("workflow:state", &[json!(self.workflow.state.as_str())]);
    }

    fn status(&self) -> Value {
        json!({
            "port": self.options.port,
            "baudrate": self.options.baudrate,
            "rtscts": self.options.rtscts,
            "sockets": self.sockets.ids().iter().map(u64::to_string).collect::<Vec<_>>(),
            "ready": self.ready,
            "controller": {
                "type": TYPE,
                "settings": self.settings.as_ref().map_or_else(|| json!({}), |s| json!(s)),
                "state": self.state.as_ref().map_or_else(|| json!({}), |s| json!(s)),
            },
            "feeder": self.feeder.to_json(),
            "sender": self.sender.to_json(),
            "workflow": { "state": self.workflow.state.as_str() },
        })
    }

    fn add_socket(&mut self, socket: Socket) {
        if self.port.is_some() {
            let open = json!({ "port": self.options.port, "baudrate": self.options.baudrate, "controllerType": TYPE, "inuse": true });
            socket.emit("serialport:open", &[open]);
        }
        if let Some(settings) = &self.settings {
            socket.emit("controller:settings", &[json!(TYPE), json!(settings)]);
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

    /// Fires the event triggers; G-code triggers go through the feeder.
    fn trigger(&mut self, event: &str) {
        for commands in self.services.trigger(event) {
            self.gcode(&commands, Context::new());
        }
    }

    fn snapshot(&self) -> MachineSnapshot {
        let st = &self.runner.state;
        let pos = |a: &Option<Axes>| -> [f64; 6] {
            std::array::from_fn(|i| {
                let n = a.as_ref().map_or(0.0, |a| a.num(i));
                if n.is_nan() {
                    0.0
                } else {
                    n
                }
            })
        };
        MachineSnapshot {
            mpos: pos(&st.status.mpos),
            wpos: pos(&st.status.wpos),
            modal: st
                .parserstate
                .modal
                .as_ref()
                .map_or_else(|| json!({}), |m| json!(m)),
            tool: self.runner.tool(),
            params: Value::Null,
        }
    }

    fn run_effects(&mut self, effects: Vec<Effect>) {
        for effect in effects {
            match effect {
                Effect::Trigger(event) => self.trigger(event),
                Effect::EmitWorkflowState => self.emit_workflow_state(),
                Effect::ToolChange => self.tool_change(),
            }
        }
    }

    fn feeder_next(&mut self) {
        let snap = self.snapshot();
        let mut filter = Filter::new(
            Source::Feeder,
            snap,
            &mut self.message_slot,
            &mut self.shared_context,
            &self.services.config,
            &mut self.workflow,
        );
        let next = self.feeder.next(&mut |line, ctx| filter.run(line, ctx));
        let (effects, sender_hold) = (filter.effects, filter.sender_hold);
        if let Some(reason) = sender_hold {
            self.sender.hold(Some(reason));
        }
        if let Some((line, context)) = next {
            self.feeder_write(&line, context);
        }
        self.run_effects(effects);
    }

    fn feeder_write(&mut self, line: &str, mut context: Context) {
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
        if let Some(port) = &mut self.port {
            port.write(&data);
        }
    }

    fn sender_next(&mut self) {
        let snap = self.snapshot();
        let mut filter = Filter::new(
            Source::Sender,
            snap,
            &mut self.message_slot,
            &mut self.shared_context,
            &self.services.config,
            &mut self.workflow,
        );
        let out = self.sender.next(&mut |line, ctx| filter.run(line, ctx));
        let effects = filter.effects;
        if out.started.is_some() {
            self.time.sender_finish_time = 0;
        }
        if let Some(finish_time) = out.ended {
            self.time.sender_finish_time = finish_time;
        }
        for mut line in out.lines {
            let Some(port) = &mut self.port else {
                error!("Serial port {:?} is not accessible", self.options.port);
                break;
            };
            if self.workflow.is_idle() {
                error!("Unexpected workflow state: idle");
                continue;
            }
            if line.trim().is_empty() {
                warn!("Expected non-empty line: N={}", self.sender.state.sent);
                continue;
            }
            line.push('\n');
            port.write(&line);
        }
        self.run_effects(effects);
    }

    fn workflow_start(&mut self) {
        if self.workflow.start() {
            self.emit_workflow_state();
            self.sender.rewind();
        }
    }

    fn workflow_stop(&mut self) {
        if self.workflow.stop() {
            self.emit_workflow_state();
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
            // Reset feeder prior to resume program execution
            self.feeder.reset();
            self.sender.unhold();
            self.sender_next();
        }
    }

    /// Handles one line received from the serial port.
    pub fn parse(&mut self, data: &str) {
        let Some(event) = self.runner.parse(data) else {
            return;
        };
        let raw = data.trim_end();
        match event {
            // Smoothie status reports carry no RX buffer field, so the JS buffer
            // auto-sizing (`buf.rx`) never applies.
            Event::Status => {
                self.mask.query_status_report = false;
                if self.mask.reply_status_report {
                    self.mask.reply_status_report = false;
                    self.read(raw);
                }
            }
            Event::Ok => self.on_ok(raw),
            Event::Error(_) => self.on_error(raw),
            Event::Alarm(_) | Event::Version | Event::Others => self.read(raw),
            Event::Action(action) => match action.as_str() {
                "pause" => self.workflow_pause(Some(json!({ "data": "action:pause" }))),
                "resume" => self.workflow_resume(),
                "cancel" => self.workflow_stop(),
                _ => error!("Unknown action command: action:{action}"),
            },
            Event::ParserState => {
                self.mask.query_parser_state = false;
                self.mask.query_parser_state_reply = true;
                if self.mask.reply_parser_state {
                    self.read(raw);
                }
            }
            Event::Parameters { name, value } => {
                self.read(raw);
                if name == "PRB" {
                    self.on_probe(&value);
                }
            }
        }
    }

    fn on_ok(&mut self, raw: &str) {
        if self.mask.query_parser_state_reply {
            if self.mask.reply_parser_state {
                self.mask.reply_parser_state = false;
                self.read(raw);
            }
            self.mask.query_parser_state_reply = false;
            return;
        }
        let (hold, sent, received) = (
            self.sender.state.hold,
            self.sender.state.sent,
            self.sender.state.received,
        );
        if self.workflow.is_running() {
            if hold && received + 1 >= sent {
                debug!(
                    "Continue sending G-code: hold={hold}, sent={sent}, received={}",
                    received + 1
                );
                self.sender.unhold();
            }
            self.sender.ack();
            self.sender_next();
            return;
        }
        if self.workflow.is_paused() && received < sent {
            if !hold {
                error!("The sender does not hold off during the paused state");
            }
            self.sender.ack();
            self.sender_next();
            return;
        }
        self.read(raw);
        self.feeder_next();
    }

    fn on_error(&mut self, raw: &str) {
        if self.workflow.is_running() {
            let ignore_errors = self
                .services
                .config
                .get_or("state.controller.exception.ignoreErrors", false);
            let s = &self.sender.state;
            let line = s
                .received
                .checked_sub(1)
                .and_then(|i| s.lines.get(i))
                .map_or("", |l| l.trim());
            let echo = format!("> {line} (ln={})", s.received + 1);
            self.read(&echo);
            self.read(raw);
            if !ignore_errors {
                self.workflow_pause(Some(json!({ "err": true, "msg": raw })));
            }
            self.sender.ack();
            self.sender_next();
            return;
        }
        self.read(raw);
        self.feeder_next();
    }

    /// `[PRB:x,y,z:1]`: records the probed Z (work coordinates, mm) at the grid node.
    fn on_probe(&mut self, value: &Value) {
        if value.get("result").and_then(Value::as_f64) != Some(1.0) {
            return;
        }
        let status = &self.runner.state.status;
        let z_of = |a: &Option<Axes>| a.as_ref().map_or(f64::NAN, |a| a.num(2));
        let wco_z = round_to(z_of(&status.mpos) - z_of(&status.wpos), 3);
        let reported = value
            .get("z")
            .and_then(Value::as_str)
            .map_or(f64::NAN, js_str_to_num);
        let z = if reported.is_finite() { reported } else { 0.0 } - wco_z;
        let imperial = self.runner.modal_units() == "G20";
        let z_mm = if imperial { in2mm(z) } else { z };
        if let Some((update, completed)) = self.probe.record(z_mm, imperial) {
            self.emit("autolevel:update", &[update]);
            if completed {
                self.emit("autolevel:complete", &[]);
            }
        }
    }

    pub fn tick(&mut self) {
        if self.port.is_none() {
            return;
        }
        if self.feeder.peek() {
            self.emit("feeder:status", &[self.feeder.to_json()]);
        }
        if self.sender.peek() {
            self.emit("sender:status", &[self.sender.to_json()]);
        }
        let zero_offset = wpos(self.state.as_ref()) == wpos(Some(&self.runner.state));
        if self.settings.as_ref() != Some(&self.runner.settings) {
            let settings = self.runner.settings.clone();
            self.emit("controller:settings", &[json!(TYPE), json!(settings)]);
            self.settings = Some(settings);
        }
        if self.state.as_ref() != Some(&self.runner.state) {
            let state = self.runner.state.clone();
            self.emit("controller:state", &[json!(TYPE), json!(state)]);
            self.state = Some(state);
        }
        if !self.ready {
            return;
        }
        self.query_status_report();
        if self.parser_state_throttle.ready() {
            self.query_parser_state();
        }
        // Check if the machine has stopped movement after completion
        if self.time.sender_finish_time > 0 {
            let machine_idle = zero_offset && self.runner.is_idle();
            let now = now_ms();
            let timespan = (now - self.time.sender_finish_time).abs();
            if !machine_idle {
                self.time.sender_finish_time = now;
            } else if timespan > 500 {
                debug!("Finished sending G-code: timespan={timespan}");
                self.time.sender_finish_time = 0;
                self.command("gcode:stop", &[], None);
            }
        }
    }

    /// `?` is a realtime command: it does not consume the receive buffer.
    fn query_status_report(&mut self) {
        let now = now_ms();
        if self.time.query_status_report > 0 && (now - self.time.query_status_report).abs() >= 5000
        {
            debug!("Continue status report query");
            self.mask.query_status_report = false;
        }
        if self.mask.query_status_report {
            return;
        }
        if let Some(port) = &mut self.port {
            self.mask.query_status_report = true;
            self.time.query_status_report = now;
            port.write("?");
        }
    }

    fn query_parser_state(&mut self) {
        let now = now_ms();
        // Do not force `$G` while running a program: it eats 3 bytes of the receive
        // buffer each time (cncjs#176, #186).
        if self.workflow.is_idle()
            && self.runner.is_idle()
            && self.time.query_parser_state > 0
            && (now - self.time.query_parser_state).abs() >= 10000
        {
            debug!("Continue parser state query");
            self.mask.query_parser_state = false;
            self.mask.query_parser_state_reply = false;
        }
        if self.mask.query_parser_state || self.mask.query_parser_state_reply {
            return;
        }
        if let Some(port) = &mut self.port {
            self.mask.query_parser_state = true;
            self.mask.query_parser_state_reply = false;
            self.time.query_parser_state = now;
            port.write("$G\n");
        }
    }

    fn write(&mut self, data: &str, mut context: Context) {
        if self.port.is_none() {
            error!("Serial port {:?} is not accessible", self.options.port);
            return;
        }
        let cmd = data.trim();
        self.mask.reply_status_report |= cmd == "?";
        self.mask.reply_parser_state |= cmd == "$G";
        context.insert("source".into(), json!(WRITE_SOURCE_CLIENT));
        self.emit("serialport:write", &[json!(data), Value::Object(context)]);
        if let Some(port) = &mut self.port {
            port.write(data);
        }
    }

    fn writeln(&mut self, data: &str, context: Context) {
        if REALTIME_COMMANDS.contains(&data) {
            self.write(data, context);
        } else {
            self.write(&format!("{data}\n"), context);
        }
    }

    fn send(&mut self, data: &str) {
        self.writeln(data, Context::new());
    }

    /// The `gcode` command: non-blank lines go to the feeder.
    fn gcode(&mut self, commands: &str, context: Context) {
        let lines = commands
            .split('\n')
            .map(|l| l.strip_suffix('\r').unwrap_or(l))
            .filter(|l| !l.trim().is_empty())
            .map(str::to_string)
            .collect();
        self.feeder.feed(lines, context);
        if !self.feeder.is_pending() {
            self.feeder_next();
        }
    }

    fn load(&mut self, name: &str, gcode: &str, context: Context, reply_to: Option<Reply>) {
        // `%wait` (G4) empties the planner and answers `ok` once the dwell completes:
        // the fastest way to know the program finished.
        let program = format!("{gcode}\n%wait ; Wait for the planner to empty");
        if !self.sender.load(name, &program, context.clone()) {
            reply(reply_to, Err(format!("Invalid G-code: name={name}")));
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
            "Load G-code: name={name:?}, size={}, total={}",
            self.sender.state.gcode.len(),
            self.sender.state.total
        );
        self.workflow_stop();
        reply(reply_to, Ok(self.sender.to_json()));
    }

    fn reset(&mut self) {
        self.workflow_stop();
        self.feeder.reset();
        self.write("\x18", Context::new());
    }

    /// `feedOverride` / `spindleOverride`: a percentage step between 10 and 200, 0 resets
    /// to 100. The current value comes from the unguarded runner status as in JS, so an
    /// unset override yields `NaN`.
    fn apply_override(&mut self, value: &Value, spindle: bool) {
        let value = value.as_f64().unwrap_or(f64::NAN);
        let status = &self.runner.state.status;
        let current = if spindle { status.ov_s } else { status.ov_f }.unwrap_or(f64::NAN);
        // NaN stays NaN, as in JS.
        let next = if value == 0.0 {
            100.0
        } else {
            (current + value).clamp(10.0, 200.0)
        };
        let code = if spindle { "M221S" } else { "M220S" };
        self.gcode(&format!("{code}{}", js_num_to_string(next)), Context::new());
        // enforce state change
        let status = &mut self.runner.state.status;
        let slot = if spindle {
            &mut status.ov_s
        } else {
            &mut status.ov_f
        };
        *slot = Some(next);
    }

    fn find_macro(&self, id: &Value) -> Option<Value> {
        let found = self
            .services
            .config
            .get_array("macros")
            .into_iter()
            .find(|m| m.get("id") == Some(id));
        if found.is_none() {
            error!("Cannot find the macro: id={id}");
        }
        found
    }

    fn tool_change(&mut self) {
        let config = Arc::clone(&self.services.config);
        let probe_block = match tool_change_policy(&config) {
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS => {
                vec![
                    PROBE_TOOL.to_string(),
                    "G10 L20 P[mapWCSToPValue(modal.wcs)] Z[touch_plate_height]".into(),
                ]
            }
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO => {
                vec![
                    PROBE_TOOL.to_string(),
                    "%wait 1".into(),
                    "G43.1 Z[posz - touch_plate_height]".into(),
                ]
            }
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING => {
                tool_probe_custom_commands(&config)
            }
            _ => vec![],
        };
        let (lines, context) =
            tool_change_program(&config, self.runner.modal_units(), &probe_block);
        self.gcode(&lines.join("\n"), context);
    }

    pub fn command(&mut self, cmd: &str, args: &[Value], reply_to: Option<Reply>) {
        let arg = |i: usize| args.get(i).unwrap_or(&Value::Null);
        match cmd {
            "gcode:load" => {
                let (name, gcode) = (
                    arg(0).as_str().unwrap_or_default(),
                    arg(1).as_str().unwrap_or_default(),
                );
                self.load(name, gcode, context_arg(arg(2)), reply_to);
            }
            "gcode:unload" => {
                self.workflow_stop();
                self.sender.unload();
                self.emit("gcode:unload", &[]);
                self.trigger("gcode:unload");
            }
            "start" | "stop" | "pause" | "resume" => {
                warn!("Warning: The {cmd:?} command is deprecated and will be removed in a future release.");
                self.command(&format!("gcode:{cmd}"), args, reply_to);
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
                if self
                    .state
                    .as_ref()
                    .is_some_and(|s| s.status.active_state == ACTIVE_STATE_HOLD)
                {
                    self.send("resume");
                }
            }
            "gcode:pause" => {
                self.trigger("gcode:pause");
                self.workflow_pause(None);
                self.send("suspend");
            }
            "gcode:resume" => {
                self.trigger("gcode:resume");
                self.send("resume");
                self.workflow_resume();
            }
            "feeder:feed" | "gcode" => self.gcode(&commands_text(arg(0)), context_arg(arg(1))),
            "feeder:start" => {
                if self.workflow.is_running() {
                    return;
                }
                self.send("resume");
                self.feeder.unhold();
                self.feeder_next();
            }
            "feeder:stop" => self.feeder.reset(),
            "feedhold" => {
                self.trigger("feedhold");
                self.send("suspend");
            }
            "cyclestart" => {
                self.trigger("cyclestart");
                self.send("resume");
            }
            "statusreport" => self.write("?", Context::new()),
            "homing" => {
                self.trigger("homing");
                self.send("$H");
            }
            // Not supported
            "sleep" => self.trigger("sleep"),
            "unlock" => self.send("$X"),
            "reset" => self.reset(),
            "feedOverride" => self.apply_override(arg(0), false),
            "spindleOverride" => self.apply_override(arg(0), true),
            // Not supported
            "rapidOverride" => {}
            "lasertest:on" => {
                let power = num_or_zero(Some(arg(0))).max(0.0);
                let duration = num_or_zero(Some(arg(1)));
                self.send("M3");
                // Firing laser at <power>% power and entering manual mode
                self.send(&format!("fire {}", js_num_to_string(power)));
                if duration > 0.0 {
                    // Dwell P is in seconds when `grbl_mode` is set (http://smoothieware.org/g4)
                    self.send(&format!(
                        "G4P{}",
                        js_num_to_string((duration / 1000.0).max(0.0))
                    ));
                    // Turning laser off and returning to auto mode
                    self.send("fire off");
                    self.send("M5");
                }
            }
            "lasertest:off" => {
                self.send("fire off");
                self.send("M5");
            }
            "macro:run" => {
                let Some(m) = self.find_macro(arg(0)) else {
                    return;
                };
                self.trigger("macro:run");
                self.gcode(
                    m.get("content").and_then(Value::as_str).unwrap_or_default(),
                    context_arg(arg(1)),
                );
                reply(reply_to, Ok(Value::Null));
            }
            "macro:load" => {
                let Some(m) = self.find_macro(arg(0)) else {
                    return;
                };
                self.trigger("macro:load");
                let field = |k: &str| m.get(k).and_then(Value::as_str).unwrap_or_default();
                self.load(
                    field("name"),
                    field("content"),
                    context_arg(arg(1)),
                    reply_to,
                );
            }
            "watchdir:load" => {
                let file = arg(0).as_str().unwrap_or_default().to_string();
                let monitor = Arc::clone(&self.services.monitor);
                let tx = self.deferred.clone();
                tokio::spawn(async move {
                    let path = file.clone();
                    let result = tokio::task::spawn_blocking(move || {
                        monitor.read_file(&path).map_err(|e| e.to_string())
                    })
                    .await
                    .unwrap_or_else(|e| Err(e.to_string()));
                    let _ = tx.send(Deferred::WatchdirLoaded {
                        file,
                        result,
                        reply: reply_to,
                    });
                });
            }
            "tool:change" => self.tool_change(),
            "autolevel:start" => {
                let lines = self.probe.start(arg(0));
                self.gcode(&lines.join("\n"), Context::new());
            }
            "autolevel:stop" => {
                // Reset the machine to cancel the probe cycle immediately
                self.reset();
                self.probe.clear();
                info!("[autolevel:stop] Probe stopped and state cleared");
            }
            "autolevel:getProbeState" => {
                reply(reply_to, Ok(json!({ "state": self.probe.to_json() })))
            }
            "autolevel:loadFromFile" => {
                let result = self
                    .probe
                    .load_from_file(arg(0).as_str().unwrap_or_default());
                reply(reply_to, result);
            }
            "autolevel:saveToFile" => reply(
                reply_to,
                self.probe.save_to_file(arg(0).as_str().unwrap_or_default()),
            ),
            "autolevel:applyProbeCompensation" => {
                reply(reply_to, Ok(apply_probe_compensation_params(arg(0))))
            }
            _ => error!("Unknown command: {cmd}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncr_core::controller::Registry;
    use cncr_core::hub::{Frame, Hub};
    use cncr_core::monitor::Monitor;
    use cncr_core::taskrunner::TaskRunner;
    use cncr_core::TOOL_CHANGE_POLICY_SEND_M6_COMMANDS;
    use std::path::PathBuf;
    use tokio::sync::mpsc::{self, UnboundedReceiver};

    impl Port for Vec<String> {
        fn write(&mut self, data: &str) {
            self.push(data.to_string());
        }
    }

    type Ctl = Controller<Vec<String>>;

    fn create_with(
        config: Value,
        watch_root: Option<PathBuf>,
    ) -> (Ctl, UnboundedReceiver<Frame>, UnboundedReceiver<Deferred>) {
        let hub = Arc::new(Hub::default());
        let services = Services {
            config: Arc::new(ConfigStore::in_memory(config)),
            tasks: Arc::new(TaskRunner::new(Arc::clone(&hub))),
            hub,
            monitor: Arc::new(Monitor::new(watch_root)),
            registry: Arc::new(Registry::default()),
        };
        let options = ControllerOptions {
            port: "/dev/null".into(),
            baudrate: 115200,
            rtscts: false,
            pin: Default::default(),
        };
        let (tx, deferred) = mpsc::unbounded_channel();
        let mut ctl = Controller::new(services, options, Vec::new(), tx);
        let (socket, frames) = Socket::new();
        ctl.sockets.add(socket);
        (ctl, frames, deferred)
    }

    fn create(config: Value) -> (Ctl, UnboundedReceiver<Frame>) {
        let (ctl, frames, _) = create_with(config, None);
        (ctl, frames)
    }

    fn written(ctl: &Ctl) -> Vec<&str> {
        ctl.port
            .as_ref()
            .unwrap()
            .iter()
            .map(String::as_str)
            .collect()
    }

    /// `(event, args)` of every frame emitted so far.
    fn events(frames: &mut UnboundedReceiver<Frame>) -> Vec<(String, Value)> {
        let mut out = vec![];
        while let Ok(f) = frames.try_recv() {
            let v: Value = serde_json::from_str(&f).unwrap();
            out.push((v["event"].as_str().unwrap().to_string(), v["args"].clone()));
        }
        out
    }

    fn cmd(ctl: &mut Ctl, name: &str, args: &[Value]) {
        ctl.command(name, args, None);
    }

    fn cmd_reply(ctl: &mut Ctl, name: &str, args: &[Value]) -> Result<Value, String> {
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        ctl.command(name, args, Some(tx));
        rx.try_recv().expect("callback invoked")
    }

    /// Each `ok` releases the next queued feeder line; feeder holds (M0) are lifted too.
    fn pump_feeder(ctl: &mut Ctl) {
        loop {
            ctl.feeder.unhold();
            if ctl.feeder.size() == 0 && !ctl.feeder.is_pending() {
                return;
            }
            ctl.parse("ok");
        }
    }

    const PROGRAM: &str = "G0 X1\nG0 X2";

    #[test]
    fn simple_commands() {
        let (mut ctl, mut frames) = create(json!({}));
        cmd(&mut ctl, "feedhold", &[]);
        assert_eq!(
            events(&mut frames)[0],
            (
                "serialport:write".to_string(),
                json!(["suspend\n", {"source": WRITE_SOURCE_CLIENT}])
            )
        );
        for name in [
            "cyclestart",
            "statusreport",
            "homing",
            "sleep",
            "unlock",
            "rapidOverride",
            "nonexistent",
        ] {
            cmd(&mut ctl, name, &[json!(50)]);
        }
        assert_eq!(
            written(&ctl),
            ["suspend\n", "resume\n", "?", "$H\n", "$X\n"]
        );
        assert!(ctl.mask.reply_status_report);
    }

    #[test]
    fn reset_writes_control_x_and_drops_the_feeder_queue() {
        let (mut ctl, _) = create(json!({}));
        cmd(&mut ctl, "gcode", &[json!("G0 X1\nG0 X2")]);
        cmd(&mut ctl, "reset", &[]);
        assert_eq!(written(&ctl), ["G0 X1\n", "\x18"]);
        assert_eq!(ctl.feeder.size(), 0);
    }

    #[test]
    fn overrides() {
        for (value, seeded, expected) in [
            (0.0, 150.0, 100.0),
            (100.0, 150.0, 200.0),
            (-10.0, 15.0, 10.0),
            (30.0, 100.0, 130.0),
        ] {
            let (mut ctl, mut frames) = create(json!({}));
            ctl.runner.state.status.ov_f = Some(seeded);
            ctl.runner.state.status.ov_s = Some(seeded);
            cmd(&mut ctl, "feedOverride", &[json!(value)]);
            pump_feeder(&mut ctl);
            cmd(&mut ctl, "spindleOverride", &[json!(value)]);
            let e = js_num_to_string(expected);
            assert_eq!(
                written(&ctl),
                [format!("M220S{e}\n"), format!("M221S{e}\n")]
            );
            assert_eq!(
                (ctl.runner.state.status.ov_f, ctl.runner.state.status.ov_s),
                (Some(expected), Some(expected))
            );
            assert_eq!(
                events(&mut frames)[0].1[1]["source"],
                json!(WRITE_SOURCE_FEEDER)
            );
        }
        // The runner status is unguarded: without a seeded value the sum is NaN.
        let (mut ctl, _) = create(json!({}));
        cmd(&mut ctl, "feedOverride", &[json!(30)]);
        assert_eq!(written(&ctl), ["M220SNaN\n"]);
        assert!(ctl.runner.state.status.ov_f.unwrap().is_nan());
    }

    #[test]
    fn lasertest() {
        let (mut ctl, _) = create(json!({}));
        cmd(&mut ctl, "lasertest:on", &[]);
        cmd(&mut ctl, "lasertest:on", &[json!(50), json!(1000)]);
        cmd(&mut ctl, "lasertest:off", &[]);
        assert_eq!(
            written(&ctl),
            [
                "M3\n",
                "fire 0\n",
                "M3\n",
                "fire 50\n",
                "G4P1\n",
                "fire off\n",
                "M5\n",
                "fire off\n",
                "M5\n"
            ]
        );
    }

    #[test]
    fn load_appends_the_planner_dwell() {
        let (mut ctl, _) = create(json!({}));
        let snapshot = cmd_reply(
            &mut ctl,
            "gcode:load",
            &[json!("gcode-test"), json!(PROGRAM)],
        )
        .unwrap();
        assert_eq!(
            (&snapshot["name"], &snapshot["total"], &snapshot["sent"]),
            (&json!("gcode-test"), &json!(3), &json!(0))
        );
        // Even empty G-code loads (the dwell masks the invalid-input path, as in JS).
        let snapshot = cmd_reply(&mut ctl, "gcode:load", &[json!("empty"), json!("")]).unwrap();
        assert_eq!(snapshot["total"], json!(1));
        cmd(&mut ctl, "gcode:unload", &[]);
        assert_eq!(
            (ctl.sender.state.name.as_str(), ctl.sender.state.total),
            ("", 0)
        );
        assert!(written(&ctl).is_empty());
    }

    #[test]
    fn start_pause_resume_stop() {
        let (mut ctl, mut frames) = create(json!({}));
        cmd(
            &mut ctl,
            "gcode:load",
            &[json!("gcode-test"), json!(PROGRAM)],
        );
        cmd(&mut ctl, "gcode:start", &[]);
        assert_eq!(written(&ctl), ["G0 X1\n", "G0 X2\n", "G4 S0.5\n"]);
        cmd(&mut ctl, "gcode:pause", &[]);
        cmd(&mut ctl, "gcode:resume", &[]);
        assert_eq!(written(&ctl)[3..], ["suspend\n", "resume\n"]);
        let states: Vec<Value> = events(&mut frames)
            .into_iter()
            .filter(|(e, _)| e == "workflow:state")
            .map(|(_, a)| a[0].clone())
            .collect();
        assert_eq!(
            states,
            [json!("running"), json!("paused"), json!("running")]
        );
        // gcode:stop resumes only a machine on hold.
        cmd(&mut ctl, "gcode:stop", &[]);
        assert_eq!(written(&ctl).len(), 5);
        cmd(&mut ctl, "gcode:start", &[]);
        ctl.state = Some(State {
            status: crate::runner::Status {
                active_state: "Hold".into(),
                ..Default::default()
            },
            ..ctl.runner.state.clone()
        });
        cmd(&mut ctl, "stop", &[]);
        assert_eq!(written(&ctl).last(), Some(&"resume\n"));
    }

    #[test]
    fn resume_writes_the_next_line_once_the_buffer_has_space() {
        let (mut ctl, _) = create(json!({}));
        cmd(
            &mut ctl,
            "gcode:load",
            &[json!("p"), json!(vec!["G0 X1"; 30].join("\n"))],
        );
        cmd(&mut ctl, "gcode:start", &[]);
        // 19 × 6 bytes fill the 120-byte buffer.
        assert_eq!(written(&ctl).len(), 19);
        cmd(&mut ctl, "gcode:pause", &[]);
        cmd(&mut ctl, "gcode:resume", &[]);
        assert_eq!(written(&ctl)[19..], ["suspend\n", "resume\n", "G0 X1\n"]);
    }

    #[test]
    fn m0_in_a_program_pauses_until_resumed() {
        let (mut ctl, mut frames) = create(json!({}));
        cmd(
            &mut ctl,
            "gcode:load",
            &[json!("p"), json!("%msg Change bit\nG0 X1\nM0\nG0 X2")],
        );
        cmd(&mut ctl, "gcode:start", &[]);
        assert_eq!(written(&ctl), ["G0 X1\n", "M0\n"]);
        assert!(ctl.workflow.is_paused());
        assert_eq!(
            ctl.sender.state.hold_reason,
            Some(json!({"data": "M0", "msg": "Change bit"}))
        );
        assert!(events(&mut frames)
            .iter()
            .any(|(e, a)| e == "workflow:state" && a[0] == "paused"));
        // Paused: acknowledgements are still credited to the sender.
        ctl.parse("ok");
        ctl.parse("ok");
        assert_eq!(ctl.sender.state.received, 3);
        cmd(&mut ctl, "gcode:resume", &[]);
        assert_eq!(written(&ctl)[2..], ["resume\n", "G0 X2\n", "G4 S0.5\n"]);
    }

    #[test]
    fn error_while_running_pauses_the_program() {
        // Longer than the RX buffer, so the end-of-program `%wait` hold is not reached.
        let (mut ctl, mut frames) = create(json!({}));
        cmd(
            &mut ctl,
            "gcode:load",
            &[json!("p"), json!(vec!["G0 X1"; 30].join("\n"))],
        );
        cmd(&mut ctl, "gcode:start", &[]);
        ctl.parse("ok");
        events(&mut frames);
        ctl.parse("error: Unsupported command");
        let reads: Vec<Value> = events(&mut frames)
            .into_iter()
            .filter(|(e, _)| e == "serialport:read")
            .map(|(_, a)| a[0].clone())
            .collect();
        assert_eq!(
            reads,
            [json!("> G0 X1 (ln=2)"), json!("error: Unsupported command")]
        );
        assert!(ctl.workflow.is_paused());
        assert_eq!(
            ctl.sender.state.hold_reason,
            Some(json!({"err": true, "msg": "error: Unsupported command"}))
        );
        assert_eq!(ctl.sender.state.received, 2);

        // ignoreErrors keeps running.
        let (mut ctl, _) =
            create(json!({"state": {"controller": {"exception": {"ignoreErrors": true}}}}));
        cmd(&mut ctl, "gcode:load", &[json!("p"), json!(PROGRAM)]);
        cmd(&mut ctl, "gcode:start", &[]);
        ctl.parse("error: Unsupported command");
        assert!(ctl.workflow.is_running());
    }

    #[test]
    fn m6_policies_in_the_feeder() {
        let (mut ctl, _) =
            create(json!({"tool": {"toolChangePolicy": TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS}}));
        cmd(&mut ctl, "gcode", &[json!("T1 M6\nG0 X1")]);
        assert_eq!(written(&ctl), ["T1 (M6)\n"]);
        assert_eq!(
            ctl.feeder.hold_reason,
            Some(json!({"data": "M6", "msg": "T1 M6"}))
        );
        let (mut ctl, _) =
            create(json!({"tool": {"toolChangePolicy": TOOL_CHANGE_POLICY_SEND_M6_COMMANDS}}));
        cmd(&mut ctl, "gcode", &[json!("T1 M6")]);
        assert_eq!(written(&ctl), ["T1 M6\n"]);
        assert!(!ctl.feeder.hold);
    }

    #[test]
    fn feeder_commands() {
        let (mut ctl, mut frames) = create(json!({}));
        cmd(&mut ctl, "gcode", &[json!("G0 X1\n\r\nG0 X2")]);
        assert_eq!(written(&ctl), ["G0 X1\n"]);
        pump_feeder(&mut ctl);
        assert_eq!(written(&ctl), ["G0 X1\n", "G0 X2\n"]);
        let sources: Vec<Value> = events(&mut frames)
            .into_iter()
            .filter(|(e, _)| e == "serialport:write")
            .map(|(_, a)| a[1]["source"].clone())
            .collect();
        assert_eq!(
            sources,
            [json!(WRITE_SOURCE_FEEDER), json!(WRITE_SOURCE_FEEDER)]
        );

        let (mut ctl, _) = create(json!({}));
        cmd(&mut ctl, "feeder:feed", &[json!(["G0 X5", "G0 X6"])]);
        cmd(&mut ctl, "feeder:start", &[]);
        assert_eq!(written(&ctl), ["G0 X5\n", "resume\n", "G0 X6\n"]);

        let (mut ctl, _) = create(json!({}));
        cmd(&mut ctl, "feeder:feed", &[json!("G0 X5\nG0 X6")]);
        cmd(&mut ctl, "feeder:stop", &[]);
        pump_feeder(&mut ctl);
        assert_eq!(written(&ctl), ["G0 X5\n"]);

        // feeder:start is ignored while a program runs.
        let (mut ctl, _) = create(json!({}));
        cmd(&mut ctl, "gcode:load", &[json!("p"), json!(PROGRAM)]);
        cmd(&mut ctl, "gcode:start", &[]);
        cmd(&mut ctl, "feeder:feed", &[json!("G0 X5")]);
        cmd(&mut ctl, "feeder:start", &[]);
        assert_eq!(
            written(&ctl),
            ["G0 X1\n", "G0 X2\n", "G4 S0.5\n", "G0 X5\n"]
        );
    }

    #[test]
    fn feeder_stops_in_alarm() {
        let (mut ctl, _) = create(json!({}));
        ctl.parse("<Alarm|MPos:0.0000,0.0000,0.0000|WPos:0.0000,0.0000,0.0000>");
        cmd(&mut ctl, "gcode", &[json!("G0 X1\nG0 X2")]);
        assert!(written(&ctl).is_empty());
        assert_eq!(ctl.feeder.size(), 0);
    }

    #[test]
    fn macros() {
        let macros = json!({"macros": [{"id": "m1", "name": "Macro One", "content": PROGRAM}]});
        let (mut ctl, _) = create(macros.clone());
        assert_eq!(
            cmd_reply(&mut ctl, "macro:run", &[json!("m1"), json!({})]),
            Ok(Value::Null)
        );
        pump_feeder(&mut ctl);
        assert_eq!(written(&ctl), ["G0 X1\n", "G0 X2\n"]);

        let (mut ctl, _) = create(macros);
        let snapshot = cmd_reply(&mut ctl, "macro:load", &[json!("m1"), json!({})]).unwrap();
        assert_eq!(
            (&snapshot["name"], &snapshot["total"]),
            (&json!("Macro One"), &json!(3))
        );
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        ctl.command("macro:run", &[json!("missing")], Some(tx));
        assert!(rx.try_recv().is_err(), "missing macros never answer");
        assert!(written(&ctl).is_empty());
    }

    #[tokio::test]
    async fn watchdir_load_reads_through_the_monitor() {
        let root = std::env::temp_dir().join(format!("cncr-smoothie-watch-{}", std::process::id()));
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(root.join("x.nc"), PROGRAM).unwrap();
        let (mut ctl, _, mut deferred) = create_with(json!({}), Some(root.clone()));
        let (tx, rx) = tokio::sync::oneshot::channel();
        ctl.command("watchdir:load", &[json!("x.nc")], Some(tx));
        ctl.deferred(deferred.recv().await.unwrap());
        let snapshot = rx.await.unwrap().unwrap();
        assert_eq!(
            (&snapshot["name"], &snapshot["total"]),
            (&json!("x.nc"), &json!(3))
        );
        let (tx, rx) = tokio::sync::oneshot::channel();
        ctl.command("watchdir:load", &[json!("missing.nc")], Some(tx));
        ctl.deferred(deferred.recv().await.unwrap());
        assert!(rx.await.unwrap().is_err());
        std::fs::remove_dir_all(root).unwrap();
    }

    fn tool_change(policy: i64, tool: Value, units: &str) -> Vec<String> {
        let mut tool_config = json!({
            "toolChangePolicy": policy, "toolChangeX": 0, "toolChangeY": 0, "toolChangeZ": 0,
            "toolProbeX": 0, "toolProbeY": 0, "toolProbeZ": 0, "toolProbeCommand": "G38.2",
            "toolProbeDistance": 1, "toolProbeFeedrate": 10, "touchPlateHeight": 0,
        });
        tool_config
            .as_object_mut()
            .unwrap()
            .extend(tool.as_object().unwrap().clone());
        let (mut ctl, _) = create(json!({ "tool": tool_config }));
        ctl.parse(&format!(
            "[G0 G54 G17 {units} G90 G94 M0 M5 M9 T0 F2540. S0.]"
        ));
        cmd(&mut ctl, "tool:change", &[]);
        pump_feeder(&mut ctl);
        ctl.port.take().unwrap()
    }

    fn metric_sequence(policy_lines: &[&str]) -> Vec<String> {
        let head = [
            "G4 S0.5",
            "M5",
            "G90",
            "G53 G0 Z0.000",
            "G53 G0 X0.000 Y0.000",
            "G4 S0.5",
            "M0",
        ];
        let probe = ["G53 G0 X0.000 Y0.000", "G53 G0 Z0.000", "G4 S0.5"];
        let tail = [
            "G53 G0 Z0.000",
            "G53 G0 X0.000 Y0.000",
            "G4 S0.5",
            "M0",
            "G90",
            "G0 X0 Y0",
            "G0 Z0",
            "M5",
            "G4 S5",
        ];
        head.iter()
            .chain(&probe)
            .chain(policy_lines)
            .chain(&tail)
            .map(|l| format!("{l}\n"))
            .collect()
    }

    #[test]
    fn tool_change_sequences() {
        assert_eq!(
            tool_change(TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS, json!({}), "G21"),
            metric_sequence(&[])
        );
        assert_eq!(
            tool_change(TOOL_CHANGE_POLICY_SEND_M6_COMMANDS, json!({}), "G21"),
            metric_sequence(&[])
        );
        assert_eq!(
            tool_change(TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS, json!({}), "G21"),
            metric_sequence(&["G91 G38.2 F10 Z-1", "G10 L20 P1 Z0"])
        );
        assert_eq!(
            tool_change(TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO, json!({}), "G21"),
            metric_sequence(&["G91 G38.2 F10 Z-1", "G4 S1", "G43.1 Z0"])
        );
        assert_eq!(
            tool_change(
                TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING,
                json!({"toolProbeCustomCommands": "G4 P0.5\nG30 Z0.1"}),
                "G21"
            ),
            metric_sequence(&["G4 P0.5", "G30 Z0.1"])
        );
    }

    #[test]
    fn tool_change_converts_metric_config_under_g20() {
        let out = tool_change(
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS,
            json!({"toolChangeX": 25.4, "toolChangeY": 25.4, "toolChangeZ": 25.4, "toolProbeDistance": 25.4,
                   "toolProbeFeedrate": 254, "touchPlateHeight": 12.7}),
            "G20",
        );
        let expected = [
            "G4 S0.5",
            "M5",
            "G90",
            "G53 G0 Z1.0000",
            "G53 G0 X1.0000 Y1.0000",
            "G4 S0.5",
            "M0",
            "G53 G0 X0.0000 Y0.0000",
            "G53 G0 Z0.0000",
            "G4 S0.5",
            "G91 G38.2 F10 Z-1",
            "G10 L20 P1 Z0.5",
            "G53 G0 Z1.0000",
            "G53 G0 X1.0000 Y1.0000",
            "G4 S0.5",
            "M0",
            "G90",
            "G0 X0 Y0",
            "G0 Z0",
            "M5",
            "G4 S5",
        ];
        assert_eq!(out, expected.map(|l| format!("{l}\n")));
    }

    #[test]
    fn m6_in_a_program_with_manual_policy_pauses_and_feeds_the_tool_change() {
        let (mut ctl, _) = create(
            json!({"tool": {"toolChangePolicy": TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS}}),
        );
        cmd(&mut ctl, "gcode:load", &[json!("p"), json!("T2 M6\nG0 X1")]);
        cmd(&mut ctl, "gcode:start", &[]);
        assert_eq!(written(&ctl), ["T2 (M6)\n", "G4 S0.5\n"]);
        assert!(ctl.workflow.is_paused());
        assert!(ctl.feeder.size() > 0);
    }

    const GRID: &str = r#"{"startX": 0, "endX": 10, "stepX": 10, "startY": 0, "endY": 10, "stepY": 10,
        "clearanceZ": 5, "startZ": 2, "endZ": -1, "feedrate": 100}"#;

    #[test]
    fn autolevel_start() {
        let mut params: Value = serde_json::from_str(GRID).unwrap();
        let (mut ctl, _) = create(json!({}));
        params["mode"] = json!("test");
        cmd(&mut ctl, "autolevel:start", &[params.clone()]);
        assert_eq!(written(&ctl), ["G90\n"]);
        pump_feeder(&mut ctl);
        assert_eq!(
            written(&ctl),
            ["G90\n", "G0 Z5\n", "G0 Z2\n", "G38.2 Z-1 F100\n", "G0 Z5\n"]
        );
        assert!(ctl.probe.probe_points.is_empty());

        let (mut ctl, _) = create(json!({}));
        params.as_object_mut().unwrap().remove("mode");
        cmd(&mut ctl, "autolevel:start", &[params]);
        pump_feeder(&mut ctl);
        let mut expected = vec![];
        for (i, (x, y)) in [(0, 0), (10, 0), (0, 10), (10, 10)].into_iter().enumerate() {
            let f = if i == 0 { 50 } else { 100 };
            for l in [
                "G90".into(),
                "G0 Z5".into(),
                format!("G0 X{x} Y{y}"),
                "G0 Z2".into(),
                format!("G38.2 Z-1 F{f}"),
                "G0 Z5".into(),
            ] {
                expected.push(format!("{l}\n"));
            }
        }
        assert_eq!(written(&ctl), expected);
        assert_eq!(ctl.probe.probe_points.len(), 4);
    }

    #[test]
    fn probe_results_are_stored_at_the_grid_nodes() {
        let (mut ctl, mut frames) = create(json!({}));
        cmd(&mut ctl, "autolevel:start", &[serde_json::from_str(r#"{"startX": 0, "endX": 10, "stepX": 10, "startY": 0, "endY": 0, "stepY": 10, "clearanceZ": 5, "startZ": 2, "endZ": -1, "feedrate": 100}"#).unwrap()]);
        ctl.parse("<Idle|MPos:0.000,0.000,1.000|WPos:0.000,0.000,0.000>");
        events(&mut frames);
        // Quantised reports of the nodes (0,0) and (10,0); Z is in machine coordinates.
        ctl.parse("[PRB:0.001,0.001,-0.500:1]");
        ctl.parse("[PRB:10.001,-0.001,0.000:0]");
        ctl.parse("[PRB:10.001,-0.001,0.000:1]");
        let positions: Vec<_> = ctl
            .probe
            .probed_positions
            .iter()
            .map(|p| (p.x, p.y, p.z))
            .collect();
        assert_eq!(positions, [(0.0, 0.0, -1.5), (10.0, 0.0, -1.0)]);
        let names: Vec<String> = events(&mut frames)
            .into_iter()
            .map(|(e, _)| e)
            .filter(|e| e.starts_with("autolevel"))
            .collect();
        assert_eq!(
            names,
            ["autolevel:update", "autolevel:update", "autolevel:complete"]
        );

        cmd(&mut ctl, "autolevel:stop", &[]);
        assert_eq!(written(&ctl).last(), Some(&"\x18"));
        assert_eq!(ctl.probe, ProbeState::default());
    }

    #[test]
    fn timer_emits_state_changes_and_queries_the_machine() {
        let (mut ctl, mut frames) = create(json!({}));
        ctl.ready = true;
        ctl.tick();
        let names: Vec<String> = events(&mut frames).into_iter().map(|(e, _)| e).collect();
        assert_eq!(names, ["controller:settings", "controller:state"]);
        assert_eq!(written(&ctl), ["?", "$G\n"]);
        // Queries wait for their replies.
        ctl.tick();
        assert_eq!(written(&ctl).len(), 2);
        assert!(
            events(&mut frames).is_empty(),
            "unchanged state is not re-emitted"
        );
        // Replies to the timer's own queries are not forwarded to clients.
        ctl.parse("<Idle|MPos:1.0000,0.0000,0.0000|WPos:1.0000,0.0000,0.0000>");
        ctl.parse("[G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 F2540. S0.]");
        ctl.parse("ok");
        assert!(events(&mut frames).is_empty());
        ctl.tick();
        assert_eq!(written(&ctl)[2], "?");
        let (name, args) = events(&mut frames).remove(0);
        assert_eq!(
            (name.as_str(), &args[1]["status"]["mpos"]["x"]),
            ("controller:state", &json!("1.0000"))
        );
        // A client's own `?` gets the report.
        cmd(&mut ctl, "statusreport", &[]);
        events(&mut frames);
        ctl.parse("<Idle>");
        assert_eq!(
            events(&mut frames),
            [("serialport:read".to_string(), json!(["<Idle>"]))]
        );
    }

    #[test]
    fn add_socket_replays_state() {
        let (mut ctl, _) = create(json!({}));
        cmd(&mut ctl, "gcode:load", &[json!("p"), json!(PROGRAM)]);
        ctl.tick();
        let (socket, mut frames) = Socket::new();
        ctl.handle(ControllerMsg::AddSocket(socket));
        let names: Vec<String> = events(&mut frames).into_iter().map(|(e, _)| e).collect();
        assert_eq!(
            names,
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
    }
}

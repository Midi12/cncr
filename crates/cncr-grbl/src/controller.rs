//! The Grbl controller task state (port of `GrblController.js`).

use std::time::Duration;

use cncr_core::autolevel::{apply_probe_compensation_params, ProbeState};
use cncr_core::config::ConfigStore;
use cncr_core::controller::{reply, ControllerMsg, ControllerOptions, Reply, Services};
use cncr_core::feeder::Feeder;
use cncr_core::gcode::{in2mm, is_m0, is_m1, is_m6, replace_m6};
use cncr_core::hub::{Socket, Sockets};
use cncr_core::program::{
    populate_context, preprocess, sync_global, tool_change_policy, tool_change_program,
    tool_probe_custom_commands, Line,
};
use cncr_core::sender::{FilterResult, Sender, StreamingProtocol};
use cncr_core::serial::{SerialConnection, SerialEvent};
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
use tokio::sync::mpsc;
use tracing::{debug, error, info, warn};

use crate::constants::{
    GRBL_ACTIVE_STATE_HOLD, GRBL_ACTIVE_STATE_RUN, GRBL_ALARMS, GRBL_ERRORS,
    GRBL_REALTIME_COMMANDS, GRBL_SETTINGS,
};
use crate::runner::{Event, Runner};
use crate::TYPE;

/// Serial output, injectable for tests.
pub trait Port: Send {
    fn write(&mut self, data: &str);
}

impl Port for SerialConnection {
    fn write(&mut self, data: &str) {
        SerialConnection::write(self, data);
    }
}

/// Deferred work (JS `await delay(...)` / async callbacks) re-entering the task.
#[derive(Debug)]
pub enum Internal {
    /// 50 ms after `initController`.
    ControllerReady,
    /// 500 ms after a forced `gcode:stop`.
    ForceStop,
    WatchdirLoaded {
        file: String,
        result: Result<String, String>,
        reply: Option<Reply>,
    },
}

#[derive(Debug, Default)]
struct ActionMask {
    /// Waiting for the `[GC:...]` reply of a `$G` query.
    query_parser_state: bool,
    /// Waiting for the `ok` that follows it.
    query_parser_state_reply: bool,
    query_status_report: bool,
    /// The user typed `$G` / `?`: show the reply.
    reply_parser_state: bool,
    reply_status_report: bool,
}

#[derive(Debug, Default)]
struct ActionTime {
    query_parser_state: i64,
    query_status_report: i64,
    sender_finish: i64,
}

/// `ensureArray(commands).join('\n').split(/\r?\n/)` without blank lines.
fn split_lines(s: &str) -> impl Iterator<Item = String> + '_ {
    s.split('\n')
        .map(|l| l.strip_suffix('\r').unwrap_or(l))
        .filter(|l| !l.trim().is_empty())
        .map(str::to_string)
}

/// JS string conversion of a command argument (`${value}`).
fn js_string(v: Option<&Value>) -> String {
    match v {
        None => "undefined".into(),
        Some(Value::String(s)) => s.clone(),
        Some(Value::Number(n)) => js_num_to_string(n.as_f64().unwrap_or(f64::NAN)),
        Some(v) => v.to_string(),
    }
}

/// `commands` argument of the `gcode` command: a string or an array of lines.
fn gcode_lines(v: Option<&Value>) -> Vec<String> {
    let joined = match v {
        None | Some(Value::Null) => return vec![],
        Some(Value::Array(a)) => a
            .iter()
            .map(|l| match l {
                Value::Null => String::new(),
                l => js_string(Some(l)),
            })
            .collect::<Vec<_>>()
            .join("\n"),
        v => js_string(v),
    };
    split_lines(&joined).collect()
}

fn context_arg(v: Option<&Value>) -> Context {
    match v {
        Some(Value::Object(m)) => m.clone(),
        _ => Context::new(),
    }
}

/// M0 / M1 / M6 handling shared by the feeder and sender filters: the pause reasons
/// (`data`) in JS order, with M6 commented out unless the policy sends it, and
/// whether to run the manual tool change.
fn program_pauses(
    words: &[String],
    line: &mut String,
    config: &ConfigStore,
) -> (Vec<&'static str>, bool) {
    let mut pauses = Vec::new();
    if words.iter().any(|w| is_m0(w)) {
        pauses.push("M0");
    }
    if words.iter().any(|w| is_m1(w)) {
        pauses.push("M1");
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
        // TOOL_CHANGE_POLICY_SEND_M6_COMMANDS (and unknown policies) send M6 as is.
        if policy == TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS || manual {
            *line = replace_m6(line, |x| format!("({x})"));
            pauses.push("M6");
            tool_change = manual;
        }
    }
    (pauses, tool_change)
}

pub struct Controller {
    services: Services,
    options: ControllerOptions,
    sockets: Sockets,
    /// `None` once closed.
    port: Option<Box<dyn Port>>,
    internal_tx: mpsc::UnboundedSender<Internal>,
    pub(crate) runner: Runner,
    ready: bool,
    initialized: bool,
    /// Runner revisions last emitted (`None`: never, the JS `{}`).
    emitted_state_rev: Option<u64>,
    emitted_settings_rev: Option<u64>,
    /// Work position of the last emitted state (machine stopped moving when unchanged).
    emitted_wpos: Option<crate::runner::Pos>,
    mask: ActionMask,
    time: ActionTime,
    query_activity: Throttle,
    query_parser_state: Throttle,
    message_slot: MessageSlot,
    pub(crate) probe_state: ProbeState,
    pub(crate) feeder: Feeder,
    pub(crate) sender: Sender,
    pub(crate) workflow: Workflow,
    shared_context: Value,
}

impl Controller {
    pub fn new(
        services: Services,
        options: ControllerOptions,
        port: Option<Box<dyn Port>>,
        internal_tx: mpsc::UnboundedSender<Internal>,
    ) -> Self {
        Self {
            services,
            options,
            sockets: Sockets::default(),
            port,
            internal_tx,
            runner: Runner::default(),
            ready: false,
            initialized: false,
            emitted_state_rev: None,
            emitted_settings_rev: None,
            emitted_wpos: None,
            mask: ActionMask::default(),
            time: ActionTime::default(),
            // grbl-Mega needs a delay before the first status query (cncjs#889): no leading call.
            query_activity: Throttle::new(Duration::from_millis(2000), false),
            query_parser_state: Throttle::new(Duration::from_millis(500), true),
            message_slot: MessageSlot::default(),
            probe_state: ProbeState::default(),
            feeder: Feeder::new(),
            // Deduct 8 bytes from Grbl's 128 byte RX buffer to prevent overruns.
            sender: Sender::new(StreamingProtocol::char_counting(128 - 8)),
            workflow: Workflow::default(),
            shared_context: json!({}),
        }
    }

    /// Owns the controller until it closes.
    pub async fn run(
        mut self,
        mut rx: mpsc::UnboundedReceiver<ControllerMsg>,
        mut serial_rx: mpsc::UnboundedReceiver<SerialEvent>,
        mut internal_rx: mpsc::UnboundedReceiver<Internal>,
    ) {
        let period = Duration::from_millis(250);
        let mut timer = tokio::time::interval_at(tokio::time::Instant::now() + period, period);
        timer.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            tokio::select! {
                msg = rx.recv() => match msg {
                    Some(ControllerMsg::Close(done)) => {
                        self.close();
                        let _ = done.send(());
                        return;
                    }
                    Some(msg) => self.handle_msg(msg),
                    None => {
                        self.close();
                        return;
                    }
                },
                ev = serial_rx.recv() => match ev {
                    Some(SerialEvent::Line(line)) => self.on_line(&line),
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
                Some(msg) = internal_rx.recv() => self.handle_internal(msg),
                _ = timer.tick() => self.on_timer(),
            }
        }
    }

    fn handle_msg(&mut self, msg: ControllerMsg) {
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
                // Handled by `run`; reached only when driven directly.
                self.close();
                let _ = done.send(());
            }
        }
    }

    pub(crate) fn handle_internal(&mut self, msg: Internal) {
        match msg {
            Internal::ControllerReady => self.trigger("controller:ready"),
            Internal::ForceStop => {
                if self.runner.state.status.active_state == GRBL_ACTIVE_STATE_HOLD {
                    self.write("\x18", Context::new());
                }
            }
            Internal::WatchdirLoaded {
                file,
                result,
                reply: r,
            } => match result {
                Ok(data) => self.gcode_load(&file, &data, Context::new(), r),
                Err(e) => reply(r, Err(e)),
            },
        }
    }

    fn defer(&self, ms: u64, msg: Internal) {
        let tx = self.internal_tx.clone();
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(ms)).await;
            let _ = tx.send(msg);
        });
    }

    fn emit(&self, event: &str, args: &[Value]) {
        self.sockets.emit(event, args);
    }

    fn emit_read(&self, text: &str) {
        self.emit("serialport:read", &[json!(text)]);
    }

    fn emit_workflow_state(sockets: &Sockets, workflow: &Workflow) {
        sockets.emit("workflow:state", &[json!(workflow.state.as_str())]);
    }

    /// Fires the event triggers; G-code triggers go through the feeder.
    fn trigger(&mut self, event: &str) {
        for commands in self.services.trigger(event) {
            self.feed_gcode(split_lines(&commands).collect(), Context::new());
        }
    }

    fn port_info(&self, inuse: bool) -> Value {
        json!({ "port": self.options.port, "inuse": inuse })
    }

    fn settings_json(&self) -> Value {
        match self.emitted_settings_rev {
            Some(_) => serde_json::to_value(&self.runner.settings).unwrap_or_default(),
            None => json!({}),
        }
    }

    fn state_json(&self) -> Value {
        match self.emitted_state_rev {
            Some(_) => serde_json::to_value(&self.runner.state).unwrap_or_default(),
            None => json!({}),
        }
    }

    pub(crate) fn status(&self) -> Value {
        json!({
            "port": self.options.port,
            "baudrate": self.options.baudrate,
            "rtscts": self.options.rtscts,
            "sockets": self.sockets.ids().iter().map(u64::to_string).collect::<Vec<_>>(),
            "ready": self.ready,
            "controller": { "type": TYPE, "settings": self.settings_json(), "state": self.state_json() },
            "feeder": self.feeder.to_json(),
            "sender": self.sender.to_json(),
            "workflow": { "state": self.workflow.state.as_str() },
        })
    }

    fn add_socket(&mut self, socket: Socket) {
        debug!("Add socket connection: id={}", socket.id);
        if self.port.is_some() {
            socket.emit(
                "serialport:open",
                &[json!({ "port": self.options.port, "baudrate": self.options.baudrate, "controllerType": TYPE, "inuse": true })],
            );
        }
        if self.emitted_settings_rev.is_some() {
            socket.emit("controller:settings", &[json!(TYPE), self.settings_json()]);
        }
        if self.emitted_state_rev.is_some() {
            socket.emit("controller:state", &[json!(TYPE), self.state_json()]);
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

    /// JS `close()`: the task ends right after.
    pub(crate) fn close(&mut self) {
        self.ready = false;
        self.initialized = false;
        let info = self.port_info(false);
        self.emit("serialport:close", std::slice::from_ref(&info));
        self.services.hub.broadcast("serialport:change", &[info]);
        self.port = None;
        self.services.registry.remove(&self.options.port);
    }

    pub(crate) fn clear_action_values(&mut self) {
        self.mask = ActionMask::default();
        self.time = ActionTime::default();
    }

    fn init_controller(&mut self) {
        // $13=0 reports in mm, $13=1 in inches (cncjs#206).
        self.writeln("$$", Context::new());
        self.defer(50, Internal::ControllerReady);
    }

    // --- Workflow transitions and their listeners ---

    pub(crate) fn workflow_start(&mut self) {
        if self.workflow.start() {
            Self::emit_workflow_state(&self.sockets, &self.workflow);
            self.sender.rewind();
        }
    }

    pub(crate) fn workflow_stop(&mut self) {
        if self.workflow.stop() {
            Self::emit_workflow_state(&self.sockets, &self.workflow);
            self.sender.rewind();
        }
    }

    fn workflow_pause(&mut self, reason: Option<Value>) {
        if self.workflow.pause() {
            Self::emit_workflow_state(&self.sockets, &self.workflow);
            self.sender.hold(reason);
        }
    }

    fn workflow_resume(&mut self) {
        if self.workflow.resume() {
            Self::emit_workflow_state(&self.sockets, &self.workflow);
            // Reset the feeder prior to resuming program execution.
            self.feeder.reset();
            self.sender.unhold();
            self.sender_next();
        }
    }

    // --- Serial output ---

    /// Every serial write goes through here (the JS connection `writeFilter`).
    fn port_write(&mut self, data: &str) {
        // `$13=<n>` switches position reports to inches; record it right away.
        if let Some(v) = data.trim().strip_prefix("$13=") {
            if !v.is_empty() && v.bytes().all(|b| b.is_ascii_digit() || b == b'.') {
                let n = js_str_to_num(v);
                if (0.0..=65535.0).contains(&n) {
                    self.runner.set_report_inches(n != 0.0);
                }
            }
        }
        if let Some(port) = self.port.as_mut() {
            port.write(data);
        }
    }

    pub(crate) fn write(&mut self, data: &str, mut context: Context) {
        if self.port.is_none() {
            error!("Serial port {:?} is not accessible", self.options.port);
            return;
        }
        let cmd = data.trim();
        self.mask.reply_status_report |= cmd == "?";
        self.mask.reply_parser_state |= cmd == "$G";
        context.insert("source".into(), json!(WRITE_SOURCE_CLIENT));
        self.emit("serialport:write", &[json!(data), Value::Object(context)]);
        self.port_write(data);
    }

    pub(crate) fn writeln(&mut self, data: &str, context: Context) {
        // Realtime commands are single bytes and take no newline.
        let realtime = GRBL_REALTIME_COMMANDS.contains(&data)
            || data.chars().any(|c| ('\u{80}'..='\u{ff}').contains(&c));
        if realtime {
            self.write(data, context);
        } else {
            self.write(&format!("{data}\n"), context);
        }
    }

    // --- Feeder ---

    fn feed_gcode(&mut self, lines: Vec<String>, context: Context) {
        self.feeder.feed(lines, context);
        if !self.feeder.is_pending() {
            self.feeder_next();
        }
    }

    fn feeder_next(&mut self) {
        let mut tool_change = false;
        let Self {
            feeder,
            sender,
            runner,
            services,
            message_slot,
            shared_context,
            ..
        } = self;
        let out = feeder.next(&mut |raw, ctx| {
            populate_context(ctx, &runner.snapshot(), shared_context);
            let result = match preprocess(raw, ctx) {
                Line::Msg(msg) => {
                    message_slot.put(json!(msg));
                    FilterResult::skip()
                }
                Line::Wait(dwell) => {
                    // As in cncjs, `%wait` from the feeder holds the sender.
                    sender.hold(Some(
                        json!({ "data": "%wait", "msg": message_slot.take_or(json!(raw)) }),
                    ));
                    FilterResult::line(dwell)
                }
                Line::Assigned => FilterResult::skip(),
                Line::Gcode { mut line, words } => {
                    let (pauses, tc) = program_pauses(&words, &mut line, &services.config);
                    let mut result = FilterResult::line(line);
                    for data in pauses {
                        let reason =
                            json!({ "data": data, "msg": message_slot.take_or(json!(raw)) });
                        result.hold.get_or_insert(reason);
                    }
                    tool_change |= tc;
                    result
                }
            };
            sync_global(ctx, shared_context);
            result
        });
        if let Some((line, context)) = out {
            self.feeder_write(&line, context);
        }
        if tool_change {
            self.tool_change();
        }
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
        self.port_write(&data);
    }

    // --- Sender ---

    pub(crate) fn sender_next(&mut self) {
        let mut trigger_gcode: Vec<String> = Vec::new();
        let mut tool_change = false;
        let Self {
            sender,
            runner,
            services,
            message_slot,
            workflow,
            sockets,
            shared_context,
            ..
        } = self;
        let out = sender.next(&mut |raw, ctx| {
            populate_context(ctx, &runner.snapshot(), shared_context);
            let result = match preprocess(raw, ctx) {
                Line::Msg(msg) => {
                    message_slot.put(json!(msg));
                    FilterResult::skip()
                }
                Line::Wait(dwell) => FilterResult {
                    line: dwell,
                    hold: Some(json!({ "data": "%wait", "msg": message_slot.take_or(json!(raw)) })),
                },
                Line::Assigned => FilterResult::skip(),
                Line::Gcode { mut line, words } => {
                    let (pauses, tc) = program_pauses(&words, &mut line, &services.config);
                    let mut result = FilterResult::line(line);
                    for data in pauses {
                        trigger_gcode.extend(services.trigger("gcode:pause"));
                        let reason =
                            json!({ "data": data, "msg": message_slot.take_or(json!(raw)) });
                        if workflow.pause() {
                            Self::emit_workflow_state(sockets, workflow);
                            result.hold.get_or_insert(reason);
                        }
                    }
                    tool_change |= tc;
                    result
                }
            };
            sync_global(ctx, shared_context);
            result
        });
        if out.started.is_some() {
            self.time.sender_finish = 0;
        }
        if let Some(t) = out.ended {
            self.time.sender_finish = t;
        }
        // Side effects of a pausing line run before that line is written, as in JS.
        let mut lines = out.lines.into_iter();
        let before_last = lines.len().saturating_sub(1);
        for line in lines.by_ref().take(before_last) {
            self.sender_write(line);
        }
        for commands in trigger_gcode {
            self.feed_gcode(split_lines(&commands).collect(), Context::new());
        }
        if tool_change {
            self.tool_change();
        }
        if let Some(line) = lines.next() {
            self.sender_write(line);
        }
    }

    fn sender_write(&mut self, mut line: String) {
        if self.port.is_none() {
            error!("Serial port {:?} is not accessible", self.options.port);
            return;
        }
        if self.workflow.is_idle() {
            error!(
                "Unexpected workflow state: {}",
                self.workflow.state.as_str()
            );
            return;
        }
        if line.is_empty() {
            warn!("Expected non-empty line: N={}", self.sender.state.sent);
            return;
        }
        line.push('\n');
        self.port_write(&line);
    }

    // --- Received lines ---

    pub(crate) fn on_line(&mut self, line: &str) {
        let Some(event) = self.runner.parse(line) else {
            return;
        };
        let raw = line.trim_end();
        match event {
            Event::Status { rx } => self.on_status(raw, rx),
            Event::Ok => self.on_ok(raw),
            Event::Error(message) => self.on_error(raw, &message),
            Event::Alarm(message) => {
                let code = js_str_to_num(&message);
                match GRBL_ALARMS.iter().find(|(c, _)| f64::from(*c) == code) {
                    Some((c, m)) => self.emit_read(&format!("ALARM:{c} ({m})")),
                    None => self.emit_read(raw),
                }
            }
            Event::ParserState => {
                self.mask.query_parser_state = false;
                self.mask.query_parser_state_reply = true;
                if self.mask.reply_parser_state {
                    self.emit_read(raw);
                }
            }
            Event::Parameters { name, value } => {
                self.emit_read(raw);
                if name == "PRB" {
                    self.on_probe(&value);
                }
            }
            Event::Settings {
                name,
                value,
                message,
            } => match GRBL_SETTINGS.iter().find(|(s, _, _)| *s == name) {
                Some((_, m, units)) if message.is_empty() => {
                    self.emit_read(&format!("{name}={value} ({m}, {units})"))
                }
                _ => self.emit_read(raw),
            },
            Event::Startup { .. } => {
                self.emit_read(raw);
                // Printed at power-up, after a reset and at program end.
                if !self.ready {
                    self.clear_action_values();
                    self.ready = true;
                }
                if !self.initialized {
                    self.initialized = true;
                    self.init_controller();
                }
            }
            Event::Feedback(_) | Event::Others => self.emit_read(raw),
        }
    }

    fn on_status(&mut self, raw: &str, rx: f64) {
        // A status reply also means the controller is up (no startup message over UART).
        if !self.ready {
            self.ready = true;
            self.clear_action_values();
        }
        if !self.initialized {
            self.initialized = true;
            self.init_controller();
        }
        self.mask.query_status_report = false;
        if self.mask.reply_status_report {
            self.mask.reply_status_report = false;
            self.emit_read(raw);
        }
        // Grow the buffer to the reported RX buffer size while idle (cncjs#115, #133).
        if rx > 0.0 && self.workflow.is_idle() && self.sender.sp.data_length() == 0 {
            let size = rx - 8.0;
            if size > self.sender.sp.buffer_size().unwrap_or(0) as f64 {
                self.sender.sp.set_buffer_size(size as usize);
            }
        }
    }

    fn on_ok(&mut self, raw: &str) {
        if self.mask.query_parser_state_reply {
            if self.mask.reply_parser_state {
                self.mask.reply_parser_state = false;
                self.emit_read(raw);
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
        self.emit_read(raw);
        self.feeder_next();
    }

    fn on_error(&mut self, raw: &str, message: &str) {
        let code = js_str_to_num(message);
        // Grbl v1.1 reports codes, v0.9 messages.
        let text = GRBL_ERRORS
            .iter()
            .find(|(c, _)| f64::from(*c) == code)
            .map(|(c, m)| format!("error:{c} ({m})"));
        if self.workflow.is_running() {
            let ignore = matches!(
                self.services
                    .config
                    .get("state.controller.exception.ignoreErrors"),
                Some(Value::Bool(true))
            );
            let received = self.sender.state.received;
            let line = received
                .checked_sub(1)
                .and_then(|i| self.sender.state.lines.get(i))
                .map_or("", |l| l.trim());
            self.emit_read(&format!("> {line} (ln={})", received + 1));
            let msg = text.unwrap_or_else(|| raw.to_string());
            self.emit_read(&msg);
            if !ignore {
                self.workflow_pause(Some(json!({ "err": true, "msg": msg })));
            }
            self.sender.ack();
            self.sender_next();
            return;
        }
        self.emit_read(text.as_deref().unwrap_or(raw));
        self.feeder_next();
    }

    /// `[PRB:x,y,z:result]`: record the probed height for auto-leveling.
    fn on_probe(&mut self, value: &Value) {
        if value.get("result").and_then(Value::as_f64) != Some(1.0) {
            return;
        }
        let s = &self.runner.state.status;
        // PRB is in machine coordinates: WCO = MPos - WPos.
        let wco_z = round_to(s.mpos.num(2) - s.wpos.num(2), 3);
        let z = value
            .get("z")
            .and_then(Value::as_str)
            .map_or(f64::NAN, js_str_to_num);
        let z = if z.is_finite() { z } else { 0.0 } - wco_z;
        // $13=1 reports positions (PRB included) in inches; probe data is kept in mm.
        let report_inches = self
            .runner
            .settings
            .settings
            .get("$13")
            .and_then(Value::as_str)
            == Some("1");
        let z = if report_inches { in2mm(z) } else { z };
        // Probe points are in the units of the emitted probe G-code (modal state).
        let imperial = self.runner.modal_units() == "G20";
        if let Some((update, completed)) = self.probe_state.record(z, imperial) {
            self.emit("autolevel:update", &[update]);
            if completed {
                self.emit("autolevel:complete", &[]);
            }
        }
    }

    // --- 250 ms timer (JS queryTimer) ---

    pub(crate) fn on_timer(&mut self) {
        if self.port.is_none() {
            return;
        }
        if self.feeder.peek() {
            self.emit("feeder:status", &[self.feeder.to_json()]);
        }
        if self.sender.peek() {
            self.emit("sender:status", &[self.sender.to_json()]);
        }
        let zero_offset = self.emitted_wpos.as_ref() == Some(&self.runner.state.status.wpos);
        if self.emitted_settings_rev != Some(self.runner.settings_rev) {
            self.emitted_settings_rev = Some(self.runner.settings_rev);
            self.emit("controller:settings", &[json!(TYPE), self.settings_json()]);
        }
        if self.emitted_state_rev != Some(self.runner.state_rev) {
            self.emitted_state_rev = Some(self.runner.state_rev);
            self.emitted_wpos = Some(self.runner.state.status.wpos.clone());
            self.emit("controller:state", &[json!(TYPE), self.state_json()]);
        }

        if !self.ready {
            if self.query_activity.ready() {
                self.port_write("?");
            }
            return;
        }
        self.query_status_report();
        if self.query_parser_state.ready() {
            self.query_parser_state();
        }

        // Stop the workflow once the machine has stopped after the last line.
        if self.time.sender_finish > 0 {
            let machine_idle = zero_offset && self.runner.is_idle();
            let now = now_ms();
            let timespan = (now - self.time.sender_finish).abs();
            if !machine_idle {
                self.time.sender_finish = now;
            } else if timespan > 500 {
                debug!("Finished sending G-code: timespan={timespan}");
                self.time.sender_finish = 0;
                self.gcode_stop(false);
            }
        }
    }

    fn query_status_report(&mut self) {
        let now = now_ms();
        // `?` is a realtime command: it does not consume the RX buffer.
        let last = self.time.query_status_report;
        if last > 0 && (now - last).abs() >= 5000 {
            debug!(
                "Continue status report query: timespan={}ms",
                (now - last).abs()
            );
            self.mask.query_status_report = false;
        }
        if self.mask.query_status_report {
            return;
        }
        self.mask.query_status_report = true;
        self.time.query_status_report = now;
        self.port_write("?");
    }

    fn query_parser_state(&mut self) {
        let now = now_ms();
        // Only force `$G` when idle: it takes 3 bytes of RX buffer each time (cncjs#176, #186).
        if self.workflow.is_idle() && self.runner.is_idle() {
            let last = self.time.query_parser_state;
            if last > 0 && (now - last).abs() >= 10000 {
                debug!(
                    "Continue parser state query: timespan={}ms",
                    (now - last).abs()
                );
                self.mask.query_parser_state = false;
                self.mask.query_parser_state_reply = false;
            }
        }
        if self.mask.query_parser_state || self.mask.query_parser_state_reply {
            return;
        }
        self.mask.query_parser_state = true;
        self.mask.query_parser_state_reply = false;
        self.time.query_parser_state = now;
        self.port_write("$G\n");
    }

    // --- Commands ---

    fn gcode_load(&mut self, name: &str, gcode: &str, context: Context, r: Option<Reply>) {
        // A trailing dwell empties the planner: its `ok` marks the end of all motion.
        let program = format!("{gcode}\n%wait ; Wait for the planner to empty");
        if !self.sender.load(name, &program, context) {
            reply(r, Err(format!("Invalid G-code: name={name}")));
            return;
        }
        let s = &self.sender.state;
        self.emit(
            "gcode:load",
            &[
                json!(name),
                json!(s.gcode),
                Value::Object(s.context.clone()),
            ],
        );
        self.trigger("gcode:load");
        debug!(
            "Load G-code: name={name:?}, size={}, total={}",
            self.sender.state.gcode.len(),
            self.sender.state.total
        );
        self.workflow_stop();
        reply(r, Ok(self.sender.to_json()));
    }

    fn gcode_stop(&mut self, force: bool) {
        self.trigger("gcode:stop");
        self.workflow_stop();
        if force {
            if self.runner.state.status.active_state == GRBL_ACTIVE_STATE_RUN {
                self.write("!", Context::new());
            }
            self.defer(500, Internal::ForceStop);
        }
    }

    fn find_macro(&self, id: Option<&Value>) -> Option<Value> {
        let id = id?;
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
        let config = &self.services.config;
        let probe: Vec<String> = match tool_change_policy(config) {
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS => vec![
                "G91 [tool_probe_command] F[tool_probe_feedrate] Z[tool_probe_z - mposz - tool_probe_distance]".into(),
                "G10 L20 P[mapWCSToPValue(modal.wcs)] Z[touch_plate_height]".into(),
            ],
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO => vec![
                "G91 [tool_probe_command] F[tool_probe_feedrate] Z[tool_probe_z - mposz - tool_probe_distance]".into(),
                "%wait 1".into(),
                "G43.1 Z[posz - touch_plate_height]".into(),
            ],
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING => tool_probe_custom_commands(config),
            _ => vec![],
        };
        let (lines, context) = tool_change_program(config, self.runner.modal_units(), &probe);
        self.feed_gcode(split_lines(&lines.join("\n")).collect(), context);
    }

    pub(crate) fn command(&mut self, cmd: &str, args: &[Value], r: Option<Reply>) {
        let arg = |i: usize| args.get(i);
        let num = |i: usize| arg(i).and_then(Value::as_f64);
        match cmd {
            "gcode:load" => self.gcode_load(
                &js_string(arg(0)),
                &js_string(arg(1)),
                context_arg(arg(2)),
                r,
            ),
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
                let force = matches!(arg(0).and_then(|o| o.get("force")), Some(Value::Bool(true)));
                self.gcode_stop(force);
            }
            "gcode:pause" => {
                self.trigger("gcode:pause");
                self.workflow_pause(None);
                self.write("!", Context::new());
            }
            "gcode:resume" => {
                self.trigger("gcode:resume");
                self.write("~", Context::new());
                self.workflow_resume();
            }
            "feeder:feed" | "gcode" => self.feed_gcode(gcode_lines(arg(0)), context_arg(arg(1))),
            "feeder:start" => {
                if self.workflow.is_running() {
                    return;
                }
                self.write("~", Context::new());
                self.feeder.unhold();
                self.feeder_next();
            }
            "feeder:stop" => self.feeder.reset(),
            "feedhold" => {
                self.trigger("feedhold");
                self.write("!", Context::new());
            }
            "cyclestart" => {
                self.trigger("cyclestart");
                self.write("~", Context::new());
            }
            "statusreport" => self.write("?", Context::new()),
            "homing" => {
                self.trigger("homing");
                self.writeln("$H", Context::new());
            }
            "sleep" => {
                self.trigger("sleep");
                self.writeln("$SLP", Context::new());
            }
            "unlock" => self.writeln("$X", Context::new()),
            "reset" => {
                self.workflow_stop();
                self.feeder.reset();
                self.write("\x18", Context::new());
            }
            // https://github.com/gnea/grbl/blob/master/doc/markdown/jogging.md
            "jogCancel" => self.write("\u{85}", Context::new()),
            "feedOverride" | "spindleOverride" | "rapidOverride" => {
                let byte = match (cmd, num(0)) {
                    ("feedOverride", Some(v)) => [
                        (0.0, "\u{90}"),
                        (10.0, "\u{91}"),
                        (-10.0, "\u{92}"),
                        (1.0, "\u{93}"),
                        (-1.0, "\u{94}"),
                    ]
                    .iter()
                    .find(|(k, _)| *k == v)
                    .map(|(_, b)| *b),
                    ("spindleOverride", Some(v)) => [
                        (0.0, "\u{99}"),
                        (10.0, "\u{9a}"),
                        (-10.0, "\u{9b}"),
                        (1.0, "\u{9c}"),
                        (-1.0, "\u{9d}"),
                    ]
                    .iter()
                    .find(|(k, _)| *k == v)
                    .map(|(_, b)| *b),
                    ("rapidOverride", Some(v)) => [
                        (0.0, "\u{95}"),
                        (100.0, "\u{95}"),
                        (50.0, "\u{96}"),
                        (25.0, "\u{97}"),
                    ]
                    .iter()
                    .find(|(k, _)| *k == v)
                    .map(|(_, b)| *b),
                    _ => None,
                };
                if let Some(b) = byte {
                    self.write(b, Context::new());
                }
            }
            "lasertest:on" => {
                let n = |i: usize, default: f64| arg(i).map_or(default, |v| num_or_zero(Some(v)));
                let (power, duration, max_s) = (n(0, 0.0), n(1, 0.0), n(2, 1000.0));
                let positive = |v: f64| if v > 0.0 { v } else { 0.0 };
                // The laser only turns on in a G1/G2/G3 motion mode (Grbl v1.1 laser mode).
                let mut commands = vec![
                    "G1F1".to_string(),
                    format!("M3S{}", js_num_to_string(positive(max_s * (power / 100.0)))),
                ];
                if duration > 0.0 {
                    commands.push(format!(
                        "G4P{}",
                        js_num_to_string(positive(duration / 1000.0))
                    ));
                    commands.push("M5S0".into());
                }
                self.feed_gcode(commands, Context::new());
            }
            "lasertest:off" => self.feed_gcode(vec!["M5S0".into()], Context::new()),
            "macro:run" => {
                let Some(m) = self.find_macro(arg(0)) else {
                    return;
                };
                self.trigger("macro:run");
                self.feed_gcode(gcode_lines(m.get("content")), context_arg(arg(1)));
                reply(r, Ok(Value::Null));
            }
            "macro:load" => {
                let Some(m) = self.find_macro(arg(0)) else {
                    return;
                };
                self.trigger("macro:load");
                self.gcode_load(
                    &js_string(m.get("name")),
                    &js_string(m.get("content")),
                    context_arg(arg(1)),
                    r,
                );
            }
            "watchdir:load" => {
                let file = js_string(arg(0));
                let monitor = self.services.monitor.clone();
                let tx = self.internal_tx.clone();
                tokio::task::spawn_blocking(move || {
                    let result = monitor.read_file(&file).map_err(|e| e.to_string());
                    let _ = tx.send(Internal::WatchdirLoaded {
                        file,
                        result,
                        reply: r,
                    });
                });
            }
            "tool:change" => self.tool_change(),
            "autolevel:start" => {
                let params = arg(0).cloned().unwrap_or_else(|| json!({}));
                let gcode = self.probe_state.start(&params);
                self.feed_gcode(gcode, Context::new());
            }
            "autolevel:stop" => {
                // Reset the machine to cancel the probe cycle immediately.
                self.command("reset", &[], None);
                self.probe_state.clear();
                info!("[autolevel:stop] Probe stopped and state cleared");
            }
            "autolevel:getProbeState" => {
                reply(r, Ok(json!({ "state": self.probe_state.to_json() })))
            }
            "autolevel:loadFromFile" => {
                let result = self.probe_state.load_from_file(&js_string(arg(0)));
                if let Err(e) = &result {
                    error!("[autolevel:load] Error loading probe data: {e}");
                }
                reply(r, result);
            }
            "autolevel:saveToFile" => {
                let result = self.probe_state.save_to_file(&js_string(arg(0)));
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
mod tests;

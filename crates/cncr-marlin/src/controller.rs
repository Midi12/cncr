//! Marlin controller logic (port of `MarlinController`). One [`Controller`] owns all
//! of a port's state and is driven by the controller task.

use std::sync::{Arc, LazyLock};

use cncr_core::autolevel::{apply_probe_compensation_params, ProbeState};
use cncr_core::config::ConfigStore;
use cncr_core::controller::{reply, ControllerMsg, ControllerOptions, Reply, Services};
use cncr_core::feeder::Feeder;
use cncr_core::gcode::{in2mm, is_m0, is_m1, is_m109, is_m190, is_m6, replace_m6};
use cncr_core::hub::{Socket, Sockets};
use cncr_core::program::{
    populate_context, preprocess, sync_global, tool_change_policy, tool_change_program,
    tool_probe_custom_commands, Line, MachineSnapshot,
};
use cncr_core::sender::{FilterResult, Sender, StreamingProtocol};
use cncr_core::util::{
    js_num_to_string, js_str_to_num, now_ms, num_or_zero, MessageSlot, Throttle,
};
use cncr_core::workflow::Workflow;
use cncr_core::{
    json, Context, Value, TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS,
    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING,
    TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO, TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS,
    WRITE_SOURCE_CLIENT, WRITE_SOURCE_FEEDER, WRITE_SOURCE_SENDER, WRITE_SOURCE_SERVER,
};
use regex::Regex;
use tokio::sync::mpsc;
use tracing::{debug, error, info, trace, warn};

use crate::runner::{Response, Runner, Settings, State};
use crate::TYPE;

/// Where written data goes: the serial connection, or a recorder in tests.
pub trait Port: Send {
    fn write(&mut self, data: &str);
}

/// Results of work done off the controller task.
#[derive(Debug)]
pub enum Internal {
    WatchdirLoaded {
        file: String,
        result: Result<String, String>,
        reply: Option<Reply>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Query {
    /// M114: current position.
    Position,
    /// M105: temperatures.
    Temperature,
}

/// Commands the JS ran re-entrantly from inside the sender's data filter.
enum Deferred {
    Gcode(String),
    ToolChange,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum M6Action {
    /// Comment the M6 out and pause.
    Ignore,
    /// Comment the M6 out, pause and run the manual tool change program.
    Manual,
    /// Send M6 as is.
    Pass,
}

fn m6_action(config: &ConfigStore) -> M6Action {
    match tool_change_policy(config) {
        TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS => M6Action::Ignore,
        TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS
        | TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO
        | TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING => M6Action::Manual,
        _ => M6Action::Pass,
    }
}

static PROBE_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"(?i)G38\.[2-5]").unwrap());

/// Pause/hold reason `{data, msg}`; `msg` is the pending `%msg` or the line itself.
fn hold_reason(data: &str, slot: &mut MessageSlot, raw: &str) -> Value {
    json!({ "data": data, "msg": slot.take_or(json!(raw)) })
}

/// The front half shared by the feeder and sender filters.
enum Pre {
    Skip,
    /// Dwell line and the `%wait` hold reason.
    Wait(String, Value),
    /// Stripped line and its flattened words.
    Gcode(String, Vec<String>),
}

fn prepare(
    raw: &str,
    ctx: &mut Context,
    snap: &MachineSnapshot,
    pose: f64,
    global: &mut Value,
    slot: &mut MessageSlot,
) -> Pre {
    populate_context(ctx, snap, global);
    ctx.insert("pose".into(), json!(pose));
    let line = preprocess(raw, ctx);
    sync_global(ctx, global);
    match line {
        Line::Msg(msg) => {
            slot.put(Value::String(msg));
            Pre::Skip
        }
        Line::Assigned => Pre::Skip,
        // Marlin dwells in seconds with S (P is milliseconds).
        Line::Wait(dwell) => Pre::Wait(
            dwell.replacen("G4 P", "G4 S", 1),
            hold_reason("%wait", slot, raw),
        ),
        Line::Gcode { line, words } => Pre::Gcode(line, words),
    }
}

fn has(words: &[String], is: fn(&str) -> bool) -> bool {
    words.iter().any(|w| is(w))
}

/// `ensureArray(commands)` items as strings (non-strings become empty lines).
fn js_lines(v: Option<&Value>) -> Vec<String> {
    let item = |v: &Value| match v {
        Value::String(s) => s.clone(),
        Value::Number(n) => n.to_string(),
        Value::Bool(b) => b.to_string(),
        _ => String::new(),
    };
    match v {
        None | Some(Value::Null) => vec![],
        Some(Value::Array(a)) => a.iter().map(item).collect(),
        Some(v) => vec![item(v)],
    }
}

fn obj_arg(v: Option<&Value>) -> Context {
    v.and_then(Value::as_object).cloned().unwrap_or_default()
}

fn take_str(args: &mut [Value], i: usize) -> String {
    match args.get_mut(i).map(std::mem::take) {
        Some(Value::String(s)) => s,
        Some(Value::Number(n)) => n.to_string(),
        _ => String::new(),
    }
}

/// Numeric argument with a JS default parameter value for a missing argument.
fn num_arg(args: &[Value], i: usize, default: f64) -> f64 {
    args.get(i).map_or(default, |v| num_or_zero(Some(v)))
}

/// `ensurePositiveNumber`.
fn positive(n: f64) -> f64 {
    if n.is_nan() {
        0.0
    } else {
        n.max(0.0)
    }
}

/// Feed/spindle override step: 0 resets to 100%, the result stays within 10..=500.
fn override_value(current: f64, value: f64) -> f64 {
    if value == 0.0 {
        100.0
    } else {
        (current + value).clamp(10.0, 500.0)
    }
}

pub struct Controller {
    services: Services,
    options: ControllerOptions,
    /// `None` once closed.
    port: Option<Box<dyn Port>>,
    internal: mpsc::UnboundedSender<Internal>,
    sockets: Sockets,
    ready: bool,
    pub(crate) runner: Runner,
    /// Last emitted state / settings (`None` before the first emission).
    state: Option<State>,
    settings: Option<Settings>,
    /// Source and trimmed text of the last write (JS `history`).
    pub(crate) write_source: Option<&'static str>,
    pub(crate) write_line: String,
    message_slot: MessageSlot,
    pub(crate) feeder: Feeder,
    pub(crate) sender: Sender,
    pub(crate) workflow: Workflow,
    sender_finish_time: i64,
    shared_context: Value,
    pub(crate) probe: ProbeState,
    /// Waiting for the M114 report that follows a completed G38.x probe.
    pub(crate) pending_probe_capture: bool,
    query: Option<Query>,
    last_query_time: i64,
    position_throttle: Throttle,
    position_query_time: i64,
    temperature_throttle: Throttle,
    temperature_query_time: i64,
    /// Sender line written right after the deferred filter commands run (see `sender_next`).
    sender_in_flight: usize,
}

impl Controller {
    pub fn new(
        services: Services,
        options: ControllerOptions,
        port: Box<dyn Port>,
        internal: mpsc::UnboundedSender<Internal>,
    ) -> Self {
        Self {
            services,
            options,
            port: Some(port),
            internal,
            sockets: Sockets::default(),
            ready: false,
            runner: Runner::default(),
            state: None,
            settings: None,
            write_source: None,
            write_line: String::new(),
            message_slot: MessageSlot::default(),
            feeder: Feeder::new(),
            sender: Sender::new(StreamingProtocol::SendResponse),
            workflow: Workflow::default(),
            sender_finish_time: 0,
            shared_context: json!({}),
            probe: ProbeState::default(),
            pending_probe_capture: false,
            query: None,
            last_query_time: 0,
            position_throttle: Throttle::new(std::time::Duration::from_millis(500), true),
            position_query_time: 0,
            temperature_throttle: Throttle::new(std::time::Duration::from_millis(1000), true),
            temperature_query_time: 0,
            sender_in_flight: 0,
        }
    }

    pub fn is_closed(&self) -> bool {
        self.port.is_none()
    }

    /// Work done right after the port opened.
    pub fn post_open(&mut self) {
        // M115: the firmware report takes us to the ready state.
        self.conn_write("M115\n", WRITE_SOURCE_SERVER);
        self.workflow_stop();
    }

    fn emit(&self, event: &str, args: &[Value]) {
        self.sockets.emit(event, args);
    }

    fn emit_read(&self, raw: &str) {
        self.emit("serialport:read", &[json!(raw)]);
    }

    fn emit_workflow_state(&self) {
        self.emit("workflow:state", &[json!(self.workflow.state.as_str())]);
    }

    fn source_is_client_or_feeder(&self) -> bool {
        matches!(
            self.write_source,
            Some(WRITE_SOURCE_CLIENT | WRITE_SOURCE_FEEDER)
        )
    }

    pub fn handle(&mut self, msg: ControllerMsg) {
        match msg {
            ControllerMsg::AddSocket(socket) => self.add_socket(socket),
            ControllerMsg::RemoveSocket(id) => self.sockets.remove(id),
            ControllerMsg::Command { cmd, args, reply } => self.command(&cmd, args, reply),
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
            }
        }
    }

    pub fn internal(&mut self, msg: Internal) {
        match msg {
            Internal::WatchdirLoaded {
                file,
                result: Ok(data),
                reply,
            } => self.gcode_load(file, data, Context::new(), reply),
            Internal::WatchdirLoaded {
                result: Err(e),
                reply: r,
                ..
            } => reply(r, Err(e)),
        }
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
        debug!("Add socket connection: id={}", socket.id);
        if !self.is_closed() {
            let o = &self.options;
            socket.emit(
                "serialport:open",
                &[json!({ "port": o.port, "baudrate": o.baudrate, "controllerType": TYPE, "inuse": true })],
            );
        }
        if let Some(settings) = self
            .settings
            .as_ref()
            .filter(|s| **s != Settings::default())
        {
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

    /// JS `close()`: also deregisters the controller; the task stops afterwards.
    pub fn close(&mut self) {
        let port = &self.options.port;
        self.ready = false;
        self.emit(
            "serialport:close",
            &[json!({ "port": port, "inuse": false })],
        );
        self.services.hub.broadcast(
            "serialport:change",
            &[json!({ "port": port, "inuse": false })],
        );
        self.services.registry.remove(port);
        self.port = None;
    }

    /// `connection.write` through the JS write filter: records the write history and
    /// tracks the modal state of the written line.
    fn conn_write(&mut self, data: &str, source: &'static str) {
        let Some(port) = &mut self.port else { return };
        let line = data.trim();
        self.write_source = Some(source);
        self.write_line.clear();
        self.write_line.push_str(line);
        if !line.is_empty() {
            self.runner.interpret(line);
        }
        port.write(data);
        trace!("> {line}");
    }

    pub fn write(&mut self, data: &str, mut context: Context) {
        if self.is_closed() {
            error!("Serial port {:?} is not accessible", self.options.port);
            return;
        }
        context.insert("source".into(), json!(WRITE_SOURCE_CLIENT));
        self.emit("serialport:write", &[json!(data), Value::Object(context)]);
        self.conn_write(data, WRITE_SOURCE_CLIENT);
    }

    pub fn writeln(&mut self, data: &str, context: Context) {
        self.write(&format!("{data}\n"), context);
    }

    fn clear_history(&mut self) {
        self.write_source = None;
        self.write_line.clear();
    }

    fn trigger(&mut self, event: &str) {
        for commands in self.services.trigger(event) {
            self.feed_gcode(&[commands], Context::new());
        }
    }

    // Workflow transitions with the JS listener side effects.

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
            // Reset the feeder prior to resuming program execution.
            self.feeder.reset();
            self.sender.unhold();
            self.sender_next();
        }
    }

    /// Values exposed to macros: work position (also used as machine position) and pose.
    fn snapshot(&self) -> (MachineSnapshot, f64) {
        let (axes, pose) = self.runner.state.pos.numbers();
        let snap = MachineSnapshot {
            mpos: axes,
            wpos: axes,
            modal: serde_json::to_value(&self.runner.state.modal).unwrap_or_default(),
            tool: 0.0,
            params: Value::Null,
        };
        (snap, pose)
    }

    /// Sends the next queued feeder command. Returns the feeder's pending flag.
    pub(crate) fn feeder_next(&mut self) -> bool {
        let (snap, pose) = self.snapshot();
        let mut tool_change = false;
        let Self {
            feeder,
            sender,
            message_slot,
            shared_context,
            services,
            ..
        } = self;
        let next = feeder.next(&mut |raw, ctx| {
            let (mut line, words) =
                match prepare(raw, ctx, &snap, pose, shared_context, message_slot) {
                    Pre::Skip => return FilterResult::skip(),
                    Pre::Wait(dwell, reason) => {
                        debug!("%wait: line={raw:?}");
                        sender.hold(Some(reason));
                        return FilterResult::line(dwell);
                    }
                    Pre::Gcode(line, words) => (line, words),
                };
            let mut hold = None;
            // M109/M190 wait for the target temperature; M0/M1 pause.
            for (data, is) in [
                ("M109", is_m109 as fn(&str) -> bool),
                ("M190", is_m190),
                ("M0", is_m0),
                ("M1", is_m1),
            ] {
                if has(&words, is) {
                    debug!("{data}: line={raw:?}");
                    let reason = hold_reason(data, message_slot, raw);
                    hold.get_or_insert(reason);
                }
            }
            if has(&words, is_m6) {
                debug!("M6 Tool Change: line={raw:?}");
                let action = m6_action(&services.config);
                if action != M6Action::Pass {
                    line = replace_m6(&line, |x| format!("({x})"));
                    let reason = hold_reason("M6", message_slot, raw);
                    hold.get_or_insert(reason);
                    tool_change = action == M6Action::Manual;
                }
            }
            FilterResult { line, hold }
        });
        if let Some((line, context)) = next {
            self.feeder_write(&line, context);
        }
        if tool_change {
            self.tool_change();
        }
        self.feeder.is_pending()
    }

    fn feeder_write(&mut self, line: &str, mut context: Context) {
        if self.is_closed() {
            error!("Serial port {:?} is not accessible", self.options.port);
            return;
        }
        let line = line.trim();
        if line.is_empty() {
            return;
        }
        let data = format!("{line}\n");
        context.insert("source".into(), json!(WRITE_SOURCE_FEEDER));
        self.emit("serialport:write", &[json!(data), Value::Object(context)]);
        self.conn_write(&data, WRITE_SOURCE_FEEDER);
    }

    /// Sends the next program line.
    pub(crate) fn sender_next(&mut self) {
        let (snap, pose) = self.snapshot();
        let mut deferred = Vec::new();
        let Self {
            sender,
            workflow,
            sockets,
            services,
            message_slot,
            shared_context,
            ..
        } = self;
        let out = sender.next(&mut |raw, ctx| {
            let (mut line, words) =
                match prepare(raw, ctx, &snap, pose, shared_context, message_slot) {
                    Pre::Skip => return FilterResult::skip(),
                    Pre::Wait(dwell, reason) => {
                        return FilterResult {
                            line: dwell,
                            hold: Some(reason),
                        }
                    }
                    Pre::Gcode(line, words) => (line, words),
                };
            let mut hold = None;
            for (data, is) in [("M109", is_m109 as fn(&str) -> bool), ("M190", is_m190)] {
                if has(&words, is) {
                    debug!("{data}: line={raw:?}");
                    let reason = hold_reason(data, message_slot, raw);
                    hold.get_or_insert(reason);
                }
            }
            let mut pause = |data: &str, hold: &mut Option<Value>| {
                debug!("{data} Program Pause: line={raw:?}");
                deferred.extend(
                    services
                        .trigger("gcode:pause")
                        .into_iter()
                        .map(Deferred::Gcode),
                );
                let reason = hold_reason(data, message_slot, raw);
                if workflow.pause() {
                    sockets.emit("workflow:state", &[json!(workflow.state.as_str())]);
                    hold.get_or_insert(reason);
                }
            };
            for (data, is) in [("M0", is_m0 as fn(&str) -> bool), ("M1", is_m1)] {
                if has(&words, is) {
                    pause(data, &mut hold);
                }
            }
            if has(&words, is_m6) {
                let action = m6_action(&services.config);
                if action != M6Action::Pass {
                    line = replace_m6(&line, |x| format!("({x})"));
                    pause("M6", &mut hold);
                    if action == M6Action::Manual {
                        deferred.push(Deferred::ToolChange);
                    }
                }
            }
            FilterResult { line, hold }
        });
        if out.started.is_some() {
            self.sender_finish_time = 0;
        }
        if let Some(t) = out.ended {
            self.sender_finish_time = t;
        }
        // The JS ran these from inside the filter: before the line was written and
        // while it did not count as sent yet.
        self.sender_in_flight = out.lines.len();
        for d in deferred {
            match d {
                Deferred::Gcode(commands) => self.feed_gcode(&[commands], Context::new()),
                Deferred::ToolChange => self.tool_change(),
            }
        }
        self.sender_in_flight = 0;
        for line in &out.lines {
            self.sender_write(line);
        }
    }

    fn sender_write(&mut self, line: &str) {
        if self.is_closed() {
            error!("Serial port {:?} is not accessible", self.options.port);
            return;
        }
        if self.workflow.is_idle() {
            error!("Unexpected workflow state: idle");
            return;
        }
        let line = line.trim();
        if line.is_empty() {
            warn!("Expected non-empty line: N={}", self.sender.state.sent);
            return;
        }
        self.conn_write(&format!("{line}\n"), WRITE_SOURCE_SENDER);
    }

    /// The `gcode` command: queues the lines and starts the feeder when nothing is in flight.
    pub(crate) fn feed_gcode<S: AsRef<str>>(&mut self, commands: &[S], context: Context) {
        let lines: Vec<String> = commands
            .iter()
            .flat_map(|c| c.as_ref().split('\n'))
            .map(|l| l.strip_suffix('\r').unwrap_or(l))
            .filter(|l| !l.trim().is_empty())
            .map(str::to_string)
            .collect();
        self.feeder.feed(lines, context);
        let s = &self.sender.state;
        let sender_idle = s.sent == s.received + self.sender_in_flight;
        if self.write_source.is_none() && sender_idle && !self.feeder.is_pending() {
            self.feeder_next();
        }
    }

    fn gcode_load(&mut self, name: String, mut gcode: String, context: Context, r: Option<Reply>) {
        // A dwell empties the planner queue and answers ok once it is done: the fastest
        // way to know the program finished without polling.
        gcode.push_str("\n%wait ; Wait for the planner to empty");
        if !self.sender.load(&name, &gcode, context) {
            reply(r, Err(format!("Invalid G-code: name={name}")));
            return;
        }
        let s = &self.sender.state;
        self.emit(
            "gcode:load",
            &[
                json!(s.name),
                json!(s.gcode),
                Value::Object(s.context.clone()),
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

    fn reset(&mut self) {
        self.workflow_stop();
        self.feeder.reset();
        // M112: emergency stop
        self.writeln("M112", Context::new());
    }

    /// Manual tool change program (Marlin has no work coordinate systems).
    fn tool_change(&mut self) {
        let config = Arc::clone(&self.services.config);
        let probe = "G91 [tool_probe_command] F[tool_probe_feedrate] Z[tool_probe_z - posz - tool_probe_distance]";
        let block: Vec<String> = match tool_change_policy(&config) {
            // Set the current work Z position to the touch plate height.
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS => {
                vec![probe.into(), "G92 Z[touch_plate_height]".into()]
            }
            // Subtract the touch plate height from the current work Z position.
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO => {
                vec![
                    probe.into(),
                    "%wait 1".into(),
                    "G92 Z[posz - touch_plate_height]".into(),
                ]
            }
            TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING => {
                tool_probe_custom_commands(&config)
            }
            _ => vec![],
        };
        let (lines, context) = tool_change_program(&config, &self.runner.state.modal.units, &block);
        self.feed_gcode(&lines, context);
    }

    fn find_macro(&self, id: Option<&Value>) -> Option<Value> {
        self.services
            .config
            .get_array("macros")
            .into_iter()
            .find(|m| m.get("id") == id)
    }

    pub fn command(&mut self, cmd: &str, mut args: Vec<Value>, r: Option<Reply>) {
        match cmd {
            "gcode:load" => {
                let context = obj_arg(args.get(2));
                let (name, gcode) = (take_str(&mut args, 0), take_str(&mut args, 1));
                self.gcode_load(name, gcode, context, r);
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
            }
            "gcode:pause" => {
                self.trigger("gcode:pause");
                self.workflow_pause(None);
            }
            "gcode:resume" => {
                self.trigger("gcode:resume");
                self.workflow_resume();
            }
            "gcode" | "feeder:feed" => {
                self.feed_gcode(&js_lines(args.first()), obj_arg(args.get(1)))
            }
            "feeder:start" => {
                if !self.workflow.is_running() {
                    self.feeder.unhold();
                    self.feeder_next();
                }
            }
            "feeder:stop" => self.feeder.reset(),
            "feedhold" | "cyclestart" | "sleep" => self.trigger(cmd),
            "homing" => {
                self.trigger("homing");
                // G28: auto home
                self.writeln("G28", Context::new());
            }
            // Unsupported by Marlin.
            "unlock" | "rapidOverride" => {}
            "reset" => self.reset(),
            "feedOverride" | "spindleOverride" => {
                let value = args.first().and_then(Value::as_f64).unwrap_or(f64::NAN);
                let feed = cmd == "feedOverride";
                let state = &mut self.runner.state;
                let current = if feed {
                    &mut state.ov_f
                } else {
                    &mut state.ov_s
                };
                *current = override_value(*current, value);
                // M220: speed factor override, M221: extruder factor override
                let line = format!(
                    "{}S{}",
                    if feed { "M220" } else { "M221" },
                    js_num_to_string(*current)
                );
                self.feed_gcode(&[line], Context::new());
            }
            // M17: enable all stepper motors, M18: disable them
            "motor:enable" => self.feed_gcode(&["M17"], Context::new()),
            "motor:disable" => self.feed_gcode(&["M18"], Context::new()),
            "laser:on" => {
                let (power, max_s) = (num_arg(&args, 0, 0.0), num_arg(&args, 1, 255.0));
                self.feed_gcode(
                    &[format!(
                        "M3S{}",
                        js_num_to_string(positive(max_s * (power / 100.0)))
                    )],
                    Context::new(),
                );
            }
            "lasertest:on" => {
                let (power, duration, max_s) = (
                    num_arg(&args, 0, 0.0),
                    num_arg(&args, 1, 0.0),
                    num_arg(&args, 2, 255.0),
                );
                let mut commands = vec![format!(
                    "M3S{}",
                    js_num_to_string(positive(max_s * (power / 100.0)))
                )];
                if duration > 0.0 {
                    // G4 P<milliseconds>
                    commands.push(format!("G4 P{}", js_num_to_string(positive(duration))));
                    commands.push("M5".into());
                }
                self.feed_gcode(&commands, Context::new());
            }
            "lasertest:off" => self.writeln("M5", Context::new()),
            "macro:run" | "macro:load" => {
                let Some(m) = self.find_macro(args.first()) else {
                    error!(
                        "Cannot find the macro: id={}",
                        args.first().cloned().unwrap_or_default()
                    );
                    return;
                };
                self.trigger(cmd);
                let context = obj_arg(args.get(1));
                if cmd == "macro:run" {
                    self.feed_gcode(&js_lines(m.get("content")), context);
                    reply(r, Ok(Value::Null));
                } else {
                    let mut m = vec![
                        m.get("name").cloned().unwrap_or_default(),
                        m.get("content").cloned().unwrap_or_default(),
                    ];
                    self.gcode_load(take_str(&mut m, 0), take_str(&mut m, 1), context, r);
                }
            }
            "watchdir:load" => {
                let file = take_str(&mut args, 0);
                let monitor = Arc::clone(&self.services.monitor);
                let tx = self.internal.clone();
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
                let params = args.first().cloned().unwrap_or_else(|| json!({}));
                if params.get("mode").and_then(Value::as_str) != Some("test") {
                    self.pending_probe_capture = false;
                }
                let gcode = self.probe.start(&params);
                self.feed_gcode(&gcode, Context::new());
            }
            "autolevel:stop" => {
                // Reset the machine to cancel the probe cycle immediately.
                self.reset();
                self.probe.clear();
                self.pending_probe_capture = false;
                info!("[autolevel:stop] Probe stopped and state cleared");
            }
            "autolevel:getProbeState" => reply(r, Ok(json!({ "state": self.probe.to_json() }))),
            "autolevel:loadFromFile" | "autolevel:saveToFile" => {
                let path = take_str(&mut args, 0);
                let result = if cmd == "autolevel:loadFromFile" {
                    self.probe.load_from_file(&path)
                } else {
                    self.probe.save_to_file(&path)
                };
                if let Err(e) = &result {
                    error!("[{cmd}] Error with probe data file {path:?}: {e}");
                }
                reply(r, result);
            }
            "autolevel:applyProbeCompensation" => {
                let params = args.into_iter().next().unwrap_or_default();
                tokio::task::spawn_blocking(move || {
                    let result = apply_probe_compensation_params(&params);
                    info!("[autolevel:applyProbeCompensation] Probe compensation applied");
                    reply(r, Ok(result));
                });
            }
            _ => error!("Unknown command: {cmd}"),
        }
    }

    /// Handles one line received from the controller.
    pub fn parse(&mut self, line: &str) {
        trace!("< {line}");
        let raw = line.trim_end();
        let Some(res) = self.runner.parse(raw) else {
            return;
        };
        match res {
            Response::Start => {
                self.emit_read(raw);
                // Marlin prints 'start' after power-on only, and may miss the M115 sent on
                // open while booting: ask again for the firmware report.
                self.conn_write("M115\n", WRITE_SOURCE_SERVER);
            }
            Response::Echo(_) | Response::Others => self.emit_read(raw),
            Response::Firmware(_) => {
                self.emit_read(raw);
                if !self.ready {
                    self.ready = true;
                    self.trigger("controller:ready");
                }
            }
            Response::Position(_) => self.on_pos(raw),
            Response::Temperature(t) => {
                if self.source_is_client_or_feeder() {
                    self.emit_read(raw);
                }
                // `ok T:...` answers M105: an ok with an empty response follows.
                if t.ok {
                    self.on_ok(None);
                }
            }
            Response::Ok => self.on_ok(Some(raw)),
            Response::Error(_) => self.on_error(raw),
        }
    }

    fn on_pos(&mut self, raw: &str) {
        if self.pending_probe_capture {
            self.pending_probe_capture = false;
            let imperial = self.runner.state.modal.units == "G20";
            let z = Some(js_str_to_num(&self.runner.state.pos.z))
                .filter(|z| z.is_finite())
                .unwrap_or(0.0);
            // Probe data is stored in mm, at the intended grid node's XY.
            let z_mm = if imperial { in2mm(z) } else { z };
            if let Some((update, completed)) = self.probe.record(z_mm, imperial) {
                self.emit("autolevel:update", &[update]);
                if completed {
                    self.emit("autolevel:complete", &[]);
                }
            }
        }
        if self.source_is_client_or_feeder() {
            self.emit_read(raw);
        }
    }

    fn on_ok(&mut self, raw: Option<&str>) {
        if let Some(raw) = raw {
            match self.write_source {
                Some(WRITE_SOURCE_CLIENT | WRITE_SOURCE_FEEDER) => self.emit_read(raw),
                None => {
                    self.emit_read(raw);
                    error!("\"history.writeSource\" should not be empty");
                }
                _ => {}
            }
        }

        // Marlin does not report probe results like Grbl's PRB: query the position once
        // a G38.x completes and capture it from the M114 report.
        if PROBE_RE.is_match(&self.write_line) {
            debug!("[autolevel] Probe command completed, querying position with M114");
            self.clear_history();
            self.writeln("M114", Context::new());
            self.pending_probe_capture = true;
            // Wait for the M114 response before sending anything else (send-response).
            return;
        }

        self.clear_history();

        // Preemptive query to prevent starvation.
        if self.query.is_some() && (now_ms() - self.last_query_time).abs() > 2000 {
            self.query_issue();
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
            if received + 1 >= sent {
                debug!(
                    "Stop sending G-code: hold={hold}, sent={sent}, received={}",
                    received + 1
                );
            }
            self.sender.ack();
            self.sender_next();
            return;
        }
        if self.feeder_next() {
            return;
        }
        self.query_issue();
    }

    fn on_error(&mut self, raw: &str) {
        if self.workflow.is_running() {
            let ignore_errors = self
                .services
                .config
                .get_or("state.controller.exception.ignoreErrors", false);
            let s = &self.sender.state;
            // Like the JS: the line before the acknowledged one.
            let line = s
                .received
                .checked_sub(1)
                .and_then(|i| s.lines.get(i))
                .map_or("", |l| l.trim());
            let msg = format!("> {line} (ln={})", s.received + 1);
            self.emit_read(&msg);
            self.emit_read(raw);
            if !ignore_errors {
                self.workflow_pause(Some(json!({ "err": true, "msg": raw })));
            }
            self.sender.ack();
            self.sender_next();
            return;
        }
        self.emit_read(raw);
        self.feeder_next();
    }

    fn schedule_query(&mut self, kind: Query) {
        let (throttle, last, tolerance) = match kind {
            Query::Position => (
                &mut self.position_throttle,
                &mut self.position_query_time,
                5000,
            ),
            Query::Temperature => (
                &mut self.temperature_throttle,
                &mut self.temperature_query_time,
                10000,
            ),
        };
        if !throttle.ready() {
            return;
        }
        let now = now_ms();
        // A query still waiting to be issued is rescheduled after the tolerance time.
        if self.query.is_none() || (*last > 0 && (now - *last).abs() >= tolerance) {
            self.query = Some(kind);
            *last = now;
        }
    }

    fn query_issue(&mut self) {
        let Some(kind) = self.query.take() else {
            return;
        };
        let data = match kind {
            Query::Position => "M114\n",
            Query::Temperature => "M105\n",
        };
        self.conn_write(data, WRITE_SOURCE_SERVER);
        self.last_query_time = now_ms();
    }

    /// The 250 ms query timer.
    pub fn tick(&mut self) {
        if self.is_closed() {
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
            .is_some_and(|s| s.pos == self.runner.state.pos);
        if self.settings.as_ref() != Some(&self.runner.settings) {
            self.settings = Some(self.runner.settings.clone());
            self.emit(
                "controller:settings",
                &[json!(TYPE), json!(self.runner.settings)],
            );
        }
        if self.state.as_ref() != Some(&self.runner.state) {
            self.state = Some(self.runner.state.clone());
            self.emit("controller:state", &[json!(TYPE), json!(self.runner.state)]);
        }
        if !self.ready {
            return;
        }
        self.schedule_query(Query::Position);
        self.schedule_query(Query::Temperature);
        let s = &self.sender.state;
        if self.write_source.is_none() && s.sent == s.received && self.feeder.size() == 0 {
            self.query_issue();
        }
        // Stop the workflow once the machine stopped moving after the program finished.
        if self.sender_finish_time > 0 {
            let now = now_ms();
            if !zero_offset {
                self.sender_finish_time = now;
            } else if (now - self.sender_finish_time).abs() > 500 {
                trace!("Finished sending G-code");
                self.sender_finish_time = 0;
                self.trigger("gcode:stop");
                self.workflow_stop();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use cncr_core::autolevel::{Point2, Point3};
    use cncr_core::controller::Registry;
    use cncr_core::hub::{Frame, Hub};
    use cncr_core::monitor::Monitor;
    use cncr_core::taskrunner::TaskRunner;
    use cncr_core::TOOL_CHANGE_POLICY_SEND_M6_COMMANDS;

    use super::*;

    struct Recorder(Arc<Mutex<Vec<String>>>);

    impl Port for Recorder {
        fn write(&mut self, data: &str) {
            self.0.lock().unwrap().push(data.to_string());
        }
    }

    struct Harness {
        ctl: Controller,
        writes: Arc<Mutex<Vec<String>>>,
        frames: mpsc::UnboundedReceiver<Frame>,
        _internal: mpsc::UnboundedReceiver<Internal>,
    }

    impl Harness {
        fn new(config: Value) -> Self {
            let hub = Arc::new(Hub::default());
            let services = Services {
                config: Arc::new(ConfigStore::in_memory(config)),
                tasks: Arc::new(TaskRunner::new(Arc::clone(&hub))),
                hub,
                monitor: Arc::new(Monitor::new(None)),
                registry: Arc::new(Registry::default()),
            };
            let options = ControllerOptions {
                port: "/dev/null".into(),
                baudrate: 115200,
                rtscts: false,
                pin: Default::default(),
            };
            let writes = Arc::new(Mutex::new(Vec::new()));
            let (tx, internal) = mpsc::unbounded_channel();
            let mut ctl = Controller::new(
                services,
                options,
                Box::new(Recorder(Arc::clone(&writes))),
                tx,
            );
            let (socket, frames) = Socket::new();
            ctl.add_socket(socket);
            let mut h = Self {
                ctl,
                writes,
                frames,
                _internal: internal,
            };
            h.events("");
            h
        }

        fn writes(&self) -> Vec<String> {
            self.writes.lock().unwrap().clone()
        }

        fn cmd(&mut self, cmd: &str, args: Vec<Value>) {
            self.ctl.command(cmd, args, None);
        }

        /// Arguments of every `event` emitted since the last call (drains all frames).
        fn events(&mut self, event: &str) -> Vec<Value> {
            let mut out = Vec::new();
            while let Ok(f) = self.frames.try_recv() {
                let v: Value = serde_json::from_str(&f).unwrap();
                if v["event"] == event {
                    out.push(v["args"].clone());
                }
            }
            out
        }

        /// Feeder lines advance one per "ok" or hold release; pull them all.
        fn drain_feeder(&mut self) {
            loop {
                let written = self.writes.lock().unwrap().len();
                self.ctl.feeder.unhold();
                self.ctl.feeder_next();
                if self.ctl.feeder.size() == 0 && self.writes.lock().unwrap().len() == written {
                    break;
                }
            }
        }
    }

    fn setup() -> Harness {
        Harness::new(json!({}))
    }

    #[test]
    fn basic_commands() {
        let mut h = setup();
        h.cmd("homing", vec![]);
        h.cmd("reset", vec![]);
        h.cmd("lasertest:off", vec![]);
        h.cmd("unlock", vec![]);
        h.cmd("rapidOverride", vec![json!(50)]);
        h.cmd("statusreport", vec![]);
        assert_eq!(h.writes(), ["G28\n", "M112\n", "M5\n"]);
        assert_eq!(h.ctl.write_source, Some(WRITE_SOURCE_CLIENT));
    }

    #[test]
    fn nothing_is_written_once_closed() {
        let mut h = setup();
        h.ctl.close();
        h.cmd("homing", vec![]);
        h.cmd("reset", vec![]);
        h.cmd("gcode", vec![json!("G0 X1")]);
        h.cmd("lasertest:off", vec![]);
        assert!(h.writes().is_empty());
    }

    #[test]
    fn overrides() {
        for (cmd, m) in [("feedOverride", "M220"), ("spindleOverride", "M221")] {
            for (value, expected) in [(0, 100), (450, 500), (-200, 10), (50, 150)] {
                let mut h = setup();
                h.cmd(cmd, vec![json!(value)]);
                assert_eq!(h.writes(), [format!("{m}S{expected}\n")]);
                assert_eq!(h.ctl.write_source, Some(WRITE_SOURCE_FEEDER));
                let state = &h.ctl.runner.state;
                assert_eq!(
                    if m == "M220" { state.ov_f } else { state.ov_s },
                    f64::from(expected)
                );
            }
        }
    }

    #[test]
    fn motor_and_laser() {
        let cases: [(&str, Vec<Value>, &[&str]); 6] = [
            ("motor:enable", vec![], &["M17\n"]),
            ("motor:disable", vec![], &["M18\n"]),
            ("laser:on", vec![], &["M3S0\n"]),
            ("laser:on", vec![json!(50), json!(1000)], &["M3S500\n"]),
            (
                "lasertest:on",
                vec![json!(100), json!(50), json!(255)],
                &["M3S255\n", "G4 P50\n", "M5\n"],
            ),
            ("lasertest:on", vec![json!(50)], &["M3S127.5\n"]),
        ];
        for (cmd, args, expected) in cases {
            let mut h = setup();
            h.cmd(cmd, args);
            h.drain_feeder();
            assert_eq!(h.writes(), expected, "{cmd}");
        }
    }

    #[test]
    fn gcode_feeds_one_line_per_ok() {
        let mut h = setup();
        h.cmd("gcode", vec![json!("G0 X1\n\nG0 Y2")]);
        assert_eq!(h.writes(), ["G0 X1\n"]);
        let writes = h.events("serialport:write");
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0][0], "G0 X1\n");
        assert_eq!(writes[0][1]["source"], "feeder");
        h.ctl.parse("ok");
        assert_eq!(h.writes(), ["G0 X1\n", "G0 Y2\n"]);
        assert_eq!(h.events("serialport:read"), [json!(["ok"])]);
    }

    #[test]
    fn gcode_drops_blank_and_non_string_lines() {
        let mut h = setup();
        h.cmd("feeder:feed", vec![json!(["", "  ", null, "G0 X1"])]);
        assert_eq!(h.writes(), ["G0 X1\n"]);
    }

    #[test]
    fn heating_holds_the_feeder_until_feeder_start() {
        let mut h = setup();
        h.cmd("gcode", vec![json!("M109 S200")]);
        h.cmd("gcode", vec![json!("G0 X1")]);
        assert_eq!(h.writes(), ["M109 S200\n"]);
        assert_eq!(
            h.ctl.feeder.hold_reason,
            Some(json!({"data": "M109", "msg": "M109 S200"}))
        );
        h.cmd("feeder:start", vec![]);
        assert_eq!(h.writes(), ["M109 S200\n", "G0 X1\n"]);
    }

    #[test]
    fn feeder_start_does_nothing_while_running() {
        let mut h = setup();
        h.cmd("gcode:load", vec![json!("test"), json!("G0 X0\nG0 Y2")]);
        h.cmd("gcode:start", vec![]);
        h.cmd("gcode", vec![json!("G0 X9")]);
        h.cmd("feeder:start", vec![]);
        assert_eq!(h.writes(), ["G0 X0\n"]);
    }

    #[test]
    fn feeder_stop_clears_the_queue() {
        let mut h = setup();
        h.cmd("gcode", vec![json!(["G0 X1", "G0 Y2"])]);
        h.cmd("feeder:stop", vec![]);
        h.ctl.parse("ok");
        assert_eq!(h.writes(), ["G0 X1\n"]);
    }

    #[test]
    fn temperature_report_with_ok_acknowledges() {
        let mut h = setup();
        h.cmd("gcode", vec![json!(["M105", "G0 X1"])]);
        h.ctl.parse("ok T:20.0 /0.0 B:25.0 /60.0 @:0 B@:0");
        assert_eq!(h.writes(), ["M105\n", "G0 X1\n"]);
        assert_eq!(
            h.ctl.runner.state.heated_bed.deg_target.as_deref(),
            Some("60.0")
        );
        assert_eq!(
            h.events("serialport:read"),
            [json!(["ok T:20.0 /0.0 B:25.0 /60.0 @:0 B@:0"])]
        );
    }

    #[test]
    fn gcode_load_appends_the_planner_dwell() {
        let mut h = setup();
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        h.ctl.command(
            "gcode:load",
            vec![json!("test"), json!("G0 X0\nG0 Y2")],
            Some(tx),
        );
        let json = rx.try_recv().unwrap().unwrap();
        assert_eq!(
            (
                json["name"].as_str(),
                json["total"].as_u64(),
                json["sent"].as_u64()
            ),
            (Some("test"), Some(3), Some(0))
        );
        assert!(h.writes().is_empty());

        // The dwell line keeps an empty program loadable.
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        h.ctl
            .command("gcode:load", vec![json!("test"), json!("")], Some(tx));
        assert_eq!(rx.try_recv().unwrap().unwrap()["total"], 1);
    }

    #[test]
    fn sender_workflow() {
        let mut h = setup();
        h.cmd("gcode:load", vec![json!("test"), json!("G0 X0\nG0 Y2")]);
        h.cmd("gcode:start", vec![]);
        assert_eq!(h.writes(), ["G0 X0\n"]);
        assert!(h.ctl.workflow.is_running());
        assert_eq!(h.ctl.write_source, Some(WRITE_SOURCE_SENDER));

        h.cmd("gcode:pause", vec![]);
        assert!(h.ctl.workflow.is_paused());
        h.cmd("gcode:resume", vec![]);
        assert_eq!(h.writes(), ["G0 X0\n", "G0 Y2\n"]);

        h.cmd("gcode:stop", vec![]);
        assert!(h.ctl.workflow.is_idle());
        h.cmd("start", vec![]);
        assert_eq!(
            h.writes(),
            ["G0 X0\n", "G0 Y2\n", "G0 X0\n"],
            "stop rewinds"
        );

        h.cmd("gcode:unload", vec![]);
        assert!(h.ctl.sender.state.gcode.is_empty());
        assert_eq!(h.ctl.sender.state.total, 0);
        assert!(h.ctl.workflow.is_idle());
    }

    #[test]
    fn program_runs_through_m0_and_the_final_dwell() {
        let mut h = setup();
        h.cmd(
            "gcode:load",
            vec![json!("p"), json!("G0 X1\n%msg Check\nM0\nG0 X2")],
        );
        h.cmd("gcode:start", vec![]);
        h.events("");
        h.ctl.parse("ok");
        assert_eq!(h.writes(), ["G0 X1\n", "M0\n"]);
        assert!(h.ctl.workflow.is_paused());
        assert_eq!(
            h.ctl.sender.state.hold_reason,
            Some(json!({"data": "M0", "msg": "Check"}))
        );
        assert_eq!(h.events("workflow:state"), [json!(["paused"])]);

        h.ctl.parse("ok");
        assert_eq!(h.writes().len(), 2, "held while paused");
        h.cmd("gcode:resume", vec![]);
        h.ctl.parse("ok");
        assert_eq!(h.writes()[2..], ["G0 X2\n", "G4 S0.5\n"]);
        assert_eq!(
            h.ctl.sender.state.hold_reason.as_ref().unwrap()["data"],
            "%wait"
        );

        h.ctl.parse("ok");
        assert!(!h.ctl.sender.state.hold);
        assert_eq!(h.ctl.sender.state.received, h.ctl.sender.state.total);
        assert!(h.ctl.sender_finish_time > 0, "program finished");
    }

    #[test]
    fn error_while_running_pauses_the_program() {
        let mut h = setup();
        h.cmd("gcode:load", vec![json!("p"), json!("G0 X1\nG0 X2\nG0 X3")]);
        h.cmd("gcode:start", vec![]);
        h.ctl.parse("ok");
        h.events("");
        h.ctl.parse("Error:Unknown command");
        assert_eq!(
            h.events("serialport:read"),
            [json!(["> G0 X1 (ln=2)"]), json!(["Error:Unknown command"])]
        );
        assert!(h.ctl.workflow.is_paused());
        assert_eq!(
            h.ctl.sender.state.hold_reason,
            Some(json!({"err": true, "msg": "Error:Unknown command"}))
        );
        assert_eq!(h.writes(), ["G0 X1\n", "G0 X2\n"]);

        let mut h =
            Harness::new(json!({"state": {"controller": {"exception": {"ignoreErrors": true}}}}));
        h.cmd("gcode:load", vec![json!("p"), json!("G0 X1\nG0 X2")]);
        h.cmd("gcode:start", vec![]);
        h.ctl.parse("Error:Unknown command");
        assert!(h.ctl.workflow.is_running());
        assert_eq!(h.writes(), ["G0 X1\n", "G0 X2\n"]);
    }

    #[test]
    fn manual_tool_change_in_a_program_starts_before_the_m6_line_is_written() {
        let mut h = Harness::new(
            json!({"tool": {"toolChangePolicy": TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS}}),
        );
        h.cmd("gcode:load", vec![json!("p"), json!("M6 T1\nG0 X1")]);
        h.cmd("gcode:start", vec![]);
        assert_eq!(h.writes(), ["G4 S0.5\n", "(M6) T1\n"]);
        assert!(h.ctl.workflow.is_paused());
        assert_eq!(
            h.ctl.sender.state.hold_reason,
            Some(json!({"data": "M6", "msg": "M6 T1"}))
        );
        assert!(h.ctl.feeder.size() > 0);
    }

    #[test]
    fn macros() {
        let config = json!({"macros": [{"id": "m1", "name": "M1", "content": "G0 X1\nG0 Y2"}]});
        let mut h = Harness::new(config.clone());
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        h.ctl.command("macro:run", vec![json!("m1")], Some(tx));
        assert_eq!(rx.try_recv().unwrap(), Ok(Value::Null));
        assert_eq!(h.writes(), ["G0 X1\n"]);

        let (tx, mut rx) = tokio::sync::oneshot::channel();
        h.ctl.command("macro:run", vec![json!("missing")], Some(tx));
        assert!(rx.try_recv().is_err(), "no callback for an unknown macro");

        let mut h = Harness::new(config);
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        h.ctl.command("macro:load", vec![json!("m1")], Some(tx));
        let json = rx.try_recv().unwrap().unwrap();
        assert_eq!(
            (json["name"].as_str(), json["total"].as_u64()),
            (Some("M1"), Some(3))
        );
    }

    fn tool_change_sequence(policy_lines: &[&str], zero: &str) -> Vec<String> {
        let mut out: Vec<String> = vec![
            "G4 S0.5".into(),
            "M5".into(),
            "G90".into(),
            format!("G53 G0 Z{zero}"),
            format!("G53 G0 X{zero} Y{zero}"),
            "G4 S0.5".into(),
            "M0".into(),
            format!("G53 G0 X{zero} Y{zero}"),
            format!("G53 G0 Z{zero}"),
            "G4 S0.5".into(),
        ];
        out.extend(policy_lines.iter().map(|l| l.to_string()));
        out.extend([
            format!("G53 G0 Z{zero}"),
            format!("G53 G0 X{zero} Y{zero}"),
            "G4 S0.5".into(),
            "M0".into(),
            "G90".into(),
            "G0 X0 Y0".into(),
            "G0 Z0".into(),
            "M5".into(),
            "G4 S5".into(),
        ]);
        out.into_iter().map(|l| l + "\n").collect()
    }

    fn run_tool_change(policy: i64, units: &str) -> Harness {
        let mut h = Harness::new(json!({"tool": {
            "toolChangeX": 0, "toolChangeY": 0, "toolChangeZ": 0,
            "toolProbeX": 0, "toolProbeY": 0, "toolProbeZ": 0,
            "toolProbeCommand": "G38.2", "toolProbeDistance": 1, "toolProbeFeedrate": 10,
            "touchPlateHeight": 0, "toolProbeCustomCommands": "G53 G0 X5\nG4 P100",
            "toolChangePolicy": policy,
        }}));
        h.ctl.runner.state.modal.units = units.into();
        h.cmd("tool:change", vec![]);
        h.drain_feeder();
        h
    }

    #[test]
    fn tool_change_sequences() {
        let cases: [(i64, &[&str]); 5] = [
            (TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS, &[]),
            (TOOL_CHANGE_POLICY_SEND_M6_COMMANDS, &[]),
            (
                TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS,
                &["G91 G38.2 F10 Z-1", "G92 Z0"],
            ),
            (
                TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO,
                &["G91 G38.2 F10 Z-1", "G4 S1", "G92 Z0"],
            ),
            (
                TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING,
                &["G53 G0 X5", "G4 P100"],
            ),
        ];
        for (policy, lines) in cases {
            let h = run_tool_change(policy, "G21");
            assert_eq!(
                h.writes(),
                tool_change_sequence(lines, "0.000"),
                "policy {policy}"
            );
        }
        let h = run_tool_change(TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS, "G20");
        assert_eq!(
            h.writes(),
            tool_change_sequence(&["G91 G38.2 F0.3937 Z-0.0394", "G92 Z0"], "0.0000")
        );
    }

    #[test]
    fn autolevel_start() {
        let mut h = setup();
        h.cmd(
            "autolevel:start",
            vec![
                json!({"mode": "test", "clearanceZ": 5, "startZ": 2, "endZ": -1, "feedrate": 100}),
            ],
        );
        h.drain_feeder();
        assert_eq!(
            h.writes(),
            ["G90\n", "G0 Z5\n", "G0 Z2\n", "G38.2 Z-1 F100\n", "G0 Z5\n"]
        );
        assert!(h.ctl.probe.probe_points.is_empty());

        let mut h = setup();
        let params = json!({"mode": "full", "startX": 0, "endX": 10, "stepX": 10, "startY": 0, "endY": 10, "stepY": 10,
            "clearanceZ": 5, "startZ": 2, "endZ": -1, "feedrate": 100});
        h.cmd("autolevel:start", vec![params.clone()]);
        h.drain_feeder();
        let expected: Vec<String> = [
            ("0", "0", "50"),
            ("10", "0", "100"),
            ("0", "10", "100"),
            ("10", "10", "100"),
        ]
        .iter()
        .flat_map(|(x, y, f)| {
            [
                "G90".to_string(),
                "G0 Z5".into(),
                format!("G0 X{x} Y{y}"),
                "G0 Z2".into(),
                format!("G38.2 Z-1 F{f}"),
                "G0 Z5".into(),
            ]
        })
        .map(|l| l + "\n")
        .collect();
        assert_eq!(h.writes(), expected);
        assert_eq!(h.ctl.probe.probe_points.len(), 4);
        let mut config = params;
        config.as_object_mut().unwrap().remove("mode");
        assert_eq!(h.ctl.probe.config, Some(config));
    }

    #[test]
    fn autolevel_stop_resets_and_clears() {
        let mut h = setup();
        h.ctl.probe.probed_positions = vec![Point3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        }];
        h.ctl.probe.probe_points = vec![Point2 { x: 1.0, y: 2.0 }];
        h.ctl.pending_probe_capture = true;
        h.cmd("autolevel:stop", vec![]);
        assert_eq!(h.writes(), ["M112\n"]);
        assert_eq!(h.ctl.probe, ProbeState::default());
        assert!(!h.ctl.pending_probe_capture);
    }

    #[test]
    fn probe_completion_queries_and_records_the_grid_node() {
        let mut h = setup();
        h.ctl.probe.probe_points = vec![Point2 { x: 0.0, y: 0.0 }, Point2 { x: 10.0, y: 0.0 }];

        // Quantised reports: the commanded nodes (0,0) and (10,0) read back off-node.
        for (report, n) in [
            ("X:0.001 Y:0.001 Z:-1.500 E:0.000", 1),
            ("X:10.001 Y:-0.001 Z:-1.000 E:0.000", 2),
        ] {
            h.cmd("gcode", vec![json!("G38.2 Z-1")]);
            h.ctl.parse("ok");
            assert_eq!(h.writes().last().map(String::as_str), Some("M114\n"));
            assert!(h.ctl.pending_probe_capture);
            assert_eq!(h.writes().len(), 2 * n);
            h.ctl.parse(report);
            assert!(!h.ctl.pending_probe_capture);
            h.ctl.parse("ok");
        }
        assert_eq!(
            h.ctl.probe.probed_positions,
            [
                Point3 {
                    x: 0.0,
                    y: 0.0,
                    z: -1.5
                },
                Point3 {
                    x: 10.0,
                    y: 0.0,
                    z: -1.0
                }
            ]
        );
        assert_eq!(
            (h.ctl.probe.min_z, h.ctl.probe.max_z),
            (Some(-1.5), Some(-1.0))
        );
        h.events("");
        // Completed: further reports are not recorded.
        h.cmd("gcode", vec![json!("G38.2 Z-1")]);
        h.ctl.parse("ok");
        h.ctl.parse("X:0 Y:0 Z:-3 E:0");
        assert_eq!(h.ctl.probe.probed_positions.len(), 2);
        assert!(h.events("autolevel:update").is_empty());
    }

    #[test]
    fn probe_events() {
        let mut h = setup();
        h.ctl.probe.probe_points = vec![Point2 { x: 0.0, y: 0.0 }];
        h.cmd("gcode", vec![json!("G38.2 Z-1")]);
        h.ctl.parse("ok");
        h.events("");
        h.ctl.parse("X:0.000 Y:0.000 Z:-0.250 E:0.000");
        let mut frames = Vec::new();
        while let Ok(f) = h.frames.try_recv() {
            frames.push(serde_json::from_str::<Value>(&f).unwrap());
        }
        let names: Vec<&str> = frames
            .iter()
            .map(|f| f["event"].as_str().unwrap())
            .collect();
        assert_eq!(
            names,
            ["autolevel:update", "autolevel:complete", "serialport:read"]
        );
        assert_eq!(frames[0]["args"][0]["current"], 1);
        assert_eq!(
            frames[0]["args"][0]["probedPos"],
            json!({"x": 0.0, "y": 0.0, "z": -0.25})
        );
    }

    #[test]
    fn firmware_report_makes_the_controller_ready_and_queries_follow() {
        let mut h = setup();
        h.ctl.post_open();
        h.ctl.parse("start");
        assert_eq!(h.writes(), ["M115\n", "M115\n"]);
        h.ctl.parse("FIRMWARE_NAME:Marlin 2.1.2 (Github) PROTOCOL_VERSION:1.0 MACHINE_TYPE:RepRap EXTRUDER_COUNT:1");
        h.events("");
        h.ctl.tick();
        let settings = h.events("controller:settings");
        assert_eq!(settings[0][1]["firmwareName"], "Marlin 2.1.2");

        // The firmware report's ok: the position query scheduled by the timer goes out.
        h.ctl.parse("ok");
        assert_eq!(h.writes().last().map(String::as_str), Some("M114\n"));
        assert!(
            h.events("serialport:read").is_empty(),
            "replies to server queries are not echoed"
        );
        h.ctl.parse("X:1.00 Y:2.00 Z:3.00 E:0.00 Count X:0 Y:0 Z:0");
        h.ctl.parse("ok");
        h.ctl.tick();
        let state = h.events("controller:state");
        assert_eq!(
            state[0][1]["pos"],
            json!({"x": "1.00", "y": "2.00", "z": "3.00", "e": "0.00"})
        );
    }

    #[test]
    fn add_socket_replays_the_state() {
        let mut h = setup();
        h.cmd("gcode:load", vec![json!("p"), json!("G0 X1")]);
        h.ctl.tick();
        let (socket, mut rx) = Socket::new();
        h.ctl.add_socket(socket);
        let mut names = Vec::new();
        while let Ok(f) = rx.try_recv() {
            names.push(
                serde_json::from_str::<Value>(&f).unwrap()["event"]
                    .as_str()
                    .unwrap()
                    .to_string(),
            );
        }
        assert_eq!(
            names,
            [
                "serialport:open",
                "controller:state",
                "feeder:status",
                "sender:status",
                "gcode:load",
                "workflow:state"
            ]
        );
    }
}

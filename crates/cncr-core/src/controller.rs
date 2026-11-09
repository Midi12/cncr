//! Contract between the server (engine, REST API) and the controller crates.
//!
//! Each controller crate exposes
//! `pub async fn open(services: Services, options: ControllerOptions) -> Result<ControllerHandle, String>`
//! which opens the serial port, spawns the controller task and returns its handle.
//! The engine registers the handle in [`Registry`] after a successful open; the
//! controller task removes itself from the registry when its port closes.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use serde::Deserialize;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot};

use crate::config::ConfigStore;
use crate::events::Trigger;
use crate::hub::{Hub, Socket, SocketId};
use crate::monitor::Monitor;
use crate::taskrunner::TaskRunner;
use crate::Context;

/// Socket.io-style acknowledgement: `Ok(data)` → `[null, data]`, `Err(msg)` → `[msg]`.
pub type Reply = oneshot::Sender<Result<Value, String>>;

/// Answers an optional reply channel.
pub fn reply(r: Option<Reply>, result: Result<Value, String>) {
    if let Some(r) = r {
        let _ = r.send(result);
    }
}

#[derive(Debug, Clone, Default, Deserialize, PartialEq)]
pub struct PinOptions {
    pub dtr: Option<bool>,
    pub rts: Option<bool>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ControllerOptions {
    pub port: String,
    pub baudrate: u32,
    pub rtscts: bool,
    pub pin: PinOptions,
}

/// Requests from the engine / API to a controller task.
#[derive(Debug)]
pub enum ControllerMsg {
    /// Attach a client; the controller replays its current state to it
    /// (`serialport:open`, settings, state, feeder/sender status, loaded G-code, workflow).
    AddSocket(Socket),
    RemoveSocket(SocketId),
    /// `socket.emit('command', port, cmd, ...args)`.
    Command {
        cmd: String,
        args: Vec<Value>,
        reply: Option<Reply>,
    },
    Write {
        data: String,
        context: Context,
    },
    Writeln {
        data: String,
        context: Context,
    },
    /// `controller.status` for `GET /api/controllers`.
    Status(oneshot::Sender<Value>),
    /// `sender.toJSON()` plus `name` and the raw `gcode` (`GET /api/gcode*`).
    Gcode(oneshot::Sender<(Value, String, String)>),
    /// Close the port, emit `serialport:close`, deregister and stop the task.
    Close(oneshot::Sender<()>),
}

#[derive(Debug, Clone)]
pub struct ControllerHandle {
    /// `Grbl`, `Marlin`, `Smoothie` or `TinyG`.
    pub kind: &'static str,
    pub port: String,
    tx: mpsc::UnboundedSender<ControllerMsg>,
}

impl ControllerHandle {
    pub fn new(kind: &'static str, port: String, tx: mpsc::UnboundedSender<ControllerMsg>) -> Self {
        Self { kind, port, tx }
    }

    /// Returns false once the controller task has stopped.
    pub fn send(&self, msg: ControllerMsg) -> bool {
        self.tx.send(msg).is_ok()
    }

    pub fn is_alive(&self) -> bool {
        !self.tx.is_closed()
    }
}

/// Open controllers by port path (cncjs `store.controllers`).
#[derive(Debug, Default)]
pub struct Registry(Mutex<HashMap<String, ControllerHandle>>);

impl Registry {
    pub fn get(&self, port: &str) -> Option<ControllerHandle> {
        self.0
            .lock()
            .unwrap()
            .get(port)
            .filter(|h| h.is_alive())
            .cloned()
    }

    pub fn insert(&self, handle: ControllerHandle) {
        self.0.lock().unwrap().insert(handle.port.clone(), handle);
    }

    pub fn remove(&self, port: &str) {
        self.0.lock().unwrap().remove(port);
    }

    pub fn all(&self) -> Vec<ControllerHandle> {
        self.0
            .lock()
            .unwrap()
            .values()
            .filter(|h| h.is_alive())
            .cloned()
            .collect()
    }

    pub fn ports_in_use(&self) -> Vec<String> {
        self.all().into_iter().map(|h| h.port).collect()
    }
}

/// Process-wide services handed to every controller.
#[derive(Debug, Clone)]
pub struct Services {
    pub config: Arc<ConfigStore>,
    pub hub: Arc<Hub>,
    pub tasks: Arc<TaskRunner>,
    pub monitor: Arc<Monitor>,
    pub registry: Arc<Registry>,
}

impl Services {
    /// Fires the user event triggers for `event`. `system` triggers run as shell
    /// tasks; G-code triggers are returned for the caller to feed (cncjs runs them
    /// through `this.command('gcode', commands)`).
    pub fn trigger(&self, event: &str) -> Vec<String> {
        let mut gcode = Vec::new();
        for t in crate::events::triggers(&self.config, event) {
            tracing::debug!("EventTrigger: event={event:?}, trigger={t:?}");
            match t {
                Trigger::System(cmd) => {
                    self.tasks.run(&cmd, "");
                }
                Trigger::Gcode(cmd) => gcode.push(cmd),
            }
        }
        gcode
    }
}

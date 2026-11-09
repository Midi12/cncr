//! WebSocket engine (cncjs `CNCEngine`): one connection per client, JSON frames
//! `{"event", "args", "id"?}` in both directions; frames with an `id` are answered
//! with `{"event": "ack", "id", "args": [err, data]}`.

use std::sync::Arc;

use axum::extract::ws::{Message, WebSocket};
use cncr_core::controller::{
    ControllerHandle, ControllerMsg, ControllerOptions, PinOptions, Reply,
};
use cncr_core::hub::Socket;
use cncr_core::serial::list_ports;
use cncr_core::Context;
use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use serde_json::{json, Value};
use tokio::sync::oneshot;
use tracing::{debug, error, warn};

use crate::AppState;

#[derive(Debug, Deserialize)]
struct Incoming {
    event: String,
    #[serde(default)]
    args: Vec<Value>,
    #[serde(default)]
    id: Option<u64>,
}

fn ack(socket: &Socket, id: Option<u64>, result: Result<Value, String>) {
    let Some(id) = id else { return };
    let args = match result {
        Ok(data) => json!([null, data]),
        Err(e) => json!([e]),
    };
    socket.send_frame(
        json!({ "event": "ack", "id": id, "args": args })
            .to_string()
            .into(),
    );
}

/// Serves one client until it disconnects.
pub async fn serve(state: Arc<AppState>, ws: WebSocket, peer: String) {
    let (socket, mut outbox) = Socket::new();
    debug!("New connection from {peer}: id={}", socket.id);
    let services = &state.services;
    services.hub.add(socket.clone());

    let (mut sink, mut stream) = ws.split();
    let writer = tokio::spawn(async move {
        while let Some(frame) = outbox.recv().await {
            if sink
                .send(Message::Text(frame.as_ref().into()))
                .await
                .is_err()
            {
                break;
            }
        }
    });

    socket.emit(
        "startup",
        &[json!({
            "loadedControllers": state.controllers,
            "baudrates": services.config.get_array("baudrates"),
            "ports": services.config.get_array("ports"),
        })],
    );

    while let Some(msg) = stream.next().await {
        let text = match msg {
            Ok(Message::Text(t)) => t,
            Ok(Message::Close(_)) | Err(_) => break,
            Ok(_) => continue,
        };
        match serde_json::from_str::<Incoming>(text.as_str()) {
            Ok(m) => handle(&state, &socket, m).await,
            Err(e) => warn!("Invalid frame from {peer}: {e}"),
        }
    }

    debug!("Disconnected from {peer}: id={}", socket.id);
    for handle in services.registry.all() {
        handle.send(ControllerMsg::RemoveSocket(socket.id));
    }
    services.hub.remove(socket.id);
    writer.abort();
}

fn str_arg(args: &[Value], i: usize) -> String {
    match args.get(i) {
        Some(Value::String(s)) => s.clone(),
        Some(Value::Null) | None => String::new(),
        Some(v) => v.to_string(),
    }
}

fn context_arg(args: &[Value], i: usize) -> Context {
    match args.get(i) {
        Some(Value::Object(m)) => m.clone(),
        _ => Context::new(),
    }
}

/// Open controller for `port`, logging like cncjs when it is not.
fn controller(state: &AppState, port: &str) -> Option<ControllerHandle> {
    let h = state.services.registry.get(port);
    if h.is_none() {
        error!("Serial port {port:?} not accessible");
    }
    h
}

async fn handle(state: &Arc<AppState>, socket: &Socket, m: Incoming) {
    let services = &state.services;
    let args = &m.args;
    match m.event.as_str() {
        "list" => {
            let config_ports = services.config.get_array("ports");
            let in_use = services.registry.ports_in_use();
            let mut ports = tokio::task::spawn_blocking(list_ports)
                .await
                .unwrap_or_default();
            ports.extend(
                config_ports.into_iter().map(
                    |p| json!({ "port": p.get("path"), "manufacturer": p.get("manufacturer") }),
                ),
            );
            let ports: Vec<Value> = ports
                .into_iter()
                .map(|p| {
                    let path = p
                        .get("port")
                        .and_then(Value::as_str)
                        .unwrap_or_default()
                        .to_string();
                    let mut o = json!({ "port": path, "inuse": in_use.contains(&path) });
                    if let Some(m) = p.get("manufacturer").filter(|m| !m.is_null()) {
                        o["manufacturer"] = m.clone();
                    }
                    o
                })
                .collect();
            socket.emit("serialport:list", &[Value::Array(ports)]);
            ack(socket, m.id, Ok(Value::Null));
        }
        "open" => {
            let port = str_arg(args, 0);
            let result = open(
                state,
                socket,
                &port,
                args.get(1).cloned().unwrap_or(Value::Null),
            )
            .await;
            ack(socket, m.id, result.map(|()| Value::Null));
        }
        "close" => {
            let port = str_arg(args, 0);
            let Some(handle) = controller(state, &port) else {
                ack(
                    socket,
                    m.id,
                    Err(format!("Serial port \"{port}\" not accessible")),
                );
                return;
            };
            // System trigger: close a serial port.
            services.trigger("port:close");
            let (tx, rx) = oneshot::channel();
            if handle.send(ControllerMsg::Close(tx)) {
                let _ = rx.await;
            }
            services.registry.remove(&port);
            ack(socket, m.id, Ok(Value::Null));
        }
        "command" => {
            let port = str_arg(args, 0);
            let Some(handle) = controller(state, &port) else {
                return;
            };
            let cmd = str_arg(args, 1);
            let cmd_args = args.get(2..).map(<[Value]>::to_vec).unwrap_or_default();
            let reply = m.id.map(|id| {
                let (tx, rx): (Reply, _) = oneshot::channel();
                let socket = socket.clone();
                tokio::spawn(async move {
                    // A dropped reply (command without callback) acknowledges with no error.
                    ack(&socket, Some(id), rx.await.unwrap_or(Ok(Value::Null)));
                });
                tx
            });
            handle.send(ControllerMsg::Command {
                cmd,
                args: cmd_args,
                reply,
            });
        }
        ev @ ("write" | "writeln") => {
            let port = str_arg(args, 0);
            let Some(handle) = controller(state, &port) else {
                return;
            };
            let (data, context) = (str_arg(args, 1), context_arg(args, 2));
            handle.send(if ev == "write" {
                ControllerMsg::Write { data, context }
            } else {
                ControllerMsg::Writeln { data, context }
            });
            ack(socket, m.id, Ok(Value::Null));
        }
        other => warn!("Unknown event {other:?}"),
    }
}

async fn open(
    state: &Arc<AppState>,
    socket: &Socket,
    port: &str,
    options: Value,
) -> Result<(), String> {
    let services = &state.services;
    debug!("socket.open({port:?}, {options}): id={}", socket.id);
    if let Some(handle) = services.registry.get(port) {
        handle.send(ControllerMsg::AddSocket(socket.clone()));
        return Ok(());
    }
    let mut kind = options
        .get("controllerType")
        .and_then(Value::as_str)
        .unwrap_or(cncr_grbl::TYPE);
    if kind == "TinyG2" {
        // TinyG2 is a deprecated alias.
        kind = cncr_tinyg::TYPE;
    }
    if !state.controllers.contains(&kind) {
        let err = format!("Not supported controller: {kind}");
        error!("{err}");
        return Err(err);
    }
    let baudrate = options
        .get("baudrate")
        .and_then(Value::as_f64)
        .unwrap_or(115_200.0) as u32;
    let opts = ControllerOptions {
        port: port.to_string(),
        baudrate,
        rtscts: options
            .get("rtscts")
            .and_then(Value::as_bool)
            .unwrap_or(false),
        pin: options
            .get("pin")
            .and_then(|p| serde_json::from_value::<PinOptions>(p.clone()).ok())
            .unwrap_or_default(),
    };
    let s = services.clone();
    let opened = match kind {
        cncr_grbl::TYPE => cncr_grbl::open(s, opts).await,
        cncr_marlin::TYPE => cncr_marlin::open(s, opts).await,
        cncr_smoothie::TYPE => cncr_smoothie::open(s, opts).await,
        _ => cncr_tinyg::open(s, opts).await,
    };
    match opened {
        Ok(handle) => {
            // System trigger: open a serial port.
            services.trigger("port:open");
            services.registry.insert(handle.clone());
            handle.send(ControllerMsg::AddSocket(socket.clone()));
            Ok(())
        }
        Err(err) => {
            error!("Error opening serial port {port:?}: {err}");
            socket.emit("serialport:error", &[json!({ "err": err, "port": port })]);
            Err(err)
        }
    }
}

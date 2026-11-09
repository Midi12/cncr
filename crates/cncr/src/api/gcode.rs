//! `/api/gcode*` (program of a port's sender) and `/api/controllers`.

use axum::body::Bytes;
use axum::extract::{Query, State};
use axum::http::header;
use axum::response::{IntoResponse, Response};
use cncr_core::controller::{ControllerHandle, ControllerMsg};
use serde::Deserialize;
use serde_json::{json, Value};
use tokio::sync::oneshot;

use super::{err, ok, AppRef};
use crate::body_object;

#[derive(Debug, Deserialize, Default)]
pub struct PortQuery {
    port: Option<String>,
}

fn controller(s: &AppRef, port: Option<&str>) -> Result<ControllerHandle, Response> {
    let port = port
        .filter(|p| !p.is_empty())
        .ok_or_else(|| err(400, "No port specified"))?;
    s.services
        .registry
        .get(port)
        .ok_or_else(|| err(400, "Controller not found"))
}

async fn sender_snapshot(h: &ControllerHandle) -> Result<(Value, String, String), Response> {
    let (tx, rx) = oneshot::channel();
    h.send(ControllerMsg::Gcode(tx));
    rx.await.map_err(|_| err(400, "Controller not found"))
}

pub async fn upload(State(s): State<AppRef>, body: Bytes) -> Response {
    let b = body_object(&body);
    let h = match controller(&s, b.get("port").and_then(Value::as_str)) {
        Ok(h) => h,
        Err(r) => return r,
    };
    let gcode = b.get("gcode").and_then(Value::as_str).unwrap_or_default();
    if gcode.is_empty() {
        return err(400, "Empty G-code");
    }
    let name = b.get("name").cloned().unwrap_or(Value::Null);
    let context = b.get("context").cloned().unwrap_or_else(|| json!({}));
    let (tx, rx) = oneshot::channel();
    h.send(ControllerMsg::Command {
        cmd: "gcode:load".into(),
        args: vec![name, json!(gcode), context],
        reply: Some(tx),
    });
    match rx.await {
        Ok(Ok(state)) => ok(state),
        Ok(Err(e)) => err(500, format!("Failed to load G-code: {e}")),
        Err(_) => err(500, "Failed to load G-code: controller closed"),
    }
}

pub async fn fetch(State(s): State<AppRef>, Query(q): Query<PortQuery>) -> Response {
    let h = match controller(&s, q.port.as_deref()) {
        Ok(h) => h,
        Err(r) => return r,
    };
    match sender_snapshot(&h).await {
        Ok((mut status, _name, gcode)) => {
            status["data"] = json!(gcode);
            ok(status)
        }
        Err(r) => r,
    }
}

pub async fn download(
    State(s): State<AppRef>,
    Query(q): Query<PortQuery>,
    body: Bytes,
) -> Response {
    let body_port = body_object(&body)
        .get("port")
        .and_then(Value::as_str)
        .map(str::to_string);
    let h = match controller(&s, q.port.as_deref().or(body_port.as_deref())) {
        Ok(h) => h,
        Err(r) => return r,
    };
    match sender_snapshot(&h).await {
        Ok((_, name, gcode)) => {
            let filename = if name.is_empty() {
                "noname.txt".to_string()
            } else {
                name
            };
            let encoded: String = filename
                .bytes()
                .map(|b| match b {
                    b'A'..=b'Z'
                    | b'a'..=b'z'
                    | b'0'..=b'9'
                    | b'-'
                    | b'_'
                    | b'.'
                    | b'!'
                    | b'~'
                    | b'*'
                    | b'\''
                    | b'('
                    | b')' => (b as char).to_string(),
                    _ => format!("%{b:02X}"),
                })
                .collect();
            (
                [
                    (
                        header::CONTENT_DISPOSITION,
                        format!("attachment; filename={encoded}"),
                    ),
                    (header::CONNECTION, "close".into()),
                ],
                gcode,
            )
                .into_response()
        }
        Err(r) => r,
    }
}

pub async fn controllers(State(s): State<AppRef>) -> Response {
    let mut list = Vec::new();
    for h in s.services.registry.all() {
        let (tx, rx) = oneshot::channel();
        if h.send(ControllerMsg::Status(tx)) {
            if let Ok(status) = rx.await {
                list.push(status);
            }
        }
    }
    ok(Value::Array(list))
}

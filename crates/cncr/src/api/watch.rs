//! `/api/watch/*`: the watch directory.

use axum::body::{Body, Bytes};
use axum::extract::{Query, State};
use axum::http::{header, HeaderMap};
use axum::response::Response;
use cncr_core::monitor::{Monitor, MonitorError};
use futures_util::StreamExt;
use serde::Deserialize;
use serde_json::{json, Map, Value};
use tokio::io::AsyncWriteExt;

use super::{err, ok, AppRef, MAX_BODY};
use crate::body_object;

#[derive(Debug, Deserialize, Default)]
pub struct WatchQuery {
    path: Option<String>,
    file: Option<String>,
    to: Option<String>,
}

/// `req.body.<k> || req.query.<k> || ''`.
fn param(body: &Map<String, Value>, query: Option<&String>, k: &str) -> String {
    body.get(k)
        .and_then(Value::as_str)
        .filter(|s| !s.is_empty())
        .map(str::to_string)
        .or_else(|| query.filter(|s| !s.is_empty()).cloned())
        .unwrap_or_default()
}

fn monitor_err(e: MonitorError) -> Response {
    err(e.status(), e.to_string())
}

pub async fn status(State(s): State<AppRef>) -> Response {
    ok(json!({ "configured": s.services.monitor.is_configured() }))
}

pub async fn files(State(s): State<AppRef>, Query(q): Query<WatchQuery>, body: Bytes) -> Response {
    let path = param(&body_object(&body), q.path.as_ref(), "path");
    let monitor = s.services.monitor.clone();
    let p = path.clone();
    let files = tokio::task::spawn_blocking(move || monitor.get_files(&p))
        .await
        .unwrap_or_default();
    ok(json!({ "path": path, "files": files }))
}

pub async fn read_file(
    State(s): State<AppRef>,
    Query(q): Query<WatchQuery>,
    body: Bytes,
) -> Response {
    let file = param(&body_object(&body), q.file.as_ref(), "file");
    let monitor = s.services.monitor.clone();
    let f = file.clone();
    match tokio::task::spawn_blocking(move || monitor.read_file(&f)).await {
        Ok(Ok(data)) => ok(json!({ "file": file, "data": data })),
        Ok(Err(MonitorError::NotFound)) => err(404, "File not found"),
        Ok(Err(e @ MonitorError::InvalidName)) => monitor_err(e),
        _ => err(500, "Failed reading file"),
    }
}

/// JSON `{file, data}` or a raw `application/octet-stream` body with `?file=`
/// (streamed to a temporary file, renamed into place when complete).
pub async fn write_file(
    State(s): State<AppRef>,
    Query(q): Query<WatchQuery>,
    headers: HeaderMap,
    body: Body,
) -> Response {
    let ctype = headers
        .get(header::CONTENT_TYPE)
        .and_then(|v| v.to_str().ok())
        .unwrap_or_default();
    let mime = ctype
        .split(';')
        .next()
        .unwrap_or_default()
        .trim()
        .to_ascii_lowercase();
    let monitor = &s.services.monitor;

    if mime == "application/octet-stream" {
        let file = q.file.unwrap_or_default();
        if file.is_empty() {
            return err(400, "No file specified");
        }
        let (temp, target) = match monitor.upload_paths(&file) {
            Ok(p) => p,
            Err(e) => return monitor_err(e),
        };
        let result: Result<(), (u16, String)> = async {
            let mut out = tokio::fs::File::create(&temp)
                .await
                .map_err(|e| (500, e.to_string()))?;
            let mut stream = body.into_data_stream();
            let mut size = 0usize;
            while let Some(chunk) = stream.next().await {
                let chunk = chunk.map_err(|e| (500, e.to_string()))?;
                size += chunk.len();
                if size > MAX_BODY {
                    return Err((413, "Payload too large".into()));
                }
                out.write_all(&chunk)
                    .await
                    .map_err(|e| (500, e.to_string()))?;
            }
            out.flush().await.map_err(|e| (500, e.to_string()))?;
            Ok(())
        }
        .await;
        if let Err((status, msg)) = result {
            let _ = tokio::fs::remove_file(&temp).await;
            return err(status, msg);
        }
        return match Monitor::commit_upload(&temp, &target) {
            Ok(()) => ok(json!({ "file": file })),
            Err(e) => monitor_err(e),
        };
    }

    if mime != "application/json" {
        return err(415, "Unsupported content type");
    }
    let bytes = match axum::body::to_bytes(body, MAX_BODY).await {
        Ok(b) => b,
        Err(_) => return err(413, "Payload too large"),
    };
    let b = body_object(&bytes);
    let file = b
        .get("file")
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_string();
    if file.is_empty() {
        return err(400, "No file specified");
    }
    let data = b.get("data").and_then(Value::as_str).unwrap_or_default();
    match monitor.write_file(&file, data.as_bytes()) {
        Ok(()) => ok(json!({ "file": file })),
        Err(e) => monitor_err(e),
    }
}

pub async fn rename_file(
    State(s): State<AppRef>,
    Query(q): Query<WatchQuery>,
    body: Bytes,
) -> Response {
    let b = body_object(&body);
    let (file, to) = (
        param(&b, q.file.as_ref(), "file"),
        param(&b, q.to.as_ref(), "to"),
    );
    if file.is_empty() || to.is_empty() {
        return err(400, "No file specified");
    }
    match s.services.monitor.rename_file(&file, &to) {
        Ok(()) => ok(json!({ "file": to })),
        Err(e) => monitor_err(e),
    }
}

pub async fn delete_file(
    State(s): State<AppRef>,
    Query(q): Query<WatchQuery>,
    body: Bytes,
) -> Response {
    let file = param(&body_object(&body), q.file.as_ref(), "file");
    if file.is_empty() {
        return err(400, "No file specified");
    }
    match s.services.monitor.delete_file(&file) {
        Ok(()) => ok(json!({ "file": file })),
        Err(e) => monitor_err(e),
    }
}

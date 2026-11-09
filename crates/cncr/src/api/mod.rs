//! REST API (`/api/*`) and the WebSocket endpoint, with cncjs access rules:
//! every request must come from an allowed address, and everything except
//! `/api/signin` needs a valid access token (`Authorization: Bearer` or `?token=`).

mod gcode;
mod records;
mod state;
mod users;
mod watch;

use std::net::SocketAddr;
use std::sync::Arc;

use axum::extract::{ConnectInfo, DefaultBodyLimit, Query, Request, State, WebSocketUpgrade};
use axum::http::{header, StatusCode};
use axum::middleware::{self, Next};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use serde_json::{json, Value};
use tracing::warn;

use crate::auth::{ip_allowed, validate_user, verify_token};
use crate::AppState;

pub type AppRef = Arc<AppState>;

/// Upload / JSON body limit (cncjs: 256 MB).
pub const MAX_BODY: usize = 256 * 1024 * 1024;

/// JSON error response `{msg}`.
pub fn err(status: u16, msg: impl Into<String>) -> Response {
    let code = StatusCode::from_u16(status).unwrap_or(StatusCode::INTERNAL_SERVER_ERROR);
    (code, Json(json!({ "msg": msg.into() }))).into_response()
}

pub fn ok(v: Value) -> Response {
    Json(v).into_response()
}

/// `?paging=1&page=&pageLength=` support shared by the record collections.
#[derive(Debug, Default, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Paging {
    pub paging: Option<String>,
    pub page: Option<String>,
    pub page_length: Option<String>,
}

impl Paging {
    /// `{records}` or `{pagination, records}` like cncjs.
    pub fn respond(&self, records: Vec<Value>) -> Response {
        if self
            .paging
            .as_deref()
            .is_none_or(|p| p.is_empty() || p == "false" || p == "0")
        {
            return ok(json!({ "records": records }));
        }
        let total = records.len();
        let mut page = self
            .page
            .as_deref()
            .and_then(|p| p.parse::<usize>().ok())
            .filter(|p| *p >= 1)
            .unwrap_or(1);
        let len = self
            .page_length
            .as_deref()
            .and_then(|p| p.parse::<usize>().ok())
            .filter(|p| *p >= 1)
            .unwrap_or(10);
        if (page - 1) * len >= total {
            page = total.div_ceil(len).max(1);
        }
        let begin = ((page - 1) * len).min(total);
        let end = (begin + len).min(total);
        ok(json!({
            "pagination": { "page": page, "pageLength": len, "totalRecords": total },
            "records": records[begin..end],
        }))
    }
}

pub fn router(state: AppRef) -> Router {
    let api = Router::new()
        .route("/api/signin", post(users::signin))
        .route(
            "/api/state",
            get(state::get_state)
                .post(state::set_state)
                .delete(state::unset_state),
        )
        .route("/api/tool", get(state::get_tool).post(state::set_tool))
        .route("/api/gcode", get(gcode::fetch).post(gcode::upload))
        .route(
            "/api/gcode/download",
            get(gcode::download).post(gcode::download),
        )
        .route("/api/controllers", get(gcode::controllers))
        .route(
            "/api/commands",
            get(records::commands_fetch).post(records::commands_create),
        )
        .route(
            "/api/commands/{id}",
            get(records::commands_read)
                .put(records::commands_update)
                .delete(records::commands_delete),
        )
        .route("/api/commands/run/{id}", post(records::commands_run))
        .route(
            "/api/events",
            get(records::events_fetch).post(records::events_create),
        )
        .route("/api/events/", post(records::events_create))
        .route(
            "/api/events/{id}",
            get(records::events_read)
                .put(records::events_update)
                .delete(records::events_delete),
        )
        .route(
            "/api/machines",
            get(records::machines_fetch).post(records::machines_create),
        )
        .route(
            "/api/machines/{id}",
            get(records::machines_read)
                .put(records::machines_update)
                .delete(records::machines_delete),
        )
        .route(
            "/api/macros",
            get(records::macros_fetch).post(records::macros_create),
        )
        .route(
            "/api/macros/{id}",
            get(records::macros_read)
                .put(records::macros_update)
                .delete(records::macros_delete),
        )
        .route(
            "/api/mdi",
            get(records::mdi_fetch)
                .post(records::mdi_create)
                .put(records::mdi_bulk_update),
        )
        .route(
            "/api/mdi/{id}",
            get(records::mdi_read)
                .put(records::mdi_update)
                .delete(records::mdi_delete),
        )
        .route("/api/users", get(users::fetch).post(users::create))
        .route("/api/users/", post(users::create))
        .route(
            "/api/users/{id}",
            get(users::read).put(users::update).delete(users::delete),
        )
        .route("/api/watch/status", get(watch::status))
        .route("/api/watch/files", get(watch::files).post(watch::files))
        .route(
            "/api/watch/file",
            get(watch::read_file)
                .post(watch::read_file)
                .put(watch::write_file)
                .delete(watch::delete_file),
        )
        .route("/api/watch/file/rename", post(watch::rename_file))
        .route("/ws", get(websocket))
        .layer(middleware::from_fn_with_state(
            Arc::clone(&state),
            require_token,
        ));

    api.layer(middleware::from_fn_with_state(
        Arc::clone(&state),
        require_address,
    ))
    .layer(DefaultBodyLimit::max(MAX_BODY))
    .with_state(state)
}

async fn require_address(
    State(state): State<AppRef>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    req: Request,
    next: Next,
) -> Response {
    if !ip_allowed(addr.ip(), state.allow_remote_access) {
        warn!("Forbidden: unauthorized IP address {}", addr.ip());
        return (StatusCode::FORBIDDEN, "Forbidden Access").into_response();
    }
    next.run(req).await
}

#[derive(serde::Deserialize, Default)]
struct TokenQuery {
    token: Option<String>,
}

fn request_token(req: &Request) -> Option<String> {
    if let Some(h) = req
        .headers()
        .get(header::AUTHORIZATION)
        .and_then(|v| v.to_str().ok())
    {
        if let Some(t) = h
            .strip_prefix("Bearer ")
            .or_else(|| h.strip_prefix("bearer "))
        {
            return Some(t.trim().to_string());
        }
    }
    let q = req.uri().query().unwrap_or_default();
    serde_urlencoded_token(q)
}

fn serde_urlencoded_token(q: &str) -> Option<String> {
    Query::<TokenQuery>::try_from_uri(&format!("/?{q}").parse().ok()?)
        .ok()
        .and_then(|Query(t)| t.token)
}

async fn require_token(State(state): State<AppRef>, req: Request, next: Next) -> Response {
    if req.uri().path() == "/api/signin" {
        return next.run(req).await;
    }
    let authorized = request_token(&req)
        .and_then(|t| verify_token(&state.secret, &t))
        .is_some_and(|claims| validate_user(&state.services.config, &claims));
    if !authorized {
        warn!(
            "Forbidden: {} {} (missing or invalid access token)",
            req.method(),
            req.uri().path()
        );
        return (StatusCode::FORBIDDEN, "Forbidden Access").into_response();
    }
    next.run(req).await
}

async fn websocket(
    State(state): State<AppRef>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    upgrade: WebSocketUpgrade,
) -> Response {
    upgrade.on_upgrade(move |ws| crate::engine::serve(state, ws, addr.to_string()))
}

//! `/api/signin` and `/api/users`.

use axum::body::Bytes;
use axum::extract::{Path, Query, State};
use axum::response::Response;
use serde_json::{json, Map, Value};
use tracing::debug;

use super::{err, ok, AppRef, Paging};
use crate::auth::{enabled_users, issue_token, verify_token};
use crate::body_object;
use cncr_core::util::now_ms;

const USERS: &str = "users";
const FIELDS: [&str; 4] = ["id", "mtime", "enabled", "name"];

fn str_of<'a>(m: &'a Map<String, Value>, k: &str) -> &'a str {
    m.get(k).and_then(Value::as_str).unwrap_or_default()
}

/// Users with ids assigned and `enabled` defaulted (persisted silently when ids were missing).
fn sanitized(s: &AppRef) -> Vec<Map<String, Value>> {
    let config = &s.services.config;
    let mut changed = false;
    let records: Vec<Map<String, Value>> = config
        .get_array(USERS)
        .into_iter()
        .map(|u| {
            let mut u = match u {
                Value::Object(m) => m,
                _ => Map::new(),
            };
            if str_of(&u, "id").is_empty() {
                u.insert("id".into(), json!(uuid::Uuid::new_v4().to_string()));
                changed = true;
            }
            u.entry("enabled").or_insert(json!(true));
            u
        })
        .collect();
    if changed {
        config.set_silent(
            USERS,
            Value::Array(records.iter().cloned().map(Value::Object).collect()),
        );
    }
    records
}

fn save(s: &AppRef, records: &[Map<String, Value>]) {
    s.services.config.set(
        USERS,
        Value::Array(records.iter().cloned().map(Value::Object).collect()),
    );
}

fn public(u: &Map<String, Value>) -> Value {
    Value::Object(
        FIELDS
            .iter()
            .filter_map(|k| u.get(*k).map(|v| ((*k).to_string(), v.clone())))
            .collect(),
    )
}

/// bcrypt cost 10 (cncjs `genSaltSync` default); hashing runs off the async workers.
async fn hash(password: &str) -> Result<String, Response> {
    let password = password.trim().to_string();
    tokio::task::spawn_blocking(move || bcrypt::hash(password, 10))
        .await
        .map_err(|e| e.to_string())
        .and_then(|r| r.map_err(|e| e.to_string()))
        .map_err(|e| err(500, format!("Failed to hash password: {e}")))
}

async fn verify(password: &str, hash: &str) -> bool {
    let (password, hash) = (password.to_string(), hash.to_string());
    tokio::task::spawn_blocking(move || bcrypt::verify(password, &hash).unwrap_or(false))
        .await
        .unwrap_or(false)
}

pub async fn signin(State(s): State<AppRef>, body: Bytes) -> Response {
    let b = body_object(&body);
    let (token, name, password) = (
        str_of(&b, "token"),
        str_of(&b, "name"),
        str_of(&b, "password"),
    );
    sanitized(&s);
    let users = enabled_users(&s.services.config);

    if users.is_empty() {
        let token = issue_token(&s.secret, s.token_lifetime_secs, "", "");
        return ok(json!({ "enabled": false, "token": token, "name": "" }));
    }

    if token.is_empty() {
        let Some(user) = users.iter().find(|u| str_of(u, "name") == name) else {
            return err(401, "Authentication failed");
        };
        if !verify(password, str_of(user, "password")).await {
            return err(401, "Authentication failed");
        }
        let token = issue_token(
            &s.secret,
            s.token_lifetime_secs,
            str_of(user, "id"),
            str_of(user, "name"),
        );
        return ok(json!({ "enabled": true, "token": token, "name": str_of(user, "name") }));
    }

    let Some(claims) = verify_token(&s.secret, token) else {
        return err(500, "Internal server error");
    };
    debug!(
        "jwt.verify: user.id={}, user.name={}",
        claims.id, claims.name
    );
    match users
        .iter()
        .find(|u| str_of(u, "id") == claims.id && str_of(u, "name") == claims.name)
    {
        Some(user) => ok(json!({ "enabled": true, "token": token, "name": str_of(user, "name") })),
        None => err(401, "Authentication failed"),
    }
}

pub async fn fetch(State(s): State<AppRef>, Query(p): Query<Paging>) -> Response {
    p.respond(sanitized(&s).iter().map(public).collect())
}

pub async fn create(State(s): State<AppRef>, body: Bytes) -> Response {
    let b = body_object(&body);
    let (name, password) = (str_of(&b, "name"), str_of(&b, "password"));
    if name.is_empty() {
        return err(400, "The \"name\" parameter must not be empty");
    }
    if password.is_empty() {
        return err(400, "The \"password\" parameter must not be empty");
    }
    let mut records = sanitized(&s);
    if records.iter().any(|u| str_of(u, "name") == name) {
        return err(409, "The specified user already exists");
    }
    let hashed = match hash(password).await {
        Ok(h) => h,
        Err(r) => return r,
    };
    let id = uuid::Uuid::new_v4().to_string();
    let mtime = now_ms();
    let enabled = b.get("enabled").and_then(Value::as_bool).unwrap_or(true);
    let record =
        json!({ "id": id, "mtime": mtime, "enabled": enabled, "name": name, "password": hashed });
    records.push(record.as_object().cloned().unwrap_or_default());
    save(&s, &records);
    ok(json!({ "id": id, "mtime": mtime }))
}

pub async fn read(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    match sanitized(&s).iter().find(|u| str_of(u, "id") == id) {
        Some(u) => ok(public(u)),
        None => err(404, "Not found"),
    }
}

pub async fn update(State(s): State<AppRef>, Path(id): Path<String>, body: Bytes) -> Response {
    let b = body_object(&body);
    let mut records = sanitized(&s);
    let Some(idx) = records.iter().position(|u| str_of(u, "id") == id) else {
        return err(404, "Not found");
    };
    let record = &records[idx];
    let name = b
        .get("name")
        .and_then(Value::as_str)
        .unwrap_or(str_of(record, "name"))
        .to_string();
    let enabled = b
        .get("enabled")
        .and_then(Value::as_bool)
        .unwrap_or_else(|| {
            record
                .get("enabled")
                .and_then(Value::as_bool)
                .unwrap_or(true)
        });
    let (old_pw, new_pw) = (str_of(&b, "oldPassword"), str_of(&b, "newPassword"));
    let change_pw = !old_pw.is_empty() && !new_pw.is_empty();
    if change_pw && !verify(old_pw, str_of(record, "password")).await {
        return err(412, "Incorrect password");
    }
    if records
        .iter()
        .any(|u| str_of(u, "id") != id && str_of(u, "name") == name)
    {
        return err(409, "The specified user already exists");
    }
    let new_hash = if change_pw {
        match hash(new_pw).await {
            Ok(h) => Some(h),
            Err(r) => return r,
        }
    } else {
        None
    };
    let mtime = now_ms();
    let record = &mut records[idx];
    record.insert("mtime".into(), json!(mtime));
    record.insert("enabled".into(), json!(enabled));
    record.insert("name".into(), json!(name));
    if let Some(h) = new_hash {
        record.insert("password".into(), json!(h));
    }
    save(&s, &records);
    ok(json!({ "id": id, "mtime": mtime }))
}

pub async fn delete(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    let records = sanitized(&s);
    if !records.iter().any(|u| str_of(u, "id") == id) {
        return err(404, "Not found");
    }
    let kept: Vec<_> = records
        .into_iter()
        .filter(|u| str_of(u, "id") != id)
        .collect();
    save(&s, &kept);
    ok(json!({ "id": id }))
}

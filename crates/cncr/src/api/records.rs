//! Config-backed record collections: commands, events, macros, mdi, machines.

use axum::body::Bytes;
use axum::extract::{Path, Query, State};
use axum::response::Response;
use serde_json::{json, Map, Value};
use tracing::info;

use super::{err, ok, AppRef, Paging};
use crate::body_object;
use cncr_core::util::{now_ms, num_or_zero};

type Record = Map<String, Value>;

/// JS `String(x || '')`.
fn js_string(v: Option<&Value>) -> String {
    match v {
        None | Some(Value::Null) | Some(Value::Bool(false)) => String::new(),
        Some(Value::String(s)) => s.clone(),
        Some(Value::Number(n)) if n.as_f64() == Some(0.0) => String::new(),
        Some(v) => v.to_string(),
    }
}

/// JS truthiness of a JSON value.
fn truthy(v: Option<&Value>) -> bool {
    match v {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(Value::Number(n)) => n.as_f64().is_some_and(|f| f != 0.0 && !f.is_nan()),
        Some(Value::String(s)) => !s.is_empty(),
        Some(_) => true,
    }
}

fn new_id() -> String {
    uuid::Uuid::new_v4().to_string()
}

/// Records of `key` with missing ids assigned (persisted silently, like cncjs) and
/// `fix` applied to each record.
fn sanitized(state: &AppRef, key: &str, fix: fn(&mut Record)) -> Vec<Record> {
    let config = &state.services.config;
    let mut changed = false;
    let records: Vec<Record> = config
        .get_array(key)
        .into_iter()
        .map(|r| {
            let mut r = match r {
                Value::Object(m) => m,
                _ => Map::new(),
            };
            if !truthy(r.get("id")) {
                r.insert("id".into(), json!(new_id()));
                changed = true;
            }
            fix(&mut r);
            r
        })
        .collect();
    if changed {
        config.set_silent(key, records_json(&records));
    }
    records
}

fn records_json(records: &[Record]) -> Value {
    Value::Array(records.iter().cloned().map(Value::Object).collect())
}

fn save(state: &AppRef, key: &str, records: &[Record]) {
    state.services.config.set(key, records_json(records));
}

/// `{id, ...fields}` with missing fields omitted (JS drops `undefined`).
fn project(r: &Record, fields: &[&str]) -> Value {
    let mut out = Map::new();
    for f in std::iter::once(&"id").chain(fields) {
        if let Some(v) = r.get(*f) {
            out.insert((*f).to_string(), v.clone());
        }
    }
    Value::Object(out)
}

fn find<'a>(records: &'a mut [Record], id: &str) -> Option<&'a mut Record> {
    records
        .iter_mut()
        .find(|r| r.get("id").and_then(Value::as_str) == Some(id))
}

fn fetch(
    state: &AppRef,
    key: &str,
    fix: fn(&mut Record),
    fields: &[&str],
    paging: &Paging,
) -> Response {
    let records = sanitized(state, key, fix);
    paging.respond(records.iter().map(|r| project(r, fields)).collect())
}

fn read(state: &AppRef, key: &str, fix: fn(&mut Record), fields: &[&str], id: &str) -> Response {
    let mut records = sanitized(state, key, fix);
    match find(&mut records, id) {
        Some(r) => ok(project(r, fields)),
        None => err(404, "Not found"),
    }
}

fn delete(state: &AppRef, key: &str, fix: fn(&mut Record), id: &str) -> Response {
    let records = sanitized(state, key, fix);
    if !records
        .iter()
        .any(|r| r.get("id").and_then(Value::as_str) == Some(id))
    {
        return err(404, "Not found");
    }
    let kept: Vec<Record> = records
        .into_iter()
        .filter(|r| r.get("id").and_then(Value::as_str) != Some(id))
        .collect();
    save(state, key, &kept);
    ok(json!({ "id": id }))
}

fn require(body: &Record, names: &[&str]) -> Result<(), Response> {
    for n in names {
        if !truthy(body.get(*n)) {
            return Err(err(400, format!("The \"{n}\" parameter must not be empty")));
        }
    }
    Ok(())
}

/// `field = body.field ?? record.field` (JS destructuring defaults: only `undefined`
/// falls back), then `String(field || '')`.
fn merged_string(body: &Record, record: &Record, k: &str) -> String {
    js_string(body.get(k).or_else(|| record.get(k)))
}

fn fix_enabled_commands(r: &mut Record) {
    if !r.contains_key("enabled") {
        r.insert("enabled".into(), json!(true));
    }
    if !truthy(r.get("commands")) {
        let c = js_string(r.get("command"));
        r.insert("commands".into(), json!(c));
        r.remove("command");
    }
}

fn fix_none(_: &mut Record) {}

// ---------------------------------------------------------------- commands

const COMMANDS: &str = "commands";
const COMMAND_FIELDS: &[&str] = &["mtime", "enabled", "title", "commands"];

pub async fn commands_fetch(State(s): State<AppRef>, Query(p): Query<Paging>) -> Response {
    fetch(&s, COMMANDS, fix_enabled_commands, COMMAND_FIELDS, &p)
}

pub async fn commands_create(State(s): State<AppRef>, body: Bytes) -> Response {
    let b = body_object(&body);
    if let Err(r) = require(&b, &["title", "commands"]) {
        return r;
    }
    let mut records = sanitized(&s, COMMANDS, fix_enabled_commands);
    let record = json!({
        "id": new_id(), "mtime": now_ms(),
        "enabled": b.get("enabled").is_none_or(|v| truthy(Some(v))),
        "title": b["title"], "commands": b["commands"],
    });
    let reply = json!({ "id": record["id"], "mtime": record["mtime"] });
    records.push(record.as_object().cloned().unwrap_or_default());
    save(&s, COMMANDS, &records);
    ok(reply)
}

pub async fn commands_read(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    read(&s, COMMANDS, fix_enabled_commands, COMMAND_FIELDS, &id)
}

pub async fn commands_update(
    State(s): State<AppRef>,
    Path(id): Path<String>,
    body: Bytes,
) -> Response {
    let b = body_object(&body);
    let mut records = sanitized(&s, COMMANDS, fix_enabled_commands);
    let Some(r) = find(&mut records, &id) else {
        return err(404, "Not found");
    };
    let enabled = truthy(b.get("enabled").or_else(|| r.get("enabled")));
    let title = merged_string(&b, r, "title");
    let commands = merged_string(&b, r, "commands");
    let mtime = now_ms();
    r.insert("mtime".into(), json!(mtime));
    r.insert("enabled".into(), json!(enabled));
    r.insert("title".into(), json!(title));
    r.insert("commands".into(), json!(commands));
    r.remove("command");
    save(&s, COMMANDS, &records);
    ok(json!({ "id": id, "mtime": mtime }))
}

pub async fn commands_delete(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    delete(&s, COMMANDS, fix_enabled_commands, &id)
}

pub async fn commands_run(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    let mut records = sanitized(&s, COMMANDS, fix_enabled_commands);
    let Some(r) = find(&mut records, &id) else {
        return err(404, "Not found");
    };
    let (title, commands) = (js_string(r.get("title")), js_string(r.get("commands")));
    info!("run: title={title:?}, commands={commands:?}");
    let task_id = s.services.tasks.run(&commands, &title);
    ok(json!({ "taskId": task_id }))
}

// ---------------------------------------------------------------- events

const EVENTS: &str = "events";
const EVENT_FIELDS: &[&str] = &["mtime", "enabled", "event", "trigger", "commands"];

pub async fn events_fetch(State(s): State<AppRef>, Query(p): Query<Paging>) -> Response {
    fetch(&s, EVENTS, fix_enabled_commands, EVENT_FIELDS, &p)
}

pub async fn events_create(State(s): State<AppRef>, body: Bytes) -> Response {
    let b = body_object(&body);
    if let Err(r) = require(&b, &["event", "trigger", "commands"]) {
        return r;
    }
    let mut records = sanitized(&s, EVENTS, fix_enabled_commands);
    let record = json!({
        "id": new_id(), "mtime": now_ms(),
        "enabled": b.get("enabled").is_none_or(|v| truthy(Some(v))),
        "event": b["event"], "trigger": b["trigger"], "commands": b["commands"],
    });
    let reply = json!({ "id": record["id"], "mtime": record["mtime"] });
    records.push(record.as_object().cloned().unwrap_or_default());
    save(&s, EVENTS, &records);
    ok(reply)
}

pub async fn events_read(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    read(&s, EVENTS, fix_enabled_commands, EVENT_FIELDS, &id)
}

pub async fn events_update(
    State(s): State<AppRef>,
    Path(id): Path<String>,
    body: Bytes,
) -> Response {
    let b = body_object(&body);
    let mut records = sanitized(&s, EVENTS, fix_enabled_commands);
    let Some(r) = find(&mut records, &id) else {
        return err(404, "Not found");
    };
    let enabled = truthy(b.get("enabled").or_else(|| r.get("enabled")));
    let values: Vec<(&str, String)> = ["event", "trigger", "commands"]
        .iter()
        .map(|k| (*k, merged_string(&b, r, k)))
        .collect();
    let mtime = now_ms();
    r.insert("mtime".into(), json!(mtime));
    r.insert("enabled".into(), json!(enabled));
    for (k, v) in values {
        r.insert(k.into(), json!(v));
    }
    r.remove("command");
    save(&s, EVENTS, &records);
    ok(json!({ "id": id, "mtime": mtime }))
}

pub async fn events_delete(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    delete(&s, EVENTS, fix_enabled_commands, &id)
}

// ---------------------------------------------------------------- macros

const MACROS: &str = "macros";
const MACRO_FIELDS: &[&str] = &["mtime", "name", "content"];

pub async fn macros_fetch(State(s): State<AppRef>, Query(p): Query<Paging>) -> Response {
    fetch(&s, MACROS, fix_none, MACRO_FIELDS, &p)
}

pub async fn macros_create(State(s): State<AppRef>, body: Bytes) -> Response {
    let b = body_object(&body);
    if let Err(r) = require(&b, &["name", "content"]) {
        return r;
    }
    let mut records = sanitized(&s, MACROS, fix_none);
    let record =
        json!({ "id": new_id(), "mtime": now_ms(), "name": b["name"], "content": b["content"] });
    let reply = json!({ "id": record["id"], "mtime": record["mtime"] });
    records.push(record.as_object().cloned().unwrap_or_default());
    save(&s, MACROS, &records);
    ok(reply)
}

pub async fn macros_read(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    read(&s, MACROS, fix_none, MACRO_FIELDS, &id)
}

pub async fn macros_update(
    State(s): State<AppRef>,
    Path(id): Path<String>,
    body: Bytes,
) -> Response {
    let b = body_object(&body);
    let mut records = sanitized(&s, MACROS, fix_none);
    let Some(r) = find(&mut records, &id) else {
        return err(404, "Not found");
    };
    let (name, content) = (
        merged_string(&b, r, "name"),
        merged_string(&b, r, "content"),
    );
    if name.is_empty() {
        return err(400, "The \"name\" parameter must not be empty");
    }
    if content.is_empty() {
        return err(400, "The \"content\" parameter must not be empty");
    }
    let mtime = now_ms();
    r.insert("mtime".into(), json!(mtime));
    r.insert("name".into(), json!(name));
    r.insert("content".into(), json!(content));
    save(&s, MACROS, &records);
    ok(json!({ "id": id, "mtime": mtime }))
}

pub async fn macros_delete(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    delete(&s, MACROS, fix_none, &id)
}

// ---------------------------------------------------------------- mdi

const MDI: &str = "mdi";
const MDI_FIELDS: &[&str] = &["name", "command", "grid"];

fn fix_grid(r: &mut Record) {
    if !r.get("grid").is_some_and(Value::is_object) {
        r.insert("grid".into(), json!({}));
    }
}

pub async fn mdi_fetch(State(s): State<AppRef>, Query(p): Query<Paging>) -> Response {
    fetch(&s, MDI, fix_grid, MDI_FIELDS, &p)
}

pub async fn mdi_create(State(s): State<AppRef>, body: Bytes) -> Response {
    let b = body_object(&body);
    if let Err(r) = require(&b, &["name", "command"]) {
        return r;
    }
    let mut records = sanitized(&s, MDI, fix_grid);
    let grid = b.get("grid").cloned().unwrap_or_else(|| json!({}));
    let id = new_id();
    records.push(
        json!({ "id": id, "name": b["name"], "command": b["command"], "grid": grid })
            .as_object()
            .cloned()
            .unwrap_or_default(),
    );
    save(&s, MDI, &records);
    ok(json!({ "id": id }))
}

pub async fn mdi_read(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    read(&s, MDI, fix_grid, MDI_FIELDS, &id)
}

pub async fn mdi_update(State(s): State<AppRef>, Path(id): Path<String>, body: Bytes) -> Response {
    let b = body_object(&body);
    let mut records = sanitized(&s, MDI, fix_grid);
    let Some(r) = find(&mut records, &id) else {
        return err(404, "Not found");
    };
    let (name, command) = (
        merged_string(&b, r, "name"),
        merged_string(&b, r, "command"),
    );
    if name.is_empty() {
        return err(400, "The \"name\" parameter must not be empty");
    }
    if command.is_empty() {
        return err(400, "The \"command\" parameter must not be empty");
    }
    let grid = b
        .get("grid")
        .or_else(|| r.get("grid"))
        .filter(|g| g.is_object())
        .cloned()
        .unwrap_or_else(|| json!({}));
    r.insert("name".into(), json!(name));
    r.insert("command".into(), json!(command));
    r.insert("grid".into(), grid);
    save(&s, MDI, &records);
    ok(json!({ "id": id }))
}

pub async fn mdi_bulk_update(State(s): State<AppRef>, body: Bytes) -> Response {
    let b = body_object(&body);
    let Some(list) = b.get("records").filter(|r| truthy(Some(r))) else {
        return err(400, "The \"records\" parameter must not be empty");
    };
    let items: Vec<Value> = match list {
        Value::Array(a) => a.clone(),
        v => vec![v.clone()],
    };
    let records: Vec<Record> = items
        .into_iter()
        .filter_map(|r| match r {
            Value::Object(m) => Some(m),
            _ => None,
        })
        .map(|mut r| {
            if !truthy(r.get("id")) {
                r.insert("id".into(), json!(new_id()));
            }
            let (name, command) = (js_string(r.get("name")), js_string(r.get("command")));
            r.insert("name".into(), json!(name));
            r.insert("command".into(), json!(command));
            fix_grid(&mut r);
            r
        })
        .collect();
    save(&s, MDI, &records);
    ok(json!({ "err": null }))
}

pub async fn mdi_delete(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    delete(&s, MDI, fix_grid, &id)
}

// ---------------------------------------------------------------- machines

const MACHINES: &str = "machines";
const LIMITS: [&str; 6] = ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"];

fn machine_profile(r: &Record) -> Value {
    let limits = r.get("limits");
    let mut l = Map::new();
    for k in LIMITS {
        l.insert(k.into(), json!(num_or_zero(limits.and_then(|v| v.get(k)))));
    }
    json!({ "id": r.get("id"), "name": js_string(r.get("name")), "limits": l })
}

pub async fn machines_fetch(State(s): State<AppRef>, Query(p): Query<Paging>) -> Response {
    let records = sanitized(&s, MACHINES, fix_none);
    p.respond(records.iter().map(machine_profile).collect())
}

pub async fn machines_create(State(s): State<AppRef>, body: Bytes) -> Response {
    let mut b = body_object(&body);
    if let Err(r) = require(&b, &["name"]) {
        return r;
    }
    let mut records = sanitized(&s, MACHINES, fix_none);
    let id = new_id();
    b.insert("id".into(), json!(id));
    records.push(machine_profile(&b).as_object().cloned().unwrap_or_default());
    save(&s, MACHINES, &records);
    ok(json!({ "id": id }))
}

pub async fn machines_read(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    let mut records = sanitized(&s, MACHINES, fix_none);
    match find(&mut records, &id) {
        Some(r) => ok(machine_profile(r)),
        None => err(404, "Not found"),
    }
}

pub async fn machines_update(
    State(s): State<AppRef>,
    Path(id): Path<String>,
    body: Bytes,
) -> Response {
    let b = body_object(&body);
    let mut records = sanitized(&s, MACHINES, fix_none);
    let Some(r) = find(&mut records, &id) else {
        return err(404, "Not found");
    };
    if let Some(name) = b.get("name") {
        r.insert("name".into(), json!(js_string(Some(name))));
    }
    if let Some(Value::Object(limits)) = b.get("limits") {
        let entry = r.entry("limits").or_insert_with(|| json!({}));
        if !entry.is_object() {
            *entry = json!({});
        }
        for k in LIMITS {
            if let Some(v) = limits.get(k) {
                entry[k] = json!(num_or_zero(Some(v)));
            }
        }
    }
    save(&s, MACHINES, &records);
    ok(json!({ "id": id }))
}

pub async fn machines_delete(State(s): State<AppRef>, Path(id): Path<String>) -> Response {
    delete(&s, MACHINES, fix_none, &id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn js_coercions() {
        assert_eq!(js_string(Some(&json!(0))), "");
        assert_eq!(js_string(Some(&json!(12))), "12");
        assert_eq!(js_string(None), "");
        assert!(truthy(Some(&json!("x"))) && !truthy(Some(&json!(""))) && !truthy(Some(&json!(0))));
    }
}

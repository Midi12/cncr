//! `/api/state` (client UI state in the config) and `/api/tool` (tool change settings).

use axum::body::Bytes;
use axum::extract::{Query, State};
use axum::response::Response;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{err, ok, AppRef};

#[derive(Debug, Deserialize, Default)]
pub struct KeyQuery {
    key: Option<String>,
}

/// Leaf paths of a JSON object (`deep-keys`): `{a: {b: 1}, c: []}` → `a.b`, `c`.
fn deep_keys(v: &Value, prefix: &str, out: &mut Vec<(String, Value)>) {
    match v {
        Value::Object(m) if !m.is_empty() => {
            for (k, child) in m {
                let path = if prefix.is_empty() {
                    k.clone()
                } else {
                    format!("{prefix}.{k}")
                };
                deep_keys(child, &path, out);
            }
        }
        _ if !prefix.is_empty() => out.push((prefix.to_string(), v.clone())),
        _ => {}
    }
}

/// Writes every leaf of `data` under `root` (objects are merged into existing objects).
fn merge_into(s: &AppRef, root: &str, data: &Value) {
    let config = &s.services.config;
    let mut leaves = Vec::new();
    deep_keys(data, "", &mut leaves);
    for (k, new) in leaves {
        let key = format!("{root}.{k}");
        match (config.get(&key), &new) {
            (Some(Value::Object(mut old)), Value::Object(n)) => {
                old.extend(n.clone());
                config.set(&key, Value::Object(old));
            }
            _ => config.set(&key, new),
        }
    }
}

pub async fn get_state(State(s): State<AppRef>, Query(q): Query<KeyQuery>) -> Response {
    let config = &s.services.config;
    match q.key.filter(|k| !k.is_empty()) {
        None => ok(config.get("state").unwrap_or_else(|| json!({}))),
        Some(k) => match config.get(&format!("state.{k}")) {
            Some(v) => ok(v),
            None => err(404, "Not found"),
        },
    }
}

pub async fn unset_state(State(s): State<AppRef>, Query(q): Query<KeyQuery>) -> Response {
    let config = &s.services.config;
    match q.key.filter(|k| !k.is_empty()) {
        None => ok(config.get("state").unwrap_or_else(|| json!({}))),
        Some(k) => {
            let key = format!("state.{k}");
            if !config.has(&key) {
                return err(404, "Not found");
            }
            config.unset(&key);
            ok(json!({ "err": false }))
        }
    }
}

pub async fn set_state(
    State(s): State<AppRef>,
    Query(q): Query<KeyQuery>,
    body: Bytes,
) -> Response {
    let data = Value::Object(crate::body_object(&body));
    match q.key.filter(|k| !k.is_empty()) {
        Some(k) => s.services.config.set(&format!("state.{k}"), data),
        None => merge_into(&s, "state", &data),
    }
    ok(json!({ "err": false }))
}

const TOOL_KEYS: [&str; 12] = [
    "toolChangePolicy",
    "toolChangeX",
    "toolChangeY",
    "toolChangeZ",
    "toolProbeX",
    "toolProbeY",
    "toolProbeZ",
    "toolProbeCustomCommands",
    "toolProbeCommand",
    "toolProbeDistance",
    "toolProbeFeedrate",
    "touchPlateHeight",
];

pub async fn get_tool(State(s): State<AppRef>) -> Response {
    ok(s.services.config.get("tool").unwrap_or(Value::Null))
}

pub async fn set_tool(State(s): State<AppRef>, body: Bytes) -> Response {
    let data = Value::Object(crate::body_object(&body));
    let mut leaves = Vec::new();
    deep_keys(&data, "", &mut leaves);
    let invalid: Vec<&str> = leaves
        .iter()
        .map(|(k, _)| k.as_str())
        .filter(|k| !TOOL_KEYS.contains(k))
        .collect();
    if !invalid.is_empty() {
        return err(
            400,
            format!("Invalid keys specified: {}", invalid.join(", ")),
        );
    }
    merge_into(&s, "tool", &data);
    ok(json!({ "err": false }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deep_keys_lists_leaves() {
        let mut out = Vec::new();
        deep_keys(&json!({"a": {"b": 1, "c": {}}, "d": [1]}), "", &mut out);
        let keys: Vec<_> = out.iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(keys, vec!["a.b", "a.c", "d"]);
    }
}

//! `~/.cncrc` JSON configuration store with lodash-style key paths
//! (`tool.toolChangeX`, `macros[0].id`, `a["b.c"]`).

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, SystemTime};

use serde::de::DeserializeOwned;
use serde_json::{json, Map, Value};
use tracing::{error, info};

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Seg {
    Key(String),
    Index(usize),
}

/// Splits a lodash path: `a.b[0]["c.d"]` → `a`, `b`, `0`, `c.d`.
fn parse_path(path: &str) -> Vec<Seg> {
    let mut segs = Vec::new();
    let mut cur = String::new();
    let mut chars = path.chars().peekable();
    let push = |cur: &mut String, segs: &mut Vec<Seg>| {
        if !cur.is_empty() {
            let s = std::mem::take(cur);
            segs.push(s.parse::<usize>().map_or(Seg::Key(s), Seg::Index));
        }
    };
    while let Some(c) = chars.next() {
        match c {
            '.' => push(&mut cur, &mut segs),
            '[' => {
                push(&mut cur, &mut segs);
                let mut inner = String::new();
                let quote = match chars.peek() {
                    Some(&q @ ('"' | '\'')) => {
                        chars.next();
                        Some(q)
                    }
                    _ => None,
                };
                while let Some(c) = chars.next() {
                    if let Some(q) = quote {
                        if c == q && chars.peek() == Some(&']') {
                            chars.next();
                            break;
                        }
                    } else if c == ']' {
                        break;
                    }
                    inner.push(c);
                }
                match (quote, inner.parse::<usize>()) {
                    (None, Ok(i)) => segs.push(Seg::Index(i)),
                    _ => segs.push(Seg::Key(inner)),
                }
            }
            _ => cur.push(c),
        }
    }
    push(&mut cur, &mut segs);
    segs
}

fn get_path<'a>(mut v: &'a Value, path: &[Seg]) -> Option<&'a Value> {
    for seg in path {
        v = match (seg, v) {
            (Seg::Key(k), Value::Object(m)) => m.get(k)?,
            (Seg::Index(i), Value::Array(a)) => a.get(*i)?,
            (Seg::Index(i), Value::Object(m)) => m.get(&i.to_string())?,
            (Seg::Key(k), Value::Array(a)) => a.get(k.parse::<usize>().ok()?)?,
            _ => return None,
        };
    }
    Some(v)
}

/// lodash `_.set`: creates missing objects (arrays for numeric segments).
pub(crate) fn set_path(root: &mut Value, path: &[Seg], value: Value) {
    let Some((last, parents)) = path.split_last() else {
        *root = value;
        return;
    };
    let mut v = root;
    for (i, seg) in parents.iter().enumerate() {
        let next_is_index = matches!(path[i + 1], Seg::Index(_));
        let fresh = || if next_is_index { json!([]) } else { json!({}) };
        coerce_container(v, seg);
        let slot = match (seg, v) {
            (Seg::Index(idx), Value::Array(a)) => {
                if a.len() <= *idx {
                    a.resize(*idx + 1, Value::Null);
                }
                &mut a[*idx]
            }
            (seg, Value::Object(m)) => m.entry(seg_key(seg)).or_insert_with(fresh),
            _ => unreachable!("coerce_container guarantees a matching container"),
        };
        if !slot.is_object() && !slot.is_array() {
            *slot = fresh();
        }
        v = slot;
    }
    coerce_container(v, last);
    match (last, v) {
        (Seg::Index(i), Value::Array(a)) => {
            if a.len() <= *i {
                a.resize(*i + 1, Value::Null);
            }
            a[*i] = value;
        }
        (seg, Value::Object(m)) => {
            m.insert(seg_key(seg), value);
        }
        _ => unreachable!("coerce_container guarantees a matching container"),
    }
}

fn seg_key(seg: &Seg) -> String {
    match seg {
        Seg::Key(k) => k.clone(),
        Seg::Index(i) => i.to_string(),
    }
}

/// Makes `v` a container `seg` can index: scalars become objects, and an array indexed
/// by a non-numeric key becomes an object keyed by the former indices.
fn coerce_container(v: &mut Value, seg: &Seg) {
    match (seg, &mut *v) {
        (_, Value::Object(_)) | (Seg::Index(_), Value::Array(_)) => {}
        (Seg::Key(_), Value::Array(a)) => {
            let m: Map<String, Value> = a
                .drain(..)
                .enumerate()
                .map(|(i, x)| (i.to_string(), x))
                .collect();
            *v = Value::Object(m);
        }
        _ => *v = json!({}),
    }
}

fn unset_path(root: &mut Value, path: &[Seg]) -> bool {
    let Some((last, parents)) = path.split_last() else {
        return false;
    };
    let mut v = root;
    for seg in parents {
        v = match (seg, v) {
            (Seg::Key(k), Value::Object(m)) => match m.get_mut(k) {
                Some(x) => x,
                None => return false,
            },
            (Seg::Index(i), Value::Array(a)) => match a.get_mut(*i) {
                Some(x) => x,
                None => return false,
            },
            _ => return false,
        };
    }
    match (last, v) {
        (Seg::Key(k), Value::Object(m)) => m.remove(k).is_some(),
        (Seg::Index(i), Value::Array(a)) if *i < a.len() => {
            // lodash `unset` leaves a hole; JSON has no holes, so it becomes null.
            a[*i] = Value::Null;
            true
        }
        _ => false,
    }
}

/// Reads a lodash-style path out of any JSON value.
pub fn get_in<'a>(v: &'a Value, path: &str) -> Option<&'a Value> {
    get_path(v, &parse_path(path))
}

/// Writes a lodash-style path into any JSON value, creating parents.
pub fn set_in(v: &mut Value, path: &str, value: Value) {
    set_path(v, &parse_path(path), value);
}

fn default_state() -> Value {
    json!({
        "allowAnonymousUsageDataCollection": false,
        "checkForUpdates": true,
        "controller": { "exception": { "ignoreErrors": false } }
    })
}

/// The configuration file. All reads go to the in-memory copy; every write reloads
/// the file first (it may have been edited by hand) and writes it back.
#[derive(Debug)]
pub struct ConfigStore {
    file: PathBuf,
    data: RwLock<Value>,
    mtime: Mutex<Option<SystemTime>>,
}

impl ConfigStore {
    /// Loads (creating an empty `{}` file when missing).
    pub fn load(file: impl AsRef<Path>) -> Self {
        let file = file.as_ref().to_path_buf();
        if !file.exists() {
            if let Err(e) = std::fs::write(&file, "{}") {
                error!("Unable to create {}: {e}", file.display());
            }
        }
        let store = Self {
            file,
            data: RwLock::new(json!({})),
            mtime: Mutex::new(None),
        };
        store.reload();
        store
    }

    /// In-memory store that never touches the disk (tests).
    pub fn in_memory(data: Value) -> Self {
        let store = Self {
            file: PathBuf::new(),
            data: RwLock::new(data),
            mtime: Mutex::new(None),
        };
        store.apply_default_state();
        store
    }

    pub fn file(&self) -> &Path {
        &self.file
    }

    fn file_mtime(&self) -> Option<SystemTime> {
        std::fs::metadata(&self.file)
            .and_then(|m| m.modified())
            .ok()
    }

    fn apply_default_state(&self) {
        let mut data = self.data.write().unwrap();
        let mut state = default_state();
        if let (Value::Object(defaults), Some(Value::Object(user))) =
            (&mut state, data.get("state"))
        {
            for (k, v) in user {
                defaults.insert(k.clone(), v.clone());
            }
        }
        data["state"] = state;
    }

    /// Re-reads the file. Returns false (keeping the old data) when it is not valid JSON.
    pub fn reload(&self) -> bool {
        if self.file.as_os_str().is_empty() {
            return true;
        }
        if self.file.exists() {
            let parsed = std::fs::read_to_string(&self.file)
                .map_err(|e| e.to_string())
                .and_then(|s| serde_json::from_str::<Value>(&s).map_err(|e| e.to_string()));
            match parsed {
                Ok(v @ Value::Object(_)) => *self.data.write().unwrap() = v,
                Ok(_) => {
                    error!("{:?} does not contain valid JSON", self.file);
                    *self.data.write().unwrap() = json!({});
                }
                Err(e) => {
                    error!("Unable to load data from {:?}: {e}", self.file);
                    return false;
                }
            }
        }
        *self.mtime.lock().unwrap() = self.file_mtime();
        self.apply_default_state();
        true
    }

    fn sync(&self) -> bool {
        if self.file.as_os_str().is_empty() {
            return true;
        }
        let content = serde_json::to_string_pretty(&*self.data.read().unwrap()).unwrap_or_default();
        if let Err(e) = std::fs::write(&self.file, content) {
            error!("Unable to write data to {:?}: {e}", self.file);
            return false;
        }
        *self.mtime.lock().unwrap() = self.file_mtime();
        true
    }

    /// Whole config.
    pub fn all(&self) -> Value {
        self.data.read().unwrap().clone()
    }

    pub fn has(&self, key: &str) -> bool {
        get_path(&self.data.read().unwrap(), &parse_path(key)).is_some()
    }

    pub fn get(&self, key: &str) -> Option<Value> {
        get_path(&self.data.read().unwrap(), &parse_path(key)).cloned()
    }

    /// Typed read; `default` when missing or of the wrong type.
    pub fn get_or<T: DeserializeOwned>(&self, key: &str, default: T) -> T {
        self.get(key)
            .and_then(|v| serde_json::from_value(v).ok())
            .unwrap_or(default)
    }

    /// Value at `key` as an array (non-arrays and missing keys give an empty list).
    pub fn get_array(&self, key: &str) -> Vec<Value> {
        match self.get(key) {
            Some(Value::Array(a)) => a,
            Some(Value::Null) | None => vec![],
            Some(v) => vec![v],
        }
    }

    /// Sets and writes the file.
    pub fn set(&self, key: &str, value: Value) {
        self.set_inner(key, value, false);
    }

    /// Sets in memory only (cncjs `{ silent: true }`).
    pub fn set_silent(&self, key: &str, value: Value) {
        self.set_inner(key, value, true);
    }

    fn set_inner(&self, key: &str, value: Value, silent: bool) {
        let ok = self.reload();
        set_path(&mut self.data.write().unwrap(), &parse_path(key), value);
        if ok && !silent {
            self.sync();
        }
    }

    pub fn unset(&self, key: &str) {
        let ok = self.reload();
        unset_path(&mut self.data.write().unwrap(), &parse_path(key));
        if ok {
            self.sync();
        }
    }

    /// Polls the file every second and calls `on_change` after an external edit
    /// has been reloaded (cncjs `fs.watchFile`).
    pub fn watch(self: &Arc<Self>, on_change: impl Fn() + Send + 'static) {
        let store = Arc::clone(self);
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(1));
            loop {
                tick.tick().await;
                let current = store.file_mtime();
                let known = *store.mtime.lock().unwrap();
                if current.is_some() && current != known {
                    info!("{:?} has been changed", store.file);
                    if store.reload() {
                        on_change();
                    } else {
                        *store.mtime.lock().unwrap() = current;
                    }
                }
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lodash_paths() {
        assert_eq!(
            parse_path(r#"a.b[0]["c.d"]"#),
            vec![
                Seg::Key("a".into()),
                Seg::Key("b".into()),
                Seg::Index(0),
                Seg::Key("c.d".into())
            ]
        );
        let mut v = json!({});
        set_in(&mut v, "a.list[1].x", json!(5));
        assert_eq!(v, json!({"a": {"list": [null, {"x": 5}]}}));
        assert_eq!(get_in(&v, "a.list[1].x"), Some(&json!(5)));
        set_in(&mut v, "a.list", json!("replaced"));
        assert_eq!(get_in(&v, "a.list"), Some(&json!("replaced")));
    }

    #[test]
    fn default_state_is_merged_not_replaced() {
        let store = ConfigStore::in_memory(json!({"state": {"checkForUpdates": false}}));
        assert_eq!(store.get("state.checkForUpdates"), Some(json!(false)));
        assert_eq!(
            store.get("state.controller.exception.ignoreErrors"),
            Some(json!(false))
        );
        store.unset("state.checkForUpdates");
        assert!(!store.has("state.checkForUpdates"));
    }
}

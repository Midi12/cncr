//! Grbl response parsing and machine state (port of `GrblRunner.js` and the
//! `GrblLineParserResult*.js` parsers).

use std::sync::LazyLock;

use cncr_core::gcode::decimal_places;
use cncr_core::program::MachineSnapshot;
use cncr_core::util::js_str_to_num;
use regex::Regex;
use serde::ser::SerializeMap;
use serde::{Serialize, Serializer};
use serde_json::{json, Map, Value};

use crate::constants::{GRBL_ACTIVE_STATE_ALARM, GRBL_ACTIVE_STATE_IDLE, GRBL_MODAL_GROUPS};

const AXES: [&str; 6] = ["x", "y", "z", "a", "b", "c"];

/// A JS number: integral values serialize without a fraction, NaN/±Infinity as `null`
/// (what `JSON.stringify` does).
#[derive(Debug, Clone, Copy, Default)]
pub struct Num(pub f64);

impl PartialEq for Num {
    /// lodash `isEqual`: NaN equals NaN, 0 equals -0.
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0 || (self.0.is_nan() && other.0.is_nan())
    }
}

impl Serialize for Num {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let n = self.0;
        if !n.is_finite() {
            s.serialize_none()
        } else if n.fract() == 0.0 && n.abs() < 9e15 {
            s.serialize_i64(n as i64)
        } else {
            s.serialize_f64(n)
        }
    }
}

fn num_value(n: f64) -> Value {
    serde_json::to_value(Num(n)).unwrap_or(Value::Null)
}

/// JS `Number.prototype.toFixed` (`-0` prints without its sign).
pub fn to_fixed(n: f64, digits: usize) -> String {
    let n = if n == 0.0 { 0.0 } else { n };
    format!("{n:.digits$}")
}

/// Axis values as reported (`{x: "1.000", y: ...}`); only reported axes are present.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Pos(pub [Option<String>; 6]);

impl Pos {
    fn from_values(values: &str) -> Self {
        let mut p = Pos::default();
        for (slot, v) in p.0.iter_mut().zip(values.split(',')) {
            *slot = Some(v.to_string());
        }
        p
    }

    fn xyz(v: &str) -> Self {
        Pos([
            Some(v.into()),
            Some(v.into()),
            Some(v.into()),
            None,
            None,
            None,
        ])
    }

    pub fn get(&self, axis: usize) -> Option<&str> {
        self.0[axis].as_deref()
    }

    /// JS `Number(pos[axis])`: NaN when the axis is missing.
    pub fn num(&self, axis: usize) -> f64 {
        self.get(axis).map_or(f64::NAN, js_str_to_num)
    }

    /// `Number(pos[axis]) || 0` for every axis.
    fn nums_or_zero(&self) -> [f64; 6] {
        std::array::from_fn(|i| {
            let n = self.num(i);
            if n.is_nan() {
                0.0
            } else {
                n
            }
        })
    }

    /// WPos from MPos (`sign = -1`) or MPos from WPos (`sign = 1`) with the work
    /// coordinate offset, keeping the decimal places of each reported value.
    fn offset(&self, wco: Option<&Pos>, sign: f64) -> Pos {
        let mut out = Pos::default();
        for (i, slot) in out.0.iter_mut().enumerate() {
            if let Some(v) = self.get(i) {
                let w = wco.and_then(|w| w.get(i)).map_or(0.0, js_str_to_num);
                *slot = Some(to_fixed(js_str_to_num(v) + sign * w, decimal_places(v)));
            }
        }
        out
    }
}

impl Serialize for Pos {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut m = s.serialize_map(None)?;
        for (axis, v) in AXES.iter().zip(&self.0) {
            if let Some(v) = v {
                m.serialize_entry(axis, v)?;
            }
        }
        m.end()
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize)]
pub struct Buf {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub planner: Option<Num>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rx: Option<Num>,
}

/// `state.status`: every report is merged into it.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Status {
    pub active_state: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sub_state: Option<Num>,
    pub mpos: Pos,
    pub wpos: Pos,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wco: Option<Pos>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub buf: Option<Buf>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ln: Option<Num>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feedrate: Option<Num>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spindle: Option<Num>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pin_state: Option<String>,
    pub ov: Vec<Num>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accessory_state: Option<String>,
}

impl Default for Status {
    fn default() -> Self {
        Self {
            active_state: String::new(),
            sub_state: None,
            mpos: Pos::xyz("0.000"),
            wpos: Pos::xyz("0.000"),
            wco: None,
            buf: None,
            ln: None,
            feedrate: None,
            spindle: None,
            pin_state: None,
            ov: Vec::new(),
            accessory_state: None,
        }
    }
}

/// One parsed `<...>` status report.
#[derive(Debug, Default, PartialEq)]
pub struct StatusReport {
    pub active_state: String,
    pub sub_state: f64,
    pub mpos: Option<Pos>,
    pub wpos: Option<Pos>,
    pub wco: Option<Pos>,
    pub buf: Option<Buf>,
    pub ln: Option<f64>,
    pub feedrate: Option<f64>,
    pub spindle: Option<f64>,
    pub pin_state: Option<String>,
    pub ov: Option<Vec<Num>>,
    pub accessory_state: Option<String>,
}

/// `state.parserstate` (`$G` reply). `modal` values are a word or, for several
/// words of one group (`M7 M8`), an array.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ParserState {
    #[serde(skip_serializing_if = "Map::is_empty")]
    pub modal: Map<String, Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feedrate: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spindle: Option<String>,
}

impl Default for ParserState {
    fn default() -> Self {
        let modal = [
            ("motion", "G0"),
            ("wcs", "G54"),
            ("plane", "G17"),
            ("units", "G21"),
            ("distance", "G90"),
            ("feedrate", "G94"),
            ("program", "M0"),
            ("spindle", "M5"),
            ("coolant", "M9"),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), json!(v)))
        .collect();
        Self {
            modal,
            tool: Some(String::new()),
            feedrate: Some(String::new()),
            spindle: Some(String::new()),
        }
    }
}

/// `controller:state` payload.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct State {
    pub status: Status,
    pub parserstate: ParserState,
}

/// `controller:settings` payload.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct Settings {
    pub version: String,
    pub parameters: Map<String, Value>,
    pub settings: Map<String, Value>,
}

/// What a received line was (the JS runner events).
#[derive(Debug, PartialEq)]
pub enum Event {
    /// Status report; `rx` is the reported free RX buffer (`buf.rx`, 0 if absent).
    Status {
        rx: f64,
    },
    Ok,
    Error(String),
    Alarm(String),
    ParserState,
    Parameters {
        name: String,
        value: Value,
    },
    Feedback(String),
    Settings {
        name: String,
        value: String,
        message: String,
    },
    Startup {
        firmware: String,
        version: String,
        message: String,
    },
    Others,
}

// Lives on the stack for one line; boxing would allocate on every status report.
#[allow(clippy::large_enum_variant)]
enum Parsed {
    Status(StatusReport),
    ParserState(ParserState),
    Event(Event),
}

#[derive(Debug, Default)]
pub struct Runner {
    pub state: State,
    pub settings: Settings,
    /// Bumped on every deep change of `state` / `settings` (the JS "enforce change").
    pub state_rev: u64,
    pub settings_rev: u64,
}

fn set<T: PartialEq>(dst: &mut T, v: T, changed: &mut bool) {
    if *dst != v {
        *dst = v;
        *changed = true;
    }
}

impl Runner {
    /// Parses one received line; `None` for blank lines.
    pub fn parse(&mut self, data: &str) -> Option<Event> {
        let line = data.trim_end();
        if line.is_empty() {
            return None;
        }
        Some(match parse_line(line) {
            Parsed::Status(report) => {
                let rx = report.buf.and_then(|b| b.rx).map_or(0.0, |n| n.0);
                self.apply_status(report);
                Event::Status {
                    rx: if rx.is_nan() { 0.0 } else { rx },
                }
            }
            Parsed::ParserState(ps) => {
                if self.state.parserstate != ps {
                    self.state.parserstate = ps;
                    self.state_rev += 1;
                }
                Event::ParserState
            }
            Parsed::Event(event) => {
                match &event {
                    Event::Alarm(_)
                        if self.state.status.active_state != GRBL_ACTIVE_STATE_ALARM =>
                    {
                        self.state.status.active_state = GRBL_ACTIVE_STATE_ALARM.into();
                        self.state_rev += 1;
                    }
                    Event::Parameters { name, value } => {
                        self.set_setting(true, name, value.clone())
                    }
                    Event::Settings { name, value, .. } => {
                        self.set_setting(false, name, json!(value))
                    }
                    Event::Startup { version, .. } if self.settings.version != *version => {
                        self.settings.version.clone_from(version);
                        self.settings_rev += 1;
                    }
                    _ => {}
                }
                event
            }
        })
    }

    fn set_setting(&mut self, parameter: bool, name: &str, value: Value) {
        let map = if parameter {
            &mut self.settings.parameters
        } else {
            &mut self.settings.settings
        };
        if map.get(name) != Some(&value) {
            map.insert(name.to_string(), value);
            self.settings_rev += 1;
        }
    }

    /// `$13=<n>` sent to Grbl: record the report units immediately (always counts as a change).
    pub fn set_report_inches(&mut self, inches: bool) {
        self.settings
            .settings
            .insert("$13".into(), json!(if inches { "1" } else { "0" }));
        self.settings_rev += 1;
    }

    fn apply_status(&mut self, mut p: StatusReport) {
        let s = &mut self.state.status;
        // Grbl v1.1 reports MPos or WPos plus WCO from time to time: WPos = MPos - WCO.
        let wco = p.wco.as_ref().or(s.wco.as_ref());
        if p.wpos.is_none() {
            if let Some(mpos) = &p.mpos {
                p.wpos = Some(mpos.offset(wco, -1.0));
            }
        } else if p.mpos.is_none() {
            if let Some(wpos) = &p.wpos {
                p.mpos = Some(wpos.offset(wco, 1.0));
            }
        }

        let mut changed = false;
        set(&mut s.active_state, p.active_state, &mut changed);
        set(&mut s.sub_state, Some(Num(p.sub_state)), &mut changed);
        if let Some(v) = p.mpos {
            set(&mut s.mpos, v, &mut changed);
        }
        if let Some(v) = p.wpos {
            set(&mut s.wpos, v, &mut changed);
        }
        if p.wco.is_some() {
            set(&mut s.wco, p.wco, &mut changed);
        }
        if p.buf.is_some() {
            set(&mut s.buf, p.buf, &mut changed);
        }
        for (dst, v) in [
            (&mut s.ln, p.ln),
            (&mut s.feedrate, p.feedrate),
            (&mut s.spindle, p.spindle),
        ] {
            if let Some(v) = v {
                set(dst, Some(Num(v)), &mut changed);
            }
        }
        // Grbl omits Pn: when no pin is triggered: absence means "all clear".
        set(
            &mut s.pin_state,
            Some(p.pin_state.unwrap_or_default()),
            &mut changed,
        );
        // A: only comes with Ov:, so a report without Ov: carries no accessory
        // snapshot (keep the previous one) and Ov: without A: means all off.
        if let Some(ov) = p.ov {
            set(&mut s.ov, ov, &mut changed);
            set(
                &mut s.accessory_state,
                Some(p.accessory_state.unwrap_or_default()),
                &mut changed,
            );
        } else if p.accessory_state.is_some() {
            set(&mut s.accessory_state, p.accessory_state, &mut changed);
        }
        if changed {
            self.state_rev += 1;
        }
    }

    pub fn is_alarm(&self) -> bool {
        self.state.status.active_state == GRBL_ACTIVE_STATE_ALARM
    }

    pub fn is_idle(&self) -> bool {
        self.state.status.active_state == GRBL_ACTIVE_STATE_IDLE
    }

    /// Modal units word (`G20`/`G21`, empty when unknown).
    pub fn modal_units(&self) -> &str {
        self.state
            .parserstate
            .modal
            .get("units")
            .and_then(Value::as_str)
            .unwrap_or("")
    }

    /// Machine values for macro contexts (`populateContext`).
    pub fn snapshot(&self) -> MachineSnapshot {
        let tool = self
            .state
            .parserstate
            .tool
            .as_deref()
            .map_or(f64::NAN, js_str_to_num);
        MachineSnapshot {
            mpos: self.state.status.mpos.nums_or_zero(),
            wpos: self.state.status.wpos.nums_or_zero(),
            modal: Value::Object(self.state.parserstate.modal.clone()),
            tool: if tool.is_nan() { 0.0 } else { tool },
            params: Value::Object(self.settings.parameters.clone()),
        }
    }
}

static PARSER_STATE_RE: LazyLock<[Regex; 2]> = LazyLock::new(|| {
    [
        // Grbl v0.9 and v1.x
        Regex::new(r"^\[(?:GC:)?((?:[GMTFS]\d+(?:\.\d*)?\s+)+(?:[GMTFS]\d+(?:\.\d*)?))\]$")
            .unwrap(),
        // Grbl v1.x forks printing words without a value (`M`, `T`)
        Regex::new(r"^\[GC:((?:[GMTFS]\d*(?:\.\d*)?\s+)+(?:[GMTFS]\d+(?:\.\d*)?))\]$").unwrap(),
    ]
});
static SETTINGS_RE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^(\$[^=]+)=([^ ]*)\s*(.*)").unwrap());
static STARTUP_RE: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"^([a-zA-Z0-9]+)\s+((?:\d+\.){1,2}\d+[a-zA-Z0-9\-\.]*)([^\[]*\[[^\]]+\].*)?")
        .unwrap()
});

const PARAMETER_NAMES: [&str; 11] = [
    "G54", "G55", "G56", "G57", "G58", "G59", "G28", "G30", "G92", "TLO", "PRB",
];

/// `^prefix\s*(.+)$`
fn after_prefix<'a>(line: &'a str, prefix: &str) -> Option<&'a str> {
    line.strip_prefix(prefix)
        .map(str::trim_start)
        .filter(|s| !s.is_empty())
}

fn parse_line(line: &str) -> Parsed {
    if let Some(inner) = line
        .strip_prefix('<')
        .and_then(|l| l.strip_suffix('>'))
        .filter(|s| !s.is_empty())
    {
        return Parsed::Status(parse_status(inner));
    }
    let event = if line == "ok" {
        Event::Ok
    } else if let Some(m) = after_prefix(line, "error:") {
        Event::Error(m.into())
    } else if let Some(m) = after_prefix(line, "ALARM:") {
        Event::Alarm(m.into())
    } else if let Some(ps) = parse_parser_state(line) {
        return Parsed::ParserState(ps);
    } else if let Some(inner) = line
        .strip_prefix('[')
        .and_then(|l| l.strip_suffix(']'))
        .filter(|s| !s.is_empty())
    {
        parse_bracketed(inner)
    } else if let Some(c) = SETTINGS_RE.captures(line) {
        Event::Settings {
            name: c[1].into(),
            value: c[2].into(),
            message: c[3].trim_matches(|ch| ch == '(' || ch == ')').into(),
        }
    } else if let Some(c) = STARTUP_RE.captures(line) {
        Event::Startup {
            firmware: c[1].into(),
            version: c[2].into(),
            message: c.get(3).map_or("", |m| m.as_str().trim()).into(),
        }
    } else {
        Event::Others
    };
    Parsed::Event(event)
}

/// `[...]` lines: parameters, help/version/option/echo (no runner event: "others"),
/// and feedback messages.
fn parse_bracketed(inner: &str) -> Event {
    if let Some((name, value)) = inner
        .split_once(':')
        .filter(|(n, v)| PARAMETER_NAMES.contains(n) && !v.is_empty())
    {
        let value = match name {
            "TLO" => json!(value),
            "PRB" => {
                // [PRB:0.000,0.000,1.492:1]: the suffix tells whether the probe succeeded.
                let mut parts = value.split(':');
                let axes = parts.next().unwrap_or("");
                let mut m = Map::new();
                m.insert(
                    "result".into(),
                    num_value(parts.next().map_or(f64::NAN, js_str_to_num)),
                );
                for (axis, v) in AXES.iter().zip(axes.split(',')) {
                    m.insert((*axis).into(), json!(v));
                }
                Value::Object(m)
            }
            _ => Value::Object(
                AXES.iter()
                    .zip(value.split(','))
                    .map(|(a, v)| ((*a).to_string(), json!(v)))
                    .collect(),
            ),
        };
        return Event::Parameters {
            name: name.into(),
            value,
        };
    }
    if ["HLP:", "VER:", "OPT:", "echo:"]
        .iter()
        .any(|p| inner.strip_prefix(p).is_some_and(|r| !r.is_empty()))
    {
        return Event::Others;
    }
    Event::Feedback(
        inner
            .strip_prefix("MSG:")
            .filter(|r| !r.is_empty())
            .unwrap_or(inner)
            .into(),
    )
}

fn parse_parser_state(line: &str) -> Option<ParserState> {
    let caps = PARSER_STATE_RE.iter().find_map(|re| re.captures(line))?;
    let mut ps = ParserState {
        modal: Map::new(),
        tool: None,
        feedrate: None,
        spindle: None,
    };
    for word in caps[1].split(' ').map(str::trim).filter(|w| w.len() > 1) {
        match word.as_bytes()[0] {
            b'G' | b'M' => {
                let Some((group, _)) = GRBL_MODAL_GROUPS
                    .iter()
                    .find(|(_, modes)| modes.contains(&word))
                else {
                    continue;
                };
                match ps.modal.get_mut(*group) {
                    Some(Value::Array(words)) => words.push(json!(word)),
                    Some(prev) => *prev = json!([prev.take(), word]),
                    None => {
                        ps.modal.insert((*group).into(), json!(word));
                    }
                }
            }
            b'T' => ps.tool = Some(word[1..].into()),
            b'F' => ps.feedrate = Some(word[1..].into()),
            b'S' => ps.spindle = Some(word[1..].into()),
            _ => {}
        }
    }
    Some(ps)
}

fn count(b: &[u8], from: usize, pred: impl Fn(u8) -> bool) -> usize {
    b.get(from..)
        .map_or(0, |r| r.iter().take_while(|&&c| pred(c)).count())
}

/// Next match of `/(?<=\|)(?:Pn|A):[a-zA-Z]+(?=\||$)|[a-zA-Z]+(:[0-9.\-]+(,[0-9.\-]+){0,5})?/g`.
/// The first branch keeps the letter-valued v1.1 `Pn:`/`A:` fields whole; the second
/// splits v0.9 comma-separated fields (`MPos:..,WPos:..`) at their boundary.
fn next_field<'a>(s: &'a str, pos: &mut usize) -> Option<&'a str> {
    let b = s.as_bytes();
    let alpha = |c: u8| c.is_ascii_alphabetic();
    let numeric = |c: u8| c.is_ascii_digit() || c == b'.' || c == b'-';
    while *pos < b.len() {
        let i = *pos;
        if i > 0 && b[i - 1] == b'|' {
            let rest = &s[i..];
            if let Some(after) = rest.strip_prefix("Pn:").or_else(|| rest.strip_prefix("A:")) {
                let start = i + rest.len() - after.len();
                let n = count(b, start, alpha);
                let end = start + n;
                if n > 0 && (end == b.len() || b[end] == b'|') {
                    *pos = end;
                    return Some(&s[i..end]);
                }
            }
        }
        let letters = count(b, i, alpha);
        if letters == 0 {
            *pos += 1;
            continue;
        }
        let mut end = i + letters;
        if b.get(end) == Some(&b':') {
            let n = count(b, end + 1, numeric);
            if n > 0 {
                end += 1 + n;
                for _ in 0..5 {
                    let n = if b.get(end) == Some(&b',') {
                        count(b, end + 1, numeric)
                    } else {
                        0
                    };
                    if n == 0 {
                        break;
                    }
                    end += 1 + n;
                }
            }
        }
        *pos = end;
        return Some(&s[i..end]);
    }
    None
}

const FIELDS: [&str; 13] = [
    "MPos", "WPos", "WCO", "Buf", "RX", "Bf", "Ln", "F", "FS", "Lim", "Pn", "Ov", "A",
];

fn parse_status(inner: &str) -> StatusReport {
    let mut pos = 0;
    let first = next_field(inner, &mut pos).unwrap_or("");
    let mut states = first.split(':');
    let mut r = StatusReport {
        active_state: states.next().unwrap_or("").into(),
        sub_state: js_str_to_num(states.next().unwrap_or("")),
        ..Default::default()
    };

    let mut fields: [Option<&str>; FIELDS.len()] = [None; FIELDS.len()];
    while let Some(field) = next_field(inner, &mut pos) {
        if let Some((name, value)) = field.rsplit_once(':') {
            if let Some(i) = FIELDS.iter().position(|f| *f == name) {
                fields[i] = Some(value);
            }
        }
    }
    let [mpos, wpos, wco, buf, rx, bf, ln, f, fs, lim, pn, ov, a] = fields;
    let nth = |v: &str, n: usize| v.split(',').nth(n).map_or(0.0, js_str_to_num);

    r.mpos = mpos.map(Pos::from_values);
    r.wpos = wpos.map(Pos::from_values);
    r.wco = wco.map(Pos::from_values);
    if let Some(v) = buf {
        r.buf.get_or_insert_default().planner = Some(Num(nth(v, 0)));
    }
    if let Some(v) = rx {
        r.buf.get_or_insert_default().rx = Some(Num(nth(v, 0)));
    }
    if let Some(v) = bf {
        // Bf:15,128: free planner blocks, free RX buffer bytes.
        r.buf = Some(Buf {
            planner: Some(Num(nth(v, 0))),
            rx: Some(Num(nth(v, 1))),
        });
    }
    r.ln = ln.map(|v| nth(v, 0));
    r.feedrate = f.map(|v| nth(v, 0));
    if let Some(v) = fs {
        r.feedrate = Some(nth(v, 0));
        r.spindle = Some(nth(v, 1));
    }
    if let Some(v) = lim {
        // v0.9 limit pins bitmask (the A bit duplicates Z, as in cncjs).
        let n = nth(v, 0);
        let n = if n.is_finite() { n as i64 } else { 0 };
        let pins: String = [(0, 'X'), (1, 'Y'), (2, 'Z'), (2, 'A')]
            .iter()
            .filter(|(bit, _)| n & (1 << bit) != 0)
            .map(|(_, c)| c)
            .collect();
        r.pin_state = Some(pins);
    }
    if let Some(v) = pn {
        r.pin_state = Some(v.split(',').next().unwrap_or("").into());
    }
    r.ov = ov.map(|v| v.split(',').map(|n| Num(js_str_to_num(n))).collect());
    r.accessory_state = a.map(|v| v.split(',').next().unwrap_or("").into());
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    fn status(runner: &Runner) -> Value {
        serde_json::to_value(&runner.state.status).unwrap()
    }

    #[test]
    fn status_reports_v09() {
        let mut r = Runner::default();
        assert_eq!(r.parse("<Idle>\r\n"), Some(Event::Status { rx: 0.0 }));
        assert_eq!(r.state.status.active_state, "Idle");
        assert_eq!(r.state.status.pin_state.as_deref(), Some(""));

        r.parse("<Idle,MPos:5.529,0.560,7.000,0.100,0.250,0.500,WPos:1.529,-5.440,-0.000,0.100,0.250,0.500>");
        let s = status(&r);
        assert_eq!(
            s["mpos"],
            json!({"x": "5.529", "y": "0.560", "z": "7.000", "a": "0.100", "b": "0.250", "c": "0.500"})
        );
        assert_eq!(
            s["wpos"],
            json!({"x": "1.529", "y": "-5.440", "z": "-0.000", "a": "0.100", "b": "0.250", "c": "0.500"})
        );
        assert_eq!(s["subState"], json!(0));

        assert_eq!(
            r.parse("<Idle,MPos:5.529,0.560,7.000,WPos:1.529,-5.440,-0.000,Buf:0,RX:0,Lim:000>"),
            Some(Event::Status { rx: 0.0 })
        );
        let s = status(&r);
        assert_eq!(s["buf"], json!({"planner": 0, "rx": 0}));
        assert_eq!(s["pinState"], json!(""));
        assert_eq!(s["mpos"], json!({"x": "5.529", "y": "0.560", "z": "7.000"}));

        r.parse(
            "<Run,MPos:0.000,0.000,0.000,WPos:0.000,0.000,0.000,Buf:3,RX:96,Ln:12,F:500.,Lim:101>",
        );
        let s = status(&r);
        assert_eq!(
            (
                s["ln"].clone(),
                s["feedrate"].clone(),
                s["pinState"].clone()
            ),
            (json!(12), json!(500), json!("XZA"))
        );
    }

    #[test]
    fn status_reports_v11_derive_positions_from_wco() {
        let mut r = Runner::default();
        assert_eq!(
            r.parse("<Hold:1|MPos:5.000,2.000,0.000|Bf:15,128|FS:500,8000|WCO:1.5,-2.25,0.000|Ov:100,50,120>"),
            Some(Event::Status { rx: 128.0 })
        );
        let s = status(&r);
        assert_eq!(
            (s["activeState"].clone(), s["subState"].clone()),
            (json!("Hold"), json!(1))
        );
        assert_eq!(s["wpos"], json!({"x": "3.500", "y": "4.250", "z": "0.000"}));
        assert_eq!(
            (s["feedrate"].clone(), s["spindle"].clone()),
            (json!(500), json!(8000))
        );
        assert_eq!(s["ov"], json!([100, 50, 120]));
        // Later reports without WCO reuse the last offset.
        r.parse("<Idle|MPos:6.00,2.000,1.000|FS:0,0>");
        assert_eq!(
            status(&r)["wpos"],
            json!({"x": "4.50", "y": "4.250", "z": "1.000"})
        );
        // WPos reports derive MPos.
        r.parse("<Idle|WPos:1.000,1.000,1.000|FS:0,0>");
        assert_eq!(
            status(&r)["mpos"],
            json!({"x": "2.500", "y": "-1.250", "z": "1.000"})
        );
    }

    #[test]
    fn pin_state_clears_when_omitted() {
        let mut r = Runner::default();
        let mut seen = vec![];
        for line in [
            "<Idle|MPos:148.694,108.851,0.000|FS:0,0|Pn:X>",
            "<Idle|MPos:5.000,2.000,0.000|FS:0,0|Pn:PZ>",
            "<Idle|MPos:5.000,2.000,0.000|FS:0,0>",
        ] {
            r.parse(line);
            seen.push(r.state.status.pin_state.clone().unwrap());
        }
        assert_eq!(seen, ["X", "PZ", ""]);
    }

    #[test]
    fn accessory_state_clears_on_next_override_refresh() {
        let mut r = Runner::default();
        r.parse("<Idle|MPos:5.000,2.000,0.000|FS:0,0|Ov:100,100,100|A:SFM>");
        assert_eq!(r.state.status.accessory_state.as_deref(), Some("SFM"));
        r.parse("<Idle|MPos:5.000,2.000,0.000|FS:0,0>");
        assert_eq!(r.state.status.accessory_state.as_deref(), Some("SFM"));
        r.parse("<Idle|MPos:5.000,2.000,0.000|FS:0,0|Ov:100,100,100>");
        assert_eq!(r.state.status.accessory_state.as_deref(), Some(""));
    }

    #[test]
    fn state_revision_changes_only_on_deep_change() {
        let mut r = Runner::default();
        r.parse("<Idle|MPos:1.000,2.000,0.000|FS:0,0>");
        let rev = r.state_rev;
        r.parse("<Idle|MPos:1.000,2.000,0.000|FS:0,0>");
        assert_eq!(r.state_rev, rev);
        r.parse("<Idle|MPos:1.000,2.001,0.000|FS:0,0>");
        assert_eq!(r.state_rev, rev + 1);
    }

    #[test]
    fn ok_error_alarm() {
        let mut r = Runner::default();
        assert_eq!(r.parse("ok"), Some(Event::Ok));
        assert_eq!(
            r.parse("error: Expected command letter"),
            Some(Event::Error("Expected command letter".into()))
        );
        assert_eq!(
            r.parse("ALARM: Probe fail"),
            Some(Event::Alarm("Probe fail".into()))
        );
        assert!(r.is_alarm());
        assert_eq!(r.parse("   "), None);
    }

    #[test]
    fn parser_state_variants() {
        let mut r = Runner::default();
        let modal = |coolant: Value, spindle: &str, program: Option<&str>| {
            let mut m = json!({"motion": "G0", "wcs": "G54", "plane": "G17", "units": "G21", "distance": "G90", "feedrate": "G94"});
            if let Some(p) = program {
                m["program"] = json!(p);
            }
            m["spindle"] = json!(spindle);
            m["coolant"] = coolant;
            m
        };
        let ps = |r: &Runner| serde_json::to_value(&r.state.parserstate).unwrap();
        for line in [
            "[G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 F2540. S0.]",
            "[GC:G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 F2540. S0.]",
        ] {
            assert_eq!(r.parse(line), Some(Event::ParserState));
            assert_eq!(
                ps(&r),
                json!({"modal": modal(json!("M9"), "M5", Some("M0")), "tool": "0", "feedrate": "2540.", "spindle": "0."})
            );
        }
        r.parse("[GC:G0 G54 G17 G21 G90 G94 M0 M3 M7 M8 T0 F2000 S20]");
        assert_eq!(
            ps(&r),
            json!({"modal": modal(json!(["M7", "M8"]), "M3", Some("M0")), "tool": "0", "feedrate": "2000", "spindle": "20"})
        );
        r.parse("[GC:G0 G54 G17 G21 G90 G94 M5 M M9 T0 F0 S0]");
        assert_eq!(
            ps(&r),
            json!({"modal": modal(json!("M9"), "M5", None), "tool": "0", "feedrate": "0", "spindle": "0"})
        );
        r.parse("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T F0 S0]");
        assert_eq!(
            ps(&r),
            json!({"modal": modal(json!("M9"), "M5", None), "feedrate": "0", "spindle": "0"})
        );
        assert_eq!(r.snapshot().tool, 0.0);
        assert_eq!(
            r.parse("[Invalid Parser State Output]"),
            Some(Event::Feedback("Invalid Parser State Output".into()))
        );
    }

    #[test]
    fn parameters() {
        let mut r = Runner::default();
        for name in [
            "G54", "G55", "G56", "G57", "G58", "G59", "G28", "G30", "G92",
        ] {
            assert_eq!(
                r.parse(&format!("[{name}:0.000,0.000,0.000]")),
                Some(Event::Parameters {
                    name: name.into(),
                    value: json!({"x": "0.000", "y": "0.000", "z": "0.000"})
                })
            );
        }
        assert_eq!(
            r.parse("[TLO:0.000]"),
            Some(Event::Parameters {
                name: "TLO".into(),
                value: json!("0.000")
            })
        );
        assert_eq!(
            r.parse("[PRB:0.000,0.000,1.492:1]"),
            Some(Event::Parameters {
                name: "PRB".into(),
                value: json!({"result": 1, "x": "0.000", "y": "0.000", "z": "1.492"})
            })
        );
        assert_eq!(r.settings.parameters["TLO"], json!("0.000"));
        assert_eq!(r.settings.parameters.len(), 11);
    }

    #[test]
    fn feedback_and_others() {
        let mut r = Runner::default();
        for line in [
            "[0.9j.20160303:]",
            "[Reset to continue]",
            "['$H'|'$X' to unlock]",
            "[Caution: Unlocked]",
            "[Enabled]",
            "[Disabled]",
        ] {
            assert_eq!(
                r.parse(line),
                Some(Event::Feedback(line.trim_matches(['[', ']']).into()))
            );
        }
        assert_eq!(
            r.parse("[MSG:Pgm End]"),
            Some(Event::Feedback("Pgm End".into()))
        );
        assert_eq!(r.parse("[VER:1.1f.20170801:]"), Some(Event::Others));
        assert_eq!(r.parse("Not supported output format"), Some(Event::Others));
    }

    #[test]
    fn settings() {
        let mut r = Runner::default();
        assert_eq!(
            r.parse("$11=0.020 (junction deviation, mm)"),
            Some(Event::Settings {
                name: "$11".into(),
                value: "0.020".into(),
                message: "junction deviation, mm".into()
            })
        );
        assert_eq!(
            r.parse("$13=0"),
            Some(Event::Settings {
                name: "$13".into(),
                value: "0".into(),
                message: String::new()
            })
        );
        assert_eq!(r.settings.settings["$11"], json!("0.020"));
        let rev = r.settings_rev;
        r.parse("$13=0");
        assert_eq!(r.settings_rev, rev);
    }

    #[test]
    fn startup_messages() {
        let cases = [
            ("Grbl 0.9j ['$' for help]", "Grbl", "0.9j", "['$' for help]"),
            ("Grbl 1.1f ['$' for help]", "Grbl", "1.1f", "['$' for help]"),
            ("Grbl 1.2.3", "Grbl", "1.2.3", ""),
            (
                "Grbl 1.1h: LongMill build ['$' for help]",
                "Grbl",
                "1.1h",
                ": LongMill build ['$' for help]",
            ),
            (
                "Grbl 1.1h ['$' for help] LongMill build Feb 25, 2020",
                "Grbl",
                "1.1h",
                "['$' for help] LongMill build Feb 25, 2020",
            ),
            (
                "vCarvin 2.0.0 ['$' for help]",
                "vCarvin",
                "2.0.0",
                "['$' for help]",
            ),
        ];
        for (line, firmware, version, message) in cases {
            let mut r = Runner::default();
            assert_eq!(
                r.parse(line),
                Some(Event::Startup {
                    firmware: firmware.into(),
                    version: version.into(),
                    message: message.into()
                }),
                "{line}"
            );
            assert_eq!(r.settings.version, version);
        }
    }
}

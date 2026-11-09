//! Smoothieware response parsing and machine state (port of `SmoothieRunner` and the
//! `SmoothieLineParserResult*` parsers).

use std::sync::LazyLock;

use cncr_core::util::{js_str_to_num, json_num};
use regex::Regex;
use serde::{Serialize, Serializer};
use serde_json::{Map, Value};

pub const ACTIVE_STATE_IDLE: &str = "Idle";
pub const ACTIVE_STATE_HOLD: &str = "Hold";
pub const ACTIVE_STATE_ALARM: &str = "Alarm";

const AXES: [&str; 6] = ["x", "y", "z", "a", "b", "c"];

/// <http://linuxcnc.org/docs/html/gcode/overview.html#cap:modal-groups>
const MODAL_GROUPS: [(&str, &[&str]); 9] = [
    (
        "motion",
        &[
            "G0", "G1", "G2", "G3", "G38.2", "G38.3", "G38.4", "G38.5", "G80",
        ],
    ),
    ("wcs", &["G54", "G55", "G56", "G57", "G58", "G59"]),
    ("plane", &["G17", "G18", "G19"]),
    ("units", &["G20", "G21"]),
    ("distance", &["G90", "G91"]),
    ("feedrate", &["G93", "G94"]),
    ("program", &["M0", "M1", "M2", "M30"]),
    ("spindle", &["M3", "M4", "M5"]),
    ("coolant", &["M7", "M8", "M9"]),
];

/// Axis values as reported (`{x: "1.000", ...}`), kept as strings.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Axes(pub Vec<String>);

impl Axes {
    fn parse(list: &str) -> Self {
        Self(
            list.split(',')
                .take(AXES.len())
                .map(str::to_string)
                .collect(),
        )
    }

    /// `Number(pos[axis])`: NaN when the axis is missing.
    pub fn num(&self, i: usize) -> f64 {
        self.0.get(i).map_or(f64::NAN, |v| js_str_to_num(v))
    }
}

impl Serialize for Axes {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_map(AXES.iter().zip(&self.0))
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Status {
    pub active_state: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mpos: Option<Axes>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wpos: Option<Axes>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub current_feedrate: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feedrate: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feedrate_override: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub laser_power: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub laser_intensity: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub current_temperature: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_temperature: Option<f64>,
    /// Set by the `feedOverride` command until the next status report replaces it.
    #[serde(rename = "ovF", skip_serializing_if = "Option::is_none")]
    pub ov_f: Option<f64>,
    /// Set by the `spindleOverride` command until the next status report replaces it.
    #[serde(rename = "ovS", skip_serializing_if = "Option::is_none")]
    pub ov_s: Option<f64>,
}

/// A modal group value: one word, or several (`[M7, M8]`).
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(untagged)]
pub enum Word {
    One(String),
    Many(Vec<String>),
}

/// Modal groups in [`MODAL_GROUPS`] order; unreported groups are omitted.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Modal([Option<Word>; 9]);

impl Modal {
    fn initial() -> Self {
        Self(
            ["G0", "G54", "G17", "G21", "G90", "G94", "M0", "M5", "M9"]
                .map(|w| Some(Word::One(w.into()))),
        )
    }

    fn push(&mut self, group: usize, word: &str) {
        let slot = &mut self.0[group];
        *slot = Some(match slot.take() {
            None => Word::One(word.into()),
            Some(Word::One(prev)) => Word::Many(vec![prev, word.into()]),
            Some(Word::Many(mut words)) => {
                words.push(word.into());
                Word::Many(words)
            }
        });
    }

    pub fn get(&self, group: &str) -> Option<&Word> {
        MODAL_GROUPS
            .iter()
            .position(|(g, _)| *g == group)
            .and_then(|i| self.0[i].as_ref())
    }
}

impl Serialize for Modal {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_map(
            MODAL_GROUPS
                .iter()
                .zip(&self.0)
                .filter_map(|((g, _), w)| w.as_ref().map(|w| (g, w))),
        )
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct ParserState {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub modal: Option<Modal>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feedrate: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spindle: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct State {
    pub status: Status,
    pub parserstate: ParserState,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct Build {
    pub version: String,
    pub date: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Hardware {
    pub mcu: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sysclk: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Settings {
    pub build: Build,
    pub hardware: Hardware,
    pub parameters: Map<String, Value>,
}

/// What a line was, after the runner updated its state.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Status,
    Ok,
    Error(String),
    Action(String),
    Alarm(String),
    ParserState,
    Parameters { name: String, value: Value },
    Version,
    Others,
}

#[derive(Debug, Clone)]
pub struct Runner {
    pub state: State,
    pub settings: Settings,
}

impl Default for Runner {
    fn default() -> Self {
        let zero = || Some(Axes(vec!["0.0000".into(); 3]));
        Self {
            state: State {
                status: Status {
                    mpos: zero(),
                    wpos: zero(),
                    ..Default::default()
                },
                parserstate: ParserState {
                    modal: Some(Modal::initial()),
                    tool: Some(String::new()),
                    feedrate: Some(String::new()),
                    spindle: Some(String::new()),
                },
            },
            settings: Settings {
                build: Build::default(),
                hardware: Hardware {
                    mcu: String::new(),
                    sysclk: Some(String::new()),
                },
                parameters: Map::new(),
            },
        }
    }
}

impl Runner {
    /// Parses one received line; `None` for blank lines.
    pub fn parse(&mut self, data: &str) -> Option<Event> {
        let line = data.trim_end();
        if line.is_empty() {
            return None;
        }
        if let Some(status) = parse_status(line) {
            self.state.status = status;
            return Some(Event::Status);
        }
        if line == "ok" {
            return Some(Event::Ok);
        }
        if let Some(message) = message_after(line, "error:") {
            return Some(Event::Error(message));
        }
        if let Some(message) = line.strip_prefix("// action:").filter(|m| !m.is_empty()) {
            return Some(Event::Action(message.to_string()));
        }
        if let Some(message) = message_after(line, "ALARM:") {
            return Some(Event::Alarm(message));
        }
        if let Some(parserstate) = parse_parser_state(line) {
            self.state.parserstate = parserstate;
            return Some(Event::ParserState);
        }
        if let Some((name, value)) = parse_parameters(line) {
            self.settings.parameters.insert(name.clone(), value.clone());
            return Some(Event::Parameters { name, value });
        }
        if let Some(v) = parse_version(line) {
            if let Some(version) = v.version {
                self.settings.build.version = version;
            }
            if let Some(date) = v.date {
                self.settings.build.date = date;
            }
            self.settings.hardware = Hardware {
                mcu: v.mcu,
                sysclk: v.sysclk,
            };
            return Some(Event::Version);
        }
        Some(Event::Others)
    }

    pub fn is_idle(&self) -> bool {
        self.state.status.active_state == ACTIVE_STATE_IDLE
    }

    pub fn is_alarm(&self) -> bool {
        self.state.status.active_state == ACTIVE_STATE_ALARM
    }

    /// `Number(parserstate.tool) || 0`.
    pub fn tool(&self) -> f64 {
        let n = self
            .state
            .parserstate
            .tool
            .as_deref()
            .map_or(f64::NAN, js_str_to_num);
        if n.is_nan() {
            0.0
        } else {
            n
        }
    }

    /// Modal units word (`G20`/`G21`), empty when unknown.
    pub fn modal_units(&self) -> &str {
        match self
            .state
            .parserstate
            .modal
            .as_ref()
            .and_then(|m| m.get("units"))
        {
            Some(Word::One(units)) => units,
            _ => "",
        }
    }
}

/// `^<prefix>\s*(.+)$`
fn message_after(line: &str, prefix: &str) -> Option<String> {
    let message = line.strip_prefix(prefix)?.trim_start();
    (!message.is_empty()).then(|| message.to_string())
}

/// `ensureNumber(v) || 0`
fn num(v: Option<&str>) -> f64 {
    let n = v.map_or(0.0, js_str_to_num);
    if n.is_nan() {
        0.0
    } else {
        n
    }
}

static STATUS_PARAM_RE: LazyLock<Regex> = LazyLock::new(|| {
    const NUM: &str = r"[+-]?(?:[0-9]+(?:[.][0-9]*)?|[.][0-9]+)";
    Regex::new(&format!(r"[a-zA-Z]+(?::{NUM}(?:,{NUM}){{0,5}})?")).unwrap()
});

/// Old format `<Idle,MPos:x,y,z,a,b,c,WPos:x,y,z>`, new format
/// `<Run|MPos:x,y,z|WPos:x,y,z|F:current,requested,override|L:lp|S:sr|T:current,target>`.
fn parse_status(line: &str) -> Option<Status> {
    let body = line.strip_prefix('<')?.strip_suffix('>')?;
    if body.is_empty() {
        return None;
    }
    let mut params = STATUS_PARAM_RE.find_iter(body).map(|m| m.as_str());
    let active_state = params.next().unwrap_or("").split(':').next().unwrap_or("");
    let mut status = Status {
        active_state: active_state.to_string(),
        ..Default::default()
    };
    let (mut f, mut l, mut s, mut t) = (None, None, None, None);
    for param in params {
        let Some((name, value)) = param.split_once(':') else {
            continue;
        };
        match name {
            "MPos" => status.mpos = Some(Axes::parse(value)),
            "WPos" => status.wpos = Some(Axes::parse(value)),
            "F" => f = Some(value),
            "L" => l = Some(value),
            "S" => s = Some(value),
            "T" => t = Some(value),
            _ => {}
        }
    }
    if let Some(f) = f {
        let mut v = f.split(',');
        if matches!(active_state, "Home" | "Run") {
            status.current_feedrate = Some(num(v.next()));
        }
        status.feedrate = Some(num(v.next()));
        status.feedrate_override = Some(num(v.next()));
    }
    if let Some(l) = l {
        status.laser_power = Some(num(l.split(',').next()));
    }
    if let Some(s) = s {
        status.laser_intensity = Some(num(s.split(',').next()));
    }
    if let Some(t) = t {
        let mut v = t.split(',');
        status.current_temperature = Some(num(v.next()));
        status.target_temperature = Some(num(v.next()));
    }
    Some(status)
}

static PARSER_STATE_RE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^\[(?:GC:)?((?:[a-zA-Z][0-9]+(?:\.[0-9]*)?\s*)+)\]$").unwrap());

/// `[G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 F2540. S0.]`, optionally prefixed with `GC:`
/// (Smoothieware edge since 11/2020).
fn parse_parser_state(line: &str) -> Option<ParserState> {
    let caps = PARSER_STATE_RE.captures(line)?;
    let mut ps = ParserState::default();
    for word in caps[1].split(' ').map(str::trim).filter(|w| !w.is_empty()) {
        match word.as_bytes()[0] {
            b'G' | b'M' => {
                if let Some(group) = MODAL_GROUPS
                    .iter()
                    .position(|(_, modes)| modes.contains(&word))
                {
                    ps.modal
                        .get_or_insert_with(Modal::default)
                        .push(group, word);
                }
            }
            b'T' => ps.tool = Some(word[1..].to_string()),
            b'F' => ps.feedrate = Some(word[1..].to_string()),
            b'S' => ps.spindle = Some(word[1..].to_string()),
            _ => {}
        }
    }
    Some(ps)
}

static PARAMETERS_RE: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"^\[(G54|G55|G56|G57|G58|G59|G59.1|G59.2|G59.3|G28|G30|G92|TLO|PRB):(.+)\]$")
        .unwrap()
});
static GCODE_NAME_RE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"(?i)^G\d+(\.\d+)?$").unwrap());

/// `[G54:0.000,0.000,0.000]`, `[TLO:0.000]`, `[PRB:0.000,0.000,0.000:1]` (the suffix
/// tells whether the last probe cycle succeeded).
fn parse_parameters(line: &str) -> Option<(String, Value)> {
    let caps = PARAMETERS_RE.captures(line)?;
    let (name, value) = (&caps[1], &caps[2]);
    let value = if GCODE_NAME_RE.is_match(name) {
        serde_json::to_value(Axes::parse(value)).unwrap_or_default()
    } else if name == "TLO" {
        Value::String(value.to_string())
    } else if name == "PRB" {
        let mut parts = value.split(':');
        let list = parts.next().unwrap_or_default();
        let mut prb = Map::new();
        prb.insert(
            "result".into(),
            json_num(parts.next().map_or(f64::NAN, js_str_to_num)),
        );
        for (axis, v) in AXES.iter().zip(list.split(',')) {
            prb.insert((*axis).into(), Value::String(v.to_string()));
        }
        Value::Object(prb)
    } else {
        Value::String(String::new())
    };
    Some((name.to_string(), value))
}

struct Version {
    version: Option<String>,
    date: Option<String>,
    mcu: String,
    sysclk: Option<String>,
}

static VERSION_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"[a-zA-Z0-9\s]+:[^,]+").unwrap());

/// `Build version: edge-3332442, Build date: xxx, MCU: LPC1769, System Clock: 120MHz`
/// (an LPC1768/LPC1769 MCU means Smoothie).
fn parse_version(line: &str) -> Option<Version> {
    if !line.contains("LPC176") {
        return None;
    }
    let mut v = Version {
        version: None,
        date: None,
        mcu: String::new(),
        sysclk: None,
    };
    for m in VERSION_RE.find_iter(line) {
        let Some((name, value)) = m.as_str().trim_start().split_once(':') else {
            continue;
        };
        let value = value.trim_start();
        let name = name.to_ascii_lowercase();
        if name.contains("build version") {
            v.version = Some(value.to_string());
        }
        if name.contains("build date") {
            v.date = Some(value.to_string());
        }
        if name.contains("mcu") {
            v.mcu = value.to_string();
        }
        if name.contains("system clock") {
            v.sysclk = Some(value.to_string());
        }
    }
    // MCU is a required field
    (!v.mcu.is_empty()).then_some(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn status(line: &str) -> Value {
        let mut r = Runner::default();
        assert_eq!(r.parse(line), Some(Event::Status));
        json!(r.state.status)
    }

    #[test]
    fn status_reports() {
        assert_eq!(status("<Idle>"), json!({"activeState": "Idle"}));
        assert_eq!(
            status("<Idle,MPos:200.0000,200.0000,0.0000,0.0000,0.0000,0.0000,WPos:200.0000,200.0000,0.0000>"),
            json!({
                "activeState": "Idle",
                "mpos": {"x": "200.0000", "y": "200.0000", "z": "0.0000", "a": "0.0000", "b": "0.0000", "c": "0.0000"},
                "wpos": {"x": "200.0000", "y": "200.0000", "z": "0.0000"},
            })
        );
        assert_eq!(
            status("<Idle|MPos:200.0000,200.0000,0.0000|WPos:200.0000,200.0000,0.0000|F:4000.0,100.0|L:0.0000|S:0.8000>"),
            json!({
                "activeState": "Idle",
                "mpos": {"x": "200.0000", "y": "200.0000", "z": "0.0000"},
                "wpos": {"x": "200.0000", "y": "200.0000", "z": "0.0000"},
                "feedrate": 4000.0, "feedrateOverride": 100.0, "laserPower": 0.0, "laserIntensity": 0.8,
            })
        );
        // Home and Run report the current feedrate first.
        assert_eq!(
            status("<Run|MPos:15.8250,15.8250,0.0000|WPos:15.8250,15.8250,0.0000|F:4000.0,4000.0,100.0|T:21.5,200.0>"),
            json!({
                "activeState": "Run",
                "mpos": {"x": "15.8250", "y": "15.8250", "z": "0.0000"},
                "wpos": {"x": "15.8250", "y": "15.8250", "z": "0.0000"},
                "currentFeedrate": 4000.0, "feedrate": 4000.0, "feedrateOverride": 100.0,
                "currentTemperature": 21.5, "targetTemperature": 200.0,
            })
        );
    }

    #[test]
    fn replies() {
        let mut r = Runner::default();
        assert_eq!(r.parse("ok\r"), Some(Event::Ok));
        assert_eq!(
            r.parse("error: Expected command letter"),
            Some(Event::Error("Expected command letter".into()))
        );
        assert_eq!(
            r.parse("ALARM: Probe fail"),
            Some(Event::Alarm("Probe fail".into()))
        );
        assert_eq!(
            r.parse("// action:pause"),
            Some(Event::Action("pause".into()))
        );
        assert_eq!(r.parse("Not supported output format"), Some(Event::Others));
        assert_eq!(r.parse("  \r"), None);
    }

    #[test]
    fn parser_state() {
        let mut r = Runner::default();
        assert_eq!(
            r.parse("[G0 G54 G17 G21 G90 G94 M0 M5 M7 M8 T2 F2540. S0.]"),
            Some(Event::ParserState)
        );
        assert_eq!(
            json!(r.state.parserstate),
            json!({
                "modal": {"motion": "G0", "wcs": "G54", "plane": "G17", "units": "G21", "distance": "G90",
                          "feedrate": "G94", "program": "M0", "spindle": "M5", "coolant": ["M7", "M8"]},
                "tool": "2", "feedrate": "2540.", "spindle": "0.",
            })
        );
        assert_eq!(r.tool(), 2.0);
        r.parse("[GC:G1 G55 G17 G20 G90 G94 M0 M3 M9 T0 F2000.0 S2000.0]");
        assert_eq!((r.modal_units(), r.tool()), ("G20", 0.0));
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
                value: json!({"result": 1.0, "x": "0.000", "y": "0.000", "z": "1.492"})
            })
        );
        assert_eq!(r.settings.parameters["TLO"], json!("0.000"));
    }

    #[test]
    fn version() {
        let mut r = Runner::default();
        let line =
            "Build version: edge-3332442, Build date: xxx, MCU: LPC1769, System Clock: 120MHz";
        assert_eq!(r.parse(line), Some(Event::Version));
        assert_eq!(
            json!(r.settings),
            json!({"build": {"version": "edge-3332442", "date": "xxx"}, "hardware": {"mcu": "LPC1769", "sysclk": "120MHz"}, "parameters": {}})
        );
    }
}

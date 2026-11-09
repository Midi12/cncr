//! Marlin response parsing and machine state (port of `MarlinRunner`, the
//! `MarlinLineParserResult*` parsers and `interpret`).

use std::collections::BTreeMap;
use std::sync::LazyLock;

use cncr_core::gcode::{decimal_places, parse_line, WordValue};
use cncr_core::util::js_str_to_num;
use regex::Regex;
use serde::{Serialize, Serializer};

/// Serializes an `f64` like `JSON.stringify` does for JS numbers (`100`, not `100.0`).
fn js_num<S: Serializer>(n: &f64, s: S) -> Result<S::Ok, S::Error> {
    if n.fract() == 0.0 && n.abs() < 9_007_199_254_740_992.0 {
        s.serialize_i64(*n as i64)
    } else {
        s.serialize_f64(*n)
    }
}

/// Firmware report fields (`controller:settings`).
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Settings {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub firmware_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub protocol_version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub machine_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extruder_count: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uuid: Option<String>,
}

/// One heater (`{deg, degTarget, power}`), values kept as reported strings.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Heater {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deg: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deg_target: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub power: Option<String>,
}

impl Heater {
    /// `{...self, ...other}` for the keys `other` defines.
    fn merge(&mut self, other: &Heater) {
        for (dst, src) in [
            (&mut self.deg, &other.deg),
            (&mut self.deg_target, &other.deg_target),
            (&mut self.power, &other.power),
        ] {
            if src.is_some() {
                dst.clone_from(src);
            }
        }
    }
}

/// Temperature report. Probe/board/SoC/redundant sensors are parsed by the JS but
/// never used, so they are matched and dropped.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Temperature {
    pub ok: bool,
    pub extruder: Heater,
    pub heated_bed: Heater,
    pub heated_chamber: Heater,
    pub cooler: Heater,
    pub hotend: BTreeMap<String, Heater>,
    pub wait: Option<String>,
}

/// A parsed response line.
#[derive(Debug, Clone, PartialEq)]
pub enum Response {
    Start,
    Firmware(Settings),
    /// `(axis, value)` with lower-case axes, values normalized like the JS (`toFixed`).
    Position(Vec<(char, String)>),
    Echo(String),
    Error(String),
    Temperature(Box<Temperature>),
    /// `ok`, or an ADVANCED_OK `ok N10 P15 B3` (its buffer fields are unused).
    Ok,
    Others,
}

macro_rules! re {
    ($name:ident, $re:expr) => {
        static $name: LazyLock<Regex> = LazyLock::new(|| Regex::new($re).unwrap());
    };
}

re!(START_RE, r"^(?:echo:)?start$");
re!(FIRMWARE_RE, r"(?i)^FIRMWARE_NAME:");
re!(
    FIRMWARE_NAME_RE,
    r"FIRMWARE_NAME:([a-zA-Z_\-]+(\s+[0-9.]+)?)"
);
re!(PROTOCOL_VERSION_RE, r"PROTOCOL_VERSION:([0-9.]+)");
re!(MACHINE_TYPE_RE, r"MACHINE_TYPE:([A-Za-z0-9_]+)");
re!(EXTRUDER_COUNT_RE, r"EXTRUDER_COUNT:([0-9]+)");
re!(UUID_RE, r"UUID:([a-zA-Z0-9\-]+)");
re!(POSITION_RE, r"(?i)^(?:[XYZABCE]:[0-9.\-]+\s+)+");
re!(POSITION_PARAM_RE, r"(?i)([XYZABCE]):([0-9.\-]+)");
re!(ECHO_RE, r"(?i)^echo:\s*(.+)$");
re!(ERROR_RE, r"(?i)^Error:\s*(.+)$");
re!(
    TEMPERATURE_RE,
    r"(?i)^(ok)?\s+(T[0-9]+|T|B|C|L|P|M|S|R):[0-9.\-]+"
);
re!(
    TEMPERATURE_PARAM_RE,
    r"(?i)(?:(T[0-9]+|T|B|C|L|P|M|S|R):([0-9.\-]+)\s+/([0-9.\-]+)(?:\s+\([0-9.\-]+\))?)|(?:(@|B@|C@|@[0-9]+):([0-9.\-]+))|(?:(W):(\?|[0-9]+))"
);
re!(
    ADVANCED_OK_RE,
    r"(?i)^ok(?:\s+N[0-9]+)?\s+P[0-9]+\s+B[0-9]+"
);

fn capture(re: &Regex, line: &str) -> Option<String> {
    re.captures(line).map(|c| c[1].to_string())
}

/// `Number(pos).toFixed(decimalPlaces(pos))`.
fn normalize_position(pos: &str) -> String {
    let n = js_str_to_num(pos);
    if n.is_nan() {
        return "NaN".into();
    }
    // `+ 0.0` turns -0 into 0 like JS `toFixed`.
    format!("{:.*}", decimal_places(pos), n + 0.0)
}

fn parse_temperature(line: &str) -> Temperature {
    let mut t = Temperature {
        ok: line.starts_with("ok"),
        ..Default::default()
    };
    let s = |c: &regex::Captures, i: usize| c.get(i).map(|m| m.as_str().to_string());
    for c in TEMPERATURE_PARAM_RE.captures_iter(line) {
        let key = c
            .get(1)
            .or_else(|| c.get(4))
            .or_else(|| c.get(6))
            .map_or("", |m| m.as_str());
        if let Some(index) = key
            .strip_prefix('T')
            .filter(|i| !i.is_empty())
            .and_then(|i| i.parse::<u64>().ok())
        {
            let hotend = t.hotend.entry(format!("T{index}")).or_default();
            hotend.deg = s(&c, 2);
            hotend.deg_target = s(&c, 3);
            // The active hot end may be reported as T0 before B.
            if index == 0 {
                t.extruder.deg = t.extruder.deg.take().or_else(|| s(&c, 2));
                t.extruder.deg_target = t.extruder.deg_target.take().or_else(|| s(&c, 3));
            }
            continue;
        }
        match key {
            "T" => {
                t.extruder.deg = t.extruder.deg.take().or_else(|| s(&c, 2));
                t.extruder.deg_target = t.extruder.deg_target.take().or_else(|| s(&c, 3));
            }
            "B" => (t.heated_bed.deg, t.heated_bed.deg_target) = (s(&c, 2), s(&c, 3)),
            "C" => (t.heated_chamber.deg, t.heated_chamber.deg_target) = (s(&c, 2), s(&c, 3)),
            "L" => (t.cooler.deg, t.cooler.deg_target) = (s(&c, 2), s(&c, 3)),
            "@" => t.extruder.power = s(&c, 5),
            "B@" => t.heated_bed.power = s(&c, 5),
            // The first C@ is the heated chamber PWM, the second one the cooler's.
            "C@" => {
                if t.heated_chamber.deg.is_some() && t.heated_chamber.power.is_none() {
                    t.heated_chamber.power = s(&c, 5);
                } else if t.cooler.deg.is_some() && t.cooler.power.is_none() {
                    t.cooler.power = s(&c, 5);
                }
            }
            // M109/M190 print the remaining wait time every second.
            "W" => t.wait = s(&c, 7),
            _ => {
                if let Some(index) = key.strip_prefix('@').and_then(|i| i.parse::<u64>().ok()) {
                    t.hotend.entry(format!("T{index}")).or_default().power = s(&c, 5);
                }
            }
        }
    }
    t
}

/// Parses one response line (already right-trimmed). Port of `MarlinLineParser.parse`.
pub fn parse_response(line: &str) -> Response {
    if START_RE.is_match(line) {
        return Response::Start;
    }
    if FIRMWARE_RE.is_match(line) {
        return Response::Firmware(Settings {
            firmware_name: capture(&FIRMWARE_NAME_RE, line),
            protocol_version: capture(&PROTOCOL_VERSION_RE, line),
            machine_type: capture(&MACHINE_TYPE_RE, line),
            extruder_count: capture(&EXTRUDER_COUNT_RE, line).and_then(|n| n.parse().ok()),
            uuid: capture(&UUID_RE, line),
        });
    }
    if let Some(m) = POSITION_RE.find(line) {
        let pos = POSITION_PARAM_RE
            .captures_iter(m.as_str())
            .map(|c| {
                (
                    c[1].chars().next().unwrap().to_ascii_lowercase(),
                    normalize_position(&c[2]),
                )
            })
            .collect();
        return Response::Position(pos);
    }
    if let Some(c) = ECHO_RE.captures(line) {
        return Response::Echo(c[1].to_string());
    }
    if let Some(c) = ERROR_RE.captures(line) {
        return Response::Error(c[1].to_string());
    }
    if TEMPERATURE_RE.is_match(line) {
        return Response::Temperature(Box::new(parse_temperature(line)));
    }
    if line == "ok" || ADVANCED_OK_RE.is_match(line) {
        return Response::Ok;
    }
    Response::Others
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Pos {
    pub x: String,
    pub y: String,
    pub z: String,
    pub e: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub a: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub b: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub c: Option<String>,
}

impl Pos {
    /// `Number(axis) || 0` for x, y, z, a, b, c and e.
    pub fn numbers(&self) -> ([f64; 6], f64) {
        let n = |s: Option<&String>| {
            s.map_or(0.0, |s| {
                Some(js_str_to_num(s))
                    .filter(|v| !v.is_nan())
                    .unwrap_or(0.0)
            })
        };
        let axes = [
            Some(&self.x),
            Some(&self.y),
            Some(&self.z),
            self.a.as_ref(),
            self.b.as_ref(),
            self.c.as_ref(),
        ];
        (axes.map(n), n(Some(&self.e)))
    }
}

/// Active coolant words: one is serialized as a string, several (M7+M8) as an array.
#[derive(Debug, Clone, PartialEq)]
pub struct Coolant(pub Vec<String>);

impl Serialize for Coolant {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        match self.0.as_slice() {
            [one] => s.serialize_str(one),
            many => many.serialize(s),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Modal {
    pub motion: String,
    pub wcs: String,
    pub plane: String,
    pub units: String,
    pub distance: String,
    pub feedrate: String,
    pub program: String,
    pub spindle: String,
    pub coolant: Coolant,
}

/// `controller:state` payload.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct State {
    pub pos: Pos,
    pub modal: Modal,
    #[serde(rename = "ovF", serialize_with = "js_num")]
    pub ov_f: f64,
    #[serde(rename = "ovS", serialize_with = "js_num")]
    pub ov_s: f64,
    pub extruder: Heater,
    pub heated_bed: Heater,
    pub hotend: BTreeMap<String, Heater>,
    /// Related to G0.
    #[serde(serialize_with = "js_num")]
    pub rapid_feedrate: f64,
    /// Related to G1, G2, G3, G38.x, G80.
    #[serde(serialize_with = "js_num")]
    pub feedrate: f64,
    /// Related to M3, M4.
    #[serde(serialize_with = "js_num")]
    pub spindle: f64,
}

impl Default for State {
    fn default() -> Self {
        let s = |v: &str| v.to_string();
        Self {
            pos: Pos {
                x: s("0.000"),
                y: s("0.000"),
                z: s("0.000"),
                e: s("0.000"),
                a: None,
                b: None,
                c: None,
            },
            modal: Modal {
                motion: s("G0"),
                wcs: s("G54"),
                plane: s("G17"),
                units: s("G21"),
                distance: s("G90"),
                feedrate: s("G94"),
                program: s("M0"),
                spindle: s("M5"),
                coolant: Coolant(vec![s("M9")]),
            },
            ov_f: 100.0,
            ov_s: 100.0,
            extruder: Heater::default(),
            heated_bed: Heater::default(),
            hotend: BTreeMap::new(),
            rapid_feedrate: 0.0,
            feedrate: 0.0,
            spindle: 0.0,
        }
    }
}

fn set(dst: &mut String, v: &str) {
    dst.clear();
    dst.push_str(v);
}

/// Last numeric value of `letter` (`fromPairs`: the last word wins).
fn param(params: &[(char, WordValue)], letter: char) -> Option<f64> {
    params
        .iter()
        .rev()
        .find(|(l, _)| *l == letter)
        .and_then(|(_, v)| v.as_f64())
}

fn apply_command(state: &mut State, cmd: &str, params: &[(char, WordValue)]) {
    let m = &mut state.modal;
    match cmd {
        "G0" | "G1" | "G2" | "G3" | "G38.2" | "G38.3" | "G38.4" | "G38.5" | "G80" => {
            set(&mut m.motion, cmd);
            if let Some(f) = param(params, 'F') {
                if cmd == "G0" {
                    state.rapid_feedrate = f;
                } else {
                    state.feedrate = f;
                }
            }
        }
        "G54" | "G55" | "G56" | "G57" | "G58" | "G59" => set(&mut m.wcs, cmd),
        "G17" | "G18" | "G19" => set(&mut m.plane, cmd),
        "G20" | "G21" => set(&mut m.units, cmd),
        "G90" | "G91" => set(&mut m.distance, cmd),
        "G93" | "G94" => set(&mut m.feedrate, cmd),
        "M0" | "M1" | "M2" | "M30" => set(&mut m.program, cmd),
        "M3" | "M4" | "M5" => {
            set(&mut m.spindle, cmd);
            if cmd != "M5" {
                if let Some(s) = param(params, 'S') {
                    state.spindle = s;
                }
            }
        }
        "M7" | "M8" | "M9" => {
            let coolant = &mut m.coolant.0;
            if cmd == "M9" || *coolant == ["M9"] {
                coolant.clear();
                coolant.push(cmd.to_string());
            } else if !coolant.iter().any(|c| c == cmd) {
                coolant.push(cmd.to_string());
                coolant.sort();
            }
        }
        _ => {}
    }
}

#[derive(Debug, Default)]
pub struct Runner {
    pub state: State,
    pub settings: Settings,
    /// `interpret`'s previous command, reused by lines without a G/M word.
    last_cmd: String,
}

impl Runner {
    /// Parses one received line, updates the state and returns the response
    /// (`None` for blank lines).
    pub fn parse(&mut self, line: &str) -> Option<Response> {
        let line = line.trim_end();
        if line.is_empty() {
            return None;
        }
        let res = parse_response(line);
        match &res {
            Response::Firmware(settings) => self.settings.clone_from(settings),
            Response::Position(pos) => {
                let p = &mut self.state.pos;
                for (axis, v) in pos {
                    match axis {
                        'x' => p.x.clone_from(v),
                        'y' => p.y.clone_from(v),
                        'z' => p.z.clone_from(v),
                        'e' => p.e.clone_from(v),
                        'a' => p.a = Some(v.clone()),
                        'b' => p.b = Some(v.clone()),
                        _ => p.c = Some(v.clone()),
                    }
                }
            }
            Response::Temperature(t) => {
                self.state.extruder.merge(&t.extruder);
                self.state.heated_bed.merge(&t.heated_bed);
                for (k, v) in &t.hotend {
                    self.state.hotend.insert(k.clone(), v.clone());
                }
            }
            _ => {}
        }
        Some(res)
    }

    /// Tracks modal state, feed rates and spindle speed from a line written to the
    /// controller (port of `interpret` + the JS write filter).
    pub fn interpret(&mut self, line: &str) {
        let words = parse_line(line).words;
        let mut i = 0;
        while i < words.len() {
            let start = i;
            i += 1;
            while i < words.len() && !matches!(words[i].0, 'G' | 'M') {
                i += 1;
            }
            let group = &words[start..i];
            let params = match group[0] {
                (letter @ ('G' | 'M'), ref value) => {
                    self.last_cmd.clear();
                    self.last_cmd.push(letter);
                    self.last_cmd.push_str(&value.to_string());
                    &group[1..]
                }
                _ => group,
            };
            apply_command(&mut self.state, &self.last_cmd, params);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn heater(deg: &str, target: &str, power: &str) -> Heater {
        Heater {
            deg: Some(deg.into()),
            deg_target: Some(target.into()),
            power: Some(power.into()),
        }
    }

    fn temperature(line: &str) -> Temperature {
        match parse_response(line) {
            Response::Temperature(t) => *t,
            other => panic!("{line:?} parsed as {other:?}"),
        }
    }

    fn two_hotends(t0: &str, t1: &str, target: &str) -> BTreeMap<String, Heater> {
        BTreeMap::from([
            ("T0".to_string(), heater(t0, target, "0")),
            ("T1".to_string(), heater(t1, target, "0")),
        ])
    }

    #[test]
    fn simple_responses() {
        assert_eq!(
            parse_response("echo:message"),
            Response::Echo("message".into())
        );
        assert_eq!(
            parse_response("Error:Printer halted. kill() called!"),
            Response::Error("Printer halted. kill() called!".into())
        );
        assert_eq!(parse_response("ok"), Response::Ok);
        assert_eq!(parse_response("ok N10 P15 B3"), Response::Ok);
        assert_eq!(parse_response("start"), Response::Start);
        assert_eq!(parse_response("echo:start"), Response::Start);
        assert_eq!(parse_response("wait"), Response::Others);
    }

    #[test]
    fn firmware() {
        let line = "FIRMWARE_NAME:Marlin 1.1.0 (Github) SOURCE_CODE_URL:https://github.com/MarlinFirmware/Marlin PROTOCOL_VERSION:1.0 MACHINE_TYPE:RepRap EXTRUDER_COUNT:1 UUID:cede2a2f-41a2-4748-9b12-c55c62f367ff";
        let expected = Settings {
            firmware_name: Some("Marlin 1.1.0".into()),
            protocol_version: Some("1.0".into()),
            machine_type: Some("RepRap".into()),
            extruder_count: Some(1),
            uuid: Some("cede2a2f-41a2-4748-9b12-c55c62f367ff".into()),
        };
        let mut runner = Runner::default();
        assert_eq!(
            runner.parse(line),
            Some(Response::Firmware(expected.clone()))
        );
        assert_eq!(runner.settings, expected);
    }

    #[test]
    fn position() {
        let mut runner = Runner::default();
        let res = runner.parse("X:1.529 Y:-5.440 Z:0.00 E:0.00 Count X:0 Y:0 Z:0");
        let pos = |v: &[(char, &str)]| {
            v.iter()
                .map(|(a, s)| (*a, s.to_string()))
                .collect::<Vec<_>>()
        };
        assert_eq!(
            res,
            Some(Response::Position(pos(&[
                ('x', "1.529"),
                ('y', "-5.440"),
                ('z', "0.00"),
                ('e', "0.00")
            ])))
        );
        assert_eq!(
            serde_json::to_value(&runner.state.pos).unwrap(),
            json!({"x": "1.529", "y": "-5.440", "z": "0.00", "e": "0.00"})
        );

        let res = runner.parse("X:20.000 Y:41.000 Z:38.000 A:34.000 B:24.000 C:17.000 Count X:9311 Y:18922 Z:15200 A:536 B:378 C:268");
        assert_eq!(
            res,
            Some(Response::Position(pos(&[
                ('x', "20.000"),
                ('y', "41.000"),
                ('z', "38.000"),
                ('a', "34.000"),
                ('b', "24.000"),
                ('c', "17.000")
            ])))
        );
        assert_eq!(
            runner.state.pos.e, "0.00",
            "axes missing from the report keep their value"
        );
        assert_eq!(normalize_position("-0.000"), "0.000");
    }

    #[test]
    fn temperature_without_target_matches_nothing() {
        let t = temperature("ok T:0");
        assert!(t.ok);
        assert_eq!(
            t,
            Temperature {
                ok: true,
                ..Default::default()
            }
        );
    }

    #[test]
    fn temperature_single_hotend() {
        for (line, ok) in [
            ("ok T:293.0 /0.0 B:25.9 /0.0 @:0 B@:0", true),
            (" T:293.0 /0.0 B:25.9 /0.0 @:0 B@:0", false),
        ] {
            let t = temperature(line);
            assert_eq!(t.ok, ok);
            assert_eq!(t.extruder, heater("293.0", "0.0", "0"));
            assert_eq!(t.heated_bed, heater("25.9", "0.0", "0"));
            assert!(t.hotend.is_empty());
            assert_eq!(t.wait, None);
        }
    }

    #[test]
    fn temperature_multi_hotend() {
        for (line, ok, wait) in [
            ("ok T:293.0 /0.0 B:25.9 /0.0 T0:293.0 /0.0 T1:100.0 /0.0 @:0 B@:0 @0:0 @1:0", true, None),
            ("ok T:293.0 /0.0 (0.0) B:25.9 /0.0 T0:293.0 /0.0 (0.0) T1:100.0 /0.0 (0.0) @:0 B@:0 @0:0 @1:0", true, None),
            ("ok T:293.0 /0.0 (0.0) B:25.9 /0.0 T0:293.0 /0.0 (0.0) T1:100.0 /0.0 (0.0) @:0 B@:0 @0:0 @1:0 W:?", true, Some("?")),
            ("ok T:293.0 /0.0 (0.0) B:25.9 /0.0 T0:293.0 /0.0 (0.0) T1:100.0 /0.0 (0.0) @:0 B@:0 @0:0 @1:0 W:0", true, Some("0")),
            (" T:293.0 /0.0 B:25.9 /0.0 T0:293.0 /0.0 T1:100.0 /0.0 @:0 B@:0 @0:0 @1:0", false, None),
            (" T:293.0 /0.0 (0.0) B:25.9 /0.0 T0:293.0 /0.0 (0.0) T1:100.0 /0.0 (0.0) @:0 B@:0 @0:0 @1:0", false, None),
        ] {
            let t = temperature(line);
            assert_eq!(t.ok, ok, "{line}");
            assert_eq!(t.extruder, heater("293.0", "0.0", "0"), "{line}");
            assert_eq!(t.heated_bed, heater("25.9", "0.0", "0"), "{line}");
            assert_eq!(t.hotend, two_hotends("293.0", "100.0", "0.0"), "{line}");
            assert_eq!(t.wait.as_deref(), wait, "{line}");
        }
    }

    #[test]
    fn temperature_active_hotend_reported_as_t0() {
        let t = temperature(
            "ok T0:27.58 /0.00 B:28.27 /0.00 T0:27.58 /0.00 T1:27.37 /0.00 @:0 B@:0 @0:0 @1:0",
        );
        assert_eq!(t.extruder, heater("27.58", "0.00", "0"));
        assert_eq!(t.heated_bed, heater("28.27", "0.00", "0"));
        assert_eq!(t.hotend, two_hotends("27.58", "27.37", "0.00"));

        // The first T0 is the active hot end; the later T0 entry updates the hotend map only.
        let t = temperature(
            " T0:27.28 /0.00 B:28.38 /0.00 T0:27.72 /0.00 T1:27.28 /0.00 @:0 B@:0 @0:0 @1:0",
        );
        assert!(!t.ok);
        assert_eq!(t.extruder, heater("27.28", "0.00", "0"));
        assert_eq!(t.heated_bed, heater("28.38", "0.00", "0"));
        assert_eq!(t.hotend, two_hotends("27.72", "27.28", "0.00"));
    }

    #[test]
    fn temperature_chamber_and_cooler_pwm() {
        let t = temperature("ok L:21.84 /0.00 @:0 C@:0");
        assert_eq!(
            t.extruder,
            Heater {
                power: Some("0".into()),
                ..Default::default()
            }
        );
        assert_eq!(t.cooler, heater("21.84", "0.00", "0"));

        let t = temperature("ok C:25.9 /0.00 L:21.84 /0.00 @:0 C@:127 C@:0");
        assert_eq!(t.heated_chamber, heater("25.9", "0.00", "127"));
        assert_eq!(t.cooler, heater("21.84", "0.00", "0"));
    }

    #[test]
    fn temperature_merges_into_state() {
        let mut runner = Runner::default();
        runner.parse("ok T:20.0 /0.0 B:25.9 /0.0 @:0 B@:0");
        runner.parse(" T:21.0 /0.0 T0:21.0 /0.0");
        assert_eq!(
            runner.state.extruder,
            heater("21.0", "0.0", "0"),
            "power kept from the previous report"
        );
        assert_eq!(
            serde_json::to_value(&runner.state.hotend).unwrap(),
            json!({"T0": {"deg": "21.0", "degTarget": "0.0"}})
        );
    }

    #[test]
    fn interpret_tracks_modal_state() {
        let mut runner = Runner::default();
        runner.interpret("G20 G91 G1 X1 F200");
        runner.interpret("G0 X0 F1000");
        runner.interpret("M3 S1200");
        runner.interpret("M8");
        runner.interpret("M7");
        let s = &runner.state;
        assert_eq!(
            (
                s.modal.units.as_str(),
                s.modal.distance.as_str(),
                s.modal.motion.as_str()
            ),
            ("G20", "G91", "G0")
        );
        assert_eq!(
            (s.feedrate, s.rapid_feedrate, s.spindle),
            (200.0, 1000.0, 1200.0)
        );
        assert_eq!(
            serde_json::to_value(&s.modal.coolant).unwrap(),
            json!(["M7", "M8"])
        );

        // Continuation lines reuse the previous command.
        runner.interpret("G1 X1");
        runner.interpret("X2 F300");
        assert_eq!(runner.state.feedrate, 300.0);

        runner.interpret("M9");
        assert_eq!(
            serde_json::to_value(&runner.state.modal.coolant).unwrap(),
            json!("M9")
        );
        runner.interpret("G38.2 Z-5 F10");
        assert_eq!(runner.state.modal.motion, "G38.2");
    }

    #[test]
    fn state_serializes_like_js() {
        let v = serde_json::to_value(State::default()).unwrap();
        assert_eq!(v["ovF"], json!(100));
        assert_eq!(v["modal"]["coolant"], json!("M9"));
        assert_eq!(v["extruder"], json!({}));
        assert_eq!(v["rapidFeedrate"], json!(0));
    }
}

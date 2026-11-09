//! TinyG / g2core JSON response parser and machine state (port of `TinyGRunner` and
//! the `TinyGLineParserResult*` classes).

use serde::Serialize;
use serde_json::{json, Map, Value};

use cncr_core::util::num_or_zero;

use crate::constants::*;

/// JS truthiness of a JSON value.
pub(crate) fn truthy(v: &Value) -> bool {
    match v {
        Value::Null => false,
        Value::Bool(b) => *b,
        Value::Number(n) => n.as_f64().is_some_and(|n| n != 0.0 && !n.is_nan()),
        Value::String(s) => !s.is_empty(),
        _ => true,
    }
}

/// `_.get(data, 'r.<key>') || _.get(data, '<key>')`, kept only when truthy.
pub(crate) fn either<'a>(data: &'a Value, key: &str) -> Option<&'a Value> {
    data.get("r")
        .and_then(|r| r.get(key))
        .filter(|v| truthy(v))
        .or_else(|| data.get(key))
        .filter(|v| truthy(v))
}

/// JSON number the way JS prints it (integers without a fraction).
pub(crate) fn js_num(n: f64) -> Value {
    if n.fract() == 0.0 && n.abs() < 9e15 {
        json!(n as i64)
    } else {
        cncr_core::util::json_num(n)
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
    pub path: String,
    pub spindle: String,
    /// `"M7"`, `"M8"`, `"M9"` or `["M7", "M8"]`.
    pub coolant: Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arcdistance: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Sr {
    pub machine_state: Value,
    pub velocity: Value,
    pub line: Value,
    pub feedrate: Value,
    pub mpos: Map<String, Value>,
    pub wpos: Map<String, Value>,
    pub modal: Modal,
    pub tool: Value,
    pub tlo: Map<String, Value>,
    pub spe: Value,
    pub spd: Value,
    pub spc: Value,
    pub sps: Value,
}

/// `controller:state` payload.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct State {
    pub mt: Value,
    pub pwr: Value,
    pub qr: Value,
    pub sr: Sr,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Footer {
    pub revision: Value,
    pub status_code: Value,
    pub rx_buffer_info: Value,
}

/// The parts of a response the controller acts on.
#[derive(Debug, PartialEq)]
pub enum Report {
    Other,
    /// Queue report: free planner buffers.
    Qr(f64),
    /// Receive report (`{"r":{...}}` not claimed by a more specific parser).
    R(Value),
}

fn pos(x: &str) -> Map<String, Value> {
    ["x", "y", "z"]
        .iter()
        .map(|a| ((*a).to_string(), json!(x)))
        .collect()
}

fn lookup(v: &Value, table: &[&str]) -> String {
    v.as_f64()
        .filter(|n| n.fract() == 0.0 && *n >= 0.0)
        .and_then(|n| table.get(n as usize))
        .copied()
        .unwrap_or_default()
        .to_string()
}

fn has_coolant(c: &Value, word: &str) -> bool {
    match c {
        Value::Array(a) => a.iter().any(|x| x == word),
        Value::String(s) => s == word,
        _ => false,
    }
}

impl Sr {
    fn update_spindle(&mut self) {
        self.modal.spindle = if !truthy(&self.spe) {
            "M5"
        } else if self.spd.as_f64() == Some(0.0) {
            "M3"
        } else {
            "M4"
        }
        .into();
    }

    fn update_coolant(&mut self, v: &Value, own: &str, other: &str) {
        self.modal.coolant = if v.as_f64() == Some(0.0) {
            json!("M9")
        } else if has_coolant(&self.modal.coolant, other) {
            json!(["M7", "M8"])
        } else {
            json!(own)
        };
    }

    /// Applies a status report in the JS keymap order.
    fn apply(&mut self, p: &Value) {
        let set = |dst: &mut Value, k: &str| {
            if let Some(v) = p.get(k) {
                *dst = v.clone();
            }
        };
        set(&mut self.machine_state, "stat");
        set(&mut self.line, "line");
        set(&mut self.velocity, "vel");
        set(&mut self.feedrate, "feed");
        let modal = [
            ("unit", GCODE_UNITS),
            ("coor", GCODE_COORDINATE),
            ("momo", GCODE_MOTION),
            ("plan", GCODE_PLANE),
            ("path", GCODE_PATH),
            ("dist", GCODE_DISTANCE),
            ("admo", GCODE_DISTANCE),
            ("frmo", GCODE_FEEDRATE),
        ];
        for (k, table) in modal {
            let Some(v) = p.get(k) else { continue };
            let g = lookup(v, table);
            let m = &mut self.modal;
            match k {
                "unit" => m.units = g,
                "coor" => m.wcs = g,
                "momo" => m.motion = g,
                "plan" => m.plane = g,
                "path" => m.path = g,
                "dist" => m.distance = g,
                "admo" => m.arcdistance = Some(g),
                _ => m.feedrate = g,
            }
        }
        set(&mut self.tool, "tool");
        if let Some(v) = p.get("spe") {
            self.spe = v.clone();
            self.update_spindle();
        }
        if let Some(v) = p.get("spd") {
            self.spd = v.clone();
            self.update_spindle();
        }
        if let Some(v) = p.get("spc") {
            match v.as_f64() {
                Some(0.0) => self.modal.spindle = "M5".into(),
                Some(1.0) => self.modal.spindle = "M3".into(),
                Some(2.0) => self.modal.spindle = "M4".into(),
                _ => {}
            }
        }
        set(&mut self.sps, "sps");
        if let Some(v) = p.get("com") {
            self.update_coolant(v, "M7", "M8");
        }
        if let Some(v) = p.get("cof") {
            self.update_coolant(v, "M8", "M7");
        }
        for (prefix, map) in [
            ("tof", &mut self.tlo),
            ("pos", &mut self.wpos),
            ("mpo", &mut self.mpos),
        ] {
            for axis in ["x", "y", "z", "a", "b", "c"] {
                if let Some(v) = p.get(format!("{prefix}{axis}").as_str()) {
                    map.insert(axis.to_string(), v.clone());
                }
            }
        }
    }
}

#[derive(Debug)]
pub struct Runner {
    pub state: State,
    pub settings: Map<String, Value>,
    pub footer: Footer,
    /// Largest queue report seen: the planner buffer pool size.
    pub planner_buffer_pool_size: f64,
    state_changed: bool,
    settings_changed: bool,
}

impl Default for Runner {
    fn default() -> Self {
        let settings = json!({
            "fb": 0, "fbs": "", "fbc": "", "fv": 0, "hp": 0, "hv": 0, "id": "",
            "mfo": 1, "mto": 1, "sso": 1,
        });
        Self {
            state: State {
                mt: json!(0),
                pwr: json!({}),
                qr: json!(0),
                sr: Sr {
                    machine_state: json!(""),
                    velocity: json!(0),
                    line: json!(0),
                    feedrate: json!(0),
                    mpos: pos("0.000"),
                    wpos: pos("0.000"),
                    modal: Modal {
                        motion: String::new(),
                        wcs: String::new(),
                        plane: String::new(),
                        units: String::new(),
                        distance: String::new(),
                        feedrate: String::new(),
                        path: String::new(),
                        spindle: String::new(),
                        coolant: json!(""),
                        arcdistance: None,
                    },
                    tool: json!(0),
                    tlo: ["x", "y", "z"]
                        .iter()
                        .map(|a| ((*a).to_string(), json!(0)))
                        .collect(),
                    spe: json!(0),
                    spd: json!(0),
                    spc: json!(0),
                    sps: json!(0),
                },
            },
            settings: match settings {
                Value::Object(m) => m,
                _ => unreachable!(),
            },
            footer: Footer {
                revision: json!(0),
                status_code: json!(0),
                rx_buffer_info: json!(0),
            },
            planner_buffer_pool_size: 0.0,
            // The controller has emitted nothing yet.
            state_changed: true,
            settings_changed: true,
        }
    }
}

impl Runner {
    /// Parses one trimmed, non-empty line. Returns the report and the footer (`f`), if any.
    pub fn parse(&mut self, data: &str) -> (Report, Option<Vec<Value>>) {
        if !data.starts_with('{') {
            return (Report::Other, None);
        }
        let mut data: Value = serde_json::from_str(data).unwrap_or_else(|_| json!({}));
        let footer = match data.get_mut("f").map(Value::take) {
            Some(Value::Array(f)) if !f.is_empty() => Some(f),
            _ => None,
        };
        let status_ok = footer
            .as_ref()
            .and_then(|f| f.get(1))
            .and_then(Value::as_f64)
            == Some(0.0);
        let report = self.parse_report(&mut data, status_ok);
        if let Some(f) = &footer {
            let at = |i: usize| f.get(i).cloned().unwrap_or(Value::Null);
            self.footer = Footer {
                revision: at(0),
                status_code: at(1),
                rx_buffer_info: at(2),
            };
        }
        (report, footer)
    }

    fn parse_report(&mut self, data: &mut Value, status_ok: bool) -> Report {
        let r = data.get("r");
        if let Some(mt) = r.and_then(|r| r.get("mt")) {
            if truthy(mt) && status_ok && self.state.mt != *mt {
                self.state.mt = mt.clone();
                self.state_changed = true;
            }
            return Report::Other;
        }
        if let Some(pwr) = r.and_then(|r| r.get("pwr")) {
            if truthy(pwr) && status_ok && self.state.pwr != *pwr {
                self.state.pwr = pwr.clone();
                self.state_changed = true;
            }
            return Report::Other;
        }
        if let Some(qr) = either(data, "qr") {
            let qr = num_or_zero(Some(qr));
            if qr > self.planner_buffer_pool_size {
                self.planner_buffer_pool_size = qr;
            }
            let v = js_num(qr);
            if self.state.qr != v {
                self.state.qr = v;
                self.state_changed = true;
            }
            return Report::Qr(qr);
        }
        if let Some(sr) = either(data, "sr") {
            let mut next = self.state.sr.clone();
            next.apply(sr);
            if next != self.state.sr {
                self.state.sr = next;
                self.state_changed = true;
            }
            return Report::Other;
        }
        if let Some(sys) = either(data, "sys") {
            if let Value::Object(sys) = sys {
                for (k, v) in sys {
                    self.settings.insert(k.clone(), v.clone());
                }
            }
            self.settings_changed = true;
            return Report::Other;
        }
        if let Some(r) = r.filter(|r| ["mfo", "mto", "sso"].iter().any(|k| r.get(k).is_some())) {
            for k in ["mfo", "mto", "sso"] {
                if let Some(v) = r.get(k).filter(|v| truthy(v) && status_ok) {
                    self.settings.insert(k.to_string(), v.clone());
                }
            }
            self.settings_changed = true;
            return Report::Other;
        }
        let Some(r) = data.get_mut("r") else {
            return Report::Other;
        };
        let inner = r.get_mut("r").filter(|v| truthy(v)).map(Value::take);
        let r = inner.unwrap_or_else(|| r.take());
        if !truthy(&r) {
            return Report::Other;
        }
        if let Value::Object(m) = &r {
            for (k, v) in m {
                if let Some(s) = self.settings.get_mut(k) {
                    *s = v.clone();
                    self.settings_changed = true;
                }
            }
        }
        Report::R(r)
    }

    /// True once after the state changed (the JS runner replaced its state object).
    pub fn take_state_changed(&mut self) -> bool {
        std::mem::take(&mut self.state_changed)
    }

    /// True once after the settings were replaced.
    pub fn take_settings_changed(&mut self) -> bool {
        std::mem::take(&mut self.settings_changed)
    }

    pub fn is_alarm(&self) -> bool {
        self.state.sr.machine_state.as_f64() == Some(MACHINE_STATE_ALARM)
    }

    pub fn is_idle(&self) -> bool {
        matches!(self.state.sr.machine_state.as_f64(), Some(s) if s == MACHINE_STATE_READY || s == MACHINE_STATE_STOP || s == MACHINE_STATE_END)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motor_timeout_and_power_need_a_zero_status_code() {
        let mut r = Runner::default();
        r.parse("{\"r\":{\"mt\":2},\"f\":[1,0,8]}");
        assert_eq!(r.state.mt, json!(2));
        r.parse("{\"r\":{\"mt\":5},\"f\":[1,100,8]}");
        assert_eq!(r.state.mt, json!(2), "error responses are ignored");
        r.parse("{\"r\":{\"pwr\":{\"1\":0,\"2\":0,\"3\":0,\"4\":0}},\"f\":[1,0,9]}");
        assert_eq!(r.state.pwr, json!({"1": 0, "2": 0, "3": 0, "4": 0}));
    }

    #[test]
    fn queue_reports_track_the_pool_size() {
        let mut r = Runner::default();
        assert_eq!(r.parse("{\"qr\":48}").0, Report::Qr(48.0));
        assert_eq!(
            r.parse("{\"r\":{\"qr\":32,\"qi\":0,\"qo\":1},\"f\":[1,0,8]}")
                .0,
            Report::Qr(32.0)
        );
        assert_eq!(r.state.qr, json!(32));
        assert_eq!(r.planner_buffer_pool_size, 48.0);
    }

    #[test]
    fn status_reports_update_state() {
        let mut r = Runner::default();
        r.take_state_changed();
        r.parse("{\"sr\":{\"line\":8,\"stat\":5,\"vel\":688.81,\"posx\":0.248,\"mpoy\":1.5,\"posa\":3,\"tool\":3}}");
        assert!(r.take_state_changed());
        let sr = &r.state.sr;
        assert_eq!(
            (
                sr.line.clone(),
                sr.machine_state.clone(),
                sr.velocity.clone()
            ),
            (json!(8), json!(5), json!(688.81))
        );
        assert_eq!(
            sr.wpos,
            serde_json::from_value::<Map<String, Value>>(
                json!({"x": 0.248, "y": "0.000", "z": "0.000", "a": 3})
            )
            .unwrap()
        );
        assert_eq!(sr.mpos["y"], json!(1.5));
        assert_eq!(sr.tool, json!(3));
        r.parse("{\"sr\":{\"line\":8}}");
        assert!(!r.take_state_changed(), "no change, no new state");
    }

    #[test]
    fn status_report_modal_groups() {
        let mut r = Runner::default();
        r.parse("{\"r\":{\"sr\":{\"unit\":0,\"coor\":2,\"momo\":1,\"plan\":0,\"path\":1,\"dist\":1,\"admo\":0,\"frmo\":1,\"spe\":1,\"spd\":1}},\"f\":[1,0,8]}");
        let m = &r.state.sr.modal;
        assert_eq!(
            [
                &m.units,
                &m.wcs,
                &m.motion,
                &m.plane,
                &m.path,
                &m.distance,
                &m.feedrate,
                &m.spindle
            ],
            ["G20", "G55", "G1", "G17", "G61.1", "G91", "G94", "M4"]
        );
        assert_eq!(m.arcdistance.as_deref(), Some("G90"));
        r.parse("{\"sr\":{\"com\":1,\"cof\":1}}");
        assert_eq!(r.state.sr.modal.coolant, json!(["M7", "M8"]));
        r.parse("{\"sr\":{\"cof\":0,\"spc\":1}}");
        assert_eq!(r.state.sr.modal.coolant, json!("M9"));
        assert_eq!(r.state.sr.modal.spindle, "M3");
        r.parse("{\"sr\":{\"spe\":0}}");
        assert_eq!(r.state.sr.modal.spindle, "M5");
    }

    #[test]
    fn settings_from_sys_overrides_and_receive_reports() {
        let mut r = Runner::default();
        r.take_settings_changed();
        r.parse("{\"r\":{\"sys\":{\"fb\":100.19,\"fv\":0.99,\"jt\":0.75}},\"f\":[1,0,9]}");
        assert!(r.take_settings_changed());
        assert_eq!(
            (r.settings["fb"].clone(), r.settings["jt"].clone()),
            (json!(100.19), json!(0.75))
        );
        r.parse("{\"r\":{\"mfo\":1.5,\"sso\":0},\"f\":[1,0,21]}");
        assert_eq!(
            (r.settings["mfo"].clone(), r.settings["sso"].clone()),
            (json!(1.5), json!(1))
        );
        let (report, f) = r.parse("{\"r\":{\"fv\":1.0,\"xyz\":1},\"f\":[1,0,4]}");
        assert_eq!(report, Report::R(json!({"fv": 1.0, "xyz": 1})));
        assert_eq!(f, Some(vec![json!(1), json!(0), json!(4)]));
        assert_eq!(r.settings["fv"], json!(1.0));
        assert!(
            !r.settings.contains_key("xyz"),
            "only known settings are taken from receive reports"
        );
        assert_eq!(r.footer.rx_buffer_info, json!(4));
    }

    #[test]
    fn empty_receive_report_still_acknowledges() {
        let mut r = Runner::default();
        assert_eq!(r.parse("{\"r\":{},\"f\":[1,0,4]}").0, Report::R(json!({})));
        assert_eq!(r.parse("ok>").0, Report::Other);
        assert_eq!(r.parse("{bad json").1, None);
    }
}

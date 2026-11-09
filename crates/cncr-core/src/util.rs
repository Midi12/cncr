//! Small helpers shared by every controller.

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde_json::Value;

/// Milliseconds since the Unix epoch (JS `Date.now()`).
pub fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

/// JS `String(number)` for the values that occur in G-code: integers print without a
/// fraction, `-0` prints as `0`, everything else uses the shortest round-trip form.
pub fn js_num_to_string(n: f64) -> String {
    if n.is_nan() {
        return "NaN".into();
    }
    if n.is_infinite() {
        return if n > 0.0 { "Infinity" } else { "-Infinity" }.into();
    }
    if n == 0.0 {
        return "0".into();
    }
    if n.fract() == 0.0 && n.abs() < 1e21 {
        return format!("{}", n as i128);
    }
    format!("{n}")
}

/// JS `Number(string)`: trims, empty string is 0, otherwise a full decimal literal or NaN.
pub fn js_str_to_num(s: &str) -> f64 {
    let t = s.trim();
    if t.is_empty() {
        return 0.0;
    }
    match t {
        "Infinity" | "+Infinity" => return f64::INFINITY,
        "-Infinity" => return f64::NEG_INFINITY,
        _ => {}
    }
    if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        return i64::from_str_radix(hex, 16)
            .map(|v| v as f64)
            .unwrap_or(f64::NAN);
    }
    // Rust accepts "inf"/"nan"/"infinity" which JS rejects.
    if t.bytes()
        .any(|b| b.is_ascii_alphabetic() && b != b'e' && b != b'E')
    {
        return f64::NAN;
    }
    t.parse::<f64>().unwrap_or(f64::NAN)
}

/// JS `parseFloat`: longest numeric prefix, NaN if none.
pub fn js_parse_float(s: &str) -> f64 {
    let t = s.trim_start();
    let b = t.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    if t[i..].starts_with("Infinity") {
        return if b.first() == Some(&b'-') {
            f64::NEG_INFINITY
        } else {
            f64::INFINITY
        };
    }
    let digits_start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    if i < b.len() && b[i] == b'.' {
        i += 1;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
    }
    if i == digits_start || (i == digits_start + 1 && b[digits_start] == b'.') {
        return f64::NAN;
    }
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let mut j = i + 1;
        if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
            j += 1;
        }
        let exp_start = j;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_start {
            i = j;
        }
    }
    t[..i].parse::<f64>().unwrap_or(f64::NAN)
}

/// JS `Number(value) || 0` for a JSON value (strings are parsed, NaN becomes 0).
pub fn num_or_zero(v: Option<&Value>) -> f64 {
    let n = match v {
        Some(Value::Number(n)) => n.as_f64().unwrap_or(0.0),
        Some(Value::String(s)) => js_str_to_num(s),
        Some(Value::Bool(b)) => f64::from(u8::from(*b)),
        _ => 0.0,
    };
    if n.is_nan() {
        0.0
    } else {
        n
    }
}

/// JSON number from an `f64`, `null` when not finite (what `JSON.stringify` does).
pub fn json_num(n: f64) -> Value {
    serde_json::Number::from_f64(n)
        .map(Value::Number)
        .unwrap_or(Value::Null)
}

/// JS `Number(x.toFixed(digits))`: round half away from zero to `digits` decimals.
pub fn round_to(n: f64, digits: i32) -> f64 {
    let s = format!("{:.*}", digits.max(0) as usize, n);
    s.parse().unwrap_or(n)
}

/// `SystemTime` formatted like `JSON.stringify(new Date(...))`: `2024-01-31T12:34:56.789Z`.
pub fn iso8601(t: SystemTime) -> String {
    let ms = match t.duration_since(UNIX_EPOCH) {
        Ok(d) => d.as_millis() as i64,
        Err(e) => -(e.duration().as_millis() as i64),
    };
    let days = ms.div_euclid(86_400_000);
    let rem = ms.rem_euclid(86_400_000);
    // Howard Hinnant's civil_from_days.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}.{:03}Z",
        rem / 3_600_000,
        rem / 60_000 % 60,
        rem / 1000 % 60,
        rem % 1000
    )
}

/// Random short identifier (task ids), like `shortid.generate()`.
pub fn short_id() -> String {
    use rand::Rng;
    const ALPHABET: &[u8] = b"0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_-";
    let mut rng = rand::rng();
    (0..10)
        .map(|_| ALPHABET[rng.random_range(0..ALPHABET.len())] as char)
        .collect()
}

/// Holds one pending `%msg` value until the next pause consumes it.
#[derive(Debug, Default)]
pub struct MessageSlot(Option<Value>);

impl MessageSlot {
    pub fn put(&mut self, value: Value) {
        self.0 = Some(value);
    }

    /// Takes the stored value (`null` counts as empty) or returns `default`.
    pub fn take_or(&mut self, default: Value) -> Value {
        match self.0.take() {
            Some(Value::Null) | None => default,
            Some(v) => v,
        }
    }
}

/// Rate limiter replacing lodash `throttle` for functions that are polled from the
/// 250 ms controller timer: at most one run per `interval`.
/// `leading = false` delays the first run by one interval (lodash `{ leading: false }`).
#[derive(Debug)]
pub struct Throttle {
    interval: Duration,
    leading: bool,
    window_start: Option<Instant>,
}

impl Throttle {
    pub fn new(interval: Duration, leading: bool) -> Self {
        Self {
            interval,
            leading,
            window_start: None,
        }
    }

    /// Returns true when the throttled function should run now.
    pub fn ready(&mut self) -> bool {
        let now = Instant::now();
        match self.window_start {
            None => {
                self.window_start = Some(now);
                self.leading
            }
            Some(start) if now.duration_since(start) >= self.interval => {
                self.window_start = Some(now);
                true
            }
            Some(_) => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn number_formatting_matches_js() {
        assert_eq!(js_num_to_string(2.0), "2");
        assert_eq!(js_num_to_string(-0.0), "0");
        assert_eq!(js_num_to_string(0.1 + 0.2), "0.30000000000000004");
        assert_eq!(js_num_to_string(-12.5), "-12.5");
        assert_eq!(round_to(1.23456, 3), 1.235);
    }

    #[test]
    fn js_number_parsing() {
        assert_eq!(js_str_to_num(" 12.5 "), 12.5);
        assert_eq!(js_str_to_num(""), 0.0);
        assert!(js_str_to_num("12abc").is_nan());
        assert!(js_str_to_num("inf").is_nan());
        assert_eq!(js_parse_float("12.5mm"), 12.5);
        assert_eq!(js_parse_float("-.5"), -0.5);
        assert!(js_parse_float("abc").is_nan());
    }

    #[test]
    fn iso_dates() {
        let t = UNIX_EPOCH + Duration::from_millis(1_706_704_496_789);
        assert_eq!(iso8601(t), "2024-01-31T12:34:56.789Z");
    }
}

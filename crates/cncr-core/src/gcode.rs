//! G-code line helpers: a port of `gcode-parser`'s `parseLine`, the M0/M1/M6 matchers,
//! M6 replacement, the `%msg` / `%wait` builtin commands and unit conversions.

use std::sync::LazyLock;

use regex::Regex;

use crate::util::js_num_to_string;
use crate::{IMPERIAL_UNITS, METRIC_UNITS};

/// Value of a parsed word: a number, or the raw text when it is not numeric (`X1-2`).
#[derive(Debug, Clone, PartialEq)]
pub enum WordValue {
    Num(f64),
    Text(String),
}

impl std::fmt::Display for WordValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WordValue::Num(n) => f.write_str(&js_num_to_string(*n)),
            WordValue::Text(s) => f.write_str(s),
        }
    }
}

impl WordValue {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            WordValue::Num(n) => Some(*n),
            WordValue::Text(_) => None,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ParsedLine {
    /// Comments removed, trimmed, inner whitespace kept (`lineMode: 'stripped'`).
    pub stripped: String,
    /// Comments and all whitespace removed (`lineMode: 'compact'`).
    pub compact: String,
    /// `(letter, value)` words, letters upper-cased; `N` and `*` excluded.
    pub words: Vec<(char, WordValue)>,
    /// `%...`, `{...}` and `$...` commands.
    pub cmds: Vec<String>,
    /// Comment texts.
    pub comments: Vec<String>,
    /// Line number (`N` word).
    pub ln: Option<f64>,
}

impl ParsedLine {
    /// Words flattened like `parseLine(line, { flatten: true })`: `G0`, `X1.5`.
    pub fn flat_words(&self) -> Vec<String> {
        self.words.iter().map(|(l, v)| format!("{l}{v}")).collect()
    }
}

/// Removes `( ... )` (nested) and `; ...` comments outside parentheses.
fn strip_comments(line: &str) -> (String, Vec<String>) {
    let mut result = String::with_capacity(line.len());
    let mut current = String::new();
    let mut comments = Vec::new();
    let mut open = 0usize;
    for (i, c) in line.char_indices() {
        if c == ';' && open == 0 {
            comments.push(line[i + 1..].trim().to_string());
            break;
        }
        match c {
            '(' => {
                if open > 0 {
                    current.push(c);
                } else {
                    current.clear();
                }
                open += 1;
            }
            ')' => {
                open = open.saturating_sub(1);
                if open == 0 {
                    comments.push(current.trim().to_string());
                    current.clear();
                } else {
                    current.push(c);
                }
            }
            _ if open > 0 => current.push(c),
            _ => result.push(c),
        }
    }
    (result.trim().to_string(), comments)
}

static WORD_RE: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)(%.*)|(\{.*)|((?:\$\$)|(?:\$[a-zA-Z0-9#]*))|([a-zA-Z][0-9+\-.]+)|(\*[0-9]+)")
        .unwrap()
});

/// Parses one G-code line (port of `gcode-parser` 2.x `parseLine`).
pub fn parse_line(line: &str) -> ParsedLine {
    let (stripped, comments) = strip_comments(line);
    let compact: String = stripped.chars().filter(|c| !c.is_whitespace()).collect();
    let mut out = ParsedLine {
        comments,
        ..Default::default()
    };
    let mut cs_seen = false;
    for m in WORD_RE.find_iter(&compact) {
        let word = m.as_str();
        let mut chars = word.chars();
        let letter = chars.next().unwrap().to_ascii_uppercase();
        let argument = chars.as_str();
        match letter {
            '%' | '{' => out.cmds.push(line.trim().to_string()),
            '$' => out.cmds.push(format!("${argument}")),
            'N' if out.ln.is_none() => out.ln = Some(crate::util::js_str_to_num(argument)),
            '*' if !cs_seen => cs_seen = true,
            _ => {
                let n = crate::util::js_str_to_num(argument);
                let value = if n.is_nan() {
                    WordValue::Text(argument.to_string())
                } else {
                    WordValue::Num(n)
                };
                out.words.push((letter, value));
            }
        }
    }
    out.stripped = stripped;
    out.compact = compact;
    out
}

fn is_m(word: &str, code: &str) -> bool {
    let Some(rest) = word.strip_prefix(['M', 'm']) else {
        return false;
    };
    let digits = rest.trim_start_matches('0');
    !rest.is_empty()
        && rest.bytes().all(|b| b.is_ascii_digit())
        && (digits == code || (code == "0" && digits.is_empty()))
}

/// `M0`, `M00`, `m000` … (flattened word).
pub fn is_m0(word: &str) -> bool {
    is_m(word, "0")
}

pub fn is_m1(word: &str) -> bool {
    is_m(word, "1")
}

pub fn is_m6(word: &str) -> bool {
    is_m(word, "6")
}

pub fn is_m109(word: &str) -> bool {
    is_m(word, "109")
}

pub fn is_m190(word: &str) -> bool {
    is_m(word, "190")
}

static M6_RE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"(?i)(\d|\b)(M0*6)(\b|[a-zA-Z])").unwrap());

/// Rewrites every M6 word with `f` (`replaceM6(line, x => \`(${x})\`)`).
pub fn replace_m6(gcode: &str, f: impl Fn(&str) -> String) -> String {
    gcode
        .split('\n')
        .map(|line| {
            M6_RE
                .replace_all(line, |c: &regex::Captures| {
                    format!("{}{}{}", &c[1], f(&c[2]), &c[3])
                })
                .into_owned()
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Builtin `%msg` / `%wait` commands.
#[derive(Debug, Clone, PartialEq)]
pub enum Builtin {
    Msg(String),
    Wait(String),
}

/// Matches `%msg text` / `%wait seconds` (case-insensitive, `;` comments stripped).
pub fn match_builtin(line: &str) -> Option<Builtin> {
    let stripped = line.split(';').next().unwrap_or("").trim();
    let lower = stripped.to_ascii_lowercase();
    for (name, ctor) in [
        ("%msg", Builtin::Msg as fn(String) -> Builtin),
        ("%wait", Builtin::Wait),
    ] {
        if let Some(rest) = lower.strip_prefix(name) {
            // `\b`: the command name must not run into another word character.
            if rest
                .chars()
                .next()
                .is_some_and(|c| c.is_alphanumeric() || c == '_')
            {
                continue;
            }
            return Some(ctor(stripped[name.len()..].trim().to_string()));
        }
    }
    None
}

/// `%wait` argument as the dwell line: `G4 P<seconds>` (default 0.5 s).
pub fn wait_dwell(args: &str) -> String {
    let secs = crate::util::js_parse_float(args);
    let secs = if secs.is_nan() || secs == 0.0 {
        0.5
    } else {
        secs
    };
    format!("G4 P{}", js_num_to_string(crate::util::round_to(secs, 3)))
}

pub fn mm2in(v: f64) -> f64 {
    v / 25.4
}

pub fn in2mm(v: f64) -> f64 {
    v * 25.4
}

/// Units string (`"in"`/`"mm"`) for a modal units word (`G20`/`G21`).
pub fn units_from_modal(modal_units: &str) -> Option<&'static str> {
    match modal_units {
        "G20" => Some(IMPERIAL_UNITS),
        "G21" => Some(METRIC_UNITS),
        _ => None,
    }
}

/// `mapValueToUnits`: mm value converted to `units` and rounded (4 decimals in, 3 mm).
pub fn map_value_to_units(val: f64, units: Option<&str>) -> f64 {
    match units {
        Some(IMPERIAL_UNITS) => crate::util::round_to(mm2in(val), 4),
        Some(METRIC_UNITS) => crate::util::round_to(val, 3),
        _ => val,
    }
}

/// `mapPositionToUnits`: like [`map_value_to_units`] but as the fixed-point string.
pub fn map_position_to_units(pos: f64, units: Option<&str>) -> String {
    match units {
        Some(IMPERIAL_UNITS) => format!("{:.4}", mm2in(pos)),
        Some(METRIC_UNITS) => format!("{pos:.3}"),
        _ => js_num_to_string(pos),
    }
}

/// Digits after the decimal point of a numeric string (`decimal-places`).
pub fn decimal_places(s: &str) -> usize {
    let (mantissa, exp) = match s.find(['e', 'E']) {
        Some(i) => (&s[..i], s[i + 1..].parse::<i64>().unwrap_or(0)),
        None => (s, 0),
    };
    let digits = mantissa.split_once('.').map_or(0, |(_, f)| f.len()) as i64;
    (digits - exp).max(0) as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_line_matches_gcode_parser() {
        let p = parse_line("N10 G00 X1.50 (move) y-2 ; comment");
        assert_eq!(p.stripped, "N10 G00 X1.50  y-2");
        assert_eq!(p.flat_words(), vec!["G0", "X1.5", "Y-2"]);
        assert_eq!(p.ln, Some(10.0));
        assert_eq!(p.comments, vec!["move", "comment"]);
        assert_eq!(parse_line("$H").cmds, vec!["$H"]);
        assert_eq!(
            parse_line("G0 X(nested (comment)) Y1").flat_words(),
            vec!["G0", "Y1"]
        );
    }

    #[test]
    fn m_code_matchers() {
        assert!(is_m0("M0") && is_m0("M00") && is_m0("m000"));
        assert!(!is_m0("M01") && !is_m0("M") && !is_m0("M0.5"));
        assert!(is_m6("M06") && !is_m6("M60"));
        assert!(is_m109("M109") && is_m190("M0190"));
    }

    #[test]
    fn replaces_m6() {
        assert_eq!(replace_m6("T1 M6", |x| format!("({x})")), "T1 (M6)");
        assert_eq!(replace_m6("T1M06G0", |x| format!("({x})")), "T1(M06)G0");
        assert_eq!(replace_m6("M60", |x| format!("({x})")), "M60");
    }

    #[test]
    fn builtins() {
        assert_eq!(
            match_builtin("%wait ; planner"),
            Some(Builtin::Wait(String::new()))
        );
        assert_eq!(
            match_builtin("%MSG Tool Change T[tool]"),
            Some(Builtin::Msg("Tool Change T[tool]".into()))
        );
        assert_eq!(match_builtin("%waiting"), None);
        assert_eq!(wait_dwell(""), "G4 P0.5");
        assert_eq!(wait_dwell("1.23456"), "G4 P1.235");
    }

    #[test]
    fn units_and_decimals() {
        assert_eq!(map_position_to_units(25.4, Some(IMPERIAL_UNITS)), "1.0000");
        assert_eq!(map_value_to_units(1.23456, Some(METRIC_UNITS)), 1.235);
        assert_eq!(decimal_places("1.230"), 3);
        assert_eq!(decimal_places("5"), 0);
        assert_eq!(decimal_places("1.5e-2"), 3);
    }
}

//! JavaScript-expression subset used by cncjs G-code macros:
//! `G0 X[posx - 8]`, `%_x = posx, global.count += 1`, `[mapWCSToPValue(modal.wcs)]`,
//! `[Math.max(zmax, 5).toFixed(3)]`.
//!
//! Replaces the esprima-based `evaluate-expression`, `evaluate-assignment-expression`
//! and `translate-expression` modules. Values follow JS coercion rules closely enough
//! for numeric/string G-code work; unsupported syntax evaluates to `undefined` and the
//! original text is kept, exactly like the JS version.

use std::sync::LazyLock;

use regex::Regex;
use serde_json::{Map, Value};
use tracing::error;

use crate::util::{js_num_to_string, js_parse_float, js_str_to_num, json_num};

/// Runtime value.
#[derive(Debug, Clone, PartialEq)]
pub enum V {
    Undefined,
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<V>),
    Obj(Map<String, Value>),
    /// A callable reached by name (`parseFloat`, `Math.min`, `x.toFixed` …).
    Func(Func),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Func {
    /// `Math` / `JSON` namespace objects (only usable through member access).
    Ns(&'static str),
    Global(&'static str),
    Math(String),
    Json(String),
    /// Method bound to its receiver.
    Method(Box<V>, String),
}

impl V {
    fn from_json(v: &Value) -> V {
        match v {
            Value::Null => V::Null,
            Value::Bool(b) => V::Bool(*b),
            Value::Number(n) => V::Num(n.as_f64().unwrap_or(f64::NAN)),
            Value::String(s) => V::Str(s.clone()),
            Value::Array(a) => V::Arr(a.iter().map(V::from_json).collect()),
            Value::Object(o) => V::Obj(o.clone()),
        }
    }

    /// JSON form; `None` for `undefined`/functions (JS drops them).
    pub fn to_json(&self) -> Option<Value> {
        Some(match self {
            V::Undefined | V::Func(_) => return None,
            V::Null => Value::Null,
            V::Bool(b) => Value::Bool(*b),
            V::Num(n) => json_num(*n),
            V::Str(s) => Value::String(s.clone()),
            V::Arr(a) => Value::Array(
                a.iter()
                    .map(|x| x.to_json().unwrap_or(Value::Null))
                    .collect(),
            ),
            V::Obj(o) => Value::Object(o.clone()),
        })
    }

    pub fn truthy(&self) -> bool {
        match self {
            V::Undefined | V::Null => false,
            V::Bool(b) => *b,
            V::Num(n) => *n != 0.0 && !n.is_nan(),
            V::Str(s) => !s.is_empty(),
            V::Arr(_) | V::Obj(_) | V::Func(_) => true,
        }
    }

    pub fn to_num(&self) -> f64 {
        match self {
            V::Undefined | V::Func(_) | V::Obj(_) => f64::NAN,
            V::Null => 0.0,
            V::Bool(b) => f64::from(u8::from(*b)),
            V::Num(n) => *n,
            V::Str(s) => js_str_to_num(s),
            V::Arr(a) => match a.as_slice() {
                [] => 0.0,
                [x] => x.to_num(),
                _ => f64::NAN,
            },
        }
    }

    /// JS `String(value)`.
    pub fn to_js_string(&self) -> String {
        match self {
            V::Undefined => "undefined".into(),
            V::Null => "null".into(),
            V::Bool(b) => b.to_string(),
            V::Num(n) => js_num_to_string(*n),
            V::Str(s) => s.clone(),
            V::Arr(a) => a
                .iter()
                .map(|x| match x {
                    V::Undefined | V::Null => String::new(),
                    x => x.to_js_string(),
                })
                .collect::<Vec<_>>()
                .join(","),
            V::Obj(_) => "[object Object]".into(),
            V::Func(_) => "function () { [native code] }".into(),
        }
    }

    fn to_i32(&self) -> i32 {
        let n = self.to_num();
        if !n.is_finite() {
            return 0;
        }
        (n.trunc() as i64 as u64 as u32) as i32
    }

    fn is_primitive_str(&self) -> bool {
        matches!(self, V::Str(_) | V::Arr(_) | V::Obj(_))
    }
}

// ---------------------------------------------------------------------------
// Lexer

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Num(f64),
    Str(String),
    Ident(String),
    Punct(&'static str),
}

const PUNCTS: &[&str] = &[
    ">>>=", "===", "!==", ">>>", "<<=", ">>=", "**=", "&&", "||", "??", "==", "!=", "<=", ">=",
    "<<", ">>", "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "**", "?.", "+", "-", "*", "/",
    "%", "<", ">", "=", "!", "~", "&", "|", "^", "?", ":", ",", ".", "(", ")", "[", "]", "{", "}",
];

fn lex(src: &str) -> Result<Vec<Tok>, String> {
    let mut toks = Vec::new();
    let b = src.as_bytes();
    let mut i = 0;
    'outer: while i < b.len() {
        let c = b[i];
        if c.is_ascii_whitespace() {
            i += 1;
            continue;
        }
        if c.is_ascii_digit() || (c == b'.' && b.get(i + 1).is_some_and(u8::is_ascii_digit)) {
            let start = i;
            if c == b'0' && matches!(b.get(i + 1), Some(b'x' | b'X')) {
                i += 2;
                while i < b.len() && b[i].is_ascii_hexdigit() {
                    i += 1;
                }
                let v = i64::from_str_radix(&src[start + 2..i], 16).map_err(|e| e.to_string())?;
                toks.push(Tok::Num(v as f64));
                continue;
            }
            while i < b.len() && (b[i].is_ascii_digit() || b[i] == b'.') {
                i += 1;
            }
            if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
                let mut j = i + 1;
                if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
                    j += 1;
                }
                if j < b.len() && b[j].is_ascii_digit() {
                    while j < b.len() && b[j].is_ascii_digit() {
                        j += 1;
                    }
                    i = j;
                }
            }
            let n = src[start..i]
                .parse::<f64>()
                .map_err(|_| format!("Invalid number {}", &src[start..i]))?;
            toks.push(Tok::Num(n));
            continue;
        }
        if c == b'"' || c == b'\'' {
            let mut s = String::new();
            let mut chars = src[i + 1..].char_indices();
            while let Some((off, ch)) = chars.next() {
                match ch {
                    '\\' => {
                        let (_, e) = chars.next().ok_or("Unterminated string")?;
                        s.push(match e {
                            'n' => '\n',
                            't' => '\t',
                            'r' => '\r',
                            '0' => '\0',
                            other => other,
                        });
                    }
                    ch if ch as u32 == u32::from(c) => {
                        toks.push(Tok::Str(s));
                        i = i + 1 + off + 1;
                        continue 'outer;
                    }
                    ch => s.push(ch),
                }
            }
            return Err("Unterminated string".into());
        }
        if c.is_ascii_alphabetic() || c == b'_' || c == b'$' {
            let start = i;
            while i < b.len() && (b[i].is_ascii_alphanumeric() || b[i] == b'_' || b[i] == b'$') {
                i += 1;
            }
            toks.push(Tok::Ident(src[start..i].to_string()));
            continue;
        }
        for p in PUNCTS {
            if src[i..].starts_with(p) {
                toks.push(Tok::Punct(p));
                i += p.len();
                continue 'outer;
            }
        }
        return Err(format!(
            "Unexpected character {:?}",
            src[i..].chars().next().unwrap_or(' ')
        ));
    }
    Ok(toks)
}

// ---------------------------------------------------------------------------
// AST + parser

#[derive(Debug, Clone)]
enum Expr {
    Lit(V),
    Ident(String),
    Array(Vec<Expr>),
    Object(Vec<(String, Expr)>),
    Member { obj: Box<Expr>, prop: Box<Expr> },
    Call { callee: Box<Expr>, args: Vec<Expr> },
    Unary(&'static str, Box<Expr>),
    Binary(&'static str, Box<Expr>, Box<Expr>),
    Cond(Box<Expr>, Box<Expr>, Box<Expr>),
    Assign(&'static str, Box<Expr>, Box<Expr>),
    Seq(Vec<Expr>),
}

struct Parser {
    toks: Vec<Tok>,
    pos: usize,
}

fn binary_prec(op: &str) -> Option<u8> {
    Some(match op {
        "??" => 1,
        "||" => 2,
        "&&" => 3,
        "|" => 4,
        "^" => 5,
        "&" => 6,
        "==" | "!=" | "===" | "!==" => 7,
        "<" | ">" | "<=" | ">=" => 8,
        "<<" | ">>" | ">>>" => 9,
        "+" | "-" => 10,
        "*" | "/" | "%" => 11,
        "**" => 12,
        _ => return None,
    })
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos)
    }

    fn peek_punct(&self) -> Option<&'static str> {
        match self.peek() {
            Some(Tok::Punct(p)) => Some(p),
            _ => None,
        }
    }

    fn eat(&mut self, p: &str) -> bool {
        if self.peek_punct() == Some(p) {
            self.pos += 1;
            true
        } else {
            false
        }
    }

    fn expect(&mut self, p: &str) -> Result<(), String> {
        if self.eat(p) {
            Ok(())
        } else {
            Err(format!("Expected {p:?}, found {:?}", self.peek()))
        }
    }

    fn sequence(&mut self) -> Result<Expr, String> {
        let first = self.assignment()?;
        if self.peek_punct() != Some(",") {
            return Ok(first);
        }
        let mut items = vec![first];
        while self.eat(",") {
            items.push(self.assignment()?);
        }
        Ok(Expr::Seq(items))
    }

    fn assignment(&mut self) -> Result<Expr, String> {
        let left = self.conditional()?;
        if let Some(
            op @ ("=" | "+=" | "-=" | "*=" | "/=" | "%=" | "**=" | "<<=" | ">>=" | ">>>=" | "&="
            | "|=" | "^="),
        ) = self.peek_punct()
        {
            if !matches!(left, Expr::Ident(_) | Expr::Member { .. }) {
                return Err("Invalid left-hand side in assignment".into());
            }
            self.pos += 1;
            let right = self.assignment()?;
            return Ok(Expr::Assign(op, Box::new(left), Box::new(right)));
        }
        Ok(left)
    }

    fn conditional(&mut self) -> Result<Expr, String> {
        let test = self.binary(0)?;
        if self.eat("?") {
            let yes = self.assignment()?;
            self.expect(":")?;
            let no = self.assignment()?;
            return Ok(Expr::Cond(Box::new(test), Box::new(yes), Box::new(no)));
        }
        Ok(test)
    }

    /// Precedence climbing: only operators binding tighter than `min_prec` are consumed.
    fn binary(&mut self, min_prec: u8) -> Result<Expr, String> {
        let mut left = self.unary()?;
        while let Some(op) = self.peek_punct() {
            let Some(prec) = binary_prec(op) else { break };
            if prec <= min_prec {
                break;
            }
            self.pos += 1;
            // `**` is right-associative.
            let right = self.binary(if op == "**" { prec - 1 } else { prec })?;
            left = Expr::Binary(op, Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn unary(&mut self) -> Result<Expr, String> {
        if let Some(op @ ("+" | "-" | "!" | "~")) = self.peek_punct() {
            self.pos += 1;
            return Ok(Expr::Unary(op, Box::new(self.unary()?)));
        }
        if matches!(self.peek(), Some(Tok::Ident(s)) if s == "typeof") {
            self.pos += 1;
            return Ok(Expr::Unary("typeof", Box::new(self.unary()?)));
        }
        self.postfix()
    }

    fn postfix(&mut self) -> Result<Expr, String> {
        let mut e = self.primary()?;
        loop {
            if self.eat(".") || self.eat("?.") {
                match self.peek().cloned() {
                    Some(Tok::Ident(name)) => {
                        self.pos += 1;
                        e = Expr::Member {
                            obj: Box::new(e),
                            prop: Box::new(Expr::Lit(V::Str(name))),
                        };
                    }
                    other => return Err(format!("Unexpected token {other:?} after '.'")),
                }
            } else if self.eat("[") {
                let prop = self.sequence()?;
                self.expect("]")?;
                e = Expr::Member {
                    obj: Box::new(e),
                    prop: Box::new(prop),
                };
            } else if self.eat("(") {
                let mut args = Vec::new();
                if !self.eat(")") {
                    loop {
                        args.push(self.assignment()?);
                        if self.eat(")") {
                            break;
                        }
                        self.expect(",")?;
                    }
                }
                e = Expr::Call {
                    callee: Box::new(e),
                    args,
                };
            } else {
                return Ok(e);
            }
        }
    }

    fn primary(&mut self) -> Result<Expr, String> {
        let tok = self.peek().cloned().ok_or("Unexpected end of input")?;
        self.pos += 1;
        match tok {
            Tok::Num(n) => Ok(Expr::Lit(V::Num(n))),
            Tok::Str(s) => Ok(Expr::Lit(V::Str(s))),
            Tok::Ident(name) => Ok(match name.as_str() {
                "true" => Expr::Lit(V::Bool(true)),
                "false" => Expr::Lit(V::Bool(false)),
                "null" => Expr::Lit(V::Null),
                _ => Expr::Ident(name),
            }),
            Tok::Punct("(") => {
                let e = self.sequence()?;
                self.expect(")")?;
                Ok(e)
            }
            Tok::Punct("[") => {
                let mut items = Vec::new();
                if !self.eat("]") {
                    loop {
                        items.push(self.assignment()?);
                        if self.eat("]") {
                            break;
                        }
                        self.expect(",")?;
                    }
                }
                Ok(Expr::Array(items))
            }
            Tok::Punct("{") => {
                let mut props = Vec::new();
                if !self.eat("}") {
                    loop {
                        let key = match self.peek().cloned() {
                            Some(Tok::Ident(k)) | Some(Tok::Str(k)) => k,
                            Some(Tok::Num(n)) => js_num_to_string(n),
                            other => return Err(format!("Unexpected token {other:?} in object")),
                        };
                        self.pos += 1;
                        let value = if self.eat(":") {
                            self.assignment()?
                        } else {
                            Expr::Ident(key.clone())
                        };
                        props.push((key, value));
                        if self.eat("}") {
                            break;
                        }
                        self.expect(",")?;
                    }
                }
                Ok(Expr::Object(props))
            }
            other => Err(format!("Unexpected token {other:?}")),
        }
    }
}

fn parse(src: &str) -> Result<Expr, String> {
    let mut p = Parser {
        toks: lex(src)?,
        pos: 0,
    };
    let e = p.sequence()?;
    if p.pos != p.toks.len() {
        return Err(format!("Unexpected token {:?}", p.toks[p.pos]));
    }
    Ok(e)
}

// ---------------------------------------------------------------------------
// Evaluation

/// Unresolvable expression (cncjs `UNRESOLVED`), maps to `undefined` at the top level.
struct Unresolved;
type R = Result<V, Unresolved>;

const GLOBAL_FUNCS: &[&str] = &[
    "parseFloat",
    "parseInt",
    "Number",
    "String",
    "Boolean",
    "isNaN",
    "isFinite",
    "mapWCSToPValue",
];

fn lookup(name: &str, vars: &Map<String, Value>) -> R {
    if let Some(v) = vars.get(name) {
        return Ok(V::from_json(v));
    }
    if let Some(f) = GLOBAL_FUNCS.iter().find(|f| **f == name) {
        return Ok(V::Func(Func::Global(f)));
    }
    match name {
        "undefined" => Ok(V::Undefined),
        "NaN" => Ok(V::Num(f64::NAN)),
        "Infinity" => Ok(V::Num(f64::INFINITY)),
        "Math" => Ok(V::Func(Func::Ns("Math"))),
        "JSON" => Ok(V::Func(Func::Ns("JSON"))),
        _ => Err(Unresolved),
    }
}

fn math_const(name: &str) -> Option<f64> {
    use std::f64::consts as c;
    Some(match name {
        "PI" => c::PI,
        "E" => c::E,
        "LN2" => c::LN_2,
        "LN10" => c::LN_10,
        "LOG2E" => c::LOG2_E,
        "LOG10E" => c::LOG10_E,
        "SQRT2" => c::SQRT_2,
        "SQRT1_2" => c::FRAC_1_SQRT_2,
        _ => return None,
    })
}

fn member(obj: V, key: &V) -> R {
    if let V::Func(Func::Ns(ns)) = &obj {
        let k = key.to_js_string();
        return Ok(match *ns {
            "Math" => math_const(&k).map_or(V::Func(Func::Math(k)), V::Num),
            _ => V::Func(Func::Json(k)),
        });
    }
    let k = key.to_js_string();
    match &obj {
        V::Obj(m) => Ok(m.get(&k).map_or(V::Undefined, V::from_json)),
        V::Arr(a) => {
            if k == "length" {
                return Ok(V::Num(a.len() as f64));
            }
            match k.parse::<usize>() {
                Ok(i) => Ok(a.get(i).cloned().unwrap_or(V::Undefined)),
                Err(_) => Ok(V::Func(Func::Method(Box::new(obj.clone()), k))),
            }
        }
        V::Str(s) => {
            if k == "length" {
                return Ok(V::Num(s.chars().count() as f64));
            }
            match k.parse::<usize>() {
                Ok(i) => Ok(s
                    .chars()
                    .nth(i)
                    .map_or(V::Undefined, |c| V::Str(c.to_string()))),
                Err(_) => Ok(V::Func(Func::Method(Box::new(obj.clone()), k))),
            }
        }
        V::Num(_) | V::Bool(_) => Ok(V::Func(Func::Method(Box::new(obj.clone()), k))),
        // Property access on undefined/null throws in JS.
        V::Undefined | V::Null => Err(Unresolved),
        V::Func(_) => Ok(V::Undefined),
    }
}

fn to_fixed(n: f64, digits: f64) -> String {
    let d = if digits.is_nan() {
        0
    } else {
        digits.clamp(0.0, 100.0) as usize
    };
    if !n.is_finite() {
        return js_num_to_string(n);
    }
    format!("{n:.d$}")
}

fn call(f: &Func, args: &[V]) -> R {
    let arg = |i: usize| args.get(i).cloned().unwrap_or(V::Undefined);
    let num = |i: usize| arg(i).to_num();
    Ok(match f {
        Func::Global(name) => match *name {
            "parseFloat" => V::Num(js_parse_float(&arg(0).to_js_string())),
            "parseInt" => {
                let s = arg(0).to_js_string();
                let mut radix = match arg(1) {
                    V::Undefined => 0,
                    r => r.to_num() as u32,
                };
                let t = s.trim_start();
                let (neg, mut t) = match t.strip_prefix('-') {
                    Some(r) => (true, r),
                    None => (false, t.strip_prefix('+').unwrap_or(t)),
                };
                if radix == 0 || radix == 16 {
                    if let Some(h) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
                        t = h;
                        radix = 16;
                    }
                }
                if radix == 0 {
                    radix = 10;
                }
                if !(2..=36).contains(&radix) {
                    return Ok(V::Num(f64::NAN));
                }
                let digits: String = t.chars().take_while(|c| c.is_digit(radix)).collect();
                if digits.is_empty() {
                    V::Num(f64::NAN)
                } else {
                    let v = digits.chars().fold(0f64, |acc, c| {
                        acc * f64::from(radix) + f64::from(c.to_digit(radix).unwrap())
                    });
                    V::Num(if neg { -v } else { v })
                }
            }
            "Number" => V::Num(if args.is_empty() { 0.0 } else { num(0) }),
            "String" => V::Str(if args.is_empty() {
                String::new()
            } else {
                arg(0).to_js_string()
            }),
            "Boolean" => V::Bool(arg(0).truthy()),
            "isNaN" => V::Bool(num(0).is_nan()),
            "isFinite" => V::Bool(num(0).is_finite()),
            "mapWCSToPValue" => V::Num(match arg(0).to_js_string().as_str() {
                "G54" => 1.0,
                "G55" => 2.0,
                "G56" => 3.0,
                "G57" => 4.0,
                "G58" => 5.0,
                "G59" => 6.0,
                _ => 0.0,
            }),
            _ => return Err(Unresolved),
        },
        Func::Math(name) => {
            let n = num(0);
            V::Num(match name.as_str() {
                "abs" => n.abs(),
                "ceil" => n.ceil(),
                "floor" => n.floor(),
                // JS rounds .5 towards +Infinity.
                "round" => (n + 0.5).floor(),
                "trunc" => n.trunc(),
                "sign" => {
                    if n.is_nan() || n == 0.0 {
                        n
                    } else {
                        n.signum()
                    }
                }
                "sqrt" => n.sqrt(),
                "cbrt" => n.cbrt(),
                "exp" => n.exp(),
                "log" => n.ln(),
                "log10" => n.log10(),
                "log2" => n.log2(),
                "sin" => n.sin(),
                "cos" => n.cos(),
                "tan" => n.tan(),
                "asin" => n.asin(),
                "acos" => n.acos(),
                "atan" => n.atan(),
                "atan2" => n.atan2(num(1)),
                "pow" => n.powf(num(1)),
                "hypot" => args.iter().map(|a| a.to_num().powi(2)).sum::<f64>().sqrt(),
                "min" => args.iter().map(V::to_num).fold(f64::INFINITY, |a, b| {
                    if a.is_nan() || b.is_nan() {
                        f64::NAN
                    } else {
                        a.min(b)
                    }
                }),
                "max" => args.iter().map(V::to_num).fold(f64::NEG_INFINITY, |a, b| {
                    if a.is_nan() || b.is_nan() {
                        f64::NAN
                    } else {
                        a.max(b)
                    }
                }),
                "random" => rand::random::<f64>(),
                _ => return Err(Unresolved),
            })
        }
        Func::Json(name) => match name.as_str() {
            "stringify" => match arg(0).to_json() {
                Some(v) => V::Str(v.to_string()),
                None => V::Undefined,
            },
            "parse" => match serde_json::from_str::<Value>(&arg(0).to_js_string()) {
                Ok(v) => V::from_json(&v),
                Err(_) => return Err(Unresolved),
            },
            _ => return Err(Unresolved),
        },
        Func::Ns(_) => return Err(Unresolved),
        Func::Method(recv, name) => match (recv.as_ref(), name.as_str()) {
            (V::Num(n), "toFixed") => {
                V::Str(to_fixed(*n, if args.is_empty() { 0.0 } else { num(0) }))
            }
            (V::Num(n), "toString") => V::Str(js_num_to_string(*n)),
            (V::Str(s), "toUpperCase") => V::Str(s.to_uppercase()),
            (V::Str(s), "toLowerCase") => V::Str(s.to_lowercase()),
            (V::Str(s), "trim") => V::Str(s.trim().to_string()),
            (V::Str(s), "toString") => V::Str(s.clone()),
            (V::Bool(b), "toString") => V::Str(b.to_string()),
            (V::Arr(a), "join") => {
                let sep = match arg(0) {
                    V::Undefined => ",".to_string(),
                    s => s.to_js_string(),
                };
                V::Str(
                    a.iter()
                        .map(|x| {
                            if matches!(x, V::Undefined | V::Null) {
                                String::new()
                            } else {
                                x.to_js_string()
                            }
                        })
                        .collect::<Vec<_>>()
                        .join(&sep),
                )
            }
            _ => return Err(Unresolved),
        },
    })
}

fn loose_eq(l: &V, r: &V) -> bool {
    match (l, r) {
        (V::Undefined | V::Null, V::Undefined | V::Null) => true,
        (V::Undefined | V::Null, _) | (_, V::Undefined | V::Null) => false,
        (V::Str(a), V::Str(b)) => a == b,
        (V::Arr(_) | V::Obj(_), V::Arr(_) | V::Obj(_)) => false,
        (V::Arr(_) | V::Obj(_), _) => loose_eq(&V::Str(l.to_js_string()), r),
        (_, V::Arr(_) | V::Obj(_)) => loose_eq(l, &V::Str(r.to_js_string())),
        _ => l.to_num() == r.to_num(),
    }
}

fn strict_eq(l: &V, r: &V) -> bool {
    match (l, r) {
        (V::Num(a), V::Num(b)) => a == b,
        (V::Arr(_), V::Arr(_)) | (V::Obj(_), V::Obj(_)) => false,
        _ => l == r,
    }
}

fn compare(l: &V, r: &V) -> Option<std::cmp::Ordering> {
    if let (V::Str(a), V::Str(b)) = (l, r) {
        return Some(a.cmp(b));
    }
    l.to_num().partial_cmp(&r.to_num())
}

fn binop(op: &str, l: V, r: V) -> R {
    use std::cmp::Ordering::*;
    Ok(match op {
        "+" => {
            if l.is_primitive_str() || r.is_primitive_str() {
                V::Str(l.to_js_string() + &r.to_js_string())
            } else {
                V::Num(l.to_num() + r.to_num())
            }
        }
        "-" => V::Num(l.to_num() - r.to_num()),
        "*" => V::Num(l.to_num() * r.to_num()),
        "/" => V::Num(l.to_num() / r.to_num()),
        "%" => V::Num(l.to_num() % r.to_num()),
        "**" => V::Num(l.to_num().powf(r.to_num())),
        "==" => V::Bool(loose_eq(&l, &r)),
        "!=" => V::Bool(!loose_eq(&l, &r)),
        "===" => V::Bool(strict_eq(&l, &r)),
        "!==" => V::Bool(!strict_eq(&l, &r)),
        "<" => V::Bool(compare(&l, &r) == Some(Less)),
        "<=" => V::Bool(matches!(compare(&l, &r), Some(Less | Equal))),
        ">" => V::Bool(compare(&l, &r) == Some(Greater)),
        ">=" => V::Bool(matches!(compare(&l, &r), Some(Greater | Equal))),
        "&" => V::Num(f64::from(l.to_i32() & r.to_i32())),
        "|" => V::Num(f64::from(l.to_i32() | r.to_i32())),
        "^" => V::Num(f64::from(l.to_i32() ^ r.to_i32())),
        "<<" => V::Num(f64::from(l.to_i32().wrapping_shl(r.to_i32() as u32 & 31))),
        ">>" => V::Num(f64::from(l.to_i32().wrapping_shr(r.to_i32() as u32 & 31))),
        ">>>" => V::Num(f64::from(
            (l.to_i32() as u32).wrapping_shr(r.to_i32() as u32 & 31),
        )),
        _ => return Err(Unresolved),
    })
}

fn eval(e: &Expr, vars: &mut Map<String, Value>) -> R {
    match e {
        Expr::Lit(v) => Ok(v.clone()),
        Expr::Ident(name) => lookup(name, vars),
        Expr::Array(items) => Ok(V::Arr(
            items
                .iter()
                .map(|x| eval(x, vars))
                .collect::<Result<_, _>>()?,
        )),
        Expr::Object(props) => {
            let mut m = Map::new();
            for (k, x) in props {
                if let Some(v) = eval(x, vars)?.to_json() {
                    m.insert(k.clone(), v);
                }
            }
            Ok(V::Obj(m))
        }
        Expr::Member { obj, prop, .. } => {
            let o = eval(obj, vars)?;
            let p = eval(prop, vars)?;
            member(o, &p)
        }
        Expr::Call { callee, args } => {
            let f = eval(callee, vars)?;
            let V::Func(f) = f else {
                return Err(Unresolved);
            };
            let args = args
                .iter()
                .map(|a| eval(a, vars))
                .collect::<Result<Vec<_>, _>>()?;
            call(&f, &args)
        }
        Expr::Unary(op, x) => {
            if *op == "typeof" {
                let v = eval(x, vars).unwrap_or(V::Undefined);
                return Ok(V::Str(
                    match v {
                        V::Undefined => "undefined",
                        V::Null | V::Arr(_) | V::Obj(_) => "object",
                        V::Bool(_) => "boolean",
                        V::Num(_) => "number",
                        V::Str(_) => "string",
                        V::Func(_) => "function",
                    }
                    .into(),
                ));
            }
            let v = eval(x, vars)?;
            Ok(match *op {
                "+" => V::Num(v.to_num()),
                "-" => V::Num(-v.to_num()),
                "!" => V::Bool(!v.truthy()),
                "~" => V::Num(f64::from(!v.to_i32())),
                _ => return Err(Unresolved),
            })
        }
        Expr::Binary(op, l, r) => {
            let lv = eval(l, vars)?;
            match *op {
                "&&" => {
                    if !lv.truthy() {
                        return Ok(lv);
                    }
                    eval(r, vars)
                }
                "||" => {
                    if lv.truthy() {
                        return Ok(lv);
                    }
                    eval(r, vars)
                }
                "??" => {
                    if !matches!(lv, V::Undefined | V::Null) {
                        return Ok(lv);
                    }
                    eval(r, vars)
                }
                _ => {
                    let rv = eval(r, vars)?;
                    binop(op, lv, rv)
                }
            }
        }
        Expr::Cond(t, y, n) => {
            if eval(t, vars)?.truthy() {
                eval(y, vars)
            } else {
                eval(n, vars)
            }
        }
        Expr::Assign(op, target, value) => {
            let path = target_path(target, vars)?;
            let rhs = eval(value, vars)?;
            let new = if *op == "=" {
                rhs
            } else {
                let current = path_get(vars, &path);
                binop(&op[..op.len() - 1], current, rhs)?
            };
            path_set(vars, &path, new.to_json().unwrap_or(Value::Null));
            Ok(new)
        }
        Expr::Seq(items) => {
            let mut last = V::Undefined;
            for x in items {
                // Each expression of a sequence is evaluated independently (cncjs
                // `walkSequenceExpression` ignores unresolved members).
                last = eval(x, vars).unwrap_or(V::Undefined);
            }
            Ok(last)
        }
    }
}

/// Assignment target as a key path (`global.count`, `a[i]`).
fn target_path(e: &Expr, vars: &mut Map<String, Value>) -> Result<Vec<String>, Unresolved> {
    match e {
        Expr::Ident(n) => Ok(vec![n.clone()]),
        Expr::Member { obj, prop, .. } => {
            let mut p = target_path(obj, vars)?;
            p.push(eval(prop, vars)?.to_js_string());
            Ok(p)
        }
        _ => Err(Unresolved),
    }
}

fn path_get(vars: &Map<String, Value>, path: &[String]) -> V {
    let mut cur = match vars.get(&path[0]) {
        Some(v) => v,
        None => return V::Undefined,
    };
    for k in &path[1..] {
        cur = match cur {
            Value::Object(m) => match m.get(k) {
                Some(v) => v,
                None => return V::Undefined,
            },
            Value::Array(a) => match k.parse::<usize>().ok().and_then(|i| a.get(i)) {
                Some(v) => v,
                None => return V::Undefined,
            },
            _ => return V::Undefined,
        };
    }
    V::from_json(cur)
}

/// lodash `_.set(vars, path, value)`.
fn path_set(vars: &mut Map<String, Value>, path: &[String], value: Value) {
    let mut root = Value::Object(std::mem::take(vars));
    let segs: Vec<_> = path
        .iter()
        .map(|k| match k.parse::<usize>() {
            Ok(i) => crate::config::Seg::Index(i),
            Err(_) => crate::config::Seg::Key(k.clone()),
        })
        .collect();
    crate::config::set_path(&mut root, &segs, value);
    if let Value::Object(m) = root {
        *vars = m;
    }
}

/// Evaluates an expression; `None` stands for `undefined` (unresolvable or invalid).
pub fn evaluate(src: &str, vars: &mut Map<String, Value>) -> Option<V> {
    match parse(src) {
        Ok(ast) => match eval(&ast, vars) {
            Ok(V::Undefined) | Err(Unresolved) => None,
            Ok(v) => Some(v),
        },
        Err(e) => {
            error!("src={src:?}: {e}");
            None
        }
    }
}

/// Runs an assignment / sequence expression for its side effects on `vars`
/// (`%_x = posx, _y = posy`).
pub fn evaluate_assignment(src: &str, vars: &mut Map<String, Value>) {
    if src.trim().is_empty() {
        return;
    }
    if let Err(e) = parse(src).map(|ast| eval(&ast, vars)) {
        error!("src={src:?}: {e}");
    }
}

static BRACKETS: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\[[^\]]+\]").unwrap());

/// Replaces every `[expression]` by its value (`G0 X[posx - 8]` → `G0 X2`);
/// unresolvable expressions are left untouched.
pub fn translate(data: &str, vars: &mut Map<String, Value>) -> String {
    if data.is_empty() {
        return String::new();
    }
    let mut out = String::with_capacity(data.len());
    let mut last = 0;
    for m in BRACKETS.find_iter(data) {
        out.push_str(&data[last..m.start()]);
        let src = &data[m.start() + 1..m.end() - 1];
        match evaluate(src, vars) {
            Some(v) => out.push_str(&v.to_js_string()),
            None => out.push_str(m.as_str()),
        }
        last = m.end();
    }
    out.push_str(&data[last..]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn vars(v: Value) -> Map<String, Value> {
        v.as_object().unwrap().clone()
    }

    #[test]
    fn translates_gcode_expressions() {
        let mut v = vars(
            json!({"posx": 10, "ymax": 50, "modal": {"wcs": "G55", "spindle": "M3"}, "tool_probe_command": "G38.2"}),
        );
        assert_eq!(translate("G0 X[posx - 8] Y[ymax]", &mut v), "G0 X2 Y50");
        assert_eq!(
            translate("G10 L20 P[mapWCSToPValue(modal.wcs)] Z0", &mut v),
            "G10 L20 P2 Z0"
        );
        assert_eq!(translate("[modal.spindle]", &mut v), "M3");
        assert_eq!(
            translate("G91 [tool_probe_command] F10", &mut v),
            "G91 G38.2 F10"
        );
        assert_eq!(
            translate("Z[Math.max(posx, 12).toFixed(3)]", &mut v),
            "Z12.000"
        );
        // Unresolvable expressions keep their text.
        assert_eq!(translate("X[unknown + 1]", &mut v), "X[unknown + 1]");
    }

    #[test]
    fn assignments_mutate_scope() {
        let mut v = vars(json!({"posx": 1.5, "posz": -2, "global": {}}));
        evaluate_assignment("_x = posx, _z = posz * 2, global.count = 1", &mut v);
        evaluate_assignment("global.count += 2", &mut v);
        assert_eq!(v["_x"], json!(1.5));
        assert_eq!(v["_z"], json!(-4.0));
        assert_eq!(v["global"]["count"], json!(3.0));
    }

    #[test]
    fn js_semantics() {
        let mut v = Map::new();
        let s = |src: &str, v: &mut Map<String, Value>| evaluate(src, v).map(|x| x.to_js_string());
        assert_eq!(s("1 + '2'", &mut v).as_deref(), Some("12"));
        assert_eq!(s("2 ** 3 ** 2", &mut v).as_deref(), Some("512"));
        assert_eq!(s("1 + 2 * 3 - 4 / 2", &mut v).as_deref(), Some("5"));
        assert_eq!(s("0 || 'x'", &mut v).as_deref(), Some("x"));
        assert_eq!(s("'5' == 5 ? 'eq' : 'ne'", &mut v).as_deref(), Some("eq"));
        assert_eq!(s("Math.round(-2.5)", &mut v).as_deref(), Some("-2"));
        assert_eq!(s("parseInt('12.7mm')", &mut v).as_deref(), Some("12"));
        assert_eq!(s("(0.1 + 0.2).toFixed(2)", &mut v).as_deref(), Some("0.30"));
        assert_eq!(
            s("Math.PI > 3 && Math.PI < 4", &mut v).as_deref(),
            Some("true")
        );
        assert_eq!(s("undefined", &mut v), None);
    }
}

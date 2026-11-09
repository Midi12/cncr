//! Line preprocessing shared by every controller's feeder and sender filters:
//! context population, `%msg` / `%wait` / `%expression` handling, `[expression]`
//! translation, and the manual tool change program.

use serde_json::{json, Value};

use crate::config::ConfigStore;
use crate::expr::{evaluate_assignment, translate};
use crate::gcode::{
    map_position_to_units, map_value_to_units, match_builtin, parse_line, units_from_modal,
    wait_dwell, Builtin,
};
use crate::util::{js_str_to_num, num_or_zero};
use crate::{Context, TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS};

const AXES: [&str; 6] = ["x", "y", "z", "a", "b", "c"];

/// Machine values exposed to macros.
#[derive(Debug, Clone, Default)]
pub struct MachineSnapshot {
    /// Machine position x, y, z, a, b, c.
    pub mpos: [f64; 6],
    /// Work position x, y, z, a, b, c.
    pub wpos: [f64; 6],
    /// Modal group object (`motion`, `wcs`, `plane`, `units`, `distance`, `feedrate`,
    /// `program`, `spindle`, `coolant` — coolant may be a string or an array).
    pub modal: Value,
    pub tool: f64,
    /// G-code parameters (`params`).
    pub params: Value,
}

/// Position object (`{x: "1.000", ...}`) to an axis array, `Number(v) || 0` per axis.
pub fn axes_from_json(pos: &Value) -> [f64; 6] {
    let mut out = [0.0; 6];
    for (i, axis) in AXES.iter().enumerate() {
        out[i] = num_or_zero(pos.get(axis));
    }
    out
}

/// cncjs `populateContext`: adds bounding box, positions, modal state, tool, params
/// and the shared `global` object to `ctx`. After running a filter, copy
/// `ctx["global"]` back into the controller's shared context.
pub fn populate_context(ctx: &mut Context, snap: &MachineSnapshot, global: &Value) {
    ctx.insert("global".into(), global.clone());
    for k in ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"] {
        let v = num_or_zero(ctx.get(k));
        ctx.insert(k.into(), json!(v));
    }
    for (i, axis) in AXES.iter().enumerate() {
        ctx.insert(format!("mpos{axis}"), json!(snap.mpos[i]));
        ctx.insert(format!("pos{axis}"), json!(snap.wpos[i]));
    }
    let m = |k: &str| snap.modal.get(k).cloned().unwrap_or(Value::Null);
    let coolant = match snap.modal.get("coolant") {
        // M7 and M8 together on one line is a modal group violation: one per line.
        Some(Value::Array(a)) => a
            .iter()
            .filter_map(Value::as_str)
            .collect::<Vec<_>>()
            .join("\n"),
        Some(Value::String(s)) => s.clone(),
        _ => String::new(),
    };
    ctx.insert(
        "modal".into(),
        json!({
            "motion": m("motion"), "wcs": m("wcs"), "plane": m("plane"), "units": m("units"),
            "distance": m("distance"), "feedrate": m("feedrate"), "program": m("program"),
            "spindle": m("spindle"), "coolant": coolant,
        }),
    );
    ctx.insert("tool".into(), json!(snap.tool));
    ctx.insert("params".into(), snap.params.clone());
}

/// Result of preprocessing one raw line.
#[derive(Debug, Clone, PartialEq)]
pub enum Line {
    /// `%msg text`: the translated message to put in the message slot; send nothing.
    Msg(String),
    /// `%wait [seconds]`: send this dwell and hold with `{data: "%wait", msg}`.
    Wait(String),
    /// `%expression`: evaluated for its side effects; send nothing.
    Assigned,
    /// G-code after `[expression]` translation: comment-stripped line + flattened words.
    Gcode { line: String, words: Vec<String> },
}

pub fn preprocess(raw: &str, ctx: &mut Context) -> Line {
    let line = raw.trim();
    if line.starts_with('%') {
        match match_builtin(line) {
            Some(Builtin::Msg(args)) => return Line::Msg(translate(&args, ctx)),
            Some(Builtin::Wait(args)) => return Line::Wait(wait_dwell(&args)),
            None => {
                // `%_x=posx,_y=posy ; comment`
                let expr = line.split(';').next().unwrap_or("").trim();
                evaluate_assignment(&expr[1..], ctx);
                return Line::Assigned;
            }
        }
    }
    let translated = translate(line, ctx);
    let parsed = parse_line(&translated);
    let words = parsed.flat_words();
    Line::Gcode {
        line: parsed.stripped,
        words,
    }
}

/// Copies `ctx["global"]` back into the controller's shared context.
pub fn sync_global(ctx: &Context, global: &mut Value) {
    if let Some(g) = ctx.get("global") {
        *global = g.clone();
    }
}

/// `tool.toolChangePolicy` from the config.
pub fn tool_change_policy(config: &ConfigStore) -> i64 {
    match config.get("tool.toolChangePolicy") {
        Some(Value::Number(n)) => n.as_i64().unwrap_or(TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS),
        Some(Value::String(s)) => js_str_to_num(&s) as i64,
        _ => TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS,
    }
}

/// Manual tool change program (`tool:change` command). `probe_block` holds the
/// policy-specific probing lines the controller wants for the active policy (WCS,
/// TLO or custom); everything around it is common to all controllers.
pub fn tool_change_program(
    config: &ConfigStore,
    modal_units: &str,
    probe_block: &[String],
) -> (Vec<String>, Context) {
    let units = units_from_modal(modal_units);
    let num = |k: &str, d: f64| match config.get(&format!("tool.{k}")) {
        Some(v) => num_or_zero(Some(&v)),
        None => d,
    };
    let pos = |k: &str| Value::String(map_position_to_units(num(k, 0.0), units));
    let val = |k: &str, d: f64| json!(map_value_to_units(num(k, d), units));
    let mut ctx = Context::new();
    ctx.insert("tool_change_x".into(), pos("toolChangeX"));
    ctx.insert("tool_change_y".into(), pos("toolChangeY"));
    ctx.insert("tool_change_z".into(), pos("toolChangeZ"));
    ctx.insert("tool_probe_x".into(), pos("toolProbeX"));
    ctx.insert("tool_probe_y".into(), pos("toolProbeY"));
    ctx.insert("tool_probe_z".into(), pos("toolProbeZ"));
    ctx.insert(
        "tool_probe_command".into(),
        json!(config
            .get("tool.toolProbeCommand")
            .and_then(|v| v.as_str().map(str::to_string))
            .unwrap_or_else(|| "G38.2".into())),
    );
    ctx.insert("tool_probe_distance".into(), val("toolProbeDistance", 1.0));
    ctx.insert("tool_probe_feedrate".into(), val("toolProbeFeedrate", 10.0));
    ctx.insert("touch_plate_height".into(), val("touchPlateHeight", 0.0));

    let mut lines: Vec<String> = [
        "%wait",
        // Remember original position and spindle state
        "%_posx=posx",
        "%_posy=posy",
        "%_posz=posz",
        "%_modal_spindle=modal.spindle",
        "M5",
        "G90",
        "G53 G0 Z[tool_change_z]",
        "G53 G0 X[tool_change_x] Y[tool_change_y]",
        "%wait",
        "%msg Tool Change T[tool]",
        "M0",
        "G53 G0 X[tool_probe_x] Y[tool_probe_y]",
        "G53 G0 Z[tool_probe_z]",
        "%wait",
    ]
    .map(String::from)
    .to_vec();
    lines.extend(probe_block.iter().cloned());
    lines.extend(
        [
            "G53 G0 Z[tool_change_z]",
            "G53 G0 X[tool_change_x] Y[tool_change_y]",
            "%wait",
            "%msg Restart Spindle",
            "M0",
            "G90",
            "G0 X[_posx] Y[_posy]",
            "G0 Z[_posz]",
            "[_modal_spindle]",
            // Wait 5 seconds for the spindle to speed up
            "%wait 5",
        ]
        .map(String::from),
    );
    (lines, ctx)
}

/// `tool.toolProbeCustomCommands` split into lines.
pub fn tool_probe_custom_commands(config: &ConfigStore) -> Vec<String> {
    config
        .get("tool.toolProbeCustomCommands")
        .and_then(|v| {
            v.as_str()
                .map(|s| s.split('\n').map(str::to_string).collect())
        })
        .unwrap_or_else(|| vec![String::new()])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preprocess_variants() {
        let snap = MachineSnapshot {
            wpos: [1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
            modal: json!({"wcs": "G54", "coolant": ["M7", "M8"]}),
            ..Default::default()
        };
        let mut ctx = Context::new();
        populate_context(&mut ctx, &snap, &json!({}));
        assert_eq!(ctx["modal"]["coolant"], json!("M7\nM8"));
        assert_eq!(
            preprocess("%msg at [posx]", &mut ctx),
            Line::Msg("at 1".into())
        );
        assert_eq!(
            preprocess("%wait ; planner", &mut ctx),
            Line::Wait("G4 P0.5".into())
        );
        assert_eq!(
            preprocess("%_z = posz + 1 ; keep", &mut ctx),
            Line::Assigned
        );
        assert_eq!(ctx["_z"], json!(4.0));
        assert_eq!(
            preprocess("G0 X[posx - 8] (move) ; c", &mut ctx),
            Line::Gcode {
                line: "G0 X-7".into(),
                words: vec!["G0".into(), "X-7".into()]
            }
        );
        preprocess("%global.n = 5", &mut ctx);
        let mut global = json!({});
        sync_global(&ctx, &mut global);
        assert_eq!(global, json!({"n": 5.0}));
    }

    #[test]
    fn tool_change_uses_units() {
        let config = ConfigStore::in_memory(
            json!({"tool": {"toolChangeZ": 25.4, "toolProbeFeedrate": 50.8}}),
        );
        let (lines, ctx) = tool_change_program(&config, "G20", &["PROBE".into()]);
        assert_eq!(ctx["tool_change_z"], json!("1.0000"));
        assert_eq!(ctx["tool_probe_feedrate"], json!(2.0));
        assert_eq!(ctx["tool_probe_command"], json!("G38.2"));
        assert_eq!(lines[15], "PROBE");
        assert_eq!(lines.last().map(String::as_str), Some("%wait 5"));
    }
}

//! Auto-leveling: probe grid generation, probe result tracking and Z compensation of
//! G-code over the probed surface (port of cncjs `lib/autolevel.js` plus the
//! `autolevel:*` command handlers shared by every controller).

use std::collections::BTreeMap;
use std::sync::LazyLock;

use regex::Regex;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use tracing::{debug, error, info, warn};

use crate::gcode::{in2mm, mm2in, parse_line};
use crate::util::{js_num_to_string, js_parse_float, num_or_zero};

/// Default probe-grid spacing (mm) when the data isn't a usable rectangular grid.
const DEFAULT_GRID_STEP: f64 = 5.0;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    fn sub(self, o: Point3) -> Point3 {
        Point3 {
            x: self.x - o.x,
            y: self.y - o.y,
            z: self.z - o.z,
        }
    }

    fn dist2_xy(self, o: Point3) -> f64 {
        (o.x - self.x).powi(2) + (o.y - self.y).powi(2)
    }

    fn dist2(self, o: Point3) -> f64 {
        (o.x - self.x).powi(2) + (o.y - self.y).powi(2) + (o.z - self.z).powi(2)
    }
}

/// Grid of `startX..=endX` × `startY..=endY` probe points, row by row.
pub fn create_probe_xy_points(
    start_x: f64,
    end_x: f64,
    step_x: f64,
    start_y: f64,
    end_y: f64,
    step_y: f64,
) -> Vec<Point2> {
    let mut points = Vec::new();
    if !(step_x > 0.0 && step_y > 0.0) {
        return points;
    }
    let mut y = start_y;
    while y <= end_y {
        let mut x = start_x;
        while x <= end_x {
            points.push(Point2 { x, y });
            x += step_x;
        }
        y += step_y;
    }
    points
}

fn subdivide_segment(p1: Point3, p2: Point3, max_len: f64) -> Vec<Point3> {
    let v = p2.sub(p1);
    let dist = p1.dist2(p2).sqrt();
    if dist < 1e-10 {
        return vec![];
    }
    let dir = Point3 {
        x: v.x / dist,
        y: v.y / dist,
        z: v.z / dist,
    };
    let mut out = vec![p1];
    let mut d = max_len;
    while d < dist {
        let pt = Point3 {
            x: p1.x + dir.x * d,
            y: p1.y + dir.y * d,
            z: p1.z + dir.z * d,
        };
        if out.last().is_none_or(|last| pt.dist2(*last) > 1e-10) {
            out.push(pt);
        }
        d += max_len;
    }
    if out.last().is_none_or(|last| p2.dist2(*last) > 1e-10) {
        out.push(p2);
    }
    out
}

/// Plane through the three closest non-collinear points (fallback for non-grid data).
fn plane_fit_z(x: f64, y: f64, probed: &[Point3]) -> Option<f64> {
    if probed.len() < 3 {
        return None;
    }
    let q = Point3 { x, y, z: 0.0 };
    let mut sorted = probed.to_vec();
    sorted.sort_by(|a, b| a.dist2_xy(q).total_cmp(&b.dist2_xy(q)));
    let mut pts: Vec<Point3> = Vec::with_capacity(3);
    for p in sorted {
        if pts.len() == 3 {
            break;
        }
        if pts.len() == 2 {
            let (u, v) = (pts[1].sub(pts[0]), p.sub(pts[0]));
            if (u.x * v.y - u.y * v.x).abs() < 0.00001 {
                continue;
            }
        }
        pts.push(p);
    }
    if pts.len() < 3 {
        return None;
    }
    let (u, v) = (pts[1].sub(pts[0]), pts[2].sub(pts[0]));
    let n = Point3 {
        x: u.y * v.z - u.z * v.y,
        y: -(u.x * v.z - u.z * v.x),
        z: u.x * v.y - u.y * v.x,
    };
    if n.z == 0.0 {
        return None;
    }
    let b = pts[0];
    Some(b.z - (n.x * (x - b.x) + n.y * (y - b.y)) / n.z)
}

struct Grid {
    xs: Vec<f64>,
    ys: Vec<f64>,
    /// `matrix[j][i]` = height at `(xs[i], ys[j])`, `None` for unprobed nodes.
    matrix: Vec<Vec<Option<f64>>>,
}

fn round_micron(v: f64) -> i64 {
    (v * 1000.0).round() as i64
}

fn build_grid(probed: &[Point3]) -> Option<Grid> {
    let mut xi: BTreeMap<i64, usize> = probed.iter().map(|p| (round_micron(p.x), 0)).collect();
    let mut yi: BTreeMap<i64, usize> = probed.iter().map(|p| (round_micron(p.y), 0)).collect();
    if xi.len() < 2 || yi.len() < 2 {
        return None;
    }
    for (i, v) in xi.values_mut().enumerate() {
        *v = i;
    }
    for (j, v) in yi.values_mut().enumerate() {
        *v = j;
    }
    let mut matrix = vec![vec![None; xi.len()]; yi.len()];
    for p in probed {
        matrix[yi[&round_micron(p.y)]][xi[&round_micron(p.x)]] = Some(p.z);
    }
    Some(Grid {
        xs: xi.keys().map(|k| *k as f64 / 1000.0).collect(),
        ys: yi.keys().map(|k| *k as f64 / 1000.0).collect(),
        matrix,
    })
}

/// Largest cell index with `values[i] <= v`, clamped to `[0, n-2]` (edge extrapolation).
fn find_cell(values: &[f64], v: f64) -> usize {
    let mut i = 0;
    while i + 2 < values.len() && values[i + 1] <= v {
        i += 1;
    }
    i
}

fn min_spacing(sorted: &[f64]) -> f64 {
    sorted
        .windows(2)
        .map(|w| w[1] - w[0])
        .filter(|g| *g > 0.0)
        .fold(f64::INFINITY, f64::min)
}

fn bilinear_z(g: &Grid, x: f64, y: f64) -> Option<f64> {
    let (i, j) = (find_cell(&g.xs, x), find_cell(&g.ys, y));
    let (z00, z10, z01, z11) = (
        g.matrix[j][i]?,
        g.matrix[j][i + 1]?,
        g.matrix[j + 1][i]?,
        g.matrix[j + 1][i + 1]?,
    );
    let a = (x - g.xs[i]) / (g.xs[i + 1] - g.xs[i]);
    let b = (y - g.ys[j]) / (g.ys[j + 1] - g.ys[j]);
    Some(z00 * (1.0 - a) * (1.0 - b) + z10 * a * (1.0 - b) + z01 * (1.0 - a) * b + z11 * a * b)
}

struct Surface<'a> {
    grid: Option<Grid>,
    probed: &'a [Point3],
    step_x: f64,
    step_y: f64,
}

impl<'a> Surface<'a> {
    fn new(probed: &'a [Point3]) -> Self {
        let grid = build_grid(probed);
        let (step_x, step_y) = match &grid {
            Some(g) => (min_spacing(&g.xs), min_spacing(&g.ys)),
            None => (DEFAULT_GRID_STEP, DEFAULT_GRID_STEP),
        };
        Self {
            grid,
            probed,
            step_x,
            step_y,
        }
    }

    fn z_at(&self, x: f64, y: f64) -> Option<f64> {
        self.grid
            .as_ref()
            .and_then(|g| bilinear_z(g, x, y))
            .or_else(|| plane_fit_z(x, y, self.probed))
    }

    /// Compensates a point given in the current units (probe data is in mm).
    fn compensate(&self, pt: Point3, imperial: bool) -> Point3 {
        let conv = |v: f64| if imperial { in2mm(v) } else { v };
        match self.z_at(conv(pt.x), conv(pt.y)) {
            Some(dz) => {
                let z = conv(pt.z) + dz;
                Point3 {
                    x: pt.x,
                    y: pt.y,
                    z: if imperial { mm2in(z) } else { z },
                }
            }
            None => {
                warn!("Cannot compute Z compensation for point");
                pt
            }
        }
    }
}

static SKIP_RE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"(?i)^(G38.+|G5.+|G10|G4.+|G92|G92.1)$").unwrap());

/// Rewrites absolute XYZ moves so Z follows the probed surface; long moves are
/// subdivided to half the probe grid spacing. Needs at least 3 probed points.
pub fn apply_probe_compensation(gcode: &str, probed: &[Point3]) -> String {
    if probed.len() < 3 {
        error!("Not enough probed positions for compensation (need at least 3)");
        return gcode.to_string();
    }
    let surface = Surface::new(probed);
    info!(
        "Applying Z compensation (auto-detected grid: {:.2}mm × {:.2}mm, {} points)...",
        surface.step_x,
        surface.step_y,
        probed.len()
    );
    let lines: Vec<&str> = gcode.split('\n').collect();
    let mut out: Vec<String> = Vec::with_capacity(lines.len());
    let mut p0 = Point3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let mut p0_init = false;
    let mut pt = p0;
    let mut absolute = true;
    let mut imperial = false;

    for (idx, line) in lines.iter().enumerate() {
        if idx % 1000 == 0 {
            debug!("Compensation progress: {idx}/{}", lines.len());
        }
        let parsed = parse_line(line);
        let words = parsed.flat_words();
        if parsed.stripped.is_empty()
            || words.is_empty()
            || words.iter().any(|w| SKIP_RE.is_match(w))
        {
            out.push((*line).to_string());
            continue;
        }
        if words.iter().any(|w| w == "G91") {
            absolute = false;
        }
        if words.iter().any(|w| w == "G90") {
            absolute = true;
        }
        if words.iter().any(|w| w == "G20") {
            imperial = true;
        }
        if words.iter().any(|w| w == "G21") {
            imperial = false;
        }
        let (mut cx, mut cy, mut cz) = (None, None, None);
        for w in &words {
            let v = Some(js_parse_float(&w[1..]));
            match w.as_bytes()[0].to_ascii_uppercase() {
                b'X' => cx = v,
                b'Y' => cy = v,
                b'Z' => cz = v,
                _ => {}
            }
        }
        if cx.is_none() && cy.is_none() && cz.is_none() {
            out.push((*line).to_string());
            continue;
        }
        pt.x = cx.unwrap_or(pt.x);
        pt.y = cy.unwrap_or(pt.y);
        pt.z = cz.unwrap_or(pt.z);

        if absolute {
            let rest: Vec<&str> = words
                .iter()
                .filter(|w| !matches!(w.as_bytes()[0].to_ascii_uppercase(), b'X' | b'Y' | b'Z'))
                .map(String::as_str)
                .collect();
            let rest = rest.join(" ");
            let step = surface.step_x.min(surface.step_y);
            let max_len = (if imperial { mm2in(step) } else { step }) / 2.0;
            let emit = |p: Point3, out: &mut Vec<String>| {
                let c = surface.compensate(p, imperial);
                out.push(
                    format!("{rest} X{:.3} Y{:.3} Z{:.3}", c.x, c.y, c.z)
                        .trim()
                        .to_string(),
                );
            };
            if p0_init {
                for seg in subdivide_segment(p0, pt, max_len).into_iter().skip(1) {
                    emit(seg, &mut out);
                }
            } else {
                emit(pt, &mut out);
                p0_init = true;
            }
        } else {
            out.push((*line).to_string());
            warn!("Relative positioning (G91) may not produce correct compensation results");
        }
        p0 = pt;
    }
    info!("Z compensation applied successfully");
    out.join("\n")
}

/// JS template-literal rendering of a JSON parameter (`${clearanceZ}`).
fn tpl(v: Option<&Value>) -> String {
    match v {
        Some(Value::Number(n)) => js_num_to_string(n.as_f64().unwrap_or(f64::NAN)),
        Some(Value::String(s)) => s.clone(),
        Some(Value::Null) => "null".into(),
        Some(v) => v.to_string(),
        None => "undefined".into(),
    }
}

/// Probing session state (`probeState` of the JS controllers).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ProbeState {
    pub probed_positions: Vec<Point3>,
    pub min_z: Option<f64>,
    pub max_z: Option<f64>,
    pub probe_points: Vec<Point2>,
    pub config: Option<Value>,
}

impl ProbeState {
    pub fn to_json(&self) -> Value {
        json!({
            "probedPositions": self.probed_positions,
            "minZ": self.min_z,
            "maxZ": self.max_z,
            "probePoints": self.probe_points,
            "config": self.config,
        })
    }

    /// `autolevel:start`: returns the G-code to feed. `mode: "test"` probes once at
    /// the current XY; otherwise a full grid is generated and the state reset.
    pub fn start(&mut self, params: &Value) -> Vec<String> {
        let p = |k: &str| params.get(k);
        let (clearance, start_z, end_z) = (tpl(p("clearanceZ")), tpl(p("startZ")), tpl(p("endZ")));
        if p("mode").and_then(Value::as_str) == Some("test") {
            info!("[autolevel:start] Test probe: clearanceZ={clearance}, startZ={start_z}, endZ={end_z}");
            return vec![
                "G90".into(),
                format!("G0 Z{clearance}"),
                format!("G0 Z{start_z}"),
                format!("G38.2 Z{end_z} F{}", tpl(p("feedrate"))),
                format!("G0 Z{clearance}"),
            ];
        }
        let n = |k: &str, d: f64| p(k).map_or(d, |v| num_or_zero(Some(v)));
        let points = create_probe_xy_points(
            n("startX", 0.0),
            n("endX", 0.0),
            n("stepX", 10.0),
            n("startY", 0.0),
            n("endY", 0.0),
            n("stepY", 10.0),
        );
        let config: serde_json::Map<String, Value> = [
            "startX",
            "endX",
            "stepX",
            "startY",
            "endY",
            "stepY",
            "clearanceZ",
            "startZ",
            "endZ",
            "feedrate",
        ]
        .iter()
        .filter_map(|k| p(k).map(|v| ((*k).to_string(), v.clone())))
        .collect();
        let feedrate = num_or_zero(p("feedrate"));
        let mut gcode = Vec::with_capacity(points.len() * 6);
        for (i, pt) in points.iter().enumerate() {
            gcode.push(format!("(Auto Level: probing point {i})"));
            gcode.push("G90".into());
            gcode.push(format!("G0 Z{clearance}"));
            gcode.push(format!(
                "G0 X{} Y{}",
                js_num_to_string(pt.x),
                js_num_to_string(pt.y)
            ));
            gcode.push(format!("G0 Z{start_z}"));
            let f = if i == 0 { feedrate / 2.0 } else { feedrate };
            gcode.push(format!("G38.2 Z{end_z} F{}", js_num_to_string(f)));
            gcode.push(format!("G0 Z{clearance}"));
        }
        info!(
            "[autolevel:start] Starting probing with {} points",
            points.len()
        );
        *self = ProbeState {
            probe_points: points,
            config: Some(Value::Object(config)),
            ..Default::default()
        };
        gcode
    }

    /// `autolevel:stop` (the caller also resets the machine).
    pub fn clear(&mut self) {
        *self = ProbeState::default();
    }

    /// Records a successful probe (`[PRB:...:1]`). `z_mm` is the probed work Z in mm;
    /// the XY stored is the intended grid node (probe points are in the modal units,
    /// `imperial` = G20 active). Returns the `autolevel:update` payload and whether
    /// probing completed, or `None` when no grid probing is active.
    pub fn record(&mut self, z_mm: f64, imperial: bool) -> Option<(Value, bool)> {
        let done = self.probed_positions.len();
        if self.probe_points.is_empty() || done >= self.probe_points.len() {
            return None;
        }
        let node = self.probe_points[done];
        let conv = |v: f64| if imperial { in2mm(v) } else { v };
        let pos = Point3 {
            x: conv(node.x),
            y: conv(node.y),
            z: z_mm,
        };
        self.min_z = Some(self.min_z.map_or(z_mm, |m| m.min(z_mm)));
        self.max_z = Some(self.max_z.map_or(z_mm, |m| m.max(z_mm)));
        self.probed_positions.push(pos);
        let (current, total) = (self.probed_positions.len(), self.probe_points.len());
        debug!(
            "[autolevel] Probed {current}/{total}: posX={:.3}, posY={:.3}, posZ={:.3}",
            pos.x, pos.y, pos.z
        );
        let (min_z, max_z) = (self.min_z.unwrap_or(0.0), self.max_z.unwrap_or(0.0));
        let update = json!({
            "current": current,
            "total": total,
            "probedPos": pos,
            "minZ": min_z,
            "maxZ": max_z,
            "maxDeviation": max_z - min_z,
        });
        if current >= total {
            info!("[autolevel] Probing completed");
        }
        Some((update, current >= total))
    }

    /// `autolevel:loadFromFile`: whitespace separated `x y z ...` lines.
    pub fn load_from_file(&mut self, path: &str) -> Result<Value, String> {
        let data = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        let mut positions = Vec::new();
        let (mut min_z, mut max_z) = (f64::INFINITY, f64::NEG_INFINITY);
        for line in data.lines().filter(|l| !l.trim().is_empty()) {
            let mut vals = line
                .split_whitespace()
                .map(|t| t.parse::<f64>().unwrap_or(f64::NAN));
            let (x, y, z) = (
                vals.next().unwrap_or(f64::NAN),
                vals.next().unwrap_or(f64::NAN),
                vals.next().unwrap_or(f64::NAN),
            );
            positions.push(Point3 { x, y, z });
            min_z = min_z.min(z);
            max_z = max_z.max(z);
        }
        info!(
            "[autolevel:load] Loaded {} points from {path}",
            positions.len()
        );
        self.probed_positions = positions;
        self.min_z = Some(min_z);
        self.max_z = Some(max_z);
        Ok(json!({ "success": true, "state": self.to_json() }))
    }

    /// `autolevel:saveToFile`: `x y z 0 0 0 0 0 0` per probed point.
    pub fn save_to_file(&self, path: &str) -> Result<Value, String> {
        let data = self
            .probed_positions
            .iter()
            .map(|p| {
                format!(
                    "{} {} {} 0 0 0 0 0 0",
                    js_num_to_string(p.x),
                    js_num_to_string(p.y),
                    js_num_to_string(p.z)
                )
            })
            .collect::<Vec<_>>()
            .join("\n");
        std::fs::write(path, data).map_err(|e| e.to_string())?;
        info!(
            "[autolevel:saveToFile] Saved {} points to {path}",
            self.probed_positions.len()
        );
        Ok(json!({ "success": true, "filepath": path }))
    }
}

/// `autolevel:applyProbeCompensation` with `{gcode, probeData: [{x,y,z,...}]}`.
pub fn apply_probe_compensation_params(params: &Value) -> Value {
    let gcode = params
        .get("gcode")
        .and_then(Value::as_str)
        .unwrap_or_default();
    let probed: Vec<Point3> = params
        .get("probeData")
        .and_then(Value::as_array)
        .map(|a| {
            a.iter()
                .map(|p| Point3 {
                    x: crate::util::js_str_to_num(&tpl(p.get("x"))),
                    y: crate::util::js_str_to_num(&tpl(p.get("y"))),
                    z: crate::util::js_str_to_num(&tpl(p.get("z"))),
                })
                .collect()
        })
        .unwrap_or_default();
    json!({ "compensatedGcode": apply_probe_compensation(gcode, &probed) })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(f: impl Fn(f64, f64) -> f64) -> Vec<Point3> {
        create_probe_xy_points(0.0, 20.0, 10.0, 0.0, 20.0, 10.0)
            .into_iter()
            .map(|p| Point3 {
                x: p.x,
                y: p.y,
                z: f(p.x, p.y),
            })
            .collect()
    }

    #[test]
    fn probe_points_row_major() {
        let pts = create_probe_xy_points(0.0, 10.0, 10.0, 0.0, 5.0, 5.0);
        assert_eq!(
            pts.iter().map(|p| (p.x, p.y)).collect::<Vec<_>>(),
            vec![(0.0, 0.0), (10.0, 0.0), (0.0, 5.0), (10.0, 5.0)]
        );
        assert!(create_probe_xy_points(0.0, 1.0, 0.0, 0.0, 1.0, 1.0).is_empty());
    }

    #[test]
    fn compensates_and_subdivides_over_tilted_plane() {
        // z = 0.01 * x: bilinear interpolation is exact on a plane.
        let probed = grid(|x, _| 0.01 * x);
        let out =
            apply_probe_compensation("G21\nG90\nG0 X0 Y0 Z1\nG1 X20 Y0 Z1 F100\nG4 P1", &probed);
        let lines: Vec<&str> = out.lines().collect();
        assert_eq!(lines[0..2], ["G21", "G90"]);
        assert_eq!(lines[2], "G0 X0.000 Y0.000 Z1.000");
        // 20 mm move split every 5 mm (half the 10 mm grid step).
        assert_eq!(
            lines[3..7],
            [
                "G1 F100 X5.000 Y0.000 Z1.050",
                "G1 F100 X10.000 Y0.000 Z1.100",
                "G1 F100 X15.000 Y0.000 Z1.150",
                "G1 F100 X20.000 Y0.000 Z1.200"
            ]
        );
        assert_eq!(lines[7], "G4 P1", "dwell lines are not compensated");
    }

    #[test]
    fn imperial_moves_use_mm_probe_data() {
        let probed = grid(|_, _| 2.54);
        let out = apply_probe_compensation("G20\nG0 X0 Y0 Z0", &probed);
        assert_eq!(out.lines().nth(1), Some("G0 X0.000 Y0.000 Z0.100"));
    }

    #[test]
    fn record_tracks_grid_nodes() {
        let mut s = ProbeState::default();
        let gcode = s.start(
            &json!({"startX": 0, "endX": 10, "stepX": 10, "startY": 0, "endY": 0, "stepY": 10,
            "clearanceZ": 5, "startZ": 1, "endZ": -2, "feedrate": 20}),
        );
        assert_eq!(gcode[5], "G38.2 Z-2 F10", "first probe at half feedrate");
        assert_eq!(gcode.len(), 14);
        let (u, done) = s.record(-0.5, false).unwrap();
        assert_eq!((u["current"].as_u64(), done), (Some(1), false));
        let (u, done) = s.record(0.25, false).unwrap();
        assert!(done);
        assert_eq!(u["maxDeviation"], json!(0.75));
        assert_eq!(
            s.probed_positions[1],
            Point3 {
                x: 10.0,
                y: 0.0,
                z: 0.25
            }
        );
        assert!(s.record(0.0, false).is_none(), "completed");
    }
}

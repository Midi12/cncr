//! Queue of interactive commands (MDI, jog, macros) sent one at a time
//! (port of cncjs `Feeder`).
//!
//! Lines fed together share one context, as in JS: `%_x = posx` on one line is
//! visible to `G0 X[_x]` further down the same batch.

use std::collections::VecDeque;

use serde_json::{json, Value};

use crate::sender::FilterResult;
use crate::Context;

#[derive(Debug)]
struct Batch {
    lines: VecDeque<String>,
    context: Context,
}

#[derive(Debug, Default)]
pub struct Feeder {
    pub hold: bool,
    pub hold_reason: Option<Value>,
    batches: VecDeque<Batch>,
    len: usize,
    pending: bool,
    changed: bool,
}

impl Feeder {
    pub fn new() -> Self {
        Self::default()
    }

    /// `feeder:status` payload.
    pub fn to_json(&self) -> Value {
        json!({
            "hold": self.hold,
            "holdReason": self.hold_reason,
            "queue": self.len,
            "pending": self.pending,
            "changed": self.changed,
        })
    }

    pub fn feed(&mut self, lines: Vec<String>, context: Context) {
        if self.len == 0 {
            self.pending = false;
        }
        if !lines.is_empty() {
            self.len += lines.len();
            self.batches.push_back(Batch {
                lines: lines.into(),
                context,
            });
            self.changed = true;
        }
    }

    pub fn hold(&mut self, reason: Option<Value>) -> bool {
        if self.hold {
            return false;
        }
        self.hold = true;
        self.hold_reason = reason;
        self.changed = true;
        true
    }

    pub fn unhold(&mut self) -> bool {
        if !self.hold {
            return false;
        }
        self.hold = false;
        self.hold_reason = None;
        self.changed = true;
        true
    }

    pub fn clear(&mut self) {
        self.batches.clear();
        self.len = 0;
        self.pending = false;
        self.changed = true;
    }

    pub fn reset(&mut self) {
        self.hold = false;
        self.hold_reason = None;
        self.clear();
    }

    pub fn size(&self) -> usize {
        self.len
    }

    /// Pops the next command through `filter` (which gets the batch's shared context);
    /// returns the line to write and a snapshot of that context. Lines filtered to
    /// empty are skipped. A hold requested by the filter applies after the returned line.
    pub fn next(
        &mut self,
        filter: &mut dyn FnMut(&str, &mut Context) -> FilterResult,
    ) -> Option<(String, Context)> {
        let mut out = None;
        while !self.hold {
            let Some(batch) = self.batches.front_mut() else {
                break;
            };
            let Some(command) = batch.lines.pop_front() else {
                self.batches.pop_front();
                continue;
            };
            self.len -= 1;
            let result = filter(&command, &mut batch.context);
            let snapshot = (!result.line.is_empty()).then(|| batch.context.clone());
            if batch.lines.is_empty() {
                self.batches.pop_front();
            }
            if let Some(reason) = result.hold {
                self.hold(Some(reason));
            }
            let Some(context) = snapshot else { continue };
            self.pending = true;
            self.changed = true;
            out = Some((result.line, context));
            break;
        }
        if self.len == 0 {
            self.pending = false;
        }
        out
    }

    pub fn is_pending(&self) -> bool {
        self.pending
    }

    /// True once after any state change (drives `feeder:status` emission).
    pub fn peek(&mut self) -> bool {
        std::mem::take(&mut self.changed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn skips_empty_lines_and_holds_after_line() {
        let mut f = Feeder::new();
        f.feed(
            vec!["%x=1".into(), "M0".into(), "G0 X1".into()],
            Context::new(),
        );
        let mut filter = |l: &str, _: &mut Context| match l {
            "%x=1" => FilterResult::skip(),
            "M0" => FilterResult {
                line: "M0".into(),
                hold: Some(json!({"data": "M0"})),
            },
            l => FilterResult::line(l),
        };
        assert_eq!(f.next(&mut filter).map(|x| x.0).as_deref(), Some("M0"));
        assert!(f.hold && f.is_pending());
        assert!(f.next(&mut filter).is_none());
        f.unhold();
        assert_eq!(f.next(&mut filter).map(|x| x.0).as_deref(), Some("G0 X1"));
        assert!(!f.is_pending(), "queue drained");
    }

    #[test]
    fn lines_of_one_feed_share_their_context() {
        let mut f = Feeder::new();
        f.feed(vec!["%_x=5".into(), "G0 X[_x]".into()], Context::new());
        f.feed(vec!["G0 X[_x]".into()], Context::new());
        let mut filter = |l: &str, ctx: &mut Context| match crate::program::preprocess(l, ctx) {
            crate::program::Line::Gcode { line, .. } => FilterResult::line(line),
            _ => FilterResult::skip(),
        };
        assert_eq!(f.next(&mut filter).map(|x| x.0).as_deref(), Some("G0 X5"));
        assert_eq!(
            f.next(&mut filter).map(|x| x.0).as_deref(),
            Some("G0 X[_x]"),
            "other feed, own context"
        );
        assert_eq!(f.size(), 0);
    }
}

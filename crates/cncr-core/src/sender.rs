//! G-code program streamer (port of cncjs `Sender`).
//!
//! The JS version emits `data` events from inside a filter callback that may itself
//! hold the sender. Here [`Sender::next`] takes the filter as an argument and returns
//! the lines to write; a filter requests a hold through [`FilterResult::hold`], which
//! takes effect right after that line exactly like the re-entrant JS call did.

use std::collections::VecDeque;

use serde_json::{json, Value};

use crate::util::now_ms;
use crate::Context;

/// Streaming protocol type ids as reported in `sender:status` (`sp`).
pub const SP_TYPE_SEND_RESPONSE: u8 = 0;
pub const SP_TYPE_CHAR_COUNTING: u8 = 1;

const DEFAULT_BUFFER_SIZE: usize = 128;

#[derive(Debug, Clone)]
pub enum StreamingProtocol {
    /// One line in flight, the next is sent on `ok`.
    SendResponse,
    /// Fill the controller's RX buffer (Grbl/Smoothie).
    CharCounting {
        buffer_size: usize,
        data_length: usize,
        queue: VecDeque<usize>,
        line: String,
    },
}

impl StreamingProtocol {
    pub fn char_counting(buffer_size: usize) -> Self {
        let buffer_size = if buffer_size > 0 {
            buffer_size
        } else {
            DEFAULT_BUFFER_SIZE
        };
        Self::CharCounting {
            buffer_size,
            data_length: 0,
            queue: VecDeque::new(),
            line: String::new(),
        }
    }

    pub fn type_id(&self) -> u8 {
        match self {
            Self::SendResponse => SP_TYPE_SEND_RESPONSE,
            Self::CharCounting { .. } => SP_TYPE_CHAR_COUNTING,
        }
    }

    fn clear(&mut self) {
        if let Self::CharCounting {
            data_length,
            queue,
            line,
            ..
        } = self
        {
            *data_length = 0;
            queue.clear();
            line.clear();
        }
    }

    /// Bytes currently in the controller buffer (char-counting only, else 0).
    pub fn data_length(&self) -> usize {
        match self {
            Self::CharCounting { data_length, .. } => *data_length,
            Self::SendResponse => 0,
        }
    }

    pub fn buffer_size(&self) -> Option<usize> {
        match self {
            Self::CharCounting { buffer_size, .. } => Some(*buffer_size),
            Self::SendResponse => None,
        }
    }

    /// Grows/shrinks the buffer, never below the data already in it.
    pub fn set_buffer_size(&mut self, size: usize) {
        if let Self::CharCounting {
            buffer_size,
            data_length,
            ..
        } = self
        {
            if size > 0 {
                *buffer_size = size.max(*data_length);
            }
        }
    }
}

/// Outcome of filtering one line.
#[derive(Debug, Clone, Default)]
pub struct FilterResult {
    /// Line to send; empty means "skip" (acknowledged locally).
    pub line: String,
    /// Hold the sender after this line, with this reason.
    pub hold: Option<Value>,
}

impl FilterResult {
    pub fn line(line: impl Into<String>) -> Self {
        Self {
            line: line.into(),
            hold: None,
        }
    }

    pub fn skip() -> Self {
        Self::default()
    }
}

/// What a call to [`Sender::next`] produced.
#[derive(Debug, Default)]
pub struct NextOutcome {
    /// Lines to write, each without the trailing newline.
    pub lines: Vec<String>,
    /// The program started with this call (`start` event, epoch ms).
    pub started: Option<i64>,
    /// The program finished with this call (`end` event, epoch ms).
    pub ended: Option<i64>,
}

#[derive(Debug, Clone, Default)]
pub struct SenderState {
    pub hold: bool,
    pub hold_reason: Option<Value>,
    pub name: String,
    pub gcode: String,
    pub context: Context,
    pub lines: Vec<String>,
    pub total: usize,
    pub sent: usize,
    pub received: usize,
    pub start_time: i64,
    pub finish_time: i64,
    pub elapsed_time: i64,
    pub remaining_time: i64,
}

#[derive(Debug)]
pub struct Sender {
    pub sp: StreamingProtocol,
    pub state: SenderState,
    changed: bool,
}

impl Sender {
    pub fn new(sp: StreamingProtocol) -> Self {
        Self {
            sp,
            state: SenderState::default(),
            changed: false,
        }
    }

    /// `sender:status` payload.
    pub fn to_json(&self) -> Value {
        let s = &self.state;
        json!({
            "sp": self.sp.type_id(),
            "hold": s.hold,
            "holdReason": s.hold_reason,
            "name": s.name,
            "context": s.context,
            "size": s.gcode.len(),
            "total": s.total,
            "sent": s.sent,
            "received": s.received,
            "startTime": s.start_time,
            "finishTime": s.finish_time,
            "elapsedTime": s.elapsed_time,
            "remainingTime": s.remaining_time,
        })
    }

    /// Returns true if the hold state changed.
    pub fn hold(&mut self, reason: Option<Value>) -> bool {
        if self.state.hold {
            return false;
        }
        self.state.hold = true;
        self.state.hold_reason = reason;
        self.changed = true;
        true
    }

    pub fn unhold(&mut self) -> bool {
        if !self.state.hold {
            return false;
        }
        self.state.hold = false;
        self.state.hold_reason = None;
        self.changed = true;
        true
    }

    /// Loads a program. Returns false for empty G-code.
    pub fn load(&mut self, name: &str, gcode: &str, context: Context) -> bool {
        if gcode.is_empty() {
            return false;
        }
        self.sp.clear();
        let lines: Vec<String> = gcode
            .split('\n')
            .filter(|l| !l.trim().is_empty())
            .map(str::to_string)
            .collect();
        self.state = SenderState {
            name: name.to_string(),
            gcode: gcode.to_string(),
            context,
            total: lines.len(),
            lines,
            ..Default::default()
        };
        self.changed = true;
        true
    }

    pub fn unload(&mut self) {
        self.sp.clear();
        self.state = SenderState::default();
        self.changed = true;
    }

    /// An acknowledgement (`ok`/`error`) was received.
    pub fn ack(&mut self) -> bool {
        if self.state.gcode.is_empty() || self.state.received >= self.state.sent {
            return false;
        }
        self.state.received += 1;
        self.changed = true;
        true
    }

    /// Sends as much as the protocol allows. `filter` transforms each raw line.
    pub fn next(
        &mut self,
        filter: &mut dyn FnMut(&str, &mut Context) -> FilterResult,
    ) -> NextOutcome {
        let mut out = NextOutcome::default();
        if self.state.gcode.is_empty() {
            return out;
        }
        let now = now_ms();

        if self.state.total > 0 && self.state.sent == 0 {
            self.state.start_time = now;
            self.state.finish_time = 0;
            self.state.elapsed_time = 0;
            self.state.remaining_time = 0;
            out.started = Some(now);
            self.changed = true;
        }

        self.process(filter, &mut out.lines);

        self.state.elapsed_time = now - self.state.start_time;
        // Wait one second before estimating the remaining time.
        if self.state.elapsed_time >= 1000 && self.state.received > 0 {
            let per_line = self.state.elapsed_time as f64 / self.state.received as f64;
            self.state.remaining_time =
                (per_line * self.state.total as f64 - self.state.elapsed_time as f64) as i64;
        }

        if self.state.received >= self.state.total && self.state.finish_time == 0 {
            self.state.finish_time = now;
            out.ended = Some(now);
            self.changed = true;
        }
        out
    }

    fn apply(&mut self, result: FilterResult) -> String {
        if let Some(reason) = result.hold {
            self.hold(Some(reason));
        }
        result.line
    }

    fn process(
        &mut self,
        filter: &mut dyn FnMut(&str, &mut Context) -> FilterResult,
        out: &mut Vec<String>,
    ) {
        if let StreamingProtocol::CharCounting {
            queue, data_length, ..
        } = &mut self.sp
        {
            if let Some(len) = queue.pop_front() {
                *data_length -= len;
            }
        }
        while !self.state.hold && self.state.sent < self.state.total {
            // A line that did not fit the buffer last time is filtered again (as in JS).
            let pending = match &mut self.sp {
                StreamingProtocol::CharCounting { line, .. } => std::mem::take(line),
                StreamingProtocol::SendResponse => String::new(),
            };
            let raw = if pending.is_empty() {
                self.state.lines[self.state.sent].trim().to_string()
            } else {
                pending
            };
            let result = filter(&raw, &mut self.state.context);
            let filtered = self.apply(result);
            if let StreamingProtocol::CharCounting {
                buffer_size,
                data_length,
                line,
                ..
            } = &mut self.sp
            {
                // The newline also consumes RX buffer space.
                if !filtered.is_empty() && *data_length + filtered.len() + 1 >= *buffer_size {
                    *line = filtered;
                    break;
                }
            }
            self.state.sent += 1;
            self.changed = true;
            if filtered.is_empty() {
                self.ack();
                continue;
            }
            match &mut self.sp {
                StreamingProtocol::CharCounting {
                    data_length, queue, ..
                } => {
                    *data_length += filtered.len() + 1;
                    queue.push_back(filtered.len() + 1);
                    out.push(filtered);
                }
                StreamingProtocol::SendResponse => {
                    out.push(filtered);
                    break;
                }
            }
        }
    }

    /// Restarts from the first line.
    pub fn rewind(&mut self) -> bool {
        if self.state.gcode.is_empty() {
            return false;
        }
        self.sp.clear();
        self.state.hold = false;
        self.state.hold_reason = None;
        self.state.sent = 0;
        self.state.received = 0;
        self.changed = true;
        true
    }

    /// True once after any state change (drives `sender:status` emission).
    pub fn peek(&mut self) -> bool {
        std::mem::take(&mut self.changed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pass(line: &str, _: &mut Context) -> FilterResult {
        FilterResult::line(line)
    }

    #[test]
    fn char_counting_fills_buffer_and_refills_on_ack() {
        let mut s = Sender::new(StreamingProtocol::char_counting(12));
        assert!(s.load("p", "G0 X1\nG0 X2\n\nG0 X3\n", Context::new()));
        assert_eq!(s.state.total, 3);
        let out = s.next(&mut pass);
        // "G0 X1\n" is 6 bytes; a second line would reach the 12 byte limit.
        assert_eq!(out.lines, vec!["G0 X1"]);
        assert!(out.started.is_some());
        assert!(s.ack());
        let out = s.next(&mut pass);
        assert_eq!(out.lines, vec!["G0 X2"]);
        s.ack();
        assert_eq!(s.next(&mut pass).lines, vec!["G0 X3"]);
        s.ack();
        let out = s.next(&mut pass);
        assert!(out.lines.is_empty());
        assert!(out.ended.is_some());
    }

    #[test]
    fn send_response_skips_empty_filter_results_and_holds_after_line() {
        let mut s = Sender::new(StreamingProtocol::SendResponse);
        s.load("p", "%msg hi\nM0\nG0 X1", Context::new());
        let mut filter = |line: &str, _: &mut Context| match line {
            "%msg hi" => FilterResult::skip(),
            "M0" => FilterResult {
                line: "M0".into(),
                hold: Some(json!({"data": "M0"})),
            },
            l => FilterResult::line(l),
        };
        let out = s.next(&mut filter);
        assert_eq!(out.lines, vec!["M0"]);
        assert_eq!(s.state.sent, 2);
        assert_eq!(s.state.received, 1, "skipped line is acknowledged locally");
        assert!(s.state.hold);
        s.ack();
        assert!(s.next(&mut filter).lines.is_empty(), "held");
        s.unhold();
        assert_eq!(s.next(&mut filter).lines, vec!["G0 X1"]);
    }

    #[test]
    fn rewind_resets_progress() {
        let mut s = Sender::new(StreamingProtocol::SendResponse);
        s.load("p", "G0\nG1", Context::new());
        s.next(&mut pass);
        s.hold(None);
        assert!(s.rewind());
        assert_eq!(
            (s.state.sent, s.state.received, s.state.hold),
            (0, 0, false)
        );
    }
}

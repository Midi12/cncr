//! Serial port connection: a reader thread delivering newline-terminated lines and a
//! blocking writer (port of cncjs `SerialConnection`).

use std::io::{ErrorKind, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use serde_json::{json, Value};
use serialport::{FlowControl, SerialPort, SerialPortType};
use tracing::{error, warn};

/// What the reader thread reports.
#[derive(Debug)]
pub enum SerialEvent {
    /// One line without the `\n` (a trailing `\r` is kept, runners trim it).
    Line(String),
    /// The port went away; `Some` carries the error.
    Closed(Option<String>),
}

pub struct SerialConnection {
    path: String,
    writer: Box<dyn SerialPort>,
    stop: Arc<AtomicBool>,
}

impl std::fmt::Debug for SerialConnection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialConnection")
            .field("path", &self.path)
            .finish()
    }
}

/// Encodes outgoing text: code points U+0080..=U+00FF become single bytes so the Grbl
/// extended-ASCII realtime commands (`\u0085` jog cancel, `\u0090`… overrides) reach the
/// controller as one byte; everything else is UTF-8.
pub fn encode_wire(data: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len());
    for c in data.chars() {
        let cp = c as u32;
        if (0x80..=0xFF).contains(&cp) {
            out.push(cp as u8);
        } else {
            let mut buf = [0u8; 4];
            out.extend_from_slice(c.encode_utf8(&mut buf).as_bytes());
        }
    }
    out
}

impl SerialConnection {
    /// Opens the port (blocking; call from `spawn_blocking`) and starts the reader
    /// thread, which calls `on_event` for every line and once when the port closes.
    pub fn open(
        path: &str,
        baud_rate: u32,
        rtscts: bool,
        on_event: impl Fn(SerialEvent) + Send + 'static,
    ) -> Result<Self, String> {
        let flow = if rtscts {
            FlowControl::Hardware
        } else {
            FlowControl::None
        };
        let writer = serialport::new(path, baud_rate)
            .flow_control(flow)
            .timeout(Duration::from_secs(5))
            .open()
            .map_err(|e| e.to_string())?;
        let mut reader = writer.try_clone().map_err(|e| e.to_string())?;
        reader
            .set_timeout(Duration::from_millis(200))
            .map_err(|e| e.to_string())?;

        let stop = Arc::new(AtomicBool::new(false));
        let stop_reader = Arc::clone(&stop);
        let name = path.to_string();
        std::thread::Builder::new()
            .name(format!("serial-{name}"))
            .spawn(move || {
                let mut buf = [0u8; 1024];
                let mut line = Vec::<u8>::with_capacity(256);
                loop {
                    if stop_reader.load(Ordering::Relaxed) {
                        return;
                    }
                    match reader.read(&mut buf) {
                        Ok(0) => {
                            if !stop_reader.load(Ordering::Relaxed) {
                                on_event(SerialEvent::Closed(None));
                            }
                            return;
                        }
                        Ok(n) => {
                            for &b in &buf[..n] {
                                if b == b'\n' {
                                    on_event(SerialEvent::Line(
                                        String::from_utf8_lossy(&line).into_owned(),
                                    ));
                                    line.clear();
                                } else {
                                    line.push(b);
                                }
                            }
                        }
                        Err(e)
                            if matches!(
                                e.kind(),
                                ErrorKind::TimedOut
                                    | ErrorKind::Interrupted
                                    | ErrorKind::WouldBlock
                            ) => {}
                        Err(e) => {
                            if !stop_reader.load(Ordering::Relaxed) {
                                warn!("Serial port {name:?} read error: {e}");
                                on_event(SerialEvent::Closed(Some(e.to_string())));
                            }
                            return;
                        }
                    }
                }
            })
            .map_err(|e| e.to_string())?;

        Ok(Self {
            path: path.to_string(),
            writer,
            stop,
        })
    }

    pub fn path(&self) -> &str {
        &self.path
    }

    /// Writes text (see [`encode_wire`]); errors are logged, the reader reports closure.
    pub fn write(&mut self, data: &str) {
        if let Err(e) = self.writer.write_all(&encode_wire(data)) {
            error!(
                "Unexpected error while writing serial port {:?}: {e}",
                self.path
            );
        }
    }

    pub fn set_dtr(&mut self, level: bool) -> Result<(), String> {
        self.writer
            .write_data_terminal_ready(level)
            .map_err(|e| e.to_string())
    }

    pub fn set_rts(&mut self, level: bool) -> Result<(), String> {
        self.writer
            .write_request_to_send(level)
            .map_err(|e| e.to_string())
    }
}

impl Drop for SerialConnection {
    fn drop(&mut self) {
        // The reader thread exits within one read timeout; the fd closes with the last clone.
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// Serial ports present on the system: `[{port, manufacturer?}]`.
pub fn list_ports() -> Vec<Value> {
    match serialport::available_ports() {
        Ok(ports) => ports
            .into_iter()
            .map(|p| match p.port_type {
                SerialPortType::UsbPort(usb) => {
                    json!({ "port": p.port_name, "manufacturer": usb.manufacturer })
                }
                _ => json!({ "port": p.port_name }),
            })
            .collect(),
        Err(e) => {
            error!("Unable to list serial ports: {e}");
            vec![]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn realtime_bytes_are_single_bytes() {
        assert_eq!(encode_wire("\u{85}"), vec![0x85]);
        assert_eq!(encode_wire("?\u{18}"), vec![b'?', 0x18]);
        assert_eq!(encode_wire("€"), "€".as_bytes());
    }
}

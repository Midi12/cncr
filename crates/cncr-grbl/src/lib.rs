//! Grbl controller (port of cncjs `controllers/Grbl`).

mod constants;
mod controller;
mod runner;

use std::time::Duration;

use cncr_core::controller::{ControllerHandle, ControllerOptions, Services};
use cncr_core::serial::SerialConnection;
use serde_json::json;
use tokio::sync::mpsc;
use tracing::{debug, error};

use controller::Controller;

pub const TYPE: &str = "Grbl";

/// Opens the serial port and spawns the controller task.
pub async fn open(
    services: Services,
    options: ControllerOptions,
) -> Result<ControllerHandle, String> {
    let (serial_tx, serial_rx) = mpsc::unbounded_channel();
    let (path, baudrate, rtscts) = (options.port.clone(), options.baudrate, options.rtscts);
    let mut conn = tokio::task::spawn_blocking(move || {
        SerialConnection::open(&path, baudrate, rtscts, move |ev| {
            let _ = serial_tx.send(ev);
        })
    })
    .await
    .map_err(|e| e.to_string())?
    .inspect_err(|e| error!("Error opening serial port {:?}: {e}", options.port))?;

    let (dtr, rts) = (options.pin.dtr, options.pin.rts);
    if dtr.is_some() || rts.is_some() {
        tokio::time::sleep(Duration::from_millis(100)).await;
        let set = dtr
            .map_or(Ok(()), |v| conn.set_dtr(v))
            .and(rts.map_or(Ok(()), |v| conn.set_rts(v)));
        if let Err(e) = set {
            error!("Failed to set control flags: {e} (port {:?})", options.port);
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    let port = options.port.clone();
    services.hub.broadcast(
        "serialport:change",
        &[json!({ "port": port, "inuse": true })],
    );
    debug!("Connected to serial port {port:?}");

    let (tx, rx) = mpsc::unbounded_channel();
    let (internal_tx, internal_rx) = mpsc::unbounded_channel();
    let mut ctl = Controller::new(services, options, Some(Box::new(conn)), internal_tx);
    // `serialport:open` goes to sockets attached later (AddSocket replays it), and a
    // fresh controller has no program to unload.
    ctl.workflow_stop();
    ctl.clear_action_values();
    tokio::spawn(ctl.run(rx, serial_rx, internal_rx));
    Ok(ControllerHandle::new(TYPE, port, tx))
}

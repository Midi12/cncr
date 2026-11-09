//! TinyG / g2core controller (Rust port of the cncjs `TinyGController`).

mod constants;
mod controller;
mod runner;

use std::time::Duration;

use serde_json::json;
use tokio::sync::mpsc;
use tracing::{error, info};

use cncr_core::controller::{ControllerHandle, ControllerOptions, Services};
use cncr_core::serial::SerialConnection;

pub use constants::TYPE;
use controller::Controller;

/// Opens the serial port, starts the controller task and the JSON init sequence.
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
    .map_err(|e| e.to_string())??;

    let pin = options.pin.clone();
    if pin.dtr.is_some() || pin.rts.is_some() {
        tokio::time::sleep(Duration::from_millis(100)).await;
        let results = [
            pin.dtr.map(|v| conn.set_dtr(v)),
            pin.rts.map(|v| conn.set_rts(v)),
        ];
        for err in results.into_iter().flatten().filter_map(Result::err) {
            error!("Failed to set control flags on {:?}: {err}", options.port);
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    services.hub.broadcast(
        "serialport:change",
        &[json!({ "port": options.port, "inuse": true })],
    );
    info!("Connected to serial port {:?}", options.port);

    let (tx, rx) = mpsc::unbounded_channel();
    let (internal_tx, internal_rx) = mpsc::unbounded_channel();
    let handle = ControllerHandle::new(TYPE, options.port.clone(), tx);
    let mut ctl = Controller::new(services, options, conn, internal_tx);
    ctl.ready = true;
    ctl.spawn_init();
    tokio::spawn(ctl.run(rx, serial_rx, internal_rx));
    Ok(handle)
}

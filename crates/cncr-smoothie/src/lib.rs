//! Smoothieware controller (port of cncjs `controllers/Smoothie`).

mod controller;
pub mod runner;

use std::time::Duration;

use cncr_core::controller::{ControllerHandle, ControllerOptions, Services};
use cncr_core::serial::{SerialConnection, SerialEvent};
use serde_json::json;
use tokio::sync::mpsc;
use tokio::time::{interval, sleep, MissedTickBehavior};
use tracing::{error, warn};

use controller::Controller;

pub const TYPE: &str = "Smoothie";

/// Opens the serial port and spawns the controller task.
pub async fn open(
    services: Services,
    options: ControllerOptions,
) -> Result<ControllerHandle, String> {
    let (serial_tx, mut serial_rx) = mpsc::unbounded_channel();
    let (path, baudrate, rtscts) = (options.port.clone(), options.baudrate, options.rtscts);
    let mut conn = tokio::task::spawn_blocking(move || {
        SerialConnection::open(&path, baudrate, rtscts, move |event| {
            let _ = serial_tx.send(event);
        })
    })
    .await
    .map_err(|e| e.to_string())??;

    // Set DTR and RTS control flags if they exist
    if options.pin.dtr.is_some() || options.pin.rts.is_some() {
        sleep(Duration::from_millis(100)).await;
        let set = options
            .pin
            .dtr
            .map_or(Ok(()), |v| conn.set_dtr(v))
            .and_then(|()| options.pin.rts.map_or(Ok(()), |v| conn.set_rts(v)));
        if let Err(e) = set {
            error!(
                "Failed to set control flags: port={:?}, err={e}",
                options.port
            );
        }
        sleep(Duration::from_millis(100)).await;
    }

    services.hub.broadcast(
        "serialport:change",
        &[json!({ "port": options.port, "inuse": true })],
    );

    let (tx, mut rx) = mpsc::unbounded_channel();
    let (deferred_tx, mut deferred_rx) = mpsc::unbounded_channel();
    let handle = ControllerHandle::new(TYPE, options.port.clone(), tx);
    let port = options.port.clone();
    let mut ctl = Controller::new(services, options, conn, deferred_tx);
    ctl.start();

    tokio::spawn(async move {
        let mut query_timer = interval(Duration::from_millis(250));
        query_timer.set_missed_tick_behavior(MissedTickBehavior::Delay);
        loop {
            tokio::select! {
                msg = rx.recv() => match msg {
                    Some(msg) => if !ctl.handle(msg) { break },
                    None => { ctl.close(); break }
                },
                Some(event) = serial_rx.recv() => match event {
                    SerialEvent::Line(line) => ctl.parse(&line),
                    SerialEvent::Closed(err) => {
                        if let Some(err) = err {
                            warn!("Disconnected from serial port {port:?}: {err}");
                        }
                        ctl.close();
                        break;
                    }
                },
                Some(d) = deferred_rx.recv() => ctl.deferred(d),
                _ = query_timer.tick() => ctl.tick(),
            }
        }
    });
    Ok(handle)
}

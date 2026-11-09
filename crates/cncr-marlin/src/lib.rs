//! Marlin controller (port of the cncjs `MarlinController`): send-response streaming,
//! M105/M114 polling, M109/M190 heating holds and the Marlin manual tool change.

mod controller;
mod runner;

use std::time::Duration;

use cncr_core::controller::{ControllerHandle, ControllerMsg, ControllerOptions, Services};
use cncr_core::json;
use cncr_core::serial::{SerialConnection, SerialEvent};
use tokio::sync::mpsc;
use tokio::time::{interval_at, sleep, Instant, MissedTickBehavior};
use tracing::{debug, error, warn};

pub use controller::Port;
use controller::{Controller, Internal};

pub const TYPE: &str = "Marlin";

impl Port for SerialConnection {
    fn write(&mut self, data: &str) {
        SerialConnection::write(self, data);
    }
}

/// Opens the serial port and spawns the controller task.
pub async fn open(
    services: Services,
    options: ControllerOptions,
) -> Result<ControllerHandle, String> {
    let (serial_tx, serial_rx) = mpsc::unbounded_channel();
    let (path, baud, rtscts) = (options.port.clone(), options.baudrate, options.rtscts);
    let opened = tokio::task::spawn_blocking(move || {
        SerialConnection::open(&path, baud, rtscts, move |ev| {
            let _ = serial_tx.send(ev);
        })
    })
    .await
    .map_err(|e| e.to_string())?;
    let mut conn =
        opened.inspect_err(|e| error!("Error opening serial port {:?}: {e}", options.port))?;

    let pin = &options.pin;
    if pin.dtr.is_some() || pin.rts.is_some() {
        sleep(Duration::from_millis(100)).await;
        let set = pin
            .dtr
            .map_or(Ok(()), |v| conn.set_dtr(v))
            .and_then(|()| pin.rts.map_or(Ok(()), |v| conn.set_rts(v)));
        if let Err(e) = set {
            error!("Failed to set control flags on {:?}: {e}", options.port);
        }
        sleep(Duration::from_millis(100)).await;
    }

    services.hub.broadcast(
        "serialport:change",
        &[json!({ "port": options.port, "inuse": true })],
    );
    debug!("Connected to serial port {:?}", options.port);

    let (tx, rx) = mpsc::unbounded_channel();
    let (internal_tx, internal_rx) = mpsc::unbounded_channel();
    let handle = ControllerHandle::new(TYPE, options.port.clone(), tx);
    let mut ctl = Controller::new(services, options, Box::new(conn), internal_tx);
    ctl.post_open();
    tokio::spawn(run(ctl, rx, serial_rx, internal_rx));
    Ok(handle)
}

async fn run(
    mut ctl: Controller,
    mut rx: mpsc::UnboundedReceiver<ControllerMsg>,
    mut serial_rx: mpsc::UnboundedReceiver<SerialEvent>,
    mut internal_rx: mpsc::UnboundedReceiver<Internal>,
) {
    let period = Duration::from_millis(250);
    let mut timer = interval_at(Instant::now() + period, period);
    timer.set_missed_tick_behavior(MissedTickBehavior::Delay);
    loop {
        tokio::select! {
            msg = rx.recv() => match msg {
                Some(msg) => ctl.handle(msg),
                None => ctl.close(),
            },
            ev = serial_rx.recv() => match ev {
                Some(SerialEvent::Line(line)) => ctl.parse(&line),
                Some(SerialEvent::Closed(err)) => {
                    if let Some(e) = err {
                        warn!("Disconnected from serial port: {e}");
                    }
                    ctl.close();
                }
                None => ctl.close(),
            },
            Some(msg) = internal_rx.recv() => ctl.internal(msg),
            _ = timer.tick() => ctl.tick(),
        }
        if ctl.is_closed() {
            return;
        }
    }
}

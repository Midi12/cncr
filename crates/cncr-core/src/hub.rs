//! Client connections (the socket.io replacement).
//!
//! A frame is the JSON text `{"event": NAME, "args": [...]}`. Frames are serialized once
//! and shared (`Arc<str>`) between every receiving socket.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};
use tokio::sync::mpsc;

pub type SocketId = u64;
pub type Frame = Arc<str>;

static NEXT_SOCKET_ID: AtomicU64 = AtomicU64::new(1);

/// Serializes one event frame.
pub fn frame(event: &str, args: &[Value]) -> Frame {
    json!({ "event": event, "args": args }).to_string().into()
}

/// Sending half of one client connection.
#[derive(Clone, Debug)]
pub struct Socket {
    pub id: SocketId,
    tx: mpsc::UnboundedSender<Frame>,
}

impl Socket {
    /// Creates a socket and the receiver its connection task drains.
    pub fn new() -> (Self, mpsc::UnboundedReceiver<Frame>) {
        let (tx, rx) = mpsc::unbounded_channel();
        let id = NEXT_SOCKET_ID.fetch_add(1, Ordering::Relaxed);
        (Self { id, tx }, rx)
    }

    pub fn emit(&self, event: &str, args: &[Value]) {
        self.send_frame(frame(event, args));
    }

    pub fn send_frame(&self, frame: Frame) {
        // A closed receiver means the client is gone; its disconnect handler cleans up.
        let _ = self.tx.send(frame);
    }
}

/// The set of sockets attached to one controller (cncjs `controller.sockets`).
#[derive(Debug, Default)]
pub struct Sockets(HashMap<SocketId, Socket>);

impl Sockets {
    pub fn add(&mut self, socket: Socket) {
        self.0.insert(socket.id, socket);
    }

    pub fn remove(&mut self, id: SocketId) {
        self.0.remove(&id);
    }

    pub fn ids(&self) -> Vec<SocketId> {
        self.0.keys().copied().collect()
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }

    /// Emits to every attached socket.
    pub fn emit(&self, event: &str, args: &[Value]) {
        if self.0.is_empty() {
            return;
        }
        let f = frame(event, args);
        for s in self.0.values() {
            s.send_frame(f.clone());
        }
    }
}

/// Every connected client (cncjs `io`), used for server-wide broadcasts.
#[derive(Debug, Default)]
pub struct Hub {
    sockets: Mutex<HashMap<SocketId, Socket>>,
}

impl Hub {
    pub fn add(&self, socket: Socket) {
        self.sockets.lock().unwrap().insert(socket.id, socket);
    }

    pub fn remove(&self, id: SocketId) {
        self.sockets.lock().unwrap().remove(&id);
    }

    pub fn broadcast(&self, event: &str, args: &[Value]) {
        let f = frame(event, args);
        for s in self.sockets.lock().unwrap().values() {
            s.send_frame(f.clone());
        }
    }
}

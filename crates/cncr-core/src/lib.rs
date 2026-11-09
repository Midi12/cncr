//! Shared core of the cncr server (Rust rewrite of the cncjs backend).
//!
//! Controllers (Grbl, Marlin, Smoothie, TinyG) live in their own crates and are built
//! from the pieces exported here. Every controller runs as one tokio task that owns
//! all of its state (the JS single-threaded event loop, one task per port).

pub mod autolevel;
pub mod config;
pub mod controller;
pub mod events;
pub mod expr;
pub mod feeder;
pub mod gcode;
pub mod hub;
pub mod monitor;
pub mod program;
pub mod sender;
pub mod serial;
pub mod taskrunner;
pub mod util;
pub mod workflow;

pub use serde_json::{json, Map, Value};

/// Per-line / per-program variable scope (the cncjs "context" object).
pub type Context = Map<String, Value>;

// Write sources (the `source` field of the `serialport:write` context).
pub const WRITE_SOURCE_CLIENT: &str = "client";
pub const WRITE_SOURCE_SERVER: &str = "server";
pub const WRITE_SOURCE_FEEDER: &str = "feeder";
pub const WRITE_SOURCE_SENDER: &str = "sender";

// M6 tool change policies (`tool.toolChangePolicy` in the config).
pub const TOOL_CHANGE_POLICY_IGNORE_M6_COMMANDS: i64 = 0;
pub const TOOL_CHANGE_POLICY_SEND_M6_COMMANDS: i64 = 1;
pub const TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_WCS: i64 = 2;
pub const TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_TLO: i64 = 3;
pub const TOOL_CHANGE_POLICY_MANUAL_TOOL_CHANGE_CUSTOM_PROBING: i64 = 4;

pub const IMPERIAL_UNITS: &str = "in";
pub const METRIC_UNITS: &str = "mm";

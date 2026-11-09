//! `cncr`: CNC controller server (Grbl, Marlin, Smoothieware, TinyG/g2core).
//!
//! Rust rewrite of the cncjs backend: REST API under `/api`, realtime events over a
//! JSON WebSocket at `/ws` (the socket.io events of cncjs, one frame per event).

// Handlers return `Result<_, Response>` for early error responses (idiomatic axum).
#[allow(clippy::result_large_err)]
mod api;
mod auth;
mod engine;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use clap::Parser;
use cncr_core::config::ConfigStore;
use cncr_core::controller::{Registry, Services};
use cncr_core::hub::Hub;
use cncr_core::monitor::Monitor;
use cncr_core::taskrunner::TaskRunner;
use serde_json::{json, Value};
use tracing::{error, info, warn};

#[derive(Parser, Debug)]
#[command(
    name = "cncr",
    version,
    about = "CNC controller server for Grbl, Marlin, Smoothieware and TinyG/g2core"
)]
struct Cli {
    /// Listen port.
    #[arg(short, long, default_value_t = 8000)]
    port: u16,
    /// Listen address.
    #[arg(short = 'H', long, default_value = "0.0.0.0")]
    host: String,
    /// Config file (default: ~/.cncrc).
    #[arg(short, long)]
    config: Option<PathBuf>,
    /// Increase verbosity (-v, -vv, -vvv).
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,
    /// Watch a directory for G-code files.
    #[arg(short, long)]
    watch_directory: Option<PathBuf>,
    /// Access token lifetime in seconds or a time span string (e.g. 3600, 30m, 12h, 30d).
    #[arg(long)]
    access_token_lifetime: Option<String>,
    /// Allow access from non-private networks.
    #[arg(long)]
    allow_remote_access: bool,
    /// Load only this controller: Grbl | Marlin | Smoothie | TinyG | g2core.
    #[arg(long)]
    controller: Option<String>,
}

/// Process-wide server state shared by the HTTP handlers and the WebSocket engine.
pub struct AppState {
    pub services: Services,
    pub secret: String,
    pub token_lifetime_secs: u64,
    pub allow_remote_access: bool,
    pub controllers: Vec<&'static str>,
}

fn home_dir() -> PathBuf {
    std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."))
}

/// Controllers enabled by `--controller` / config `controller` (all when empty/invalid).
fn loaded_controllers(filter: &str) -> Vec<&'static str> {
    let all = [
        cncr_grbl::TYPE,
        cncr_marlin::TYPE,
        cncr_smoothie::TYPE,
        cncr_tinyg::TYPE,
    ];
    let f = filter.to_ascii_lowercase();
    let selected: Vec<&'static str> = match f.as_str() {
        "g2core" | "tinyg" => vec![cncr_tinyg::TYPE],
        _ => all
            .iter()
            .copied()
            .filter(|t| t.to_ascii_lowercase() == f)
            .collect(),
    };
    if selected.is_empty() {
        all.to_vec()
    } else {
        selected
    }
}

#[tokio::main]
async fn main() {
    let cli = Cli::parse();

    let level = match cli.verbose {
        0 => "info",
        1 | 2 => "debug",
        _ => "trace",
    };
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(level)),
        )
        .init();

    let rcfile = cli
        .config
        .clone()
        .unwrap_or_else(|| home_dir().join(".cncrc"));
    info!("Loading configuration from {:?}", rcfile);
    let config = Arc::new(ConfigStore::load(&rcfile));

    let secret = match config
        .get("secret")
        .and_then(|v| v.as_str().map(str::to_string))
    {
        Some(s) if !s.is_empty() => s,
        _ => {
            let s = auth::generate_secret();
            config.set("secret", json!(s));
            s
        }
    };

    let watch_dir = cli
        .watch_directory
        .clone()
        .or_else(|| {
            config
                .get("watchDirectory")
                .and_then(|v| v.as_str().map(PathBuf::from))
        })
        .filter(|p| !p.as_os_str().is_empty());
    let watch_root = match watch_dir {
        Some(dir) if dir.is_dir() => Some(dir),
        Some(dir) => {
            error!("The directory {:?} does not exist.", dir);
            None
        }
        None => None,
    };

    let lifetime = cli
        .access_token_lifetime
        .clone()
        .or_else(|| {
            config
                .get("accessTokenLifetime")
                .map(|v| v.as_str().map_or_else(|| v.to_string(), str::to_string))
        })
        .unwrap_or_else(|| "30d".into());
    let token_lifetime_secs = auth::parse_lifetime(&lifetime).unwrap_or_else(|| {
        warn!("Invalid access token lifetime {lifetime:?}, using 30d");
        30 * 86_400
    });

    let allow_remote_access = cli.allow_remote_access || config.get_or("allowRemoteAccess", false);
    if allow_remote_access && config.get_array("users").is_empty() {
        warn!("You've enabled remote access to the server. It's recommended to create an user account to protect against malicious attacks.");
    }

    let hub = Arc::new(Hub::default());
    let monitor = Arc::new(Monitor::new(watch_root));
    monitor.watch(Arc::clone(&hub));
    let tasks = Arc::new(TaskRunner::new(Arc::clone(&hub)));
    {
        let hub = Arc::clone(&hub);
        config.watch(move || hub.broadcast("config:change", &[]));
    }

    let controller_filter = cli
        .controller
        .clone()
        .or_else(|| {
            config
                .get("controller")
                .and_then(|v| v.as_str().map(str::to_string))
        })
        .unwrap_or_default();
    let controllers = loaded_controllers(&controller_filter);
    info!("Loaded controllers: {}", controllers.join(","));

    let services = Services {
        config,
        hub,
        tasks,
        monitor,
        registry: Arc::new(Registry::default()),
    };
    // System trigger: startup.
    services.trigger("startup");

    let state = Arc::new(AppState {
        services,
        secret,
        token_lifetime_secs,
        allow_remote_access,
        controllers,
    });
    let app = api::router(Arc::clone(&state));

    let addr: SocketAddr = match format!("{}:{}", cli.host, cli.port).parse() {
        Ok(a) => a,
        Err(e) => {
            error!("Invalid listen address {}:{}: {e}", cli.host, cli.port);
            std::process::exit(1);
        }
    };
    let listener = match tokio::net::TcpListener::bind(addr).await {
        Ok(l) => l,
        Err(e) => {
            error!("Unable to listen on {addr}: {e}");
            std::process::exit(1);
        }
    };
    info!(
        "Starting the server at http://{}",
        listener
            .local_addr()
            .map_or_else(|_| addr.to_string(), |a| a.to_string())
    );

    let mut sigterm = match tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
    {
        Ok(s) => s,
        Err(e) => {
            error!("Unable to install the SIGTERM handler: {e}");
            std::process::exit(1);
        }
    };
    // No graceful HTTP drain: open WebSockets would keep it waiting forever.
    tokio::select! {
        r = axum::serve(listener, app.into_make_service_with_connect_info::<SocketAddr>()) => {
            if let Err(e) = r {
                error!("Server error: {e}");
            }
        }
        _ = tokio::signal::ctrl_c() => info!("Shutting down"),
        _ = sigterm.recv() => info!("Shutting down"),
    }
    // Close every port so controllers stop cleanly.
    for handle in state.services.registry.all() {
        let (tx, rx) = tokio::sync::oneshot::channel();
        if handle.send(cncr_core::controller::ControllerMsg::Close(tx)) {
            let _ = rx.await;
        }
    }
}

/// Turns a JSON body into an object map (missing / invalid bodies become `{}`).
pub fn body_object(body: &[u8]) -> serde_json::Map<String, Value> {
    match serde_json::from_slice::<Value>(body) {
        Ok(Value::Object(m)) => m,
        _ => serde_json::Map::new(),
    }
}

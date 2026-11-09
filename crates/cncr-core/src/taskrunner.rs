//! Runs shell commands (`commands` API, `system` event triggers) and broadcasts
//! `task:start` / `task:finish` / `task:error`.

use std::collections::HashSet;
use std::process::Stdio;
use std::sync::{Arc, Mutex};

use serde_json::json;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;
use tracing::{error, info};

use crate::hub::Hub;
use crate::util::short_id;

#[derive(Debug)]
pub struct TaskRunner {
    hub: Arc<Hub>,
    tasks: Mutex<HashSet<String>>,
}

impl TaskRunner {
    pub fn new(hub: Arc<Hub>) -> Self {
        Self {
            hub,
            tasks: Mutex::new(HashSet::new()),
        }
    }

    /// Starts `command` with `sh -c` and returns its task id. Must run inside tokio.
    pub fn run(self: &Arc<Self>, command: &str, title: &str) -> String {
        let task_id = short_id();
        info!("run task {task_id}: title={title:?}, command={command:?}");
        let spawned = Command::new("sh")
            .arg("-c")
            .arg(command)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .process_group(0)
            .spawn();
        self.tasks.lock().unwrap().insert(task_id.clone());
        self.hub.broadcast("task:start", &[json!(task_id)]);

        let mut child = match spawned {
            Ok(child) => child,
            Err(e) => {
                error!("Failed to start a child process: {e}");
                self.tasks.lock().unwrap().remove(&task_id);
                self.hub
                    .broadcast("task:error", &[json!(task_id), json!(e.to_string())]);
                return task_id;
            }
        };
        let pid = child.id().unwrap_or(0);
        if let Some(out) = child.stdout.take() {
            tokio::spawn(async move {
                let mut lines = BufReader::new(out).lines();
                while let Ok(Some(l)) = lines.next_line().await {
                    println!("PID:{pid}> {l}");
                }
            });
        }
        if let Some(err) = child.stderr.take() {
            tokio::spawn(async move {
                let mut lines = BufReader::new(err).lines();
                while let Ok(Some(l)) = lines.next_line().await {
                    eprintln!("PID:{pid}> {l}");
                }
            });
        }
        let runner = Arc::clone(self);
        let id = task_id.clone();
        tokio::spawn(async move {
            let status = child.wait().await;
            if runner.tasks.lock().unwrap().remove(&id) {
                match status {
                    Ok(s) => runner
                        .hub
                        .broadcast("task:finish", &[json!(id), json!(s.code())]),
                    Err(e) => runner
                        .hub
                        .broadcast("task:error", &[json!(id), json!(e.to_string())]),
                }
            }
        });
        task_id
    }
}

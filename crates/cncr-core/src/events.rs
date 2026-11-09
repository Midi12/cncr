//! User-configured event triggers (`events` in the config, cncjs `EventTrigger`).

use serde_json::Value;

use crate::config::ConfigStore;

/// How a matching event runs its commands.
#[derive(Debug, Clone, PartialEq)]
pub enum Trigger {
    /// Shell command, run through the task runner.
    System(String),
    /// G-code, fed to the controller.
    Gcode(String),
}

/// Enabled triggers configured for `event` (e.g. `gcode:start`, `port:open`).
pub fn triggers(config: &ConfigStore, event: &str) -> Vec<Trigger> {
    if event.is_empty() {
        return vec![];
    }
    config
        .get_array("events")
        .iter()
        .filter(|e| e.get("event").and_then(Value::as_str) == Some(event))
        .filter(|e| e.get("enabled").and_then(Value::as_bool).unwrap_or(false))
        .map(|e| {
            let commands = e
                .get("commands")
                .and_then(Value::as_str)
                .unwrap_or_default()
                .to_string();
            match e.get("trigger").and_then(Value::as_str) {
                Some("system") => Trigger::System(commands),
                _ => Trigger::Gcode(commands),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn only_enabled_matching_events() {
        let config = ConfigStore::in_memory(json!({"events": [
            {"event": "gcode:start", "trigger": "system", "commands": "echo hi", "enabled": true},
            {"event": "gcode:start", "trigger": "gcode", "commands": "M3", "enabled": false},
            {"event": "gcode:stop", "trigger": "gcode", "commands": "M5", "enabled": true},
        ]}));
        assert_eq!(
            triggers(&config, "gcode:start"),
            vec![Trigger::System("echo hi".into())]
        );
        assert_eq!(
            triggers(&config, "gcode:stop"),
            vec![Trigger::Gcode("M5".into())]
        );
    }
}

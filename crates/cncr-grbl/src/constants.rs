//! Grbl tables (port of `controllers/Grbl/constants.js`).

/// `~` cycle start, `!` feed hold, `?` status report, `^X` reset.
pub const GRBL_REALTIME_COMMANDS: [&str; 4] = ["~", "!", "?", "\x18"];

pub const GRBL_ACTIVE_STATE_IDLE: &str = "Idle";
pub const GRBL_ACTIVE_STATE_RUN: &str = "Run";
pub const GRBL_ACTIVE_STATE_HOLD: &str = "Hold";
pub const GRBL_ACTIVE_STATE_ALARM: &str = "Alarm";

/// `GRBL_MODAL_GROUPS`: group name and its modes.
pub const GRBL_MODAL_GROUPS: [(&str, &[&str]); 9] = [
    (
        "motion",
        &[
            "G0", "G1", "G2", "G3", "G38.2", "G38.3", "G38.4", "G38.5", "G80",
        ],
    ),
    ("wcs", &["G54", "G55", "G56", "G57", "G58", "G59"]),
    ("plane", &["G17", "G18", "G19"]),
    ("units", &["G20", "G21"]),
    ("distance", &["G90", "G91"]),
    ("feedrate", &["G93", "G94"]),
    ("program", &["M0", "M1", "M2", "M30"]),
    ("spindle", &["M3", "M4", "M5"]),
    ("coolant", &["M7", "M8", "M9"]),
];

/// `GRBL_ERRORS`: code and message (descriptions are not used server-side).
pub const GRBL_ERRORS: &[(u8, &str)] = &[
    (1, "Expected command letter"),
    (2, "Bad number format"),
    (3, "Invalid statement"),
    (4, "Value < 0"),
    (5, "Setting disabled"),
    (6, "Value < 3 usec"),
    (7, "EEPROM read fail. Using defaults"),
    (8, "Not idle"),
    (9, "G-code lock"),
    (10, "Homing not enabled"),
    (11, "Line overflow"),
    (12, "Step rate > 30kHz"),
    (13, "Check Door"),
    (14, "Line length exceeded"),
    (15, "Travel exceeded"),
    (16, "Invalid jog command"),
    (17, "Setting disabled"),
    (20, "Unsupported command"),
    (21, "Modal group violation"),
    (22, "Undefined feed rate"),
    (23, "Invalid gcode ID:23"),
    (24, "Invalid gcode ID:24"),
    (25, "Invalid gcode ID:25"),
    (26, "Invalid gcode ID:26"),
    (27, "Invalid gcode ID:27"),
    (28, "Invalid gcode ID:28"),
    (29, "Invalid gcode ID:29"),
    (30, "Invalid gcode ID:30"),
    (31, "Invalid gcode ID:31"),
    (32, "Invalid gcode ID:32"),
    (33, "Invalid gcode ID:33"),
    (34, "Invalid gcode ID:34"),
    (35, "Invalid gcode ID:35"),
    (36, "Invalid gcode ID:36"),
    (37, "Invalid gcode ID:37"),
    (38, "Invalid gcode ID:38"),
];

/// `GRBL_ALARMS`: code and message.
pub const GRBL_ALARMS: &[(u8, &str)] = &[
    (1, "Hard limit"),
    (2, "Soft limit"),
    (3, "Abort during cycle"),
    (4, "Probe fail"),
    (5, "Probe fail"),
    (6, "Homing fail"),
    (7, "Homing fail"),
    (8, "Homing fail"),
    (9, "Homing fail"),
];

/// `GRBL_SETTINGS`: setting, message, units.
pub const GRBL_SETTINGS: &[(&str, &str, &str)] = &[
    ("$0", "Step pulse time", "microseconds"),
    ("$1", "Step idle delay", "milliseconds"),
    ("$2", "Step pulse invert", "mask"),
    ("$3", "Step direction invert", "mask"),
    ("$4", "Invert step enable pin", "boolean"),
    ("$5", "Invert limit pins", "boolean"),
    ("$6", "Invert probe pin", "boolean"),
    ("$10", "Status report options", "mask"),
    ("$11", "Junction deviation", "millimeters"),
    ("$12", "Arc tolerance", "millimeters"),
    ("$13", "Report in inches", "boolean"),
    ("$20", "Soft limits enable", "boolean"),
    ("$21", "Hard limits enable", "boolean"),
    ("$22", "Homing cycle enable", "boolean"),
    ("$23", "Homing direction invert", "mask"),
    ("$24", "Homing locate feed rate", "mm/min"),
    ("$25", "Homing search seek rate", "mm/min"),
    ("$26", "Homing switch debounce delay", "milliseconds"),
    ("$27", "Homing switch pull-off distance", "millimeters"),
    ("$30", "Maximum spindle speed", "RPM"),
    ("$31", "Minimum spindle speed", "RPM"),
    ("$32", "Laser-mode enable", "boolean"),
    ("$100", "X-axis travel resolution", "step/mm"),
    ("$101", "Y-axis travel resolution", "step/mm"),
    ("$102", "Z-axis travel resolution", "step/mm"),
    ("$110", "X-axis maximum rate", "mm/min"),
    ("$111", "Y-axis maximum rate", "mm/min"),
    ("$112", "Z-axis maximum rate", "mm/min"),
    ("$120", "X-axis acceleration", "mm/sec^2"),
    ("$121", "Y-axis acceleration", "mm/sec^2"),
    ("$122", "Z-axis acceleration", "mm/sec^2"),
    ("$130", "X-axis maximum travel", "millimeters"),
    ("$131", "Y-axis maximum travel", "millimeters"),
    ("$132", "Z-axis maximum travel", "millimeters"),
];

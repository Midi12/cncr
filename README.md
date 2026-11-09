# cncr

CNC controller server for [Grbl](https://github.com/gnea/grbl) (incl. Grbl-Mega),
[Marlin](https://github.com/MarlinFirmware/Marlin), [Smoothieware](https://github.com/Smoothieware/Smoothieware)
and [TinyG](https://github.com/synthetos/TinyG) / [g2core](https://github.com/synthetos/g2).

`cncr` is a Rust rewrite of the [cncjs](https://github.com/cncjs/cncjs) backend with the web
interface removed. It keeps the cncjs functional core — serial connection, controller protocols,
G-code streaming (character-counting and send-response), feeder, workflow, macros and
`[expression]` substitution, tool change, auto-leveling, watch directory, commands, events, users —
and exposes it to native or remote clients over a REST API and a JSON WebSocket. It is built to run
on small boards such as the Raspberry Pi Zero 2 W: one ~6 MB binary (glibc only), no Node.js.

The companion touch HMI is [cncjs-shopfloor-tablet](../cncjs-shopfloor-tablet) (native Rust/egui).

## Layout

| Crate | Content |
|---|---|
| `crates/cncr-core` | config store, sender, feeder, workflow, serial port, JS-expression evaluator, G-code helpers, auto-level, watch directory, task runner |
| `crates/cncr-grbl`, `cncr-marlin`, `cncr-smoothie`, `cncr-tinyg` | one controller each (one tokio task per open port) |
| `crates/cncr` | the `cncr` binary: WebSocket engine, REST API, access control, CLI |
| `grbl-simulator/` | Node.js Grbl v1.1 simulator used for manual end-to-end testing (dev tool only) |

## Build

```sh
cargo build --release                 # native
# Raspberry Pi (64-bit OS / 32-bit OS) with cargo-zigbuild; the suffix pins the glibc
# version (2.31 = Pi OS Bullseye and newer):
cargo zigbuild --release -p cncr --target aarch64-unknown-linux-gnu.2.31
cargo zigbuild --release -p cncr --target armv7-unknown-linux-gnueabihf.2.31
cargo test
```

The serial layer does not need libudev (USB manufacturer names are reported when the platform
exposes them).

## Run

```sh
cncr [options]
  -p, --port <port>                    listen port (default 8000)
  -H, --host <host>                    listen address (default 0.0.0.0)
  -c, --config <file>                  config file (default ~/.cncrc)
  -v, --verbose                        -v / -vv debug, -vvv trace (or RUST_LOG)
  -w, --watch-directory <dir>          G-code watch directory
      --access-token-lifetime <span>   3600, 30m, 12h, 30d (default 30d)
      --allow-remote-access            allow clients outside loopback/private networks
      --controller <type>              load only Grbl | Marlin | Smoothie | TinyG | g2core
```

Example systemd unit (`/etc/systemd/system/cncr.service`):

```ini
[Unit]
Description=cncr CNC controller server
After=network.target

[Service]
User=pi
ExecStart=/home/pi/cncr -w /home/pi/GCode
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

### Configuration (`~/.cncrc`)

Same file and keys as cncjs, so an existing `.cncrc` works unchanged: `watchDirectory`,
`accessTokenLifetime`, `allowRemoteAccess`, `controller`, `secret`, `ports`, `baudrates`, `users`,
`commands`, `events`, `macros`, `mdi`, `machines`, `tool` (tool change policy and positions),
`state.controller.exception.ignoreErrors`. The file is reloaded when edited by hand.
`mountPoints` is ignored (no web UI is served).

## Access control

- Requests are accepted from loopback, private (10/8, 172.16/12, 192.168/16) and link-local
  addresses only, unless `allowRemoteAccess` is set. Unlike cncjs this also applies to the REST API.
- Every `/api/*` route except `/api/signin`, and the WebSocket, require an access token
  (`Authorization: Bearer <token>` or `?token=`). Tokens are HS256 JWTs signed with `secret`
  (generated on first start). With no users configured, `/api/signin` hands out a token to any
  allowed client; with users, `{name, password}` is checked against their bcrypt hashes.

## Protocol

### REST

| Route | Methods |
|---|---|
| `/api/signin` | POST `{token?, name?, password?}` → `{enabled, token, name}` |
| `/api/controllers` | GET open controllers' status |
| `/api/gcode?port=` | GET loaded program (+ `data`); POST `{port, name, gcode, context}` loads one |
| `/api/gcode/download?port=` | GET/POST the loaded program as a file |
| `/api/watch/status`, `/api/watch/files?path=` | watch directory state and listing |
| `/api/watch/file?file=` | GET/POST read, PUT write (JSON `{file, data}` or raw `application/octet-stream` with `?file=`, streamed), DELETE |
| `/api/watch/file/rename` | POST `{file, to}` (never overwrites) |
| `/api/commands[/:id]`, `/api/commands/run/:id` | shell commands CRUD + run |
| `/api/events[/:id]` | event triggers CRUD |
| `/api/macros[/:id]`, `/api/mdi[/:id]`, `/api/machines[/:id]`, `/api/users[/:id]` | CRUD (`?paging=1&page=&pageLength=`) |
| `/api/state?key=`, `/api/tool` | client state / tool change settings |

Errors are `{"msg": "..."}` with the HTTP status.

### WebSocket `/ws?token=<token>`

One JSON text frame per event, mirroring the cncjs socket.io events one-to-one.

Client → server: `{"event": "...", "args": [...], "id": 7}` — `id` is optional; when present the
server answers `{"event": "ack", "id": 7, "args": [error_or_null, data]}` (socket.io callbacks).

| Event | Args |
|---|---|
| `list` | — (answers `serialport:list`) |
| `open` | `port, {controllerType: "Grbl"\|"Marlin"\|"Smoothie"\|"TinyG", baudrate, rtscts?, pin?: {dtr?, rts?}}` |
| `close` | `port` |
| `command` | `port, cmd, ...args` — `gcode`, `gcode:load`, `gcode:unload`, `gcode:start`, `gcode:stop` (`{force}`), `gcode:pause`, `gcode:resume`, `feeder:feed`, `feeder:start`, `feeder:stop`, `feedhold`, `cyclestart`, `statusreport`, `homing`, `sleep`, `unlock`, `reset`, `jogCancel`, `feedOverride`, `spindleOverride`, `rapidOverride`, `lasertest:on`, `lasertest:off`, `macro:run`, `macro:load`, `watchdir:load`, `tool:change`, `autolevel:start`, `autolevel:stop`, `autolevel:getProbeState`, `autolevel:loadFromFile`, `autolevel:saveToFile`, `autolevel:applyProbeCompensation` (controller-specific commands as in cncjs) |
| `write`, `writeln` | `port, data, context?` |

Server → client: `startup`, `serialport:list`, `serialport:open`, `serialport:close`,
`serialport:change`, `serialport:error`, `serialport:read`, `serialport:write`,
`controller:settings`, `controller:state`, `feeder:status`, `sender:status`, `workflow:state`,
`gcode:load`, `gcode:unload`, `task:start`, `task:finish`, `task:error`, `config:change`,
`watchdir:change`, `autolevel:update`, `autolevel:complete` — same payloads as cncjs.

## Differences from cncjs

- No web UI, no mount points / static file serving, no socket.io (plain WebSocket JSON frames, see
  above), no i18n of server messages, no `/api/version/latest` (npm update check).
- The backward-compatibility duplicates `Grbl:state`, `Grbl:settings`, `Marlin:state`, … are not
  emitted; use `controller:state` / `controller:settings`.
- Bare numeric `accessTokenLifetime` values are seconds (as documented), not milliseconds.
- `/api/watch/file` reads cannot escape the watch directory.

## End-to-end check with the simulator

```sh
cd grbl-simulator && node grbl-server.js 3333 &
socat pty,raw,echo=0,link=/tmp/ttyGRBL tcp:localhost:3333 &
cncr -c /tmp/cncrc.json        # with {"ports": [{"path": "/tmp/ttyGRBL"}]}
```

## License

MIT. Based on cncjs by Cheton Wu and contributors.

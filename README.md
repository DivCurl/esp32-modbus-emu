# ESP32 Modbus emulator

ESP32 firmware that emulates a Modbus RTU slave device over RS-232, for testing Modbus masters,
gateways and SCADA drivers on the bench. Registers, emulated data and serial settings are all
configured from a web page, with no rebuild or reboot. For an RS-485 master, add an RS-485/RS-232
converter in front of the MAX3232.

- **Modbus:** RTU slave serving all four tables. Reads only for now: master writes are coming.

  | Table | Function code | Capacity |
  |---|---|---|
  | 0x Coils | FC01 | 65,536 bits, shared with 1x |
  | 1x Discrete inputs | FC02 | (shared with 0x) |
  | 3x Input registers | FC04 | 16,384 registers, shared with 4x |
  | 4x Holding registers | FC03 | (shared with 3x) |

- **Address modes:**
  - **Open, the default:** every address in all four tables answers, reading 0 where nothing is
    stored. A rule gets storage automatically, in pages of 64 addresses.
  - **Strict:** only the blocks you define answer, and anything else returns exception 02, as from a
    real device. Use it to copy a manufacturer's published register map.
- **Memory map:** up to 320 blocks, each giving storage to a range of addresses from 0 to 65535.
- **Emulation:** up to 256 rules, each driving a range of values inside the blocks. A value can be
  static, random, a sine wave or a square wave. Static values can step for each value, so one rule
  can fill a range with 3001, 3002, 3003 and so on. Addresses with no rule keep their value.
- **Data types:** BOOL for 0x and 1x. UINT16, INT16, UINT32, INT32, FLOAT32 (REAL) and FLOAT64 for
  3x and 4x, each with a selectable byte order. 32-bit types take two registers and FLOAT64 four.
- **Status:** TX, RX and WiFi status LEDs. The web page shows error counters, recent requests and a
  capture of the last 32 raw frames in both directions, including frames that were rejected.

Modbus runs in its own task and doesn't depend on WiFi, so a network drop doesn't affect the serial
link.

## Hardware

See [docs/wiring.svg](docs/wiring.svg).

| Function | GPIO |
|---|---|
| Port 1, UART2 TX, to the MAX3232 | 17 |
| Port 1, UART2 RX, from the MAX3232 | 16 |
| Port 2, UART1 TX, to the MAX3232 (reserved for master mode) | 18 |
| Port 2, UART1 RX, from the MAX3232 (reserved for master mode) | 19 |
| TX LED | 25 |
| RX LED | 26 |
| WiFi status LED | 27 |

The WiFi LED shows the connection to your network:

| LED | Meaning |
|---|---|
| Slow blink, one second on and one off | Connecting |
| Solid | Connected, with an IP address |
| Double flash, then a pause | Fault: no credentials saved, 5 failed attempts in a row, or 60 seconds without a connection |

It keeps retrying during a fault, so the LED goes solid by itself when the network returns.
Modbus keeps running throughout.

## Defaults

| Setting | Default |
|---|---|
| Serial | 57600 baud, 8N1 |
| Slave ID | 1 |
| Registers | 3x3001 to 3x3100, wire addresses 3000 to 3099 |

Each default register holds its own number as a static value: 3x3001 reads 3001. If the master shows
3002 there, its addressing is off by one. The web page lists every request it receives, so you can
also see the exact wire addresses being read.

## Build and flash

Needs ESP-IDF 5.4.

```
idf.py build
idf.py -p COM3 flash monitor
```

## Connect to WiFi

Either way works. Credentials saved from the web page take priority.

- **From the web page:** on first boot with no credentials, join the `Modbus-Emu-Setup` network
  (password `modbus-setup`) and open <http://192.168.4.1>. The setup network stays up for a minute
  after connecting, so the page can show the new address.
- **At build time:** copy `main/secrets.example.h` to `main/secrets.h` and fill it in. This file is
  gitignored.

The address is also printed to the serial log. If the network can't be reached after five attempts,
the setup network comes back while the device keeps retrying.

Set `EMU_WEB_PASSWORD` and `EMU_AP_PASSWORD` in `main/secrets.h` to protect changes. By default,
anyone on your network can change settings. Traffic is plain HTTP, so keep this on a trusted network.

## Web API

| Request | Purpose |
|---|---|
| `GET /api/status` | WiFi, serial settings, counters and recent requests |
| `GET /api/frames` | Last 32 raw frames as `[time_ms, len, flags, hex]`, flags being the `MB_CAP_` values in `main/modbus_slave.h` |
| `POST /api/stats/reset` | Clear the counters, recent requests and raw frames |
| `GET /api/map` | `strict`, capacity, then blocks as `[kind, start, count, flags]` and rules as `[kind, start, count, type, order, mode, value, value_step, min, max, period_ms, phase_deg, phase_step_deg, duty_pct]` |
| `GET /api/values?kind=3&start=3000&count=100` | Up to 256 raw registers, or 0 and 1 for bits, with `null` outside every block |
| `POST /api/blocks` | Add a block: `kind`, `start`, `count`, `read_only` |
| `POST /api/blocks/delete` | Delete the block containing `addr` in `kind`, with its rules |
| `POST /api/rules` | Apply a rule: `kind`, `start`, `count` in values, `mode`, then any of `type`, `order`, `value`, `value_step`, `min`, `max`, `period_ms`, `phase_deg`, `phase_step_deg`, `duty_pct` |
| `POST /api/rules/clear` | Remove rules from `count` addresses at `start`, keeping the values |
| `POST /api/map/reset` | Restore the default map |
| `POST /api/map/mode` | `strict`: `true` for strict mode, `false` for open |
| `POST /api/serial` | `baud`, `parity` (`N`, `E`, `O`), `stop_bits`, `slave_id` |
| `POST /api/wifi` | `ssid`, `password` |

`kind` is 0 for coils, 1 for discrete inputs, 3 for input registers or 4 for holding registers, and
`start` is the zero-based wire address. `mode` is 0 static, 1 random, 2 sine or 3 square. `type` is
0 UINT16, 1 INT16, 2 UINT32, 3 INT32, 4 FLOAT32, 5 FLOAT64 or 6 BOOL, and `order` is 0 ABCD, 1 CDAB,
2 BADC or 3 DCBA. Block `flags` are 1 for read-only and 2 for a block added automatically. A rule
overlapping another trims it. POST bodies are JSON and need an
`X-Emu-Password` header holding
`=` followed by the password. With no password set, send just `=`.

## Known behaviour

Saving to flash pauses the CPU briefly, so a poll that lands on a save may time out once. This only
happens when you change settings.

## Tests

`test/host_test.c` covers frame handling, the memory map, emulation and the WiFi LED on a PC. The
build command is at the top of that file.

`tools/mb_poll.py` is a bench-test Modbus master for a USB-serial adapter, and needs `pyserial`:

```
python tools/mb_poll.py COM5 --start 3000 --qty 100
```

## Roadmap

- Master writes: FC05, FC06, FC15 and FC16, with read-only blocks returning an exception.
- Bit-pattern and follow modes, and map profiles saved as JSON.
- A Modbus/TCP server alongside RTU, and several slave IDs.
- A configurable Modbus master mode, for acting as a gateway.

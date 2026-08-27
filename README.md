# Growatt SPH Heater Controller

An ESP32 sketch that reads a **Growatt SPH 10000TL3 BH-UP** hybrid inverter over
Modbus RTU / RS485, serves a dashboard from its own WiFi access point, and switches
a relay to dump surplus solar into a water heater.

Single file, no build system, no external configuration — `growatt-heater-en-rs485.ino`
is the whole project.

![platform](https://img.shields.io/badge/platform-ESP32-blue)
![protocol](https://img.shields.io/badge/protocol-Modbus%20RTU-orange)
![license](https://img.shields.io/badge/license-MIT-green)

> [!IMPORTANT]
> **Plug the RS485 cable into the inverter's `RS485-3` socket.**
> On the `METER` sockets the inverter is the Modbus *master* polling its
> own energy meter, and inverter data cannot be read from there at all —
> neither actively nor passively. This is the single most common reason
> people spend weeks getting nothing back.
> See [Finding the right port](#finding-the-right-port--read-this-before-wiring).

---

## ⚠️ Safety first

The output of this controller switches **230 V AC** through a relay. If you
build this:

- The GPIO is driven **LOW as the very first statement in `setup()`** — the safe
  state after every boot and brownout. Keep it that way.
- Every fail-safe path leads to **off**, never to on. There are three watchdogs
  (see [SSR control](#ssr-control)); do not remove them.
- Mains wiring, the SSR heatsink and the heating element are outside the scope of
  this repository. If you are not qualified to do that work, have someone who is
  do it.

---

## What it does

```
  Growatt SPH ──RS485──► ESP32 ──┬──► WiFi AP + dashboard (192.168.10.1)
   (Modbus slave)                │
                                 └──► relay module ──► water heater (230 V)
```

Every 30 seconds the ESP32 polls three register blocks from the inverter and prints
the full decoded state to the serial console. Every 60 seconds it decides whether
the heater should be running.

---

## Hardware

| Part | Notes |
|---|---|
| LILYGO TTGO T-Call V1.3 | ESP32 + SIM800L + IP5306. The SIM800 is unused, but its pins are physically occupied. |
| Waveshare TTL ↔ RS485 transceiver | A / B screw terminals. Any MAX485-style module with a DE+RE line works. |
| Relay module, 5 V coil | Powered from a separate 5 V supply, input driven straight from 3.3 V logic. No series resistor. |

### GPIO map

| GPIO | Function |
|---|---|
| 19 | UART2 RX ← transceiver RO |
| 18 | UART2 TX → transceiver DI |
| 25 | transceiver DE + RE, labelled `RSE` on the Waveshare board |
| 2 | relay module IN (3.3 V logic) |
| 13 | built-in status LED |

Board photos, the wiring schematic and the inverter's installation manual are in
[`docs/`](docs/).

**Pins the T-Call board already uses — do not touch:** 4, 5 (SIM800 PWRKEY/RST),
16, 17 (PSRAM), 21, 22 (I²C / IP5306), 23 (SIM800 POWER_ON), 26, 27 (SIM800 UART),
32, 33 (SIM800 DTR/RI).

That is why UART2 is remapped to 18/19 and DE/RE sits on 25 — none of these are
the ESP32 defaults. On a plain ESP32 dev board you can use whatever you like.

### RS485 to the inverter

Pinout per the SPH installation manual, section 5.2. It is the same for the
`485-1`, `485-2`, `485-3` and `METER` sockets. With a standard **T568B** patch
cable:

| RJ45 pin | Wire colour | Signal | Transceiver terminal |
|---|---|---|---|
| 5 | white/blue | RS485 A (D+) | **A** |
| 1 | white/orange | RS485 B (D−) | **B** |
| 2 | orange | GND | GND (optional) |

Swapping A and B is harmless — it is a differential bus, reversed polarity simply
means no data — and it is the first thing to try when nothing comes back. It shows
up as a `0xE2` timeout in the log.

### Relay drive

The heater ended up being switched by an **ordinary relay module**. The solid
state relay and optocoupler this project started with turned out not to be
needed.

```
  5 V supply ──► relay module VCC
         GND ──► relay module GND ──── tied to TTGO GND
       GPIO2 ──► relay module IN       3.3 V logic, no series resistor
```

The coil runs on 5 V from a separate supply, while the module's `IN` pin is
driven straight from the ESP32's 3.3 V GPIO. That works because `IN` is a logic
input, not the coil itself — no series resistor is required. The two grounds must
be tied together, otherwise the signal has no reference.

> [!WARNING]
> The module used here is **active-HIGH**: a high `IN` energises the relay, which
> is what the firmware assumes. Plenty of cheap relay boards are **active-LOW**,
> though, and on those the whole fail-safe design inverts: the firmware drives
> the pin LOW as the first statement in `setup()`, which on an active-LOW board
> would switch the heater **on** at every boot and every brownout. Check which
> kind you have before you connect anything to mains.

Check the contact rating against your element as well. A 2 kW heater draws about
9 A, which sits at the very top of what the common 10 A relay modules are rated
for, and a resistive mains load is hard on relay contacts over thousands of
cycles. This is the argument for the solid state relay the project originally
used — the relay is simpler, the SSR lasts longer.

The firmware still calls this output `SSR_PIN` and `SSR_*` throughout. Those are
identifier names carried over from the original design, not a claim about what
is on the other end of the wire.


---

## Finding the right port — read this before wiring

This is the part that costs people weeks.

The SPH has several RJ45 sockets, and **the inverter is not a Modbus slave on all
of them.** On the `METER` sockets the *inverter itself is the master*, polling the
external energy meter about ten times a second. You cannot get inverter data from
such a port:

- **not actively** — you would be a second master on the bus, and two masters
  corrupt each other's frames. Worse, the inverter loses sight of its meter.
- **not passively** — the only traffic there is meter data, in IEEE-754 floats.
  No PV voltage, no battery SOC.

**The decision rule:** put a bus sniffer on a port and watch.

| What you see | Meaning |
|---|---|
| Request frames (`02 04 00 00 00 12`-style, valid CRC, ~5/s) | The inverter is the master here. **Wrong port.** |
| Silence | The inverter is waiting to be asked. **This is the one.** |

On the unit this was developed against, the answer was **`RS485-3`**. Note that
`485-2` is often occupied by a ShineWiFi dongle. Older SPH units have a different
socket layout — check yours rather than trusting the label.

---

## Modbus

**Slave ID 1, 9600 baud, 8N1, function `0x04` (Read Input Registers).**

Three blocks per cycle, with a **1000 ms gap between them**. The Growatt needs
roughly 850 ms of silence between requests or it simply will not answer. Do not
shorten this.

32-bit values are register pairs, high word first, scaled `× 0.1` unless noted.

### Block 0–16 — PV (mandatory)

| Register | Meaning | Scale |
|---|---|---|
| 0 | inverter status | — |
| 1–2 | total PV power | × 0.1 W |
| 3 / 4 / 5–6 | MPPT1 voltage / current / power | × 0.1 |
| 7 / 8 / 9–10 | MPPT2 voltage / current / power | × 0.1 |
| 11–16 | small stable numbers, **meaning unknown** — *not* PV3/PV4 | — |

If this block fails, the controller declares itself offline.

### Block 1000–1040 — battery, grid, house

| Register | Meaning | Scale |
|---|---|---|
| 1009–1010 | battery discharge | × 0.1 W |
| 1011–1012 | battery charge | × 0.1 W |
| 1013 | battery voltage (~267 V on this HV pack) | × 0.1 V |
| 1014 | state of charge | % |
| 1021–1022 | import from grid | × 0.1 W |
| 1029–1030 | export to grid (surplus) | × 0.1 W |
| 1037–1038 | house load | × 0.1 W |
| 1040 | battery temperature | × 0.1 °C |

Registers `1015–1020`, `1025–1027` and `1034–1036` produce nonsense when read as
32-bit pairs. They are **not** per-phase breakdowns; their meaning is unknown.

### Block 35–38 — AC (optional, partly wrong)

Read separately so a failure cannot break the main read. From the
[`Lu-Fi/ha-growatt-modbus`](https://github.com/Lu-Fi/ha-growatt-modbus) map.

- `37` frequency (× 0.01 Hz) — good.
- `38` grid voltage (× 0.1 V) — good, but it is the **line-to-line** voltage
  (~417 V), not phase voltage.
- `35–36` "Pac" — **wrong.** Checked against the energy balance: it reports
  230–350 W while the real AC output is 430–540 W, with no constant ratio.
  Compute AC output as `house load + export − import` instead.

### Does the map still hold?

Both the serial log and the dashboard continuously check an energy balance:

```
PV + discharge + import  =  house load + charge + export
```

Measured difference is single-digit watts. If the two sides drift more than 5 %
apart, the register map is wrong and none of the values can be trusted — the log
prints `!!! MISMATCH`.

This is how the map above was verified in the first place, along with two other
independent checks: the string powers sum to total PV power within 0.3 %, and
`U × I` matches the string power register.

---

## Two gotchas that make this work

Both of these are the reason a straightforward implementation reads nothing at all.

### 1. `Serial.flush()` returns too early

On the ESP32, `Serial.flush()` comes back *before* the last bits have physically
left the UART. If you drop the DE line right after it, the tail of your frame
never reaches the bus, the slave receives a truncated request, and **it does not
answer.** The symptom is a permanent timeout that looks exactly like bad wiring.

The fix is to wait two byte times before releasing the driver:

```c
void postTransmission() {
  delayMicroseconds((10UL * 1000000UL / MODBUS_BAUD) * 2);
  digitalWrite(MAX485_DE_RE_PIN, LOW);
}
```

### 2. The transceiver echoes your own transmission

The tail of what you send comes back on RX, so the **first bytes you receive are
not the reply.** A byte-oriented Modbus library reads that echo, sees a foreign
address, and returns an error (`0xE0` in ModbusMaster's numbering).

That is why this sketch does not use `ModbusMaster`. `readInputBlock()` collects
the whole stream into a buffer and then scans it for a frame with a valid CRC,
which skips the echo naturally.

---

## SSR control

A four-state machine: `SSR_OFF` and `SSR_ON` are manual, `SSR_AUTO_ON` and
`SSR_AUTO_OFF` are the automatic mode.

AUTO watches **two things only** — string voltage and battery charge. Both
conditions must hold at the same time:

```
BOTH strings > ssrMinPvVoltage   AND   SOC > ssrMinSoc   →  heater on
any condition stops holding                              →  heater off
```

Evaluated every **60 s**. The condition is symmetric, so both AUTO states share a
single branch — the moment it stops holding, the output drops. There is
deliberately **no hysteresis**: if the output oscillates, raise the threshold.

Grid flow is not consulted at all. String voltage is a coarse proxy for irradiance
and SOC protects the battery — and register `1029–1030` has never been observed
non-zero on this installation anyway, because the battery absorbs the entire
surplus below 100 %.

Pressing the AUTO button does **not** switch anything on by itself. It enters
`SSR_AUTO_OFF` and lets the data decide.

### Thresholds

| | Default | Allowed range |
|---|---|---|
| String voltage | 330 V | 300–400 V |
| Battery charge | 90 % | 80–95 % |

Both are adjustable from the dashboard at runtime. They are **not persisted to
flash** — after a power cut the defaults apply again, which is intentional. The
range check in the API handler is the binding one: browser-side JavaScript can be
bypassed, and these two numbers decide whether 230 V gets switched.

Changing a threshold forces an immediate re-evaluation instead of waiting up to
60 s, which matters when a threshold is tightened.

### Fail-safes

All three turn off **AUTO only** — a manual `SSR_ON` is the operator's conscious
decision and survives.

| Condition | Action |
|---|---|
| 90 s without a successful read | AUTO → off |
| 5 min watchdog (backstop) | AUTO → off |
| Block 1000–1040 unavailable | AUTO → off |

`updateSSR()` runs on **every pass of `loop()`**, not only after a read — otherwise
a total Modbus outage would leave the heater on until the next read attempt.

Note that `SURPLUS_WATCHDOG_MS` (90 s) must stay larger than `READ_INTERVAL_MS`
(30 s), or AUTO would switch itself off on every cycle. Change them together.

---

## Web interface

The ESP32 runs as an access point:

- **SSID:** `Growatt-heater`
- **Password:** none — an open network
- **IP:** `192.168.10.1`

> The open network is deliberate for a device sitting in the owner's own yard.
> Anyone who connects can switch the SSR and change the thresholds. To require a
> password, pass one as the second argument to `WiFi.softAP()`.

| Endpoint | Purpose |
|---|---|
| `GET /` | Dashboard, auto-refreshing every 5 s |
| `GET /api/data` | Full state as JSON |
| `POST /api/ssr/on` | Force on |
| `POST /api/ssr/off` | Force off |
| `POST /api/ssr/auto` | Surplus mode |
| `POST /api/limits?pv=330&soc=90` | Change thresholds (query parameters) |

Handlers run in an async context — **no `delay()` and no blocking Modbus calls
inside them.**

If you add a field to `/api/data`, remember the dashboard JavaScript lives in the
same file as `HTML_PAGE[] PROGMEM`, and watch the size of
`StaticJsonDocument<2048>`.

---

## Build

Arduino IDE or `arduino-cli`, board **ESP32 Dev Module**, serial monitor at
115200 baud.

Libraries:

- `WiFi.h`, `HardwareSerial.h` (ESP32 core)
- [`ESPAsyncWebServer`](https://github.com/me-no-dev/ESPAsyncWebServer) (and `AsyncTCP`)
- [`ArduinoJson`](https://arduinojson.org/) — v6 API (`StaticJsonDocument`)

There is nothing to run locally. Verification means flashing the board, so read
changes carefully — a mistake physically switches 230 V.

---

## What is verified and what is not

**Verified by measurement:** port `RS485-3`, slave address 1, registers `0–16` and
`1000–1040`. Confirmed three independent ways — string power sum, `U × I`, and the
house energy balance.

**Not verified — do not build control logic on these:**

- `35–36` "Pac" — demonstrably wrong (see above).
- The status enumeration at register `0`. It reports `5` during normal operation;
  the rest of the mapping is guesswork.
- `1029–1030` export to grid — always `0` in every measurement so far, because the
  battery absorbed the whole surplus. Needs confirming with a full battery.
- `11–16`, `1015–1020`, `1025–1027`, `1034–1036` — meaning unknown.

---

## Credits

Register map starting point:
[`Lu-Fi/ha-growatt-modbus`](https://github.com/Lu-Fi/ha-growatt-modbus).
Everything above was re-verified against the actual inverter, and one register
from that map turned out not to apply to the SPH BH-UP.

---

## License

[MIT](LICENSE) © 2026 [@Oliwer11](https://github.com/Oliwer11)

Note the warranty disclaimer in particular. This software switches mains voltage
through relay hardware you assemble yourself; you are responsible for that
installation being safe and compliant where you live.

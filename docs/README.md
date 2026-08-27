# Documentation

Reference material for the build.

| File | What it is |
|---|---|
| [`Growatt-heater-wiring-diagram.pdf`](Growatt-heater-wiring-diagram.pdf) | Wiring schematic for the whole build |
| [`Growatt_InstMan_SPH_4000-10000TL3_BH-UP_EN.pdf`](Growatt_InstMan_SPH_4000-10000TL3_BH-UP_EN.pdf) | Growatt SPH installation manual |
| [`TTGO-T-call-Pinout-Diagram-Large.jpg`](TTGO-T-call-Pinout-Diagram-Large.jpg) | LILYGO TTGO T-Call V1.3 pinout |
| [`RS485_top_view.png`](RS485_top_view.png) | The Waveshare 3485/485 transceiver actually used |
| [`UART_TTL_RS485_MAX485_5V_ARDUINO.png`](UART_TTL_RS485_MAX485_5V_ARDUINO.png) | Generic MAX485 module, the separate DE/RE variant |
| [`Relay_5V_2-channel.png`](Relay_5V_2-channel.png) | 5 V two-channel relay module |
| [`Relay_5V_2-channel_topwiev.png`](Relay_5V_2-channel_topwiev.png) | Same module, top view |

## Notes

### RS485 pinout in the inverter manual

Section **5.2**, page 16 of the printed manual (PDF page 20). It is the same for
the `485-1`, `485-2`, `485-3` and `METER` sockets — what differs is which of them
has the inverter acting as a Modbus slave. See
[Finding the right port](../README.md#finding-the-right-port--read-this-before-wiring).

### Transceiver header labels

The Waveshare board silkscreens the direction pin as **`RSE`**, not `DE`/`RE`:

| Waveshare pin | Goes to | TTGO |
|---|---|---|
| `VCC` | 3V3 | power |
| `GND` | GND | common ground |
| `RO` | UART2 RX | GPIO19 |
| `DI` | UART2 TX | GPIO18 |
| `RSE` | direction | GPIO25 |

On the generic MAX485 module in the other photo, `DE` and `RE` are two separate
pins and must be tied together before going to GPIO25.

### Licensing

The Growatt manual is included for reference only. It remains the property of
Growatt New Energy and is **not** covered by this project's MIT licence.

# esphome-husqvarna-automower-220ac

Beta ESPhome firmware for husqvarna 220 ac (and probably 230 and some other from the same generation)

An ESP32 connects to the service port on the mower's mainboard and speaks
Husqvarna's G2 protocol over UART, 9600 8N1. Sensors, buttons and the timer
schedule are exposed to Home Assistant.

## Hardware

| Part               | Choice                                                      |
| ------------------ | ----------------------------------------------------------- |
| Board              | ESP32-DevKitC V4 with WROOM-32U, external u.FL antenna      |
| Supply             | 5 V from the service port, into the board's 5V pin          |
| Fuse               | PTC 0.75 A (RXEF075) in series on the 5V conductor          |
| Decoupling 5 V     | 470 µF low ESR + 100 nF in parallel across 5V and GND       |
| Decoupling 3.3 V   | 10 µF + 100 nF in parallel across 3V3 and GND at the module |
| Connector          | 10-way Molex KK, pre-crimped cable                          |
| UART               | UART2, GPIO16 = RX, GPIO17 = TX, 9600 8N1                   |
| Enclosure          | Ventilation holes on two opposite sides                     |

## Service port pinout

The service port is the white 10-way header on the mainboard, seen from above:

```
     1   2   3   4   5   6   7   8   9   10
   ┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐
   │ ▪ │ ▪ │ ▪ │ ▪ │ ▪ │ ▪ │ ▪ │ ▪ │ ▪ │ ▪ │
   └───┴───┴───┴───┴───┴───┴───┴───┴───┴───┘
     Rx  Tx  ·   ·   ·   ·  GND  5V 3V3 18V
```

Photo of the connector with the conductors identified:
<https://static.byggahus.se/attachments/images/large/383/383020-05c5f3dbcdde991ad9f465f44a8f8ba1.jpg>

| Pos | Signal  | Used                            |
| --- | ------- | ------------------------------- |
| 1   | Rx      | yes, from the ESP32 TX (GPIO17) |
| 2   | Tx      | yes, to the ESP32 RX (GPIO16)   |
| 3   | unknown | no                              |
| 4   | unknown | no                              |
| 5   | unknown | no                              |
| 6   | unknown | no                              |
| 7   | GND     | yes                             |
| 8   | 5V      | yes                             |
| 9   | 3V3     | no                              |
| 10  | 18V     | no                              |

Only four conductors are connected. Every other conductor is cut short and
covered with heat shrink.

Rx and Tx cross: the mainboard Rx is an input and takes the ESP32 TX, the
mainboard Tx is an output and drives the ESP32 RX. This follows the usual
convention for how the pins are named rather than a measurement. If no replies
arrive, swap the two — both are logic level signals and swapping them harms
nothing.

Confirm each conductor with a multimeter before connecting anything. GND reads
zero ohms to chassis, 18V reads battery voltage, 5V and 3V3 read their nominal
values, and Rx/Tx sit near zero at idle and move when the mower transmits.

## Wiring

```
Service port (10-way Molex KK)                      ESP32-DevKitC V4


POWER

  pos 8  5V ──[ PTC 0.75 A ]──┬──────────────────────►  5V
                              │
                             ═╧═  470 µF low ESR ∥ 100 nF
                              │
  pos 7  GND ─────────────────┴──────────────────────►  GND


LOCAL DECOUPLING, at the WROOM module and not fed from the port

                        ESP32  3V3 ──┬──
                                     │
                                    ═╧═  10 µF ∥ 100 nF
                                     │
                        ESP32  GND ──┴──


SERIAL, UART2, 9600 8N1

  pos 2  Tx ─────────────────────────────────────────►  GPIO16  RX

  pos 1  Rx ◄─────────────────────────────────────────  GPIO17  TX


NOT CONNECTED, cut short and heat shrunk

  pos 3-6  unknown
  pos 9    3V3
  pos 10   18V
```

## Why these parts

**5 V rather than 3.3 V.** The board's LDO needs headroom to regulate. The
mower's 3V3 rail is also weakly rated.

**PTC fuse on the 5V conductor.** A resettable fuse protecting the mower's
supply should anything short at the ESP end. 0.75 A sits above the board's
normal draw including WiFi peaks, but below what the conductor tolerates.

**470 µF + 100 nF on 5 V.** The electrolytic absorbs the transmit peaks when
the WiFi radio fires, so the supply does not dip. The ceramic handles the fast
high-frequency content the electrolytic is too slow for. The cable from the
mainboard is long enough to have inductance of its own, which makes the local
buffering necessary.

**10 µF + 100 nF on 3.3 V at the module.** Same principle close to the WROOM
module, where the current steps happen.

**18V conductor cut.** It is not needed and would destroy the board if
miswired.

**Ventilation holes.** The board sits in a closed box inside a machine that
stands in the sun.

**UART2, not UART0.** GPIO1/GPIO3 are shared with the USB serial chip. The
bootloader writes text on TX0 at every start, which would go straight into the
mower's diagnostic port.

**External u.FL antenna.** The mower is effectively a grounded metal and
plastic box that attenuates a board-mounted antenna.

## Configuration

See [`hqam-esphome.yaml`](hqam-esphome.yaml). The device's own yaml pulls the
component and the packages from this repository, so that single file is all
that is needed.

`logger:` must be declared explicitly. `components/confs/button.yaml` uses
`logger.log`, and without it validation fails with
`Couldn't find any component that can be used for 'logger::Logger'`.

`wifi.output_power: 12dB` is used in the working configuration. Reduced
transmit power lowers the current peaks, which matters on a fused supply.

`packages.files` is an explicit list. New files in this repository are not
fetched until they are added there.

Requires ESPHome 2024.6 or later for the `datetime` platform and the `ota:`
list syntax.

## Entities

| Type | Entities |
| --- | --- |
| Select | Mode select (MAN, AUTO, HOME, DEMO) |
| Switch | Bypass timer (`0x4A4E`), Week timer Monday .. Sunday (`0x4A50`), Register scan |
| Time | Timer 1/2 weekday start/stop, Timer 1/2 weekend start/stop |
| Button | Keypad keys (0-9, A-C, HOME, MAN/AUTO, Cancel, Up, Down, Yes), Stop, Reset Stop, Sync mower clock, Restart |
| Sensor | Battery used, voltage, current, temperature, capacities; charging and mowing time; total cutting time; blade motor speed; wheel speeds; loop signal quality; square mode percent; firmware version; raw diagnostics |
| Text sensor | Current mode, Status, Mower clock, Last received code, Scan status |
| Text | Scan range |

## Protocol and register map

Every frame is five bytes: `0F`, two address bytes, two value bytes. A read
sends the address with the value bytes zero; the reply carries the same
address and the value **low byte first** (`0F 4A 50 11 00` = 0x0011). A write
sets the address's high bit (`0x4A` -> `0xCA`, `0x01` -> `0x81`) and carries
the value **high byte first** (`0F CA 50 00 11`).

Timer registers, verified against the keypad of a 220AC:

| Register | Keypad (Swedish) | Value |
| --- | --- | --- |
| `0x4A4E` | Förbikoppla timer | 1 = box ticked (schedule ignored), 0 = schedule used |
| `0x4A50` | Veckotimer | day boxes, bit 0 = Monday .. bit 6 = Sunday |
| `0x4A38` / `0x4A3A` | Dagtimer, start/stop time 1 | one time per register: hour low byte, minute high byte |
| `0x4A40` / `0x4A42` | Dagtimer, start/stop time 2 | as above |
| `0x4A3C` / `0x4A3E` | Dagtimer Fr-Sö (Helgtimer), start/stop time 1 | as above |
| `0x4A44` / `0x4A46` | Dagtimer Fr-Sö (Helgtimer), start/stop time 2 | as above |

The odd addresses in between are not separate fields: a read there returns a
word straddling two times (the minute of one and the hour of the next), which
is why older register lists show "minute" registers that look swapped. Write a
time as one frame: `0F CA <reg> <minute> <hour>`.

The Dagtimer and Helgtimer boxes have no register of their own. The keypad
shows a box ticked while any of its four times is set and unticked when all
four are 00:00 (Dagtimer `0x4A38`, `0x4A3A`, `0x4A40`, `0x4A42`; Helgtimer
`0x4A3C`, `0x4A3E`, `0x4A44`, `0x4A46`). The box cannot be unticked on its
own: the keypad only clears it by resetting the times, and writing the times
back ticks it again. The Veckotimer box likewise follows `0x4A50`: it is
unticked when all seven days are selected (`0x7F`, mow every day) and ticked
for any narrower choice. Scans of `0x4A00`-`0x4AFF` showed no other change
for any of the three boxes, so none of them needs a switch of its own.

The clock lives at `0x36B1` (second), `0x36B3` (minute), `0x36B5` (hour),
`0x36B7` (day), `0x36B9` (month) and `0x36BD` (two-digit year). The mower does
not follow daylight saving time; the "Sync mower clock" button sets it from
Home Assistant.

### Register scanner

`components/confs/scan.yaml` adds a scanner for mapping unknown registers: set
"Scan range" (hex `START-END`, reads only), turn on "Register scan", then
change a setting on the keypad. The log shows each address's first value as
`SCAN` and any later change as `SCAN CHANGE`, and "Scan status" lists the
latest changes. It is off after every restart.

## Home Assistant

[`dashboard-automower.yaml`](dashboard-automower.yaml) is a control panel
replica, pasted via Dashboard > Edit > Raw configuration editor. Requires
`button-card`, `stack-in-card`, `card-mod` and `apexcharts-card` from HACS.
The schedule, measurements and diagnostics use the same panel and LCD style
as the display; tap a value to open its history or, for a timer time, to
change it.

The dashboard and the cards expect entity IDs of the form
`sensor.automower_status`. If the device sits in an area, Home Assistant may
prefix new entities with the area name; rename them to `*.automower_*` or
adjust the IDs in the YAML.

[`cards/`](cards) holds two matching info cards for other dashboards, one
compact for a phone and one larger for a tablet. Both show mode, status and
battery, and a tap opens the Automower dashboard. They need only
`button-card`.

## Status

Beta. Several registers are read but not calibrated, and are marked as
diagnostic. The status codes in `publishStatus()` are incomplete; unknown
codes appear as `STATUS_xxxx` with the raw hex value.

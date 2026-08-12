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
   ┌────────────────────────────────────────────┐
   │  ▯   ▯   ▯   ▯   ▯   ▯   ▯   ▯   ▯   ▯     │
   └──┬───┬───┬───┬───┬───┬───┬───┬───┬───┬─────┘
      1   2   3   4   5   6   7   8   9   10
      Rx  Tx  -   -   -   -   GND 5V  3V3 18V
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
 Mainboard service port                          ESP32-DevKitC V4
 (10-way Molex KK)
                        PTC 0.75 A
   8  5V  ────────────────[ ~~~ ]───────────┬───────────  5V
                                            │
                                     470 µF ┴ 100 nF
                                            │
   7  GND ──────────────────────────────────┴───────────  GND
                                                    │
                                            10 µF ──┴── 100 nF
                                            (across 3V3-GND at the module)

   2  Tx  ──────────────────────────────────────────────  GPIO16  (RX)

   1  Rx  ◄─────────────────────────────────────────────  GPIO17  (TX)

   9  3V3 ──  cut, heat shrink
  10  18V ──  cut, heat shrink
 3-6  n/c ──  cut, heat shrink
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

## Home Assistant

[`dashboard-automower.yaml`](dashboard-automower.yaml) is a control panel
replica, pasted via Dashboard > Edit > Raw configuration editor. Requires
`button-card`, `stack-in-card` and `card-mod` from HACS.

## Status

Beta. Several registers are read but not calibrated, and are marked as
diagnostic. The status codes in `publishStatus()` are incomplete; unknown
codes appear as `STATUS_xxxx` with the raw hex value.

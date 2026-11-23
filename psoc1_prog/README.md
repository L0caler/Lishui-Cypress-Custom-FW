# Arduino Mega / ESP32 PSoC 1 programmer 

based on https://github.com/zceemja/psoc1_prog
AI edit for use with Arduino Mega or ESP32 and CY8C24533

The same `src/` builds for both boards. The ISSP logic is shared in
`src/main.cpp`, and the pin-level code lives in `src/hal_avr.h` (Mega) and
`src/hal_esp32.h` (ESP32). See [ESP32](#esp32) below for that board's
wiring and power modes.

## Wiring (Arduino Mega)

| Arduino Mega  | Cypress chip        | Note |
| ------------- | ------------------- | ---- |
| GND           | Vss (pin 14)        | |
| D16 (PH1, TX2) | P1[1] (pin 13)     | ISSP SCLK, target RX in monitor mode |
| D17 (PH0, RX2) | P1[0] (pin 15)     | ISSP SDATA, target TX in monitor mode |
| D6 (PH3)       | XRES               | only on packages that have an XRES pin |
| D7 (PH4)       | Vdd switch         | see below |
| +5V           | Vdd (pin 28)        | only if you do **not** switch the supply |

Pin numbers in the right column are for the 28-pin SSOP CY8C24533.

SCLK, SDATA, XRES and the supply switch all live on PORTH, because the bit
banger toggles the clock and the data line with a single port write. PH0 and
PH1 are also the pins of the hardware USART2, so monitor mode bridges the
same two wires through a real UART. If you need a different pin-out, change
the `ISSP_*` block at the top of `src/main.cpp` and keep all four signals on
one port.

### Switching the target supply

The CY8C24533 has no XRES pin. The only way into programming mode is to power cycle the chip while
watching SDATA.

You can drive the target's Vdd straight off D7 and leave the default
polarity, which is enough for a bare chip on a breadboard, but note that ISSP
is specified for up to 25 mA while an AVR pin is only rated for 20 mA. Voltage drops to 4.4V in my test.

The cleanest switch is a high-side p-channel MOSFET (AO3401, IRLML6402, …):

With that circuit a low pin powers the target, so build with
`-D PWR_ACTIVE_LOW` (uncomment it under `[common]` in `platformio.ini`).



If you have no switch wired at all, programming still works on any part that
*does* have an XRES pin — just not on the CY8C24533.

## ESP32

Built as the `esp32dev` environment, for a classic ESP32 DevKit (WROOM or
WROVER). Every pin can be moved with a `-D` in `platformio.ini`.

| ESP32  | Cypress chip       | Note |
| ------ | ------------------ | ---- |
| GND    | Vss (pin 14)       | always, in both power modes |
| GPIO25 | P1[0] (pin 15)     | ISSP SDATA, target TX in monitor mode |
| GPIO26 | P1[1] (pin 13)     | ISSP SCLK, target RX in monitor mode |
| GPIO27 | XRES               | only on packages that have an XRES pin |
| GPIO15 | Vdd (pin 28)       | power mode `gpio` only |
| GPIO34 | Vdd via a divider  | power mode `external` only, see below |
| GPIO2  | -                  | on-board LED, lit while ISSP is busy |

**The ESP32 is a 3.3 V part and its pins are not 5 V tolerant.** Run the
target at 3.3 V. The firmware uses the 3 V INITIALIZE-3 vectors by default
on this board; `--vdd 5` switches to the 5 V set. At 5 V, SDATA needs a
bidirectional level shifter (a BSS138 module, for example), because the
target drives that line. SCLK and XRES only go to the target, and the 3.3 V
high is above a 5 V PSoC's 2.1 V input threshold, so they can be wired
directly.

### Power modes

The CY8C24533 has no XRES pin, so the only way into programming mode is a
power-up. The ESP32 can handle that two ways. Pick one with `--power` on
any `programmer.py` command. The firmware remembers it until the ESP32
resets, and `-D POWER_EXTERNAL` in `platformio.ini` makes `external` the
boot default.

**`--power gpio`** (default): GPIO15 *is* the target's 3.3 V supply. The
ESP32 switches it off and on by itself, the same as the Mega does with D7.
A pin sources about 40 mA at most (the firmware sets the strongest drive),
and the voltage sags under load. That is enough for a bare chip on a
breadboard, but not for a full controller board with its own regulator,
op-amps and so on. You can wire a second pin in parallel with
`-D ISSP_PWR2_GPIO=22`. Keep the decoupling on the target's Vdd small
(about 0.1 µF) so the pin can charge it quickly. For more current, put a
p-channel MOSFET between the ESP32's 3V3 rail and the target, driven from
GPIO15, and build with `-D PWR_ACTIVE_LOW`. Give the gate a pull-up to
3V3, so the MOSFET stays off while the pin floats.

**`--power external`**: you switch the target's supply (bench supply,
battery, the board's own regulator) and the ESP32 watches it on GPIO34. When
`programmer.py` acquires the target, it tells you what to do:

```
PSoC1_Prog: switch the target supply OFF
PSoC1_Prog: now switch the target supply ON
PSoC1_Prog: target CY8C24533 (id 0x00b0), acquired by power cycle, 8192 bytes of flash
```

The firmware waits up to 30 s for each step. The supply has to read off
for 0.3 s before power-on is accepted, so that the chip really resets.
After that, power-on counts as soon as GPIO34 reads high *or* the target
raises SDATA. While it waits, SDATA has a pull-down on it, so an unpowered
line cannot be mistaken for the chip's power-on signal. Catching SDATA
directly means a slow sense divider cannot make the firmware miss the
short window the chip opens at power-up.

The firmware cannot restart an externally powered target itself.
`reset`, `--reset` and `monitor --reset` just ask you to power cycle it.
In this mode the GPIO15 output is switched off (left floating), so it never
drives against your supply.

The sense input needs a resistor to GND, so it reads low when the supply
is off. GPIO34 is input-only and has no internal pull resistors.

| Target supply | Wiring to GPIO34 |
| ------------- | ---------------- |
| 3.3 V | Vdd → 1 kΩ → GPIO34, plus 100 kΩ from GPIO34 to GND |
| 5 V   | Vdd → 10 kΩ → GPIO34, plus 20 kΩ from GPIO34 to GND (reads 3.3 V) |

### ESP32 notes

- Monitor mode uses UART1, routed to GPIO25 (RX) and GPIO26 (TX), so it
  uses the same two wires as on the Mega.
- The ISSP clock runs at about 500 kHz, set by `ISSP_HALF_NS`. That is
  about the Mega's speed, and well under the 8 MHz ISSP limit.
- SDATA, SCLK and XRES must stay on GPIO0–31. They are switched through
  the GPIO set/clear registers, and those registers only cover pins 0–31.

## Build and upload

```bash
platformio run -e megaatmega2560 --target upload     # Arduino Mega (COM4)
platformio run -e esp32dev --target upload           # ESP32 (port auto-detected)
```

`platformio run` with no `-e` still builds only the Mega.

## Programmer

```
python programmer.py --help
```

Flash a .hex built by PSoC Designer:

```bash
python programmer.py COM4 flash -i ../LishuiTest1/LishuiTest1/output/LishuiTest1.hex
python programmer.py COM4 monitor --reset
```

Options added for this port:

| Option | Meaning |
| ------ | ------- |
| `--init-mode {auto,reset,power}` | how to enter programming mode. `auto` (default) pulses XRES and falls back to a power cycle if that finds no known device; `power` is the one the CY8C24533 needs |
| `--blocks N` | override the target's flash size, in 64-byte blocks, when the chip is not in the device table |
| `--vdd 3` / `--vdd 5` | pick the INITIALIZE-3 vector set. Default 5 V on the Mega, 3 V on the ESP32 |
| `--power {gpio,external}` | where the target supply comes from, see [Power modes](#power-modes). `external` needs the ESP32 |
| `--ignore-checksum` | report a post-flash checksum mismatch but do not fail |
| `--count N` | now defaults to the target's real flash size instead of 64 blocks |

`flash` and `read` now use the flash size the firmware reports for the
attached part, so an 8 KB device is fully written, verified and dumped.
Flashing an image larger than the target's flash is refused up front.

## Supported devices

The device table lives in `src/devices.h` and carries the silicon ID, the
flash size in 64-byte blocks, and any vector quirks. Silicon IDs and flash
sizes were taken from the Cypress PSoC Programmer device database (the
`Devices` directory of a PSoC Programmer install), so adding a part is a
matter of copying one row out of the matching `.dat` file.

| Family | Flash | Parts |
| ------ | ----- | ----- |
| CY8C21x23  |  4 KB | CY8C21123, CY8C21223, CY8C21323 |
| CY8C24x23A |  4 KB | CY8C24123A, CY8C24223A, CY8C24423A |
| CY8C21x12  |  8 KB | CY8C21312, CY8C21512 |
| CY8C21x34  |  8 KB | CY8C21234, CY8C21334, CY8C21434, CY8C21534, CY8C21634 |
| CY8C23x33  |  8 KB | CY8C23433, CY8C23533, CY8C23033 |
| **CY8C24x33** | **8 KB** | **CY8C24533, CY8C24633, CY8C24033** |
| CY8CTMG110 |  8 KB | CY8CTMG110-32LTXI, CY8CTMG110-00PVXI |
| CY8CTST110 |  8 KB | CY8CTST110-32LTXI, CY8CTST110-00PVXI |
| CY8C27x43  | 16 KB | CY8C27143, CY8C27243, CY8C27443, CY8C27543, CY8C27643 |

## Checksum

The SROM checksum function sums `BLOCKID` flash blocks, which is why the ISSP
specification prints four different CHECKSUM-SETUP vectors. They differ in
exactly one byte, holding the part's flash size in blocks (256 encoded as 0).
Rather than carry one vector per family, the firmware patches that byte from
the device table, so the checksum now covers the whole of an 8 KB or 16 KB
part instead of only its first 4 KB.

## Serial protocol

One byte per command, every reply ends with `\r\n> `. Added since the
original firmware (`k` and `M` in fw 0x0004, the rest in fw 0x0002):

| Command | Meaning |
| ------- | ------- |
| `I` + mode | initialise with mode 0 auto, 1 XRES, 2 power cycle |
| `m` | which mode last acquired the target, in hex |
| `n` / `N` | flash size in 64-byte blocks, ASCII / binary |
| `B` + 2 bytes | override the flash size, low byte first |
| `v` + byte | supply voltage, 0 = 3 V vectors, 1 = 5 V vectors |
| `W` + 2 bytes | Tvddwait in microseconds, low byte first |
| `E` | read and clear the count of WAIT-AND-POLL timeouts, in hex |
| `k` + digits | monitor mode line rate, in ASCII, e.g. `k19200` |
| `M` | enter monitor mode, leave it with Ctrl-] (0x1D) |
| `P` + byte | power source, 0 = GPIO, 1 = external with sense pin (fw 0x0006; ignored if the board has no sense pin) |
| `p` | current power source, in hex |
| `H` | board name, `mega2560` or `esp32` |
| `T` | probe (fw 0x0007): power cycle, send no vectors, reply with SDATA's run lengths, see [Probe](#probe) |

While `I`/`i` waits for an external supply, the reply may begin with
status lines such as `#supply-off` and `#supply-on`. Then the usual
`\r\n> ` follows.

`a` (reset) now power cycles the target when programming mode was entered
that way, and releases SDATA/SCLK afterwards so the application gets P1[0]
and P1[1] back.

## Monitor mode

Once ISSP is done with them, P1[0] and P1[1] are ordinary pins again, so
application firmware can use them as a serial port. Monitor mode bridges
them to the Arduino's USB port, which makes the programming header a debug
console as well.

**The target's TX is SDATA, P1[0]. The target's RX is SCLK, P1[1].**

That is the same division of labour ISSP already uses: SDATA is the only
line the target ever drives, and SCLK is always driven by the programmer.
Keeping it means no line changes who drives it between the two modes, so
pointing the monitor at a target that is still in programming mode cannot
cause contention.

In a plain terminal (`platformio device monitor`, PuTTY, …) at 115200 baud:

```
k19200        set the line rate, ASCII digits then enter; echoes the rate
M             bridge the pins to this terminal, echoes the rate first
Ctrl-]        leave monitor mode and return to the prompt
```

`k` is optional; the rate defaults to `MON_BAUD` in `src/main.cpp`, 19200.
It is not stored across resets. Any rate from 300 to 1000000 is accepted
and the reply echoes it.

The bridge is the Mega's hardware **USART2** on PH0 (RXD2, target TX) and
PH1 (TXD2, target RX): full duplex, interrupt driven and buffered, with the
usual USART baud accuracy. USART2 is switched on only for the duration of
`M` and off again on Ctrl-], which hands the pins back to the ISSP bit
banger. RXD2 has its pull-up on while the monitor runs.

### From programmer.py

`programmer.py` has the terminal built in, so no separate terminal program is
needed:

```
python programmer.py COM4 monitor                 # attach to the running target
python programmer.py COM4 monitor --reset         # restart the target, then attach
python programmer.py COM4 flash -i app.hex --reset --monitor   # flash, restart, attach
```

It sets the line rate (`--baud`, default 19200), enters monitor mode, and
sends every key as you press it. Sent keys are echoed as `[tx b'e']` so you
can see they went out (`--no-echo` turns that off), and `--hex` shows
received bytes as hex, which helps with a baud mismatch. Ctrl-] or Ctrl-C
leaves monitor mode and returns the programmer to its prompt.

`monitor` does not put the target into programming mode, because that would
stop its application. `--reset` acquires and then power cycles it, so its
start-up output is caught from the beginning.

The host port runs at 115200 (`Serial.begin()` in `src/main.cpp` and
`baudrate` in `programmer.py`, which must match). Keep the monitor rate at
or below that, or the host side cannot drain what the target sends.

### Testing it

Jumper D16 to D17 with no target attached and run `monitor`: every key you
type comes straight back, because USART2 hears its own output.

## Flash security

A chip whose blocks are protected returns zeros for external reads, so a
`read` of a locked device looks blank even though the programmer is working
correctly. Compare `checksum` against the dump to tell the two apart: the
SROM computes the checksum over the real contents whatever the protection,
so a non-zero checksum beside an all-zero dump means protected, not empty.

The Lishui project protects every block (`W` in `flashsecurity.txt`), so
`flash --read` always reads zeros. `programmer.py` reports that, and relies
on the matching checksum instead of failing.

## Probe

If `device` reports a wrong ID such as `0xff00`, the programmer has usually
sent its vectors at the wrong moment. The target then booted its
application instead of entering programming mode. `probe` power cycles the
target (power mode `gpio`) without sending anything. It records SDATA every
10 µs for the first 30 ms:

```
python programmer.py COM5 probe
PSoC1_Prog:    0.00 ms  low  for 1.68 ms
PSoC1_Prog:    1.68 ms  HIGH for 0.02 ms     <- line crossing the input threshold
PSoC1_Prog:    1.70 ms  low  for 0.01 ms
PSoC1_Prog:    1.71 ms  HIGH for 0.26 ms
PSoC1_Prog:    1.97 ms  low  for 0.01 ms
PSoC1_Prog:    1.98 ms  HIGH for 5.66 ms     <- the power-up pulse
PSoC1_Prog:    7.64 ms  low  for 8.02 ms     <- its fall opens the programming window
PSoC1_Prog:   15.66 ms  HIGH for 14.34 ms    <- the application's UART, idle
```

That trace is the Lishui board powered from a 3.3 V ESP32 pin. The supply
rises slowly, and the spikes at the start come from SDATA hovering at the
ESP32's input threshold. The firmware only accepts a high that lasts 1 ms
(`SDATA_HIGH_MIN_US`). Until fw 0x0007 it took the first spike for the
pulse, so it sent the vectors 6 ms early and read `0xff00`.
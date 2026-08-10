# Loss of control, ELRS 3.6.0 + RadioMaster ER8GV, 2026-08-08

## The incident

### What happened

Two flights of an F5J glider, same session, same airframe. TX and RX both
ExpressLRS 3.6.0, 2.4 GHz, 333 Hz Full Res, switch mode `16ch Rate/2`,
telemetry ratio `1:2`. Receiver is a RadioMaster ER8GV driving eight servos
directly over PWM, no flight controller. In each flight there was an episode of
roughly 30–60 s where the aircraft did not respond to control input, then
recovered on its own. No failsafe, no "telemetry lost" callout, no power cycle.
Flight 1 (`f5j-new-2026-08-08-124224.csv`): peak 305.3 m at 12:50:12.690,
camber deployed at 12:50:26.690, then ~30 s of full-deflection stick with no
coherent response, recovery around 12:51:05, normal landing at 12:52:50.
Flight 2 (`f5j-new-2026-08-08-132714.csv`): onset somewhere in
13:32:47–13:33:26 at 60–30 m, and the motor was needed twice from 6–7 m
(13:34:12.7 and 13:34:23.7–13:34:28.2) to save it.

### What the logs rule out

The RF link was not the problem. `RQly` never fell below 92 during the flight-1
episode — worse values (87) occur earlier in the same flight while everything
was fine — and during the flight-2 episode the aircraft was at 20–40 m with
`1RSS` of −66 to −76 dBm, SNR ≥ 9, transmitter at **10 mW**, `RQly` 97–100. The
receiver never rebooted: its barometric vario altitude is continuous throughout
both logs with no zero-reset. Flap deployment is not the trigger — in flight 2
the flaps sat at position 0 from launch until 13:33:32.700, *after* the onset,
and were moved only as a reaction. One blind spot remains: `RxBt`
(12.5 → 11.8 V) is the flight pack read through EXT-V, not the receiver/servo
rail, so the BEC output was never observed on either flight.

## Analysis

### Leading hypothesis

The receiver continued to receive packets and send telemetry, but stopped
applying received channel data to its PWM outputs. This is the only failure
mode consistent with all of it: perfect link statistics, live telemetry, an
aircraft that flies a smooth stable trajectory while ignoring the pilot, and
spontaneous recovery. There is direct precedent in ExpressLRS on a PWM
receiver — issue [#2548][2548], *"Once the receiver lost control, it will hold
the last state for about 10 seconds and then regain control"*, with no failsafe
entered. It also explains why only this model is affected: it is the only one in
the fleet running `16ch Rate/2` with a `1:2` telemetry ratio, the
least-travelled corner of the configuration space, and the corner where the
tracker clusters ([#3157][3157], [#3617][3617], [#3631][3631] — the last being
an ER8GV glider on 3.6.3).

### The SX1280 FIFO bug as a specific candidate

Until 4.0.1, ExpressLRS used base address `0x00` for both the transmit and the
receive buffer inside the SX1280's shared 256-byte data buffer, and read the RX
buffer without first confirming the chip was in RX-with-data state. PR
[#3623][3623] separated them (`SX1280_TX_BUFFER_BASE = 0x00`,
`SX1280_RX_BUFFER_BASE = 0x80`) and added a status check; the 4.0.1 notes credit
it with fixing "RC packets with bad 881 values" and "some random failsafes and
inability to reconnect". The ER8GV is affected hardware — `ExpressLRS/targets`
resolves `er8gv` to `platform: esp32` / `Unified_ESP32_2400_RX`, which
`src/targets/common.ini:34` builds with `-DRADIO_SX128X=1` — and a `1:2`
telemetry ratio maximises how often the receiver transmits between receives,
which is precisely the exposure. **This is not backported:** at tag 3.6.3,
`src/lib/SX1280Driver/SX1280.cpp:502` still reads `hal.WriteBuffer(0x00, data,
PayloadLength, radioNumber); //todo fix offset to equal fifo addr`. The
objection to it as *the* explanation: buffer corruption should fail CRC and cost
link quality, and none was observed; and the 4.0.1 fix is tied to reset and
reconnect scenarios, neither of which occurred here.

## Status and actions

Unproven, and these logs cannot prove it — they record what the transmitter sent
and what the radio link did, and contain nothing about what the servos actually
moved. The alternative that cannot be excluded is an intermittent in the wing
servo harness, which would be equally silent. Actions taken/planned, in order of
value: move both ends to **4.0.1 or later** (not backward compatible, flash
together); switch to **12ch Mixed** instead of 16ch Rate/2 — a drop-in for this
channel map that doubles the CH1–4 update rate from 83 Hz to 166 Hz — and drop
telemetry to **1:8**; fit a small camera looking down the wing so the next
occurrence is directly observable; set the receiver failsafe to a distinctive
position (full crow) so a failsafe stops being indistinguishable from frozen
outputs; and move the ER8GV's voltage telemetry to the built-in receiver voltage
so the BEC rail is finally visible.

## Bench reproduction with a logic analyzer

The flight logs record what the transmitter sent and what the radio link did;
the hypothesis is about what the receiver *did with* the packets it received,
which no log in this repository observes. An AZ-Delivery FX2LP clone (Cypress
CY7C68013A, `0925:3881`, the same chip as a Saleae Logic) closes that gap by
watching the receiver's own pins. Captured with `sigrok-cli` at 8 MHz — 19
samples per bit at CRSF's 420000 baud, and 125 ns resolution on a servo pulse.
`data.sr` in this repository is a 1 MHz capture kept as a counter-example: at
2.38 samples per bit only 2.6% of its frames survive CRC, and the decode looks
plausible while being wrong. `data2.sr` is the 8 MHz equivalent, 100% CRC-valid,
and is what the decoder is validated against.

### A Lua script to make the input predictable

`crsf_sweep.lua` runs on the handset and drives every channel through a fixed
7.0 s cycle: a slam marker (0.2 s low, 0.2 s high, 0.2 s low), a 3 s ramp to the
top, a 0.2 s dwell, a 3 s ramp back down, and a 0.2 s dwell. The cycle is
continuous — it ends at the same value it starts at — so the only discontinuities
in the whole waveform are the three deliberate marker edges. Those edges do two
jobs: they align expected against actual, which matters because no clock is
shared between the handset and the capturing PC, and their sharpness gives an
end-to-end latency measurement that a ramp alone cannot. The ramps do the rest:
against a straight line, a frozen output, a repeated value or a bogus one is
self-evident without any reference recording.

Copy the file to **`/SCRIPTS/TOOLS/crsf_sweep.lua`** on the radio's SD card. Any
`.lua` in that directory is picked up as a tool and appears under *System menu →
Tools*, listed as "CRSF Sweep" — the name comes from the `TNS|...|TNE` marker
near the top of the script, not from the filename. It runs only while the
tool is open, which also keeps the screen awake for the length of a soak run;
EXIT stops it and leaves the output at −100%.

The script itself writes one number: it sets `GV1` between −100 and +100. It
cannot write channel values directly, because EdgeTX gives Lua no such call, and
a GVAR is not a valid mix *source* — GVARs are only usable as mix *parameters*
such as weight, offset or curve. The way round that is the `MAX`/`GV1` idiom.
`MAX` is a constant source sitting permanently at +100%, and a mix multiplies
its source by its weight, so a mix of `MAX` with weight `GV1` outputs exactly
`GV1` percent. Setting `GV1 = 42` in Lua therefore parks that channel at +42%,
or 1715 µs. That indirection is what gives the script write access to a channel
at all. One GVAR is enough for the whole sweep because every moving channel
carries the same value at the same instant, so all fifteen of them can read the
same weight.

Build this on a separate test model, not the glider's:

| Channel | Source | Weight | Result |
|---|---|---|---|
| CH1–4, CH6–16 | `MAX` | `GV1` | follows the sweep, −100% to +100% |
| CH5 | `MAX` | `+100%` fixed | ARM held high for the whole run |

The receiver sits on the bench with nothing connected but the logic analyzer, so
a throttle channel ramping to full against a held ARM drives nothing.

Two numbers matter when reading the logs. Full deflection is ±100%, which is
988–2012 µs and 172–1811 in CRSF ticks, *not* 1000–2000; and `GV1` moves in
integer percent, so each ramp is a 200-step staircase of about 5.1 µs, or 8.2
ticks, per step. A healthy capture looks like that staircase, and the default
`--max-jump` of 400 ticks sits far above it while still well below a slam.
Note too that `16ch Rate/2` halves the per-channel update rate, and that ELRS
sends channels 5–16 through the switch encoding at a much lower rate than 1–4 —
both are normal, and both look like faults if the expected pattern does not
account for them.

### What is probed decides what is proven

CRSF tells us what was commanded and what the receiver decoded; the PWM outputs
tell us what the servos were actually told to do. Capturing both at once is what
separates the candidate explanations, and is the only arrangement that can
distinguish the leading hypothesis from the servo-harness intermittent that the
logs equally cannot exclude:

| CRSF | PWM out | Reading |
|---|---|---|
| moving | moving | receiver fine — look at the harness or the servos |
| moving | frozen / bogus | **the hypothesis**: outputs stop tracking received data |
| frozen | — | failure upstream of the output stage, in RF or decode |
| absent | — | link or power, which the flight telemetry already argues against |

### Tooling

`crsf_capture.py` streams the analyzer and decodes CRSF live, writing `rc.csv`
(one row per frame, 16 channels in ticks) and `events.jsonl` (anomalies), while
showing all 16 channels updating in place on one status line. It sustains
8.02 MB/s using about 13% of one core, so capture is gapless — which matters
here, because a dropout is itself a plausible symptom and a gapped capture could
not tell one from the other. `crsf_analyze.py` summarises a run and prints, for
each anomaly window, the `crsf_slice.py` command that cuts those seconds out of
`raw.bin` into a `.sr` for PulseView. The decoder is verified byte-identical to
`sigrok-cli`'s `uart` decoder on `data2.sr`. **A PWM decoder does not exist
yet** and is the remaining piece: without it the rig captures the discriminating
signal but cannot yet read it.

### Practical notes

The decoded logs cost about 0.9 MB per minute, so a soak run of any length is
cheap — which matters, because the episodes lasted 30–60 s in flight and are
intermittent. Raw samples are not written unless `--raw` is given; they cost
0.48 GB per minute and exist only to feed `crsf_slice.py`. The sensible order is
to soak without raw until the fault reproduces at all, then repeat with `--raw`
(and `--max-gb`, which caps an unattended run) to get the waveform evidence for
the window that misbehaved.

## Acknowledgements

[`sigrok_crsf_decoder`][crsf-pd] by James Cordell is what makes a captured
window readable in PulseView: a CRSF decoder stacked on sigrok's `uart` that
annotates sync, length, type and CRC and unpacks the channel values on screen.
Install its `crsf/` directory into `/usr/share/libsigrokdecode/decoders/`, or
`~/.local/share/libsigrokdecode/decoders/` for a single user. The Python
decoder in this repository is a separate implementation, written for streaming
capture rather than display; the two were run against each other on `data2.sr`
and agree channel for channel.

[crsf-pd]: https://github.com/JamesCordell/sigrok_crsf_decoder/
[2548]: https://github.com/ExpressLRS/ExpressLRS/issues/2548
[3157]: https://github.com/ExpressLRS/ExpressLRS/issues/3157
[3617]: https://github.com/ExpressLRS/ExpressLRS/issues/3617
[3631]: https://github.com/ExpressLRS/ExpressLRS/issues/3631
[3623]: https://github.com/ExpressLRS/ExpressLRS/pull/3623

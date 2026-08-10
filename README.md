# Loss of control, ELRS 3.6.0 + RadioMaster ER8GV, 2026-08-08

## The incident

### What happened

Two flights of an F5J glider, same session, same airframe. ELRS 3.6.0 both ends,
2.4 GHz, 333 Hz Full Res, switch mode `16ch Rate/2`, telemetry ratio `1:2`. The
receiver is a RadioMaster ER8GV driving eight servos directly over PWM, no
flight controller. Each flight had an episode of roughly 30–60 s where the
aircraft ignored control input and then recovered on its own — no failsafe, no
"telemetry lost" callout, no power cycle.

Flight 1 (`f5j-new-2026-08-08-124224.csv`): peak 305.3 m at 12:50:12.690, camber
deployed at 12:50:26.690, ~30 s of full-deflection stick with no coherent
response, recovery around 12:51:05, normal landing at 12:52:50. Flight 2
(`f5j-new-2026-08-08-132714.csv`): onset somewhere in 13:32:47–13:33:26 at
60–30 m; the motor was needed twice from 6–7 m to save it.

### What the logs rule out

- **The RF link.** `RQly` never fell below 92 in the flight-1 episode, and worse
  (87) occurs earlier in the same flight while everything was fine. Flight-2
  episode: 20–40 m, `1RSS` −66 to −76 dBm, SNR ≥ 9, 10 mW, `RQly` 97–100.
- **A receiver reboot.** Barometric vario altitude is continuous through both
  logs with no zero-reset.
- **Flap deployment as trigger.** In flight 2 the flaps sat at position 0 until
  13:33:32.700, *after* the onset, and moved only as a reaction.

Blind spot: `RxBt` (12.5 → 11.8 V) is the flight pack read through EXT-V, not
the receiver/servo rail, so the BEC output was never observed.

## Analysis

### Leading hypothesis

The receiver kept receiving packets and sending telemetry but stopped applying
channel data to its PWM outputs. That is the only failure mode consistent with
perfect link statistics, live telemetry, a smooth stable trajectory while
ignoring the pilot, and spontaneous recovery. Direct precedent on a PWM receiver
exists in [#2548][2548] — *"the receiver ... will hold the last state for about
10 seconds and then regain control"*, no failsafe entered. It also explains why
only this model is affected: it alone runs `16ch Rate/2` with `1:2` telemetry,
the least-travelled corner of the configuration space and where the tracker
clusters ([#3157][3157], [#3617][3617], [#3631][3631] — the last an ER8GV glider
on 3.6.3).

### The SX1280 FIFO bug as a specific candidate

Until 4.0.1, ExpressLRS used base address `0x00` for both the transmit and the
receive buffer in the SX1280's shared 256-byte buffer, and read the RX buffer
without confirming the chip was in RX-with-data state. PR [#3623][3623]
separated them (`TX 0x00` / `RX 0x80`) and added a status check; the 4.0.1 notes
credit it with fixing "RC packets with bad 881 values" and "some random
failsafes and inability to reconnect". The ER8GV is affected hardware — `er8gv`
resolves to `Unified_ESP32_2400_RX`, built with `-DRADIO_SX128X=1`
(`src/targets/common.ini:34`) — and `1:2` telemetry maximises how often the
receiver transmits between receives, which is the exposure. **Not backported:**
at tag 3.6.3, `src/lib/SX1280Driver/SX1280.cpp:502` still reads
`hal.WriteBuffer(0x00, ...); //todo fix offset to equal fifo addr`.

Against it: corruption should fail CRC and cost link quality, and none was seen;
and the 4.0.1 fix is tied to reset and reconnect, neither of which occurred.

## Status and actions

Unproven, and these logs cannot prove it — they record what the transmitter sent
and what the link did, nothing about what the servos moved. A wing servo harness
intermittent would be equally silent and cannot be excluded. In order of value:

- **4.0.1 or later** at both ends (not backward compatible, flash together)
- **12ch Mixed** instead of `16ch Rate/2` — a drop-in for this channel map that
  doubles the CH1–4 rate from 83 to 166 Hz — and telemetry down to **1:8**
- a camera looking down the wing
- failsafe set to a distinctive position (full crow), so it stops being
  indistinguishable from frozen outputs
- voltage telemetry moved to the built-in receiver voltage, exposing the BEC rail

## Bench reproduction with a logic analyzer

The hypothesis concerns what the receiver *did with* the packets it received,
which no log here observes. An AZ-Delivery FX2LP clone (`0925:3881`, same chip
as a Saleae Logic) watches the receiver's pins at 8 MHz — 19 samples per bit at
CRSF's 420000 baud, 125 ns on a servo pulse. `data2.sr` is the reference
capture: 100% CRC-valid, and what the decoder is validated against.

### A Lua script to make the input predictable

`crsf_sweep.lua` goes in **`/SCRIPTS/TOOLS/`** on the SD card and appears under
*System → Tools*. It drives a 7.0 s cycle: a slam marker (0.2 s low, high, low),
a 3 s ramp up, 0.2 s dwell, a 3 s ramp down, 0.2 s dwell. The cycle wraps
continuously, so those three marker edges are the *only* discontinuities in the
waveform — anything else is the receiver — and they give an end-to-end latency
measurement a ramp cannot.

The script only sets `GV1` to between −100 and +100. EdgeTX has no Lua call to
write a channel, and a GVAR cannot be a mix **source**, only a parameter. Hence
the `MAX` idiom: `MAX` is a constant +100% source and a mix multiplies source by
weight, so `MAX` weighted by `GV1` outputs exactly `GV1` percent. In the Weight
field, long-press ENTER or scroll past ±100 to reach `GV1`. Build it on a
separate test model:

| Channel | Source | Weight | Result |
|---|---|---|---|
| CH1–4, CH6–16 | `MAX` | `GV1` | follows the sweep |
| CH5 | `MAX` | `+100%` | ARM held high throughout |

Full deflection is ±100% = 988–2012 µs = 172–1811 ticks, *not* 1000–2000. The
tool script runs at the radio's UI rate, about 20 Hz, so each ramp arrives as a
~60-step staircase of ~27 ticks (17 µs), not a smooth line. That staircase is
what healthy looks like, and the lag it puts between channels — up to one step —
sets the divergence threshold. `16ch Rate/2` halves the
per-channel rate, and channels 5–16 ride the switch encoding far slower than
1–4 — both normal, both fault-shaped if unaccounted for.

### What is probed decides what is proven

CRSF shows what was commanded and decoded; PWM shows what the servos were told.
Only capturing both separates the candidates:

| CRSF | PWM out | Reading |
|---|---|---|
| moving | moving | receiver fine — look at the harness or the servos |
| moving | frozen / bogus | **the hypothesis**: outputs stop tracking received data |
| frozen | — | failure upstream of the output stage, in RF or decode |
| absent | — | link or power, which the telemetry already argues against |

### Tooling

`crsf_capture.py` streams the analyzer and decodes CRSF live to `rc.csv` and
`events.jsonl`, showing all 16 channels updating in place on one status line. It
sustains 8.02 MB/s on ~13% of one core, so capture is gapless — which matters,
because a dropout is itself a plausible symptom and a gapped capture could not
tell one from the other. `crsf_analyze.py` summarises a run and prints, per
anomaly window, the `crsf_slice.py` command that cuts those seconds into a `.sr`
for PulseView. It flags two things, both computed from `rc.csv` so thresholds
can be retuned without recapturing: a channel **stuck** for ≥1 s while the
others keep moving, and a channel **diverged** from the cross-channel consensus.
Because the sweep moves every channel in lockstep, the marker slams cancel out
of the consensus and only a channel going its own way is reported. The decoder is byte-identical to `sigrok-cli`'s `uart` decoder on
`data2.sr`. **A PWM decoder does not exist yet** — without it the rig captures
the discriminating signal but cannot read it.

Logs cost ~0.9 MB/min, so soak runs are cheap. Raw samples need `--raw`, cost
0.48 GB/min, and exist only to feed `crsf_slice.py`: soak without raw until the
fault reproduces, then repeat with `--raw` (and `--max-gb`) for the waveform.

## Acknowledgements

[`sigrok_crsf_decoder`][crsf-pd] by James Cordell is what makes a captured window
readable in PulseView, annotating CRSF on top of sigrok's `uart`. Install its
`crsf/` directory into `~/.local/share/libsigrokdecode/decoders/`. The Python
decoder here is independent; the two agree channel for channel on `data2.sr`.

[crsf-pd]: https://github.com/JamesCordell/sigrok_crsf_decoder/
[2548]: https://github.com/ExpressLRS/ExpressLRS/issues/2548
[3157]: https://github.com/ExpressLRS/ExpressLRS/issues/3157
[3617]: https://github.com/ExpressLRS/ExpressLRS/issues/3617
[3631]: https://github.com/ExpressLRS/ExpressLRS/issues/3631
[3623]: https://github.com/ExpressLRS/ExpressLRS/pull/3623

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

### What `ANT` adds

The ER8GV is a diversity receiver: two antennas mounted on opposite sides of the
fuselage, and the `ANT` column records which one the receiver had selected. As
the airframe yaws, the antenna facing the transmitter changes, so `ANT` is a
coarse **receiver-side witness of aircraft heading** — the only observable in the
log that reports what the airframe actually did, as opposed to what the pilot
asked for. Everything else in the log is either the pilot's side of the link or a
scalar the receiver reports about itself.

Both logs are in `flights.db` (built by `logs_to_sqlite.py`); `ant_analyze.py`
produces everything below.

**The witness is real but noisy.** `ANT=0` tracks `1RSS > 2RSS` (−77.1/−88.3 dB
mean in flight 1) and `ANT=1` tracks the reverse, so the field is a genuine
diversity selector, not noise. Against commanded control effort — mean
`max(|Rud|,|Ail|)` over 10 s windows — switch rate correlates at r = +0.395
(flight 1) and +0.401 (flight 2), and the top effort quartile has *no*
zero-switch window in either flight. So hard turning always moved the antenna,
but the converse is weak: a still antenna is only weakly evidence of still air.

**Caveat that matters for the rest of the log:** only the *selected* antenna's
RSSI is refreshed. `1RSS` changes on 86 % of samples while `ANT=0` but only 45 %
while `ANT=1`, and symmetrically for `2RSS`. The unselected value is stale, so
`1RSS − 2RSS` is not a usable heading proxy, and any claim about link margin
during a long `ANT` freeze covers one antenna only — during flight 2's episode
antenna 2 went unsampled for 48 s.

**What the episodes look like.** In each flight the longest antenna dwell of that
flight falls in or beside the episode, while the sticks were being worked hard:

| | flight 1 | flight 2 |
|---|---|---|
| longest commanded-but-mute run | 21.5 s @ 12:50:37 | 22.5 s @ 13:33:37 |
| peak effort in it | 0.78 | 0.54 |
| min `RQly` in it | 92 | 98 |
| such runs elsewhere in flight | none | none |

In flight 2 the freeze is 48.5 s (13:33:01.7–13:33:49.7) against a 25.0 s
runner-up, and it spans the whole 50 m → 11 m descent. In flight 1 the episode
holds a 22.0 s freeze from 12:50:27.190 — one sample after camber deployment —
through which aileron swings between −725 and +1024 with no antenna response at
all.

**But it does not reach significance.** Rotating the `ANT` switch train
circularly against the stick series — which destroys the alignment while leaving
each series' own autocorrelation intact — a commanded-but-mute run this long
overlapping the episode arises by chance with p = 0.15 (flight 1) and p = 0.30
(flight 2); Fisher-combined, p = 0.19. The reason is the base rate: gliders fly
straight a lot, long antenna dwells are ordinary, and the episodes are a large
slice of each flight. The "0 of 322 windows elsewhere" figure in the per-quartile
table is not 322 independent samples — 10 s windows stepped 0.5 s apart are
roughly sixteen independent observations, and the rotation test is what accounts
for that.

So `ANT` is **consistent with** frozen PWM outputs and adds a genuinely
independent line of evidence — one that a stuck-servo or airframe explanation
would not produce, since those leave the receiver's antenna selection free to
follow a still-manoeuvring airframe. It does not on its own confirm the
hypothesis, and it does not refute it.

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

## Bench reproduction with a logic analyzer

The hypothesis concerns what the receiver *did with* the packets it received,
which no log here observes. An AZ-Delivery FX2LP clone (`0925:3881`, same chip
as a Saleae Logic) watches the receiver's pins at 8 MHz — 19 samples per bit at
CRSF's 420000 baud, 125 ns on a servo pulse.

### A Lua script to drive a sweep on the radio

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

### Tooling for Capture and Analysis

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
of the consensus and only a channel going its own way is reported. The decoder
was checked byte-identical against `sigrok-cli`'s own `uart` decoder. **A PWM
decoder does not exist yet** — without it the rig captures the discriminating
signal but cannot read it.

Logs cost ~0.9 MB/min, so soak runs are cheap. Raw samples need `--raw`, cost
0.48 GB/min, and exist only to feed `crsf_slice.py`: soak without raw until the
fault reproduces, then repeat with `--raw` (and `--max-gb`) for the waveform.

## Acknowledgements

[`sigrok_crsf_decoder`][crsf-pd] by James Cordell is what makes a captured window
readable in PulseView, annotating CRSF on top of sigrok's `uart`. Install its
`crsf/` directory into `~/.local/share/libsigrokdecode/decoders/`. The Python
decoder here is independent; run against the same capture, the two agree channel
for channel.

[crsf-pd]: https://github.com/JamesCordell/sigrok_crsf_decoder/
[2548]: https://github.com/ExpressLRS/ExpressLRS/issues/2548
[3157]: https://github.com/ExpressLRS/ExpressLRS/issues/3157
[3617]: https://github.com/ExpressLRS/ExpressLRS/issues/3617
[3631]: https://github.com/ExpressLRS/ExpressLRS/issues/3631
[3623]: https://github.com/ExpressLRS/ExpressLRS/pull/3623

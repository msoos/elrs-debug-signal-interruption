"""CRSF (Crossfire/ELRS) decoding from raw logic-analyzer samples."""

import numpy as np

SYNC_BYTES = (0xC8, 0xEE, 0xEA, 0xEC)
T_RC_CHANNELS = 0x16
T_LINK_STATS = 0x14

TYPE_NAMES = {
    0x02: "GPS", 0x08: "BATTERY", 0x14: "LINK_STATS", 0x16: "RC_CHANNELS",
    0x1E: "ATTITUDE", 0x21: "FLIGHT_MODE", 0x28: "PING", 0x29: "DEVICE_INFO",
    0x2B: "PARAM_ENTRY", 0x2D: "PARAM_READ", 0x2E: "PARAM_WRITE", 0x32: "COMMAND",
}

TICK_MIN, TICK_MAX = 172, 1811
TICK_CENTER = 992

_CRC8 = []
for _b in range(256):
    _c = _b
    for _ in range(8):
        _c = ((_c << 1) ^ 0xD5) & 0xFF if _c & 0x80 else (_c << 1) & 0xFF
    _CRC8.append(_c)


def crc8(data):
    c = 0
    for x in data:
        c = _CRC8[c ^ x]
    return c


def ticks_to_us(t):
    return t * 5.0 / 8.0 + 880.0


def us_to_ticks(us):
    return (us - 880.0) * 8.0 / 5.0


TX_POWER_MW = {0: 0, 1: 10, 2: 25, 3: 100, 4: 500, 5: 1000, 6: 2000, 7: 50}


def parse_link_stats(payload):
    """LINK_STATISTICS (0x14) payload -> dict. RSSI values are negative dBm."""
    if len(payload) < 10:
        return None
    s8 = lambda b: b - 256 if b > 127 else b
    return {
        "up_rssi1": -payload[0], "up_rssi2": -payload[1],
        "up_lq": payload[2], "up_snr": s8(payload[3]),
        "antenna": payload[4], "rf_mode": payload[5],
        "tx_power_mw": TX_POWER_MW.get(payload[6], payload[6]),
        "dn_rssi": -payload[7], "dn_lq": payload[8], "dn_snr": s8(payload[9]),
    }


def unpack_channels(payload):
    """22 bytes -> 16 channels of 11 bits, LSB-first."""
    v = int.from_bytes(payload[:22], "little")
    return [(v >> (11 * i)) & 0x7FF for i in range(16)]


class UartDecoder:
    """Streaming 8N1 UART decoder over packed logic samples.

    feed() may be called with arbitrary block sizes; bytes straddling a block
    boundary are carried over rather than lost.
    """

    def __init__(self, samplerate, baud, channel=0):
        self.bit = samplerate / baud
        self.channel = channel
        self._resid = np.empty(0, np.uint8)
        self._base = 0
        self._off = np.round(self.bit * (np.arange(1, 9) + 0.5)).astype(np.int64)
        self._stop_off = int(round(self.bit * 9.5))
        # earliest plausible next start bit; past the stop-bit sample point but
        # tolerant of a slightly early edge from clock skew
        self._next_off = int(round(self.bit * 9.5))
        self._span = self._stop_off + 1

    def feed(self, raw):
        """raw: uint8 array of packed channels. -> [(sample, value, framing_ok)]"""
        bits = ((raw >> self.channel) & 1).astype(np.uint8)
        b = np.concatenate((self._resid, bits)) if self._resid.size else bits
        n = b.size
        base = self._base

        if n < self._span + 2:
            self._resid = b
            return []

        falling = np.flatnonzero((b[:-1] == 1) & (b[1:] == 0)) + 1
        limit = n - self._span - 1
        cand = falling[falling <= limit]

        out = []
        consumed = 0
        if cand.size:
            idx = cand[:, None] + self._off[None, :]
            vals = (b[idx].astype(np.uint16) << np.arange(8, dtype=np.uint16)).sum(1)
            stops = b[cand + self._stop_off]
            last_end = -1
            for k in range(cand.size):
                s = int(cand[k])
                if s < last_end:
                    continue
                out.append((base + s, int(vals[k]), bool(stops[k])))
                last_end = s + self._next_off
            consumed = max(0, last_end)

        pending = falling[falling > limit]
        keep_from = max(consumed, int(pending[0]) - 1 if pending.size else n - 1)
        keep_from = min(keep_from, n - 1)

        self._resid = b[keep_from:].copy()
        self._base = base + keep_from
        return out


class CrsfParser:
    """Assembles CRSF frames from a UART byte stream, resyncing on bad CRC."""

    def __init__(self, sync=SYNC_BYTES, max_len=62):
        self.sync = set(sync)
        self.max_len = max_len
        self.dropped = 0
        self.dropped_bytes = []
        self._buf = []

    def _drop(self, value):
        self.dropped += 1
        if len(self.dropped_bytes) < 64:
            self.dropped_bytes.append(value)

    def feed(self, byte_events):
        self._buf.extend(byte_events)
        frames = []
        i = 0
        buf = self._buf
        n = len(buf)

        while i < n:
            if buf[i][1] not in self.sync:
                self._drop(buf[i][1])
                i += 1
                continue
            if i + 1 >= n:
                break
            length = buf[i + 1][1]
            if not (2 <= length <= self.max_len):
                self._drop(buf[i][1])
                i += 1
                continue
            if i + 2 + length > n:
                break

            body = [buf[j][1] for j in range(i + 2, i + 2 + length)]
            if crc8(body[:-1]) != body[-1]:
                self._drop(buf[i][1])
                i += 1
                continue

            framing_ok = all(buf[j][2] for j in range(i, i + 2 + length))
            frames.append({
                "sample": buf[i][0],
                "sync": buf[i][1],
                "length": length,
                "type": body[0],
                "payload": bytes(body[1:-1]),
                "crc_ok": True,
                "framing_ok": framing_ok,
            })
            i += 2 + length

        self._buf = buf[i:]
        return frames


def decode_samples(raw, samplerate, baud, channel=0):
    """One-shot convenience decode. -> (byte_events, frames)"""
    u = UartDecoder(samplerate, baud, channel)
    b = u.feed(raw)
    return b, CrsfParser().feed(b)

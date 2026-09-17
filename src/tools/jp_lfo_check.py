#!/usr/bin/env python3
"""
jp_lfo_check.py -- does the JP-8000's modulation actually MODULATE?

Every LFO parameter reads back fine from our own image, and every one of them
lands in the temp patch area whose DT1 path is already proven (cutoff 0 vs 127).
Neither fact is evidence: `jp8000_plugin.cpp` writes the image before the DT1,
so a read-back is our own store coming home, and "the area works" says nothing
about these addresses.  The arpeggiator read back ON for a whole session while
producing exactly one voice start.

So this measures the AUDIO, and it measures the one thing a wrong-patch or
right-patch-wrong-phase render cannot fake: **the modulation frequency**.  A
tremolo at the rate you asked for is the LFO; a difference in the waveform is
not.  Same argument that settled the arpeggiator -- 31 attacks at a dead-regular
0.405 s, an eighth at the performance's stored 74 BPM.

Each route is measured against BOTH controls:
  - depth at 0 (the bipolar centre, 64) must show no modulation at all, and
  - the measured rate must track `lfo1_rate` / `lfo2_rate` monotonically.
A single "it wobbles" reading proves nothing -- a chorused saw wobbles on its
own, which is exactly how a spectral-similarity score once scored 0.86 on a
patch that was 143/528 bytes wrong.

Usage:
  jp_lfo_check.py --audit                 # every LFO address vs jeLib, no ROM needed
  jp_lfo_check.py --selftest              # validate the DETECTORS, no ROM needed
  jp_lfo_check.py [--rom DIR] [--render BIN] [--out DIR] [--only NAME[,NAME]]

`--selftest` synthesises signals with a known modulation rate and asserts the
detectors recover it, and that flat controls report nothing.  Run it whenever
you touch the analysis; a detector nobody has calibrated is the instrument that
reported "channel 2 arpeggiates".

Requires JP-8000 v1.5 ROMs (8 .mid files) in --rom, and jp8000_render built.
"""
import argparse
import os
import subprocess
import sys
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "../.."))
DEFAULT_ROM = os.path.join(ROOT, "dist/jp8000/roms")
DEFAULT_RENDER = os.path.join(ROOT, "build-native/jp8000_render")

# ---------------------------------------------------------------------------
# Patch bytes.  Offsets are jeLib's `Patch` enum (jemiditypes.h).
# ---------------------------------------------------------------------------
P = dict(
    Lfo1Wave=0x10, Lfo1Rate=0x11, Lfo1Fade=0x12, Lfo2Rate=0x13, Lfo2DepthSel=0x14,
    RingMod=0x15, XMod=0x16, OscBal=0x17, Lfo1EnvDest=0x18,
    OscLfo1Depth=0x19, PitchLfo2Depth=0x1A, PitchEnvDepth=0x1B,
    PitchEnvA=0x1C, PitchEnvD=0x1D,
    Osc1Wave=0x1E, Osc1C1=0x1F, Osc1C2=0x20, Osc2Wave=0x21, Osc2Sync=0x22,
    Osc2Range=0x23, Osc2Fine=0x24, Osc2C1=0x25, Osc2C2=0x26,
    FiltType=0x27, CutSlope=0x28, Cut=0x29, Res=0x2A, CutKeyFollow=0x2B,
    FiltLfo1Depth=0x2C, FiltLfo2Depth=0x2D, FEnvDep=0x2E,
    FA=0x2F, FD=0x30, FS=0x31, FR=0x32,
    AmpLvl=0x33, AmpLfo1Depth=0x34, AmpLfo2Depth=0x35,
    AA=0x36, AD=0x37, AS=0x38, AR=0x39,
    AmpLfo1Mode=0x3A,           # AutoPanManualPanSwitch: 0 OFF, 1 AUTO, 2 MANUAL
    ToneBass=0x3B, ToneTreble=0x3C,
    FxType=0x3D, FxLvl=0x3E, DelayType=0x3F, DelayTime=0x40, DelayFb=0x41,
    DelayLvl=0x42, Porta=0x45, PortaTime=0x46, Mono=0x47,
)

# Part block (jeLib `Part`) and performance common (jeLib `PerformanceCommon`).
PART = dict(MidiChannel=0x02, Transpose=0x03, DelaySync=0x04, LfoSync=0x05,
            ChorusSync=0x06)
COMMON = dict(Tempo=0x22)   # 14-bit, BPM

# Osc1Waveform: 0 SUPER SAW .. 4 PULSE, 5 SAW, 6 TRI.  Osc2Waveform: 2 = SAW.
WAVE_SAW = 5
BIPOLAR_ZERO = 64           # -64..+63 depths are raw 0..127, centred on 64
LFO1_WAVE_TRI, LFO1_WAVE_SAW, LFO1_WAVE_SQR, LFO1_WAVE_SH = 0, 1, 2, 3
DEST_PITCH, DEST_FILTER, DEST_AMP = 0, 1, 2
MODE_OFF, MODE_AUTOPAN, MODE_MANUALPAN = 0, 1, 2


def linear_to_packed(lin):
    """Roland addresses carry 7 bits per byte; our offsets are linear."""
    return ((lin // 128) << 8) | (lin % 128)


BLOCK_COMMON, BLOCK_PART_UP, BLOCK_PATCH_UP = 0x0000, 0x1000, 0x4000


def dataset1(off, value, block=BLOCK_PATCH_UP, bit14=False):
    """One DT1 into a block of the temp performance (area 0x01 00 ....)."""
    full = block + linear_to_packed(off)
    addr = [0x01, 0x00, (full >> 8) & 0x7F, full & 0x7F]
    data = [(value >> 7) & 0x7F, value & 0x7F] if bit14 else [value & 0x7F]
    chk = (0x80 - (sum(addr + data) & 0x7F)) & 0x7F
    return bytes([0xF0, 0x41, 0x10, 0x00, 0x06, 0x12] + addr + data + [chk, 0xF7])


def base_patch(part_lfo_sync=0, tempo_bpm=None, **overrides):
    """A patch that holds still: one saw, no envelopes moving, no effects, and
    EVERY modulation depth at its bipolar centre.  Anything that moves in the
    render of this patch is the thing under test -- which is the only reason a
    'no modulation' control means anything."""
    p = {
        # sound source: one plain saw, no super-saw detune (it beats on its own)
        'Osc1Wave': WAVE_SAW, 'Osc1C1': 64, 'Osc1C2': 0, 'OscBal': 0,
        'Osc2Wave': 2, 'Osc2Range': 64, 'Osc2Fine': 64, 'RingMod': 0, 'XMod': 0,
        # filter: open and quiet, tests that need it override cutoff/resonance
        'FiltType': 2, 'CutSlope': 1, 'Cut': 110, 'Res': 0, 'CutKeyFollow': 64,
        'FEnvDep': BIPOLAR_ZERO, 'FA': 0, 'FD': 0, 'FS': 127, 'FR': 0,
        # amp: instant attack, full sustain -- a flat plateau to measure against
        'AmpLvl': 110, 'AA': 0, 'AD': 0, 'AS': 127, 'AR': 0,
        'AmpLfo1Mode': MODE_OFF,
        # every modulation depth OFF
        'OscLfo1Depth': BIPOLAR_ZERO, 'PitchLfo2Depth': BIPOLAR_ZERO,
        'PitchEnvDepth': BIPOLAR_ZERO, 'PitchEnvA': 0, 'PitchEnvD': 0,
        'FiltLfo1Depth': BIPOLAR_ZERO, 'FiltLfo2Depth': BIPOLAR_ZERO,
        'AmpLfo1Depth': BIPOLAR_ZERO, 'AmpLfo2Depth': BIPOLAR_ZERO,
        'Lfo1Wave': LFO1_WAVE_TRI, 'Lfo1Rate': 64, 'Lfo1Fade': 0,
        'Lfo2Rate': 64, 'Lfo2DepthSel': DEST_PITCH, 'Lfo1EnvDest': 0,
        # effects off: chorus and delay both move, and both would read as an LFO
        'ToneBass': 64, 'ToneTreble': 64, 'FxLvl': 0, 'DelayLvl': 0, 'DelayFb': 0,
        'Porta': 0, 'Mono': 0,
    }
    p.update(overrides)
    out = b"".join(dataset1(P[k], v) for k, v in p.items())
    # The PART block decides whether LFO1 free-runs at Lfo1Rate or follows the
    # performance tempo, and it belongs to whatever performance the snapshot
    # booted -- not to our patch.  Leaving it alone would let a synced preset
    # pin every rate in the sweep to the same tempo division and read as "the
    # rate byte does nothing".  Pin it OFF here; lfo1_sync_follows_tempo turns
    # it back on deliberately.
    out += b"".join(dataset1(off, val, BLOCK_PART_UP) for off, val in (
        (PART['DelaySync'], 0), (PART['LfoSync'], part_lfo_sync),
        (PART['ChorusSync'], 0)))
    if tempo_bpm is not None:
        out += dataset1(COMMON['Tempo'], tempo_bpm, BLOCK_COMMON, bit14=True)
    return out


# ---------------------------------------------------------------------------
# Render
# ---------------------------------------------------------------------------

def render(render_bin, rom_dir, patch_bytes, out_wav, note=60, vel=100,
           seconds=6.0, hold=5.4):
    """Long note on purpose: a slow LFO needs several cycles inside the analysis
    window, and the sysex has to clock in before the note (jeLib rate-limits it
    -- a whole patch is ~230 ms of emulated time)."""
    pf = out_wav + ".syx"
    with open(pf, "wb") as f:
        f.write(patch_bytes)
    script = out_wav + ".txt"
    with open(script, "w") as f:
        f.write(f"render_seconds {seconds}\npatch_file {pf}\n"
                f"500 on {note} {vel}\n{int(hold * 1000)} off {note}\n")
    r = subprocess.run([render_bin, rom_dir, script, out_wav],
                       capture_output=True, text=True)
    if not os.path.exists(out_wav) or os.path.getsize(out_wav) < 1000:
        raise RuntimeError(f"render produced nothing: {r.stderr.strip()[-400:]}")
    return out_wav


def read_wav(path):
    """24-bit (or 16-bit) stereo -> (sr, L, R) as floats.  Keep the channels
    APART: auto-pan lives entirely in their difference and a mono sum erases
    it, which is the one thing a mono analysis of this synth cannot see."""
    with wave.open(path, "rb") as w:
        sr, nch, sw = w.getframerate(), w.getnchannels(), w.getsampwidth()
        raw = np.frombuffer(w.readframes(w.getnframes()),
                            dtype=np.uint8).reshape(-1, nch * sw)
    chans = []
    for c in range(nch):
        b = raw[:, c * sw:c * sw + sw].astype(np.int32)
        if sw == 3:
            v = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
            v = np.where(v & 0x800000, v - 0x1000000, v).astype(np.float64) / 8388608.0
        elif sw == 2:
            v = b[:, 0] | (b[:, 1] << 8)
            v = np.where(v & 0x8000, v - 0x10000, v).astype(np.float64) / 32768.0
        else:
            raise ValueError(f"unsupported sample width {sw}")
        chans.append(v)
    if len(chans) == 1:
        chans.append(chans[0])
    return sr, chans[0], chans[1]


# ---------------------------------------------------------------------------
# Tracks: turn the audio into one number per short window, so a slow modulation
# becomes an ordinary signal we can take a spectrum of.
# ---------------------------------------------------------------------------
TRACK_WIN = 2048            # 23 ms at 88.2 kHz
TRACK_HOP = 256             # -> 344.5 Hz track rate, good to ~170 Hz of LFO


def _frames(x, sr, t0, t1, win=TRACK_WIN, hop=TRACK_HOP):
    """Clamped to the buffer: a render that came up short is a short render,
    not a NaN in the middle of a spectrum."""
    a = max(0, int(t0 * sr))
    b = min(len(x), int(t1 * sr))
    n = (b - a - win) // hop + 1
    if n < 16:
        raise ValueError("analysis window too short")
    idx = a + np.arange(n) * hop
    return idx, sr / hop


def amp_track(x, sr, t0, t1):
    """RMS per window -> the amplitude envelope."""
    idx, trate = _frames(x, sr, t0, t1)
    out = np.array([np.sqrt(np.mean(x[i:i + TRACK_WIN] ** 2)) for i in idx])
    return out, trate


def pan_track(L, R, sr, t0, t1):
    """(L-R)/(L+R) per window: 0 centred, +-1 hard.  Deliberately blind to
    tremolo -- a level change moves both channels and cancels here."""
    idx, trate = _frames(L, sr, t0, t1)
    out = []
    for i in idx:
        l = np.sqrt(np.mean(L[i:i + TRACK_WIN] ** 2))
        r = np.sqrt(np.mean(R[i:i + TRACK_WIN] ** 2))
        out.append((l - r) / (l + r) if (l + r) > 1e-12 else 0.0)
    return np.array(out), trate


def _parabolic(mag, k):
    if k <= 0 or k >= len(mag) - 1:
        return float(k)
    a, b, c = mag[k - 1], mag[k], mag[k + 1]
    d = a - 2 * b + c
    return float(k) if abs(d) < 1e-20 else k + 0.5 * (a - c) / d


def pitch_track(x, sr, t0, t1, fmin=60.0, fmax=1200.0):
    """Strongest partial per window, parabolically interpolated, returned in
    CENTS relative to its own median.  Cents because 'the pitch wobbles by 30'
    is a musical statement and a percentage of Hz is not."""
    idx, trate = _frames(x, sr, t0, t1)
    win = np.hanning(TRACK_WIN)
    freqs = np.fft.rfftfreq(TRACK_WIN, 1.0 / sr)
    lo = int(np.searchsorted(freqs, fmin))
    hi = int(np.searchsorted(freqs, fmax))
    df = sr / TRACK_WIN
    out = []
    for i in idx:
        mag = np.abs(np.fft.rfft(x[i:i + TRACK_WIN] * win))
        k = lo + int(np.argmax(mag[lo:hi]))
        out.append(_parabolic(mag, k) * df)
    f = np.array(out)
    med = np.median(f[f > 0]) if np.any(f > 0) else 1.0
    return 1200.0 * np.log2(np.maximum(f, 1e-9) / med), trate


def centroid_track(x, sr, t0, t1, fmin=40.0, fmax=18000.0):
    """Spectral centroid per window, in Hz.  A filter sweeping its corner drags
    the centroid with it; an amplitude change does not move it at all."""
    idx, trate = _frames(x, sr, t0, t1)
    win = np.hanning(TRACK_WIN)
    freqs = np.fft.rfftfreq(TRACK_WIN, 1.0 / sr)
    k = (freqs >= fmin) & (freqs <= fmax)
    fk = freqs[k]
    out = []
    for i in idx:
        mag = np.abs(np.fft.rfft(x[i:i + TRACK_WIN] * win))[k]
        s = mag.sum()
        out.append(float((mag * fk).sum() / s) if s > 1e-12 else 0.0)
    return np.array(out), trate


# ---------------------------------------------------------------------------
# The detector: is this track modulated, and at what rate?
# ---------------------------------------------------------------------------
class Mod:
    def __init__(self, hz, index, prominence, present):
        self.hz, self.index, self.prominence, self.present = hz, index, prominence, present

    def __repr__(self):
        return (f"{'MOD' if self.present else 'flat'} "
                f"{self.hz:6.3f} Hz  idx {self.index:8.3f}  prom {self.prominence:6.1f}")


def detect_mod(track, trate, fmin=0.08, fmax=40.0, relative=True,
               prom_min=8.0, index_min=0.02):
    """Spectrum of the track.  `index` is the peak-to-peak swing of the
    modulation -- as a fraction of the mean when `relative`, in the track's own
    units otherwise (cents for pitch, -1..1 for pan).

    `prominence` is the peak against the MEDIAN of the band, i.e. against this
    track's own noise floor.  Both tests have to pass: a big number with no
    single rate behind it is noise, and a sharp peak carrying no swing is
    quantisation.  A run of the unmodulated control is what sets the bar --
    see --selftest."""
    x = np.asarray(track, dtype=np.float64)
    mean = float(np.mean(x))
    # Remove mean and linear trend: a decaying note is not a modulation, and a
    # ramp smears energy across the whole low band.
    t = np.arange(len(x))
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    w = np.hanning(len(x))
    # A Hann window halves the coherent gain; 4/N recovers peak-to-peak for a
    # sinusoid, which is what "modulation depth" means to a musician.
    sp = np.abs(np.fft.rfft(x * w)) * (4.0 / len(x)) / 0.5
    fr = np.fft.rfftfreq(len(x), 1.0 / trate)
    band = (fr >= fmin) & (fr <= fmax)
    if not np.any(band):
        return Mod(0.0, 0.0, 0.0, False)
    sb, fb = sp[band], fr[band]
    k = int(np.argmax(sb))
    peak = float(sb[k])
    # Parabolic interpolation on the bin: the LFO rate is not on a bin centre.
    hz = float(np.interp(_parabolic(sb, k), np.arange(len(fb)), fb))
    floor = float(np.median(sb)) or 1e-18
    index = peak / abs(mean) if (relative and abs(mean) > 1e-12) else peak
    prom = peak / floor
    return Mod(hz, index, prom, bool(prom >= prom_min and index >= index_min))


# ---------------------------------------------------------------------------
# The engine checks
# ---------------------------------------------------------------------------
ANALYSIS = (1.6, 5.2)       # skip the note's attack and the sysex settle


def _analyse(path, kind):
    sr, L, R = read_wav(path)
    t0, t1 = ANALYSIS
    if kind == "amp":
        tr, rate = amp_track(0.5 * (L + R), sr, t0, t1)
        return detect_mod(tr, rate, index_min=0.03)
    if kind == "pan":
        tr, rate = pan_track(L, R, sr, t0, t1)
        return detect_mod(tr, rate, relative=False, index_min=0.03)
    if kind == "pitch":
        tr, rate = pitch_track(0.5 * (L + R), sr, t0, t1)
        return detect_mod(tr, rate, relative=False, index_min=6.0)   # cents
    if kind == "centroid":
        tr, rate = centroid_track(0.5 * (L + R), sr, t0, t1)
        return detect_mod(tr, rate, index_min=0.03)
    raise ValueError(kind)


# name -> (metric, patch overrides with the depth ON, overrides with it OFF)
ROUTES = {
    # LFO1 -> pitch.  Lfo1AndEnvelopeDestination stays OSC1+2.
    "lfo1_pitch":   ("pitch",    dict(OscLfo1Depth=100), dict()),
    # LFO1 -> filter.  Cutoff parked low with some resonance so the sweep has
    # somewhere to go and drags the centroid audibly.
    "lfo1_filter":  ("centroid", dict(FiltLfo1Depth=110, Cut=60, Res=40),
                                 dict(Cut=60, Res=40)),
    # LFO1 -> amp (tremolo).  Only with AmpLfo1Mode = OFF; see lfo1_pan below.
    "lfo1_amp":     ("amp",      dict(AmpLfo1Depth=120), dict()),
    # LFO1 -> pan.  Same depth byte, different meaning -- AUTO makes it pan.
    "lfo1_pan":     ("pan",      dict(AmpLfo1Depth=120, AmpLfo1Mode=MODE_AUTOPAN),
                                 dict(AmpLfo1Mode=MODE_AUTOPAN)),
    # LFO2 has ONE destination at a time: the depth byte only acts when
    # Lfo2DepthSelect points at it.  Each route therefore sets both.
    "lfo2_pitch":   ("pitch",    dict(PitchLfo2Depth=100, Lfo2DepthSel=DEST_PITCH),
                                 dict(Lfo2DepthSel=DEST_PITCH)),
    "lfo2_filter":  ("centroid", dict(FiltLfo2Depth=110, Lfo2DepthSel=DEST_FILTER,
                                      Cut=60, Res=40),
                                 dict(Lfo2DepthSel=DEST_FILTER, Cut=60, Res=40)),
    "lfo2_amp":     ("amp",      dict(AmpLfo2Depth=120, Lfo2DepthSel=DEST_AMP),
                                 dict(Lfo2DepthSel=DEST_AMP)),
}

# Rates to sweep.  The point is not the absolute Hz -- it is that the rate the
# audio shows MOVES with the byte, monotonically.  That is what no amount of
# "the patch sounds different" can accidentally produce.
RATE_SWEEP = [20, 50, 80, 110]


class Report:
    def __init__(self):
        self.rows = []
        self.failed = 0

    def add(self, name, ok, detail):
        self.rows.append((name, ok, detail))
        if not ok:
            self.failed += 1
        print(f"  {'PASS' if ok else 'FAIL'}  {name:22s} {detail}")
        sys.stdout.flush()


def run_engine_checks(args, rep):
    os.makedirs(args.out, exist_ok=True)

    def go(tag, **ov):
        wav = os.path.join(args.out, f"{tag}.wav")
        render(args.render, args.rom, base_patch(**ov), wav,
               seconds=args.seconds, hold=args.seconds - 0.6)
        return wav

    only = set(args.only.split(",")) if args.only else None

    print("\n-- routes: depth ON must modulate, depth OFF must not --")
    for name, (kind, on, off) in ROUTES.items():
        if only and name not in only:
            continue
        m_off = _analyse(go(f"{name}_off", **off), kind)
        m_on = _analyse(go(f"{name}_on", **on), kind)
        ok = m_on.present and not m_off.present
        rep.add(name, ok, f"on: {m_on}   off: {m_off}")

    if only and "rate" not in only:
        return
    print("\n-- rate: the measured modulation must follow the rate byte --")
    for lfo in ("lfo1", "lfo2"):
        seen = []
        for rate in RATE_SWEEP:
            if lfo == "lfo1":
                ov = dict(AmpLfo1Depth=120, Lfo1Rate=rate)
            else:
                ov = dict(AmpLfo2Depth=120, Lfo2DepthSel=DEST_AMP, Lfo2Rate=rate)
            m = _analyse(go(f"{lfo}_rate{rate}", **ov), "amp")
            seen.append((rate, m))
            print(f"        rate {rate:3d} -> {m}")
        hz = [m.hz for _, m in seen]
        allmod = all(m.present for _, m in seen)
        mono = all(b > a * 1.15 for a, b in zip(hz, hz[1:]))
        rep.add(f"{lfo}_rate_tracks", allmod and mono,
                "Hz: " + ", ".join(f"{h:.2f}" for h in hz) +
                ("" if allmod else "  (a step showed no modulation)") +
                ("" if mono else "  (not monotonic)"))

    if only:
        return
    print("\n-- interactions --")
    # LFO2's depth select really is exclusive: a filter depth written while the
    # select points at PITCH must do nothing.  If this one ever FAILS, the
    # module's Filter page is lying less than we thought, not more.
    m = _analyse(go("lfo2_wrongdest", FiltLfo2Depth=110, Lfo2DepthSel=DEST_PITCH,
                    Cut=60, Res=40), "centroid")
    rep.add("lfo2_dest_exclusive", not m.present, f"filter depth w/ dest=PITCH -> {m}")

    # MANUAL pan turns the same byte into a static position: a pan offset with
    # no motion.  The knob is not dead, it just is not an LFO any more.
    sr, L, R = read_wav(go("lfo1_manualpan", AmpLfo1Depth=120,
                           AmpLfo1Mode=MODE_MANUALPAN))
    tr, trate = pan_track(L, R, sr, *ANALYSIS)
    m = detect_mod(tr, trate, relative=False, index_min=0.03)
    rep.add("lfo1_manualpan_static", (not m.present) and abs(np.mean(tr)) > 0.05,
            f"pan offset {np.mean(tr):+.3f}, motion: {m}")

    # Fade delays the LFO's onset: early window flat, late window modulated.
    wav = go("lfo1_fade", AmpLfo1Depth=120, Lfo1Rate=90, Lfo1Fade=110)
    sr, L, R = read_wav(wav)
    early, _ = amp_track(0.5 * (L + R), sr, 0.7, 1.7)
    late, rate2 = amp_track(0.5 * (L + R), sr, 4.0, 5.4)
    m_e = detect_mod(early, rate2, index_min=0.03)
    m_l = detect_mod(late, rate2, index_min=0.03)
    rep.add("lfo1_fade_delays", (not m_e.present) and m_l.present,
            f"early: {m_e}   late: {m_l}")

    # LFO1 SYNC is a PART parameter, not a patch one, and when it is set the
    # rate byte stops mattering: the LFO runs off the performance tempo.  That
    # makes it the second independent confirmation, the way the arp's 0.405 s
    # matched its stored 74 BPM -- the audio has to land on a number nothing in
    # the patch mentions.  Both renders below carry a DIFFERENT Lfo1Rate on
    # purpose: if the measured rate follows the tempo and not the byte, the
    # sync path is real.  (No external clock is sent; the firmware's own clock
    # is what the arpeggiator already runs on.)
    TEMPO = 120
    for div, label, expect in ((7, "1/4", TEMPO / 60.0), (4, "1/8", TEMPO / 30.0)):
        rate_byte = 20 if div == 7 else 110
        wav = go(f"lfo1_sync_{div}", part_lfo_sync=div, tempo_bpm=TEMPO,
                 AmpLfo1Depth=120, Lfo1Rate=rate_byte)
        m = _analyse(wav, "amp")
        err = abs(m.hz - expect) / expect if m.present else 1.0
        rep.add(f"lfo1_sync_{label}", m.present and err < 0.08,
                f"{m}   expect {expect:.2f} Hz at {TEMPO} BPM "
                f"(rate byte {rate_byte}, must be ignored)")

    # Waveform: all four must modulate, and SQR/S&H must not look like TRI.
    shapes = {}
    for wname, wv in (("tri", LFO1_WAVE_TRI), ("saw", LFO1_WAVE_SAW),
                      ("sqr", LFO1_WAVE_SQR), ("s&h", LFO1_WAVE_SH)):
        wav = go(f"lfo1_wave_{wv}", AmpLfo1Depth=120, Lfo1Rate=70, Lfo1Wave=wv)
        sr, L, R = read_wav(wav)
        tr, trate = amp_track(0.5 * (L + R), sr, *ANALYSIS)
        m = detect_mod(tr, trate, index_min=0.03)
        # Crest of the FIRST DIFFERENCE separates a ramp from a step: a square
        # and a sample&hold jump, a triangle and a saw do not.
        d = np.abs(np.diff(tr))
        shapes[wname] = (m, float(d.max() / (d.mean() + 1e-18)))
        print(f"        {wname:4s} -> {m}   step-crest {shapes[wname][1]:.1f}")
    allmod = all(m.present for m, _ in shapes.values())
    stepped = shapes["sqr"][1] > 2.0 * shapes["tri"][1]
    rep.add("lfo1_waveforms", allmod and stepped,
            ("all four modulate" if allmod else "a waveform produced nothing") +
            (", square steps" if stepped else ", square does not look stepped"))


# ---------------------------------------------------------------------------
# Address audit: does every LFO knob the module exposes point where jeLib says?
#
# This is the cheap half of the question and it needs no ROM.  It is also the
# half that has been wrong before in this tree -- the system area wrote to the
# temp base and landed nowhere, and `PerformanceControlChannel` accepted a value
# the firmware rejects.  Our table stores LINEAR offsets, jemiditypes.h stores
# PACKED addresses, so the comparison has to convert.
# ---------------------------------------------------------------------------
AUDIT_OVERRIDES = {          # jeLib enum name -> our key, where snake() cannot
    "Lfo1AndEnvelopeDestination": "lfo1_env_dest",
    "AutoPanManualPanSwitch": "amp_lfo1_mode",
    "LfoSync": ("up_lfo1_sync", "lo_lfo1_sync"),
}
AREA_OF_ENUM = {"Patch": (0,), "Part": (1, 2), "PerformanceCommon": (3,)}


def _snake(name):
    import re
    n = re.sub(r'(?<=[a-z0-9])(?=[A-Z])', '_', name).lower()
    if n.startswith("control_"):
        n = "ctl_" + n[len("control_"):]
    elif n.startswith("velocity_") and n != "velocity_switch":
        n = "vel_" + n[len("velocity_"):]
    return n


def _parse_jemiditypes(path):
    """{enum block name: {member: value}} -- enough of a C++ parser for a file
    that is nothing but `Name = 0x...,` lines."""
    import re
    out, cur = {}, None
    for line in open(path):
        m = re.match(r'\s*enum class (\w+)', line)
        if m:
            cur = m.group(1)
            out[cur] = {}
            continue
        m = re.match(r'\s*(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*,', line)
        if m and cur:
            out[cur][m.group(1)] = int(m.group(2), 0)
    return out


def _parse_params(path):
    """{key: (area, linear offset, bit14)} from the generated table."""
    import re
    out = {}
    for line in open(path):
        m = re.match(r'\s*\{"([a-z0-9_]+)",\s*(\d+),\s*(0x[0-9a-f]+|\d+),\s*(\d+),', line)
        if m:
            out[m.group(1)] = (int(m.group(2)), int(m.group(3), 0), int(m.group(4)))
    return out


def audit():
    je = _parse_jemiditypes(os.path.join(
        ROOT, "libs/gearmulator/source/ronaldo/je8086/jeLib/jemiditypes.h"))
    if not je.get("Patch"):
        print("jemiditypes.h not found -- run `git submodule update --init`", file=sys.stderr)
        return 2
    ours = _parse_params(os.path.join(ROOT, "src/dsp/jp8000_params.h"))
    if not ours:
        print("src/dsp/jp8000_params.h not parseable", file=sys.stderr)
        return 2

    bad, checked, unexposed = [], 0, []
    for block, areas in AREA_OF_ENUM.items():
        for member, packed in je.get(block, {}).items():
            if "Lfo" not in member and member != "AutoPanManualPanSwitch":
                continue
            if member.startswith("DataLength"):
                continue
            keys = AUDIT_OVERRIDES.get(member, _snake(member))
            keys = (keys,) if isinstance(keys, str) else keys
            for key in keys:
                if key not in ours:
                    unexposed.append(f"{block}::{member} ({key})")
                    continue
                area, lin, _ = ours[key]
                checked += 1
                got = linear_to_packed(lin)
                if got != packed or area not in areas:
                    bad.append(f"{key}: table area {area} addr {got:#06x}, "
                               f"jeLib {block}::{member} {packed:#06x}")
    print(f"LFO address audit: {checked} keys checked against jeLib")
    for b in bad:
        print("  MISMATCH  " + b)
    for u in sorted(set(unexposed)):
        print("  not exposed by the module: " + u)
    print("  addresses:", "all agree" if not bad else f"{len(bad)} WRONG")
    return 1 if bad else 0


# ---------------------------------------------------------------------------
# Self-test: calibrate the detectors against signals whose answer we know.
# ---------------------------------------------------------------------------
def selftest():
    sr = 88200
    n = int(sr * 6.0)
    t = np.arange(n) / sr
    rng = np.random.default_rng(7)
    ok = True

    def check(label, got, want, tol, present=True):
        nonlocal ok
        good = (got.present == present) and (not present or abs(got.hz - want) <= tol)
        ok &= good
        print(f"  {'PASS' if good else 'FAIL'}  {label:26s} {got}"
              + (f"   want {want} Hz" if present else "   want: flat"))

    def saw(f0_hz_series):
        ph = 2 * np.pi * np.cumsum(f0_hz_series) / sr
        return sum(np.sin(k * ph) / k for k in range(1, 12))

    # 1. tremolo -> amp track
    car = saw(np.full(n, 220.0))
    trem = 5.3
    am = car * (1 + 0.35 * np.sin(2 * np.pi * trem * t))
    tr, rate = amp_track(am, sr, *ANALYSIS)
    check("amp: 5.3 Hz tremolo", detect_mod(tr, rate, index_min=0.03), trem, 0.15)

    # ...and the control, which is what the 'off' render has to look like.
    flat = car * (1 + 0.0015 * rng.standard_normal(n))
    tr, rate = amp_track(flat, sr, *ANALYSIS)
    check("amp: steady control", detect_mod(tr, rate, index_min=0.03), 0, 0, present=False)

    # 2. vibrato -> pitch track (40 cents, 3.1 Hz)
    vib = 3.1
    f0 = 220.0 * 2 ** ((40.0 / 1200.0) * np.sin(2 * np.pi * vib * t))
    tr, rate = pitch_track(saw(f0), sr, *ANALYSIS)
    check("pitch: 3.1 Hz vibrato", detect_mod(tr, rate, relative=False, index_min=6.0), vib, 0.15)
    tr, rate = pitch_track(saw(np.full(n, 220.0)), sr, *ANALYSIS)
    check("pitch: steady control", detect_mod(tr, rate, relative=False, index_min=6.0),
          0, 0, present=False)

    # 3. filter sweep -> centroid track.  One-pole lowpass whose coefficient
    # moves at 7.7 Hz: crude, but it moves the centroid the way a filter does.
    swp = 7.7
    src = rng.standard_normal(n)
    cut = 0.5 + 0.45 * np.sin(2 * np.pi * swp * t)
    y = np.empty(n)
    acc = 0.0
    for i in range(n):
        acc += cut[i] * (src[i] - acc)
        y[i] = acc
    tr, rate = centroid_track(y, sr, *ANALYSIS)
    check("centroid: 7.7 Hz sweep", detect_mod(tr, rate, index_min=0.03), swp, 0.2)
    acc = 0.0
    for i in range(n):
        acc += 0.5 * (src[i] - acc)
        y[i] = acc
    tr, rate = centroid_track(y, sr, *ANALYSIS)
    check("centroid: fixed control", detect_mod(tr, rate, index_min=0.03), 0, 0, present=False)

    # 4. auto-pan -> pan track, and it must be blind to tremolo (4a/4b).
    pan = 6.2
    g = 0.5 * (1 + 0.8 * np.sin(2 * np.pi * pan * t))
    tr, rate = pan_track(car * g, car * (1 - g), sr, *ANALYSIS)
    check("pan: 6.2 Hz autopan", detect_mod(tr, rate, relative=False, index_min=0.03), pan, 0.15)
    tr, rate = pan_track(am, am, sr, *ANALYSIS)
    check("pan: tremolo is not pan", detect_mod(tr, rate, relative=False, index_min=0.03),
          0, 0, present=False)

    # 5. the trap this whole file exists to avoid: a DIFFERENT SOUND is not a
    # modulation.  Two unmodulated saws a semitone apart must both read flat.
    tr, rate = amp_track(saw(np.full(n, 233.08)), sr, *ANALYSIS)
    check("amp: wrong patch reads flat", detect_mod(tr, rate, index_min=0.03),
          0, 0, present=False)

    print("\nself-test:", "OK" if ok else "FAILED")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true",
                    help="validate the detectors on synthetic signals; no ROM needed")
    ap.add_argument("--audit", action="store_true",
                    help="check every LFO address against jeLib; no ROM needed")
    ap.add_argument("--rom", default=DEFAULT_ROM)
    ap.add_argument("--render", default=DEFAULT_RENDER)
    ap.add_argument("--out", default="/tmp/jp8000_lfo")
    ap.add_argument("--only", default="", help="comma-separated route names, or 'rate'")
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    if args.audit:
        return audit()
    if args.selftest:
        return selftest()

    if not os.path.isfile(args.render):
        print(f"no renderer at {args.render} -- build it:\n"
              f"  cmake -S . -B build-native -DCMAKE_BUILD_TYPE=Release && "
              f"cmake --build build-native --target jp8000_render -j", file=sys.stderr)
        return 2
    mids = [f for f in os.listdir(args.rom)] if os.path.isdir(args.rom) else []
    if len([f for f in mids if f.endswith(".mid")]) < 8:
        print(f"need the 8 JP-8000 v1.5 ROM .mid files in {args.rom}", file=sys.stderr)
        return 2

    rep = Report()
    run_engine_checks(args, rep)
    print(f"\n{len(rep.rows) - rep.failed}/{len(rep.rows)} checks passed"
          f"   (wavs in {args.out})")
    return 1 if rep.failed else 0


if __name__ == "__main__":
    sys.exit(main())

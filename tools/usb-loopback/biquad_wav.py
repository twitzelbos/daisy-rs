#!/usr/bin/env python3
"""Apply the daisy-dsp RBJ biquad low-pass to a WAV — the host "gold standard"
for the `dsp-loop` streaming DSP test.

Coefficients match `crates/daisy-dsp/src/filter.rs` (RBJ cookbook, normalised by
a0; Transposed Direct Form II, which is what scipy.signal.lfilter implements).
Defaults match `src/dsp_loop.rs` (1 kHz, Q 0.707).

Usage:  biquad_wav.py <in.wav> <out.wav> [freq_hz] [q]
Run via:  uv run --with numpy --with scipy python biquad_wav.py ...
"""
import sys, math, wave
import numpy as np
from scipy.signal import lfilter


def rbj_lowpass(fs, freq, q):
    w0 = 2 * math.pi * freq / fs
    cw, sw = math.cos(w0), math.sin(w0)
    alpha = sw / (2 * q)
    b = [(1 - cw) / 2, 1 - cw, (1 - cw) / 2]
    a = [1 + alpha, -2 * cw, 1 - alpha]
    a0 = a[0]
    return [x / a0 for x in b], [1.0, a[1] / a0, a[2] / a0]


def main(argv):
    if len(argv) < 2:
        print(__doc__); return 2
    inp, out = argv[0], argv[1]
    freq = float(argv[2]) if len(argv) > 2 else 1000.0
    q = float(argv[3]) if len(argv) > 3 else 0.707
    w = wave.open(inp, "rb")
    fs, n, ch = w.getframerate(), w.getnframes(), w.getnchannels()
    d = np.frombuffer(w.readframes(n), dtype="<i2").reshape(-1, ch).astype(np.float64) / 32768.0
    w.close()
    b, a = rbj_lowpass(fs, freq, q)
    y = np.clip(lfilter(b, a, d, axis=0), -1, 1) * 32767.0
    wo = wave.open(out, "wb")
    wo.setnchannels(ch); wo.setsampwidth(2); wo.setframerate(fs)
    wo.writeframes(y.astype("<i2").tobytes()); wo.close()
    print(f"wrote {out}: {n} frames, biquad LP {freq:.0f} Hz Q {q}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

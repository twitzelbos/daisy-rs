#!/usr/bin/env python3
"""Analyze USB-loopback captures from loopback.sh.

Modes:
  tone <rec.wav> [freq_hz]           detect a single tone (Goertzel; no deps)
  spectral <rec.wav> <expected.wav>  compare averaged magnitude spectra (needs numpy)
  compare <rec.wav> <expected.wav>   sample-accurate align + compare (needs numpy)

IMPORTANT — the USB *audio* path is spectrally faithful but NOT sample-accurate:
macOS independently sample-rate-converts each stream to reconcile the device
clock, so samples do not line up 1:1 (peak cross-correlation ~0.16 for noise)
even though the spectrum is preserved (a tone comes back clean). So over the
audio loopback use `spectral` (or `tone`); `compare` (sample-accurate) is for a
*reliable* transport (a future CDC/bulk block-exchange path), not the iso audio
loopback. This mirrors how audio DSP is validated in practice: by frequency
response / features, not bit-exact samples.

Use `spectral`/`tone` for hardware-in-the-loop DSP checks over the audio path
(expected == the golden signal run through the same core, or an analytic target).
"""
import sys, wave, struct, math


def read_wav_mono(path):
    w = wave.open(path, "rb")
    fs, n, ch, sw = w.getframerate(), w.getnframes(), w.getnchannels(), w.getsampwidth()
    raw = w.readframes(n); w.close()
    fmt = {1: "b", 2: "h", 4: "i"}[sw]
    a = struct.unpack("<" + fmt * (len(raw) // sw), raw)
    full = float(1 << (8 * sw - 1))
    mono = [a[i] / full for i in range(0, len(a), ch)]  # left channel, normalized to ±1
    return fs, mono


def goertzel(x, fs, f):
    k = 2 * math.cos(2 * math.pi * f / fs); s1 = s2 = 0.0
    for v in x:
        s0 = v + k * s1 - s2; s2 = s1; s1 = s0
    return math.sqrt(s1 * s1 + s2 * s2 - k * s1 * s2) / max(1, len(x))


def mode_tone(argv):
    path = argv[0]; freq = float(argv[1]) if len(argv) > 1 else 1000.0
    fs, x = read_wav_mono(path)
    # analyze the loudest 1.5 s window (skip capture-startup transient)
    win = int(1.5 * fs); best_i, best_e = 0, -1
    for i in range(0, max(1, len(x) - win), int(0.25 * fs)):
        e = sum(v * v for v in x[i:i + win])
        if e > best_e: best_e, best_i = e, i
    seg = x[best_i:best_i + win]
    rms = math.sqrt(sum(v * v for v in seg) / len(seg)) if seg else 0.0
    tone = goertzel(seg, fs, freq)
    noise = sum(goertzel(seg, fs, f) for f in (freq * 0.3, freq * 0.7, freq * 1.5, freq * 2.5)) / 4
    ratio = tone / (noise + 1e-12)
    print(f"  {len(x)/fs:.2f}s @ {fs} Hz")
    print(f"  loudest window @ {best_i/fs:.1f}s  RMS {20*math.log10(rms+1e-12):.1f} dBFS")
    print(f"  {freq:.0f} Hz Goertzel {tone:.4f}  vs off-tone {noise:.4f}  ratio {ratio:.0f}x")
    ok = ratio > 5 and rms > 10 ** (-40 / 20)
    print(f"  => {'TONE PRESENT' if ok else 'no clear tone'}")
    return 0 if ok else 1


def _welch_mag_db(x, np, fs, nfft=8192):
    # average magnitude spectrum over 50%-overlap Hann windows, in dB.
    # trims to the active region so leading/trailing silence doesn't skew it.
    e = np.convolve(x * x, np.ones(fs // 20) / (fs // 20), "same")
    active = np.where(e > e.max() * 0.05)[0]
    if len(active) > nfft:
        x = x[active[0]:active[-1]]
    win = np.hanning(nfft)
    acc = np.zeros(nfft // 2 + 1)
    k = 0
    for i in range(0, len(x) - nfft, nfft // 2):
        acc += np.abs(np.fft.rfft(x[i:i + nfft] * win))
        k += 1
    if k == 0:
        return None
    return 20 * np.log10(acc / k + 1e-9)


def mode_spectral(argv):
    try:
        import numpy as np
    except ImportError:
        print("spectral needs numpy: pip3 install numpy (or: uv run --with numpy python analyze.py ...)", file=sys.stderr); return 2
    fs_r, rec = read_wav_mono(argv[0]); fs_e, ref = read_wav_mono(argv[1])
    if fs_r != fs_e:
        print(f"sample-rate mismatch: {fs_r} vs {fs_e}", file=sys.stderr); return 2
    a = _welch_mag_db(np.asarray(rec), np, fs_r); b = _welch_mag_db(np.asarray(ref), np, fs_e)
    if a is None or b is None:
        print("not enough signal to compute a spectrum", file=sys.stderr); return 2
    freqs = np.fft.rfftfreq((len(a) - 1) * 2, 1.0 / fs_r)
    # Compare only where the reference has usable level: audible band AND within
    # 40 dB of the reference's peak. A filter's deep roll-off drops below the
    # loopback's capture noise floor, so comparing there is meaningless (and
    # would swamp the metric). This is what makes the check discriminating.
    band = (freqs >= 40) & (freqs <= 18000)
    band &= b >= (b[band].max() - 40.0)
    a, b = a[band], b[band]
    a = a - a.mean(); b = b - b.mean()  # remove overall level (device volume) — compare shape
    corr = float(np.corrcoef(a, b)[0, 1])
    lsd = float(np.sqrt(np.mean((a - b) ** 2)))  # log-spectral distance, dB RMS
    print(f"  spectral correlation {corr:.4f}   log-spectral distance {lsd:.1f} dB RMS (ref band, ref > -40 dB)")
    ok = corr > 0.95 and lsd < 3.0
    print(f"  => {'SPECTRAL MATCH — device reproduced the reference response' if ok else 'spectral mismatch (corr>0.95 & LSD<3 dB expected)'}")
    return 0 if ok else 1


def mode_compare(argv):
    try:
        import numpy as np
    except ImportError:
        print("compare needs numpy: pip3 install numpy (or run via uv)", file=sys.stderr); return 2
    rec_path, ref_path = argv[0], argv[1]
    fs_r, rec = read_wav_mono(rec_path)
    fs_e, ref = read_wav_mono(ref_path)
    if fs_r != fs_e:
        print(f"sample-rate mismatch: {fs_r} vs {fs_e}", file=sys.stderr); return 2
    rec = np.asarray(rec); ref = np.asarray(ref)
    # find latency: cross-correlate a mid slice of the reference against the capture
    L = min(len(ref), int(1.0 * fs_e))
    probe = ref[len(ref)//2 - L//2: len(ref)//2 + L//2]
    if probe.std() < 1e-6:
        print("reference is silent — pick a signal with energy", file=sys.stderr); return 2
    xc = np.correlate(rec - rec.mean(), probe - probe.mean(), mode="valid")
    lag = int(np.argmax(np.abs(xc))) - (len(ref)//2 - L//2)
    # align and clip to the common region
    if lag >= 0:
        a, b = rec[lag:], ref
    else:
        a, b = rec, ref[-lag:]
    m = min(len(a), len(b)); a, b = a[:m], b[:m]
    # best-fit amplitude scale (device attenuates by the macOS volume) + residual
    denom = float(np.dot(b, b)) or 1e-12
    scale = float(np.dot(a, b) / denom)
    resid = a - scale * b
    corr = float(np.corrcoef(a, b)[0, 1]) if a.std() > 0 and b.std() > 0 else 0.0
    sig = float(np.sqrt(np.mean((scale * b) ** 2)))
    err = float(np.sqrt(np.mean(resid ** 2)))
    snr = 20 * math.log10(sig / (err + 1e-12))
    print(f"  aligned lag {lag/fs_r*1000:+.1f} ms   samples compared {m} ({m/fs_r:.2f}s)")
    print(f"  amplitude scale {scale:.3f} ({20*math.log10(abs(scale)+1e-12):+.1f} dB)")
    print(f"  correlation {corr:.4f}   residual SNR {snr:.1f} dB")
    ok = corr > 0.99 and snr > 30
    print(f"  => {'MATCH — device reproduced the reference' if ok else 'MISMATCH (corr>0.99 & SNR>30 dB expected)'}")
    return 0 if ok else 1


def main(argv):
    if len(argv) < 2:
        print(__doc__); return 2
    mode = argv[0]
    if mode == "tone": return mode_tone(argv[1:])
    if mode == "spectral": return mode_spectral(argv[1:])
    if mode == "compare": return mode_compare(argv[1:])
    print(f"unknown mode '{mode}'", file=sys.stderr); print(__doc__); return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

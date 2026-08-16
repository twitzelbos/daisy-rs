"""Reference implementations — the *ideal* output each primitive should produce.

Computed in double precision (or exact integer math for the PRNG). These encode
the intended math independently of the Rust code; the Rust ``f32`` output is
compared against these within a per-case tolerance (tight ``max_abs`` for
non-recursive/exact paths, ``min_snr_db`` for recursive filters where f32
roundoff vs this f64 ideal is expected and bounded).
"""

from __future__ import annotations

import numpy as np
import scipy.signal as sig


def run(primitive: str, x: np.ndarray, sample_rate: float, params: dict) -> np.ndarray:
    fn = _ORACLES.get(primitive)
    if fn is None:
        raise ValueError(f"no oracle for primitive {primitive!r}")
    return np.asarray(fn(x, sample_rate, params), dtype=np.float32)


def _onepole(x, sr, p):
    a = np.exp(-2.0 * np.pi * float(p["cutoff"]) / sr)
    lp = sig.lfilter([1.0 - a], [1.0, -a], x.astype(np.float64))
    if p.get("mode", "lowpass") == "highpass":
        return x.astype(np.float64) - lp
    return lp


def _biquad(x, sr, p):
    freq = float(p["freq"])
    q = float(p["q"])
    gain = float(p.get("gain_db", 0.0))
    w0 = 2.0 * np.pi * freq / sr
    cw, sw = np.cos(w0), np.sin(w0)
    alpha = sw / (2.0 * q)
    A = 10.0 ** (gain / 40.0)  # sqrt of linear gain (peaking)
    kind = p["kind"]
    if kind == "lowpass":
        b = [(1 - cw) / 2, 1 - cw, (1 - cw) / 2]
        a = [1 + alpha, -2 * cw, 1 - alpha]
    elif kind == "highpass":
        b = [(1 + cw) / 2, -(1 + cw), (1 + cw) / 2]
        a = [1 + alpha, -2 * cw, 1 - alpha]
    elif kind == "bandpass":  # constant 0 dB peak gain
        b = [alpha, 0.0, -alpha]
        a = [1 + alpha, -2 * cw, 1 - alpha]
    elif kind == "peaking":
        b = [1 + alpha * A, -2 * cw, 1 - alpha * A]
        a = [1 + alpha / A, -2 * cw, 1 - alpha / A]
    else:
        raise ValueError(f"unknown biquad kind {kind!r}")
    b = np.array(b) / a[0]
    a = np.array(a) / a[0]
    return sig.lfilter(b, a, x.astype(np.float64))


def _delay(x, sr, p):
    """Ring-buffer read_frac semantics: y[n] = (1−f)·x[n−i] + f·x[n−i−1]."""
    d = float(p["delay"])
    i = int(np.floor(d))
    frac = d - i
    x = x.astype(np.float64)
    n = len(x)
    out = np.zeros(n, dtype=np.float64)
    for k in range(n):
        a = x[k - i] if k - i >= 0 else 0.0
        b = x[k - i - 1] if k - i - 1 >= 0 else 0.0
        out[k] = (1.0 - frac) * a + frac * b
    return out


def _prng(x, sr, p):
    """xorshift32 → [-1, 1) using the top 24 bits (bit-exact vs the Rust impl).

    Output length follows the (ignored) input length, so the case just needs a
    silence input of the right size.
    """
    seed = int(p["seed"]) & 0xFFFFFFFF
    if seed == 0:
        seed = 0x9E3779B9
    state = seed
    n = len(x)
    out = np.empty(n, dtype=np.float32)
    scale = np.float32(2.0) / np.float32(16_777_216.0)  # = 2^-23, exact in f32
    for k in range(n):
        state ^= (state << 13) & 0xFFFFFFFF
        state ^= state >> 17
        state ^= (state << 5) & 0xFFFFFFFF
        state &= 0xFFFFFFFF
        bits24 = state >> 8  # 0 .. 2^24 − 1
        out[k] = np.float32(bits24) * scale - np.float32(1.0)
    return out


def _synth_hrir(sr, p):
    """The synthetic HRIR pair — mirrored bit-for-intent by `synth_hrir` in
    crates/daisy-dsp/tests/golden.rs. A decaying tone prototype; the lag ear is
    that prototype delayed by `itd` samples and attenuated by `ild_db`."""
    ln = int(p["ir_len"])
    itd = int(p["itd"])
    tau = float(p["decay_tau"])
    tone = float(p["tone_hz"])
    scale = 10.0 ** (-float(p["ild_db"]) / 20.0)
    k = np.arange(ln, dtype=np.float64)
    lead = np.exp(-k / tau) * np.sin(2.0 * np.pi * tone * k / sr)
    lag = np.zeros(ln, dtype=np.float64)
    lag[itd:] = scale * lead[: ln - itd]
    return (lead, lag) if p.get("lead", "left") == "left" else (lag, lead)


def _stereo_convolve(x, sr, p):
    """Golden for the HRIR convolver's LEFT channel: causal linear convolution of
    the input with the left IR, truncated to the input length (the block-aligned
    UPOLS convolver adds no extra latency, so y[n] = Σ_k ir_l[k]·x[n−k])."""
    ir_l, _ = _synth_hrir(sr, p)
    y = sig.fftconvolve(x.astype(np.float64), ir_l)[: len(x)]
    return y


_ORACLES = {
    "onepole": _onepole,
    "biquad": _biquad,
    "delay": _delay,
    "prng": _prng,
    "stereo_convolve": _stereo_convolve,
}

"""Deterministic input-signal generators.

These produce the *input* a case feeds a primitive. They only need to be
reproducible run-to-run (so regenerating goldens is stable) — the Rust side
loads the generated input from disk, it does not regenerate it.
"""

from __future__ import annotations

import numpy as np


def generate(spec: dict, sample_rate: float) -> np.ndarray:
    """Build an input signal from a case's ``input`` table."""
    gen = spec["gen"]
    n = int(spec["len"])
    if gen == "impulse":
        x = np.zeros(n, dtype=np.float64)
        x[int(spec.get("pos", 0))] = float(spec.get("amp", 1.0))
        return x
    if gen == "silence":
        return np.zeros(n, dtype=np.float64)
    if gen == "dc":
        return np.full(n, float(spec.get("level", 1.0)), dtype=np.float64)
    if gen == "step":
        x = np.zeros(n, dtype=np.float64)
        x[int(spec.get("pos", 0)) :] = float(spec.get("level", 1.0))
        return x
    if gen == "sine":
        t = np.arange(n) / sample_rate
        return float(spec.get("amp", 1.0)) * np.sin(2 * np.pi * float(spec["freq"]) * t)
    if gen == "noise":
        rng = np.random.default_rng(int(spec.get("seed", 0)))
        return float(spec.get("amp", 1.0)) * rng.uniform(-1.0, 1.0, n)
    if gen == "log_sweep":
        f0 = float(spec["f0"])
        f1 = float(spec["f1"])
        import scipy.signal as sig

        t = np.arange(n) / sample_rate
        return float(spec.get("amp", 1.0)) * sig.chirp(
            t, f0=f0, f1=f1, t1=t[-1], method="logarithmic"
        )
    raise ValueError(f"unknown input generator: {gen!r}")

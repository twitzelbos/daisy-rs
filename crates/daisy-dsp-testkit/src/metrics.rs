//! Signal metrics for property-based checks (host, `f64` internally).
//!
//! These validate effects whose exact samples aren't a meaningful golden — a
//! reverb's dense feedback makes f32-vs-f64 tails diverge even when both are
//! "correct", so we assert *properties* (decay time, stability) instead.

/// RT60 (seconds) via Schroeder backward energy integration: the time for the
/// energy-decay curve to fall 60 dB from its start. `None` if the response
/// never decays that far (tail too short) or carries no energy.
pub fn rt60(impulse_response: &[f32], sample_rate: f32) -> Option<f32> {
    let n = impulse_response.len();
    if n == 0 {
        return None;
    }
    // Energy decay curve: EDC[i] = Σ_{k≥i} h[k]² (reverse cumulative energy).
    let mut edc = vec![0.0f64; n];
    let mut acc = 0.0f64;
    for i in (0..n).rev() {
        let s = impulse_response[i] as f64;
        acc += s * s;
        edc[i] = acc;
    }
    let total = edc[0];
    if total <= 0.0 {
        return None;
    }
    for (i, &e) in edc.iter().enumerate() {
        if 10.0 * (e / total).log10() <= -60.0 {
            return Some(i as f32 / sample_rate);
        }
    }
    None
}

/// Peak absolute value.
pub fn peak(x: &[f32]) -> f32 {
    x.iter().fold(0.0f32, |m, &v| m.max(v.abs()))
}

/// True if every sample is finite (no NaN/Inf).
pub fn all_finite(x: &[f32]) -> bool {
    x.iter().all(|v| v.is_finite())
}

/// Split an interleaved stereo buffer (`l0, r0, l1, r1, …`) into `(L, R)`.
pub fn deinterleave(lr: &[f32]) -> (Vec<f32>, Vec<f32>) {
    let mut l = Vec::with_capacity(lr.len() / 2);
    let mut r = Vec::with_capacity(lr.len() / 2);
    for pair in lr.chunks_exact(2) {
        l.push(pair[0]);
        r.push(pair[1]);
    }
    (l, r)
}

/// Interaural time difference: the integer lag `d` in `[−max_lag, max_lag]` that
/// maximizes the cross-correlation Σ l[n]·r[n+d]. Positive → L leads (R delayed
/// by `d`), the sign convention a listener hears as "toward the left ear".
pub fn itd_lag(l: &[f32], r: &[f32], max_lag: i32) -> i32 {
    let n = l.len().min(r.len()) as i32;
    let mut best_d = 0i32;
    let mut best = f64::NEG_INFINITY;
    for d in -max_lag..=max_lag {
        let (start, end) = (0.max(-d), n.min(n - d));
        let mut acc = 0.0f64;
        let mut i = start;
        while i < end {
            acc += l[i as usize] as f64 * r[(i + d) as usize] as f64;
            i += 1;
        }
        if acc > best {
            best = acc;
            best_d = d;
        }
    }
    best_d
}

/// Interaural level difference in dB: 20·log10(rms L / rms R). Positive → L is
/// louder. A near-silent R is floored so the ratio stays finite.
pub fn ild_db(l: &[f32], r: &[f32]) -> f32 {
    let rms = |x: &[f32]| -> f64 {
        let s: f64 = x.iter().map(|&v| v as f64 * v as f64).sum();
        (s / x.len().max(1) as f64).sqrt()
    };
    (20.0 * (rms(l) / rms(r).max(1e-20)).log10()) as f32
}

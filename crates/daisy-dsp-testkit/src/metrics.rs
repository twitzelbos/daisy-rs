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

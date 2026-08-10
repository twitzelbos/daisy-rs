//! YIN fundamental-frequency (pitch) detection. `no_std`, no alloc — the
//! difference-function scratch is borrowed from the caller.
//!
//! Detects the pitch of a (quasi-)periodic buffer for synth-follow, tuning, and
//! auto-octave. Robust on musical signals where plain autocorrelation picks
//! octave errors.
//!
//! # Reference
//! - A. de Cheveigné & H. Kawahara, "YIN, a fundamental frequency estimator for
//!   speech and music," *J. Acoust. Soc. Am.* 111(4), 2002.

/// Estimate the fundamental of `samples` (Hz), or `None` if the buffer is not
/// convincingly periodic (aperiodic / silent).
///
/// - `threshold`: YIN clarity threshold (0.10–0.20 typical; lower = stricter).
/// - `cmndf`: scratch of length `samples.len() / 2` (the max lag). It receives
///   the cumulative-mean-normalised difference function.
///
/// Returns `(frequency_hz, clarity)` where clarity = `1 − d'(τ)` at the chosen
/// lag (≈ how periodic the signal is, in `[0, 1]`).
///
/// # Panics
/// Panics if `samples.len()` is less than `2 · cmndf.len()`.
#[must_use]
pub fn detect(
    samples: &[f32],
    sample_rate: f32,
    threshold: f32,
    cmndf: &mut [f32],
) -> Option<(f32, f32)> {
    let w = cmndf.len();
    assert!(samples.len() >= 2 * w, "need >= 2·cmndf.len() samples");
    if w < 2 {
        return None;
    }

    // 1) Difference function d(τ) = Σ_j (x[j] − x[j+τ])², and simultaneously the
    //    cumulative-mean normalisation (2). d'(0) = 1.
    cmndf[0] = 1.0;
    let mut running = 0.0f32;
    for tau in 1..w {
        let mut d = 0.0f32;
        for j in 0..w {
            let diff = samples[j] - samples[j + tau];
            d += diff * diff;
        }
        running += d;
        // d'(τ) = d(τ) / ((1/τ) Σ_{k=1..τ} d(k))
        cmndf[tau] = if running > 0.0 {
            d * tau as f32 / running
        } else {
            1.0
        };
    }

    // 3) Absolute threshold: first local minimum of d' that dips below threshold.
    let mut tau = 0usize;
    let mut t = 2usize;
    while t < w - 1 {
        if cmndf[t] < threshold {
            // descend to the local minimum
            while t + 1 < w && cmndf[t + 1] < cmndf[t] {
                t += 1;
            }
            tau = t;
            break;
        }
        t += 1;
    }
    // Fall back to the global minimum if nothing crossed the threshold.
    if tau == 0 {
        let mut best = 1;
        for k in 2..w {
            if cmndf[k] < cmndf[best] {
                best = k;
            }
        }
        // Too aperiodic to call a pitch.
        if cmndf[best] > 0.3 {
            return None;
        }
        tau = best;
    }

    // 4) Parabolic interpolation around τ for sub-sample precision.
    let x0 = cmndf[tau - 1];
    let x1 = cmndf[tau];
    let x2 = cmndf[(tau + 1).min(w - 1)];
    let denom = x0 + x2 - 2.0 * x1;
    let shift = if denom.abs() > 1.0e-9 {
        0.5 * (x0 - x2) / denom
    } else {
        0.0
    };
    let period = tau as f32 + shift.clamp(-1.0, 1.0);

    let clarity = 1.0 - x1;
    Some((sample_rate / period, clarity.clamp(0.0, 1.0)))
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::TAU;
    use std::vec::Vec;

    const FS: f32 = 48_000.0;

    fn tone(freq: f32, harmonics: usize, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| {
                let t = i as f32 / FS;
                let mut s = 0.0;
                for h in 1..=harmonics {
                    s += (1.0 / h as f32) * libm::sinf(TAU * freq * h as f32 * t);
                }
                s * 0.3
            })
            .collect()
    }

    #[test]
    fn detects_musical_pitches_accurately() {
        // Rich (harmonic) tones across the guitar/vocal range.
        for &f in &[82.41f32, 146.83, 220.0, 440.0, 880.0] {
            let sig = tone(f, 6, 4096);
            let mut cmndf = std::vec![0.0f32; 2048];
            let (est, clarity) = detect(&sig, FS, 0.15, &mut cmndf).expect("no pitch");
            let cents = 1200.0 * libm::log2f(est / f);
            assert!(
                cents.abs() < 15.0,
                "f={f}: est={est} ({cents:.1} cents), clarity={clarity}"
            );
            assert!(clarity > 0.8, "f={f}: low clarity {clarity}");
        }
    }

    #[test]
    fn rejects_noise() {
        let mut seed = 0x1234_5678u32;
        let sig: Vec<f32> = (0..4096)
            .map(|_| {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                (seed >> 8) as f32 / (1 << 24) as f32 * 2.0 - 1.0
            })
            .collect();
        let mut cmndf = std::vec![0.0f32; 2048];
        // White noise is aperiodic → no confident pitch.
        assert!(detect(&sig, FS, 0.15, &mut cmndf).is_none());
    }
}

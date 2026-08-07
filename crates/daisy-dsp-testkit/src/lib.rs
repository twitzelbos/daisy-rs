//! Golden-vector test framework for `daisy-dsp` (host side).
//!
//! The model: a single **`cases.toml`** is the source of truth. A uv-managed
//! Python oracle (`tools/dsp-golden`) reads it and, using scipy, writes an
//! `<name>.in.f32` input and an `<name>.out.f32` *reference* output per case.
//! The Rust side loads the input, runs the real `daisy-dsp` primitive, and
//! compares its `f32` output to the reference within a per-case tolerance —
//! each case is its own test (via `libtest-mimic`), so `cargo test <name>`
//! filters and failures are isolated.
//!
//! The crate is generic: it knows nothing about specific primitives. The caller
//! supplies a *runner* closure that maps `(Case, input) → output` — that's the
//! one place primitives are wired to their manifest names.

use std::fs;
use std::path::Path;
use std::sync::Arc;

use libtest_mimic::{Arguments, Failed, Trial};
use serde::Deserialize;

/// Acceptance tolerance for a case. At least one bound must be set; every set
/// bound must hold.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct Tolerance {
    /// Maximum absolute per-sample error. For exact / FIR / non-recursive paths.
    #[serde(rename = "max_abs_error", default)]
    pub max_abs: Option<f32>,
    /// Minimum signal-to-error ratio in dB. For recursive filters, where f32
    /// roundoff vs the f64 oracle is expected and bounded.
    #[serde(rename = "min_snr_db", default)]
    pub min_snr_db: Option<f32>,
}

/// A bag of case parameters (the TOML `params` table), with typed accessors.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(transparent)]
pub struct Params(toml::Table);

impl Params {
    fn num(&self, key: &str) -> Option<f32> {
        self.0.get(key).and_then(|v| {
            v.as_float()
                .map(|x| x as f32)
                .or_else(|| v.as_integer().map(|i| i as f32))
        })
    }
    /// Required numeric parameter as `f32` (accepts TOML int or float).
    pub fn f32(&self, key: &str) -> f32 {
        self.num(key)
            .unwrap_or_else(|| panic!("param '{key}' missing or not a number"))
    }
    /// Numeric parameter with a default.
    pub fn f32_or(&self, key: &str, default: f32) -> f32 {
        self.num(key).unwrap_or(default)
    }
    /// Required string parameter.
    pub fn str(&self, key: &str) -> &str {
        self.0
            .get(key)
            .and_then(|v| v.as_str())
            .unwrap_or_else(|| panic!("param '{key}' missing or not a string"))
    }
    /// Required integer parameter as `u32`.
    pub fn u32(&self, key: &str) -> u32 {
        self.0
            .get(key)
            .and_then(|v| v.as_integer())
            .map(|i| i as u32)
            .unwrap_or_else(|| panic!("param '{key}' missing or not an integer"))
    }
    /// Required integer parameter as `usize`.
    pub fn usize(&self, key: &str) -> usize {
        self.u32(key) as usize
    }
}

/// One test case, resolved (sample rate defaulted from the manifest).
#[derive(Debug, Clone)]
pub struct Case {
    pub name: String,
    pub primitive: String,
    pub sample_rate: f32,
    pub params: Params,
    pub tolerance: Tolerance,
}

#[derive(Deserialize)]
struct Manifest {
    sample_rate: f32,
    #[serde(rename = "case", default)]
    cases: Vec<RawCase>,
}

#[derive(Deserialize)]
struct RawCase {
    name: String,
    primitive: String,
    sample_rate: Option<f32>,
    #[serde(default)]
    params: Params,
    tolerance: Tolerance,
    // `input = { ... }` and any other keys are consumed by the Python oracle and
    // ignored here.
    #[serde(flatten)]
    _rest: toml::Table,
}

/// Parse `cases.toml`, applying the manifest's default sample rate.
pub fn load_cases(path: &Path) -> Result<Vec<Case>, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("read {}: {e}", path.display()))?;
    let m: Manifest =
        toml::from_str(&text).map_err(|e| format!("parse {}: {e}", path.display()))?;
    Ok(m.cases
        .into_iter()
        .map(|r| Case {
            sample_rate: r.sample_rate.unwrap_or(m.sample_rate),
            name: r.name,
            primitive: r.primitive,
            params: r.params,
            tolerance: r.tolerance,
        })
        .collect())
}

/// Read a raw little-endian `f32` blob.
pub fn read_f32(path: &Path) -> Result<Vec<f32>, String> {
    let bytes = fs::read(path).map_err(|e| format!("read {}: {e}", path.display()))?;
    if bytes.len() % 4 != 0 {
        return Err(format!("{}: length not a multiple of 4", path.display()));
    }
    Ok(bytes
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect())
}

/// Compare an actual output against the golden within `tol`. `Ok(())` on pass;
/// `Err(msg)` with the measured metrics on failure.
pub fn compare(actual: &[f32], expected: &[f32], tol: &Tolerance) -> Result<(), String> {
    if actual.len() != expected.len() {
        return Err(format!(
            "length mismatch: got {}, expected {}",
            actual.len(),
            expected.len()
        ));
    }
    let mut max_abs = 0.0f32;
    let mut sig_sq = 0.0f64;
    let mut err_sq = 0.0f64;
    for (&a, &e) in actual.iter().zip(expected) {
        let d = (a - e).abs();
        if d > max_abs {
            max_abs = d;
        }
        sig_sq += (e as f64) * (e as f64);
        err_sq += (d as f64) * (d as f64);
    }
    let snr_db = if err_sq > 0.0 {
        10.0 * (sig_sq / err_sq).log10()
    } else {
        f64::INFINITY
    };

    if tol.max_abs.is_none() && tol.min_snr_db.is_none() {
        return Err("case has no tolerance (set max_abs_error and/or min_snr_db)".into());
    }
    if let Some(m) = tol.max_abs {
        if max_abs > m {
            return Err(format!(
                "max_abs {max_abs:.3e} > {m:.3e}  (snr {snr_db:.1} dB)"
            ));
        }
    }
    if let Some(s) = tol.min_snr_db {
        if (snr_db as f32) < s {
            return Err(format!(
                "snr {snr_db:.1} dB < {s} dB  (max_abs {max_abs:.3e})"
            ));
        }
    }
    Ok(())
}

/// Run every case in `cases_path` as its own libtest-mimic test, loading its
/// goldens from `golden_dir` and processing the input through `runner`. Parses
/// `cargo test` args (filters, `--nocapture`, …) and **exits the process** with
/// the aggregate result — call it from a `harness = false` test `main`.
pub fn run_golden<R>(cases_path: &Path, golden_dir: &Path, runner: R) -> !
where
    R: Fn(&Case, &[f32]) -> Vec<f32> + Send + Sync + 'static,
{
    let args = Arguments::from_args();
    let cases = load_cases(cases_path).unwrap_or_else(|e| panic!("{e}"));
    let runner = Arc::new(runner);
    let golden_dir = golden_dir.to_path_buf();

    let trials = cases
        .into_iter()
        .map(|case| {
            let runner = runner.clone();
            let dir = golden_dir.clone();
            Trial::test(case.name.clone(), move || {
                run_one(&case, &dir, runner.as_ref())
            })
        })
        .collect();

    libtest_mimic::run(&args, trials).exit();
}

fn run_one(
    case: &Case,
    dir: &Path,
    runner: &(impl Fn(&Case, &[f32]) -> Vec<f32> + ?Sized),
) -> Result<(), Failed> {
    let input = read_f32(&dir.join(format!("{}.in.f32", case.name)))?;
    let expected = read_f32(&dir.join(format!("{}.out.f32", case.name)))?;
    let actual = runner(case, &input);
    compare(&actual, &expected, &case.tolerance)?;
    Ok(())
}

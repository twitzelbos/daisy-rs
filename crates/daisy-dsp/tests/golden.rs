//! Data-driven golden tests for `daisy-dsp` primitives.
//!
//! Each `[[case]]` in `tests/cases.toml` becomes one test (via the testkit's
//! `libtest-mimic` harness), so `cargo test --test golden -- biquad` filters and
//! failures are isolated. The Python oracle (`tools/dsp-golden`) produces the
//! reference `<name>.out.f32`; this file maps a case's `primitive`/`params` to
//! the real Rust primitive — the one place primitives are wired to their names.
//!
//! Run (host target — the workspace defaults to the embedded target):
//! ```text
//! uv --directory tools/dsp-golden run dsp-golden        # (re)generate goldens
//! cargo test -p daisy-dsp --target "$(rustc -vV | sed -n 's/host: //p')" --test golden
//! ```

use std::path::Path;

use daisy_dsp::delay::DelayLine;
use daisy_dsp::filter::{Biquad, OnePole};
use daisy_dsp::granular::Granular;
use daisy_dsp::noise::Prng;
use daisy_dsp::reverb::FdnReverb;
use daisy_dsp_testkit::{run_golden, Case};

/// Map one case to its primitive and run it over `input`.
fn run_case(case: &Case, input: &[f32]) -> Vec<f32> {
    let sr = case.sample_rate;
    let p = &case.params;
    match case.primitive.as_str() {
        "onepole" => {
            let mut f = match p.str("mode") {
                "lowpass" => OnePole::lowpass(sr, p.f32("cutoff")),
                "highpass" => OnePole::highpass(sr, p.f32("cutoff")),
                other => panic!("onepole: unknown mode {other:?}"),
            };
            input.iter().map(|&x| f.tick(x)).collect()
        }
        "biquad" => {
            let (freq, q) = (p.f32("freq"), p.f32("q"));
            let mut f = match p.str("kind") {
                "lowpass" => Biquad::lowpass(sr, freq, q),
                "highpass" => Biquad::highpass(sr, freq, q),
                "bandpass" => Biquad::bandpass(sr, freq, q),
                "peaking" => Biquad::peaking(sr, freq, q, p.f32("gain_db")),
                other => panic!("biquad: unknown kind {other:?}"),
            };
            input.iter().map(|&x| f.tick(x)).collect()
        }
        "delay" => {
            let mut buf = vec![0.0f32; p.usize("buf_len")];
            let mut dl = DelayLine::new(&mut buf);
            let delay = p.f32("delay");
            input
                .iter()
                .map(|&x| {
                    dl.write(x);
                    dl.read_frac(delay)
                })
                .collect()
        }
        "prng" => {
            let mut rng = Prng::new(p.u32("seed"));
            (0..input.len()).map(|_| rng.next_f32()).collect()
        }
        "fdn_reverb" => {
            let mut buf = vec![0.0f32; p.usize("buf_len")];
            let mut rv = FdnReverb::new(
                &mut buf,
                sr,
                p.f32("rt60_s"),
                p.f32("damping_hz"),
                p.f32("mix"),
            );
            let mut l = vec![0.0f32; input.len()];
            let mut r = vec![0.0f32; input.len()];
            rv.process(input, &mut l, &mut r);
            l // property checks (RT60/finite/peak) run on the left channel
        }
        "granular" => {
            let mut buf = vec![0.0f32; p.usize("buf_len")];
            let mut g = Granular::new(&mut buf, sr, p.u32("seed"));
            let mut l = vec![0.0f32; input.len()];
            let mut r = vec![0.0f32; input.len()];
            g.process(input, &mut l, &mut r);
            l // property checks (finite/peak) run on the left channel
        }
        other => panic!("unknown primitive {other:?}"),
    }
}

fn main() -> ! {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    run_golden(
        &root.join("tests/cases.toml"),
        &root.join("tests/golden"),
        run_case,
    )
}

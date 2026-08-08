//! Render the daisy-dsp choir/pad engines to WAV on the host, so the sound can
//! be auditioned on exactly the code that runs on the Daisy.
//!
//! ```text
//! cargo run -p pad-audition --target <host> -- [outdir]
//! ```
//! Writes `choir_{E,D,A}.wav` and `canon_in_d.wav` into `outdir`
//! (default `./audition-out`). Play them with `afplay`/`aplay`/etc.

use std::path::PathBuf;

use daisy_dsp::choir::ChoirVoice;
use daisy_dsp::reverb::FdnReverb;
use daisy_dsp::sampler::SamplePad;

const SR: f32 = 48_000.0;

// --- Note frequencies (Hz) ---------------------------------------------------
const D2: f32 = 73.42;
const E2: f32 = 82.41;
const FS2: f32 = 92.50;
const G2: f32 = 98.00;
const A2: f32 = 110.00;
const B2: f32 = 123.47;
const CS3: f32 = 138.59;
const D3: f32 = 146.83;
const E3: f32 = 164.81;
const FS3: f32 = 185.00;
const G3: f32 = 196.00;
const GS3: f32 = 207.65;
const A3: f32 = 220.00;
const B3: f32 = 246.94;
const CS4: f32 = 277.18;
const D4: f32 = 293.66;
const E4: f32 = 329.63;
const FS4: f32 = 369.99;

fn write_wav(path: &PathBuf, l: &[f32], r: &[f32]) {
    let n = l.len();
    let peak = l.iter().chain(r).fold(1e-9f32, |m, &x| m.max(x.abs()));
    let g = 0.9 / peak;
    let data_len = (n * 2 * 2) as u32;
    let mut b: Vec<u8> = Vec::with_capacity(44 + data_len as usize);
    b.extend_from_slice(b"RIFF");
    b.extend_from_slice(&(36 + data_len).to_le_bytes());
    b.extend_from_slice(b"WAVE");
    b.extend_from_slice(b"fmt ");
    b.extend_from_slice(&16u32.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&2u16.to_le_bytes());
    b.extend_from_slice(&(SR as u32).to_le_bytes());
    b.extend_from_slice(&((SR as u32) * 4).to_le_bytes());
    b.extend_from_slice(&4u16.to_le_bytes());
    b.extend_from_slice(&16u16.to_le_bytes());
    b.extend_from_slice(b"data");
    b.extend_from_slice(&data_len.to_le_bytes());
    for i in 0..n {
        let s = |v: f32| (v * g * 32767.0).clamp(-32768.0, 32767.0) as i16;
        b.extend_from_slice(&s(l[i]).to_le_bytes());
        b.extend_from_slice(&s(r[i]).to_le_bytes());
    }
    std::fs::write(path, b).unwrap();
}

/// One choir swell (dry stereo) of `len` samples, raised-cosine attack/release.
fn choir_swell(notes: &[f32], seed: u32, len: usize, atk: f32, rel: f32) -> (Vec<f32>, Vec<f32>) {
    let mut voice = Box::new(ChoirVoice::new(SR, seed));
    voice.set_voices_per_note(3);
    voice.set_chord(notes);
    let (mut l, mut r) = (vec![0.0f32; len], vec![0.0f32; len]);
    let mut i = 0;
    while i < len {
        let n = 64.min(len - i);
        voice.process(&mut l[i..i + n], &mut r[i..i + n]);
        i += n;
    }
    let (a, re) = ((atk * SR) as usize, (rel * SR) as usize);
    for k in 0..a.min(len) {
        let e = 0.5 - 0.5 * (std::f32::consts::PI * k as f32 / a as f32).cos();
        l[k] *= e;
        r[k] *= e;
    }
    for k in 0..re.min(len) {
        let e = 0.5 + 0.5 * (std::f32::consts::PI * k as f32 / re as f32).cos();
        l[len - 1 - k] *= e;
        r[len - 1 - k] *= e;
    }
    (l, r)
}

/// Sum to mono → FdnReverb (stereo out) → mix wet with the dry stereo.
fn reverberate(
    dl: &[f32],
    dr: &[f32],
    rt60: f32,
    damp: f32,
    dry: f32,
    wet: f32,
) -> (Vec<f32>, Vec<f32>) {
    let n = dl.len();
    let mut buf = vec![0.0f32; FdnReverb::REQUIRED_BUF];
    let mut rv = FdnReverb::new(&mut buf, SR, rt60, damp, 1.0);
    let mono: Vec<f32> = (0..n).map(|i| 0.5 * (dl[i] + dr[i])).collect();
    let (mut wl, mut wr) = (vec![0.0f32; n], vec![0.0f32; n]);
    let mut i = 0;
    while i < n {
        let b = 64.min(n - i);
        rv.process(&mono[i..i + b], &mut wl[i..i + b], &mut wr[i..i + b]);
        i += b;
    }
    let ol = (0..n).map(|i| dry * dl[i] + wet * wl[i]).collect();
    let or = (0..n).map(|i| dry * dr[i] + wet * wr[i]).collect();
    (ol, or)
}

/// Read a 16-bit PCM WAV → (mono samples, sample_rate). Scans chunks so it
/// tolerates real files with LIST/etc. before `data`.
fn read_wav(path: &str) -> (Vec<f32>, f32) {
    let b = std::fs::read(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let (mut ch, mut rate, mut bits) = (1u16, 48000u32, 16u16);
    let mut data: &[u8] = &[];
    let u16le = |s: &[u8]| u16::from_le_bytes([s[0], s[1]]);
    let u32le = |s: &[u8]| u32::from_le_bytes([s[0], s[1], s[2], s[3]]);
    let mut p = 12;
    while p + 8 <= b.len() {
        let id = &b[p..p + 4];
        let sz = u32le(&b[p + 4..p + 8]) as usize;
        p += 8;
        let end = (p + sz).min(b.len());
        if id == b"fmt " && end >= p + 16 {
            ch = u16le(&b[p + 2..]);
            rate = u32le(&b[p + 4..]);
            bits = u16le(&b[p + 14..]);
        } else if id == b"data" {
            data = &b[p..end];
        }
        p = end + (sz & 1);
    }
    assert_eq!(bits, 16, "only 16-bit PCM WAV supported (got {bits})");
    let step = 2 * ch as usize;
    let mono: Vec<f32> = data
        .chunks_exact(step)
        .map(|c| {
            let mut acc = 0.0f32;
            for k in 0..ch as usize {
                acc += i16::from_le_bytes([c[2 * k], c[2 * k + 1]]) as f32 / 32768.0;
            }
            acc / ch as f32
        })
        .collect();
    (mono, rate as f32)
}

/// One sample-pad swell (dry stereo), raised-cosine attack/release.
fn sample_swell(
    sample: &[f32],
    samp_rate: f32,
    base: f32,
    notes: &[f32],
    seed: u32,
    len: usize,
    atk: f32,
    rel: f32,
) -> (Vec<f32>, Vec<f32>) {
    let mut pad = SamplePad::new(sample, samp_rate, base, SR, 4800, seed);
    // Skip ~50 ms of edges from the loop.
    let n = sample.len();
    pad.set_loop(
        (samp_rate * 0.05) as usize,
        n.saturating_sub((samp_rate * 0.1) as usize),
    );
    pad.set_voices_per_note(3);
    pad.set_chord(notes);
    let (mut l, mut r) = (vec![0.0f32; len], vec![0.0f32; len]);
    let mut i = 0;
    while i < len {
        let n = 64.min(len - i);
        pad.process(&mut l[i..i + n], &mut r[i..i + n]);
        i += n;
    }
    let (a, re) = ((atk * SR) as usize, (rel * SR) as usize);
    for k in 0..a.min(len) {
        let e = 0.5 - 0.5 * (std::f32::consts::PI * k as f32 / a as f32).cos();
        l[k] *= e;
        r[k] *= e;
    }
    for k in 0..re.min(len) {
        let e = 0.5 + 0.5 * (std::f32::consts::PI * k as f32 / re as f32).cos();
        l[len - 1 - k] *= e;
        r[len - 1 - k] *= e;
    }
    (l, r)
}

fn main() {
    let outdir = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("audition-out"));
    std::fs::create_dir_all(&outdir).unwrap();

    // Sample mode: `pad-audition <outdir> <sample.wav> [base_freq]` renders the
    // choir chords + canon by pitching/looping a real recording (Atmosphere-style).
    if let Some(sample_path) = std::env::args().nth(2) {
        let base = std::env::args()
            .nth(3)
            .and_then(|s| s.parse::<f32>().ok())
            .unwrap_or(220.0);
        let (sample, samp_rate) = read_wav(&sample_path);
        println!(
            "sample: {} ({:.1}s @ {:.0}Hz, base {base}Hz)",
            sample_path,
            sample.len() as f32 / samp_rate,
            samp_rate
        );
        let e_maj = [E2, B2, E3, GS3, B3, E4];
        let d_maj = [D2, D3, A3, D4, FS4];
        let a_maj = [A2, E3, A3, CS4, E4];
        let b_min = [B2, D3, FS3, B3, D4];
        let fs_min = [FS2, CS3, FS3, A3, CS4];
        let g_maj = [G2, D3, G3, B3, D4];
        for (i, (name, notes)) in [
            ("sample_E", &e_maj[..]),
            ("sample_D", &d_maj[..]),
            ("sample_A", &a_maj[..]),
        ]
        .iter()
        .enumerate()
        {
            let (dl, dr) = sample_swell(
                &sample,
                samp_rate,
                base,
                notes,
                1 + i as u32,
                (SR * 12.0) as usize,
                2.5,
                2.5,
            );
            let (l, r) = reverberate(&dl, &dr, 6.0, 6000.0, 0.5, 0.9);
            let p = outdir.join(format!("{name}.wav"));
            write_wav(&p, &l, &r);
            println!("{}", p.display());
        }
        let prog: [&[f32]; 8] = [
            &d_maj, &a_maj, &b_min, &fs_min, &g_maj, &d_maj, &g_maj, &a_maj,
        ];
        let step = (SR * 2.4) as usize;
        let seg = (SR * 3.4) as usize;
        let total = step * (prog.len() * 2) + seg;
        let (mut ml, mut mr) = (vec![0.0f32; total], vec![0.0f32; total]);
        for loop_i in 0..2 {
            for (ci, ch) in prog.iter().enumerate() {
                let idx = loop_i * prog.len() + ci;
                let (l, r) = sample_swell(
                    &sample,
                    samp_rate,
                    base,
                    ch,
                    100 + idx as u32,
                    seg,
                    0.5,
                    1.2,
                );
                let s0 = idx * step;
                for k in 0..seg {
                    ml[s0 + k] += l[k];
                    mr[s0 + k] += r[k];
                }
            }
        }
        let (l, r) = reverberate(&ml, &mr, 6.0, 6000.0, 0.5, 0.85);
        let p = outdir.join("sample_canon.wav");
        write_wav(&p, &l, &r);
        println!("{}  ({:.0}s)", p.display(), total as f32 / SR);
        return;
    }

    // Chord voicings (open, guitar-ish).
    let e_maj = [E2, B2, E3, GS3, B3, E4];
    let d_maj = [D2, D3, A3, D4, FS4];
    let a_maj = [A2, E3, A3, CS4, E4];
    let b_min = [B2, D3, FS3, B3, D4];
    let fs_min = [FS2, CS3, FS3, A3, CS4];
    let g_maj = [G2, D3, G3, B3, D4];

    // Single choir chords.
    for (i, (name, notes)) in [
        ("choir_E", &e_maj[..]),
        ("choir_D", &d_maj[..]),
        ("choir_A", &a_maj[..]),
    ]
    .iter()
    .enumerate()
    {
        let (dl, dr) = choir_swell(notes, 1 + i as u32, (SR * 12.0) as usize, 2.5, 2.5);
        let (l, r) = reverberate(&dl, &dr, 6.0, 6000.0, 0.5, 0.9);
        let p = outdir.join(format!("{name}.wav"));
        write_wav(&p, &l, &r);
        println!("{}", p.display());
    }

    // Pachelbel's Canon in D — the ground progression, twice, as a choir pad.
    let prog: [&[f32]; 8] = [
        &d_maj, &a_maj, &b_min, &fs_min, &g_maj, &d_maj, &g_maj, &a_maj,
    ];
    let step = (SR * 2.4) as usize;
    let seg = (SR * 3.4) as usize;
    let total = step * (prog.len() * 2) + seg;
    let (mut ml, mut mr) = (vec![0.0f32; total], vec![0.0f32; total]);
    for loop_i in 0..2 {
        for (ci, ch) in prog.iter().enumerate() {
            let idx = loop_i * prog.len() + ci;
            let (l, r) = choir_swell(ch, 100 + idx as u32, seg, 0.5, 1.2);
            let s0 = idx * step;
            for k in 0..seg {
                ml[s0 + k] += l[k];
                mr[s0 + k] += r[k];
            }
        }
    }
    let (l, r) = reverberate(&ml, &mr, 6.0, 6000.0, 0.5, 0.85);
    let p = outdir.join("canon_in_d.wav");
    write_wav(&p, &l, &r);
    println!("{}  ({:.0}s)", p.display(), total as f32 / SR);
}

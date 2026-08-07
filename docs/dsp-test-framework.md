# DSP Test Framework — write once, validate three ways

A three-tier test framework for the DSP algorithms in `daisy-dsp`. Every
algorithm is written **once** (as a plain `no_std` processor) and validated at
three increasing levels of realism, all against **the same golden references**:

| Tier | Where | What it proves | Speed |
|------|-------|----------------|-------|
| **A — host** | `cargo test` on the dev machine | the algorithm's *math* is correct | ms |
| **B — Renode** | firmware in the simulator, in CI | it *compiles + runs on the M7 target* and integrates with the SAI/DMA audio path, without panicking | seconds |
| **C — hardware** | real Daisy over SWD, loopback rig | it computes correctly **and fits in the real-time budget** on real silicon, and the **codec + analog path** work end-to-end | seconds/test, batched |

Tier A is the **source of truth**: it produces the golden output. B and C are
validated against those goldens, so "the hardware is correct" means "the
hardware reproduces the reference math within tolerance."

The headline goal for tier C: **flash once, then batch tens of algorithms over
SWD/RTT with zero button presses** — see §6.

---

## 1. The DSP contract

Every testable algorithm implements one trait, in `daisy-dsp`. It is the *only*
thing that has to be written per algorithm; all three tiers call it.

```rust
// daisy-dsp/src/lib.rs  (no_std, no alloc)
pub const MAX_BLOCK: usize = 64;

/// A block-based, allocation-free, deterministic audio processor.
pub trait DspProcessor {
    /// Construct from a parameter set (see the manifest). Deterministic:
    /// same params ⇒ same processor state.
    fn new(sample_rate: f32, params: &Params) -> Self where Self: Sized;

    /// Process one block in place-ish. `in_`/`out` are equal length ≤ MAX_BLOCK.
    /// Interleaving is per-channel planar; mono uses channel 0 only.
    fn process(&mut self, in_l: &[f32], in_r: &[f32], out_l: &mut [f32], out_r: &mut [f32]);

    /// Reset to the just-constructed state (flush filter memory, etc.).
    fn reset(&mut self);
}
```

**Hard rules** (checked, not just documented):
- **No heap.** `#![no_std]`, no `alloc`. The framework compiles the algorithm crate with the allocator disabled.
- **No panics** on any input the manifest can feed it (NaN/Inf/denormal/DC/full-scale). Tier C runs with a panic handler that reports the failure over RTT rather than hanging.
- **Bounded work per block.** `process()` must complete inside the block period. This is a *tier-C assertion* (see §6.3) — Renode timing can't prove it (see the timing-fidelity note in the repo).
- **Determinism.** No `Instant::now()`, no RNG except seeded generators. Same input ⇒ bit-identical output on host and target (both are IEEE-754 single-precision; the M7 FPU is spec-compliant, so A and C1 outputs should match to a few ULP).

The concrete algorithms live in `daisy-dsp` and are already used by `daisy-audio`'s
block callback, so the *same* processor object runs in the real audio pipeline.

---

## 2. What each tier uniquely catches

Run all three because each catches a different class of bug:

- **A (host):** algorithm math, regressions, property violations. Runs on every commit. **Misses:** target-only issues, timing, FPU edge cases, the codec/analog path.
- **B (Renode):** target compilation (`thumbv7em-none-eabihf`), `no_std`/panic issues, and *structural* integration with SAI → DMA → DSP (does the half/full-transfer ping-pong feed the processor correctly?). Runs in CI with no hardware. **Misses:** real-time performance (Renode is not cycle-accurate), the real codec, analog.
- **C (hardware):** real M7 at real speed — **real-time budget**, FPU corner cases, cache coherency of DSP buffers, and (C2) the **actual codec + analog loopback**. The final word. **Needs:** a board + SWD probe + a loopback jumper.

---

## 3. Shared assets (used identically by all three tiers)

These live in one `no_std`-compatible crate, `daisy-dsp-testkit`, so the **host,
the Renode firmware, and the HW firmware all use the exact same code** — the
input a test feeds is defined once and reproduced everywhere.

### 3.1 Deterministic signal generators
Inputs are **generated from a spec**, never streamed, so host and target produce
identical stimuli and only the *output* needs to move across the wire:

| Generator | Params | Use |
|-----------|--------|-----|
| `impulse` | position | impulse response / filter taps |
| `sine` | freq, amp | THD, gain at a frequency |
| `log_sweep` | f0, f1, len | full magnitude/phase response (deconvolved) |
| `white` | seed | statistical / stress, denormal hunting |
| `silence` / `dc` | level | offset, denormal, stability |
| `two_tone` | f1, f2 | intermodulation |
| `step` / `full_scale` | — | clipping, headroom |
| `wav` | path | real-world material (drums, guitar DI) |

### 3.2 Metrics / comparators
The compare step is shared; only the **tolerance** changes by tier-class:

- `null_test` — sample-align (cross-correlate to find latency), subtract, report residual RMS in dBFS.
- `max_abs_error` — for near-exact tiers (A, B, C1).
- `snr_db` — residual-vs-signal, for the noisy C2 analog path.
- `magnitude_response` — assert dB at named frequencies (property, no golden needed).
- `thd_n`, `rms`, `peak`, `dc_offset`, `is_finite` (NaN/Inf guard).

### 3.3 Golden references
Produced by **tier A** (`--bless`) and committed as raw `f32` LE blobs under
`tests/golden/<algo>/<test>.f32`. B and C compare against these. A golden is
regenerated only on an intentional, reviewed algorithm change.

---

## 4. Tier A — host WAV/vector tests

Plain `cargo test`, in-process, fast, exact.

```rust
// daisy-dsp/tests/dsp_host.rs — generated/driven from the manifest
#[test]
fn biquad_lp_1k_impulse() {
    let mut p = BiquadLowpass::new(48_000.0, &params! { cutoff_hz: 1000.0, q: 0.7071 });
    let input = gen::impulse(4096, 0);
    let out = run_mono(&mut p, &input);
    golden::assert_matches("biquad_lp/lp_1k_impulse", &out, Tol::max_abs(1e-4));
}
```

- Runs in CI on every PR; the whole DSP suite is milliseconds.
- `cargo test -- --bless` (env-gated) **writes** the goldens instead of asserting — the one place goldens are minted.
- Property checks (e.g. `magnitude_response`) need no golden and are the preferred form where an analytic expectation exists.

---

## 5. Tier B — Renode simulated tests

The **`dsp-test-firmware`** (§7) built for `thumbv7em-none-eabihf` and run under
Renode, driven by a Robot test — the same pattern as the existing `*-exerciser`
crates (DTCM mailbox the Robot reads).

Two depths, pick per algorithm:

- **B1 (bare):** Robot writes the test spec into a DTCM mailbox, the firmware
  generates the input → `process()` → writes output to a DTCM buffer, Robot reads
  it back and compares to the golden. Proves *the algorithm runs on the emulated
  M7 and matches the host* (target codegen, `no_std`, panics).
- **B2 (piped):** route the block through `daisy-audio` → SAI TX → the Renode
  **SAI-loopback + circular-DMA** model → SAI RX → back to `process()`. Proves the
  **DMA half/full-transfer ping-pong and the SAI↔DSP bridge** move blocks
  correctly. (This is exactly the audio pipeline the SAI model was built to
  exercise.)

```robot
Run One DSP Vector In Renode
    [Arguments]    ${algo_id}    ${test_id}
    Execute Command    sysbus WriteDoubleWord ${MBX_ALGO} ${algo_id}
    Execute Command    sysbus WriteDoubleWord ${MBX_CMD}  ${CMD_RUN}
    Execute Command    emulation RunFor "00:00:00.2"
    ${n}=    Read Mailbox    ${MBX_OUT_LEN}
    ${blob}=    Read Bytes    ${MBX_OUT_BUF}    ${n}
    Golden Compare    ${algo_id}    ${test_id}    ${blob}    tol=max_abs:1e-4
```

**Do not** assert real-time timing here — Renode's DWT/clock model is functional,
not cycle-accurate (see the timing-fidelity note). Timing is tier C's job.

---

## 6. Tier C — hardware loopback tests (the automated batch)

The same `dsp-test-firmware`, flashed to **internal flash over SWD** (probe-rs —
*not* the QSPI/DFU path, so no BOOT/RESET button and no bootloader in the loop),
talking to the host over **RTT**.

### 6.1 Two taps, one firmware
- **C1 — digital-in-the-loop (no cable, absolute correctness on HW):** host asks
  for `(algo, test)`; firmware generates the input, runs `process()`, and streams
  the **digital output** back over RTT. Compared to the **host golden**, tight
  tolerance. This is the real-silicon correctness + real-time check. **Fully
  automatable, needs only the probe.**
- **C2 — analog loopback (needs a jumper, validates codec + analog):** firmware
  sends the DSP output to the **codec DAC → OUT jack → [patch cable] → IN jack →
  codec ADC**, captures the round-tripped signal, streams it back. Because the
  analog loop adds the codec's own response + noise, C2 compares against a
  **reference capture** (blessed once on a known-good unit) with an `snr_db`
  tolerance, and latency-aligns first. C2 is a *regression/health* test of the
  whole audio path, not an absolute-math test.

### 6.2 The no-touch batch flow
The key to "tens of algorithms, no reset presses" is: **flash once, then loop
over RTT.**

```
   host                              target (Daisy, one internal-flash image)
   ────                              ──────────────────────────────────────
   probe-rs download + reset ──────► firmware boots, sets up codec/SAI,
   (ONCE)                            then enters an RTT command loop
                                     └─ waits for a command
   ┌─ for each (algo,test) in manifest ────────────────────────────────────┐
   │ RTT ↓  {run, algo_id, test_id, gen_spec, len}  ─────────►  generate    │
   │                                                            process()   │
   │                                                            (+ codec    │
   │                                                             for C2)    │
   │ RTT ↑  {output blob, cycles/block, peak, flags}  ◄─────────  capture   │
   │ compare to golden/reference; record pass/fail                          │
   └────────────────────────────────────────────────────────────────────────┘
```

- **One `probe-rs download` + one reset**, at the start. After that the firmware
  never resets; every test is an RTT round-trip. Ten or a hundred algorithms cost
  one flash.
- Alternatively `probe-rs attach` to an already-running board and skip even that.
- Inputs are *generated on-device* from the spec (§3.1), so only outputs cross
  RTT — a few seconds of mono `f32` is ~1 MB, ≲1–2 s over SWD RTT.
- The whole thing is one host command (§8) suitable for CI on a self-hosted
  runner with a board attached.

### 6.3 Real-time budget (tier-C-only assertion)
The firmware wraps `process()` in a `DWT.CYCCNT` measurement and returns
**cycles-per-block**. The manifest declares `max_cycles_per_block`; exceeding it
is a **failure** (it would drop audio at 48 kHz). Example: block = 48 samples at
48 kHz = 1 ms = **480 000 cycles** at 480 MHz — that's the hard ceiling; the
manifest sets a stricter budget per algorithm. This is *the* thing only real
hardware can tell you, and it's why C exists even for pure-DSP algorithms.

### 6.4 The rig
- SWD probe (ST-Link V3 / the project's probe-rs backend).
- A **loopback jumper**: audio OUT → audio IN (a fixed patch cable or a jig).
  Only C2 needs it; C1 runs cable-less.
- Optional: a relay/USB-hub the runner can power-cycle if a firmware ever wedges
  (belt-and-suspenders; the RTT loop + watchdog should make it unnecessary).

---

## 7. The unified firmware — `dsp-test-firmware`

One binary, built for the target, used by **both B and C**. It contains:

- **Algorithm registry:** a `const` table mapping `algo_id → fn(&Params) -> Box-free processor`. Because there's no heap, dispatch is via an enum or a `&dyn DspProcessor` backed by a `static mut` union / `heapless`-style storage sized to the largest processor. (One image holds *all* algorithms, so a batch never reflashes.)
- **Generator registry:** the shared `daisy-dsp-testkit` generators.
- **A transport trait** with two implementations, selected by cargo feature:
  - `rtt` (tier C): `rtt-target`/`defmt-rtt` up+down channels.
  - `mailbox` (tier B): fixed DTCM addresses the Robot reads/writes.
- **The codec path** (`daisy-audio`) for C2, gated behind the `codec` feature.
- **Panic + HardFault handlers** that report the fault code over the transport instead of hanging, so a batch run records "algo X panicked" and moves on.

Runs from **internal flash** (its own `memory.x`, like the exercisers) so probe-rs
can flash+reset it directly — no bootloader, no QSPI, no DFU, no buttons.

---

## 8. The orchestrator — `daisy dsp-test`

A subcommand of the existing host CLI (or an `xtask`) that reads the manifest and
drives whichever tiers you ask for:

```sh
daisy dsp-test --tier host                 # A: cargo test, all algorithms
daisy dsp-test --tier renode               # B: build fw + run the Robot suite
daisy dsp-test --tier hw --which C1        # C1: flash once, batch over RTT
daisy dsp-test --tier hw --which C2 --algo reverb   # C2: analog loopback, one algo
daisy dsp-test --all                       # A+B, and C if a probe is present
daisy dsp-test --bless                     # regenerate goldens (host), reviewed
```

It emits a per-`(algo, test, tier)` pass/fail table and a JUnit XML for CI. A
result is only **green** when its manifest-declared tiers all pass.

---

## 9. Manifest schema (`dsp-tests.toml`)

One declarative file is the single source of what exists and how it's checked.
Adding an algorithm = one processor + one manifest block; the three runners pick
it up automatically.

```toml
schema = 1
sample_rate = 48000
block_size   = 48        # 1 ms blocks

[[algorithm]]
id    = "biquad_lp"
title = "Biquad low-pass (RBJ)"
kind  = "BiquadLowpass"          # resolves to daisy_dsp::filter::BiquadLowpass
channels = "mono"
max_cycles_per_block = 20000     # tier-C real-time budget (of 480000 available)

  [[algorithm.test]]
  id     = "lp_1k_impulse"
  params = { cutoff_hz = 1000.0, q = 0.7071 }
  input  = { gen = "impulse", len = 4096 }
  golden = "golden/biquad_lp/lp_1k_impulse.f32"
  tiers  = ["host", "renode", "hw"]
  tolerance = { max_abs_error = 1e-4, snr_db = 60 }   # exact for A/B/C1; snr for C2

  [[algorithm.test]]
  id     = "lp_sweep_response"
  params = { cutoff_hz = 1000.0, q = 0.7071 }
  input  = { gen = "log_sweep", f0 = 20.0, f1 = 20000.0, len = 96000 }
  tiers  = ["host", "hw"]
  # property check — no golden:
  check  = { type = "magnitude_response", points = [
      { freq =  100, min_db = -0.5, max_db =  0.5 },
      { freq = 1000, min_db = -3.5, max_db = -2.5 },
      { freq = 8000, min_db = -40,  max_db = -20  },
  ]}

[[algorithm]]
id = "reverb"
title = "Feedback-delay-network reverb"
kind = "Fdn8"
channels = "stereo"
max_cycles_per_block = 120000
  [[algorithm.test]]
  id = "impulse_tail"
  params = { rt60_s = 1.5, mix = 1.0 }
  input  = { gen = "impulse", len = 96000 }
  golden = "golden/reverb/impulse_tail.f32"
  tiers  = ["host", "renode", "hw"]      # C2 analog for the real tail
  tolerance = { max_abs_error = 1e-3, snr_db = 45 }
```

---

## 10. Worked example: `biquad_lp` through all three tiers

1. **Write it once** — `daisy_dsp::filter::BiquadLowpass: DspProcessor`.
2. **A:** `daisy dsp-test --tier host` → generate impulse, `process()`, assert vs `golden/biquad_lp/lp_1k_impulse.f32` (`max_abs 1e-4`) and assert the sweep magnitude response.
3. **B:** build `dsp-test-firmware` (mailbox), Robot writes `algo=biquad_lp,test=lp_1k_impulse`, `RunFor`, reads the DTCM output, compares to the same golden. (B2 also pipes it through the SAI/DMA loopback model.)
4. **C1:** `probe-rs download` once; RTT `{run biquad_lp lp_1k_impulse}`; firmware streams the digital output + `cycles/block`; compare to the golden and assert `cycles/block ≤ 20000`.
5. **C2:** with the OUT→IN jumper, RTT `{run … codec}`; firmware plays through the codec, captures the return, latency-aligns, asserts `snr_db ≥ 60` vs the reference capture.

Same algorithm, same input spec, same golden — four (five) harnesses.

---

## 11. CI wiring

- **Every PR:** tier A (host `cargo test`) + tier B (Renode) — no hardware, already how the repo runs the `*-exerciser` suites.
- **Self-hosted runner with a board + probe:** tier C1 nightly and on-demand (label a PR `hw-test`), batched over RTT. C2 on the runner that has the loopback jumper.
- A green merge requires A+B; C is advisory until the rig is a permanent fixture, then required for `daisy-dsp` changes.

---

## 12. Directory layout

```
crates/
  daisy-dsp/                 # the algorithms + the DspProcessor trait  (tier A tests live here)
  daisy-dsp-testkit/         # SHARED no_std: generators, metrics, manifest types, golden I/O
  dsp-test-firmware/         # ONE target binary for tiers B & C (registry + transports + codec)
  daisy-cli/                 # `daisy dsp-test` orchestrator
renode/
  dsp_vectors.robot          # tier B driver (mailbox + optional SAI/DMA loopback)
tests/golden/<algo>/<test>.f32   # blessed reference outputs (from tier A)
dsp-tests.toml               # the manifest
docs/dsp-test-framework.md   # this document
```

---

### Design decisions, condensed
- **One trait, one input-spec, one golden** shared by all tiers → results are directly comparable and there's no per-tier reimplementation to drift.
- **Host is truth; hardware is validated against it.** C1 = correctness+timing on real silicon; C2 = codec/analog health.
- **Flash-once + RTT command loop** is what makes tier C batchable with no button presses; internal-flash + SWD (not QSPI/DFU) is what removes the bootloader/BOOT/RESET dance.
- **Real-time budget is a first-class, hardware-only assertion** (DWT cycles/block) — the reason C matters even for pure math, and something Renode deliberately can't judge.

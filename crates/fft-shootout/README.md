# fft-shootout

Standalone firmware that **DWT-benchmarks competing real-FFT implementations
head-to-head** on the real Cortex-M7, so we can rank them by measured
cycles/transform rather than by argument.

Runs each entrant × size (256/512/1024/2048), brackets it with CYCCNT, and writes
the minimum over `ITERS` repeats to a DTCM array. Reset clock (HSI 64 MHz, 0 wait
states, no cache), all scratch in DTCM — the cleanest core-bound baseline.

## Entrants

- **`mine`** — `daisy_dsp::fft`, our mixed radix-4/2 (radix-2²), const-N specialized.
- **`mfft`** — [`microfft`](https://crates.io/crates/microfft) 0.6, radix-2, the
  all-Rust `no_std` reference.
- *(radix-4, Stockham, and the Q15 SIMD kernel are added as they land.)*

CMSIS-DSP (`arm_rfft_fast_f32` / `arm_rfft_q15`) is **C** and so is not a default
entrant (the repo is all-Rust); it can be added behind an opt-in `cmsis` feature
if a reference-vs-C number is wanted.

## Reading the ranking (hardware — authoritative)

```sh
probe-rs run --chip STM32H750IBKx \
  target/thumbv7em-none-eabihf/release/fft-shootout      # flashes + runs
# then read the results array (16 words at 0x2001F000):
probe-rs read b32 0x2001F000 16 --chip STM32H750IBKx
```

| Offset | Slot | Meaning |
|-------:|------|---------|
| `+0x00` | `RESET` | `1` = `main` reached |
| `+0x04` | `STAGES` | bitmask, `0xFF` = all eight benches ran |
| `+0x08` | `ITERS` | measurement repeats |
| `+0x0C` | `OVERHEAD` | measurement-bracket cost (subtracted from each) |
| `+0x10..0x1C` | `MINE` 256/512/1024/2048 | ours, cycles/forward-transform |
| `+0x20..0x2C` | `MFFT` 256/512/1024/2048 | microfft, cycles/forward-transform |
| `+0x3C` | `DONE` | `0xD09E` = all benches ran |

Each figure is the **minimum** over `ITERS` bracketed runs, overhead subtracted.

## Renode is NOT a ranking

`renode/fft_shootout.robot` only proves every entrant *executes without faulting*
and that CYCCNT advanced. Renode is a functional translator — CYCCNT ≈ instruction
count, not the real M7 pipeline/cache/FPU latency — so its VALUES are at best a
crude instruction-count proxy, never the ranking. Rank on hardware.

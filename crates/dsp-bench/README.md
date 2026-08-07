# dsp-bench — DWT cycles/block for the `daisy-dsp` processors

Standalone firmware that measures **cycles per 64-sample block** for each
`daisy-dsp` processor using the Cortex-M7's DWT `CYCCNT`, and writes the results
to a fixed DTCM array. It closes the gap between "the host tests pass and it
compiles for the M7" and "here is the real compute budget on silicon."

## Which number to trust

| Tier | What it tells you | Trust for a cycle budget? |
|------|-------------------|---------------------------|
| **Hardware** (`probe-rs` reads the array) | the real cycles/block on the M7 | ✅ **authoritative** |
| **Renode** (`renode/dsp_bench.robot`) | each processor executes on the M7 ISA without faulting | ❌ functional smoke only |

Renode is a **functional translator**: `CYCCNT` advances with *virtual time*
(≈ instruction count), not the real M7 pipeline / cache / FPU latencies (see
`STM32H7_DWT_Clocked`'s "Fidelity boundary" header and the
`feedback_renode_timing_fidelity` note). So the Renode run asserts only that all
seven processors ran to completion (a stage bitmask) and that the DWT counted —
**never the cycle values**. The FPU-heavy processors (reverb, pad) will differ
most between the Renode proxy and silicon.

## Operating point (this version)

Runs on the **reset clock**: HSI 64 MHz, internal-flash 0 wait states, no cache,
buffers in DTCM (zero-wait, core-coupled). That's a clean **core-bound baseline**
— no fragile PLL/cache bring-up. The definitive number at the real operating
point (480 MHz + I/D cache + AXI/SDRAM buffer placement, where wait states and
cache misses enter) is a follow-up that adds the full bring-up; expect it to be
**higher** than this baseline for memory- and FPU-bound processors.

## Running it

### Renode (functional smoke, in CI)

```sh
./renode/build-and-run.sh renode/dsp_bench.robot   # or the whole suite
```

### Hardware (the real budget) — via probe-rs

```sh
cargo build -p dsp-bench --release --target thumbv7em-none-eabihf
probe-rs run --chip STM32H750IBKx \
  target/thumbv7em-none-eabihf/release/dsp-bench    # flashes + runs
# then halt and read the results array (13 words at 0x20018000):
probe-rs read b32 0x20018000 13 --chip STM32H750IBKx
```

## Results array (DTCM `0x20018000`, one `u32` each)

| Offset | Index | Meaning |
|-------:|-------|---------|
| `+0x00` | `RESET` | `1` = `main` reached |
| `+0x04` | `BLOCK` | block size (samples) = 64 |
| `+0x08` | `ITERS` | measurement repeats |
| `+0x0C` | `OVERHEAD` | measurement-bracket cost (subtracted from each) |
| `+0x10` | `ONEPOLE` | `OnePole::process` cycles/block |
| `+0x14` | `BIQUAD` | `Biquad::process` cycles/block |
| `+0x18` | `DELAY` | `DelayLine` write + `read_frac`, ×64 |
| `+0x1C` | `FDNREVERB` | `FdnReverb::process` cycles/block |
| `+0x20` | `FREEZE` | `Freeze::tick` (frozen loop), ×64 |
| `+0x24` | `PADDRONE` | `PadDrone::process` (frozen: freeze+reverb) cycles/block |
| `+0x28` | `ENV` | `Env::tick`, ×64 |
| `+0x2C` | `STAGES` | bitmask, `0x7F` = all seven ran |
| `+0x30` | `DONE` | `0xD09E` = all benches ran |

Each figure is the **minimum** over `ITERS` bracketed runs (least-noise hot-path
estimate), with the empty-bracket overhead subtracted.

## Reading it as a budget

At the target 480 MHz, one 64-sample block at 48 kHz has
`480e6 × 64 / 48000 = 640_000` cycles. So:

```
%CPU = cycles_per_block / 640_000
```

e.g. a processor measuring 12_800 cycles/block ≈ **2.0 %** of one M7 core.
(Substitute the real operating-point number once the 480 MHz + cache bring-up
lands; the reset-clock baseline here is the lower bound.)

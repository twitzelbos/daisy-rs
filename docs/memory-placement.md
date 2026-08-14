# Memory placement — putting Rust code and data in XIP / SRAM / SDRAM

On the STM32H750 the app runs **execute-in-place (XIP) from QSPI flash** at
`0x9000_0000`, with several distinct RAMs available for data. This note is the
practical guide to putting a given `fn` or `static` in the region you want.

There is **no Rust-language "put this in SRAM" attribute**. Placement is done with
**linker sections**: you tag an item with `#[link_section = "..."]` and map that
section to a `MEMORY` region in the crate's `memory.x`. Both halves must agree —
the attribute names a section, the linker script maps the section to a region.

> Edition note: on the 2024 edition `link_section` is an `unsafe` attribute —
> write `#[unsafe(link_section = "...")]`. On 2021 and earlier use
> `#[link_section = "..."]`. Examples below use the 2021 form.

## The regions (this repo's app `memory.x`)

| Region  | Address        | Size  | Notes |
| ------- | -------------- | ----- | ----- |
| `FLASH` | `0x9000_0000`  | 8 MiB | QSPI, **XIP** — read-only executable. Slower than SRAM. |
| `RAM`   | `0x2000_0000`  | 128 K | **DTCM** — CPU-private, tightly coupled, **never cached**, fastest, no AXI arbitration. Holds the stack by default. Not reachable by all DMA masters. |
| `AXI`   | `0x2400_0000`  | 512 K | AXI SRAM — large, DMA-reachable, **cacheable**. The workhorse for audio/DSP buffers. |
| `SDRAM` | `0xC000_0000`  | 64 MiB| External SDRAM — **live only after `daisy_bsp::sdram::init()`**. Large, slower, cacheable. |

D2 SRAM at `0x3000_0000` (handy for DMA/USB buffers) is **already marked
non-cacheable by the app's MPU** (`configure_mpu_and_caches` region 0 in
`daisy-app-template/src/main.rs`), but the app `memory.x` doesn't yet define a
linker section that lands there — see
[Non-cacheable DMA buffers](#non-cacheable-dma-buffers-d2-sram).

## Default placement — what you get with no annotation

| Item | Section | Region | Lands in |
| ---- | ------- | ------ | -------- |
| `fn` (all code) | `.text` | `REGION_TEXT` → FLASH | **XIP flash** |
| `const`, string literals | `.rodata` | FLASH | XIP flash |
| `static X: T = ..` (initialised) | `.data` | `REGION_DATA` → RAM (DTCM) | DTCM, **copied from flash at boot** |
| `static X: T` (zero / uninit) | `.bss` | RAM (DTCM) | DTCM |
| stack | — | `REGION_STACK` → RAM (DTCM) | DTCM |

**Code already runs XIP by default** — that is the baseline. You only annotate to
move something *off* its default.

## Recipes

### Keep a large read-only table in XIP flash (don't spend RAM on a copy)

An initialised `static` is copied into DTCM at boot. To keep a big lookup table
resident in QSPI flash instead:

```rust
#[link_section = ".rodata"]        // stays in FLASH, no RAM copy
static SINE_LUT: [f32; 4096] = [ /* ... */ ];
```

A plain `const` also lands in `.rodata`, but may be duplicated per use-site; a
`static` in `.rodata` is a single shared copy. Trade-off: XIP reads are slower
than SRAM, so keep **hot** DSP tables in SRAM (below) and **cold** ones here.

### ⚠️ The `INSERT AFTER .bss` trap (read this first)

It is tempting to place a scratch buffer in AXI/D2/SDRAM with a NOLOAD section
`INSERT AFTER .bss`. **Don't** — for any region that isn't DTCM. cortex-m-rt
*deliberately* pushes `__ebss` past an `INSERT AFTER .bss` section so the `.bss`
loop zeroes it, which is only valid when the section is **contiguous with `.bss`'s
region (DTCM)**. AXI (`0x2400_0000`), D2 SRAM (`0x3000_0000`) and SDRAM
(`0xC000_0000`) are *separate* regions with unmapped gaps after DTCM
(`0x2002_0000`), so this drags `__ebss` across a gap and the startup zeroing loop
runs into **unmapped memory → the M7 locks up before `main`**. It "passes" on
Renode (which backs the gap) but faults on silicon. `daisy check-elf` and
`daisy flash` now reject such an ELF, and the Renode `gap_guard`/`usb_audio_codec_boot`
robots catch it in CI.

**Use a fixed-address buffer instead** (no linker section → it can't disturb
cortex-m-rt's startup symbols). This is what `daisy-audio` does for its DMA buffers.

### Data in AXI SRAM (fast, DMA-reachable) — the workhorse

AXI SRAM spans `0x2400_0000..0x2408_0000`. Point a `*mut` at it:

```rust
// Caller-managed scratch in AXI SRAM (cacheable — see the caveat below).
const DMA_TX: *mut [u16; 512] = 0x2400_0000 as *mut _;
// use once: let buf = unsafe { &mut *DMA_TX }; buf.fill(0);
```

You initialise it yourself before first read (there is no startup zeroing).

> **Cache caveat.** AXI SRAM is cacheable, so any buffer a DMA writes or reads
> needs the clean/invalidate discipline (`DCCMVAC` before a DMA read of what the
> CPU wrote; `DCIMVAC` after a DMA write before the CPU reads). This is exactly
> what the Renode cache-coherency checker enforces in CI — see
> [`renode-fidelity.md`](renode-fidelity.md) §1. To sidestep coherency entirely,
> use non-cacheable D2 SRAM instead (below).

### Non-cacheable DMA buffers (D2 SRAM)

If you'd rather not do cache maintenance around a DMA buffer, place it in D2 SRAM
at `0x3000_0000`. The app's `#[pre_init]` MPU already makes that window
**non-cacheable** (`configure_mpu_and_caches` region 0) — the non-cacheable
attribute comes from the MPU, not the linker. Also enable the D2 SRAM clock
(`RCC_AHB2ENR` SRAM1/2/3EN, off at reset) in `#[pre_init]` before first use, or the
access bus-faults. Use a **fixed address** (per the trap above), keeping it inside
the MPU's non-cacheable window at `0x3000_0000`:

```rust
// D2 SRAM (0x3000_0000): DMA-reachable, MPU-non-cacheable → no cache maintenance.
const EP_MEMORY: *mut [u32; 1024] = 0x3000_0000 as *mut _;
```

`daisy-audio` uses exactly this for its SAI/DMA buffers (`D2_SRAM_BASE`). Keep the
total under the MPU's non-cacheable window, or a buffer lands in a still-cacheable
part of D2 SRAM.

### Run a function *from* SRAM (a "ramfunc")

Put the function's machine code in a section that is loaded into RAM. The
simplest form lands it in **DTCM** (`.data` → `REGION_DATA`):

```rust
#[link_section = ".data"]   // code copied to DTCM at boot; executes from RAM
#[inline(never)]
fn time_critical() { /* ... */ }
```

DTCM sits on the CPU's tightly-coupled bus and **bypasses the L1 caches**, so a
DTCM ramfunc has **no I-cache coherency concern** — it is the safe default. The
only cost is that DTCM is small (128 K, shared with the stack).

> ⚠️ **I-cache hazard — only if the ramfunc lives in *cacheable* memory** (e.g.
> you point a custom executable section at AXI SRAM instead of DTCM). `#[pre_init]`
> runs *before* `.data`/section copy, and the apps enable the I-cache there, so
> the code bytes are written through the D-cache *after* the I-cache is on — the
> first call can fetch stale instructions on silicon. In that case do the
> maintenance before the first call: clean the D-cache to PoU (`DCCMVAU`), `DSB`,
> invalidate the I-cache (`ICIALLU`), `DSB`, `ISB` (the Renode checker's phase-3
> path is the reference sequence). Prefer the DTCM ramfunc above, or just keep
> code in XIP — reach for a cacheable-SRAM ramfunc only with a proven reason.

### Data in external SDRAM (64 MiB)

SDRAM **does not exist until `daisy_bsp::sdram::init()` runs**, and cortex-m-rt's
startup runs before that — so a linker section is doubly wrong here (`INSERT AFTER
.bss` would both drag `__ebss` across the gap *and* try to zero not-yet-live
SDRAM). Use a **fixed address** and initialise it yourself, after `init()`:

```rust
const REVERB_BUF: *mut [f32; 1 << 20] = 0xC000_0000 as *mut _;

// after sdram::init():
unsafe {
    let p = REVERB_BUF as *mut f32;
    for i in 0..(1 << 20) { p.add(i).write(0.0); }
}
```

SDRAM is external, slower, and cacheable — the same DMA-coherency rules as AXI
apply. Good for large, latency-tolerant buffers (reverb tails, sample RAM); keep
per-sample DSP state in DTCM/AXI.

### Executing code from SDRAM

> **You almost certainly do not want this.** The app already runs from 8 MiB of
> XIP flash, and with the I-cache on, XIP code is cached after its first fetch —
> so SDRAM code is *not* faster in steady state, only more fragile. Reach for a
> **DTCM ramfunc** (above) if you need fast RAM code. This section exists because
> it's askable, not because it's recommended.

It *is* possible: the app MPU marks SDRAM executable (`configure_mpu_and_caches`
region 1 sets `xn = 0`). The wrinkles are that (a) SDRAM isn't live until
`sdram::init()`, so cortex-m-rt can't copy the code for you, and (b) SDRAM is
cacheable, so this is the full I-cache hazard. You need a section whose **load
address is in flash but run address is in SDRAM**, a manual copy after init, and
cache maintenance before the first call:

```
/* memory.x — VMA in SDRAM, LMA in flash */
SECTIONS {
  .sdram_text : ALIGN(8) {
    __ssdram_text = .;
    *(.sdram_text .sdram_text.*);
    . = ALIGN(8);
    __esdram_text = .;
  } > SDRAM AT> FLASH
  __lsdram_text = LOADADDR(.sdram_text);
} INSERT AFTER .text;
```
```rust
#[link_section = ".sdram_text"]
#[inline(never)]
fn big_dsp_pass(/* ... */) { /* ... */ }

extern "C" {
    static __ssdram_text: u8;  // run address (SDRAM)
    static __esdram_text: u8;
    static __lsdram_text: u8;  // load address (flash)
}

// after sdram::init(), before the FIRST call to big_dsp_pass:
unsafe {
    let dst = &__ssdram_text as *const u8 as *mut u8;
    let src = &__lsdram_text as *const u8;
    let len = &__esdram_text as *const u8 as usize - dst as usize;
    core::ptr::copy_nonoverlapping(src, dst, len);

    // The CPU wrote the code through the D-cache into cacheable SDRAM; push it to
    // memory and drop any stale I-cache/prefetch, or the first fetch is garbage.
    // Clean the D-cache to PoU then invalidate the I-cache, with barriers — via
    // the cortex-m SCB helpers, or by hand exactly as the cache-coherency-
    // exerciser does (write DCCMVAU per line + DSB, then ICIALLU + DSB + ISB).
    cortex_m::asm::dsb();
    // ... clean D-cache (DCCMVAU) + invalidate I-cache (ICIALLU) here ...
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}
```

This is exactly the phase-3 hazard the Renode checker catches — get the
maintenance wrong and it fetches stale instructions on silicon (a green sim run
won't warn you; the checker will).

## "Running a hot loop in the stack" — a common confusion

There is no such thing, and no `#[...]` for it. Two separate things get placed:

- **Code** (the loop's instructions) runs from wherever its *function* lives —
  XIP flash by default, or a ramfunc region if you moved it. A loop is not
  independently placeable; placement is per-function (`#[link_section]`). You
  never execute *from* the stack — that region is execute-never on purpose.
- **The loop's local variables** (its scratch/state) are automatically on the
  **stack**, which already lives in **DTCM** — the fastest RAM, never cached. So
  a tight loop's locals are in fast memory *for free*, no annotation required.

So "make this loop fast" decomposes into: put its *data* where it's fast (locals
→ DTCM stack automatically; large state → a `static` in DTCM/AXI), and only if
profiling justifies it, put its *code* in a DTCM ramfunc. Watch stack size,
though — DTCM is 128 KiB shared with everything; a big `[f32; N]` local can blow
it, so large buffers belong in a `static`/section, not on the stack.

## Rule of thumb

- **Code** — XIP flash by default. Ramfunc only for a proven hot loop; prefer a
  DTCM ramfunc (uncached, safe) and mind the I-cache only if it's in cacheable SRAM.
- **Stack + tight DSP state** — DTCM (default `RAM`): fastest, never cached.
- **Audio / DMA buffers** — AXI SRAM (`.sram1`), or non-cacheable D2 SRAM
  (`.sram_d2`) to avoid coherency work.
- **Big cold tables** — XIP `.rodata`. **Huge buffers** — SDRAM (`NOLOAD`,
  initialised after `sdram::init()`).
- **Executing code from SDRAM** — possible but not worth it; use a DTCM ramfunc
  or stay in (I-cached) XIP.

## Verifying where something landed

```sh
# per-symbol addresses (region tells you where it is)
cargo size -p <app> --release --target thumbv7em-none-eabihf -- -A
arm-none-eabi-nm --print-size --size-sort target/thumbv7em-none-eabihf/release/<app> \
  | grep <SYMBOL>
```

A symbol at `0x9…` is in XIP flash, `0x20…` DTCM, `0x24…` AXI SRAM, `0x30…` D2
SRAM, `0xC0…` SDRAM.

#![no_std]
#![no_main]
#![allow(deprecated)] // cortex-m-rt 0.7 #[pre_init]; migrate later.

//! Binaural spatializer — **phase 2**: mono codec IN → HRIR *direction* + a
//! parametric *room* → stereo codec OUT, so a mono guitar patched to the left
//! input images *behind and to the left*, a few feet away, on headphones.
//!
//! Signal path (in the DMA1_STR1 audio callback, 48-frame stereo blocks):
//! ```text
//!                        ┌► [48→64 reblock] ─► StereoConvolver(HRIR_L/R) ─► [64→48] ─┐ direct
//!   input L (mono) ──────┤                                                            ├─► + ─► out L/R
//!                        └► air-abs LP ─┬► EarlyReflections ──────────────────────────┤
//!                                       └► FdnReverb (late) ──────────────────────────┘ room
//! ```
//! The **HRIR** ([`daisy_dsp::convolution::StereoConvolver`], fully wet) gives
//! *direction*: the measured left/right IRs carry the interaural time/level/pinna
//! cues that place the source behind-left — a pan/delay cannot. The **room**
//! gives *distance / externalization* (the biggest "out there, not in-head"
//! factor): early reflections ([`daisy_dsp::room::EarlyReflections`], the
//! strongest cue) + late reverb ([`daisy_dsp::reverb::FdnReverb`]) + an
//! air-absorption low-pass, mixed against the direct with fixed distance gains.
//! [`SampleFifo`] adapters bridge the codec's 48-frame callback to the FFT
//! convolver's 64-sample block (`daisy_dsp::reblock`); the first early reflection
//! (8 ms) is set beyond the convolver's ~2 ms latency so it never precedes the
//! direct sound.
//!
//! XIP app: executes from QSPI at `0x9000_0000` and recovers the bootloader's
//! frozen `CoreClocks` (an XIP app cannot `freeze()` its own). `#[pre_init]` sets
//! up the MPU + L1 caches + DWT, identical to the app template / daisy-usb-audio.
//!
//! HARDWARE-ONLY (needs a real codec + the clock hand-off). The `renode_test`
//! feature skips the codec bring-up so the XIP boot can be smoke-tested in sim.
//!
//! Phases 1-2 of `docs/binaural-spatializer.md`; later phases add a second
//! source and live positioning from the Hothouse knobs. Room params are fixed
//! here (phase 4 wires them to the knobs).

mod hrir_data;

use cortex_m_rt::{entry, pre_init};
use daisy_bsp::hal;
use hal::pac;
#[cfg(all(feature = "codec", not(feature = "renode_test")))]
use hal::prelude::*; // RccExt::constrain, GpioExt::split, SAI kernel mux
use panic_halt as _;

use daisy_dsp::convolution::StereoConvolver;
use daisy_dsp::filter::OnePole;
use daisy_dsp::reblock::SampleFifo;
use daisy_dsp::reverb::FdnReverb;
use daisy_dsp::room::{room_taps, EarlyReflections, Tap};
use hrir_data::{HRIR_BEHIND_LEFT_L, HRIR_BEHIND_LEFT_R};

// --- audio geometry ----------------------------------------------------------
/// Codec sample rate (Hz).
const SR: f32 = 48_000.0;
/// FFT partition / block size (power of two ≥ 8). 64 ≈ 1.33 ms latency @ 48 kHz.
const B: usize = 64;
/// HRIR length (taps) — both ears are the same length.
const IR_LEN: usize = HRIR_BEHIND_LEFT_L.len();

/// Scratch a [`StereoConvolver`] needs — the const twin of its runtime
/// `required_scratch` (which is not a `const fn`), so the backing store can be a
/// `static` array. Kept in sync by construction; `StereoConvolver::new` panics
/// if it is ever too small, so a mismatch fails loudly rather than corrupting.
const fn stereo_scratch(ir_len: usize, b: usize) -> usize {
    // Per ear: 4·P·S ring/IR spectra + N window + (2N+2S) fft/acc/ifft scratch,
    // with N = 2b, S = N/2+1, P = ceil(ir_len / b).
    let n = 2 * b;
    let s = n / 2 + 1;
    let p = ir_len.div_ceil(b);
    let one = 4 * p * s + n + (2 * n + 2 * s);
    2 * one // left + right
}
const SCRATCH: usize = stereo_scratch(IR_LEN, B);

// --- room parameters (fixed "a few feet behind" for phase 2) -----------------
/// Early-reflection count.
const NTAPS: usize = 8;
/// Early-reflection delay buffer length (> the longest tap: 40 ms · 48 = 1920).
const EARLY_BUF_LEN: usize = 2048;
/// First early-reflection delay (ms). Must exceed the direct path's reblock +
/// convolver latency (~2 ms) so no reflection precedes the direct sound.
const ER_FIRST_MS: f32 = 8.0;
/// Last early-reflection delay (ms).
const ER_SPREAD_MS: f32 = 40.0;
/// Late-reverb decay (a small room) and feedback damping.
const REVERB_RT60_S: f32 = 0.5;
const REVERB_DAMPING_HZ: f32 = 6_000.0;
/// Air-absorption low-pass on the room send — distance rolls off the highs.
const AIR_CUTOFF_HZ: f32 = 7_000.0;
/// Mix gains: direct (HRIR) vs early reflections vs late reverb (dry/wet balance
/// = perceived distance; more wet = farther / more externalized).
#[cfg(feature = "codec")]
const DRY: f32 = 0.7;
#[cfg(feature = "codec")]
const WET_EARLY: f32 = 0.5;
#[cfg(feature = "codec")]
const WET_LATE: f32 = 0.35;

// --- DSP state: backing buffers + the objects that borrow them ---------------
static mut CONV_SCRATCH: [f32; SCRATCH] = [0.0; SCRATCH];
static mut REVERB_BUF: [f32; FdnReverb::REQUIRED_BUF] = [0.0; FdnReverb::REQUIRED_BUF];
static mut EARLY_BUF: [f32; EARLY_BUF_LEN] = [0.0; EARLY_BUF_LEN];
static mut ROOM_TAPS: [Tap; NTAPS] = [Tap {
    delay: 0.0,
    gain_l: 0.0,
    gain_r: 0.0,
}; NTAPS];

/// The whole spatializer voice: the HRIR direct path (with its 48↔64 reblock
/// FIFOs) plus the room (early reflections, late reverb, air-absorption). Built
/// once in `main` into the static buffers above, then driven by the callback.
// The renode_test boot smoke builds this (to exercise the real init + static
// allocation from XIP) but compiles out the callback that reads the fields.
#[cfg_attr(not(feature = "codec"), allow(dead_code))]
struct Dsp {
    conv: StereoConvolver<'static>,
    early: EarlyReflections<'static>,
    reverb: FdnReverb<'static>,
    air_lp: OnePole,
    in_fifo: SampleFifo<128>, // covers the largest 48/64 carry (< 2 blocks)
    out_l: SampleFifo<256>,
    out_r: SampleFifo<256>,
}
static mut DSP: Option<Dsp> = None;

/// Frames per codec callback, and the interleaved stereo block length.
#[cfg(feature = "codec")]
const FRAMES: usize = daisy_audio::BLOCK_SIZE;
#[cfg(feature = "codec")]
const STEREO: usize = 2 * FRAMES;

/// The audio callback: mono (left in) → [direct HRIR] + [room] → stereo out.
/// Runs in the DMA1_STR1 interrupt. The HRIR gives *direction* (behind-left); the
/// room (early reflections + late reverb + air-absorption) gives *distance /
/// externalization* — makes it sound "out there," not inside the head.
#[cfg(feature = "codec")]
fn spatialize(input: &[f32; STEREO], output: &mut [f32; STEREO]) {
    // SAFETY: runs only in the audio ISR; `DSP` and its buffers are touched
    // nowhere else, and `DSP` was built in `main` before `start` unmasked the
    // interrupt. Raw-pointer access avoids `static mut` reference UB.
    unsafe {
        let dsp = match (*core::ptr::addr_of_mut!(DSP)).as_mut() {
            Some(d) => d,
            None => {
                output.fill(0.0);
                return;
            }
        };

        // Mono source = LEFT input channel (patch the mono guitar to the left in).
        let mut mono = [0.0f32; FRAMES];
        for (f, m) in mono.iter_mut().enumerate() {
            *m = input[2 * f];
        }

        // Direct path: HRIR via 48→64 reblock → convolver → 64→48 reblock.
        dsp.in_fifo.extend(&mono);
        let mut blk = [0.0f32; B];
        let (mut ol, mut or) = ([0.0f32; B], [0.0f32; B]);
        while dsp.in_fifo.pop(&mut blk) {
            dsp.conv.process_block(&blk, &mut ol, &mut or);
            dsp.out_l.extend(&ol);
            dsp.out_r.extend(&or);
        }
        let (mut dl, mut dr) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        let primed = dsp.out_l.len() >= FRAMES;
        if primed {
            dsp.out_l.pop(&mut dl);
            dsp.out_r.pop(&mut dr);
        }

        // Room path: air-absorption LP send → early reflections + late reverb.
        let mut send = [0.0f32; FRAMES];
        dsp.air_lp.process(&mono, &mut send);
        let (mut el, mut erf) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        dsp.early.process(&send, &mut el, &mut erf);
        let (mut rl, mut rr) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        dsp.reverb.process(&send, &mut rl, &mut rr);

        // Mix direct + early + late with the fixed distance gains.
        for f in 0..FRAMES {
            let (d_l, d_r) = if primed { (dl[f], dr[f]) } else { (0.0, 0.0) };
            output[2 * f] = DRY * d_l + WET_EARLY * el[f] + WET_LATE * rl[f];
            output[2 * f + 1] = DRY * d_r + WET_EARLY * erf[f] + WET_LATE * rr[f];
        }
    }
}

/// Build the whole DSP voice into its static buffers. Call once, before starting
/// audio (the ISR is still masked).
fn init_dsp() {
    // SAFETY: single-threaded init; each static buffer is borrowed exactly once
    // (they then live for the program's life inside `DSP`).
    unsafe {
        let conv_scratch: &'static mut [f32] = &mut *core::ptr::addr_of_mut!(CONV_SCRATCH);
        let conv = StereoConvolver::new(
            &HRIR_BEHIND_LEFT_L,
            &HRIR_BEHIND_LEFT_R,
            B,
            1.0,
            conv_scratch,
        );

        room_taps(
            &mut *core::ptr::addr_of_mut!(ROOM_TAPS),
            SR,
            ER_FIRST_MS,
            ER_SPREAD_MS,
        );
        let early = EarlyReflections::new(
            &mut *core::ptr::addr_of_mut!(EARLY_BUF),
            &*core::ptr::addr_of!(ROOM_TAPS),
        );
        let reverb = FdnReverb::new(
            &mut *core::ptr::addr_of_mut!(REVERB_BUF),
            SR,
            REVERB_RT60_S,
            REVERB_DAMPING_HZ,
            1.0, // fully wet — the dry/direct is the HRIR path
        );
        let air_lp = OnePole::lowpass(SR, AIR_CUTOFF_HZ);

        *core::ptr::addr_of_mut!(DSP) = Some(Dsp {
            conv,
            early,
            reverb,
            air_lp,
            in_fifo: SampleFifo::new(),
            out_l: SampleFifo::new(),
            out_r: SampleFifo::new(),
        });
    }
}

// --- MPU / cache / DWT #[pre_init] (identical to the app template) -----------
const GPIOC_MODER: *mut u32 = 0x5802_0800 as *mut u32;
#[cfg(feature = "renode_test")]
const GPIOC_BSRR: *mut u32 = 0x5802_0818 as *mut u32;
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;

#[inline(always)]
unsafe fn led_output() {
    let mut m = core::ptr::read_volatile(GPIOC_MODER);
    m &= !(0b11 << 14);
    m |= 0b01 << 14;
    core::ptr::write_volatile(GPIOC_MODER, m);
}

#[inline(always)]
unsafe fn enable_dwt() {
    core::ptr::write_volatile(DEMCR, core::ptr::read_volatile(DEMCR) | (1 << 24));
    core::ptr::write_volatile(DWT_CTRL, core::ptr::read_volatile(DWT_CTRL) | 1);
}

#[inline(always)]
#[allow(clippy::too_many_arguments)] // one arg per MPU RASR field
unsafe fn mpu_region(
    n: u32,
    base: u32,
    log2_bytes: u32,
    tex: u32,
    s: u32,
    c: u32,
    b: u32,
    xn: u32,
) {
    core::ptr::write_volatile(MPU_RNR, n);
    core::ptr::write_volatile(MPU_RBAR, base);
    let rasr = 1
        | ((log2_bytes - 1) << 1)
        | (b << 16)
        | (c << 17)
        | (s << 18)
        | (tex << 19)
        | (0b011 << 24)
        | (xn << 28);
    core::ptr::write_volatile(MPU_RASR, rasr);
}

/// MPU (libDaisy regions) + L1 caches; MPU-before-cache ordering is mandatory.
unsafe fn configure_mpu_and_caches() {
    core::ptr::write_volatile(MPU_CTRL, 0);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    mpu_region(0, 0x3000_0000, 15, 1, 1, 0, 0, 0); // SRAM_D2 DMA pool, non-cacheable
    mpu_region(1, 0xC000_0000, 26, 0, 0, 1, 1, 0); // SDRAM, write-back cacheable
    mpu_region(2, 0x3880_0000, 12, 1, 1, 0, 0, 0); // Backup SRAM, non-cacheable
    mpu_region(3, 0x9000_0000, 23, 0, 0, 1, 0, 0); // QSPI XIP flash (8 MB), write-through + exec
    core::ptr::write_volatile(MPU_CTRL, (1 << 0) | (1 << 2)); // ENABLE | PRIVDEFENA
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    let mut cp = cortex_m::Peripherals::steal();
    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
}

#[pre_init]
unsafe fn pre_init() {
    configure_mpu_and_caches();
    enable_dwt();
    led_output();
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    run(dp)
}

/// Renode boot check: sim has no SAI/codec model, so skip the codec bring-up and
/// prove the XIP app booted + `pre_init` ran by blinking PC7 (Renode samples it).
#[cfg(feature = "renode_test")]
fn run(_dp: pac::Peripherals) -> ! {
    init_dsp(); // still exercises the full DSP build + static buffers
    let mut hb: u32 = 0;
    loop {
        hb = hb.wrapping_add(1);
        unsafe {
            core::ptr::write_volatile(
                GPIOC_BSRR,
                if hb & 0x8_0000 != 0 { 1 << 7 } else { 1 << 23 },
            );
        }
    }
}

#[cfg(all(feature = "codec", not(feature = "renode_test")))]
fn run(dp: pac::Peripherals) -> ! {
    init_dsp();

    // Recover the bootloader's frozen clock tree (an XIP app can't `freeze()`),
    // then mint the peripheral rec tokens without re-configuring the clocks.
    let clocks = unsafe { daisy_bsp::clocks::handoff::restore() }
        .expect("CoreClocks hand-off from the bootloader");
    let rcc = dp.RCC.constrain();
    let rec = unsafe { rcc.steal_peripheral_rec() };

    // SAI1 kernel mux → PLL3P (the bootloader ran PLL3 ≈ 49.152 MHz), so the HAL
    // computes the codec's MCLK divider from the real kernel clock.
    let sai1_rec = rec.SAI1.kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::Pll3P);

    // Classic Daisy Seed: daisy-audio auto-detects the codec (AK4556 / WM8731 /
    // PCM3060) from the PD3/PD4 straps and runs its init — one binary, all three.
    #[cfg(not(feature = "seed3"))]
    let mut audio = {
        let gpiob = dp.GPIOB.split(rec.GPIOB);
        let gpiod = dp.GPIOD.split(rec.GPIOD);
        let gpioe = dp.GPIOE.split(rec.GPIOE);
        let gpioh = dp.GPIOH.split(rec.GPIOH);
        let pins = daisy_audio::Pins {
            mclk_a: gpioe.pe2,
            sck_a: gpioe.pe5,
            fs_a: gpioe.pe4,
            sd_a: gpioe.pe6,
            sd_b: gpioe.pe3,
            pd3: gpiod.pd3,
            pd4: gpiod.pd4,
            scl: gpioh.ph4,
            ctrl: gpiob.pb11,
        };
        daisy_audio::Audio::new(
            dp.SAI1, dp.DMA1, rec.DMA1, sai1_rec, dp.I2C2, rec.I2C2, pins, &clocks,
        )
    };

    // Seed 3 (TAC5242): hardware-strapped, SAI-only, no I2C.
    #[cfg(feature = "seed3")]
    let mut audio = {
        let gpioe = dp.GPIOE.split(rec.GPIOE);
        let pins = daisy_audio::Pins {
            mclk_a: gpioe.pe2,
            sck_a: gpioe.pe5,
            fs_a: gpioe.pe4,
            sd_a: gpioe.pe6,
            sd_b: gpioe.pe3,
        };
        daisy_audio::Audio::new(dp.SAI1, dp.DMA1, rec.DMA1, sai1_rec, pins, &clocks)
    };

    audio.start(spatialize);

    loop {
        cortex_m::asm::wfi();
    }
}

#![no_std]
#![no_main]
#![allow(deprecated)] // cortex-m-rt 0.7 #[pre_init]; migrate later.

//! Binaural spatializer — **phase 1**: mono codec IN → one hardcoded HRIR pair
//! (MIT KEMAR "behind-left") → stereo codec OUT, so a mono guitar patched to the
//! left input images *behind and to the left* on headphones.
//!
//! Signal path (in the DMA1_STR1 audio callback, 48-frame stereo blocks):
//! ```text
//!   input L (mono guitar) ─► [48→64 reblock] ─► StereoConvolver(HRIR_L, HRIR_R)
//!                                                     │            │
//!                                                  out L        out R
//!                                              [64→48 reblock, both] ─► output L/R
//! ```
//! The convolver ([`daisy_dsp::convolution::StereoConvolver`], fully wet) applies
//! the measured left/right HRIRs; the interaural time/level/pinna cues in that
//! measured pair are what place the source behind-left — a pan/delay cannot. The
//! [`SampleFifo`] adapters bridge the codec's 48-frame callback to the FFT
//! convolver's 64-sample block (see `daisy_dsp::reblock`). One-block priming
//! latency; the output is silent until the first blocks fill.
//!
//! XIP app: executes from QSPI at `0x9000_0000` and recovers the bootloader's
//! frozen `CoreClocks` (an XIP app cannot `freeze()` its own). `#[pre_init]` sets
//! up the MPU + L1 caches + DWT, identical to the app template / daisy-usb-audio.
//!
//! HARDWARE-ONLY (needs a real codec + the clock hand-off). The `renode_test`
//! feature skips the codec bring-up so the XIP boot can be smoke-tested in sim.
//!
//! Phase 1 of `docs/binaural-spatializer.md`; later phases add the room
//! (externalization), a second source, and live positioning from the knobs.

mod hrir_data;

use cortex_m_rt::{entry, pre_init};
use daisy_bsp::hal;
use hal::pac;
#[cfg(all(feature = "codec", not(feature = "renode_test")))]
use hal::prelude::*; // RccExt::constrain, GpioExt::split, SAI kernel mux
use panic_halt as _;

use daisy_dsp::convolution::StereoConvolver;
#[cfg(feature = "codec")]
use daisy_dsp::reblock::SampleFifo;
use hrir_data::{HRIR_BEHIND_LEFT_L, HRIR_BEHIND_LEFT_R};

// --- convolver geometry ------------------------------------------------------
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

// --- DSP state (owned by the audio callback; init'd in main before start) ----
static mut CONV_SCRATCH: [f32; SCRATCH] = [0.0; SCRATCH];
static mut CONV: Option<StereoConvolver<'static>> = None;
// Capacities cover the largest 48/64 carry (< 2 blocks) with headroom. Used only
// in the codec callback; the renode_test boot smoke doesn't run audio.
#[cfg(feature = "codec")]
static mut IN_FIFO: SampleFifo<128> = SampleFifo::new();
#[cfg(feature = "codec")]
static mut OUT_L: SampleFifo<256> = SampleFifo::new();
#[cfg(feature = "codec")]
static mut OUT_R: SampleFifo<256> = SampleFifo::new();

/// Frames per codec callback, and the interleaved stereo block length.
#[cfg(feature = "codec")]
const FRAMES: usize = daisy_audio::BLOCK_SIZE;
#[cfg(feature = "codec")]
const STEREO: usize = 2 * FRAMES;

/// The audio callback: mono (left in) → HRIR → stereo out, re-blocked to the
/// convolver's 64-sample block. Runs in the DMA1_STR1 interrupt.
#[cfg(feature = "codec")]
fn spatialize(input: &[f32; STEREO], output: &mut [f32; STEREO]) {
    // SAFETY: this runs only in the audio ISR; the statics below are touched
    // nowhere else, and `CONV` was initialised in `main` before `start` unmasked
    // the interrupt. Raw-pointer access avoids `static mut` reference UB.
    unsafe {
        let conv = match (*core::ptr::addr_of_mut!(CONV)).as_mut() {
            Some(c) => c,
            None => {
                output.fill(0.0);
                return;
            }
        };
        let in_fifo = &mut *core::ptr::addr_of_mut!(IN_FIFO);
        let out_l = &mut *core::ptr::addr_of_mut!(OUT_L);
        let out_r = &mut *core::ptr::addr_of_mut!(OUT_R);

        // Mono source = LEFT input channel (patch the mono guitar to the left in).
        for f in 0..FRAMES {
            in_fifo.push(input[2 * f]);
        }
        // Convolve every full 64-block that is ready.
        let mut blk = [0.0f32; B];
        let (mut ol, mut or) = ([0.0f32; B], [0.0f32; B]);
        while in_fifo.pop(&mut blk) {
            conv.process_block(&blk, &mut ol, &mut or);
            out_l.extend(&ol);
            out_r.extend(&or);
        }
        // Emit one callback's worth of stereo, or silence until primed.
        let (mut cl, mut cr) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        if out_l.len() >= FRAMES {
            out_l.pop(&mut cl);
            out_r.pop(&mut cr);
            for f in 0..FRAMES {
                output[2 * f] = cl[f];
                output[2 * f + 1] = cr[f];
            }
        } else {
            output.fill(0.0);
        }
    }
}

/// Build the HRIR convolver into its static scratch. Call once, before starting
/// audio (the ISR is still masked).
fn init_convolver() {
    // SAFETY: single-threaded init; `CONV_SCRATCH` is borrowed exactly once here
    // (it then lives for the program's life inside `CONV`).
    unsafe {
        let scratch: &'static mut [f32] = &mut *core::ptr::addr_of_mut!(CONV_SCRATCH);
        let conv = StereoConvolver::new(&HRIR_BEHIND_LEFT_L, &HRIR_BEHIND_LEFT_R, B, 1.0, scratch);
        *core::ptr::addr_of_mut!(CONV) = Some(conv);
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
    init_convolver(); // still exercises the convolver build + static scratch
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
    init_convolver();

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

#![no_std]
#![no_main]
#![allow(deprecated)] // cortex-m-rt 0.7 #[pre_init]; migrate later.

//! Binaural spatializer — **phase 4**: live positioning from the Hothouse knobs.
//! Source 1 (left in) is **movable** — a knob crossfades it live between
//! behind-left and behind-right; source 2 (right in) sits fixed in **front**;
//! both go through one shared room whose distance / reverb / level also track the
//! knobs. On headphones the guitar sweeps around behind you as you turn the knob.
//!
//! Signal path (in the DMA1_STR1 audio callback, 48-frame stereo blocks):
//! ```text
//!   input L ─► Voice1a [HRIR behind-LEFT ]─┐ crossfade
//!          └─► Voice1b [HRIR behind-RIGHT]─┴─(azimuth knob)─┐ src1
//!   input R ─► Voice2  [HRIR FRONT       ]──────────────────┤─► ×gains ─► out L/R
//!   ½(L+R) ─► air-abs LP ─┬► EarlyReflections ──────────────┤ shared room
//!                         └► FdnReverb (late) ───────────────┘ (distance/reverb)
//! ```
//! Each **HRIR** ([`daisy_dsp::convolution::StereoConvolver`], fully wet) gives
//! *direction*: the measured IRs carry the interaural time/level/pinna cues a
//! pan/delay cannot. Live movement is a **crossfade between two fixed HRIR
//! positions** — real-time-safe (only gains change, no convolver rebuild) and the
//! economical realization of "one movable source" (~3 convolvers total). The
//! **shared room** ([`daisy_dsp::room::EarlyReflections`] — the strongest
//! externalization cue — + [`daisy_dsp::reverb::FdnReverb`] + an air-absorption
//! low-pass) gives *distance / externalization*. Knobs: K1 = azimuth, K2 =
//! distance, K3 = reverb, K6 = master; the main loop reads ADC1 and publishes the
//! values through atomics the ISR reads each block.
//!
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
//! Phases 1-4 of `docs/binaural-spatializer.md`. The movable source is a 2-point
//! crossfade; a denser HRIR set + true interpolation (and SDRAM storage) is the
//! phase-5 stretch.

mod hrir_data;

use cortex_m_rt::{entry, pre_init};
use daisy_bsp::hal;
use hal::pac;
#[cfg(all(feature = "codec", not(feature = "renode_test")))]
use hal::prelude::*; // RccExt::constrain, GpioExt::split, SAI kernel mux
use panic_halt as _;

#[cfg(feature = "codec")]
use core::sync::atomic::{AtomicU32, Ordering};

use daisy_dsp::convolution::StereoConvolver;
use daisy_dsp::filter::OnePole;
use daisy_dsp::reblock::SampleFifo;
use daisy_dsp::reverb::FdnReverb;
use daisy_dsp::room::{room_taps, EarlyReflections, Tap};
use hrir_data::{
    HRIR_BEHIND_LEFT_L, HRIR_BEHIND_LEFT_R, HRIR_BEHIND_RIGHT_L, HRIR_BEHIND_RIGHT_R, HRIR_FRONT_L,
    HRIR_FRONT_R,
};

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

// --- room parameters (rt60 / damping / air-cutoff fixed; levels are live) ----
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
// --- live parameters (Hothouse knobs) ----------------------------------------
// The main loop reads the ADC and publishes normalized 0..1 knob values here;
// the audio ISR reads them each block. f32 stored as its bits in an AtomicU32 —
// a lock-free single-writer/single-reader hand-off (Relaxed: each value is
// independent, and a 32-bit load/store is atomic, so no torn reads).
#[cfg(feature = "codec")]
static AZIMUTH: AtomicU32 = AtomicU32::new(0x3F00_0000); // 0.5 — src1 behind-L↔R
#[cfg(feature = "codec")]
static DISTANCE: AtomicU32 = AtomicU32::new(0x3E99_999A); // 0.3 — 0 near … 1 far
#[cfg(feature = "codec")]
static REVERB: AtomicU32 = AtomicU32::new(0x3F00_0000); // 0.5 — room amount
#[cfg(feature = "codec")]
static MASTER: AtomicU32 = AtomicU32::new(0x3F4C_CCCD); // 0.8 — output level

// Only the Hothouse (non-seed3) knob loop writes params; Seed 3 has no knobs and
// runs on the defaults above.
#[cfg(all(feature = "codec", not(feature = "seed3")))]
fn store_param(a: &AtomicU32, v: f32) {
    a.store(v.to_bits(), Ordering::Relaxed);
}
#[cfg(feature = "codec")]
fn load_param(a: &AtomicU32) -> f32 {
    f32::from_bits(a.load(Ordering::Relaxed))
}

// --- DSP state: backing buffers + the objects that borrow them ---------------
// Three convolver scratches: source 1 is a crossfade between TWO HRIR positions
// (behind-left ↔ behind-right, the movable source), source 2 is one fixed
// position (front).
static mut CONV_SCRATCH_1A: [f32; SCRATCH] = [0.0; SCRATCH];
static mut CONV_SCRATCH_1B: [f32; SCRATCH] = [0.0; SCRATCH];
static mut CONV_SCRATCH_2: [f32; SCRATCH] = [0.0; SCRATCH];
static mut REVERB_BUF: [f32; FdnReverb::REQUIRED_BUF] = [0.0; FdnReverb::REQUIRED_BUF];
static mut EARLY_BUF: [f32; EARLY_BUF_LEN] = [0.0; EARLY_BUF_LEN];
static mut ROOM_TAPS: [Tap; NTAPS] = [Tap {
    delay: 0.0,
    gain_l: 0.0,
    gain_r: 0.0,
}; NTAPS];

/// One HRIR convolver + the 48↔64 reblock FIFOs that feed it. `process` takes a
/// 48-frame mono block and fills the 48-frame spatialized stereo; the output
/// buffers are left at silence until the reblock+convolver pipeline primes.
#[cfg_attr(not(feature = "codec"), allow(dead_code))]
struct Voice {
    conv: StereoConvolver<'static>,
    in_fifo: SampleFifo<128>, // covers the largest 48/64 carry (< 2 blocks)
    out_l: SampleFifo<256>,
    out_r: SampleFifo<256>,
}

#[cfg(feature = "codec")]
impl Voice {
    /// Push one 48-frame mono block; fill `(dl, dr)` with the spatialized stereo.
    /// Leaves `dl`/`dr` untouched (so the caller's zeroed buffers stay silent)
    /// until the pipeline has primed.
    fn process(&mut self, mono: &[f32], dl: &mut [f32], dr: &mut [f32]) {
        self.in_fifo.extend(mono);
        let mut blk = [0.0f32; B];
        let (mut ol, mut or) = ([0.0f32; B], [0.0f32; B]);
        while self.in_fifo.pop(&mut blk) {
            self.conv.process_block(&blk, &mut ol, &mut or);
            self.out_l.extend(&ol);
            self.out_r.extend(&or);
        }
        if self.out_l.len() >= dl.len() {
            self.out_l.pop(dl);
            self.out_r.pop(dr);
        }
    }
}

/// The full render: a MOVABLE source (source 1, crossfaded live between
/// behind-left and behind-right by the azimuth knob) + a FIXED source (source 2,
/// front), summed, plus one SHARED room (early reflections, late reverb,
/// air-absorption) fed by the source mix. Sharing the room is both physically
/// right (one room) and the economical choice (one reverb, not three).
#[cfg_attr(not(feature = "codec"), allow(dead_code))]
struct Dsp {
    voice1a: Voice, // source 1 endpoint A: behind-left
    voice1b: Voice, // source 1 endpoint B: behind-right
    voice2: Voice,  // source 2 (fixed): front
    early: EarlyReflections<'static>,
    reverb: FdnReverb<'static>,
    air_lp: OnePole,
}
static mut DSP: Option<Dsp> = None;

/// Frames per codec callback, and the interleaved stereo block length.
#[cfg(feature = "codec")]
const FRAMES: usize = daisy_audio::BLOCK_SIZE;
#[cfg(feature = "codec")]
const STEREO: usize = 2 * FRAMES;

/// The audio callback: two mono inputs → HRIR *directions* (source 1 live-movable
/// via crossfade) + a shared *room*, all under the live knob params → stereo out.
/// Runs in the DMA1_STR1 interrupt.
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

        // Live knob params (published by the main loop).
        let az = load_param(&AZIMUTH).clamp(0.0, 1.0); // src1 behind-L(0) ↔ behind-R(1)
        let dist = load_param(&DISTANCE).clamp(0.0, 1.0); // 0 near … 1 far
        let rev = load_param(&REVERB).clamp(0.0, 1.0);
        let master = load_param(&MASTER).clamp(0.0, 1.0);
        // Distance shapes the dry/wet balance (closer = drier, farther = wetter);
        // the reverb knob scales the late tail; master is the overall level.
        let dry = master * (0.9 - 0.55 * dist);
        let early_g = master * (0.25 + 0.45 * dist);
        let late_g = master * (0.15 + 0.5 * dist) * (0.2 + 0.8 * rev);

        // Two mono sources: left input → src1, right input → src2.
        let (mut m1, mut m2) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        for f in 0..FRAMES {
            m1[f] = input[2 * f];
            m2[f] = input[2 * f + 1];
        }

        // Direct HRIR paths. Source 1 runs both crossfade endpoints (fed the same
        // mono); source 2 its one fixed position. Zeroed buffers stay silent until
        // each voice primes.
        let (mut a_l, mut a_r) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        let (mut b_l, mut b_r) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        let (mut c_l, mut c_r) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        dsp.voice1a.process(&m1, &mut a_l, &mut a_r);
        dsp.voice1b.process(&m1, &mut b_l, &mut b_r);
        dsp.voice2.process(&m2, &mut c_l, &mut c_r);

        // Shared room, fed by the source mix → air-absorption LP → early + late.
        let mut room_in = [0.0f32; FRAMES];
        for f in 0..FRAMES {
            room_in[f] = 0.5 * (m1[f] + m2[f]);
        }
        let mut send = [0.0f32; FRAMES];
        dsp.air_lp.process(&room_in, &mut send);
        let (mut el, mut erf) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        dsp.early.process(&send, &mut el, &mut erf);
        let (mut rl, mut rr) = ([0.0f32; FRAMES], [0.0f32; FRAMES]);
        dsp.reverb.process(&send, &mut rl, &mut rr);

        // Mix: source 1 (crossfaded A↔B) + source 2, then the shared room.
        for f in 0..FRAMES {
            let s1_l = (1.0 - az) * a_l[f] + az * b_l[f];
            let s1_r = (1.0 - az) * a_r[f] + az * b_r[f];
            let dir_l = s1_l + c_l[f];
            let dir_r = s1_r + c_r[f];
            output[2 * f] = dry * dir_l + early_g * el[f] + late_g * rl[f];
            output[2 * f + 1] = dry * dir_r + early_g * erf[f] + late_g * rr[f];
        }
    }
}

/// Build a `Voice` from an HRIR pair over the given static scratch.
fn make_voice(scratch: &'static mut [f32], ir_l: &[f32], ir_r: &[f32]) -> Voice {
    Voice {
        conv: StereoConvolver::new(ir_l, ir_r, B, 1.0, scratch),
        in_fifo: SampleFifo::new(),
        out_l: SampleFifo::new(),
        out_r: SampleFifo::new(),
    }
}

/// Build the whole DSP render into its static buffers. Call once, before starting
/// audio (the ISR is still masked).
fn init_dsp() {
    // SAFETY: single-threaded init; each static buffer is borrowed exactly once
    // (they then live for the program's life inside `DSP`).
    unsafe {
        let voice1a = make_voice(
            &mut *core::ptr::addr_of_mut!(CONV_SCRATCH_1A),
            &HRIR_BEHIND_LEFT_L,
            &HRIR_BEHIND_LEFT_R,
        );
        let voice1b = make_voice(
            &mut *core::ptr::addr_of_mut!(CONV_SCRATCH_1B),
            &HRIR_BEHIND_RIGHT_L,
            &HRIR_BEHIND_RIGHT_R,
        );
        let voice2 = make_voice(
            &mut *core::ptr::addr_of_mut!(CONV_SCRATCH_2),
            &HRIR_FRONT_L,
            &HRIR_FRONT_R,
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
            voice1a,
            voice1b,
            voice2,
            early,
            reverb,
            air_lp,
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

    // Classic Daisy Seed in a Hothouse: daisy-audio auto-detects the codec
    // (AK4556 / WM8731 / PCM3060) on SAI1, and the six panel knobs read on ADC1.
    #[cfg(not(feature = "seed3"))]
    {
        let gpioa = dp.GPIOA.split(rec.GPIOA);
        let gpiob = dp.GPIOB.split(rec.GPIOB);
        let gpioc = dp.GPIOC.split(rec.GPIOC);
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
        let mut audio = daisy_audio::Audio::new(
            dp.SAI1, dp.DMA1, rec.DMA1, sai1_rec, dp.I2C2, rec.I2C2, pins, &clocks,
        );

        // Hothouse knobs (all on ADC1). SysTick drives the ADC power-up delay.
        let mut knobs = daisy_bsp::hothouse::Knobs::new(
            gpioa.pa3, gpiob.pb1, gpioa.pa7, gpioa.pa6, gpioc.pc1, gpioc.pc4,
        );
        let cp = unsafe { cortex_m::Peripherals::steal() };
        let mut delay = cp.SYST.delay(clocks);
        let adc1 = hal::adc::Adc::adc1(dp.ADC1, 4.MHz(), &mut delay, rec.ADC12, &clocks);
        let mut adc1 = adc1.enable();

        audio.start(spatialize);

        // Poll the knobs and publish the params the audio ISR reads. K1 = azimuth
        // (source-1 behind-left↔right), K2 = distance, K3 = reverb, K6 = master
        // (K4/K5 reserved for later phases). ~200 Hz is ample for a control knob.
        loop {
            let k = knobs.read_all(&mut adc1);
            store_param(&AZIMUTH, k[0]);
            store_param(&DISTANCE, k[1]);
            store_param(&REVERB, k[2]);
            store_param(&MASTER, k[5]);
            delay.delay_ms(5u32);
        }
    }

    // Seed 3 (TAC5242): hardware-strapped, SAI-only, no I2C, no Hothouse knobs.
    #[cfg(feature = "seed3")]
    {
        let gpioe = dp.GPIOE.split(rec.GPIOE);
        let pins = daisy_audio::Pins {
            mclk_a: gpioe.pe2,
            sck_a: gpioe.pe5,
            fs_a: gpioe.pe4,
            sd_a: gpioe.pe6,
            sd_b: gpioe.pe3,
        };
        let mut audio =
            daisy_audio::Audio::new(dp.SAI1, dp.DMA1, rec.DMA1, sai1_rec, pins, &clocks);
        audio.start(spatialize);
        loop {
            cortex_m::asm::wfi();
        }
    }
}

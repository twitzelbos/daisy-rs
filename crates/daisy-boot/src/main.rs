#![no_std]
#![no_main]
// The `renode_test` feature skips USB init and the alive-blink loop,
// leaving the DFU/USB helper items unused. They're kept in-source
// because the default (hardware) build needs them — silence the
// dead-code warnings only when building for Renode.
#![cfg_attr(feature = "renode_test", allow(dead_code, unused_imports))]
// The boot-flow overview and the QSPI errata register sequence in the docs
// below use deliberate hand-aligned ASCII; clippy's doc-list reindenting would
// mangle that alignment (and fights rustfmt). Allow the two stylistic doc lints.
#![allow(clippy::doc_lazy_continuation, clippy::doc_overindented_list_items)]

//! daisy-boot — Rust bootloader for the Daisy Seed.
//!
//! Boot flow:
//!
//!   1. Sample the "stay in bootloader" magic word at the base of Backup
//!      SRAM. Apps write it before requesting SYSRESETREQ when they need
//!      to be reprogrammed. We consume-and-clear so a second reset boots
//!      normally.
//!   2. Init clocks to 400 MHz sysclk, HSI48 for USB, GPIO for LED + QSPI
//!      + USB. QSPI is brought up in memory-mapped mode so we can read
//!      the app's vector table below.
//!   3. Snapshot MSP + reset vector at 0x9000_0000 and hand them to
//!      `daisy_bsp::boot_check::is_plausible_image`.
//!   4. If the magic is set OR the QSPI slot is empty/corrupt, go straight
//!      to service mode.
//!   5. Otherwise blink the alive LED four times (~2 s) while polling USB
//!      DFU. If a host actually sends a DFU command in that window, stay
//!      in service mode. Otherwise jump to the app.
//!
//! Service mode: run `usbd-dfu` against the IS25LP064A via `QspiDfuMem`
//! forever; the LED slow-blinks at 1 Hz to indicate "waiting for host".

use cortex_m_rt::entry;
use panic_halt as _;

use daisy_bsp::boot_check::{is_plausible_image, APP_BASE};
use daisy_bsp::hal;
use daisy_bsp::hal::prelude::*;
use hal::pac;
use hal::rcc::rec::UsbClkSel;
// USB2 is the STM32H7's OTG2_HS peripheral — but the Daisy Seed's on-board
// micro-USB connector is wired to PA11/PA12 (AF10 = OTG2_HS FS mode via
// internal FS PHY), NOT to PB14/PB15 (which are OTG1_HS on the D29/D30
// breakout pins, not the micro connector). Confirmed by the Daisy Seed
// pinout CSV in electro-smith/DaisyWiki and by daisy-embassy's usb.rs.
use hal::usb_hs::{UsbBus, USB2};

use usb_device::prelude::*;
use usbd_dfu::DFUClass;

use core::sync::atomic::{AtomicU32, Ordering};

mod dfu_mem;
mod qspi;

use dfu_mem::QspiDfuMem;

/// Runtime sysclk in Hz — the actual frozen `sys_ck` (400 MHz on rev Y / non-
/// rev-V, 480 MHz on rev V), stored by `run()` right after `clocks::init`. The
/// DWT-based `delay_ms`/`delay_us` read it, so they're exact on whichever clock
/// the silicon-revision gate chose. Defaults to the 400 MHz minimum for any
/// (currently none) delay that might run before the store.
pub static SYSCLK_HZ: AtomicU32 = AtomicU32::new(400_000_000);

/// Block for `ms` milliseconds using DWT_CYCCNT as the precise wall-clock
/// reference. `cortex_m::asm::delay` alone takes 1–4 cycles per iteration
/// depending on dual-issue and instruction prefetch state, so its wall-
/// clock time varies with fetch bandwidth (roughly 2× slower when running
/// from QSPI XIP without I-cache). DWT ticks at sysclk with zero jitter.
///
/// Implementation: check DWT_CYCCNT to decide when to exit, but burn
/// `BURN_CHUNK` cycles of asm-loop time between checks so Renode simulates
/// the delay ~1000× faster than a tight DWT poll. Overshoot bound:
/// `BURN_CHUNK / (SYSCLK_HZ / 1_000_000)` µs = ~10 µs at 400 MHz.
///
/// DWT_CYCCNT is 32-bit → wraps every ~10.7 s at 400 MHz. Callers must
/// keep single delays under that; use several calls for longer periods.
pub fn delay_ms(ms: u32) {
    let n = ms.saturating_mul(SYSCLK_HZ.load(Ordering::Relaxed) / 1000);
    delay_cycles(n);
}

/// Block for `us` microseconds. Same DWT+burn pattern as `delay_ms`.
pub fn delay_us(us: u32) {
    let n = us.saturating_mul(SYSCLK_HZ.load(Ordering::Relaxed) / 1_000_000);
    delay_cycles(n);
}

/// STM32H750 ES0392 §2.7.4 "QUADSPI internal timing criticality" workaround.
///
/// **Not currently needed.** Our first hardware failure looked like a
/// timing-criticality issue but was actually the alternate-bytes /
/// mode-bits phase being missing in `qspi::enter_memory_mapped` (see
/// IS25LP064A datasheet §8.7). Keeping this function around because it
/// remains a documented workaround for real-world QUADSPI hangs, and if
/// we ever hit those symptoms again we already have the sequence.
#[allow(dead_code)]
/// Original errata description follows.
/// workaround. Errata verbatim: "The timing of some internal signals of
/// the QUADSPI peripheral is critical. At certain conditions, this can
/// lead to a general failure of the peripheral. As these conditions
/// cannot be exactly determined, it is recommended to systematically
/// apply the workaround as described."
///
/// Sequence (all writes to QSPI_CR at 0x5200_5000 and QSPI_CCR at
/// 0x5200_5014):
///   1. CR = 0                        ; disable, ensure not at max prescaler
///   2. wait for SR.BUSY == 0
///   3. CR = 0xFF000001               ; max prescaler + EN=1
///   4. CCR = 0x2000_0000             ; activate free-running clock
///   5. CCR = 0x2000_0000             ; repeat (per errata, prevent back-to-back)
///   6. CR = 0                        ; disable — MUST complete <127 kernel
///                                      clocks after step 4
///   7. wait for SR.BUSY == 0
///
/// The tight window in step 6 is achieved by putting steps 4-6 into a
/// single inline-asm block with `nostack, nomem` so the compiler can't
/// reorder or spill.
///
/// Errata says apply "upon reset AND upon switching from memory-mapped
/// to any other mode".
pub fn qspi_errata_2_7_4_workaround() {
    const QSPI_CR: *mut u32 = 0x5200_5000 as *mut u32;
    const QSPI_SR: *const u32 = 0x5200_5008 as *const u32;
    const QSPI_CCR: *mut u32 = 0x5200_5014 as *mut u32;
    unsafe {
        // Step 1 + 2: disable QSPI, wait for BUSY == 0.
        core::ptr::write_volatile(QSPI_CR, 0);
        while (core::ptr::read_volatile(QSPI_SR) & 0x20) != 0 {}
        // Steps 3-7 in one inline-asm block — the critical timing window
        // between step 4's first CCR write and step 6's CR disable must
        // fit in <127 kernel clocks. Constants loaded via literal pool
        // (`ldr =imm32`) because Thumb-2 `mov imm` can't encode 0xFF000001
        // or 0x20000000 directly.
        core::arch::asm!(
            "ldr {cr_addr}, =0x52005000",
            "ldr {ccr_addr}, =0x52005014",
            "ldr {tmp}, =0xFF000001",
            "str {tmp}, [{cr_addr}]",       // step 3: CR = 0xFF000001
            "ldr {tmp}, =0x20000000",
            "str {tmp}, [{ccr_addr}]",      // step 4: CCR = free-running
            "str {tmp}, [{ccr_addr}]",      // step 5: repeat
            "mov {tmp}, #0",
            "str {tmp}, [{cr_addr}]",       // step 6: disable QSPI
            cr_addr = out(reg) _,
            ccr_addr = out(reg) _,
            tmp = out(reg) _,
            options(nomem, nostack),
        );
        // Step 7: wait for BUSY == 0 (outside the tight-timing window).
        while (core::ptr::read_volatile(QSPI_SR) & 0x20) != 0 {}
    }
}

/// Block until DWT_CYCCNT has advanced by at least `n` cycles from now.
/// Uses `asm::delay(BURN_CHUNK)` between each DWT read (Renode simulates
/// this ~1000× faster than a tight DWT poll). Also caps the iteration
/// count so a stalled DWT counter can't infinite-loop us — in that
/// pathological case the loop still burns ~n cycles via `asm::delay`
/// and exits, giving an imprecise-but-finite delay.
fn delay_cycles(n: u32) {
    // 4096 cycles at 400 MHz is ~10 µs — small overshoot, ~1000× fewer
    // DWT reads than a tight poll.
    const BURN_CHUNK: u32 = 4096;
    let start = cortex_m::peripheral::DWT::cycle_count();
    let max_iters = (n / BURN_CHUNK).saturating_add(64).saturating_mul(3);
    let mut iters = 0u32;
    loop {
        if cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start) >= n {
            return;
        }
        if iters >= max_iters {
            return; // DWT stalled — bail out, don't hang.
        }
        iters = iters.wrapping_add(1);
        cortex_m::asm::delay(BURN_CHUNK);
    }
}

// The "stay in DFU" request flag an app sets before resetting lives in Backup
// SRAM; its address + magic + read/write helpers are in `daisy_bsp::reset`
// (flag at 0x3880_0FFC, clear of the clocks hand-off struct at 0x3880_0000).
// `daisy_seed_helpers.py` (Renode) plants it there for reset-into-DFU tests.

/// USB VID:PID. Reusing STM's ROM-DFU identifiers so `daisy flash`
/// (which searches for 0483:df11) finds us with no config change.
/// TODO: migrate to a pid.codes-assigned VID once we're closer to shipping.
const VID: u16 = 0x0483;
const PID: u16 = 0xdf11;

/// USB endpoint FIFO memory. 1 KiB is plenty for one interface with a
/// 128 B control endpoint and no data endpoints. Zero-initialised so we
/// never take a shared reference to uninitialised memory.
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

/// 8-second boot window at 400 MHz sysclk. Long enough to `daisy list`
/// and start `daisy flash` from a human-driven shell; short enough to
/// stay safely under the DWT cycle counter's 32-bit wraparound
/// (~10.7 s @ 400 MHz). Once DFU + hardware bring-up is stable we'll
/// shorten this to a couple seconds — or switch to a 64-bit millisecond
/// tick if we ever want longer.
// Nominal at 400 MHz. These are margins (DFU window) / cosmetic (blink rates),
// so they stay compile-time literals: on a rev-V board running at 480 the window
// is ~6.7 s (still ample) and the blink ~20% faster (imperceptible), and both
// stay well under the DWT 32-bit wrap. The *delays* (delay_ms/delay_us) ARE made
// runtime-exact via the SYSCLK_HZ atomic, since QSPI settle timing needs it.
const BOOT_WINDOW_CYCLES: u32 = 400_000_000 * 8;
/// ~250 ms per LED half-cycle → ~2 Hz blink.
const ALIVE_HALF_PERIOD_CYCLES: u32 = 400_000_000 / 4;
/// ~500 ms per LED half-cycle → ~1 Hz service-mode blink.
const SERVICE_HALF_PERIOD_CYCLES: u32 = 400_000_000 / 2;

#[entry]
fn main() -> ! {
    // ES0392 §2.2.10 workaround — silicon revisions before Rev V had a
    // multi-master AXI SRAM read corruption. Set the AXI switch matrix
    // read-issuing capability to 1 for target 7 (AXI SRAM). Mirror of
    // libDaisy's early-init: `system_stm32h7xx.c:233-235`.
    //
    // DBGMCU->IDCODE bits [31:16] encode the revision (0x1xxx = Rev Y
    // and earlier, 0x2xxx = Rev V and later). Applying on newer silicon
    // is harmless — the register write is idempotent and the field is
    // already 1 on those parts — but we match libDaisy's gate exactly.
    unsafe {
        const DBGMCU_IDCODE: *const u32 = 0x5C00_1000 as *const u32;
        const AXI_TARG7_FN_MOD: *mut u32 = 0x5100_8108 as *mut u32;
        if core::ptr::read_volatile(DBGMCU_IDCODE) & 0xFFFF_0000 < 0x2000_0000 {
            core::ptr::write_volatile(AXI_TARG7_FN_MOD, 0x0000_0001);
        }
    }

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Enable the L1 I-cache. At 480 MHz / VOS0 internal flash runs at ~5 wait
    // states, so UNCACHED the bootloader executes several× slower — too slow to
    // drain the OTG RX FIFO during USB enumeration, which stalls the DFU
    // control-transfer exchange (the device connects + the host resets it, but
    // enumeration never completes). I-cache keeps fetches fast enough. Internal
    // flash is cacheable under the default memory map, so no MPU is needed;
    // D-cache stays OFF (the bootloader touches QSPI/backup SRAM without
    // cache maintenance). Harmless at 400 MHz too.
    cp.SCB.enable_icache();

    // Whether to force DFU service mode is read from the Backup-SRAM flag an app
    // sets via `daisy_bsp::reset::reboot_to_bootloader` — but the read must wait
    // until RCC_AHB4ENR.BKPRAMEN is enabled (reading Backup SRAM before that
    // hard-faults). `handoff::stash` (below) enables it, so the flag is consumed
    // just after the stash.

    // Enable the DWT cycle counter for our boot-window timer.
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    let mut ccdr = daisy_bsp::clocks::init(dp.PWR, dp.RCC, &dp.SYSCFG);
    // Record the actual frozen sysclk (400 or 480 MHz, chosen by the rev gate in
    // clocks::init) so the DWT delays below are exact on this silicon.
    SYSCLK_HZ.store(ccdr.clocks.sys_ck().raw(), Ordering::Relaxed);

    // Stash the frozen CoreClocks into Backup SRAM so the QSPI-XIP app (which
    // runs post-freeze and can't mint its own) can recover it and use HAL
    // drivers that need `&CoreClocks` (SAI, I2C). Also enables BKPRAMEN, so the
    // Backup SRAM is now safe to read (resolves the TODO above).
    unsafe { daisy_bsp::clocks::handoff::stash(&ccdr.clocks) };

    // Backup SRAM is now clocked (stash enabled BKPRAMEN). Consume any "stay in
    // DFU" request an app left before resetting — a sealed pedal's only way into
    // DFU without the RESET button. Consume-and-clear so the *next* reset boots
    // the app normally.
    let force_dfu = unsafe { daisy_bsp::reset::take_dfu_request() };

    // USB kernel clock: HSI48 (48 MHz internal RC). `freeze()` enables it
    // by default; we just point USB1's kernel mux at it.
    let _ = ccdr
        .clocks
        .hsi48_ck()
        .expect("HSI48 must be running for USB");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

    let mut led = daisy_bsp::led::user_led(gpioc.pc7);

    // Enable the USB VDD33 voltage-level detector. Daisy Seed supplies
    // VDD33USB *externally* (confirmed by libDaisy's system.cpp calling
    // HAL_PWREx_EnableUSBVoltageDetector() — that maps to setting
    // USB33DEN, not USBREGEN). If we enabled the internal regulator
    // (USBREGEN) instead, it would fight the external rail and USB33RDY
    // would never assert, hanging this poll forever.
    unsafe {
        let pwr = &*pac::PWR::ptr();
        pwr.cr3.modify(|_, w| w.usb33den().set_bit());
        while pwr.cr3.read().usb33rdy().bit_is_clear() {}
    }

    // QSPI into memory-mapped mode so we can inspect the app's vector table.
    qspi::configure_pins(gpiof, gpiog);
    qspi::init_memory_mapped(dp.QUADSPI, ccdr.peripheral.QSPI);

    // Snapshot MSP + reset vector while the QSPI window is readable.
    let vt = APP_BASE as *const u32;
    let msp = unsafe { core::ptr::read_volatile(vt) };
    let reset = unsafe { core::ptr::read_volatile(vt.add(1)) };
    let qspi_looks_valid = is_plausible_image(msp, reset);

    // We need the raw QUADSPI back if we're going to enter DFU service
    // mode (QspiDfuMem consumes it). Steal it from the PAC — the HAL
    // already gave up ownership in init_memory_mapped, and no other code
    // touches it after this point.
    let quadspi = unsafe { pac::Peripherals::steal().QUADSPI };

    #[cfg(not(feature = "renode_test"))]
    let (pin_dm, pin_dp) = {
        // GPIOs for the on-board micro-USB — PA11/PA12 with AF10 (OTG2_HS
        // in FS mode). See usb_hs::USB2::new signature in stm32h7xx-hal 0.16.
        (
            gpioa.pa11.into_alternate::<10>(),
            gpioa.pa12.into_alternate::<10>(),
        )
    };
    #[cfg(feature = "renode_test")]
    let _ = &gpioa; // keep the binding used

    if force_dfu || !qspi_looks_valid {
        #[cfg(not(feature = "renode_test"))]
        {
            // Skip the boot window; user wants DFU (or has no app to run).
            enter_service_mode(
                led,
                quadspi,
                dp.OTG2_HS_GLOBAL,
                dp.OTG2_HS_DEVICE,
                dp.OTG2_HS_PWRCLK,
                pin_dm,
                pin_dp,
                ccdr.peripheral.USB2OTG,
                &ccdr.clocks,
            )
        }
        #[cfg(feature = "renode_test")]
        {
            // In Renode we can't do USB; run the alive-blink forever so
            // the test can observe LED toggling as proof of clocks::init
            // completing.
            let _ = quadspi;
            renode_alive_blink_forever(&mut led);
        }
    }

    // Valid app in QSPI. Bring up USB and give a host 3 seconds to interrupt
    // us. Blink 4 times at 2 Hz during that window as our "alive" indicator.
    #[cfg(not(feature = "renode_test"))]
    let (mut dfu, mut usb_dev, usb_bus_alloc);
    #[cfg(not(feature = "renode_test"))]
    {
        let usb2 = USB2::new(
            dp.OTG2_HS_GLOBAL,
            dp.OTG2_HS_DEVICE,
            dp.OTG2_HS_PWRCLK,
            pin_dm,
            pin_dp,
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks,
        );
        let ep_mem = init_ep_memory();
        usb_bus_alloc = UsbBus::new(usb2, ep_mem);
        dfu = DFUClass::new(&usb_bus_alloc, QspiDfuMem::new(quadspi));
        usb_dev = build_usb_device(&usb_bus_alloc);
    }
    #[cfg(feature = "renode_test")]
    {
        // Under Renode: no USB. Do a brief alive-blink (proof clocks::init
        // completed), re-enter memory-mapped mode, then jump to the app —
        // the same jump path hardware uses. This makes the bootloader-to-
        // app transition testable in Renode.
        let _ = dp.OTG2_HS_GLOBAL;
        let _ = dp.OTG2_HS_DEVICE;
        let _ = dp.OTG2_HS_PWRCLK;
        let _ = ccdr.peripheral.USB2OTG;
        renode_boot_and_jump(&mut led, &quadspi, vt);
    }

    #[cfg(not(feature = "renode_test"))]
    {
        let start = cortex_m::peripheral::DWT::cycle_count();
        let mut led_on = false;
        loop {
            let elapsed = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start);
            if elapsed >= BOOT_WINDOW_CYCLES {
                break;
            }

            // Toggle LED every 250 ms based on wall-clock cycles.
            let should_be_on = (elapsed / ALIVE_HALF_PERIOD_CYCLES).is_multiple_of(2);
            if should_be_on != led_on {
                if should_be_on {
                    led.set_high();
                } else {
                    led.set_low();
                }
                led_on = should_be_on;
            }

            usb_dev.poll(&mut [&mut dfu]);

            // Any DFU-level activity commits us to service mode. USB
            // enumeration alone (`state() == Configured`) is not enough — a
            // user with USB plugged in for power shouldn't get stuck.
            if dfu_saw_activity(&dfu) {
                // Commit to service mode IMMEDIATELY — do NOT block here. The
                // host has an in-flight DFU control transfer (the DNLOAD that
                // tripped this flag); any stretch where we stop calling
                // usb_dev.poll() stalls that transfer and the host cancels it
                // ("transfer was cancelled"). A previous blocking 6-pulse +
                // 500 ms blink here corrupted every flash for exactly this
                // reason. Leave the LED solid on as the "in service mode"
                // indicator and go straight into the polling loop.
                led.set_high();
                run_service_mode_loop(&mut led, &mut usb_dev, &mut dfu);
            }
        }

        // Assert a clean USB soft-disconnect before handing the bus to the app.
        // Our DFU device's D+ pullup was asserted for the entire boot window; if
        // we jump with it still on, the host never sees a disconnect and keeps
        // its stale view of us — so the APP, which re-inits the same OTG core to
        // address 0, is never enumerated (observed on HW: app runs fine, VTOR at
        // 0x9000_0000, no fault, but DCFG device address stays 0 and the core
        // sits SUSPENDED). Setting DCTL.SDIS drops the pullup; the ~6 s of
        // stage_pulses below then hold the disconnect long enough for the host to
        // register the removal, and the app re-asserts the pullup on its own USB
        // bringup, enumerating fresh. Raw write — usb_dev is no longer polled, so
        // the usb-device state machine won't fight us.
        const OTG2_DCTL: *mut u32 = 0x4008_0804 as *mut u32;
        unsafe {
            let v = core::ptr::read_volatile(OTG2_DCTL);
            core::ptr::write_volatile(OTG2_DCTL, v | (1 << 1)); // SDIS = 1
        }

        stage_pulses(&mut led, 2); // stage-1: alive-blink loop exited cleanly
        let mem = dfu.release();
        let quadspi = mem.release();
        stage_pulses(&mut led, 3); // stage-2: DFU/QSPI ownership recovered
        qspi::enter_memory_mapped(&quadspi);
        stage_pulses(&mut led, 4); // stage-3: memory-mapped mode re-entered
        jump_to_app(APP_BASE, vt);
    }
}

/// Renode-only: blink PC7 forever at 2 Hz (250 ms on/off). Used on the
/// invalid-app / force-dfu path where there's nothing to jump to.
#[cfg(feature = "renode_test")]
fn renode_alive_blink_forever(led: &mut daisy_bsp::led::UserLed) -> ! {
    loop {
        led.set_high();
        delay_ms(250);
        led.set_low();
        delay_ms(250);
    }
}

/// Renode-only: brief alive-blink, then jump to the app at `vt`. Exercises
/// exactly the same jump_to_app path hardware uses. If this succeeds in
/// Renode but fails on hardware, the bug is silicon-specific (QSPI XIP,
/// cache, silicon errata); if it fails in Renode, we have a repro.
#[cfg(feature = "renode_test")]
fn renode_boot_and_jump(
    led: &mut daisy_bsp::led::UserLed,
    quadspi: &pac::QUADSPI,
    vt: *const u32,
) -> ! {
    // 4 quick pulses so the test can see "bootloader was here".
    for _ in 0..4 {
        led.set_high();
        delay_ms(50);
        led.set_low();
        delay_ms(50);
    }
    // Renode's QSPI XIP window is a plain Memory.MappedMemory, so reads
    // at 0x9000_0000 already work without the QUADSPI controller being in
    // memory-mapped mode. We still call enter_memory_mapped() to keep
    // the code path parallel with the non-renode_test build. Note:
    // enter_memory_mapped now configures the alternate-bytes phase per
    // IS25LP064A datasheet §8.7 (0xA0 mode bits) — WITHOUT this the
    // flash chip's Fast Read Quad I/O state machine never sees the
    // required mode-bits clocks and instruction fetches from XIP fail.
    qspi::enter_memory_mapped(quadspi);
    jump_to_app(APP_BASE, vt);
}

/// Emit `count` LED pulses (200 ms on, 200 ms off) then a 1 s gap.
/// Deliberately slow so 2 vs 3 vs 4 pulse bursts are trivially
/// distinguishable to the human eye. Timing goes through DWT_CYCCNT
/// so it's precise regardless of dual-issue / QSPI XIP fetch state.
fn stage_pulses(led: &mut daisy_bsp::led::UserLed, count: u32) {
    for _ in 0..count {
        led.set_high();
        delay_ms(200);
        led.set_low();
        delay_ms(200);
    }
    delay_ms(1000); // visible gap between stages
}

fn dfu_saw_activity<B: usb_device::bus::UsbBus>(_dfu: &DFUClass<B, QspiDfuMem>) -> bool {
    // Commit to service mode only once a host has issued a real flash write
    // (buffer store / erase / program). We used to key off the DFUse address
    // pointer having moved from its initial value, but a host merely
    // *enumerating* the DFU interface — notably macOS, which has no DFU driver —
    // moved the pointer and tripped this spuriously, pinning us in service mode
    // so the QSPI app never booted while plugged into a Mac (observed on HW:
    // VTOR stuck at 0x0800_0000 past the boot window). Enumeration never touches
    // the flash-write paths, so `dfu_saw_write()` can't be tripped by it.
    dfu_mem::dfu_saw_write()
}

fn build_usb_device<'a, B: usb_device::bus::UsbBus>(
    bus: &'a usb_device::bus::UsbBusAllocator<B>,
) -> UsbDevice<'a, B> {
    UsbDeviceBuilder::new(bus, UsbVidPid(VID, PID))
        .strings(&[usb_device::device::StringDescriptors::default()
            .manufacturer("daisy-rs")
            .product("Daisy Rust Bootloader")
            .serial_number("0001")])
        .unwrap()
        // DFU is a self-powered device from the host's perspective — we
        // don't draw significant current until service mode runs.
        .max_power(100)
        .unwrap()
        .build()
}

/// Get a mutable reference to the endpoint FIFO memory. The static is
/// zero-initialised at load time, so this doesn't need to run any init.
fn init_ep_memory() -> &'static mut [u32; 1024] {
    // SAFETY: EP_MEMORY is only referenced from this single call site (the
    // one call to UsbBus::new); its lifetime lasts until the CPU resets.
    unsafe { &mut *core::ptr::addr_of_mut!(EP_MEMORY) }
}

/// Bring up USB and enter the "wait for DFU forever" loop. Used when the
/// QSPI slot is empty/corrupt or when the app requested reflash.
///
/// The parameter list is unfortunate — we take everything needed for USB
/// bring-up rather than trying to hand them through a builder — but
/// this only has one call site so the ugliness is contained.
#[allow(clippy::too_many_arguments)]
fn enter_service_mode(
    mut led: daisy_bsp::led::UserLed,
    quadspi: pac::QUADSPI,
    otg_global: pac::OTG2_HS_GLOBAL,
    otg_device: pac::OTG2_HS_DEVICE,
    otg_pwrclk: pac::OTG2_HS_PWRCLK,
    pin_dm: hal::gpio::gpioa::PA11<hal::gpio::Alternate<10>>,
    pin_dp: hal::gpio::gpioa::PA12<hal::gpio::Alternate<10>>,
    prec: hal::rcc::rec::Usb2Otg,
    clocks: &hal::rcc::CoreClocks,
) -> ! {
    let usb2 = USB2::new(
        otg_global, otg_device, otg_pwrclk, pin_dm, pin_dp, prec, clocks,
    );
    let ep_mem = init_ep_memory();
    let usb_bus_alloc = UsbBus::new(usb2, ep_mem);
    let mut dfu = DFUClass::new(&usb_bus_alloc, QspiDfuMem::new(quadspi));
    let mut usb_dev = build_usb_device(&usb_bus_alloc);
    run_service_mode_loop(&mut led, &mut usb_dev, &mut dfu);
}

/// Service-mode inner loop: poll DFU forever, blink LED at a rate that
/// **reflects the USB device state** so we can visually diagnose whether
/// the host is enumerating us. Never returns.
///
/// Blink rate encodes `UsbDeviceState`:
///   - Default (not yet addressed): 1 Hz (500 ms half)
///   - Addressed (host sent SET_ADDRESS): 3 Hz (~167 ms half)
///   - Configured (SET_CONFIGURATION done): 5 Hz (100 ms half)
///   - Suspend: LED steadily on
///
/// If the LED stays at 1 Hz forever, the host never even sees us on the
/// bus. Any other rate proves the peripheral is talking to a host.
fn run_service_mode_loop<B: usb_device::bus::UsbBus>(
    led: &mut daisy_bsp::led::UserLed,
    usb_dev: &mut UsbDevice<B>,
    dfu: &mut DFUClass<B, QspiDfuMem>,
) -> ! {
    use usb_device::device::UsbDeviceState;
    let start = cortex_m::peripheral::DWT::cycle_count();
    let mut led_on = false;
    loop {
        usb_dev.poll(&mut [dfu]);

        let half_period = match usb_dev.state() {
            UsbDeviceState::Default => SERVICE_HALF_PERIOD_CYCLES, // 500 ms → 1 Hz
            UsbDeviceState::Addressed => SERVICE_HALF_PERIOD_CYCLES / 3, // ~167 ms → 3 Hz
            UsbDeviceState::Configured => SERVICE_HALF_PERIOD_CYCLES / 5, // 100 ms → 5 Hz
            UsbDeviceState::Suspend => u32::MAX,                   // effectively steady
        };
        let elapsed = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start);
        let should_be_on = if half_period == u32::MAX {
            true
        } else {
            (elapsed / half_period).is_multiple_of(2)
        };
        if should_be_on != led_on {
            if should_be_on {
                led.set_high();
            } else {
                led.set_low();
            }
            led_on = should_be_on;
        }
    }
}

/// Retarget the vector table and jump to the app's reset handler. Never returns.
///
/// # Safety
/// Caller must have validated the words at `vt` (MSP + reset) point to a
/// plausible image; otherwise the CPU will fault immediately after the jump.
fn jump_to_app(base: u32, vt: *const u32) -> ! {
    // Match libDaisy's `__JUMPTOQSPI` naked-asm exactly — that pattern
    // is known to work on identical silicon (Daisy Seed + STM32H750 +
    // IS25LP064A). We used to do CPSID i, clear CONTROL.SPSEL, and DSB
    // /ISB pairs around this, but on hardware the app never executed
    // its first QSPI XIP instruction; libDaisy's simpler pattern uses
    // none of those, and works.
    //
    //   LDR R1, =0xE000ED00   ; SCB base
    //   LDR R0, =base         ; APP_BASE
    //   STR R0, [R1, #8]      ; SCB->VTOR = base
    //   LDR SP, [R0, #0]      ; SP  = *(base + 0)  — MSP
    //   LDR R0, [R0, #4]      ; R0  = *(base + 4)  — Reset handler
    //   BX  R0
    let _ = vt; // vt is used implicitly via the base pointer in the asm
    unsafe {
        core::arch::asm!(
            "ldr r1, =0xE000ED00",
            "str {base}, [r1, #8]",
            "ldr sp, [{base}, #0]",
            "ldr r0, [{base}, #4]",
            "bx r0",
            base = in(reg) base,
            options(noreturn, nostack)
        );
    }
}

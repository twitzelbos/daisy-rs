//! Standalone firmware that runs the real STM32H7 ADC bring-up so the
//! `STM32H7_ADC` Renode model is validated against the HAL, not just against a
//! hand-written register sequence.
//!
//! It mirrors daisy-hothouse's ADC init exactly: `clocks::init` (VOS1/400 MHz,
//! PLL2P = 40 MHz ADC kernel clock), then `Adc::adc1(...).enable()` — which
//! internally polls ISR.LDORDY, CR.ADCAL self-clear and ISR.ADRDY — then one
//! blocking one-shot conversion of PA3 (Daisy knob K1 = ADC1 channel 15).
//!
//! Progress is written to a DTCM marker at each step, and the converted value
//! to a second marker, so the Renode test can assert the whole bring-up
//! completed and that the conversion returned the value it injected for the
//! channel (via `adc1 SetChannelValue 15 <v>`).

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;

use bsp::hal::adc::Adc;
use bsp::hal::block;
use bsp::hal::hal::adc::OneShot as _;
use bsp::hal::pac;
use bsp::hal::prelude::*;
use daisy_bsp as bsp;

// DTCM markers the Renode test reads back.
const MARK_STAGE: *mut u32 = 0x2001_0000 as *mut u32; // progress stage
const MARK_VALUE: *mut u32 = 0x2001_0004 as *mut u32; // ADC conversion result

#[inline(always)]
fn mark(stage: u32) {
    unsafe { core::ptr::write_volatile(MARK_STAGE, stage) };
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    mark(0x10); // entered

    // Real clock tree — same as the app. PLL2P = 40 MHz is the ADC kernel clock
    // (≤ 80 MHz at VOS1, or Adc::adc1's assert panics).
    let ccdr = bsp::clocks::init(dp.PWR, dp.RCC, &dp.SYSCFG);
    mark(0x11); // clocks::init returned

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let mut pa3 = gpioa.pa3.into_analog(); // Daisy knob K1 = ADC1 channel 15
    mark(0x12); // GPIO ready

    let mut delay = cp.SYST.delay(ccdr.clocks);
    // The H7 ADC bring-up handshake runs inside here: DEEPPWD=0/ADVREGEN=1 →
    // poll LDORDY; ADCAL=1 → poll self-clear.
    let adc1 = Adc::adc1(
        dp.ADC1,
        4.MHz(),
        &mut delay,
        ccdr.peripheral.ADC12,
        &ccdr.clocks,
    );
    mark(0x13); // Adc::adc1 constructed (regulator + calibration done)
    let mut adc1 = adc1.enable(); // ADEN=1 → poll ADRDY
    mark(0x14); // ADC enabled

    // Blocking one-shot conversion. In the model this drives ADSTART → EOC and
    // returns the injected channel value from DR.
    let raw: u32 = block!(adc1.read(&mut pa3)).unwrap();
    unsafe { core::ptr::write_volatile(MARK_VALUE, raw) };
    mark(0x15); // conversion complete

    loop {
        cortex_m::asm::nop();
    }
}

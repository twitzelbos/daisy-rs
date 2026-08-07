//! Electrosmith **Daisy Pod** — the small host board: 2 pots, 2 buttons, a
//! rotary encoder (with push), two RGB LEDs, and a UART MIDI port.
//!
//! The control-to-Seed-pin map is transcribed from libDaisy's `daisy_pod.cpp`
//! (current pinout); the `D<n>→STM32` map is from libDaisy `daisy_seed.h`:
//!
//! | Control              | Seed pin      | STM32                     |
//! |----------------------|---------------|---------------------------|
//! | Knob 1 / 2 (pots)    | D21 / D15     | PC4 / PC0 (both ADC1)     |
//! | Button 1 / 2         | D27 / D28     | PG9 / PA2                 |
//! | Encoder A / B        | D26 / D25     | PD11 / PA0                |
//! | Encoder click        | D13           | PB6                       |
//! | LED 1 R / G / B      | D20 / D19 / D18 | PC1 / PA6 / PA7         |
//! | LED 2 R / G / B      | D17 / D24 / D23 | PB1 / PA1 / PA4         |
//! | MIDI (USART1)        | tx D13 / rx D14 | PB6 / PB7 @ 31250 baud  |
//!
//! **About the PB6 overlap:** USART1 *TX* (PB6) is the same pin as the encoder
//! click. But the Pod's MIDI hardware is **input-only** (TRS MIDI-IN → RX = PB7),
//! so in practice there is nothing to conflict: use **MIDI-IN (RX = PB7) +
//! encoder click (PB6)** and configure the UART RX-only. (libDaisy's DaisyPod
//! inits MIDI as TX_RX, which claims PB6 and can break the click — this BSP
//! avoids that.) See [`midi`].
//!
//! The buttons/encoder are wired as libDaisy configures them: internal
//! **pull-up**, so a closed contact reads **low** (`is_low()` == engaged),
//! debounced with libDaisy's 8-sample shift register. The RGB LEDs are
//! **inverted** (active-low, `RgbLed::Init(..., invert=true)`): a lit channel
//! drives its pin **low**.
//!
//! Audio (SAI/codec) and USB are on the Seed itself — see `daisy-audio` and the
//! USB apps, not this module.
//!
//! The pure control logic ([`Debouncer`], [`EncoderDecoder`]) is host-testable;
//! the pin binding is target-gated.

// ===========================================================================
// Pure, host-testable control logic (no HAL — runs under `cargo test`).
// ===========================================================================

/// libDaisy's switch debouncer: an 8-bit shift register clocked at the sample
/// rate. Feed one sample per [`update`](Debouncer::update); `0xFF` = eight
/// consecutive engaged reads (pressed), `0x7F`/`0x80` = the clean edges. (Same
/// primitive libDaisy uses for every button — see also the hothouse module.)
#[derive(Copy, Clone, Default)]
pub struct Debouncer {
    state: u8,
}

impl Debouncer {
    pub const fn new() -> Self {
        Self { state: 0 }
    }

    /// Shift in one sample. `engaged` is the debounced-contact sense: for the
    /// pull-up-wired Pod buttons that is `pin.is_low()`.
    pub fn update(&mut self, engaged: bool) {
        self.state = (self.state << 1) | engaged as u8;
    }

    /// Eight consecutive engaged samples.
    pub fn pressed(&self) -> bool {
        self.state == 0xff
    }

    /// The sample that completed a press (libDaisy `RisingEdge`).
    pub fn rising_edge(&self) -> bool {
        self.state == 0x7f
    }

    /// The sample that completed a release (libDaisy `FallingEdge`).
    pub fn falling_edge(&self) -> bool {
        self.state == 0x80
    }
}

/// Quadrature decoder for the rotary encoder, a faithful port of libDaisy's
/// `Encoder::Debounce` increment logic: it keeps an 8-bit history of each raw
/// (pull-up) phase line and emits +1 when A falls while B is low, −1 when B
/// falls while A is low, else 0. Feed the *raw pin reads* (`is_high()`), one
/// pair per tick, at a steady rate (libDaisy runs it at 1 kHz).
#[derive(Copy, Clone)]
pub struct EncoderDecoder {
    a: u8,
    b: u8,
    inc: i32,
}

impl Default for EncoderDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl EncoderDecoder {
    /// History starts all-high (idle, pull-up), as libDaisy inits `a_ = b_ = 0xff`.
    pub const fn new() -> Self {
        Self {
            a: 0xff,
            b: 0xff,
            inc: 0,
        }
    }

    /// Shift in one raw phase sample per line and return this tick's increment
    /// (−1, 0, or +1). `a_high`/`b_high` are the raw pin levels (idle = high).
    pub fn update(&mut self, a_high: bool, b_high: bool) -> i32 {
        self.a = (self.a << 1) | a_high as u8;
        self.b = (self.b << 1) | b_high as u8;
        self.inc = 0;
        if (self.a & 0x03) == 0x02 && (self.b & 0x03) == 0x00 {
            self.inc = 1;
        } else if (self.b & 0x03) == 0x02 && (self.a & 0x03) == 0x00 {
            self.inc = -1;
        }
        self.inc
    }

    /// This tick's increment (as last returned by [`update`](Self::update)).
    pub fn increment(&self) -> i32 {
        self.inc
    }
}

/// The two knobs (potentiometers).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Knob {
    One,
    Two,
}

/// The two momentary buttons.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Button {
    One,
    Two,
}

/// The two RGB LEDs.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Led {
    One,
    Two,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn debounce_needs_eight_samples() {
        let mut d = Debouncer::new();
        for _ in 0..7 {
            d.update(true);
            assert!(!d.pressed());
        }
        assert!(d.rising_edge()); // 0x7f
        d.update(true);
        assert!(d.pressed()); // 0xff
    }

    #[test]
    fn encoder_counts_up() {
        // B settles low, then A falls while B low → +1 (libDaisy's CW arm).
        let mut e = EncoderDecoder::new();
        assert_eq!(e.update(true, false), 0);
        assert_eq!(e.update(true, false), 0); // b history now ...00
        assert_eq!(e.update(false, false), 1); // a falls: a=...10, b=...00
    }

    #[test]
    fn encoder_counts_down() {
        // A settles low, then B falls while A low → −1 (the CCW arm).
        let mut e = EncoderDecoder::new();
        assert_eq!(e.update(false, true), 0);
        assert_eq!(e.update(false, true), 0); // a history now ...00
        assert_eq!(e.update(false, false), -1); // b falls: b=...10, a=...00
    }

    #[test]
    fn encoder_idle_is_zero() {
        let mut e = EncoderDecoder::new();
        for _ in 0..8 {
            assert_eq!(e.update(true, true), 0); // no motion, both idle-high
        }
    }
}

// ===========================================================================
// Bare-metal pin binding (target only — needs the HAL).
// ===========================================================================

#[cfg(target_os = "none")]
pub use bare::*;

#[cfg(target_os = "none")]
mod bare {
    use super::*;

    use crate::hal::adc::{Adc, Enabled};
    use crate::hal::block;
    use crate::hal::gpio::gpioa::{PA0, PA1, PA2, PA4, PA6, PA7};
    use crate::hal::gpio::gpiob::{PB1, PB6};
    use crate::hal::gpio::gpioc::{PC0, PC1, PC4};
    use crate::hal::gpio::gpiod::PD11;
    use crate::hal::gpio::gpiog::PG9;
    use crate::hal::gpio::{Analog, ErasedPin, Input, Output, PinMode, PushPull};
    use crate::hal::hal::adc::OneShot as _;
    use crate::hal::pac::ADC1;

    /// The two knobs. Both are on **ADC1** (PC4 = ADC12_INP4, PC0 = ADC123_INP10),
    /// so a single enabled `Adc<ADC1>` reads them via blocking one-shots.
    pub struct Knobs {
        k1: PC4<Analog>,
        k2: PC0<Analog>,
    }

    impl Knobs {
        /// Bind the two pot pins (any mode) as analog inputs.
        pub fn new<M0, M1>(k1: PC4<M0>, k2: PC0<M1>) -> Self
        where
            M0: PinMode,
            M1: PinMode,
        {
            Self {
                k1: k1.into_analog(),
                k2: k2.into_analog(),
            }
        }

        /// Blocking one-shot read of one knob, normalised to `0.0..~1.0` (fully
        /// CCW = 0.0, CW ≈ 1.0), dividing by the ADC full-scale `slope()`.
        pub fn read(&mut self, adc: &mut Adc<ADC1, Enabled>, knob: Knob) -> f32 {
            let full = adc.slope() as f32;
            let raw: u32 = match knob {
                Knob::One => block!(adc.read(&mut self.k1)).unwrap(),
                Knob::Two => block!(adc.read(&mut self.k2)).unwrap(),
            };
            raw as f32 / full
        }

        /// Read both knobs in KNOB_1, KNOB_2 order.
        pub fn read_all(&mut self, adc: &mut Adc<ADC1, Enabled>) -> [f32; 2] {
            [self.read(adc, Knob::One), self.read(adc, Knob::Two)]
        }
    }

    /// The two momentary buttons: debounced pull-up inputs (engaged = low).
    pub struct Buttons {
        pins: [ErasedPin<Input>; 2],
        debouncers: [Debouncer; 2],
    }

    impl Buttons {
        pub fn new<M0, M1>(b1: PG9<M0>, b2: PA2<M1>) -> Self
        where
            M0: PinMode,
            M1: PinMode,
        {
            Self {
                pins: [
                    b1.into_pull_up_input().erase(),
                    b2.into_pull_up_input().erase(),
                ],
                debouncers: [Debouncer::new(); 2],
            }
        }

        /// Sample and debounce both buttons. Call at a steady rate (libDaisy
        /// debounces at 1 kHz; once per audio block is typical).
        pub fn update(&mut self) {
            for (deb, pin) in self.debouncers.iter_mut().zip(self.pins.iter()) {
                deb.update(pin.is_low()); // pull-up wired: engaged = low
            }
        }

        fn index(b: Button) -> usize {
            match b {
                Button::One => 0,
                Button::Two => 1,
            }
        }

        /// Button is currently held down (debounced).
        pub fn pressed(&self, b: Button) -> bool {
            self.debouncers[Self::index(b)].pressed()
        }

        /// The sample on which the button was just pressed.
        pub fn rising(&self, b: Button) -> bool {
            self.debouncers[Self::index(b)].rising_edge()
        }

        /// The sample on which the button was just released.
        pub fn falling(&self, b: Button) -> bool {
            self.debouncers[Self::index(b)].falling_edge()
        }
    }

    /// The rotary encoder: two quadrature phase inputs (A, B) plus the push
    /// switch, all pull-up. [`update`](Encoder::update) advances the quadrature
    /// decoder and debounces the click; it returns the tick's ±1 increment.
    ///
    /// **Note:** the click is on PB6, which is also MIDI TX — see the module
    /// docs. Don't also bind [`midi`] TX if you use the click.
    pub struct Encoder {
        a: ErasedPin<Input>,
        b: ErasedPin<Input>,
        click: ErasedPin<Input>,
        decoder: EncoderDecoder,
        click_deb: Debouncer,
    }

    impl Encoder {
        pub fn new<M0, M1, M2>(a: PD11<M0>, b: PA0<M1>, click: PB6<M2>) -> Self
        where
            M0: PinMode,
            M1: PinMode,
            M2: PinMode,
        {
            Self {
                a: a.into_pull_up_input().erase(),
                b: b.into_pull_up_input().erase(),
                click: click.into_pull_up_input().erase(),
                decoder: EncoderDecoder::new(),
                click_deb: Debouncer::new(),
            }
        }

        /// Sample the phases + click and return this tick's rotation increment
        /// (−1, 0, +1). Call at a steady rate.
        pub fn update(&mut self) -> i32 {
            self.click_deb.update(self.click.is_low());
            // Raw phase levels (idle = high on the pull-ups).
            self.decoder.update(self.a.is_high(), self.b.is_high())
        }

        /// This tick's rotation increment (as last returned by [`update`](Self::update)).
        pub fn increment(&self) -> i32 {
            self.decoder.increment()
        }

        /// Encoder button currently held down (debounced).
        pub fn pressed(&self) -> bool {
            self.click_deb.pressed()
        }

        /// The sample on which the encoder button was just pressed / released.
        pub fn rising(&self) -> bool {
            self.click_deb.rising_edge()
        }
        pub fn falling(&self) -> bool {
            self.click_deb.falling_edge()
        }
    }

    /// The two RGB LEDs. Each colour channel is a push-pull GPIO; the LEDs are
    /// **inverted** (active-low), so [`set`](RgbLeds::set) drives a lit channel
    /// low. This gives the eight primary/secondary colours (per-channel on/off);
    /// full-colour mixing would need software PWM (a future addition).
    pub struct RgbLeds {
        led1: [ErasedPin<Output<PushPull>>; 3], // R, G, B
        led2: [ErasedPin<Output<PushPull>>; 3],
    }

    impl RgbLeds {
        #[allow(clippy::too_many_arguments)]
        pub fn new<M0, M1, M2, M3, M4, M5>(
            l1r: PC1<M0>,
            l1g: PA6<M1>,
            l1b: PA7<M2>,
            l2r: PB1<M3>,
            l2g: PA1<M4>,
            l2b: PA4<M5>,
        ) -> Self
        where
            M0: PinMode,
            M1: PinMode,
            M2: PinMode,
            M3: PinMode,
            M4: PinMode,
            M5: PinMode,
        {
            let mut leds = Self {
                led1: [
                    l1r.into_push_pull_output().erase(),
                    l1g.into_push_pull_output().erase(),
                    l1b.into_push_pull_output().erase(),
                ],
                led2: [
                    l2r.into_push_pull_output().erase(),
                    l2g.into_push_pull_output().erase(),
                    l2b.into_push_pull_output().erase(),
                ],
            };
            leds.set(Led::One, false, false, false); // start dark (drives pins high)
            leds.set(Led::Two, false, false, false);
            leds
        }

        /// Set an LED's three channels. `true` = lit; the pin is driven low
        /// because the Pod LEDs are active-low.
        pub fn set(&mut self, led: Led, r: bool, g: bool, b: bool) {
            let pins = match led {
                Led::One => &mut self.led1,
                Led::Two => &mut self.led2,
            };
            for (pin, on) in pins.iter_mut().zip([r, g, b]) {
                // Active-low: lit → low, off → high.
                if on {
                    pin.set_low();
                } else {
                    pin.set_high();
                }
            }
        }
    }

    /// UART MIDI facts for the Pod.
    ///
    /// The Pod's MIDI hardware is **INPUT only** (a 3.5 mm TRS MIDI-IN jack), so
    /// in practice you only need **RX = PB7** (Seed D14). The BSP doesn't own the
    /// UART (that needs the app's clock config), so set up `Serial<USART1>`
    /// yourself: **USART1**, RX = PB7, **31250 baud, 8N1**.
    ///
    /// About the "conflict": USART1 *TX* is PB6 — the same pin as the encoder
    /// click. Because the Pod has no MIDI-OUT, leave TX unused (configure the
    /// UART RX-only) and PB6 stays free for the click — **no conflict**. The trap
    /// is that libDaisy's `DaisyPod::InitMidi` initialises the UART in TX_RX mode,
    /// which claims PB6 as TX and can break the encoder click; this BSP avoids
    /// that by treating MIDI as RX-only, matching the hardware.
    pub mod midi {
        /// MIDI baud rate (bytes per second on the wire = 31250).
        pub const BAUD: u32 = 31_250;
    }

    /// The whole Daisy Pod front panel. Compose from the split GPIO pins, or
    /// build the individual groups if you only need some of them.
    pub struct DaisyPod {
        pub knobs: Knobs,
        pub buttons: Buttons,
        pub encoder: Encoder,
        pub leds: RgbLeds,
    }

    impl DaisyPod {
        pub fn new(knobs: Knobs, buttons: Buttons, encoder: Encoder, leds: RgbLeds) -> Self {
            Self {
                knobs,
                buttons,
                encoder,
                leds,
            }
        }
    }
}

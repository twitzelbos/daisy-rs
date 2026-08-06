//! Cleveland Music Co. **Hothouse** — a DIY guitar-pedal host board for the
//! Daisy Seed (Seed 3-compatible).
//!
//! This wraps the Hothouse's front-panel controls as typed HAL handles. The
//! control-to-Seed-pin map is transcribed from the Hothouse BSP
//! (`reference/HothouseExamples/src/hothouse.{h,cpp}`); the `D<n>→STM32` map is
//! from libDaisy `daisy_seed.h`:
//!
//! | Control            | Seed pin   | STM32           |
//! |--------------------|------------|-----------------|
//! | Toggle 1 (3-way)   | D9 / D10   | PB4 / PB5       |
//! | Toggle 2 (3-way)   | D7 / D8    | PG10 / PG11     |
//! | Toggle 3 (3-way)   | D5 / D6    | PD2 / PC12      |
//! | Footswitch 1 / 2   | D25 / D26  | PA0 / PD11      |
//! | Knob 1..6 (pots)   | D16..D21   | PA3,PB1,PA7,PA6,PC1,PC4 (all ADC1) |
//! | LED 1 / 2          | D22 / D23  | PA5 / PA4       |
//!
//! Audio is the on-Seed SAI/codec (TAC5242 on Seed 3) — see `daisy-audio`, not
//! this module. The switches are wired as libDaisy configures them: internal
//! **pull-up**, so a closed contact reads **low** (`is_low()` == engaged), and
//! debounced with libDaisy's 8-sample shift register.
//!
//! The pure control logic ([`Debouncer`], [`decode_toggle`],
//! [`ToggleswitchPosition`]) is host-testable; the pin binding is target-gated.

// ===========================================================================
// Pure, host-testable control logic (no HAL — runs under `cargo test`).
// ===========================================================================

/// Position of an ON-OFF-ON (or ON-ON) toggle switch.
///
/// `Middle` is unreachable on an ON-ON switch — write code that tolerates that,
/// as the Hothouse BSP header warns.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum ToggleswitchPosition {
    Up,
    Middle,
    Down,
}

/// libDaisy's switch debouncer: an 8-bit shift register clocked at the sample
/// rate. Feed one sample per [`update`](Debouncer::update); `0xFF` = eight
/// consecutive engaged reads (pressed), `0x7F`/`0x80` = the clean edges.
#[derive(Copy, Clone, Default)]
pub struct Debouncer {
    state: u8,
}

impl Debouncer {
    pub const fn new() -> Self {
        Self { state: 0 }
    }

    /// Shift in one sample. `engaged` is the debounced-contact sense: for the
    /// pull-up-wired Hothouse switches that is `pin.is_low()`.
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

/// Decode a 3-way toggle from its two (debounced) contact states, matching the
/// Hothouse BSP: up wins, then down, else middle.
pub fn decode_toggle(up_pressed: bool, down_pressed: bool) -> ToggleswitchPosition {
    if up_pressed {
        ToggleswitchPosition::Up
    } else if down_pressed {
        ToggleswitchPosition::Down
    } else {
        ToggleswitchPosition::Middle
    }
}

/// The three front-panel toggle switches.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Toggle {
    One,
    Two,
    Three,
}

/// The two footswitches.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Footswitch {
    One,
    Two,
}

/// The six knobs (potentiometers).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Knob {
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
}

/// The two footswitch LEDs.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum LedId {
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
        assert!(d.rising_edge()); // 0x7f — the eighth-from-completing sample
        d.update(true);
        assert!(d.pressed()); // 0xff
        assert!(!d.rising_edge());
    }

    #[test]
    fn debounce_release_edge() {
        let mut d = Debouncer::new();
        for _ in 0..8 {
            d.update(true);
        }
        assert!(d.pressed()); // 0xff
                              // Release: 0xff shifts down 0xfe,fc,f8,f0,e0,c0,...
        for _ in 0..6 {
            d.update(false);
        }
        assert!(!d.falling_edge()); // 0xc0, not yet
        d.update(false);
        assert!(d.falling_edge()); // 0x80 — the release edge
        d.update(false);
        assert!(!d.falling_edge()); // 0x00, fully released
        assert!(!d.pressed());
    }

    #[test]
    fn toggle_decode_matches_reference() {
        assert_eq!(decode_toggle(true, false), ToggleswitchPosition::Up);
        assert_eq!(decode_toggle(false, true), ToggleswitchPosition::Down);
        assert_eq!(decode_toggle(false, false), ToggleswitchPosition::Middle);
        // Up wins if the switch somehow reports both (transient).
        assert_eq!(decode_toggle(true, true), ToggleswitchPosition::Up);
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
    use crate::hal::gpio::gpioa::{PA0, PA3, PA4, PA5, PA6, PA7};
    use crate::hal::gpio::gpiob::{PB1, PB4, PB5};
    use crate::hal::gpio::gpioc::{PC1, PC12, PC4};
    use crate::hal::gpio::gpiod::{PD11, PD2};
    use crate::hal::gpio::gpiog::{PG10, PG11};
    use crate::hal::gpio::{Analog, ErasedPin, Input, Output, PushPull};
    use crate::hal::hal::adc::OneShot as _;
    use crate::hal::pac::ADC1;

    // Switch array order: [SW1_UP, SW1_DOWN, SW2_UP, SW2_DOWN, SW3_UP,
    // SW3_DOWN, FSW1, FSW2] — matching the Hothouse BSP enum.
    const FSW1: usize = 6;
    const FSW2: usize = 7;

    /// The three toggles + two footswitches: eight debounced pull-up inputs.
    pub struct Switches {
        pins: [ErasedPin<Input>; 8],
        debouncers: [Debouncer; 8],
    }

    impl Switches {
        /// Bind the switch pins (as split from GPIO in their reset `Analog`
        /// state) and configure each as a pull-up input.
        #[allow(clippy::too_many_arguments)]
        pub fn new(
            sw1_up: PB4<Analog>,
            sw1_down: PB5<Analog>,
            sw2_up: PG10<Analog>,
            sw2_down: PG11<Analog>,
            sw3_up: PD2<Analog>,
            sw3_down: PC12<Analog>,
            fsw1: PA0<Analog>,
            fsw2: PD11<Analog>,
        ) -> Self {
            Self {
                pins: [
                    sw1_up.into_pull_up_input().erase(),
                    sw1_down.into_pull_up_input().erase(),
                    sw2_up.into_pull_up_input().erase(),
                    sw2_down.into_pull_up_input().erase(),
                    sw3_up.into_pull_up_input().erase(),
                    sw3_down.into_pull_up_input().erase(),
                    fsw1.into_pull_up_input().erase(),
                    fsw2.into_pull_up_input().erase(),
                ],
                debouncers: [Debouncer::new(); 8],
            }
        }

        /// Sample and debounce every contact. Call at a steady rate — libDaisy
        /// debounces at 1 kHz; once per audio block is the usual place — so the
        /// 8-sample window behaves as designed.
        pub fn update(&mut self) {
            for (deb, pin) in self.debouncers.iter_mut().zip(self.pins.iter()) {
                // Pull-up wired: an engaged contact pulls the pin low.
                deb.update(pin.is_low());
            }
        }

        /// Current position of a toggle switch.
        pub fn toggle(&self, t: Toggle) -> ToggleswitchPosition {
            let (up, down) = match t {
                Toggle::One => (0, 1),
                Toggle::Two => (2, 3),
                Toggle::Three => (4, 5),
            };
            decode_toggle(
                self.debouncers[up].pressed(),
                self.debouncers[down].pressed(),
            )
        }

        fn fsw_index(f: Footswitch) -> usize {
            match f {
                Footswitch::One => FSW1,
                Footswitch::Two => FSW2,
            }
        }

        /// Footswitch is currently held down (debounced).
        pub fn footswitch_pressed(&self, f: Footswitch) -> bool {
            self.debouncers[Self::fsw_index(f)].pressed()
        }

        /// The sample on which the footswitch was just pressed.
        pub fn footswitch_rising(&self, f: Footswitch) -> bool {
            self.debouncers[Self::fsw_index(f)].rising_edge()
        }

        /// The sample on which the footswitch was just released.
        pub fn footswitch_falling(&self, f: Footswitch) -> bool {
            self.debouncers[Self::fsw_index(f)].falling_edge()
        }

        /// Both footswitches held — the Hothouse gesture that libDaisy turns
        /// into a reset-to-bootloader after ~2 s. This is just the predicate;
        /// the app decides when/whether to jump to DFU.
        pub fn dfu_gesture(&self) -> bool {
            self.footswitch_pressed(Footswitch::One) && self.footswitch_pressed(Footswitch::Two)
        }
    }

    /// The two footswitch LEDs (active-high, as libDaisy inits them).
    pub struct Leds {
        led1: ErasedPin<Output<PushPull>>,
        led2: ErasedPin<Output<PushPull>>,
    }

    impl Leds {
        pub fn new(led1: PA5<Analog>, led2: PA4<Analog>) -> Self {
            Self {
                led1: led1.into_push_pull_output().erase(),
                led2: led2.into_push_pull_output().erase(),
            }
        }

        /// Drive an LED. `on` = lit (active-high).
        pub fn set(&mut self, led: LedId, on: bool) {
            let pin = match led {
                LedId::One => &mut self.led1,
                LedId::Two => &mut self.led2,
            };
            if on {
                pin.set_high();
            } else {
                pin.set_low();
            }
        }
    }

    /// The six knobs. All are on **ADC1** (channels 15, 5, 7, 3, 11, 4), so a
    /// single enabled `Adc<ADC1>` reads them via blocking one-shot conversions.
    pub struct Knobs {
        k1: PA3<Analog>,
        k2: PB1<Analog>,
        k3: PA7<Analog>,
        k4: PA6<Analog>,
        k5: PC1<Analog>,
        k6: PC4<Analog>,
    }

    impl Knobs {
        #[allow(clippy::too_many_arguments)]
        pub fn new(
            k1: PA3<Analog>,
            k2: PB1<Analog>,
            k3: PA7<Analog>,
            k4: PA6<Analog>,
            k5: PC1<Analog>,
            k6: PC4<Analog>,
        ) -> Self {
            Self {
                k1,
                k2,
                k3,
                k4,
                k5,
                k6,
            }
        }

        /// Blocking one-shot read of one knob, normalised to `0.0..~1.0`. No
        /// flip/invert (libDaisy `InitSingle` default): fully CCW = 0.0, fully
        /// CW ≈ 1.0. Divides by the ADC `slope()` (full scale = 2^bits) — the
        /// HAL-recommended, gain-error-free denominator — so the top code reads
        /// `(slope-1)/slope`, just shy of 1.0. Resolution sets the step size.
        pub fn read(&mut self, adc: &mut Adc<ADC1, Enabled>, knob: Knob) -> f32 {
            let full = adc.slope() as f32;
            let raw: u32 = match knob {
                Knob::One => block!(adc.read(&mut self.k1)).unwrap(),
                Knob::Two => block!(adc.read(&mut self.k2)).unwrap(),
                Knob::Three => block!(adc.read(&mut self.k3)).unwrap(),
                Knob::Four => block!(adc.read(&mut self.k4)).unwrap(),
                Knob::Five => block!(adc.read(&mut self.k5)).unwrap(),
                Knob::Six => block!(adc.read(&mut self.k6)).unwrap(),
            };
            raw as f32 / full
        }

        /// Read all six knobs in KNOB_1..KNOB_6 order.
        pub fn read_all(&mut self, adc: &mut Adc<ADC1, Enabled>) -> [f32; 6] {
            [
                self.read(adc, Knob::One),
                self.read(adc, Knob::Two),
                self.read(adc, Knob::Three),
                self.read(adc, Knob::Four),
                self.read(adc, Knob::Five),
                self.read(adc, Knob::Six),
            ]
        }
    }

    /// The whole Hothouse front panel. Compose from the split GPIO pins, or
    /// build the [`Switches`]/[`Leds`]/[`Knobs`] groups individually if you only
    /// need some of them.
    pub struct Hothouse {
        pub switches: Switches,
        pub leds: Leds,
        pub knobs: Knobs,
    }

    impl Hothouse {
        pub fn new(switches: Switches, leds: Leds, knobs: Knobs) -> Self {
            Self {
                switches,
                leds,
                knobs,
            }
        }
    }
}

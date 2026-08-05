//! User LED on the Daisy Seed (active-high on PC7).

use crate::hal::gpio::gpioc::PC7;
use crate::hal::gpio::{Analog, Output, PushPull};

pub type UserLed = PC7<Output<PushPull>>;

/// Configure PC7 as a push-pull output driving the on-board LED.
///
/// The caller is responsible for splitting GPIOC (usually with
/// `dp.GPIOC.split(ccdr.peripheral.GPIOC)`) and passing `pc7` here, because
/// GPIOC is shared with other Seed pins (e.g. SDMMC data lines on PC8..PC12).
pub fn user_led(pc7: PC7<Analog>) -> UserLed {
    pc7.into_push_pull_output()
}

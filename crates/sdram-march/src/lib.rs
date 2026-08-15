#![no_std]

//! Pure-logic SDRAM memory-test suite for the Daisy Seed's 64 MiB SDRAM.
//!
//! The suite is expressed as **data** — a list of [`Phase`]s, each one pass over
//! the whole word array — plus a single generic runner ([`run_phase`]) driven
//! through a [`Harness`] implementation. The app's harness does volatile
//! reads/writes over the real DRAM and repaints the TUI between chunks; the
//! tests' harness is a `Vec<u32>` with an injected fault. This keeps the actual
//! test *definition* (what gets written, what is expected, in which direction)
//! host-verifiable, while the target adds only the volatile access + reporting.
//!
//! Coverage (classic memory-test faults, per the March literature — see
//! `docs/references.md`):
//!   * **own-address** — write each cell's own index, read it back. Catches
//!     address-decoder / FMC-mapping faults and aliasing (the most likely
//!     failure of a mis-configured controller) and stuck data bits.
//!   * **data patterns** — 0x0000_0000, 0xFFFF_FFFF, 0xAAAA_AAAA, 0x5555_5555:
//!     stuck-at and adjacent-line shorts on the 32-bit data bus.
//!   * **March C-** — ⇑(w0); ⇑(r0,w1); ⇑(r1,w0); ⇓(r0,w1); ⇓(r1,w0); ⇓(r0):
//!     SAF (stuck-at), TF (transition), CF (coupling), AF (address) faults.

/// A value-generating function of a word index: the value written, or the value
/// expected on read, at index `i`. (Fn item, so it is `const`-usable in [`SUITE`].)
pub type ValueFn = fn(usize) -> u32;

fn v_zero(_: usize) -> u32 {
    0
}
fn v_ones(_: usize) -> u32 {
    !0
}
fn v_aaaa(_: usize) -> u32 {
    0xAAAA_AAAA
}
fn v_5555(_: usize) -> u32 {
    0x5555_5555
}
fn v_own(i: usize) -> u32 {
    i as u32
}

/// A named test group, for grouping phases in the UI. Indices into [`GROUPS`].
pub const GROUPS: &[&str] = &[
    "Own-address",
    "Pattern 0x00000000",
    "Pattern 0xFFFFFFFF",
    "Pattern 0xAAAAAAAA",
    "Pattern 0x55555555",
    "March C-",
];
/// Number of test groups.
pub const NGROUPS: usize = GROUPS.len();

/// One pass over the whole word array.
#[derive(Clone, Copy, Debug)]
pub struct Phase {
    /// Short human label, e.g. `"M1 ⇑ r0,w1"`.
    pub label: &'static str,
    /// Which [`GROUPS`] entry this phase belongs to.
    pub group: usize,
    /// Ascending (`true`) or descending (`false`) word order.
    pub ascending: bool,
    /// Expected value at index `i` to compare on read; `None` skips the read.
    pub read: Option<ValueFn>,
    /// Value to write at index `i`; `None` skips the write.
    pub write: Option<ValueFn>,
}

/// The full test suite, in execution order.
pub const SUITE: &[Phase] = &[
    // [0] own-address: write each cell's index, then read it back.
    Phase {
        label: "write index",
        group: 0,
        ascending: true,
        read: None,
        write: Some(v_own),
    },
    Phase {
        label: "verify index",
        group: 0,
        ascending: true,
        read: Some(v_own),
        write: None,
    },
    // [1..=4] data patterns: write, then verify.
    Phase {
        label: "write 0",
        group: 1,
        ascending: true,
        read: None,
        write: Some(v_zero),
    },
    Phase {
        label: "verify 0",
        group: 1,
        ascending: true,
        read: Some(v_zero),
        write: None,
    },
    Phase {
        label: "write 1",
        group: 2,
        ascending: true,
        read: None,
        write: Some(v_ones),
    },
    Phase {
        label: "verify 1",
        group: 2,
        ascending: true,
        read: Some(v_ones),
        write: None,
    },
    Phase {
        label: "write A",
        group: 3,
        ascending: true,
        read: None,
        write: Some(v_aaaa),
    },
    Phase {
        label: "verify A",
        group: 3,
        ascending: true,
        read: Some(v_aaaa),
        write: None,
    },
    Phase {
        label: "write 5",
        group: 4,
        ascending: true,
        read: None,
        write: Some(v_5555),
    },
    Phase {
        label: "verify 5",
        group: 4,
        ascending: true,
        read: Some(v_5555),
        write: None,
    },
    // [5] March C-: ⇑(w0); ⇑(r0,w1); ⇑(r1,w0); ⇓(r0,w1); ⇓(r1,w0); ⇓(r0).
    Phase {
        label: "M0 ⇑ w0",
        group: 5,
        ascending: true,
        read: None,
        write: Some(v_zero),
    },
    Phase {
        label: "M1 ⇑ r0,w1",
        group: 5,
        ascending: true,
        read: Some(v_zero),
        write: Some(v_ones),
    },
    Phase {
        label: "M2 ⇑ r1,w0",
        group: 5,
        ascending: true,
        read: Some(v_ones),
        write: Some(v_zero),
    },
    Phase {
        label: "M3 ⇓ r0,w1",
        group: 5,
        ascending: false,
        read: Some(v_zero),
        write: Some(v_ones),
    },
    Phase {
        label: "M4 ⇓ r1,w0",
        group: 5,
        ascending: false,
        read: Some(v_ones),
        write: Some(v_zero),
    },
    Phase {
        label: "M5 ⇓ r0",
        group: 5,
        ascending: false,
        read: Some(v_zero),
        write: None,
    },
];

/// The differing bits between an expected and an observed word — a bit set here
/// is a failing data line / cell. `bit N` maps to SDRAM data pin `Dn`.
#[inline]
pub fn fault_bits(expected: u32, got: u32) -> u32 {
    expected ^ got
}

/// The environment a [`run_phase`] drives: the backing store, error sink, and a
/// between-chunks service hook. One `&mut` receiver (rather than four separate
/// closures) so an implementor can freely touch shared state — UI, error log,
/// control flags — in all four methods without borrow conflicts.
pub trait Harness {
    /// Read the word at `index`.
    fn read(&mut self, index: usize) -> u32;
    /// Write `value` to the word at `index`.
    fn write(&mut self, index: usize, value: u32);
    /// A read mismatch: `expected` vs `got` at `index`.
    fn on_error(&mut self, index: usize, expected: u32, got: u32);
    /// Called after every `chunk` words with the running count. Return `false`
    /// to **abort** the current phase early (e.g. a stop request); the default
    /// keeps going. The target uses this to poll USB, repaint, and pause/stop.
    fn service(&mut self, done: usize) -> bool {
        let _ = done;
        true
    }
}

/// Run one [`Phase`] across `words` indices in its direction.
///
/// For each index `i` (ascending or descending): if the phase reads, compare
/// `h.read(i)` against the phase's expected value and, on mismatch, call
/// `h.on_error(i, expected, got)`; then, if the phase writes, `h.write(i, …)`.
/// `h.service(done)` runs after every `chunk` words and may abort the phase.
/// `chunk` of 0 is treated as 1.
pub fn run_phase<H: Harness + ?Sized>(phase: &Phase, words: usize, chunk: usize, h: &mut H) {
    let chunk = chunk.max(1);
    let mut done = 0usize;
    while done < words {
        let n = (words - done).min(chunk);
        for k in 0..n {
            let i = if phase.ascending {
                done + k
            } else {
                words - 1 - (done + k)
            };
            if let Some(expect) = phase.read {
                let exp = expect(i);
                let got = h.read(i);
                if got != exp {
                    h.on_error(i, exp, got);
                }
            }
            if let Some(value) = phase.write {
                h.write(i, value(i));
            }
        }
        done += n;
        if !h.service(done) {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec;
    use std::vec::Vec;

    /// A simulated SDRAM: a `Vec<u32>`, optionally with one index's reads forced
    /// through a stuck-at-0 mask (a coupling/stuck fault), plus an error sink and
    /// an optional abort-after-N-services counter.
    struct Sim {
        mem: Vec<u32>,
        fault: Option<(usize, u32)>,
        errs: Vec<(usize, u32, u32)>,
        abort_after: Option<u32>,
        services: u32,
    }

    impl Harness for Sim {
        fn read(&mut self, index: usize) -> u32 {
            let raw = self.mem[index];
            match self.fault {
                Some((a, mask)) if a == index => raw & !mask,
                _ => raw,
            }
        }
        fn write(&mut self, index: usize, value: u32) {
            self.mem[index] = value;
        }
        fn on_error(&mut self, index: usize, expected: u32, got: u32) {
            self.errs.push((index, expected, got));
        }
        fn service(&mut self, _done: usize) -> bool {
            self.services += 1;
            match self.abort_after {
                Some(n) => self.services < n,
                None => true,
            }
        }
    }

    /// Run the whole suite against a fresh `Sim`, returning reported errors.
    fn run_suite(words: usize, fault: Option<(usize, u32)>) -> Vec<(usize, u32, u32)> {
        let mut sim = Sim {
            mem: vec![0u32; words],
            fault,
            errs: Vec::new(),
            abort_after: None,
            services: 0,
        };
        for phase in SUITE {
            run_phase(phase, words, 4096, &mut sim);
        }
        sim.errs
    }

    #[test]
    fn clean_memory_passes() {
        assert!(run_suite(1024, None).is_empty());
    }

    #[test]
    fn service_can_abort_a_phase_early() {
        // Abort on the 2nd service call: fewer than `words` cells get written.
        let words = 4096;
        let mut sim = Sim {
            mem: vec![0xEEEE_EEEE; words],
            fault: None,
            errs: Vec::new(),
            abort_after: Some(2),
            services: 0,
        };
        // M0 writes 0 everywhere; aborting early leaves the tail un-written.
        run_phase(&SUITE[10], words, 1024, &mut sim); // "M0 ⇑ w0"
        assert!(
            sim.mem.iter().any(|&w| w != 0),
            "abort did not stop the phase"
        );
        assert!(
            sim.mem[0] == 0,
            "the pre-abort prefix should still be written"
        );
    }

    #[test]
    fn suite_covers_all_groups_and_directions() {
        // Every group is represented, and March has both directions.
        for g in 0..NGROUPS {
            assert!(SUITE.iter().any(|p| p.group == g), "group {g} missing");
        }
        assert!(SUITE.iter().any(|p| p.group == 5 && p.ascending));
        assert!(SUITE.iter().any(|p| p.group == 5 && !p.ascending));
        // own-address: phase 0 writes the index (no read), phase 1 reads it back.
        assert!(SUITE[0].read.is_none());
        assert_eq!(SUITE[0].write.map(|f| f(3)), Some(3));
        assert_eq!(SUITE[1].read.map(|f| f(3)), Some(3));
        assert!(SUITE[1].write.is_none());
    }

    #[test]
    fn stuck_bit_is_detected_and_localised() {
        // Force bit 5 stuck-at-0 at index 100. Own-address verify writes 100
        // (bit 5 set) and reads back 100 with bit 5 cleared → a reported error.
        let errs = run_suite(1024, Some((100, 1 << 5)));
        assert!(!errs.is_empty(), "stuck bit went undetected");
        // Every reported error is at the faulty index, and the differing bit is 5.
        for (i, exp, got) in &errs {
            assert_eq!(*i, 100);
            assert_eq!(fault_bits(*exp, *got), 1 << 5);
        }
    }

    #[test]
    fn march_detects_a_read_only_fault_a_pattern_might_miss() {
        // A cell that always reads 0 regardless of what's written: own-address
        // writes a non-zero index so its verify catches it, and March M1 (r0,w1
        // then later r1) also catches it. Confirm detection comes through.
        let errs = run_suite(512, Some((7, !0))); // all bits stuck-at-0 at idx 7
        assert!(errs.iter().any(|&(i, _, _)| i == 7));
    }

    #[test]
    fn fault_bits_isolates_data_lines() {
        assert_eq!(fault_bits(0xFFFF_FFFF, 0xFFFF_FFDF), 1 << 5);
        assert_eq!(fault_bits(0xAAAA_AAAA, 0x2AAA_AAAA), 1 << 31);
        assert_eq!(fault_bits(0x1234, 0x1234), 0);
    }
}

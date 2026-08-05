#![no_std]

// TODO(daisy-dsp):
//   - Oscillators: sine (polynomial approx or LUT), saw/square with PolyBLEP,
//     triangle, noise (xorshift).
//   - Filters: 1-pole LPF/HPF, SVF (Chamberlin / TPT), biquad with cookbook
//     coefficient helpers.
//   - Envelopes: ADSR, AR, follower.
//   - Delay lines: fixed-size ring buffers over `&mut [f32]`.
//   - Utility: dB<->linear, midi<->freq, softclip, DC blocker.
// All types generic over sample rate at construction, no runtime allocation.

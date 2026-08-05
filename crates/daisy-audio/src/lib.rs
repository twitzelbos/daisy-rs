#![no_std]

// TODO(daisy-audio):
//   - SAI1 configuration: 48 kHz / 24-bit / stereo, master mode driving MCLK.
//   - WM8731 codec init over I2C2 (address 0x1A), datasheet register writes.
//   - Double-buffered DMA (DMA1 stream 0/1) into f32 blocks, default 48 frames.
//   - Public API shape: `Audio::start(|in_l, in_r, out_l, out_r| { ... })`
//     where the closure runs in the DMA half/complete IRQ.
//   - Sample-rate options: 32/48/96 kHz; block-size options: 4..=256 frames.

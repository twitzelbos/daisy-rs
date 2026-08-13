# Vendored `usb-device` 0.3.2 — larger EP0 control buffer

Local copy of the crates.io `usb-device` 0.3.2 with **one** change, wired into the
workspace via `[patch.crates-io]` in the root `Cargo.toml`.

## The patch

`src/control_pipe.rs`: raise the default `CONTROL_BUF_LEN` from **128** to **512**.

```rust
// was: 128 (256 with feature "control-buffer-256")
const CONTROL_BUF_LEN: usize = 512;
```

## Why

`usb-device` serves `GET_DESCRIPTOR(CONFIGURATION)` by assembling the **entire**
configuration descriptor — every class's interfaces, endpoints and class-specific
descriptors — into this single control buffer (`ControlPipe::accept_in`, which
runs the `DescriptorWriter` over `self.buf`) before EP0 streams it to the host. So
`CONTROL_BUF_LEN` is a hard cap on total config-descriptor size.

`daisy-usb-audio` is a composite **CDC-ACM + UAC1 (stereo in/out, volume Feature
Units, explicit feedback) + USB-MIDI** device whose config descriptor is **~369
bytes** — larger than both the 128-byte default and the 256-byte
`control-buffer-256` feature. The overflow makes `get_configuration_descriptors`
return `BufferOverflow`, which **stalls EP0**: the device enumerates its device
descriptor and accepts an address, but the host can never read the configuration,
so it rejects and re-resets the device forever (observed on hardware as
`DCFG` address stuck at 0 / `DSTS.SUSPSTS`=1, cycling).

512 bytes fits the composite with headroom. Cost is 384 bytes of extra RAM in the
`ControlPipe`, which is negligible on the STM32H750.

Upstream has no configurable buffer beyond the 256 feature and 0.3.2 is the newest
published release, so vendoring is the only way to raise it without dropping a
function from the composite.

## Re-syncing with upstream

Diff `src/control_pipe.rs` against a fresh 0.3.2 checkout; the only intended delta
is the `CONTROL_BUF_LEN` block. The `[[test]]` target and dev-dependencies (`rusb`,
`rand`) were removed from `Cargo.toml` — they aren't built through `[patch]` and
would pull in a libusb build dep.

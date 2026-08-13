# usb-loopback — Mac USB-audio loopback test harness

Plays a signal to the Daisy's USB **output**, records from its USB **input**, and
checks what came back. With the default `daisy-usb-audio` build (which loops the
host's UAC playback stream straight back to its capture stream) this validates
the full-duplex **USB isochronous** path on real hardware. It's also the
foundation for **hardware-in-the-loop DSP testing** (see below).

Status: the pass-through loopback is **HW-validated** — a 1 kHz tone played to the
device is recorded back intact (correlation ≈ 1, peak sample matches the
device-side counters exactly).

## Requirements (macOS)

```sh
brew install ffmpeg switchaudio-osx
# `compare` mode additionally needs numpy:  pip3 install numpy
```

The board must be running the USB-audio app and enumerated as a composite audio
device (`daisy flash -p daisy-usb-audio`, then it appears in Audio MIDI Setup).

## Use

```sh
# 1. make a test signal (or use any 48 kHz 16-bit stereo WAV)
ffmpeg -f lavfi -i "sine=frequency=1000:duration=3:sample_rate=48000" \
       -ac 2 -c:a pcm_s16le /tmp/tone1k.wav

# 2. play it to the device, record what comes back
./loopback.sh /tmp/tone1k.wav /tmp/captured.wav 6

# 3a. quick check: is the tone there?  (Goertzel, no deps)
./analyze.py tone /tmp/captured.wav 1000

# 3b. broadband check: did the device reproduce the reference *spectrum*?
#     (use a sweep/noise input, not a tone; needs numpy)
uv run --with numpy python analyze.py spectral /tmp/captured.wav /tmp/ref.wav
```

## What the audio loopback can and can't validate

The USB *audio* path is **spectrally faithful but NOT sample-accurate**. macOS
independently sample-rate-converts each stream to reconcile the device's clock,
so the returned samples do not line up 1:1 with what you sent — even though the
frequency content is preserved. Measured on this hardware with a pass-through
loopback:

| Signal | `tone`/`spectral` result | sample-accurate (`compare`) |
| ------ | ------------------------ | --------------------------- |
| 1 kHz tone | recovered clean, ratio ~185× | — |
| pink noise | spectral corr **0.96**, LSD **1.3 dB** | peak x-corr **0.16** (not aligned) |

So: use `tone` and `spectral` over the audio path. `analyze.py compare`
(sample-accurate: align + amplitude-normalize + residual SNR) is included for a
**reliable** transport — a future CDC/bulk block-exchange path — not the iso
audio loopback. This is exactly how audio DSP is validated in practice: by
frequency response and features, not bit-exact samples.

## Two macOS gotchas this harness defeats

These cost us real debugging time; the script handles both automatically:

1. **Bluetooth outputs steal the default.** AirPods (and similar) aggressively
   re-grab the default output device. `afplay` only plays to the *default*, so
   the script sets the Daisy as default output **immediately before** playing and
   restores the previous default on exit. If audio still comes out of your
   AirPods, disconnect them for the test.
2. **The capture index changes on every re-enumeration.** The Daisy's
   avfoundation input index is *not* stable — it shifts whenever the device
   re-enumerates (e.g. after a reflash). Hard-coding an index is how every early
   recording came back silent: we were recording the MacBook mic. The script
   re-detects the index **by device name** each run.

Set `DAISY_MATCH=...` to match a different device-name substring (default
`Daisy`), e.g. for the standalone `pod-midi` build.

## Hardware-in-the-loop DSP testing (the roadmap)

The repo already has a host-side golden framework: `tools/dsp-golden` (a
numpy/scipy oracle) generates `<name>.in.f32` / `<name>.out.f32` vectors from
`crates/daisy-dsp/tests/cases.toml`, and `cargo test -p daisy-dsp --test golden`
checks the **host** f32 code against them.

This harness extends that to **silicon**: run the same DSP core on the real M7
(inserted into the loopback) and check its output against the reference. Because
the audio path isn't sample-accurate (above), there are two tiers:

1. **Spectral / feature check over the audio loopback (works today).** Wire a
   selectable `daisy-dsp` core into the loopback, play a sweep / tone / noise
   through it, and compare the *response* — `analyze.py spectral` for frequency
   response, or feature metrics (THD for `waveshaper`, cutoff/attenuation for
   `filter`, RT60 for `reverb`, pitch for `yin`, …) against the DSP's analytic
   target or a golden run through the same core. Catches real-hardware FPU
   rounding, block-boundary state, and gross regressions. Needs one firmware
   addition: a build that routes the loopback through a chosen core.

2. **Sample-accurate check over a reliable transport (future).** For bit-exact
   validation against the golden `.out.f32`, exchange fixed sample blocks over a
   *lossless* channel (CDC-ACM or a bulk endpoint) instead of iso audio: host
   sends a golden `.in.f32` block → device runs the core → returns the output
   block → `analyze.py compare` (or a direct f32 diff). This sidesteps macOS SRC
   and iso drops entirely. This directory is where that host tooling will live.

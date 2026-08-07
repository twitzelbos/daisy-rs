# fx-loop — your guitar pedals as a Mac audio insert

Turn the Daisy-in-a-Hothouse into a USB audio interface whose **send/return is a
pair of pedal jacks**, so any hardware guitar pedal patched between them becomes
a real-time insert effect on a track in your DAW. Patch a fuzz (or a whole
pedalboard) between OUT and IN, drop an insert plugin on a track, and your
outboard gear behaves like a plugin.

## The idea in one picture

```
 Mac track out ──USB playback──► Daisy ──codec DAC──► Hothouse OUT jack
                                                          │
                                                    [ your pedal(s) ]
                                                          │
 Mac track in  ◄──USB capture── Daisy ◄─codec ADC── Hothouse IN jack
```

The firmware just faithfully carries **playback → OUT** and **IN → capture**; the
pedal in the middle does the work. To the DAW it's an audio interface with an
external insert.

## Why the Daisy/Hothouse and not just any interface

You *can* do the generic version with any interface that has a spare output and
input + a DAW insert plugin. The point of doing it on the Hothouse is that it's a
**dedicated stompbox** for the job:

- **Footswitch true-bypass on the floor** — stomp the pedal in/out of the chain without touching the mouse.
- **Physical send/return trim and wet/dry blend** on the knobs, with **status LEDs** — set-and-forget gain staging, live blend.
- **It lives on your pedalboard**, in a pedal enclosure, powered like a pedal.
- **It's all-Rust and hackable** — mix on-device DSP into the loop, add MIDI control, do stereo/mono routing tricks, whatever. A generic interface can't be reprogrammed; this one is the whole point.

## It's a mode of `daisy-usb-audio`, not a new app

Everything hard already exists in `daisy-usb-audio`: the USB composite device
(CDC + UAC1 stereo in/out + MIDI), the freeze-free USB bring-up, and the
`daisy-audio` codec bridge (USB playback ↔ codec ↔ USB capture). fx-loop is a
**cargo feature** (`fx_loop`) that adds a thin processing/routing layer plus the
Hothouse control mapping on top. No new USB or codec plumbing.

## Controls (Hothouse mapping)

| Control | Function |
|---|---|
| **Footswitch 1** | **Bypass** — true bypass: capture = playback, the OUT/IN jacks are dropped from the chain. LED 1 = engaged. |
| **Footswitch 2** | **Kill/mute** the return (silence, e.g. to swap pedals without a pop), or A/B a second blend. |
| **Knob 1** | **Send trim** — level driving the pedal (instrument vs line level). |
| **Knob 2** | **Return trim** — level coming back, into the ADC (avoid clipping). |
| **Knob 3** | **Wet/dry blend** — 100 % = pure series insert; < 100 % = parallel blend with the (delay-matched) dry. |
| **Toggle 1** | Mono (send one channel / sum) vs stereo send/return. |
| **Toggle 2/3, Knobs 4–6** | spare — headroom trim, output level, or on-device DSP mixed into the loop. |

## The processing (in the codec callback)

Per audio block, in `daisy-audio`'s callback:

1. `send = playback * send_trim` → codec OUT (to the pedal).
2. `ret = codec_in * return_trim` (from the pedal).
3. `capture = bypass ? playback : mix(dry_delayed, ret, wet)`.

- **Series insert (wet = 100 %):** `capture = ret` — the pedal's output replaces the input. The normal case; zero coloration added by us.
- **Bypass:** `capture = playback` — pedal removed, unity, no round-trip.
- **Parallel blend (wet < 100 %):** blend the return with the dry — but the dry must be **delayed to match the loop's round-trip latency** (a short delay line), or the two comb-filter. This delay line is the only real DSP in the app.

## Latency & DAW plugin-delay-compensation

The round-trip = USB buffer + codec + the pedal's own delay. DAW insert plugins
(**Ableton "External Audio Effect"**, **Logic "I/O"**) *measure and compensate*
this automatically with a ping — **provided the Daisy's latency is stable**. So
the firmware's one hard requirement is a **rock-solid, fixed-latency stream**
(and, for long sessions, the UAC async feedback endpoint already on the roadmap
to stop sample-rate drift). No variable buffering, no dropouts.

## Gain staging (the thing that makes or breaks it)

Guitar pedals expect instrument/pedal levels; the codec in/out are line-ish.
Without staging you either clip the pedal's input or the ADC's. The **send/return
trims (Knobs 1–2)** are exactly for this — set send so the pedal sees a healthy
level, set return so hot pedal output doesn't clip the ADC. LEDs can flash on
near-clip as a meter.

## Mono vs stereo

Most pedals are mono. **Toggle 1** picks: mono (send channel L or a sum out, take
mono back, duplicate to both capture channels) or stereo (independent L/R
send/return, for stereo pedals). Mono is the default guitar case.

## Implementation delta

```
daisy-usb-audio  --features seed3,fx_loop     # the fx-loop soundcard build
```

- `fx_loop` feature ⇒ pull in the Hothouse BSP (`daisy-bsp::hothouse`) for the controls, and switch the codec bridge from straight passthrough to the routing above.
- Read the controls (debounced footswitches, ADC knobs) once per block, as the hothouse app already does.
- Add a small dry-delay line (only used when wet < 100 %). Everything else — USB, iso endpoints, codec DMA — is unchanged.

The bypass/trim/blend logic is pure, host-testable f32 (it can even ride the DSP
test framework's tier A). The USB + codec path is the same one the soundcard uses.

## Status & the one dependency

fx-loop is **code-ready on top of the merged soundcard**, but it rides entirely
on the **codec audio path working on real hardware** — still the HW-unvalidated
piece (Renode models no codec/analog). So it's "small firmware away, the moment
the soundcard proves out on a board," and the same board bring-up validates both.

## DAW quick-start (once it's running on hardware)

1. Flash the `seed3,fx_loop` build; the Mac sees a "daisy fx-loop" audio device.
2. Patch a pedal: Hothouse OUT → pedal in, pedal out → Hothouse IN.
3. In the DAW, insert **External Audio Effect** (Ableton) / **I/O** (Logic) on a
   track; set its output/input to the Daisy device; run the latency ping.
4. Set send/return trim on the knobs; stomp Footswitch 1 to A/B the pedal.
5. Your fuzz is now a plugin.

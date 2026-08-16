# hrir-gen

Fetches the **MIT KEMAR** compact HRIR set and exports selected directions as
`f32` left/right impulse-response pairs for the `daisy-spatializer`.

The KEMAR compact set (Gardner & Martin, MIT Media Lab, 1994) is publicly
redistributable measurement data: 128-tap HRIRs per ear at 44.1 kHz, big-endian
`int16`, interleaved L/R, one file per (elevation, azimuth) with azimuth measured
0–180° on the **right** hemisphere (head symmetry gives the left hemisphere by
swapping ears). See <https://sound.media.mit.edu/resources/KEMAR.html>.

This tool:
1. downloads `compact.tar.Z` (cached under `.kemar/`) if not already present,
2. reads the requested direction files, swapping ears for left-hemisphere
   sources,
3. resamples 44.1 kHz → 48 kHz (the Daisy codec rate) with `scipy.resample_poly`,
4. peak-normalizes each **pair jointly** (a single common factor, so the
   interaural level difference is preserved),
5. writes `crates/daisy-spatializer/src/hrir_data.rs` — a generated Rust source
   with one `[f32; N]` array per ear per direction, plus provenance.

Run (only needed to regenerate/extend the baked-in data):

```
uv --directory tools/hrir-gen run hrir-gen
```

The generated `hrir_data.rs` is committed, so building the firmware needs no
network access.

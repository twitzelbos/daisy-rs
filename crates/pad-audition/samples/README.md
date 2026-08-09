# Audition samples

Drop your own **16-bit PCM WAV** here (a sustained choir/pad works best) and render:

```
cargo run -p pad-audition -- <outdir> crates/pad-audition/samples/yourfile.wav <base_freq_hz>
```

`base_freq_hz` is the musical pitch of the recording (the pad plays it back pitched
to each chord). WAV files are gitignored — they're your material, not the repo's.

CC0 source used during development: AKWF (Adventure Kid Waveforms, public domain) —
`AKWF_hvoice` recorded human-voice single-cycles, github.com/KristofferKarlAxelEkstrand/AKWF-FREE.

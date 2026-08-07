# dsp-golden — the numpy/scipy oracle

Generates the **golden reference vectors** the `daisy-dsp` primitive tests
validate against. This is the *independent* reference: it encodes the intended
DSP math (double precision, or exact integer for the PRNG), so the Rust `f32`
implementation is checked against real scipy — not against itself.

Managed with [uv](https://docs.astral.sh/uv/).

## Use

```sh
# from tools/dsp-golden/ :
uv run dsp-golden

# or from the repo root :
uv --directory tools/dsp-golden run dsp-golden
```

That reads the shared source of truth `crates/daisy-dsp/tests/cases.toml` and
writes `<name>.in.f32` + `<name>.out.f32` for each case into
`crates/daisy-dsp/tests/golden/`. Then run the Rust side:

```sh
cargo test -p daisy-dsp --target "$(rustc -vV | sed -n 's/host: //p')" --test golden
```

## Layout

```
src/dsp_golden/
  generators.py   # input signals (impulse, sine, sweep, noise, dc, step)
  oracles.py      # per-primitive reference math (scipy.signal.lfilter, …)
  main.py         # reads cases.toml → writes goldens
```

## Adding a primitive

1. Add a `[[case]]` to `crates/daisy-dsp/tests/cases.toml`.
2. Add an oracle in `oracles.py` (keyed by the case's `primitive`).
3. Add a match arm in `crates/daisy-dsp/tests/golden.rs`.
4. `uv run dsp-golden` to (re)generate, then `cargo test`.

Goldens are committed, so this only runs when a case or an algorithm changes —
regenerating is an intentional, reviewed step (like blessing a snapshot).

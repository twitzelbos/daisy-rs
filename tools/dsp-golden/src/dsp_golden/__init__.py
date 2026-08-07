"""numpy/scipy oracle for daisy-dsp golden vectors.

The *independent reference*: for each case in ``crates/daisy-dsp/tests/cases.toml``
this package generates an input signal and computes the mathematically-ideal
output (double precision), which the Rust ``f32`` primitive is validated against.
"""

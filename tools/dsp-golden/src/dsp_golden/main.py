"""Generate golden input/output vectors for every case in cases.toml.

Reads ``crates/daisy-dsp/tests/cases.toml`` (the single source of truth, shared
with the Rust harness) and writes ``<name>.in.f32`` + ``<name>.out.f32`` per case
into ``crates/daisy-dsp/tests/golden/``.
"""

from __future__ import annotations

import sys
import tomllib
from pathlib import Path

import numpy as np

from . import generators, oracles


def find_repo_root(start: Path) -> Path:
    """Walk up to the workspace root (the Cargo.toml that has [workspace])."""
    for d in [start, *start.parents]:
        cargo = d / "Cargo.toml"
        if cargo.is_file() and "[workspace]" in cargo.read_text():
            return d
    raise SystemExit("could not locate the daisy-rs workspace root")


def write_f32(path: Path, data: np.ndarray) -> None:
    data.astype("<f4").tofile(path)


def main() -> None:
    root = find_repo_root(Path(__file__).resolve())
    cases_path = root / "crates/daisy-dsp/tests/cases.toml"
    golden_dir = root / "crates/daisy-dsp/tests/golden"
    golden_dir.mkdir(parents=True, exist_ok=True)

    manifest = tomllib.loads(cases_path.read_text())
    default_sr = float(manifest["sample_rate"])
    cases = manifest.get("case", [])

    print(f"generating {len(cases)} golden case(s) → {golden_dir.relative_to(root)}")
    for case in cases:
        name = case["name"]
        sr = float(case.get("sample_rate", default_sr))
        x = generators.generate(case["input"], sr)
        y = oracles.run(case["primitive"], x, sr, case.get("params", {}))
        write_f32(golden_dir / f"{name}.in.f32", x.astype(np.float32))
        write_f32(golden_dir / f"{name}.out.f32", y)
        print(f"  {name:<28} {case['primitive']:<8} {len(x):>6} samples")

    print("done.")


if __name__ == "__main__":
    sys.exit(main())

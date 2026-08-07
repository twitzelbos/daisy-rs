# Renode source patches

Local fidelity patches applied on top of the vendored Renode source
(`renode-infrastructure` and its nested `tlib` submodule). Renode is built from
source per the project's build recipe; apply these before building.

## `*.patch` (this directory) — renode-infrastructure

Apply from the `renode-infrastructure` checkout (Renode's `src/Infrastructure`):

```
cd <renode>/src/Infrastructure
git am ../../vendor/renode-infrastructure/patches/000*.patch   # adjust path
```

- `0001` — sysbus region-aware multibyte block access (XIP/ConnectionRegion).
- `0002` — generalize executable-IO opt-in to `IExecutableInPlaceMemory`.
- `0003` — plumb `MPU_CTRL.PRIVDEFENA` from the NVIC into the Cortex-M MPU
  (pairs with `tlib/0001`).

## `tlib/*.patch` — the tlib submodule

Apply from the `tlib` checkout (`src/Emulator/Cores/tlib`):

```
cd <renode>/src/Emulator/Cores/tlib
git am ../../../../../vendor/renode-infrastructure/patches/tlib/000*.patch   # adjust path
```

- `0001` — honour `MPU_CTRL.PRIVDEFENA` in the Cortex-M (PMSAv7) MPU background
  path: a privileged access outside every region now faults when PRIVDEFENA=0
  (ARMv7-M ARM §B3.5.3), instead of always using the default map. Verified by
  `renode/mpu.robot`.

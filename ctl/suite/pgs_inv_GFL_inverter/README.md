# Three-phase grid-following inverter

**English** | [简体中文](README_CN.md)

This suite implements a three-phase, two-level grid-following converter. Its shared controller source is used by F280039C Iris, LaunchXL-F280049C, PC simulation, and STM32G431 projects. The C2000 targets are hardware-validated; simulation supports software verification, while new STM32 deployments should follow the incremental commissioning procedure.

## Control stages

- `BUILD_LEVEL=0`: PLL and sampling diagnostics with PWM hard-disabled.
- `BUILD_LEVEL=1`: open-loop voltage generation.
- `BUILD_LEVEL=2`: PLL-synchronised positive-sequence current loop.
- `BUILD_LEVEL=3`: grid synchronization with PLL and sequence-current control.
- `BUILD_LEVEL=4`: decoupling, damping, and lead compensation.
- `BUILD_LEVEL=5`: complete active/reactive-power control.

The suite uses a two-layer SDPE model: common control settings are kept in `sdpe_general/`, while sampling, PWM, protection, and board mappings remain target-specific. The validated hardware combination includes the Helios three-phase GaN inverter and Harmonia LC filter.

Grid-connected commissioning involves hazardous voltage and energy. Complete isolated low-voltage tests, polarity checks, protection tests, and the lower build levels before connection to a live grid. See the [Chinese guide](README_CN.md) for detailed configuration and validation notes.

## SIL build-level validation

The PC project accepts an MSBuild property that temporarily overrides the
generated `BUILD_LEVEL` without rewriting the SDPE JSON:

```powershell
msbuild DigialPower_simulink.vcxproj /t:Build /p:Configuration=Debug /p:Platform=x64 `
  /p:GflBuildLevel=2 /p:OutDir=validation_build\bl2\
```

After building, run an isolated-port simulation and analyze its logged buses
from `project/simulate`:

```matlab
result = run_bl_simulation(2, 0.40);
stats = analyze_bl_result(result);
```

`run_bl_simulation` configures the four actual GMP SIL mask port parameters,
starts the selected controller, configures a validation-copy breaker/load
scenario, and stores compact Monitor, Enable, and PWM arrays. It never saves
the loaded model.

Run the complete matrix from `project/simulate`:

```matlab
report = run_bl_test_matrix();
```

The runner creates `validation_results/<timestamp>/` with:

- a non-overwriting model copy made by `configure_bl_validation_model`;
- an independent three-phase grid breaker;
- a floating, switched delta load with configurable `Zab`, `Zbc`, `Zca`, and
  an additive `ZbcAdditional`;
- isolated executables and full build logs for every test case;
- per-case MAT/config/PNG files plus `summary.csv`, `summary.md`,
  `build_summary.md`, and a threshold snapshot.

`GflFeatureFlags` is a PC-build-only property used for BL3 negative-sequence
A/B testing and the cumulative BL4 feature matrix. If the property is absent,
hardware projects retain their normal `BUILD_LEVEL` behaviour. BL5 can opt
into the fixed SIL profile: P changes from 0 to +0.05 pu at 0.30 s and Q
changes from 0 to +0.02 pu at 0.55 s; the validation load is switched at
0.75 s.

The original gridconn model's voltage feedback is on the inverter/filter side
of the inserted breaker. Therefore the BL2 matrix closes the breaker at
0.05 s and verifies PLL lock plus the rate-limited current command afterward;
it is not evidence of PLL lock while the power breaker remains open. Move or
duplicate the voltage measurement to the grid side before making that claim.
All thresholds are PC-plant commissioning gates, not hardware or grid-code
certification limits.

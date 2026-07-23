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
starts the selected controller, and logs the Monitor, Enable, and PWM buses
through `logsout`. It does not save or overwrite the `.slx`; results are
written to the system temporary directory by default.

The three `Series RLC Branch` blocks in
`DP_STD_MDL_DCAC_3ph_2level_gridconn.slx` are filter branches between the
inverter and grid. The current model has neither an independent grid breaker
nor an unbalanced three-wire delta load. Consequently, BL1 open-loop
modulation must be assessed with the grid manually disconnected, and BL3--BL5
unbalance tests require a model copy with floating `Zab/Zbc/Zca` branches and
a switched extra impedance on one branch.

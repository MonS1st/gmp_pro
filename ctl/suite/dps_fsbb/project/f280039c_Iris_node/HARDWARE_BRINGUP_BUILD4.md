# Iris FSBB Build 4 hardware bring-up and sensor calibration

## Safety status

The committed hardware configuration is intentionally limited to `BUILD_LEVEL=(1)`. Vout and Iout use nominal placeholder parameters and both calibration flags remain zero. This is a software adaptation state, not evidence that the physical sensor wiring, polarity, calibration, PWM pins, gate-driver interface, or power stage has been verified.

Do not energize Build 3 or Build 4 until the required sensor measurements have been completed and reviewed. Do not set a `*_SENSOR_CALIBRATED` flag merely to bypass a compile error.

The safe committed defaults are:

| Setting | Default |
| --- | ---: |
| `BUILD_LEVEL` | `1` |
| `FSBB_DEFAULT_OUTPUT_VOLTAGE` | `5.0 V` |
| `FSBB_DEFAULT_CURRENT_LIMIT` | `0.2 A` |
| `FSBB_ENABLE_VIN_SAMPLE` | disabled |
| `FSBB_ENABLE_IOUT_SAMPLE` | disabled |
| `FSBB_VOUT_SENSOR_CALIBRATED` | `0` |
| `FSBB_IOUT_SENSOR_CALIBRATED` | `0` |
| `FSBB_BUILD4_SELF_TEST_ENABLE` | `0` |
| `FSBB_HARDWARE_SENSOR_CALIBRATION_MODE` | `0` |

## ADC software mapping

| Signal | SDPE channel | ADC module/SOC | ADC input |
| --- | --- | --- | --- |
| Vin | `ADC_CH1` | ADCA SOC0 | ADCIN6 |
| Vout | `ADC_CH2` | ADCC SOC0 | ADCIN6 |
| IL | `ADC_CH3` | ADCB SOC0 | ADCIN3 |
| Iout | `ADC_CH4` | ADCC SOC1 | ADCIN9 |

This mapping is confirmed in `F280039_Iris_node.syscfg` and the Iris SDPE bindings. It confirms only the software channel selection. It does not confirm PCB routing, connector pinout, sensor polarity, analog gain, offset, or isolation-stage behavior.

## Sensor conversion and configuration

The shared ADC channel framework implements the project conversion as:

```text
physical_value = polarity * (adc_voltage - bias) / sensitivity
```

Use `+1` for a measured positive-going sensor and `-1` for a measured negative-going sensor. Configure each output sensor through:

- `FSBB_VOUT_SENSOR_POLARITY`, `FSBB_VOUT_SENSOR_BIAS_V`, and `FSBB_VOUT_SENSOR_SENSITIVITY`
- `FSBB_IOUT_SENSOR_POLARITY`, `FSBB_IOUT_SENSOR_BIAS_V`, and `FSBB_IOUT_SENSOR_SENSITIVITY`
- `FSBB_VOUT_SENSOR_CALIBRATED` and `FSBB_IOUT_SENSOR_CALIBRATED`

The committed Vout QuadSensor values and Iout LVFB values are nominal placeholders. They are not measured Iris values and must not be reported as calibration results.

For two measured points, calculate:

```text
sensitivity =
    (adc_voltage_2 - adc_voltage_1) /
    (physical_value_2 - physical_value_1)

bias = adc_voltage at physical value 0
```

Keep sensitivity positive and represent direction with the polarity setting. If the fitted slope is negative, use its absolute magnitude as sensitivity and set polarity to `-1`.

## Acquisition-only calibration mode

Set `FSBB_HARDWARE_SENSOR_CALIBRATION_MODE` to `1` only while `BUILD_LEVEL` is `1`, regenerate the SDPE header, and rebuild. Compile-time validation rejects any other build level.

In this mode the ISR repeatedly forces One-Shot Trip on both FSBB ePWM modules, drives `PWM_ENABLE_PORT` low, clears the output-enabled state, keeps voltage commands at zero, and skips closed-loop control, protection self-test, and Build 4 self-test. It only samples and exports calibration data. Confirm the trip and gate-disable levels with a scope before connecting power.

The complete data set is available in `g_fsbb_sensor_monitor` for CCS watch:

- four raw ADC codes;
- four ADC pin voltages;
- four debiased ADC voltages;
- current bias and sensitivity parameters;
- reconstructed Vin, Vout, IL, and Iout values;
- filtered Vout for filter observation.

Calibration-mode CAN messages retain the existing message-object IDs but switch their payload meaning:

| CAN ID | Payload word 0 | Payload word 1 | Scale |
| --- | --- | --- | --- |
| `0x201` | Vin raw code | Vout raw code | integer ADC code |
| `0x202` | IL raw code | Iout raw code | integer ADC code |
| `0x203` | Vin ADC voltage | Vout ADC voltage | microvolts |
| `0x204` | IL ADC voltage | Iout ADC voltage | microvolts |
| `0x205` | reconstructed Vin | reconstructed Vout | value × 10000 |
| `0x206` | reconstructed IL | reconstructed Iout | value × 10000 |
| `0x207` | parameter page | parameter value | value × 1,000,000 |

`0x207` pages 0–3 are Vin, Vout, IL, and Iout bias; pages 4–7 are the corresponding sensitivity values. The page advances on each monitor transmission.

After calibration, restore `FSBB_HARDWARE_SENSOR_CALIBRATION_MODE` to `0` before any normal run.

## Measurement procedure

1. Disconnect or disable the power path. Use current-limited bench sources and independently verify that gate-drive outputs remain off.
2. Select Build 1 and calibration mode. Confirm both ePWM pairs are tripped and `PWM_ENABLE_PORT` is low before applying any sensor stimulus.
3. For Vout, record points at 0 V, 5 V, 12 V, and 24 V.
4. For Iout, record 0 A and at least two known positive-current points within the safe range of the fixture.
5. At every point record the reference physical value, raw ADC code, calculated ADC pin voltage, ambient conditions, and measurement instrument.
6. Fit bias, sensitivity, and polarity. Check residual error at every recorded point rather than relying on only the two fitted points.
7. Update `sdpe_requirement.json`, regenerate the header, and independently review the values.
8. Set a calibration flag to `1` only after the corresponding channel passes the measurement review.

Suggested record:

| Signal | Physical value | Raw ADC code | ADC pin voltage | Bias | Sensitivity | Polarity | Residual/error | Instrument/date |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Vout | 0 V |  |  |  |  |  |  |  |
| Vout | 5 V |  |  |  |  |  |  |  |
| Vout | 12 V |  |  |  |  |  |  |  |
| Vout | 24 V |  |  |  |  |  |  |  |
| Iout | 0 A |  |  |  |  |  |  |  |
| Iout | known point 1 |  |  |  |  |  |  |  |
| Iout | known point 2 |  |  |  |  |  |  |  |

## Closed-loop compile gates

- Build 1 and Build 2 do not require Vout or Iout calibration.
- Build 3 requires `FSBB_VOUT_SENSOR_CALIBRATED == 1`.
- Build 4 requires Iout sampling to be enabled, `FSBB_VOUT_SENSOR_CALIBRATED == 1`, and `FSBB_IOUT_SENSOR_CALIBRATED == 1`.
- Iris hardware rejects `FSBB_BUILD4_SELF_TEST_ENABLE != 0`.

Before calibration, only Build 1 and a Build 2 whose IL path and hardware shutdown behavior have been independently verified may run. Build 3 and Build 4 are prohibited.

## Vout filter and voltage-loop bandwidth

The Iris configuration reuses `dcdc_core.filter_v_out`, a first-order IIR low-pass, with `FSBB_VOUT_FILTER_ENABLE=1`, type 1, and a 150 Hz cutoff. It is consumed only by the voltage outer loop; the inductor-current loop and fast ADC protection continue to use their own unfiltered/current paths. Clearing the controller also clears the filter state, and output enable primes it from the latest Vout sample so residual output voltage is not interpreted as zero on the first closed-loop sample.

The initial voltage-loop bandwidth is 10 Hz. This is the more conservative offered value: it is fifteen times below the 150 Hz filter cutoff, so the filter contributes only about 3.8 degrees of phase lag near the requested crossover while leaving additional margin for unmeasured sensor and plant delay.

Filtering can suppress noise. It cannot recover ADC codes or physical resolution that the analog path did not provide.

## Trip Zone audit and first-board verification

The two FSBB phase bindings resolve to the Iris ePWM pair configured for complementary outputs with dead band. `PWM_MODULATOR_USING_NEGATIVE_LOGIC` in this project inverts duty-to-compare mapping for the configured action qualifier; it does not mean that an asserted-low driver input turns a MOSFET on.

The LVFB preset identifies a UCC21520DWR dual-channel gate driver. Its input/output truth table makes both driver outputs low when INA and INB are low. The firmware therefore configures both `TZA` and `TZB` to force low, then latches a software One-Shot Trip on both phase ePWMs during setup and every shutdown. `PWM_ENABLE_PORT` is additionally driven low; the established Iris control convention is `1 = enable`, `0 = disable`.

Every fault dispatch also zeros `v_req` and `dcdc_core.v_out_formal`, clears controller/filter states, trips both PWM modules, drives `PWM_ENABLE_PORT` low, and clears `g_fsbb_output_enabled`.

The source audit supports low as the intended safe logic state, but it is not a substitute for board measurement. Before any energized Build 2–4 run:

1. Keep the DC bus disconnected and use a scope or logic analyzer at the MCU PWM pins, driver inputs, and gate-driver outputs.
2. Confirm startup and forced OST make ePWM1A/B and ePWM2A/B low.
3. Confirm GPIO58 (`IRIS_GPIO1`/`PWM_ENABLE_PORT`) low disables the gate-drive interface.
4. Confirm both high-side and low-side gate voltages remain at the MOSFET-off level during reset, debugger halt, calibration mode, software fault, and OST.
5. Confirm clearing OST cannot produce an unsafe pulse before compare values and controller state are initialized.

Record the measured logic levels, timing, probe points, board revision, and gate-driver disable/reset wiring. If any result disagrees with the assumptions above, keep `BUILD_LEVEL=(1)` and correct the board binding and Trip Zone action before continuing.

## Items still requiring physical confirmation

- Vout and Iout polarity, bias, sensitivity, linearity, noise, and usable ADC resolution;
- physical routing from each listed ADC input to the intended sensor output;
- Vin and IL nominal sensor parameters on this exact board revision;
- GPIO58 gate-enable polarity and GPIO40 reset behavior at the driver;
- actual ePWM1/ePWM2 pin and gate states for startup, OST, reset, debugger halt, and fault paths;
- acceptable Vout filter cutoff and 10 Hz voltage-loop bandwidth on the measured plant;
- current-limit accuracy and the safe inductor-current reference for energized Build 4.

No physical calibration, oscilloscope verification, energized power-stage test, or hardware Build 3/4 run is documented as complete here.

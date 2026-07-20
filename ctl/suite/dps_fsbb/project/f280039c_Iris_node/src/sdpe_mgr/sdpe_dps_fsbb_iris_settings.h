/**
 * @file sdpe_dps_fsbb_iris_settings.h
 * @brief SDPE project bindings for DPS FSBB F280039C IRIS Node.
 * @note Four-switch buck-boost converter SDPE requirement for F280039C Iris Node. The LVFB entity remains the switching stage and Vin/IL nominal sensor source; the Vout interface uses the QuadSensor nominal model only as an explicitly uncalibrated placeholder until board measurements replace it.
 */

#ifndef _PROJECT_SDPE_DPS_FSBB_IRIS_SETTINGS_H_
#define _PROJECT_SDPE_DPS_FSBB_IRIS_SETTINGS_H_

#include <ctl/hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h>
#include <ctl/hardware_preset/mcu_board/iris_f280039c_node.h>
#include <ctl/hardware_preset/voltage_sensor/quad_sensor_voltage_channel.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Project-specific includes can be added here if the migrated project needs extra headers. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define SDPE_PROJECT_ID "dps_fsbb_f280039c_Iris_node"
#define SDPE_PROJECT_SUITE "dps_fsbb"
#define SDPE_PROJECT_VERSION "0.1.0"
#define SDPE_PROJECT_UPDATED_AT "2026-07-19"

//=================================================================================================
/**
 * @brief Controller Settings.
 */

/**
 * @brief Enable ADC calibration before normal control starts.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable PIL simulation mode. This mode disables direct controller output for safe communication-based simulation.
 */
// #define ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Enable GMP framework debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

/**
 * @brief Invert duty-to-CMP mapping because the IRIS ePWM action qualifier sets high on up-count CMPA and low on down-count CMPA.
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC 1

//=================================================================================================
/**
 * @brief Optional Sampling Channels.
 */

/**
 * @brief Enable output/load current sampling path.
 */
#define FSBB_ENABLE_IOUT_SAMPLE

/**
 * @brief Enable input voltage sampling path.
 */
// #define FSBB_ENABLE_VIN_SAMPLE

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief 1: modulator, PWM, and hardware check; 2: inductor-current inner loop; 3: cascaded voltage outer loop; 4: dual voltage/output-current outer-loop CV/CC control.
 *        Options: (1), (2), (3), (4)
 */
#define BUILD_LEVEL (4)

/**
 * @brief Vout feedback filter type: 0 bypass, 1 existing first-order CTL IIR low-pass.
 *        Options: (0), (1)
 */
#define FSBB_VOUT_FILTER_TYPE (1)

//=================================================================================================
/**
 * @brief Board GPIO.
 */

/**
 * @brief Gate-driver enable GPIO.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_ENABLE_PORT IRIS_GPIO1

/**
 * @brief Gate-driver reset GPIO.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_RESET_PORT IRIS_GPIO3

/**
 * @brief System status LED.
 *        Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
 */
#define SYSTEM_LED IRIS_LED1

//=================================================================================================
/**
 * @brief PWM Channel.
 */

/**
 * @brief Buck phase ePWM base.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_BUCK_BASE IRIS_EPWM1_BASE

/**
 * @brief Boost phase ePWM base.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_BOOST_BASE IRIS_EPWM2_BASE

//=================================================================================================
/**
 * @brief ADC Channel.
 */

/**
 * @brief Input voltage ADC result register base.
 *        Options: ADC_CH1_ADC_BASE, ADC_CH2_ADC_BASE, ADC_CH3_ADC_BASE, ADC_CH4_ADC_BASE, ADC_CH5_ADC_BASE, ADC_CH6_ADC_BASE, ADC_CH7_ADC_BASE, ADC_CH8_ADC_BASE, ADC_CH9_ADC_BASE, ADC_CH10_ADC_BASE, ADC_CH11_ADC_BASE, ADC_CH12_ADC_BASE
 */
#define FSBB_VIN_ADC_BASE ADC_CH1_ADC_BASE

/**
 * @brief Input voltage ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_VIN ADC_CH1

/**
 * @brief Output voltage ADC result register base.
 *        Options: ADC_CH1_ADC_BASE, ADC_CH2_ADC_BASE, ADC_CH3_ADC_BASE, ADC_CH4_ADC_BASE, ADC_CH5_ADC_BASE, ADC_CH6_ADC_BASE, ADC_CH7_ADC_BASE, ADC_CH8_ADC_BASE, ADC_CH9_ADC_BASE, ADC_CH10_ADC_BASE, ADC_CH11_ADC_BASE, ADC_CH12_ADC_BASE
 */
#define FSBB_VOUT_ADC_BASE ADC_CH2_ADC_BASE

/**
 * @brief Output voltage ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_VOUT ADC_CH2

/**
 * @brief Inductor current ADC result register base.
 *        Options: ADC_CH1_ADC_BASE, ADC_CH2_ADC_BASE, ADC_CH3_ADC_BASE, ADC_CH4_ADC_BASE, ADC_CH5_ADC_BASE, ADC_CH6_ADC_BASE, ADC_CH7_ADC_BASE, ADC_CH8_ADC_BASE, ADC_CH9_ADC_BASE, ADC_CH10_ADC_BASE, ADC_CH11_ADC_BASE, ADC_CH12_ADC_BASE
 */
#define FSBB_IL_ADC_BASE ADC_CH3_ADC_BASE

/**
 * @brief Inductor current ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_IL ADC_CH3

/**
 * @brief Input current ADC result register base, available when input-current sampling is enabled.
 *        Options: ADC_CH1_ADC_BASE, ADC_CH2_ADC_BASE, ADC_CH3_ADC_BASE, ADC_CH4_ADC_BASE, ADC_CH5_ADC_BASE, ADC_CH6_ADC_BASE, ADC_CH7_ADC_BASE, ADC_CH8_ADC_BASE, ADC_CH9_ADC_BASE, ADC_CH10_ADC_BASE, ADC_CH11_ADC_BASE, ADC_CH12_ADC_BASE
 */
#define FSBB_IIN_ADC_BASE ADC_CH4_ADC_BASE

/**
 * @brief Input current ADC channel index, available when input-current sampling is enabled.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_IIN ADC_CH4

/**
 * @brief Output/load current ADC result register base.
 *        Options: ADC_CH1_ADC_BASE, ADC_CH2_ADC_BASE, ADC_CH3_ADC_BASE, ADC_CH4_ADC_BASE, ADC_CH5_ADC_BASE, ADC_CH6_ADC_BASE, ADC_CH7_ADC_BASE, ADC_CH8_ADC_BASE, ADC_CH9_ADC_BASE, ADC_CH10_ADC_BASE, ADC_CH11_ADC_BASE, ADC_CH12_ADC_BASE
 */
#define FSBB_IOUT_ADC_BASE ADC_CH4_ADC_BASE

/**
 * @brief Output/load current ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_IOUT ADC_CH4

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief PWM carrier frequency used by the FSBB power stage.
 */
#define PWM_FREQ (20e3f)

/**
 * @brief Main control ISR frequency.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief PWM compare maximum value for the configured ePWM time base.
 */
#define CTRL_PWM_CMP_MAX (3000 - 1)

/**
 * @brief PWM deadband compare count.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief CPU main clock frequency.
 */
#define CTRL_SYS_FREQUENCY (120e6)

/**
 * @brief C2000 system tick divider derived from PWM period.
 */
#define DSP_C2000_DSP_TIME_DIV (CTRL_SYS_FREQUENCY / 1000 / CTRL_PWM_CMP_MAX / 2)

/**
 * @brief ADC voltage reference used by sensor gain and bias conversion.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief F280039C ADC resolution used by the shared sensor conversion and compile-test platform.
 */
#define CTRL_ADC_RESOLUTION (12)

/**
 * @brief Voltage per-unit base value, using the peak value of the nominal 24 Vrms system.
 */
#define CTRL_VOLTAGE_BASE (34.0f)

/**
 * @brief Current per-unit base value, using the peak value of the nominal 10 Arms system.
 */
#define CTRL_CURRENT_BASE (14.14f)

/**
 * @brief Input voltage ADC sensing sensitivity provided by the LVFB voltage sensor path.
 */
#define CTRL_FSBB_VIN_SENSITIVITY GMP_LVFB_VOLTAGE_SENSITIVITY

/**
 * @brief Input voltage ADC bias voltage provided by the LVFB voltage sensor path.
 */
#define CTRL_FSBB_VIN_BIAS GMP_LVFB_VOLTAGE_BIAS_V

/**
 * @brief Provisional polarity sign (+1 or -1). This is not a hardware verification; uncalibrated use is allowed only by the explicit FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP exception.
 */
#define FSBB_VOUT_SENSOR_POLARITY (1)

/**
 * @brief Nominal QuadSensor bias placeholder in volts; replace with measured calibration data before setting FSBB_VOUT_SENSOR_CALIBRATED to 1.
 */
#define FSBB_VOUT_SENSOR_BIAS_V QUAD_SENSOR_VCH_BIAS_V

/**
 * @brief Nominal QuadSensor sensitivity placeholder in V/V; it is not an actual Iris hardware calibration result.
 */
#define FSBB_VOUT_SENSOR_SENSITIVITY QUAD_SENSOR_VCH_SENSITIVITY_V_PER_V

/**
 * @brief Set to 1 only after the physical Vout channel bias, sensitivity, and polarity have been measured.
 */
#define FSBB_VOUT_SENSOR_CALIBRATED (0)

/**
 * @brief Compatibility alias used by the existing adc_channel conversion path.
 */
#define CTRL_FSBB_VOUT_SENSITIVITY (FSBB_VOUT_SENSOR_SENSITIVITY)

/**
 * @brief Compatibility alias used by the existing adc_channel conversion path.
 */
#define CTRL_FSBB_VOUT_BIAS (FSBB_VOUT_SENSOR_BIAS_V)

/**
 * @brief Inductor current ADC sensing sensitivity provided by the LVFB current sensor path.
 */
#define CTRL_FSBB_IL_SENSITIVITY GMP_LVFB_CURRENT_SENSITIVITY

/**
 * @brief Inductor current ADC bias voltage provided by the LVFB current sensor path.
 */
#define CTRL_FSBB_IL_BIAS GMP_LVFB_CURRENT_BIAS_V

/**
 * @brief Provisional polarity sign (+1 or -1). This is not a hardware verification; uncalibrated use is allowed only by the explicit FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP exception.
 */
#define FSBB_IOUT_SENSOR_POLARITY (1)

/**
 * @brief Existing LVFB nominal current-sensor bias placeholder in volts; replace with the measured Iout channel calibration.
 */
#define FSBB_IOUT_SENSOR_BIAS_V GMP_LVFB_CURRENT_BIAS_V

/**
 * @brief Existing LVFB nominal sensitivity placeholder in V/A; it is not an actual Iout channel calibration result.
 */
#define FSBB_IOUT_SENSOR_SENSITIVITY GMP_LVFB_CURRENT_SENSITIVITY

/**
 * @brief Set to 1 only after the physical Iout channel bias, sensitivity, and polarity have been measured.
 */
#define FSBB_IOUT_SENSOR_CALIBRATED (0)

/**
 * @brief Compatibility alias used by the existing adc_channel conversion path.
 */
#define CTRL_FSBB_IOUT_SENSITIVITY (FSBB_IOUT_SENSOR_SENSITIVITY)

/**
 * @brief Compatibility alias used by the existing adc_channel conversion path.
 */
#define CTRL_FSBB_IOUT_BIAS (FSBB_IOUT_SENSOR_BIAS_V)

/**
 * @brief Minimum load resistance used by FSBB controller initialization.
 */
#define FSBB_PARAM_RLOAD_MIN (20.0f)

/**
 * @brief Input DC-link capacitance.
 */
#define FSBB_PARAM_CIN (440e-6f)

/**
 * @brief Output capacitance used by FSBB controller initialization.
 */
#define FSBB_PARAM_COUT (440e-6f)

/**
 * @brief Output capacitor equivalent series resistance.
 */
#define FSBB_PARAM_COUT_ESR (0.1f)

/**
 * @brief Main FSBB inductor value.
 */
#define FSBB_PARAM_L (1.5e-3f)

/**
 * @brief Main inductor equivalent series resistance.
 */
#define FSBB_PARAM_L_ESR (0.05f)

/**
 * @brief 
 */
#define FSBB_INPUT_VOLTAGE_MAX (60.0f)

/**
 * @brief Minimum allowed FSBB input voltage.
 */
#define FSBB_INPUT_VOLTAGE_MIN (12.0f)

/**
 * @brief Nominal input voltage used when input-voltage sampling is disabled.
 */
#define FSBB_INPUT_VOLTAGE_NOMINAL (12.0f)

/**
 * @brief Maximum allowed FSBB output voltage command.
 */
#define FSBB_OUTPUT_VOLTAGE_MAX (72.0f)

/**
 * @brief Minimum allowed FSBB output voltage command.
 */
#define FSBB_OUTPUT_VOLTAGE_MIN (3.0f)

/**
 * @brief Maximum allowed FSBB output/load current.
 */
#define FSBB_OUTPUT_CURRENT_LIM (10.0f)

/**
 * @brief Conservative first-hardware startup output-voltage command.
 */
#define FSBB_DEFAULT_OUTPUT_VOLTAGE (5.0f)

/**
 * @brief Conservative first-hardware startup current command and user limit.
 */
#define FSBB_DEFAULT_CURRENT_LIMIT (0.2f)

/**
 * @brief Equivalent output-voltage command used by BUILD_LEVEL 1.
 */
#define FSBB_OPEN_LOOP_VOLTAGE_COMMAND (12.0f)

/**
 * @brief Voltage-reference slew rate in PU/s.
 */
#define FSBB_VOLTAGE_RAMP_PU_S (1.0f)

/**
 * @brief Current-reference slew rate in PU/s.
 */
#define FSBB_CURRENT_RAMP_PU_S (1.0f)

/**
 * @brief Requested current-loop crossover frequency.
 */
#define FSBB_CURRENT_LOOP_BANDWIDTH (800.0f)

/**
 * @brief Conservative 10 Hz first-hardware voltage-loop crossover; chosen well below the 150 Hz Vout feedback filter cutoff.
 */
#define FSBB_VOLTAGE_LOOP_BANDWIDTH (10.0f)

/**
 * @brief Maximum Build 4 inductor-current command for conservative first-hardware operation.
 */
#define FSBB_INDUCTOR_CURRENT_REF_MAX (0.3f)

/**
 * @brief CV/CC candidate-selection hysteresis in amperes.
 */
#define FSBB_CVCC_SWITCH_HYSTERESIS (0.02f)

/**
 * @brief Conservative proportional gain for the Build 4 output-current outer loop.
 */
#define FSBB_OUTPUT_CURRENT_LOOP_KP (0.2f)

/**
 * @brief Conservative Build 4 output-current outer-loop integral time in seconds.
 */
#define FSBB_OUTPUT_CURRENT_LOOP_TI (0.05f)

/**
 * @brief Must remain 0 on Iris hardware; compile-time validation rejects fault-injection self test.
 */
#define FSBB_BUILD4_SELF_TEST_ENABLE (0)

/**
 * @brief Explicit first-low-voltage hardware bring-up exception. Allows Build 4 to use the uncalibrated QuadSensor Vout and LVFB Iout nominal coefficients; this is not formal sensor calibration.
 */
#define FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP (1)

/**
 * @brief Enable the existing dcdc_core first-order IIR only on the voltage-loop Vout feedback path.
 */
#define FSBB_VOUT_FILTER_ENABLE (1)

/**
 * @brief First-order IIR Vout cutoff frequency in hertz; filtering suppresses noise but cannot recover ADC resolution.
 */
#define FSBB_VOUT_FILTER_PARAMETER (150.0f)

/**
 * @brief Set to 1 only with BUILD_LEVEL 1 to keep PWM tripped while sampling and exporting sensor calibration data.
 */
#define FSBB_HARDWARE_SENSOR_CALIBRATION_MODE (0)

/**
 * @brief Consecutive valid Vin samples required before latching UV/OV faults (10 ms at 20 kHz).
 */
#define FSBB_VIN_PROTECTION_DEBOUNCE_SAMPLES (200)

/**
 * @brief Maximum FSBB leg duty ratio.
 */
#define FSBB_DUTY_MAX (0.95f)

/**
 * @brief Minimum FSBB leg duty ratio.
 */
#define FSBB_DUTY_MIN (0.05f)

/**
 * @brief Lower voltage-ratio boundary of the buck-boost transition region.
 */
#define FSBB_TRANSITION_RATIO_LOW (0.90f)

/**
 * @brief Upper voltage-ratio boundary of the buck-boost transition region.
 */
#define FSBB_TRANSITION_RATIO_HIGH (1.10f)

/**
 * @brief Positive inductor current protection threshold.
 */
#define FSBB_PROTECT_IL_MAX (25.0f)

/**
 * @brief Negative inductor current protection threshold.
 */
#define FSBB_PROTECT_IL_MIN (-2.0f)

/**
 * @brief ADC calibration timeout in ms.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

/**
 * @brief SPLL close-loop convergence threshold.
 */
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

// User project tail code
/* Backward compatibility for the historical misspelled PIL switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

/* Compatibility names for the hardware calibration interface. */
#define FSBB_VOUT_CALIBRATION_ENABLE FSBB_VOUT_SENSOR_CALIBRATED
#define FSBB_VOUT_CALIBRATION_BIAS_V FSBB_VOUT_SENSOR_BIAS_V
#define FSBB_VOUT_CALIBRATION_SENSITIVITY FSBB_VOUT_SENSOR_SENSITIVITY

/* Iris hardware compile-time safety validation. */
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1, 2, 3, or 4."
#endif
#if (FSBB_VOUT_SENSOR_POLARITY != 1) && (FSBB_VOUT_SENSOR_POLARITY != -1)
#error "FSBB_VOUT_SENSOR_POLARITY must be +1 or -1 after physical polarity verification."
#endif
#if (FSBB_IOUT_SENSOR_POLARITY != 1) && (FSBB_IOUT_SENSOR_POLARITY != -1)
#error "FSBB_IOUT_SENSOR_POLARITY must be +1 or -1 after physical polarity verification."
#endif
#if (FSBB_VOUT_FILTER_TYPE < 0) || (FSBB_VOUT_FILTER_TYPE > 1)
#error "FSBB_VOUT_FILTER_TYPE must be 0 (bypass) or 1 (first-order IIR)."
#endif
#if (FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP != 0) && (FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP != 1)
#error "FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP must be 0 or 1."
#endif
#if (FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP == 1) && (BUILD_LEVEL != 4)
#error "FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP is only allowed with BUILD_LEVEL 4."
#endif
#if (FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP == 1) && ((FSBB_VOUT_SENSOR_CALIBRATED != 0) || (FSBB_IOUT_SENSOR_CALIBRATED != 0))
#error "Disable FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP before declaring formal Vout/Iout calibration."
#endif
#if (FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP == 1)
#warning "FSBB: Build 4 nominal sensor bring-up uses uncalibrated hardware-preset Vout/Iout coefficients; NOT FORMAL CALIBRATION."
#endif
#if (FSBB_HARDWARE_SENSOR_CALIBRATION_MODE == 1) && (BUILD_LEVEL != 1)
#error "FSBB_HARDWARE_SENSOR_CALIBRATION_MODE requires BUILD_LEVEL 1 so no closed loop can run."
#endif
#if (BUILD_LEVEL == 3) && (FSBB_VOUT_SENSOR_CALIBRATED != 1)
#error "BUILD_LEVEL 3 requires calibrated Vout sensing: set FSBB_VOUT_SENSOR_CALIBRATED=1 only after measurement."
#endif
#if (BUILD_LEVEL == 4) && !defined FSBB_ENABLE_IOUT_SAMPLE
#error "BUILD_LEVEL 4 requires output-current sampling (FSBB_ENABLE_IOUT_SAMPLE)."
#endif
#if (BUILD_LEVEL == 4) && (FSBB_BUILD4_NOMINAL_SENSOR_BRINGUP == 0) && ((FSBB_VOUT_SENSOR_CALIBRATED != 1) || (FSBB_IOUT_SENSOR_CALIBRATED != 1))
#error "Formal BUILD_LEVEL 4 requires FSBB_VOUT_SENSOR_CALIBRATED=1 and FSBB_IOUT_SENSOR_CALIBRATED=1 after physical measurement."
#endif
#if (FSBB_BUILD4_SELF_TEST_ENABLE != 0)
#error "FSBB_BUILD4_SELF_TEST_ENABLE must remain 0 on F280039C Iris hardware."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DPS_FSBB_IRIS_SETTINGS_H_

/**
 * @file xplt.ctl_interface.h
 * @brief F280039C hardware callbacks for the FSBB controller.
 */

#ifndef _FILE_XPLT_CTL_INTERFACE_H_
#define _FILE_XPLT_CTL_INTERFACE_H_

#include <xplt.peripheral.h>

#ifdef __cplusplus
extern "C"
{
#endif

void GPIO_WritePin(uint16_t gpioNumber, uint16_t outVal);

GMP_STATIC_INLINE float ctl_fsbb_adc_code_to_voltage(adc_gt raw)
{
    return ((float)raw * CTRL_ADC_VOLTAGE_REF) / (float)(1UL << CTRL_ADC_RESOLUTION);
}

GMP_STATIC_INLINE void ctl_fsbb_update_sensor_monitor(adc_gt vin_raw, adc_gt vout_raw, adc_gt il_raw,
                                                       adc_gt iout_raw)
{
    g_fsbb_sensor_monitor.raw_code[FSBB_SENSOR_MONITOR_VIN] = (uint16_t)vin_raw;
    g_fsbb_sensor_monitor.raw_code[FSBB_SENSOR_MONITOR_VOUT] = (uint16_t)vout_raw;
    g_fsbb_sensor_monitor.raw_code[FSBB_SENSOR_MONITOR_IL] = (uint16_t)il_raw;
    g_fsbb_sensor_monitor.raw_code[FSBB_SENSOR_MONITOR_IOUT] = (uint16_t)iout_raw;

    g_fsbb_sensor_monitor.adc_pin_voltage_v[FSBB_SENSOR_MONITOR_VIN] = ctl_fsbb_adc_code_to_voltage(vin_raw);
    g_fsbb_sensor_monitor.adc_pin_voltage_v[FSBB_SENSOR_MONITOR_VOUT] = ctl_fsbb_adc_code_to_voltage(vout_raw);
    g_fsbb_sensor_monitor.adc_pin_voltage_v[FSBB_SENSOR_MONITOR_IL] = ctl_fsbb_adc_code_to_voltage(il_raw);
    g_fsbb_sensor_monitor.adc_pin_voltage_v[FSBB_SENSOR_MONITOR_IOUT] = ctl_fsbb_adc_code_to_voltage(iout_raw);

    g_fsbb_sensor_monitor.bias_v[FSBB_SENSOR_MONITOR_VIN] = CTRL_FSBB_VIN_BIAS;
    g_fsbb_sensor_monitor.bias_v[FSBB_SENSOR_MONITOR_VOUT] = FSBB_VOUT_SENSOR_BIAS_V;
    g_fsbb_sensor_monitor.bias_v[FSBB_SENSOR_MONITOR_IL] = CTRL_FSBB_IL_BIAS;
    g_fsbb_sensor_monitor.bias_v[FSBB_SENSOR_MONITOR_IOUT] = FSBB_IOUT_SENSOR_BIAS_V;

    g_fsbb_sensor_monitor.sensitivity[FSBB_SENSOR_MONITOR_VIN] = CTRL_FSBB_VIN_SENSITIVITY;
    g_fsbb_sensor_monitor.sensitivity[FSBB_SENSOR_MONITOR_VOUT] = FSBB_VOUT_SENSOR_SENSITIVITY;
    g_fsbb_sensor_monitor.sensitivity[FSBB_SENSOR_MONITOR_IL] = CTRL_FSBB_IL_SENSITIVITY;
    g_fsbb_sensor_monitor.sensitivity[FSBB_SENSOR_MONITOR_IOUT] = FSBB_IOUT_SENSOR_SENSITIVITY;

    g_fsbb_sensor_monitor.debiased_voltage_v[FSBB_SENSOR_MONITOR_VIN] =
        g_fsbb_sensor_monitor.adc_pin_voltage_v[FSBB_SENSOR_MONITOR_VIN] - CTRL_FSBB_VIN_BIAS;
    g_fsbb_sensor_monitor.debiased_voltage_v[FSBB_SENSOR_MONITOR_VOUT] =
        g_fsbb_sensor_monitor.adc_pin_voltage_v[FSBB_SENSOR_MONITOR_VOUT] - FSBB_VOUT_SENSOR_BIAS_V;
    g_fsbb_sensor_monitor.debiased_voltage_v[FSBB_SENSOR_MONITOR_IL] =
        g_fsbb_sensor_monitor.adc_pin_voltage_v[FSBB_SENSOR_MONITOR_IL] - CTRL_FSBB_IL_BIAS;
    g_fsbb_sensor_monitor.debiased_voltage_v[FSBB_SENSOR_MONITOR_IOUT] =
        g_fsbb_sensor_monitor.adc_pin_voltage_v[FSBB_SENSOR_MONITOR_IOUT] - FSBB_IOUT_SENSOR_BIAS_V;

    g_fsbb_sensor_monitor.physical_value[FSBB_SENSOR_MONITOR_VIN] =
        ctrl2float(adc_v_in.control_port.value) * CTRL_VOLTAGE_BASE;
    g_fsbb_sensor_monitor.physical_value[FSBB_SENSOR_MONITOR_VOUT] =
        ctrl2float(adc_v_out.control_port.value) * CTRL_VOLTAGE_BASE;
    g_fsbb_sensor_monitor.physical_value[FSBB_SENSOR_MONITOR_IL] =
        ctrl2float(adc_i_L.control_port.value) * CTRL_CURRENT_BASE;
    g_fsbb_sensor_monitor.physical_value[FSBB_SENSOR_MONITOR_IOUT] =
        ctrl2float(adc_i_load.control_port.value) * CTRL_CURRENT_BASE;
    g_fsbb_sensor_monitor.vout_filtered_v = ctrl2float(dcdc_core.filter_v_out.out) * CTRL_VOLTAGE_BASE;
}

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    adc_gt vin_raw = ADC_readResult(FSBB_VIN_ADC_BASE, FSBB_VIN);
    adc_gt vout_raw = ADC_readResult(FSBB_VOUT_ADC_BASE, FSBB_VOUT);
    adc_gt il_raw = ADC_readResult(FSBB_IL_ADC_BASE, FSBB_IL);
    adc_gt iout_raw = ADC_readResult(FSBB_IOUT_ADC_BASE, FSBB_IOUT);
    uint16_t active_faults;

#if defined FSBB_ENABLE_VIN_SAMPLE || (FSBB_HARDWARE_SENSOR_CALIBRATION_MODE == 1)
    ctl_step_adc_channel(&adc_v_in, vin_raw);
#else
    adc_v_in.control_port.value = float2ctrl(FSBB_INPUT_VOLTAGE_NOMINAL / CTRL_VOLTAGE_BASE);
#endif
    ctl_step_adc_channel(&adc_v_out, vout_raw);
    ctl_step_adc_channel(&adc_i_L, il_raw);
#if defined FSBB_ENABLE_IOUT_SAMPLE || (FSBB_HARDWARE_SENSOR_CALIBRATION_MODE == 1)
    ctl_step_adc_channel(&adc_i_load, iout_raw);
#else
    adc_i_load.control_port.value = float2ctrl(0.0f);
#endif

#if (FSBB_HARDWARE_SENSOR_CALIBRATION_MODE == 1)
    /* Filter observation is allowed, but no controller or protection runs. */
    ctl_step_filter_iir1(&dcdc_core.filter_v_out, adc_v_out.control_port.value);
#endif
    ctl_fsbb_update_sensor_monitor(vin_raw, vout_raw, il_raw, iout_raw);

#if (FSBB_HARDWARE_SENSOR_CALIBRATION_MODE == 0)
    if (!flag_enable_adc_calibrator)
    {
        active_faults = ctl_fsbb_active_faults();
#if defined FSBB_ENABLE_VIN_SAMPLE
        {
            static uint16_t vin_fault_count = 0U;
            uint16_t vin_faults = active_faults & (FSBB_FAULT_VIN_UNDERVOLTAGE | FSBB_FAULT_VIN_OVERVOLTAGE);

            if (!g_fsbb_output_enabled || (vin_faults == FSBB_FAULT_NONE))
            {
                vin_fault_count = 0U;
                active_faults &= (uint16_t)~(FSBB_FAULT_VIN_UNDERVOLTAGE | FSBB_FAULT_VIN_OVERVOLTAGE);
            }
            else if (vin_fault_count < FSBB_VIN_PROTECTION_DEBOUNCE_SAMPLES)
            {
                vin_fault_count++;
                active_faults &= (uint16_t)~(FSBB_FAULT_VIN_UNDERVOLTAGE | FSBB_FAULT_VIN_OVERVOLTAGE);
            }
        }
#endif
        g_fsbb_faults |= active_faults;
    }
#endif
}

GMP_STATIC_INLINE uint16_t ctl_fsbb_dac_value(ctrl_gt value)
{
    ctrl_gt bounded = ctl_sat(value, float2ctrl(1.0f), -float2ctrl(1.0f));
    return (uint16_t)((bounded + float2ctrl(1.0f)) * 2047.5f);
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
#if !defined ENABLE_GMP_DL_PIL_SIM
#if (FSBB_HARDWARE_SENSOR_CALIBRATION_MODE == 1)
    EPWM_forceTripZoneEvent(PHASE_BUCK_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_BOOST_BASE, EPWM_TZ_FORCE_EVENT_OST);
    GPIO_WritePin(PWM_ENABLE_PORT, 0);
    g_fsbb_output_enabled = 0;
    g_fsbb_sensor_monitor.vout_filtered_v = ctrl2float(dcdc_core.filter_v_out.out) * CTRL_VOLTAGE_BASE;
    return;
#endif

    if (g_fsbb_faults != FSBB_FAULT_NONE)
    {
        EPWM_forceTripZoneEvent(PHASE_BUCK_BASE, EPWM_TZ_FORCE_EVENT_OST);
        EPWM_forceTripZoneEvent(PHASE_BOOST_BASE, EPWM_TZ_FORCE_EVENT_OST);
        GPIO_WritePin(PWM_ENABLE_PORT, 0);
        g_fsbb_output_enabled = 0;
        return;
    }

    EPWM_setCounterCompareValue(PHASE_BUCK_BASE, EPWM_COUNTER_COMPARE_A, ctl_get_fsbb_buck_cmp(&fsbb_mod));
    EPWM_setCounterCompareValue(PHASE_BOOST_BASE, EPWM_COUNTER_COMPARE_A, ctl_get_fsbb_boost_cmp(&fsbb_mod));
    g_fsbb_sensor_monitor.vout_filtered_v = ctrl2float(dcdc_core.filter_v_out.out) * CTRL_VOLTAGE_BASE;

#if BUILD_LEVEL >= 1
    DAC_setShadowValue(IRIS_DACA_BASE, ctl_fsbb_dac_value(adc_v_out.control_port.value));
    DAC_setShadowValue(IRIS_DACB_BASE, ctl_fsbb_dac_value(dcdc_core.v_out_formal));
#endif
#endif
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
#if (FSBB_HARDWARE_SENSOR_CALIBRATION_MODE == 1)
    EPWM_forceTripZoneEvent(PHASE_BUCK_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_BOOST_BASE, EPWM_TZ_FORCE_EVENT_OST);
    GPIO_WritePin(PWM_ENABLE_PORT, 0);
    g_fsbb_output_enabled = 0;
    return;
#endif
    EPWM_clearTripZoneFlag(PHASE_BUCK_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(PHASE_BOOST_BASE, EPWM_TZ_FLAG_OST);
    clear_all_controllers();
    GPIO_WritePin(PWM_ENABLE_PORT, 1);
    GPIO_WritePin(CONTROLLER_LED, 0);
}

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    EPWM_forceTripZoneEvent(PHASE_BUCK_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_BOOST_BASE, EPWM_TZ_FORCE_EVENT_OST);
    GPIO_WritePin(PWM_ENABLE_PORT, 0);
    GPIO_WritePin(CONTROLLER_LED, 1);
}

typedef enum _tag_dcdc_adc_index_items
{
    DCDC_ADC_ID_VIN = 0,
    DCDC_ADC_ID_VOUT = 1,
    DCDC_ADC_ID_IL = 2,
    DCDC_ADC_ID_ILOAD = 3,
    DCDC_ADC_SENSOR_NUMBER = 4
} dcdc_adc_index_items;

GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx)
{
    ctl_step_adc_channel(&adc_v_in, rx->adc_result[DCDC_ADC_ID_VIN]);
    ctl_step_adc_channel(&adc_v_out, rx->adc_result[DCDC_ADC_ID_VOUT]);
    ctl_step_adc_channel(&adc_i_L, rx->adc_result[DCDC_ADC_ID_IL]);
    ctl_step_adc_channel(&adc_i_load, rx->adc_result[DCDC_ADC_ID_ILOAD]);
}

GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx)
{
    tx->pwm_cmp[0] = ctl_get_fsbb_buck_cmp(&fsbb_mod);
    tx->pwm_cmp[1] = ctl_get_fsbb_boost_cmp(&fsbb_mod);
    tx->monitor[0] = adc_v_out.control_port.value;
    tx->monitor[1] = dcdc_core.v_out_formal;
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_XPLT_CTL_INTERFACE_H_

/** @file xplt.ctl_interface.h @brief Raw ADC/PWM mapping for FSBB SIL. */
#ifndef _FILE_FSBB_SIM_XPLT_CTL_INTERFACE_H_
#define _FILE_FSBB_SIM_XPLT_CTL_INTERFACE_H_
#include <xplt.peripheral.h>

#ifdef __cplusplus
extern "C"
{
#endif
typedef enum _tag_dcdc_adc_index_items
{
    DCDC_ADC_ID_VIN = 0,
    DCDC_ADC_ID_VOUT = 1,
    DCDC_ADC_ID_IL = 2,
    DCDC_ADC_ID_IOUT = 3,
    DCDC_ADC_ID_IIN = 4,
    DCDC_ADC_SENSOR_NUMBER = 5
} dcdc_adc_index_items;

extern fast_gt g_fsbb_sim_enable_pending;

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    /* Sensor blocks already output quantized ADC codes. */
    ctl_step_adc_channel(&adc_v_in, simulink_rx_buffer.adc_result[DCDC_ADC_ID_VIN]);
    ctl_step_adc_channel(&adc_v_out, simulink_rx_buffer.adc_result[DCDC_ADC_ID_VOUT]);
    ctl_step_adc_channel(&adc_i_L, simulink_rx_buffer.adc_result[DCDC_ADC_ID_IL]);
    ctl_step_adc_channel(&adc_i_load, simulink_rx_buffer.adc_result[DCDC_ADC_ID_IOUT]);
    if (g_fsbb_output_enabled)
        g_fsbb_faults |= ctl_fsbb_active_faults();
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    simulink_tx_buffer.pwm_cmp[0] = ctl_get_fsbb_buck_cmp(&fsbb_mod);
    /* CH2 is the Boost low-side Q4 duty, while the Simulink phase input
       directly defines the upper Q3 gate duty. Send its complement. */
    simulink_tx_buffer.pwm_cmp[1] = CTRL_PWM_CMP_MAX - ctl_get_fsbb_boost_cmp(&fsbb_mod);
    simulink_tx_buffer.monitor[0] = ctrl2float(adc_v_in.control_port.value) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[1] = ctrl2float(adc_v_out.control_port.value) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[2] = ctrl2float(adc_i_L.control_port.value) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[3] = ctrl2float(adc_i_load.control_port.value) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[4] = ctrl2float(fsbb_build4.i_L_ref_cv) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[5] = ctrl2float(fsbb_build4.i_L_ref_cc) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[6] = ctrl2float(fsbb_build4.i_L_ref_cmd) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[7] = (double)dcdc_core.is_current_dominant;
    simulink_tx_buffer.monitor[8] = (double)cia402_sm.current_state;
    simulink_tx_buffer.monitor[9] = (double)g_fsbb_faults;
    simulink_tx_buffer.monitor[10] = (double)g_fsbb_output_enabled;
    simulink_tx_buffer.monitor[11] = (double)g_fsbb_fault_reset_result;
    simulink_tx_buffer.monitor[12] = 0.0;
    simulink_tx_buffer.monitor[13] = 0.0;
    simulink_tx_buffer.monitor[14] = 0.0;
    simulink_tx_buffer.monitor[15] = 0.0;

    if (g_fsbb_sim_enable_pending)
    {
        csp_sl_enable_output();
        g_fsbb_sim_enable_pending = 0;
    }
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    clear_all_controllers();
    /* Commit Enable in ctl_output_callback(), after the freshly calculated
       PWM compare values have been copied into the same UDP frame. */
    g_fsbb_sim_enable_pending = 1;
    g_fsbb_output_enabled = 1;
}
GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    g_fsbb_sim_enable_pending = 0;
    csp_sl_disable_output();
    g_fsbb_output_enabled = 0;
}
#ifdef __cplusplus
}
#endif
#endif

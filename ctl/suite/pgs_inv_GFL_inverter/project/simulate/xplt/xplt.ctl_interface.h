//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of controller objects in this file.
//
// User should implement the Main ISR of the controller tasks.
//
// User should ensure that all the controller codes here is platform-independent.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <ctl/component/motor_control/interface/std_sil_motor_interface.h>

#include <xplt.peripheral.h>

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// Board peripheral mapping

typedef enum _tag_sinv_adc_index_items
{
    INV_ADC_ID_IDC = 0,
    INV_ADC_ID_VDC = 1,
    INV_ADC_ID_UAB = 2,
    INV_ADC_ID_UBC = 3,
    INV_ADC_ID_IA = 4,
    INV_ADC_ID_IB = 5,
    INV_ADC_ID_IC = 6,
    INV_ADC_SENSOR_NUMBER = 7

} inv_adc_index_items;

//=================================================================================================
// Controller interface

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    // copy source ADC data
    vabc_src[phase_A] = simulink_rx_buffer.adc_result[INV_ADC_ID_UAB];
    vabc_src[phase_B] = simulink_rx_buffer.adc_result[INV_ADC_ID_UBC];
    vabc_src[phase_C] = 0;

    iabc_src[phase_A] = simulink_rx_buffer.adc_result[INV_ADC_ID_IA];
    iabc_src[phase_B] = simulink_rx_buffer.adc_result[INV_ADC_ID_IB];
    iabc_src[phase_C] = simulink_rx_buffer.adc_result[INV_ADC_ID_IC];

    uuvw_src[phase_U] = 0;
    uuvw_src[phase_V] = 0;
    uuvw_src[phase_W] = 0;

    iuvw_src[phase_U] = 0;
    iuvw_src[phase_V] = 0;
    iuvw_src[phase_W] = 0;

    udc_src = simulink_rx_buffer.adc_result[INV_ADC_ID_VDC];
    idc_src = simulink_rx_buffer.adc_result[INV_ADC_ID_IDC];

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&vabc);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);

#if BUILD_LEVEL == 6
    // Reuse the existing 16-channel SIL panel ABI for BL6 commands.  The
    // validation model supplies these channels; hardware targets retain the
    // same command object through the Datalink tunable dictionary.
    rectifier_cmd.enable = simulink_rx_buffer.panel[0] >= 0.5;
    rectifier_cmd.clear_fault = simulink_rx_buffer.panel[1] >= 0.5;
    rectifier_cmd.udc_ref_v = float2ctrl((float)simulink_rx_buffer.panel[2]);
    rectifier_cmd.iq_ref_pu = float2ctrl((float)simulink_rx_buffer.panel[3]);
    rectifier_cmd.current_limit_pu = float2ctrl((float)simulink_rx_buffer.panel[4]);
    rectifier_cmd.manual_current_mode = simulink_rx_buffer.panel[5] >= 0.5;
    rectifier_cmd.manual_id_ref_pu = float2ctrl((float)simulink_rx_buffer.panel[6]);
    rectifier_cmd.manual_iq_ref_pu = float2ctrl((float)simulink_rx_buffer.panel[7]);
#endif
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    //
    // PWM channel
    //
    simulink_tx_buffer.pwm_cmp[0] = spwm.pwm_out[phase_U];
    simulink_tx_buffer.pwm_cmp[1] = spwm.pwm_out[phase_V];
    simulink_tx_buffer.pwm_cmp[2] = spwm.pwm_out[phase_W];

    //
    // monitor
    //

    // Common monitor layout:
    // [Ia, Ib, Ic, I0, Id, Iq, Id_ref, Iq_ref, Vd_grid, Vq_grid,
    //  Vd_cmd, Vq_cmd, PLL_error, PLL_frequency, level_specific_0,
    //  level_specific_1].
    simulink_tx_buffer.monitor[0] = inv_ctrl.iabc.dat[phase_A];
    simulink_tx_buffer.monitor[1] = inv_ctrl.iabc.dat[phase_B];
    simulink_tx_buffer.monitor[2] = inv_ctrl.iabc.dat[phase_C];
    simulink_tx_buffer.monitor[3] = inv_ctrl.iab0.dat[phase_0];
    simulink_tx_buffer.monitor[4] = inv_ctrl.idq.dat[phase_d];
    simulink_tx_buffer.monitor[5] = inv_ctrl.idq.dat[phase_q];
    simulink_tx_buffer.monitor[6] = inv_ctrl.idq_set.dat[phase_d];
    simulink_tx_buffer.monitor[7] = inv_ctrl.idq_set.dat[phase_q];
    simulink_tx_buffer.monitor[8] = inv_ctrl.vdq.dat[phase_d];
    simulink_tx_buffer.monitor[9] = inv_ctrl.vdq.dat[phase_q];
    simulink_tx_buffer.monitor[10] = inv_ctrl.vdq_out_comp.dat[phase_d];
    simulink_tx_buffer.monitor[11] = inv_ctrl.vdq_out_comp.dat[phase_q];
    simulink_tx_buffer.monitor[12] = ctl_get_gfl_pll_error(&inv_ctrl);
#ifdef USING_DSOGI_PLL
    simulink_tx_buffer.monitor[13] = inv_ctrl.pll.srf_pll.freq_pu;
#else
    simulink_tx_buffer.monitor[13] = inv_ctrl.pll.freq_pu;
#endif // USING_DSOGI_PLL
#if BUILD_LEVEL == 0
    simulink_tx_buffer.monitor[14] = inv_ctrl.angle;
    simulink_tx_buffer.monitor[15] = ctl_check_pll_locked();
#elif BUILD_LEVEL == 1
    simulink_tx_buffer.monitor[14] = inv_ctrl.angle;
    simulink_tx_buffer.monitor[15] = inv_ctrl.flag_enable_system;
#elif BUILD_LEVEL == 2
    simulink_tx_buffer.monitor[14] = ctl_check_pll_locked();
    simulink_tx_buffer.monitor[15] = inv_ctrl.flag_enable_system;
#elif BUILD_LEVEL == 3
    simulink_tx_buffer.monitor[14] = neg_current_ctrl.idqn.dat[phase_d];
    simulink_tx_buffer.monitor[15] = neg_current_ctrl.idqn.dat[phase_q];
#elif BUILD_LEVEL == 4
    simulink_tx_buffer.monitor[14] = inv_ctrl.vdq_ff_external.dat[phase_d];
    simulink_tx_buffer.monitor[15] = inv_ctrl.vdq_ff_decouple.dat[phase_d];
#elif BUILD_LEVEL == 5
    simulink_tx_buffer.monitor[14] = pq_ctrl.pq_meas.dat[0];
    simulink_tx_buffer.monitor[15] = pq_ctrl.pq_meas.dat[1];
#elif BUILD_LEVEL == 6
    // BL6 level-specific diagnostic layout:
    // [Ia, Ib, Ic, Udc_V, Id, Iq, Id_ref, Iq_ref, PLL_f_pu,
    //  PLL_error, state, fault_bits, Udc_PI_i, Udc_PI_out,
    //  flags(bit0 PLL, bit1 PWM, bit2 current-limit, bit3 PI-sat),
    //  internal_Udc_ref_V].
    simulink_tx_buffer.monitor[3] = rectifier_status.udc_meas_v;
    simulink_tx_buffer.monitor[8] = rectifier_status.pll_freq_pu;
    simulink_tx_buffer.monitor[9] = rectifier_status.pll_error;
    simulink_tx_buffer.monitor[10] = rectifier_status.state;
    simulink_tx_buffer.monitor[11] = rectifier_status.fault_bits;
    simulink_tx_buffer.monitor[12] = rectifier_dc_voltage_ctrl.voltage_pi.i_term;
    simulink_tx_buffer.monitor[13] = rectifier_dc_voltage_ctrl.voltage_pi.out;
    simulink_tx_buffer.monitor[14] =
        rectifier_status.pll_locked + 2 * rectifier_status.pwm_enabled +
        4 * rectifier_status.current_limited + 8 * rectifier_status.voltage_pi_saturated;
    simulink_tx_buffer.monitor[15] = rectifier_dc_voltage_ctrl.udc_ref_v;
#endif // BUILD_LEVEL
}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_fast_enable_output()
{
    csp_sl_enable_output();

    // The host simulation CSP owns the physical-output state.
}

// Disable Output
GMP_STATIC_INLINE void ctl_fast_disable_output()
{
    csp_sl_disable_output();
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_

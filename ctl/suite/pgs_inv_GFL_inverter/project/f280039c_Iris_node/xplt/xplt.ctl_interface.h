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

#include <xplt.peripheral.h>

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// Controller interface

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    // copy source ADC data
    vabc_src[phase_A] = ADC_readResult(INV_UA_RESULT_BASE, INV_UA);
    vabc_src[phase_B] = ADC_readResult(INV_UB_RESULT_BASE, INV_UB);
    vabc_src[phase_C] = ADC_readResult(INV_UC_RESULT_BASE, INV_UC);

    iabc_src[phase_A] = ADC_readResult(INV_IA_RESULT_BASE, INV_IA);
    iabc_src[phase_B] = ADC_readResult(INV_IB_RESULT_BASE, INV_IB);
    iabc_src[phase_C] = ADC_readResult(INV_IC_RESULT_BASE, INV_IC);

    uuvw_src[phase_U] = ADC_readResult(INV_UU_RESULT_BASE, INV_UU);
    uuvw_src[phase_V] = ADC_readResult(INV_UV_RESULT_BASE, INV_UV);
    uuvw_src[phase_W] = ADC_readResult(INV_UW_RESULT_BASE, INV_UW);

    iuvw_src[phase_U] = ADC_readResult(INV_IU_RESULT_BASE, INV_IU);
    iuvw_src[phase_V] = ADC_readResult(INV_IV_RESULT_BASE, INV_IV);
    iuvw_src[phase_W] = ADC_readResult(INV_IW_RESULT_BASE, INV_IW);

    udc_src = ADC_readResult(INV_VBUS_RESULT_BASE, INV_VBUS);
    idc_src = ADC_readResult(INV_IBUS_RESULT_BASE, INV_IBUS);

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&vabc);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    // Write ePWM peripheral CMP
#if defined USING_NPC_MODULATOR

#if BOARD_SELECTION == LAUNCHPAD
    EPWM_setCounterCompareValue(EPWM_J4_PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_A_OUTER]);
    EPWM_setCounterCompareValue(EPWM_J4_PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_B_OUTER]);
    EPWM_setCounterCompareValue(EPWM_J4_PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_C_OUTER]);
    EPWM_setCounterCompareValue(EPWM_J8_PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_A_INNER]);
    EPWM_setCounterCompareValue(EPWM_J8_PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_B_INNER]);
    EPWM_setCounterCompareValue(EPWM_J8_PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_C_INNER]);

#elif BOARD_SELECTION == GMP_IRIS
    EPWM_setCounterCompareValue(IRIS_EPWM1_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_A_INNER]);
    EPWM_setCounterCompareValue(IRIS_EPWM2_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_A_OUTER]);
    EPWM_setCounterCompareValue(IRIS_EPWM3_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_B_INNER]);
    EPWM_setCounterCompareValue(IRIS_EPWM4_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_B_OUTER]);
    EPWM_setCounterCompareValue(IRIS_EPWM5_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_C_INNER]);
    EPWM_setCounterCompareValue(IRIS_EPWM6_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_C_OUTER]);

#endif // BOARD_SELECTION

#else // USING_NPC_MODULATOR

    EPWM_setCounterCompareValue(PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[phase_U]);
    EPWM_setCounterCompareValue(PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[phase_V]);
    EPWM_setCounterCompareValue(PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[phase_W]);

#endif // USING_NPC_MODULATOR

    // Monitor Port
#if BUILD_LEVEL == 1

    //    DAC_setShadowValue(IRIS_DACB_BASE, inv_ctrl.angle * 2048 + 2048);
    //    DAC_setShadowValue(IRIS_DACA_BASE, inv_ctrl.abc_out.dat[phase_B]  * 2048 + 2048);

    // grid current and inverter current
    //    DAC_setShadowValue(IRIS_DACA_BASE, iabc.control_port.value.dat[phase_C] * 2048 + 2048);
    //    DAC_setShadowValue(IRIS_DACB_BASE, iuvw.control_port.value.dat[phase_C] * 2048 + 2048);

    DAC_setShadowValue(IRIS_DACA_BASE, iabc.control_port.value.dat[phase_C] * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACB_BASE, iuvw.control_port.value.dat[phase_C] * 2048 + 2048);

    // grid voltage and inverter voltage
    //    DAC_setShadowValue(IRIS_DACB_BASE, uuvw.control_port.value.dat[phase_C] * 2048 + 2048);Zq
    //    DAC_setShadowValue(IRIS_DACB_BASE, EPWM_getCounterCompareValue(PHASE_U_BASE, EPWM_COUNTER_COMPARE_A)/2);
    //    DAC_setShadowValue(IRIS_DACA_BASE, EPWM_getCounterCompareValue(PHASE_V_BASE, EPWM_COUNTER_COMPARE_A)/2);

    //    DAC_setShadowValue(IRIS_DACB_BASE, pwm_out.value[2]/2);
    //    DAC_setShadowValue(IRIS_DACA_BASE, pwm_out.value[1]/2);
    //    inv_ctrl.

#elif BUILD_LEVEL == 2

    DAC_setShadowValue(IRIS_DACA_BASE, inv_ctrl.vab0.dat[phase_A] * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACB_BASE, inv_ctrl.pll.phasor.dat[phasor_cos] * 2048 + 2048);

#elif BUILD_LEVEL == 3

    //    DAC_setShadowValue(IRIS_DACA_BASE, inv_ctrl.vabc.dat[phase_A] * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACA_BASE, inv_ctrl.vab0.dat[phase_A] * 2048 + 2048);

    DAC_setShadowValue(IRIS_DACB_BASE, inv_ctrl.pll.phasor.dat[phasor_cos] * 2048 + 2048);
    //    DAC_setShadowValue(IRIS_DACB_BASE, inv_ctrl.pll.v_pos_seq.dat[0] * 2048 + 2048);
    //    DAC_setShadowValue(IRIS_DACB_BASE, inv_ctrl.pll.srf_pll.theta * 2048 + 2048);

#endif // BUILD_LEVEL
}

// function prototype
void GPIO_WritePin(uint16_t gpioNumber, uint16_t outVal);

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_fast_enable_output()
{
    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);

    // PWM enable
    GPIO_WritePin(PWM_ENABLE_PORT, 1);

    GPIO_WritePin(PWM_RESET_PORT, 0);

    GPIO_WritePin(CONTROLLER_LED, 0);
}

// Disable Output
GMP_STATIC_INLINE void ctl_fast_disable_output()
{
    // Disables the PWM device
    EPWM_forceTripZoneEvent(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);

    // PWM disable
    GPIO_WritePin(PWM_ENABLE_PORT, 0);

    GPIO_WritePin(CONTROLLER_LED, 1);
}

//=================================================================================================
// Controller interface

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

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx)
{
    // copy source ADC data
    vabc_src[phase_A] = rx->adc_result[INV_ADC_ID_UAB];
    vabc_src[phase_B] = rx->adc_result[INV_ADC_ID_UBC];
    vabc_src[phase_C] = 0;

    iabc_src[phase_A] = rx->adc_result[INV_ADC_ID_IA];
    iabc_src[phase_B] = rx->adc_result[INV_ADC_ID_IB];
    iabc_src[phase_C] = rx->adc_result[INV_ADC_ID_IC];

    uuvw_src[phase_U] = 0;
    uuvw_src[phase_V] = 0;
    uuvw_src[phase_W] = 0;

    iuvw_src[phase_U] = 0;
    iuvw_src[phase_V] = 0;
    iuvw_src[phase_W] = 0;

    // Keep the PIL buffer ABI aligned with the simulation enum:
    // [IDC, VDC, UAB, UBC, IA, IB, IC].
    udc_src = rx->adc_result[INV_ADC_ID_VDC];
    idc_src = rx->adc_result[INV_ADC_ID_IDC];

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&vabc);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx)
{
    // Write ePWM peripheral CMP
    tx->pwm_cmp[0] = spwm.pwm_out[phase_U];
    tx->pwm_cmp[1] = spwm.pwm_out[phase_V];
    tx->pwm_cmp[2] = spwm.pwm_out[phase_W];

    // Monitor layout matches the Windows SIL target.
    tx->monitor[0] = inv_ctrl.iabc.dat[phase_A];
    tx->monitor[1] = inv_ctrl.iabc.dat[phase_B];
    tx->monitor[2] = inv_ctrl.iabc.dat[phase_C];
    tx->monitor[3] = inv_ctrl.iab0.dat[phase_0];
    tx->monitor[4] = inv_ctrl.idq.dat[phase_d];
    tx->monitor[5] = inv_ctrl.idq.dat[phase_q];
    tx->monitor[6] = inv_ctrl.idq_set.dat[phase_d];
    tx->monitor[7] = inv_ctrl.idq_set.dat[phase_q];
    tx->monitor[8] = inv_ctrl.vdq.dat[phase_d];
    tx->monitor[9] = inv_ctrl.vdq.dat[phase_q];
    tx->monitor[10] = inv_ctrl.vdq_out_comp.dat[phase_d];
    tx->monitor[11] = inv_ctrl.vdq_out_comp.dat[phase_q];
    tx->monitor[12] = ctl_get_gfl_pll_error(&inv_ctrl);
#ifdef USING_DSOGI_PLL
    tx->monitor[13] = inv_ctrl.pll.srf_pll.freq_pu;
#else
    tx->monitor[13] = inv_ctrl.pll.freq_pu;
#endif // USING_DSOGI_PLL
#if BUILD_LEVEL == 0
    tx->monitor[14] = inv_ctrl.angle;
    tx->monitor[15] = ctl_check_pll_locked();
#elif BUILD_LEVEL == 1
    tx->monitor[14] = inv_ctrl.angle;
    tx->monitor[15] = inv_ctrl.flag_enable_system;
#elif BUILD_LEVEL == 2
    tx->monitor[14] = ctl_check_pll_locked();
    tx->monitor[15] = inv_ctrl.flag_enable_system;
#elif BUILD_LEVEL == 3
    tx->monitor[14] = neg_current_ctrl.idqn.dat[phase_d];
    tx->monitor[15] = neg_current_ctrl.idqn.dat[phase_q];
#elif BUILD_LEVEL == 4
    tx->monitor[14] = inv_ctrl.vdq_ff_external.dat[phase_d];
    tx->monitor[15] = inv_ctrl.vdq_ff_decouple.dat[phase_d];
#elif BUILD_LEVEL == 5
    tx->monitor[14] = pq_ctrl.pq_meas.dat[0];
    tx->monitor[15] = pq_ctrl.pq_meas.dat[1];
#endif // BUILD_LEVEL
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_

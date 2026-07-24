/**
 * @file ctl_main.cpp
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <xplt.peripheral.h>

//=================================================================================================
// include Necessary control modules

#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/digital_power/inv/gfl_core.h>
#include <ctl/component/digital_power/inv/gfl_pq_ctrl.h>
#include <ctl/component/digital_power/inv/inv_hcm.h>
#include <ctl/component/digital_power/inv/inv_neg_ctrl.h>
#include <ctl/component/digital_power/inv/rectifier_dc_voltage_ctrl.h>

#include <ctl/component/interface/spwm_modulator.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>

#include <ctl/framework/cia402_state_machine.h>

#include "gfl_validation_options.h"

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// extern controller modules

// System framework
extern cia402_sm_t cia402_sm;

// Control Law Core
extern gfl_inv_ctrl_init_t gfl_init;
extern gfl_inv_ctrl_t inv_ctrl;
extern gfl_pq_ctrl_t pq_ctrl;
extern inv_neg_ctrl_init_t gfl_neg_init;
extern inv_neg_ctrl_t neg_current_ctrl;

// Input channel

// Output channel
#if defined USING_NPC_MODULATOR
extern npc_modulator_t spwm;
#else
extern spwm_modulator_t spwm;
#endif // USING_NPC_MODULATOR

// Protection module

// ADC Calibrator
extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;
extern uint32_t pq_loop_tick;
extern ctl_vector2_t gfl_current_ref_target;
extern ctl_slope_limiter_t gfl_current_ref_ramp[2];

// User commands
typedef enum _tag_rectifier_state_t
{
    RECTIFIER_STATE_IDLE = 0,
    RECTIFIER_STATE_WAIT_PLL,
    RECTIFIER_STATE_ZERO_CURRENT,
    RECTIFIER_STATE_VOLTAGE_RAMP,
    RECTIFIER_STATE_RUN,
    RECTIFIER_STATE_FAULT
} rectifier_state_t;

enum
{
    RECTIFIER_FAULT_NONE = 0,
    RECTIFIER_FAULT_SOFTWARE_OVERCURRENT = (1u << 0),
    RECTIFIER_FAULT_DC_OVERVOLTAGE = (1u << 1),
    RECTIFIER_FAULT_DC_UNDERVOLTAGE = (1u << 2),
    RECTIFIER_FAULT_PLL_LOSS = (1u << 3),
    RECTIFIER_FAULT_ADC_INVALID = (1u << 4)
};

typedef struct _tag_rectifier_cmd_t
{
    uint16_t enable;
    uint16_t clear_fault;

    ctrl_gt udc_ref_v;
    ctrl_gt iq_ref_pu;
    ctrl_gt current_limit_pu;

    uint16_t manual_current_mode;
    ctrl_gt manual_id_ref_pu;
    ctrl_gt manual_iq_ref_pu;
} rectifier_cmd_t;

typedef struct _tag_rectifier_status_t
{
    uint16_t state;
    uint32_t fault_bits;

    ctrl_gt udc_ref_v;
    ctrl_gt udc_meas_v;
    ctrl_gt udc_error_v;

    ctrl_gt id_ref_pu;
    ctrl_gt iq_ref_pu;
    ctrl_gt id_meas_pu;
    ctrl_gt iq_meas_pu;

    ctrl_gt pll_freq_pu;
    ctrl_gt pll_error;

    uint16_t pll_locked;
    uint16_t pwm_enabled;
    uint16_t current_limited;
    uint16_t voltage_pi_saturated;
} rectifier_status_t;

extern rectifier_dc_voltage_ctrl_t rectifier_dc_voltage_ctrl;
extern rectifier_cmd_t rectifier_cmd;
extern rectifier_status_t rectifier_status;

void ctl_step_rectifier_bl6(void);

//=================================================================================================
// controller process

// periodic callback function things.
GMP_STATIC_INLINE void ctl_dispatch(void)
{
    // ADC calibrator routine
    if (flag_enable_adc_calibrator)
    {
        if (index_adc_calibrator == 13)
            ctl_step_adc_calibrator(&adc_calibrator, idc.control_port.value);
        else if (index_adc_calibrator == 12)
            ctl_step_adc_calibrator(&adc_calibrator, udc.control_port.value);
        else if (index_adc_calibrator <= 11 && index_adc_calibrator >= 9)
            ctl_step_adc_calibrator(&adc_calibrator, uuvw.control_port.value.dat[index_adc_calibrator - 9]);
        else if (index_adc_calibrator <= 8 && index_adc_calibrator >= 6)
            ctl_step_adc_calibrator(&adc_calibrator, vabc.control_port.value.dat[index_adc_calibrator - 6]);
        else if (index_adc_calibrator <= 5 && index_adc_calibrator >= 3)
            ctl_step_adc_calibrator(&adc_calibrator, iabc.control_port.value.dat[index_adc_calibrator - 3]);
        else if (index_adc_calibrator <= 2)
            ctl_step_adc_calibrator(&adc_calibrator, iuvw.control_port.value.dat[index_adc_calibrator]);
    }

    // normal controller routine
    else
    {
        // Apply the commissioning reference through a rate limiter.  The
        // limiter is held at zero while the algorithm is stopped, so a
        // CiA402 restart never inherits a stale or instantaneous current step.
        if (inv_ctrl.flag_enable_system)
        {
            ctl_set_gfl_inv_current(
                &inv_ctrl,
                ctl_step_slope_limiter(&gfl_current_ref_ramp[phase_d],
                                       gfl_current_ref_target.dat[phase_d]),
                ctl_step_slope_limiter(&gfl_current_ref_ramp[phase_q],
                                       gfl_current_ref_target.dat[phase_q]));
        }
        else
        {
            ctl_clear_slope_limiter(&gfl_current_ref_ramp[phase_d]);
            ctl_clear_slope_limiter(&gfl_current_ref_ramp[phase_q]);
            ctl_set_gfl_inv_current(&inv_ctrl, 0, 0);
        }

        // The common GFL core accepts the measured grid voltage as an
        // externally supplied dq feed-forward term.  It is only populated for
        // BL4/BL5; lower levels remain a pure current-loop baseline.
#if BUILD_LEVEL >= 4 && GMP_GFL_ENABLE_EXTERNAL_FEEDFORWARD
        ctl_vector2_copy(&inv_ctrl.vdq_ff_external, &inv_ctrl.vdq);
#else
        ctl_vector2_clear(&inv_ctrl.vdq_ff_external);
#endif

        // run controller body
        ctl_step_gfl_inv_ctrl(&inv_ctrl);
        ctl_step_neg_inv_ctrl(&neg_current_ctrl);

#if BUILD_LEVEL == 6
        // The voltage loop and rectifier state machine consume the freshly
        // filtered Udc/Id/Iq/PLL state and prepare the next ISR reference.
        ctl_step_rectifier_bl6();
#endif

        // The validation profile is compile-time opt-in and therefore cannot
        // change hardware-target behaviour. Its timestamps are measured from
        // the controller ISR tick: P steps to +0.05 pu at 0.30 s and Q steps
        // to +0.02 pu at 0.55 s.
#if BUILD_LEVEL == 5 && GMP_GFL_PQ_PROFILE
        if (inv_ctrl.isr_tick >= (uint32_t)(CONTROLLER_FREQUENCY * 0.30f))
        {
            ctl_set_gfl_pq_ref(&pq_ctrl, float2ctrl(0.05f), pq_ctrl.pq_set.dat[1]);
        }
        if (inv_ctrl.isr_tick >= (uint32_t)(CONTROLLER_FREQUENCY * 0.55f))
        {
            ctl_set_gfl_pq_ref(&pq_ctrl, pq_ctrl.pq_set.dat[0], float2ctrl(0.02f));
        }
#endif

        // Run the P/Q outer loop at its own lower rate. The current loop keeps
        // executing every ISR and consumes the most recent current reference.
#if BUILD_LEVEL == 5
        ++pq_loop_tick;
        if (pq_loop_tick >= GFL_PQ_LOOP_DIVIDER)
        {
            pq_loop_tick = 0;
            ctl_step_gfl_pq(&pq_ctrl);

            if (pq_ctrl.flag_enable)
            {
                gfl_current_ref_target.dat[phase_d] = pq_ctrl.idq_set_out.dat[phase_d];
                gfl_current_ref_target.dat[phase_q] = pq_ctrl.idq_set_out.dat[phase_q];
            }
        }
#endif

        // Mix all output.  The legacy inverter levels use CTRL_VOLTAGE_BASE
        // with a fixed 2*base DC source.  BL6 has a variable DC link, so
        // convert the voltage command to modulation index using measured Udc.
#if BUILD_LEVEL == 6
        ctrl_gt rectifier_modulation_scale = 0;
        ctrl_gt rectifier_udc_pu_min =
            float2ctrl(RECTIFIER_DC_UNDERVOLTAGE_V / CTRL_VOLTAGE_BASE);
        if (inv_ctrl.filter_udc.out >= rectifier_udc_pu_min)
            rectifier_modulation_scale = ctl_div(float2ctrl(2.0f), inv_ctrl.filter_udc.out);
        spwm.vab0_out.dat[phase_A] =
            ctl_mul(inv_ctrl.vab0_out.dat[phase_A], rectifier_modulation_scale);
        spwm.vab0_out.dat[phase_B] =
            ctl_mul(inv_ctrl.vab0_out.dat[phase_B], rectifier_modulation_scale);
        spwm.vab0_out.dat[phase_0] =
            ctl_mul(inv_ctrl.vab0_out.dat[phase_0], rectifier_modulation_scale);
#else
        spwm.vab0_out.dat[phase_A] = inv_ctrl.vab0_out.dat[phase_A] + neg_current_ctrl.vab_out.dat[phase_A];
        spwm.vab0_out.dat[phase_B] = inv_ctrl.vab0_out.dat[phase_B] + neg_current_ctrl.vab_out.dat[phase_B];
        spwm.vab0_out.dat[phase_0] = inv_ctrl.vab0_out.dat[phase_0];
#endif

        // modulation
#if defined USING_NPC_MODULATOR
        ctl_step_npc_modulator(&spwm);
#else
        ctl_step_spwm_modulator(&spwm);
#endif // USING_NPC_MODULATOR
    }
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_

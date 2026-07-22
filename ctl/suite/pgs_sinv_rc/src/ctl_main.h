/**
 * @file ctl_main.h
 * @author GMP Library Contributors
 * @brief Top-level Controller for Single-Phase Inverter/AFE.
 * @version 1.0
 * @date 2026-05-05
 *
 * @copyright Copyright GMP(c) 2024-2026
 *
 */

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#include <xplt.peripheral.h>

#include <core/pm/function_scheduler.h>

#include <core/dev/pil_core.h>

#include <ctl/component/interface/adc_channel.h>
#include <ctl/framework/cia402_state_machine.h>

// SINV Control Modules
#include <ctl/component/digital_power/sinv/sinv_protect.h>
#include <ctl/component/digital_power/sinv/sinv_rc_core.h>
#include <ctl/component/digital_power/sinv/sinv_ref_gen.h>
#include <ctl/component/digital_power/sinv/sinv_outer_loop.h>
#include <ctl/component/digital_power/sinv/sms_pq.h>
#include <ctl/component/digital_power/sinv/spll_sogi.h>
#include <ctl/component/interface/hpwm_modulator.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>

/* The simulation project must select its BL2 application explicitly. Other
   targets retain their historical BL2 behavior unless they add APP_MODE. */
#if BUILD_LEVEL == 2
#if defined(SPECIFY_PC_ENVIRONMENT) && !defined(SINV_APP_MODE)
#error SINV_BL2_simulation_requires_explicit_SINV_APP_MODE
#endif
#if defined(SINV_APP_MODE)
#if !defined(SINV_APP_MODE_STANDARD_BL2) || \
    !defined(SINV_APP_MODE_2023A_SINGLE)
#error SINV_APP_MODE_constants_are_missing
#endif
#if (SINV_APP_MODE != SINV_APP_MODE_STANDARD_BL2) && \
    (SINV_APP_MODE != SINV_APP_MODE_2023A_SINGLE)
#error SINV_APP_MODE_is_invalid
#endif
#endif
#endif

#if BUILD_LEVEL == 2 && defined(SINV_APP_MODE) && \
    defined(SINV_APP_MODE_2023A_SINGLE) && \
    (SINV_APP_MODE == SINV_APP_MODE_2023A_SINGLE)
#define SINV_2023A_SINGLE_MODE_ACTIVE
#if !defined(SINV_2023A_UO_REF_RMS_V) || \
    !defined(SINV_2023A_OUTPUT_FREQ_HZ) || \
    !defined(SINV_2023A_SOFTSTART_TIME_S) || \
    !defined(SINV_2023A_VOLTAGE_LOOP_KP) || \
    !defined(SINV_2023A_VOLTAGE_LOOP_KR) || \
    !defined(SINV_2023A_VOLTAGE_LOOP_WI_HZ) || \
    !defined(SINV_2023A_CURRENT_REF_LIMIT_PEAK_PU) || \
    !defined(SINV_2023A_VOLTAGE_LOOP_OUTPUT_LIMIT_PU) || \
    !defined(SINV_2023A_ENABLE_DEADTIME_COMP) || \
    !defined(SINV_2023A_ENABLE_LOAD_CURRENT_FF) || \
    !defined(SINV_2023A_LOAD_CURRENT_FF_GAIN) || \
    !defined(SINV_2023A_ENABLE_CAP_CURRENT_FF) || \
    !defined(SINV_2023A_CAP_CURRENT_FF_GAIN)
#error SINV_2023A_voltage_loop_parameters_are_missing
#endif
#if (SINV_2023A_ENABLE_DEADTIME_COMP != 0) && \
    (SINV_2023A_ENABLE_DEADTIME_COMP != 1)
#error SINV_2023A_ENABLE_DEADTIME_COMP_must_be_0_or_1
#endif
#if (SINV_2023A_ENABLE_LOAD_CURRENT_FF != 0) && \
    (SINV_2023A_ENABLE_LOAD_CURRENT_FF != 1)
#error SINV_2023A_ENABLE_LOAD_CURRENT_FF_must_be_0_or_1
#endif
#if (SINV_2023A_ENABLE_CAP_CURRENT_FF != 0) && \
    (SINV_2023A_ENABLE_CAP_CURRENT_FF != 1)
#error SINV_2023A_ENABLE_CAP_CURRENT_FF_must_be_0_or_1
#endif
#include "ctl_2023a_voltage_loop.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// extern controller modules

// System framework
extern cia402_sm_t cia402_sm;

// Control Law Core
extern spll_sogi_t pll;
extern ctl_sms_pq_t pq_meter;
extern ctl_sinv_ref_gen_t ref_gen;
extern ctl_sinv_rc_core_t rc_core;
extern ctl_sinv_outer_loop_t outer_loop;
extern ctl_ramp_generator_t rg;

#ifdef SINV_2023A_SINGLE_MODE_ACTIVE
extern ctl_2023a_voltage_loop_t voltage_loop_2023a;
#endif

// Input channel
extern adc_channel_t adc_v_grid;
extern adc_channel_t adc_i_ac;
extern adc_channel_t adc_v_bus;
#ifdef SINV_2023A_SINGLE_MODE_ACTIVE
extern adc_channel_t adc_i_load;
#endif

// Output channel
extern single_phase_H_modulation_t hpwm;

// Protection module
extern ctl_sinv_protect_t protection;

// ADC Calibrator
extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;

// User commands
extern ctrl_gt g_p_ref_user;
extern ctrl_gt g_q_ref_user;
extern ctrl_gt g_vbus_ref_user;

extern ctrl_gt openloop_v_ref;
extern vector2_gt phasor;

//=================================================================================================
// function prototype
void clear_all_controllers(void);
void ctl_init(void);
void ctl_mainloop(void);
fast_gt ctl_exec_adc_calibration(void);
fast_gt ctl_exec_dc_voltage_ready(void);
fast_gt ctl_check_pll_locked(void);
fast_gt ctl_check_compliance(void);
fast_gt ctl_fault_recover_routine(void);

//=================================================================================================
// Background Controller Tasks

//=================================================================================================
// controller process

/**
 * @brief periodic callback function things.
 * @details Executed at the highest ISR frequency (e.g., 20kHz).
 */
GMP_STATIC_INLINE void ctl_dispatch(void)
{
    // ADC input will handled by input process



    // ADC calibrator routine
    if (flag_enable_adc_calibrator)
    {
        // Calibrate only the zero-current channel. Grid voltage can be present
        // during startup and therefore is not a valid zero-offset source.
        ctl_step_adc_calibrator(&adc_calibrator, adc_i_ac.control_port.value);
    }
    // normal controller routine
    else
    {
        // 1. Grid Synchronization (PLL)
        ctl_step_single_phase_pll(&pll, adc_v_grid.control_port.value);

        // 2. Real-time PQ Measurement
        ctl_step_sms_pq(&pq_meter, pll.uab.dat[phase_alpha], pll.uab.dat[phase_beta], adc_i_ac.control_port.value);

        // 3. Command generation for the selected commissioning level.
        if (cia402_sm.state_word.bits.operation_enabled)
        {
#if BUILD_LEVEL <= 2
            ctl_step_ramp_generator(&rg);
            ctl_set_phasor_via_angle(rg.current, &phasor);
#endif
#if BUILD_LEVEL == 1
            ctl_clear_sinv_ref_gen(&ref_gen);
#elif BUILD_LEVEL == 2
#ifdef SINV_2023A_SINGLE_MODE_ACTIVE
            ref_gen.i_ref_inst = ctl_step_2023a_voltage_loop(
                &voltage_loop_2023a,
                phasor.dat[phasor_sin], phasor.dat[phasor_cos],
                adc_v_grid.control_port.value,
                adc_i_load.control_port.value);
#else
            ref_gen.i_ref_inst = ctl_mul(float2ctrl(SINV_LEVEL2_CURRENT_REF_PEAK_PU),
                                         phasor.dat[phasor_sin]);
#endif
#elif BUILD_LEVEL == 3
            ctl_step_sinv_ref_gen_pq(&ref_gen, g_p_ref_user, g_q_ref_user, ctl_abs(pll.v_mag), &pll.phasor);
#elif BUILD_LEVEL == 4
            ctl_step_sinv_ref_gen_pq(&ref_gen,
                ctl_step_sinv_power_loop(&outer_loop, g_p_ref_user, pq_meter.active_power_p),
                g_q_ref_user, ctl_abs(pll.v_mag), &pll.phasor);
#elif BUILD_LEVEL == 5
            ctl_step_sinv_ref_gen_pq(&ref_gen,
                ctl_step_sinv_dc_bus_loop(&outer_loop, g_vbus_ref_user,
                    adc_v_bus.control_port.value, float2ctrl(-1.0f)),
                float2ctrl(0.0f), ctl_abs(pll.v_mag), &pll.phasor);
#endif
        }
        else
        {
            ctl_clear_sinv_ref_gen(&ref_gen);
            ctl_clear_sinv_outer_loop(&outer_loop);
#ifdef SINV_2023A_SINGLE_MODE_ACTIVE
            ctl_clear_2023a_voltage_loop(&voltage_loop_2023a);
            rg.current = rg.minimum;
#endif
        }

        // 4. Inner Current Controller (RC Core)
#if BUILD_LEVEL == 1
        rc_core.flag_enable_ctrl = 0;
#else
        rc_core.flag_enable_ctrl = cia402_sm.state_word.bits.operation_enabled;
#endif

#ifdef SINV_2023A_SINGLE_MODE_ACTIVE
        // Standalone FDRC follows the commanded oscillator, never the PLL.
        rc_core.fundamental_freq = SINV_2023A_OUTPUT_FREQ_HZ;
#elif BUILD_LEVEL >= 3
        // PLL frequency is per-unit; FDRC requires the actual fundamental in Hz.
        rc_core.fundamental_freq = CTRL_GRID_FREQUENCY * ctrl2float(pll.frequency);
#else
        rc_core.fundamental_freq = CTRL_GRID_FREQUENCY;
#endif

        // Pass I_ref from ref generator. Fdbk ptrs (ADC) are already zero-copy bound in init()
        ctl_step_sinv_rc_core(&rc_core, ref_gen.i_ref_inst);

        // 5. Fast Protection Callback (ISR Level)
        if (ctl_step_sinv_protect_fast(&protection, adc_v_bus.control_port.value, adc_i_ac.control_port.value,
                                       rc_core.v_out_unsat))
        {
            cia402_fault_request(&cia402_sm);
        }

        // PWM Modulation
        if (cia402_sm.state_word.bits.operation_enabled)
        {
#if BUILD_LEVEL == 1
            ctl_step_single_phase_H_modulation(&hpwm, ctl_mul(openloop_v_ref, phasor.dat[phasor_sin]), adc_i_ac.control_port.value);
#else
            ctl_step_single_phase_H_modulation(&hpwm, rc_core.v_out_ref, adc_i_ac.control_port.value);
#endif // BUILD_LEVEL


        }
        else
        {
            ctl_clear_single_phase_H_modulation(&hpwm);
        }

    }
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_

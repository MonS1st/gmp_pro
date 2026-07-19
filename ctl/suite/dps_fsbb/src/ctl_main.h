/**
 * @file ctl_main.h
 * @brief Top-level controller interface for the four-switch buck-boost converter.
 */

#ifndef _FILE_CTRL_MAIN_H_
#define _FILE_CTRL_MAIN_H_

#ifndef SDPE_FSBB_SETTINGS_HEADER
#define SDPE_FSBB_SETTINGS_HEADER <sdpe_dps_fsbb_iris_settings.h>
#endif
#include SDPE_FSBB_SETTINGS_HEADER
#include "fsbb_build4_controller.h"
#include "fsbb_inner_current_loop.h"
#include <core/dev/pil_core.h>
#include <core/pm/function_scheduler.h>
#include <ctl/component/digital_power/dcdc/dcdc_core.h>
#include <ctl/component/digital_power/dcdc/fsbb.h>
#include <ctl/component/interface/adc_channel.h>
#include <ctl/framework/cia402_state_machine.h>
#ifdef __cplusplus
extern "C"
{
#endif

extern cia402_sm_t cia402_sm;
extern ctl_dcdc_core_t dcdc_core;
extern fsbb_build4_controller_t fsbb_build4;

extern adc_channel_t adc_v_in;
extern adc_channel_t adc_v_out;
extern adc_channel_t adc_i_L;
extern adc_channel_t adc_i_load;
extern fsbb_modulator_t fsbb_mod;

extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;

extern ctrl_gt g_v_out_ref_user;
extern ctrl_gt g_i_limit_user;
extern ctrl_gt v_req;

typedef enum _tag_fsbb_fault
{
    FSBB_FAULT_NONE = 0,
    FSBB_FAULT_VIN_UNDERVOLTAGE = 1U << 0,
    FSBB_FAULT_VIN_OVERVOLTAGE = 1U << 1,
    FSBB_FAULT_VOUT_OVERVOLTAGE = 1U << 2,
    FSBB_FAULT_IL_POSITIVE_OVERCURRENT = 1U << 3,
    FSBB_FAULT_IL_NEGATIVE_OVERCURRENT = 1U << 4,
    FSBB_FAULT_IOUT_OVERCURRENT = 1U << 5
} fsbb_fault_t;

typedef enum _tag_fsbb_fault_reset_result
{
    FSBB_FAULT_RESET_IDLE = 0,
    FSBB_FAULT_RESET_ACCEPTED,
    FSBB_FAULT_RESET_REJECTED
} fsbb_fault_reset_result_t;

extern volatile uint16_t g_fsbb_faults;
extern volatile fast_gt g_fsbb_output_enabled;
extern volatile fast_gt g_fsbb_fault_reset_result;

void ctl_init(void);
void ctl_mainloop(void);
void ctl_enable_pwm(void);
void ctl_disable_pwm(void);
void clear_all_controllers(void);
fast_gt ctl_fsbb_try_fault_reset(void);
gmp_task_status_t tsk_protect(gmp_task_t* tsk);

/** Return faults present in the latest unfiltered ADC sample set. */
GMP_STATIC_INLINE uint16_t ctl_fsbb_active_faults(void)
{
    uint16_t faults = FSBB_FAULT_NONE;

#if defined FSBB_ENABLE_VIN_SAMPLE
    if (adc_v_in.control_port.value < float2ctrl(FSBB_INPUT_VOLTAGE_MIN / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VIN_UNDERVOLTAGE;
    if (adc_v_in.control_port.value > float2ctrl(FSBB_INPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VIN_OVERVOLTAGE;
#endif
#if !defined FSBB_VOUT_SENSOR_CALIBRATED || (FSBB_VOUT_SENSOR_CALIBRATED == 1)
    if (adc_v_out.control_port.value > float2ctrl(FSBB_OUTPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VOUT_OVERVOLTAGE;
#endif
    if (adc_i_L.control_port.value > float2ctrl(FSBB_PROTECT_IL_MAX / CTRL_CURRENT_BASE))
        faults |= FSBB_FAULT_IL_POSITIVE_OVERCURRENT;
    if (adc_i_L.control_port.value < float2ctrl(FSBB_PROTECT_IL_MIN / CTRL_CURRENT_BASE))
        faults |= FSBB_FAULT_IL_NEGATIVE_OVERCURRENT;
#if defined FSBB_ENABLE_IOUT_SAMPLE && \
    (!defined FSBB_IOUT_SENSOR_CALIBRATED || (FSBB_IOUT_SENSOR_CALIBRATED == 1))
    if ((adc_i_load.control_port.value > float2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE)) ||
        (adc_i_load.control_port.value < -float2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE)))
        faults |= FSBB_FAULT_IOUT_OVERCURRENT;
#endif

    return faults;
}

/** Execute one control sample after the platform input callback has run. */
GMP_STATIC_INLINE void ctl_dispatch(void)
{
#if defined FSBB_HARDWARE_SENSOR_CALIBRATION_MODE && (FSBB_HARDWARE_SENSOR_CALIBRATION_MODE == 1)
    /*
     * Hardware calibration mode is acquisition-only.  Keep every command at
     * zero and reassert the hardware shutdown on every control interrupt.
     */
    v_req = float2ctrl(0.0f);
    dcdc_core.v_out_formal = float2ctrl(0.0f);
    ctl_disable_pwm();
    return;
#endif

#if defined SPECIFY_ENABLE_ADC_CALIBRATE
    if (flag_enable_adc_calibrator)
    {
        if (index_adc_calibrator == 0)
            ctl_step_adc_calibrator(&adc_calibrator, adc_i_L.control_port.value);
#if defined FSBB_ENABLE_IOUT_SAMPLE
        else if (index_adc_calibrator == 1)
            ctl_step_adc_calibrator(&adc_calibrator, adc_i_load.control_port.value);
#endif
        return;
    }
#endif

    if (g_fsbb_faults != FSBB_FAULT_NONE)
    {
        /*
        * Prevent the previous control command from being reused
        * by diagnostic or subsequent control logic.
        */
        v_req = float2ctrl(0.0f);

        /*
        * Disable PWM immediately instead of waiting for the
        * slower CiA402 fault-state transition.
        */
        dcdc_core.v_out_formal = float2ctrl(0.0f);
        clear_all_controllers();
        ctl_disable_pwm();
        return;
    }

#if (BUILD_LEVEL == 1)
    dcdc_core.mode = CTL_DCDC_MODE_OPENLOOP;
    dcdc_core.v_target = float2ctrl(FSBB_OPEN_LOOP_VOLTAGE_COMMAND / CTRL_VOLTAGE_BASE);
    v_req = ctl_step_dcdc_open_loop(&dcdc_core);
#elif (BUILD_LEVEL == 2)
    dcdc_core.mode = CTL_DCDC_MODE_CURRENTLOOP;
    dcdc_core.i_target =
        ctl_sat(g_i_limit_user, float2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE), float2ctrl(0.0f));
    v_req = ctl_step_dcdc_current_loop(&dcdc_core);
#elif (BUILD_LEVEL == 3)
    {
        ctrl_gt current_limit =
            ctl_sat(g_i_limit_user, float2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE), float2ctrl(0.0f));
        dcdc_core.mode = CTL_DCDC_MODE_VOLTAGELOOP;
        dcdc_core.v_target = ctl_sat(g_v_out_ref_user, float2ctrl(FSBB_OUTPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE),
                                     float2ctrl(FSBB_OUTPUT_VOLTAGE_MIN / CTRL_VOLTAGE_BASE));
        ctl_set_pid_limit(&dcdc_core.voltage_pid, current_limit, float2ctrl(0.0f));
        ctl_set_pid_int_limit(&dcdc_core.voltage_pid, current_limit, float2ctrl(0.0f));
        v_req = ctl_step_dcdc_cascade(&dcdc_core);
    }
#elif (BUILD_LEVEL == 4)

    {
#if (FSBB_BUILD4_SELF_TEST_ENABLE == 1)
        /*
         * Dynamic CV/CC switching test.
         *
         * Stage 1: CV = 0.8 A, CC = 1.0 A -> CV mode
         * Stage 2: CV = 0.8 A, CC = 0.6 A -> CC mode
         * Stage 3: CV = 0.8 A, CC = 1.0 A -> CV mode
         */
        static uint32_t build4_test_counter = 0U;

        ctrl_gt i_L_ref_cv_test = float2ctrl(0.8f / CTRL_CURRENT_BASE);

        ctrl_gt i_L_ref_cc_test;

        /*
         * Keep test timing relative to PWM enable.
         */

        if (!g_fsbb_output_enabled)
        {
            build4_test_counter = 0U;
        }
        else
        {
            build4_test_counter++;
        }

        if (build4_test_counter < (uint32_t)(0.2f * CONTROLLER_FREQUENCY))
        {
            /* Clearly inside CV region */
            i_L_ref_cc_test = float2ctrl(0.85f / CTRL_CURRENT_BASE);
        }
        else if (build4_test_counter < (uint32_t)(0.4f * CONTROLLER_FREQUENCY))
        {
            /* CC is slightly lower, but still inside hysteresis band */
            i_L_ref_cc_test = float2ctrl(0.79f / CTRL_CURRENT_BASE);
        }
        else if (build4_test_counter < (uint32_t)(0.6f * CONTROLLER_FREQUENCY))
        {
            /* Cross the CV-to-CC threshold */
            i_L_ref_cc_test = float2ctrl(0.77f / CTRL_CURRENT_BASE);
        }
        else if (build4_test_counter < (uint32_t)(0.8f * CONTROLLER_FREQUENCY))
        {
            /* Rise inside the hysteresis band; retain CC */
            i_L_ref_cc_test = float2ctrl(0.81f / CTRL_CURRENT_BASE);
        }
        else
        {
            /* Cross the CC-to-CV threshold */
            i_L_ref_cc_test = float2ctrl(0.83f / CTRL_CURRENT_BASE);
        }
        /*
        * Temporary fault injection for fast-shutdown verification.
         * Trigger after all five selector stages have been observed.
        */
        if (build4_test_counter == (uint32_t)(1.0f * CONTROLLER_FREQUENCY))
        {
            g_fsbb_faults |= FSBB_FAULT_IL_POSITIVE_OVERCURRENT;
        }

        v_req = ctl_step_fsbb_build4_self_test(&fsbb_build4, &dcdc_core, i_L_ref_cv_test, i_L_ref_cc_test);
#else
        dcdc_core.mode = CTL_DCDC_MODE_VOLTAGELOOP;
        dcdc_core.v_target = ctl_sat(g_v_out_ref_user, float2ctrl(FSBB_OUTPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE),
                                     float2ctrl(FSBB_OUTPUT_VOLTAGE_MIN / CTRL_VOLTAGE_BASE));
        dcdc_core.i_target =
            ctl_sat(g_i_limit_user, float2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE), float2ctrl(0.0f));
        v_req = ctl_step_fsbb_build4(&fsbb_build4, &dcdc_core, dcdc_core.v_target, dcdc_core.i_target);
#endif
    }

#endif

    ctl_step_fsbb_modulator(&fsbb_mod, v_req, adc_v_in.control_port.value);
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_CTRL_MAIN_H_

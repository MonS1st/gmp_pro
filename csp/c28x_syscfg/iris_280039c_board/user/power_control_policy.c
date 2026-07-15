#include "power_control_policy.h"

#include "analog_io_test.h"
#include "power_app.h"
#include "power_mode_monitor.h"

#include <ctrl_settings.h>

volatile uint16_t g_control_policy_enabled = 0U;
volatile uint16_t g_control_strategy = PSU_CONTROL_STRATEGY_AUTO;

volatile uint16_t g_control_policy_fault = PSU_POLICY_FAULT_NONE;
volatile uint16_t g_control_policy_fault_latched = 0U;

volatile uint16_t g_control_policy_mismatch_candidate =
    PSU_POLICY_FAULT_NONE;
volatile uint16_t g_control_policy_mismatch_confirm_count = 0U;

volatile uint16_t g_control_policy_target_active = 0U;
volatile uint16_t g_control_policy_using_test_mode = 0U;

volatile uint16_t g_control_policy_reset_requested = 0U;

volatile uint16_t g_control_policy_test_trip_enable = 0U;

volatile uint16_t g_control_policy_trip_actual_mode =
    PSU_MODE_DISPLAY_OFF;
volatile uint16_t g_control_policy_trip_strategy =
    PSU_CONTROL_STRATEGY_AUTO;

volatile uint16_t g_control_policy_trip_voltage_set_mv = 0U;
volatile uint16_t g_control_policy_trip_current_set_ma = 0U;
volatile uint16_t g_control_policy_trip_voltage_meas_mv = 0U;
volatile uint16_t g_control_policy_trip_current_meas_ma = 0U;

volatile uint32_t g_control_policy_strategy_change_count = 0U;
volatile uint32_t g_control_policy_strategy_reject_count = 0U;

volatile uint32_t g_control_policy_mismatch_count = 0U;
volatile uint32_t g_control_policy_cv_lost_count = 0U;
volatile uint32_t g_control_policy_cc_lost_count = 0U;

volatile uint32_t g_control_policy_fault_shutdown_count = 0U;
volatile uint32_t g_control_policy_reset_count = 0U;
volatile uint32_t g_control_policy_reset_reject_count = 0U;

static uint16_t s_control_policy_last_voltage_set_mv = 0U;
static uint16_t s_control_policy_last_current_set_ma = 0U;

static void power_control_policy_clear_mismatch(void)
{
    g_control_policy_mismatch_candidate = PSU_POLICY_FAULT_NONE;
    g_control_policy_mismatch_confirm_count = 0U;
}

static bool power_control_policy_hard_fault_active(void)
{
    return g_power_app.fault_latched ||
           (g_power_app.fault != POWER_FAULT_NONE) ||
           (g_analog_board_feedback_fault != 0U) ||
           (g_analog_board_fault_hold_active != 0U) ||
           (g_analog_board_fault_shutdown_active != 0U);
}

static bool power_control_policy_reset_allowed(void)
{
    return (g_output_switch_requested == 0U) &&
           (g_output_switch_active == 0U) &&
           (g_output_switch_precharge_active == 0U) &&
           (g_power_app.state != POWER_STATE_STARTING) &&
           !power_control_policy_hard_fault_active();
}

static bool power_control_policy_strategy_change_allowed(void)
{
    return (g_control_policy_enabled == 1U) &&
           (g_power_app.state == POWER_STATE_OFF) &&
           (g_output_switch_requested == 0U) &&
           (g_output_switch_active == 0U) &&
           (g_output_switch_precharge_active == 0U) &&
           !power_control_policy_hard_fault_active() &&
           !power_control_policy_fault_active();
}

static bool power_control_policy_test_trip_allowed(void)
{
#if PSU_CONTROL_POLICY_TEST_TRIP_ENABLE
    return (g_mode_monitor_using_test_data == 1U) &&
           (g_mode_monitor_test_enable == 1U) &&
           (g_control_policy_test_trip_enable == 1U);
#else
    return false;
#endif
}

static void power_control_policy_capture_trip(psu_policy_fault_t fault)
{
    uint16_t voltage_meas_mv;
    uint16_t current_meas_ma;

    if (g_mode_monitor_using_test_data != 0U)
    {
        voltage_meas_mv = g_mode_monitor_test_voltage_mv;
        current_meas_ma = g_mode_monitor_test_current_ma;
    }
    else
    {
        voltage_meas_mv = g_power_app.voltage_meas_mv;
        current_meas_ma = g_power_app.current_meas_ma;
    }

    g_control_policy_trip_actual_mode = g_mode_monitor_display_mode;
    g_control_policy_trip_strategy = g_control_strategy;
    g_control_policy_trip_voltage_set_mv = g_power_app.voltage_set_mv;
    g_control_policy_trip_current_set_ma = g_power_app.current_set_ma;
    g_control_policy_trip_voltage_meas_mv = voltage_meas_mv;
    g_control_policy_trip_current_meas_ma = current_meas_ma;

    g_control_policy_fault = (uint16_t)fault;
    g_control_policy_fault_latched = 1U;
    if (fault == PSU_POLICY_FAULT_CV_REGULATION_LOST)
    {
        ++g_control_policy_cv_lost_count;
    }
    else if (fault == PSU_POLICY_FAULT_CC_REGULATION_LOST)
    {
        ++g_control_policy_cc_lost_count;
    }

    ++g_control_policy_fault_shutdown_count;
    power_app_output_switch_policy_shutdown();
}

static void power_control_policy_confirm_mismatch(psu_policy_fault_t fault)
{
    if (g_control_policy_mismatch_candidate != (uint16_t)fault)
    {
        g_control_policy_mismatch_candidate = (uint16_t)fault;
        g_control_policy_mismatch_confirm_count = 0U;
    }

    ++g_control_policy_mismatch_count;
    if (g_control_policy_mismatch_confirm_count <
        PSU_CONTROL_POLICY_MISMATCH_CONFIRM_COUNT)
    {
        ++g_control_policy_mismatch_confirm_count;
    }

    if ((g_control_policy_mismatch_confirm_count >=
         PSU_CONTROL_POLICY_MISMATCH_CONFIRM_COUNT) &&
        !power_control_policy_hard_fault_active())
    {
        power_control_policy_capture_trip(fault);
    }
}

static bool power_control_policy_command_changed(void)
{
    uint16_t voltage_set_mv = g_power_app.voltage_set_mv;
    uint16_t current_set_ma = g_power_app.current_set_ma;

    if ((voltage_set_mv == s_control_policy_last_voltage_set_mv) &&
        (current_set_ma == s_control_policy_last_current_set_ma))
    {
        return false;
    }

    s_control_policy_last_voltage_set_mv = voltage_set_mv;
    s_control_policy_last_current_set_ma = current_set_ma;
    power_control_policy_clear_mismatch();
    return true;
}

void power_control_policy_init(void)
{
    g_control_policy_enabled = PSU_CONTROL_POLICY_ENABLE ? 1U : 0U;
    g_control_strategy = PSU_CONTROL_POLICY_DEFAULT_STRATEGY;
    g_control_policy_fault = PSU_POLICY_FAULT_NONE;
    g_control_policy_fault_latched = 0U;
    g_control_policy_mismatch_candidate = PSU_POLICY_FAULT_NONE;
    g_control_policy_mismatch_confirm_count = 0U;
    g_control_policy_target_active = 0U;
    g_control_policy_using_test_mode = 0U;
    g_control_policy_reset_requested = 0U;
    g_control_policy_test_trip_enable = 0U;
    g_control_policy_trip_actual_mode = PSU_MODE_DISPLAY_OFF;
    g_control_policy_trip_strategy = PSU_CONTROL_STRATEGY_AUTO;
    g_control_policy_trip_voltage_set_mv = 0U;
    g_control_policy_trip_current_set_ma = 0U;
    g_control_policy_trip_voltage_meas_mv = 0U;
    g_control_policy_trip_current_meas_ma = 0U;
    g_control_policy_strategy_change_count = 0U;
    g_control_policy_strategy_reject_count = 0U;
    g_control_policy_mismatch_count = 0U;
    g_control_policy_cv_lost_count = 0U;
    g_control_policy_cc_lost_count = 0U;
    g_control_policy_fault_shutdown_count = 0U;
    g_control_policy_reset_count = 0U;
    g_control_policy_reset_reject_count = 0U;
    s_control_policy_last_voltage_set_mv = g_power_app.voltage_set_mv;
    s_control_policy_last_current_set_ma = g_power_app.current_set_ma;
}

bool power_control_policy_set_strategy(psu_control_strategy_t strategy)
{
    if (((uint16_t)strategy > (uint16_t)PSU_CONTROL_STRATEGY_CC_ONLY) ||
        !power_control_policy_strategy_change_allowed())
    {
        ++g_control_policy_strategy_reject_count;
        return false;
    }

    if (g_control_strategy != (uint16_t)strategy)
    {
        g_control_strategy = (uint16_t)strategy;
        ++g_control_policy_strategy_change_count;
    }
    power_control_policy_clear_mismatch();
    g_control_policy_target_active = 0U;
    return true;
}

psu_control_strategy_t power_control_policy_get_strategy(void)
{
    return (psu_control_strategy_t)g_control_strategy;
}

const char *power_control_policy_strategy_text(
    psu_control_strategy_t strategy)
{
    switch (strategy)
    {
    case PSU_CONTROL_STRATEGY_AUTO:
        return "AUTO";
    case PSU_CONTROL_STRATEGY_CV_ONLY:
        return "CV";
    case PSU_CONTROL_STRATEGY_CC_ONLY:
        return "CC";
    default:
        return "--";
    }
}

const char *power_control_policy_fault_text(psu_policy_fault_t fault)
{
    switch (fault)
    {
    case PSU_POLICY_FAULT_NONE:
        return "NONE";
    case PSU_POLICY_FAULT_CV_REGULATION_LOST:
        return "CV_LOST";
    case PSU_POLICY_FAULT_CC_REGULATION_LOST:
        return "CC_LOST";
    default:
        return "UNKNOWN";
    }
}

bool power_control_policy_fault_active(void)
{
    return (g_control_policy_fault_latched != 0U) ||
           (g_control_policy_fault != PSU_POLICY_FAULT_NONE);
}

bool power_control_policy_output_start_allowed(void)
{
    return !power_control_policy_fault_active();
}

void power_control_policy_request_reset(void)
{
    g_control_policy_reset_requested = 1U;
}

gmp_task_status_t power_control_policy_task(gmp_task_t *tsk)
{
    psu_control_strategy_t strategy;
    psu_mode_display_t actual_mode;
    bool command_changed;

    GMP_UNUSED_VAR(tsk);

    command_changed = power_control_policy_command_changed();
    g_control_policy_using_test_mode =
        (g_mode_monitor_using_test_data != 0U) ? 1U : 0U;

    if (g_control_policy_enabled != 1U)
    {
        power_control_policy_clear_mismatch();
        g_control_policy_target_active = 0U;
        g_control_policy_reset_requested = 0U;
        return GMP_TASK_DONE;
    }

    if (power_control_policy_hard_fault_active())
    {
        power_control_policy_clear_mismatch();
        g_control_policy_target_active = 0U;
        if (g_control_policy_reset_requested != 0U)
        {
            g_control_policy_reset_requested = 0U;
            if (g_control_policy_fault_latched != 0U)
            {
                ++g_control_policy_reset_reject_count;
            }
        }
        return GMP_TASK_DONE;
    }

    if (power_control_policy_fault_active())
    {
        if (g_control_policy_reset_requested != 0U)
        {
            g_control_policy_reset_requested = 0U;
            if (power_control_policy_reset_allowed())
            {
                g_control_policy_fault = PSU_POLICY_FAULT_NONE;
                g_control_policy_fault_latched = 0U;
                g_control_policy_target_active = 0U;
                power_control_policy_clear_mismatch();
                power_app_output_switch_policy_reset();
                ++g_control_policy_reset_count;
            }
            else
            {
                ++g_control_policy_reset_reject_count;
                power_app_output_switch_policy_shutdown();
            }
        }
        else
        {
            power_app_output_switch_policy_shutdown();
        }
        return GMP_TASK_DONE;
    }

    g_control_policy_reset_requested = 0U;

    if ((g_power_app.state == POWER_STATE_OFF) &&
        (g_output_switch_requested == 0U) &&
        (g_output_switch_active == 0U) &&
        (g_output_switch_precharge_active == 0U))
    {
        power_control_policy_clear_mismatch();
        g_control_policy_target_active = 0U;
        return GMP_TASK_DONE;
    }

    if ((g_power_app.state == POWER_STATE_STARTING) ||
        (g_output_switch_precharge_active != 0U))
    {
        power_control_policy_clear_mismatch();
        g_control_policy_target_active = 0U;
        return GMP_TASK_DONE;
    }

    if (g_output_switch_active == 0U)
    {
        power_control_policy_clear_mismatch();
        g_control_policy_target_active = 0U;
        return GMP_TASK_DONE;
    }

    if (g_mode_monitor_feedback_valid == 0U)
    {
        power_control_policy_clear_mismatch();
        g_control_policy_target_active = 0U;
        return GMP_TASK_DONE;
    }

    strategy = power_control_policy_get_strategy();
    actual_mode = (psu_mode_display_t)g_mode_monitor_display_mode;

    if (strategy == PSU_CONTROL_STRATEGY_AUTO)
    {
        g_control_policy_target_active = 1U;
        power_control_policy_clear_mismatch();
        return GMP_TASK_DONE;
    }

    if (strategy == PSU_CONTROL_STRATEGY_CV_ONLY)
    {
        g_control_policy_target_active =
            (g_power_app.voltage_set_mv != 0U) ? 1U : 0U;
    }
    else if (strategy == PSU_CONTROL_STRATEGY_CC_ONLY)
    {
        g_control_policy_target_active =
            (g_power_app.current_set_ma != 0U) ? 1U : 0U;
    }
    else
    {
        g_control_policy_target_active = 0U;
        power_control_policy_clear_mismatch();
        return GMP_TASK_DONE;
    }

    if ((g_control_policy_target_active == 0U) || command_changed)
    {
        power_control_policy_clear_mismatch();
        return GMP_TASK_DONE;
    }

    if ((g_mode_monitor_cv_confirm_count != 0U) ||
        (g_mode_monitor_cc_confirm_count != 0U))
    {
        return GMP_TASK_DONE;
    }

    if ((g_mode_monitor_using_test_data != 0U) &&
        !power_control_policy_test_trip_allowed())
    {
        power_control_policy_clear_mismatch();
        return GMP_TASK_DONE;
    }

    if (strategy == PSU_CONTROL_STRATEGY_CV_ONLY)
    {
        if (actual_mode == PSU_MODE_DISPLAY_CV)
        {
            power_control_policy_clear_mismatch();
        }
        else if (actual_mode == PSU_MODE_DISPLAY_CC)
        {
            power_control_policy_confirm_mismatch(
                PSU_POLICY_FAULT_CV_REGULATION_LOST);
        }
        return GMP_TASK_DONE;
    }

    if (actual_mode == PSU_MODE_DISPLAY_CC)
    {
        power_control_policy_clear_mismatch();
    }
    else if (actual_mode == PSU_MODE_DISPLAY_CV)
    {
        power_control_policy_confirm_mismatch(
            PSU_POLICY_FAULT_CC_REGULATION_LOST);
    }

    return GMP_TASK_DONE;
}

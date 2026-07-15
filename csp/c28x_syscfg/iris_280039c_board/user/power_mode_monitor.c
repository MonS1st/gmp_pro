#include "power_mode_monitor.h"

#include "analog_io_test.h"
#include "power_app.h"

#include <ctrl_settings.h>

volatile uint16_t g_mode_monitor_enabled = 0U;
volatile uint16_t g_mode_monitor_feedback_valid = 0U;
volatile uint16_t g_mode_monitor_display_mode = PSU_MODE_DISPLAY_OFF;

volatile uint16_t g_mode_monitor_test_enable = 0U;
volatile uint16_t g_mode_monitor_test_voltage_mv = 0U;
volatile uint16_t g_mode_monitor_test_current_ma = 0U;
volatile uint16_t g_mode_monitor_using_test_data = 0U;

volatile uint16_t g_mode_monitor_voltage_candidate = 0U;
volatile uint16_t g_mode_monitor_current_candidate = 0U;
volatile uint16_t g_mode_monitor_cv_confirm_count = 0U;
volatile uint16_t g_mode_monitor_cc_confirm_count = 0U;

volatile uint32_t g_mode_monitor_update_count = 0U;
volatile uint32_t g_mode_monitor_cv_enter_count = 0U;
volatile uint32_t g_mode_monitor_cc_enter_count = 0U;
volatile uint32_t g_mode_monitor_no_feedback_count = 0U;
volatile uint32_t g_mode_monitor_reset_count = 0U;

static uint16_t s_mode_monitor_last_voltage_set_mv = 0U;
static uint16_t s_mode_monitor_last_current_set_ma = 0U;

static bool power_mode_monitor_analog_fault_active(void)
{
    return (g_analog_board_feedback_fault != 0U) ||
           (g_analog_board_fault_hold_active != 0U) ||
           (g_analog_board_fault_shutdown_active != 0U);
}

static void power_mode_monitor_reset_candidates(void)
{
    if ((g_mode_monitor_voltage_candidate != 0U) ||
        (g_mode_monitor_current_candidate != 0U) ||
        (g_mode_monitor_cv_confirm_count != 0U) ||
        (g_mode_monitor_cc_confirm_count != 0U))
    {
        ++g_mode_monitor_reset_count;
    }

    g_mode_monitor_voltage_candidate = 0U;
    g_mode_monitor_current_candidate = 0U;
    g_mode_monitor_cv_confirm_count = 0U;
    g_mode_monitor_cc_confirm_count = 0U;
}

static void power_mode_monitor_set_non_regulating_mode(psu_mode_display_t mode)
{
    if (((g_mode_monitor_display_mode == (uint16_t)PSU_MODE_DISPLAY_CV) ||
         (g_mode_monitor_display_mode == (uint16_t)PSU_MODE_DISPLAY_CC)) &&
        (g_mode_monitor_voltage_candidate == 0U) &&
        (g_mode_monitor_current_candidate == 0U) &&
        (g_mode_monitor_cv_confirm_count == 0U) &&
        (g_mode_monitor_cc_confirm_count == 0U))
    {
        ++g_mode_monitor_reset_count;
    }

    power_mode_monitor_reset_candidates();
    g_mode_monitor_display_mode = (uint16_t)mode;
}

static void power_mode_monitor_enter_mode(psu_mode_display_t mode)
{
    if (g_mode_monitor_display_mode != (uint16_t)mode)
    {
        if (mode == PSU_MODE_DISPLAY_CV)
        {
            ++g_mode_monitor_cv_enter_count;
        }
        else if (mode == PSU_MODE_DISPLAY_CC)
        {
            ++g_mode_monitor_cc_enter_count;
        }
    }

    g_mode_monitor_display_mode = (uint16_t)mode;
    g_mode_monitor_cv_confirm_count = 0U;
    g_mode_monitor_cc_confirm_count = 0U;
}

bool power_mode_monitor_feedback_valid(void)
{
#if PSU_MODE_MONITOR_TEST_INJECTION
    if (g_mode_monitor_test_enable == 1U)
    {
        return true;
    }
#endif

#if PSU_REAL_FEEDBACK_CONNECTED
    return (g_analog_board_real_feedback_active == 1U) &&
           (g_analog_board_feedback_settled == 1U) &&
           (g_analog_board_feedback_fault == 0U) &&
           (g_analog_board_fault_hold_active == 0U) &&
           !g_power_app.fault_latched;
#else
    return false;
#endif
}

static void power_mode_monitor_evaluate(uint16_t voltage_meas_mv,
                                        uint16_t current_meas_ma,
                                        uint16_t voltage_set_mv,
                                        uint16_t current_set_ma)
{
    bool cc_candidate;
    bool cv_candidate;
    psu_mode_display_t mode =
        (psu_mode_display_t)g_mode_monitor_display_mode;

    cc_candidate =
        (((uint32_t)current_meas_ma + PSU_CC_ENTER_CURRENT_MARGIN_MA) >=
         (uint32_t)current_set_ma) &&
        (((uint32_t)voltage_meas_mv + PSU_CC_ENTER_VOLTAGE_DROP_MV) <
         (uint32_t)voltage_set_mv);
    cv_candidate =
        (((uint32_t)current_meas_ma + PSU_CV_RETURN_CURRENT_MARGIN_MA) <
         (uint32_t)current_set_ma) ||
        (((uint32_t)voltage_meas_mv + PSU_CV_RETURN_VOLTAGE_MARGIN_MV) >=
         (uint32_t)voltage_set_mv);

    g_mode_monitor_current_candidate = cc_candidate ? 1U : 0U;
    g_mode_monitor_voltage_candidate = cv_candidate ? 1U : 0U;

    if (mode == PSU_MODE_DISPLAY_CC)
    {
        g_mode_monitor_cc_confirm_count = 0U;
        if (cv_candidate)
        {
            if (g_mode_monitor_cv_confirm_count <
                PSU_MODE_MONITOR_CONFIRM_COUNT)
            {
                ++g_mode_monitor_cv_confirm_count;
            }
            if (g_mode_monitor_cv_confirm_count >=
                PSU_MODE_MONITOR_CONFIRM_COUNT)
            {
                power_mode_monitor_enter_mode(PSU_MODE_DISPLAY_CV);
            }
        }
        else
        {
            g_mode_monitor_cv_confirm_count = 0U;
        }
        return;
    }

    if (mode == PSU_MODE_DISPLAY_CV)
    {
        g_mode_monitor_cv_confirm_count = 0U;
        if (cc_candidate)
        {
            if (g_mode_monitor_cc_confirm_count <
                PSU_MODE_MONITOR_CONFIRM_COUNT)
            {
                ++g_mode_monitor_cc_confirm_count;
            }
            if (g_mode_monitor_cc_confirm_count >=
                PSU_MODE_MONITOR_CONFIRM_COUNT)
            {
                power_mode_monitor_enter_mode(PSU_MODE_DISPLAY_CC);
            }
        }
        else
        {
            g_mode_monitor_cc_confirm_count = 0U;
        }
        return;
    }

    if (cc_candidate)
    {
        g_mode_monitor_cv_confirm_count = 0U;
        if (g_mode_monitor_cc_confirm_count < PSU_MODE_MONITOR_CONFIRM_COUNT)
        {
            ++g_mode_monitor_cc_confirm_count;
        }
        if (g_mode_monitor_cc_confirm_count >= PSU_MODE_MONITOR_CONFIRM_COUNT)
        {
            power_mode_monitor_enter_mode(PSU_MODE_DISPLAY_CC);
        }
    }
    else if (cv_candidate)
    {
        g_mode_monitor_cc_confirm_count = 0U;
        if (g_mode_monitor_cv_confirm_count < PSU_MODE_MONITOR_CONFIRM_COUNT)
        {
            ++g_mode_monitor_cv_confirm_count;
        }
        if (g_mode_monitor_cv_confirm_count >= PSU_MODE_MONITOR_CONFIRM_COUNT)
        {
            power_mode_monitor_enter_mode(PSU_MODE_DISPLAY_CV);
        }
    }
    else
    {
        g_mode_monitor_cv_confirm_count = 0U;
        g_mode_monitor_cc_confirm_count = 0U;
    }
}

void power_mode_monitor_init(void)
{
    g_mode_monitor_enabled = PSU_MODE_MONITOR_ENABLE ? 1U : 0U;
    g_mode_monitor_feedback_valid = 0U;
    g_mode_monitor_display_mode = PSU_MODE_DISPLAY_OFF;
    g_mode_monitor_test_enable = 0U;
    g_mode_monitor_test_voltage_mv = 0U;
    g_mode_monitor_test_current_ma = 0U;
    g_mode_monitor_using_test_data = 0U;
    g_mode_monitor_voltage_candidate = 0U;
    g_mode_monitor_current_candidate = 0U;
    g_mode_monitor_cv_confirm_count = 0U;
    g_mode_monitor_cc_confirm_count = 0U;
    g_mode_monitor_update_count = 0U;
    g_mode_monitor_cv_enter_count = 0U;
    g_mode_monitor_cc_enter_count = 0U;
    g_mode_monitor_no_feedback_count = 0U;
    g_mode_monitor_reset_count = 0U;
    s_mode_monitor_last_voltage_set_mv = g_power_app.voltage_set_mv;
    s_mode_monitor_last_current_set_ma = g_power_app.current_set_ma;
}

gmp_task_status_t power_mode_monitor_task(gmp_task_t *tsk)
{
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    uint16_t voltage_meas_mv;
    uint16_t current_meas_ma;
    bool feedback_valid;

    GMP_UNUSED_VAR(tsk);

    if (g_mode_monitor_enabled != 1U)
    {
        g_mode_monitor_feedback_valid = 0U;
        g_mode_monitor_using_test_data = 0U;
        power_mode_monitor_set_non_regulating_mode(PSU_MODE_DISPLAY_OFF);
        return GMP_TASK_DONE;
    }

    ++g_mode_monitor_update_count;
    feedback_valid = power_mode_monitor_feedback_valid();
    g_mode_monitor_feedback_valid = feedback_valid ? 1U : 0U;
#if PSU_MODE_MONITOR_TEST_INJECTION
    g_mode_monitor_using_test_data =
        (g_mode_monitor_test_enable == 1U) ? 1U : 0U;
#else
    g_mode_monitor_using_test_data = 0U;
#endif

    if (g_power_app.fault_latched ||
        (g_power_app.fault != POWER_FAULT_NONE) ||
        power_mode_monitor_analog_fault_active())
    {
        power_mode_monitor_set_non_regulating_mode(PSU_MODE_DISPLAY_FAULT);
        return GMP_TASK_DONE;
    }

    if (g_output_switch_precharge_active != 0U)
    {
        power_mode_monitor_set_non_regulating_mode(PSU_MODE_DISPLAY_START);
        return GMP_TASK_DONE;
    }

    if (g_output_switch_active == 0U)
    {
        power_mode_monitor_set_non_regulating_mode(PSU_MODE_DISPLAY_OFF);
        return GMP_TASK_DONE;
    }

    if (!feedback_valid)
    {
        ++g_mode_monitor_no_feedback_count;
        power_mode_monitor_set_non_regulating_mode(
            PSU_MODE_DISPLAY_NO_FEEDBACK);
        return GMP_TASK_DONE;
    }

    voltage_set_mv = g_power_app.voltage_set_mv;
    current_set_ma = g_power_app.current_set_ma;
    if ((voltage_set_mv != s_mode_monitor_last_voltage_set_mv) ||
        (current_set_ma != s_mode_monitor_last_current_set_ma))
    {
        power_mode_monitor_reset_candidates();
        s_mode_monitor_last_voltage_set_mv = voltage_set_mv;
        s_mode_monitor_last_current_set_ma = current_set_ma;
    }

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

    power_mode_monitor_evaluate(voltage_meas_mv,
                                current_meas_ma,
                                voltage_set_mv,
                                current_set_ma);
    return GMP_TASK_DONE;
}

const char *power_mode_monitor_text(psu_mode_display_t mode)
{
    switch (mode)
    {
    case PSU_MODE_DISPLAY_OFF:
        return "OFF";
    case PSU_MODE_DISPLAY_START:
        return "START";
    case PSU_MODE_DISPLAY_NO_FEEDBACK:
        return "--";
    case PSU_MODE_DISPLAY_CV:
        return "CV";
    case PSU_MODE_DISPLAY_CC:
        return "CC";
    case PSU_MODE_DISPLAY_FAULT:
        return "FAULT";
    default:
        return "--";
    }
}

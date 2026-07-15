#include "power_debug.h"

#include "analog_io_test.h"
#include "power_app.h"
#include "power_control_policy.h"
#include "power_mode_monitor.h"
#include "power_self_test.h"

#include <ctrl_settings.h>
#include <gmp_core.h>

#include "user_main.h"

volatile uint16_t g_power_debug_command = POWER_DEBUG_CMD_NONE;
volatile uint16_t g_power_debug_arg0 = 0U;
volatile uint16_t g_power_debug_arg1 = 0U;
volatile uint16_t g_power_debug_command_count = 0U;
volatile uint16_t g_power_debug_ack_count = 0U;

void power_debug_process_command(void)
{
    uint16_t pending_count;
    uint16_t command;
    uint16_t arg0;
    uint16_t arg1;
    uint16_t current;
    uint32_t next;

    pending_count = g_power_debug_command_count;
    if (pending_count == g_power_debug_ack_count)
    {
        return;
    }

    // CCS must update command_count last. Snapshot the single command slot
    // before executing it, then acknowledge the count observed above.
    command = g_power_debug_command;
    arg0 = g_power_debug_arg0;
    arg1 = g_power_debug_arg1;

    switch ((power_debug_command_t)command)
    {
    case POWER_DEBUG_CMD_VOLTAGE_UP:
        current = power_app_get_voltage_mv();
        next = (uint32_t)current + (uint32_t)PSU_VOLTAGE_STEP_MV;
        power_app_set_voltage_mv((next > PSU_COMMAND_VOLTAGE_LIMIT_MV) ?
                                     PSU_COMMAND_VOLTAGE_LIMIT_MV :
                                     (uint16_t)next);
        power_ui_request_led_setpoint_update_from_command();
        break;

    case POWER_DEBUG_CMD_VOLTAGE_DOWN:
        current = power_app_get_voltage_mv();
        power_app_set_voltage_mv((current < PSU_VOLTAGE_STEP_MV) ?
                                     0U : (uint16_t)(current - PSU_VOLTAGE_STEP_MV));
        power_ui_request_led_setpoint_update_from_command();
        break;

    case POWER_DEBUG_CMD_CURRENT_UP:
        current = power_app_get_current_ma();
        next = (uint32_t)current + (uint32_t)PSU_CURRENT_STEP_MA;
        power_app_set_current_ma((next > PSU_COMMAND_CURRENT_LIMIT_MA) ?
                                     PSU_COMMAND_CURRENT_LIMIT_MA :
                                     (uint16_t)next);
        power_ui_request_led_setpoint_update_from_command();
        break;

    case POWER_DEBUG_CMD_CURRENT_DOWN:
        current = power_app_get_current_ma();
        power_app_set_current_ma((current < PSU_CURRENT_STEP_MA) ?
                                     0U : (uint16_t)(current - PSU_CURRENT_STEP_MA));
        power_ui_request_led_setpoint_update_from_command();
        break;

    case POWER_DEBUG_CMD_OUTPUT_TOGGLE:
#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
        power_app_request_logical_output(g_output_switch_requested == 0U);
#elif PSU_SAFE_BRINGUP || !PSU_ALLOW_OUTPUT_REQUEST
        // Exercise the common request barrier without producing repeated logs.
        power_app_request_output(true);
#else
        power_app_request_output(!g_power_app.output_requested);
#endif
        break;

    case POWER_DEBUG_CMD_FAULT_RESET:
        power_app_reset_fault();
        power_control_policy_request_reset();
        break;

    case POWER_DEBUG_CMD_SET_VOLTAGE_MV:
        power_app_set_voltage_mv(arg0);
        power_ui_request_led_setpoint_update_from_command();
        break;

    case POWER_DEBUG_CMD_SET_CURRENT_MA:
        power_app_set_current_ma(arg0);
        power_ui_request_led_setpoint_update_from_command();
        break;

    case POWER_DEBUG_CMD_SET_LOAD_OHM:
        power_self_test_set_load_ohm(arg0);
        break;

    case POWER_DEBUG_CMD_SET_MEASUREMENT_OVERRIDE:
        power_self_test_set_measurement_override(arg0, arg1);
        break;

    case POWER_DEBUG_CMD_CLEAR_MEASUREMENT_OVERRIDE:
        power_self_test_clear_measurement_override();
        break;

    case POWER_DEBUG_CMD_NONE:
    default:
        break;
    }

    g_power_debug_ack_count = pending_count;
}

bool power_debug_safe_bringup_self_test(void)
{
#if PSU_SAFE_BRINGUP
    uint16_t test_count = (uint16_t)(g_power_debug_ack_count + 1U);

    g_power_debug_command = POWER_DEBUG_CMD_OUTPUT_TOGGLE;
    g_power_debug_command_count = test_count;
    power_debug_process_command();
    g_power_debug_command = POWER_DEBUG_CMD_NONE;

    return (g_power_debug_ack_count == test_count) &&
           !g_power_app.output_requested;
#else
    return true;
#endif
}

void power_debug_print_status(void)
{
    power_state_t state;
    power_fault_t fault;
    uint16_t voltage_set_mv;
    uint16_t voltage_meas_mv;
    uint16_t current_set_ma;
    uint16_t current_meas_ma;
    uint16_t dac_voltage_code;
    uint16_t dac_current_code;
    const char *hardware_fault_text;
    const char *actual_mode_text;

    // Take a field-by-field snapshot of the ISR-owned volatile application
    // state so formatting happens only in this background task.
    state = g_power_app.state;
    fault = g_power_app.fault;
    voltage_set_mv = g_power_app.voltage_set_mv;
    current_set_ma = g_power_app.current_set_ma;
    if ((g_mode_monitor_feedback_valid != 0U) &&
        (g_mode_monitor_using_test_data != 0U))
    {
        voltage_meas_mv = g_mode_monitor_test_voltage_mv;
        current_meas_ma = g_mode_monitor_test_current_ma;
    }
    else
    {
        voltage_meas_mv = g_power_app.voltage_meas_mv;
        current_meas_ma = g_power_app.current_meas_ma;
    }
    dac_voltage_code = g_power_app.dac_voltage_code;
    dac_current_code = g_power_app.dac_current_code;

    if (fault == POWER_FAULT_OVERVOLTAGE)
    {
        hardware_fault_text = "OVP";
    }
    else if (fault == POWER_FAULT_OVERCURRENT)
    {
        hardware_fault_text = "OCP";
    }
    else if ((g_analog_board_feedback_fault != 0U) ||
             (g_analog_board_fault_hold_active != 0U) ||
             (g_analog_board_fault_shutdown_active != 0U))
    {
        hardware_fault_text = "ADC";
    }
    else
    {
        hardware_fault_text = "NONE";
    }

    if (g_mode_monitor_display_mode == (uint16_t)PSU_MODE_DISPLAY_CV)
    {
        actual_mode_text = "CV";
    }
    else if (g_mode_monitor_display_mode == (uint16_t)PSU_MODE_DISPLAY_CC)
    {
        actual_mode_text = "CC";
    }
    else
    {
        actual_mode_text = "--";
    }

    gmp_base_print(
        "PWR S=%u F=%u VSET=%u ISET=%u VM=%u IM=%u OUT=%u HW_FAULT=%s DAC=%u/%u\r\n",
        (unsigned int)state,
        (unsigned int)fault,
        (unsigned int)voltage_set_mv,
        (unsigned int)current_set_ma,
        (unsigned int)voltage_meas_mv,
        (unsigned int)current_meas_ma,
        (unsigned int)g_output_switch_active,
        hardware_fault_text,
        (unsigned int)dac_voltage_code,
        (unsigned int)dac_current_code);
    gmp_base_print(
        "PWR CTRL=%s ACT=%s POLICY_FAULT=%s POLICY_LATCH=%u MISMATCH_COUNT=%u\r\n",
        power_control_policy_strategy_text(
            power_control_policy_get_strategy()),
        actual_mode_text,
        power_control_policy_fault_text(
            (psu_policy_fault_t)g_control_policy_fault),
        (unsigned int)g_control_policy_fault_latched,
        (unsigned int)g_control_policy_mismatch_confirm_count);
}

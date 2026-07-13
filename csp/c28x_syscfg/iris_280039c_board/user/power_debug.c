#include "power_debug.h"

#include "power_app.h"
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
        power_app_set_voltage_mv((next > PSU_VOLTAGE_CMD_MAX_MV) ?
                                     PSU_VOLTAGE_CMD_MAX_MV : (uint16_t)next);
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
        power_app_set_current_ma((next > PSU_CURRENT_CMD_MAX_MA) ?
                                     PSU_CURRENT_CMD_MAX_MA : (uint16_t)next);
        power_ui_request_led_setpoint_update_from_command();
        break;

    case POWER_DEBUG_CMD_CURRENT_DOWN:
        current = power_app_get_current_ma();
        power_app_set_current_ma((current < PSU_CURRENT_STEP_MA) ?
                                     0U : (uint16_t)(current - PSU_CURRENT_STEP_MA));
        power_ui_request_led_setpoint_update_from_command();
        break;

    case POWER_DEBUG_CMD_OUTPUT_TOGGLE:
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_OUTPUT_REQUEST
        // Exercise the common request barrier without producing repeated logs.
        power_app_request_output(true);
#else
        power_app_request_output(!g_power_app.output_requested);
#endif
        break;

    case POWER_DEBUG_CMD_FAULT_RESET:
        power_app_reset_fault();
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

    // Take a field-by-field snapshot of the ISR-owned volatile application
    // state so formatting happens only in this background task.
    state = g_power_app.state;
    fault = g_power_app.fault;
    voltage_set_mv = g_power_app.voltage_set_mv;
    voltage_meas_mv = g_power_app.voltage_meas_mv;
    current_set_ma = g_power_app.current_set_ma;
    current_meas_ma = g_power_app.current_meas_ma;

    gmp_base_print(
        "PWR S=%u F=%u V=%u/%u I=%u/%u\r\n",
        (unsigned int)state,
        (unsigned int)fault,
        (unsigned int)voltage_set_mv,
        (unsigned int)voltage_meas_mv,
        (unsigned int)current_set_ma,
        (unsigned int)current_meas_ma);
}

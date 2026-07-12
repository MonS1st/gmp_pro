#include "power_debug.h"

#include "power_app.h"
#include "power_self_test.h"

#include <ctrl_settings.h>
#include <gmp_core.h>

volatile uint16_t g_power_debug_command = POWER_DEBUG_CMD_NONE;
volatile uint16_t g_power_debug_arg0 = 0U;
volatile uint16_t g_power_debug_arg1 = 0U;
volatile uint16_t g_power_debug_command_count = 0U;
volatile uint16_t g_power_debug_ack_count = 0U;

static const char *power_debug_state_text(power_state_t state)
{
    switch (state)
    {
    case POWER_STATE_OFF:
        return "OFF";
    case POWER_STATE_STARTING:
        return "START";
    case POWER_STATE_CV:
        return "CV";
    case POWER_STATE_CC:
        return "CC";
    case POWER_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static const char *power_debug_fault_text(power_fault_t fault)
{
    switch (fault)
    {
    case POWER_FAULT_NONE:
        return "NONE";
    case POWER_FAULT_OVERVOLTAGE:
        return "OVP";
    case POWER_FAULT_OVERCURRENT:
        return "OCP";
    default:
        return "UNKNOWN";
    }
}

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
        break;

    case POWER_DEBUG_CMD_VOLTAGE_DOWN:
        current = power_app_get_voltage_mv();
        power_app_set_voltage_mv((current < PSU_VOLTAGE_STEP_MV) ?
                                     0U : (uint16_t)(current - PSU_VOLTAGE_STEP_MV));
        break;

    case POWER_DEBUG_CMD_CURRENT_UP:
        current = power_app_get_current_ma();
        next = (uint32_t)current + (uint32_t)PSU_CURRENT_STEP_MA;
        power_app_set_current_ma((next > PSU_CURRENT_CMD_MAX_MA) ?
                                     PSU_CURRENT_CMD_MAX_MA : (uint16_t)next);
        break;

    case POWER_DEBUG_CMD_CURRENT_DOWN:
        current = power_app_get_current_ma();
        power_app_set_current_ma((current < PSU_CURRENT_STEP_MA) ?
                                     0U : (uint16_t)(current - PSU_CURRENT_STEP_MA));
        break;

    case POWER_DEBUG_CMD_OUTPUT_TOGGLE:
        power_app_request_output(!g_power_app.output_requested);
        break;

    case POWER_DEBUG_CMD_FAULT_RESET:
        power_app_reset_fault();
        break;

    case POWER_DEBUG_CMD_SET_VOLTAGE_MV:
        power_app_set_voltage_mv(arg0);
        break;

    case POWER_DEBUG_CMD_SET_CURRENT_MA:
        power_app_set_current_ma(arg0);
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

void power_debug_print_status(void)
{
    power_app_t app;
    uint16_t load_ohm;

    // Take a field-by-field snapshot of the ISR-owned volatile application
    // state so formatting happens only in this background task.
    app.voltage_set_mv = g_power_app.voltage_set_mv;
    app.current_set_ma = g_power_app.current_set_ma;
    app.voltage_meas_mv = g_power_app.voltage_meas_mv;
    app.current_meas_ma = g_power_app.current_meas_ma;
    app.dac_voltage_code = g_power_app.dac_voltage_code;
    app.dac_current_code = g_power_app.dac_current_code;
    app.state = g_power_app.state;
    app.fault = g_power_app.fault;
    app.output_requested = g_power_app.output_requested;
    app.output_enabled = g_power_app.output_enabled;
    app.fault_latched = g_power_app.fault_latched;
    load_ohm = power_self_test_get_load_ohm();

    gmp_base_print("PWR state=%s fault=%s latch=%u req=%u en=%u Vset=%u Vmeas=%u Iset=%u Imeas=%u DACV=%u DACI=%u load=%u\r\n",
                   power_debug_state_text(app.state),
                   power_debug_fault_text(app.fault),
                   (unsigned int)app.fault_latched,
                   (unsigned int)app.output_requested,
                   (unsigned int)app.output_enabled,
                   (unsigned int)app.voltage_set_mv,
                   (unsigned int)app.voltage_meas_mv,
                   (unsigned int)app.current_set_ma,
                   (unsigned int)app.current_meas_ma,
                   (unsigned int)app.dac_voltage_code,
                   (unsigned int)app.dac_current_code,
                   (unsigned int)load_ohm);
}

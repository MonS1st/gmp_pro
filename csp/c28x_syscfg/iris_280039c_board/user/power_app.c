#include "power_app.h"

#include "power_hal.h"
#include "power_self_test.h"

#include <ctrl_settings.h>
#if !PSU_SOFT_TEST_MODE
#include <gmp_core.h>
#include <xplt.peripheral.h>
#endif

volatile power_app_t g_power_app;

#if !PSU_SOFT_TEST_MODE
static uint16_t power_float_to_u16(float value)
{
    if (value <= 0.0f)
    {
        return 0U;
    }
    if (value >= 65535.0f)
    {
        return 65535U;
    }
    return (uint16_t)(value + 0.5f);
}
#endif

static void power_app_limit_commands(void)
{
    if (g_power_app.voltage_set_mv > PSU_VOLTAGE_CMD_MAX_MV)
    {
        g_power_app.voltage_set_mv = PSU_VOLTAGE_CMD_MAX_MV;
    }
    if (g_power_app.current_set_ma > PSU_CURRENT_CMD_MAX_MA)
    {
        g_power_app.current_set_ma = PSU_CURRENT_CMD_MAX_MA;
    }
}

static void power_app_set_output_off(power_state_t state)
{
    power_dac_set_zero();
    power_output_hw_set(false);
    g_power_app.dac_voltage_code = 0U;
    g_power_app.dac_current_code = 0U;
    g_power_app.output_enabled = false;
    g_power_app.state = state;
#if PSU_SOFT_TEST_MODE
    g_power_app.voltage_meas_mv = 0U;
    g_power_app.current_meas_ma = 0U;
#endif
}

static void power_app_apply_commands(void)
{
    g_power_app.dac_voltage_code = power_voltage_mv_to_dac(g_power_app.voltage_set_mv);
    g_power_app.dac_current_code = power_current_ma_to_dac(g_power_app.current_set_ma);
    power_dac_set_voltage_mv(g_power_app.voltage_set_mv);
    power_dac_set_current_ma(g_power_app.current_set_ma);
}

void power_app_init(void)
{
    g_power_app.voltage_set_mv = 0U;
    g_power_app.current_set_ma = 0U;
    g_power_app.voltage_meas_mv = 0U;
    g_power_app.current_meas_ma = 0U;
    g_power_app.dac_voltage_code = 0U;
    g_power_app.dac_current_code = 0U;
    g_power_app.state = POWER_STATE_OFF;
    g_power_app.fault = POWER_FAULT_NONE;
    g_power_app.output_requested = false;
    g_power_app.output_enabled = false;
    g_power_app.fault_latched = false;

    power_dac_set_zero();
    power_output_hw_set(false);
}

void power_app_request_output(bool enable)
{
    g_power_app.output_requested = enable;
}

void power_app_reset_fault(void)
{
    if (!g_power_app.output_requested)
    {
        g_power_app.fault = POWER_FAULT_NONE;
        g_power_app.fault_latched = false;
        power_app_set_output_off(POWER_STATE_OFF);
    }
}

void power_app_fast_step(void)
{
    power_app_limit_commands();

#if !PSU_SOFT_TEST_MODE
    g_power_app.voltage_meas_mv = power_float_to_u16(g_vout_meas_v * 1000.0f);
    g_power_app.current_meas_ma = power_float_to_u16(g_iout_meas_ma);
#endif

    if (g_power_app.fault_latched || (g_power_app.fault != POWER_FAULT_NONE))
    {
        g_power_app.fault_latched = true;
        power_app_set_output_off(POWER_STATE_FAULT);
        return;
    }

    if (!g_power_app.output_requested)
    {
        power_app_set_output_off(POWER_STATE_OFF);
        return;
    }

    switch (g_power_app.state)
    {
    case POWER_STATE_OFF:
        power_app_set_output_off(POWER_STATE_STARTING);
        break;

    case POWER_STATE_STARTING:
        power_app_apply_commands();
        power_output_hw_set(true);
        g_power_app.output_enabled = power_output_hw_get();
        if (g_power_app.output_enabled)
        {
            g_power_app.state = POWER_STATE_CV;
        }
        break;

    case POWER_STATE_CV:
    case POWER_STATE_CC:
        power_app_apply_commands();
        power_output_hw_set(true);
        g_power_app.output_enabled = power_output_hw_get();
        break;

    case POWER_STATE_FAULT:
    default:
        power_app_set_output_off(POWER_STATE_FAULT);
        break;
    }
}

void power_app_slow_step(void)
{
#if PSU_SOFT_TEST_MODE
    power_self_test_step();
#endif
}

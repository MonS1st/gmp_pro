#include "power_self_test.h"

#include "power_app.h"

#include <stdint.h>

volatile float g_virtual_load_ohm = 200.0f;

static uint16_t power_self_test_to_u16(float value)
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

void power_self_test_step(void)
{
    float load_current_ma;

    if (!g_power_app.output_enabled)
    {
        g_power_app.voltage_meas_mv = 0U;
        g_power_app.current_meas_ma = 0U;
        if (!g_power_app.output_requested && !g_power_app.fault_latched)
        {
            g_power_app.state = POWER_STATE_OFF;
        }
        return;
    }

    if (g_virtual_load_ohm <= 0.0f)
    {
        g_power_app.voltage_meas_mv = 0U;
        g_power_app.current_meas_ma = g_power_app.current_set_ma;
        g_power_app.state = POWER_STATE_CC;
        return;
    }

    // mV / ohm = mA for the ideal resistive-load model.
    load_current_ma = (float)g_power_app.voltage_set_mv / g_virtual_load_ohm;

    if (load_current_ma <= (float)g_power_app.current_set_ma)
    {
        g_power_app.voltage_meas_mv = g_power_app.voltage_set_mv;
        g_power_app.current_meas_ma = power_self_test_to_u16(load_current_ma);
        g_power_app.state = POWER_STATE_CV;
    }
    else
    {
        g_power_app.current_meas_ma = g_power_app.current_set_ma;
        g_power_app.voltage_meas_mv = power_self_test_to_u16((float)g_power_app.current_set_ma *
                                                             g_virtual_load_ohm);
        g_power_app.state = POWER_STATE_CC;
    }
}

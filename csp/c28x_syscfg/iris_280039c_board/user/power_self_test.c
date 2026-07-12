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
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    bool output_enabled;
    float load_ohm;
    float load_current_ma;

    output_enabled = g_power_app.output_enabled;
    voltage_set_mv = g_power_app.voltage_set_mv;
    current_set_ma = g_power_app.current_set_ma;
    load_ohm = g_virtual_load_ohm;

    if (!output_enabled)
    {
        g_power_app.voltage_meas_mv = 0U;
        g_power_app.current_meas_ma = 0U;
        return;
    }

    if (load_ohm <= 0.0f)
    {
        g_power_app.voltage_meas_mv = 0U;
        g_power_app.current_meas_ma = current_set_ma;
        return;
    }

    // mV / ohm = mA for the ideal resistive-load model.
    load_current_ma = (float)voltage_set_mv / load_ohm;

    if (load_current_ma <= (float)current_set_ma)
    {
        g_power_app.voltage_meas_mv = voltage_set_mv;
        g_power_app.current_meas_ma = power_self_test_to_u16(load_current_ma);
    }
    else
    {
        g_power_app.current_meas_ma = current_set_ma;
        g_power_app.voltage_meas_mv = power_self_test_to_u16((float)current_set_ma * load_ohm);
    }
}

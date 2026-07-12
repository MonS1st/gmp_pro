#include "power_self_test.h"

#include "power_app.h"

#include <stddef.h>
#include <stdint.h>

volatile float g_virtual_load_ohm = 200.0f;
volatile uint16_t g_virtual_voltage_mv = 0U;
volatile uint16_t g_virtual_current_ma = 0U;
volatile uint16_t g_virtual_measurement_seq = 0U;
volatile bool g_virtual_measurement_override_enable = false;
volatile uint16_t g_virtual_override_voltage_mv = 0U;
volatile uint16_t g_virtual_override_current_ma = 0U;

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
    uint16_t next_voltage_mv;
    uint16_t next_current_ma;
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
        next_voltage_mv = 0U;
        next_current_ma = 0U;
    }
    else if (load_ohm <= 0.0f)
    {
        next_voltage_mv = 0U;
        next_current_ma = current_set_ma;
    }
    else
    {
        // mV / ohm = mA for the ideal resistive-load model.
        load_current_ma = (float)voltage_set_mv / load_ohm;

        if (load_current_ma <= (float)current_set_ma)
        {
            next_voltage_mv = voltage_set_mv;
            next_current_ma = power_self_test_to_u16(load_current_ma);
        }
        else
        {
            next_current_ma = current_set_ma;
            next_voltage_mv = power_self_test_to_u16((float)current_set_ma * load_ohm);
        }
    }

    // The override remains active with the output disabled so unsafe FAULT
    // release conditions can be exercised without hardware.
    if (g_virtual_measurement_override_enable)
    {
        next_voltage_mv = g_virtual_override_voltage_mv;
        next_current_ma = g_virtual_override_current_ma;
    }

    // Odd means an update is in progress; the following even value publishes the pair.
    ++g_virtual_measurement_seq;
    g_virtual_voltage_mv = next_voltage_mv;
    g_virtual_current_ma = next_current_ma;
    ++g_virtual_measurement_seq;
}

bool power_self_test_get_measurement(uint16_t *voltage_mv, uint16_t *current_ma)
{
    uint16_t seq_before;
    uint16_t seq_after;
    uint16_t local_voltage_mv;
    uint16_t local_current_ma;

    if ((voltage_mv == NULL) || (current_ma == NULL))
    {
        return false;
    }

    seq_before = g_virtual_measurement_seq;
    if ((seq_before & 1U) != 0U)
    {
        return false;
    }

    local_voltage_mv = g_virtual_voltage_mv;
    local_current_ma = g_virtual_current_ma;
    seq_after = g_virtual_measurement_seq;

    if ((seq_before != seq_after) || ((seq_after & 1U) != 0U))
    {
        return false;
    }

    *voltage_mv = local_voltage_mv;
    *current_ma = local_current_ma;
    return true;
}

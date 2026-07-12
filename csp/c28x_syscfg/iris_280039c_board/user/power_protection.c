#include "power_protection.h"

#include <ctrl_settings.h>

volatile power_protection_t g_power_protection;

void power_protection_init(void)
{
    g_power_protection.ovp_confirm_count = 0U;
    g_power_protection.ocp_confirm_count = 0U;
    g_power_protection.last_voltage_mv = 0U;
    g_power_protection.last_current_ma = 0U;
}

void power_protection_reset(void)
{
    g_power_protection.ovp_confirm_count = 0U;
    g_power_protection.ocp_confirm_count = 0U;
}

power_protection_result_t power_protection_step(uint16_t voltage_mv,
                                                uint16_t current_ma,
                                                bool enabled)
{
    g_power_protection.last_voltage_mv = voltage_mv;
    g_power_protection.last_current_ma = current_ma;

    if (!enabled)
    {
        power_protection_reset();
        return POWER_PROTECTION_RESULT_NONE;
    }

    if (voltage_mv >= PSU_OVP_TRIP_MV)
    {
        if (g_power_protection.ovp_confirm_count < PSU_OVP_CONFIRM_COUNT)
        {
            ++g_power_protection.ovp_confirm_count;
        }
    }
    else
    {
        g_power_protection.ovp_confirm_count = 0U;
    }

    if (current_ma >= PSU_OCP_TRIP_MA)
    {
        if (g_power_protection.ocp_confirm_count < PSU_OCP_CONFIRM_COUNT)
        {
            ++g_power_protection.ocp_confirm_count;
        }
    }
    else
    {
        g_power_protection.ocp_confirm_count = 0U;
    }

    // OVP has priority if both protection counters confirm on the same cycle.
    if (g_power_protection.ovp_confirm_count >= PSU_OVP_CONFIRM_COUNT)
    {
        return POWER_PROTECTION_RESULT_OVERVOLTAGE;
    }
    if (g_power_protection.ocp_confirm_count >= PSU_OCP_CONFIRM_COUNT)
    {
        return POWER_PROTECTION_RESULT_OVERCURRENT;
    }

    return POWER_PROTECTION_RESULT_NONE;
}

bool power_protection_release_safe(uint16_t voltage_mv, uint16_t current_ma)
{
    return (voltage_mv <= PSU_OVP_RELEASE_MV) &&
           (current_ma <= PSU_OCP_RELEASE_MA);
}

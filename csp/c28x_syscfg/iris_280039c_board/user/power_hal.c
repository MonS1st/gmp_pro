#include "power_hal.h"

#include <ctrl_settings.h>
#include <driverlib.h>
#include <board.h>

static volatile bool s_power_output_hw_enabled = false;

static uint16_t power_limit_dac_code(uint32_t code)
{
    return (code > PSU_DAC_MAX_CODE) ? PSU_DAC_MAX_CODE : (uint16_t)code;
}

uint16_t power_voltage_mv_to_dac(uint16_t voltage_mv)
{
    uint32_t limited_mv = voltage_mv;
    uint32_t code;

    if (limited_mv > PSU_VOLTAGE_CMD_MAX_MV)
    {
        limited_mv = PSU_VOLTAGE_CMD_MAX_MV;
    }

    // Vset = Vout_cmd / 4; DAC code = Vset / 3300 mV * 4096.
    code = limited_mv * PSU_DAC_CODE_SCALE / (PSU_VSET_DIVIDER * PSU_DAC_VREF_MV);
    return power_limit_dac_code(code);
}

uint16_t power_current_ma_to_dac(uint16_t current_ma)
{
    uint32_t limited_ma = current_ma;
    uint32_t code;

    if (limited_ma > PSU_CURRENT_CMD_MAX_MA)
    {
        limited_ma = PSU_CURRENT_CMD_MAX_MA;
    }

    // V_ISET(mV) = 20 * I_cmd(mA); DAC code = V_ISET / 3300 mV * 4096.
    code = limited_ma * PSU_ISET_GAIN_MV_PER_MA * PSU_DAC_CODE_SCALE / PSU_DAC_VREF_MV;
    return power_limit_dac_code(code);
}

void power_dac_set_voltage_mv(uint16_t voltage_mv)
{
    DAC_setShadowValue(IRIS_DACA_BASE, power_voltage_mv_to_dac(voltage_mv));
}

void power_dac_set_current_ma(uint16_t current_ma)
{
    DAC_setShadowValue(IRIS_DACB_BASE, power_current_ma_to_dac(current_ma));
}

void power_dac_set_zero(void)
{
    DAC_setShadowValue(IRIS_DACA_BASE, 0U);
    DAC_setShadowValue(IRIS_DACB_BASE, 0U);
}

void power_output_hw_set(bool enable)
{
    // TODO: Bind this software state to the confirmed relay or MOSFET enable pin.
    // No GPIO is touched until the hardware output-enable connection is known.
    s_power_output_hw_enabled = enable;
}

bool power_output_hw_get(void)
{
    return s_power_output_hw_enabled;
}

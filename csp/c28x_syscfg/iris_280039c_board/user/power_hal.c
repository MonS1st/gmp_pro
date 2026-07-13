#include "power_hal.h"

#include <ctrl_settings.h>
#include <driverlib.h>
#include <board.h>

static volatile bool s_power_output_hw_enabled = false;

#if !PSU_ENABLE_ANALOG_IO_TEST
static void power_dac_force_physical_zero(void)
{
    DAC_setShadowValue(IRIS_DACA_BASE, 0U);
    DAC_setShadowValue(IRIS_DACB_BASE, 0U);
}
#endif

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
#if PSU_ENABLE_ANALOG_IO_TEST
    // The isolated calibration task is the only runtime owner of real DACs.
    (void)voltage_mv;
#elif PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_DAC || PSU_SOFT_TEST_MODE
    // SOFT_TEST does not automatically isolate the physical hardware. Force
    // both real DAC targets to zero explicitly on every command path.
    (void)voltage_mv;
    power_dac_force_physical_zero();
#else
    DAC_setShadowValue(IRIS_DACA_BASE, power_voltage_mv_to_dac(voltage_mv));
#endif
}

void power_dac_set_current_ma(uint16_t current_ma)
{
#if PSU_ENABLE_ANALOG_IO_TEST
    // The isolated calibration task is the only runtime owner of real DACs.
    (void)current_ma;
#elif PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_DAC || PSU_SOFT_TEST_MODE
    // SOFT_TEST does not automatically isolate the physical hardware. Force
    // both real DAC targets to zero explicitly on every command path.
    (void)current_ma;
    power_dac_force_physical_zero();
#else
    DAC_setShadowValue(IRIS_DACB_BASE, power_current_ma_to_dac(current_ma));
#endif
}

void power_dac_set_zero(void)
{
#if PSU_ENABLE_ANALOG_IO_TEST
    // Zeroing is owned by analog_io_test so power_app cannot overwrite a
    // manually armed calibration code from the 20 kHz control callback.
    return;
#else
    power_dac_force_physical_zero();
#endif
}

void power_output_hw_set(bool enable)
{
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE
    // The relay, MOSFET, and gate-enable pins are unconfirmed. Do not touch a
    // GPIO in safe bringup, regardless of the requested software state.
    (void)enable;
    s_power_output_hw_enabled = false;
#else
    // TODO: Bind this software state to the confirmed relay or MOSFET enable pin.
    // No GPIO is touched until the hardware output-enable connection is known.
    s_power_output_hw_enabled = enable;
#endif
}

bool power_output_hw_get(void)
{
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE
    s_power_output_hw_enabled = false;
    return false;
#else
    return s_power_output_hw_enabled;
#endif
}

bool power_hal_safe_bringup_self_test(void)
{
#if PSU_SAFE_BRINGUP
    power_output_hw_set(true);
    power_dac_set_voltage_mv(PSU_VOLTAGE_CMD_MAX_MV);
    power_dac_set_current_ma(PSU_CURRENT_CMD_MAX_MA);

    return !power_output_hw_get() &&
           (DAC_getShadowValue(IRIS_DACA_BASE) == 0U) &&
           (DAC_getShadowValue(IRIS_DACB_BASE) == 0U);
#else
    return true;
#endif
}

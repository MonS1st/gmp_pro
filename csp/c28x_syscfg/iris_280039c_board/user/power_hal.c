#include "power_hal.h"

#include <gmp_core.h>
#include <xplt.peripheral.h>
#include <ctrl_settings.h>
#include <driverlib.h>
#include <board.h>

static volatile bool s_power_output_hw_enabled = false;
static volatile bool s_power_output_hw_requested = false;
static volatile bool s_power_output_hw_update_pending = true;

static uint16_t s_fpga_r1_shadow = 0U;
static bool s_fpga_r1_shadow_valid = false;

#define POWER_FPGA_R1_ADDR               (0x01U)
#define POWER_FPGA_R1_RELAY_ALLOW_MASK   (0x0010U)

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

    // V_ISET(mV) = 22 * I_cmd(mA); DAC code = V_ISET / 3300 mV * 4096.
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

bool power_fpga_r1_write_masked(uint16_t mask, uint16_t value)
{
    uint16_t updated_value;

    if (!s_fpga_r1_shadow_valid)
    {
        s_fpga_r1_shadow = SPI_readReg(POWER_FPGA_R1_ADDR);
        s_fpga_r1_shadow_valid = true;
    }

    updated_value = (s_fpga_r1_shadow & (uint16_t)(~mask)) |
                    (value & mask);
    if (updated_value == s_fpga_r1_shadow)
    {
        return true;
    }

    SPI_writeReg(POWER_FPGA_R1_ADDR, updated_value);
    s_fpga_r1_shadow = SPI_readReg(POWER_FPGA_R1_ADDR);

    return s_fpga_r1_shadow == updated_value;
}

void power_output_hw_set(bool enable)
{
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE || \
    !PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE
    (void)enable;
    s_power_output_hw_requested = false;
    s_power_output_hw_enabled = false;
#else
    if (enable != s_power_output_hw_requested)
    {
        s_power_output_hw_requested = enable;
        s_power_output_hw_update_pending = true;
    }

    if (!enable)
    {
        s_power_output_hw_enabled = false;
    }
#endif
}

void power_output_hw_service(void)
{
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE || \
    !PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE
    s_power_output_hw_requested = false;
    s_power_output_hw_enabled = false;
    s_power_output_hw_update_pending = false;
#else
    bool requested;
    bool update_ok;

    if (!s_power_output_hw_update_pending)
    {
        return;
    }

    requested = s_power_output_hw_requested;
    s_power_output_hw_update_pending = false;
    update_ok = power_fpga_r1_write_masked(
        POWER_FPGA_R1_RELAY_ALLOW_MASK,
        requested ? POWER_FPGA_R1_RELAY_ALLOW_MASK : 0U);

    // A fast-loop fault may revoke the request while the background SPI
    // transaction is in progress. Never report enabled, and restore cutoff.
    if (requested && (!update_ok || !s_power_output_hw_requested))
    {
        (void)power_fpga_r1_write_masked(
            POWER_FPGA_R1_RELAY_ALLOW_MASK, 0U);
        s_power_output_hw_requested = false;
        s_power_output_hw_update_pending = false;
        s_power_output_hw_enabled = false;
        return;
    }

    s_power_output_hw_enabled = requested && update_ok;
#endif
}

bool power_output_hw_get(void)
{
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE || \
    !PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE
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

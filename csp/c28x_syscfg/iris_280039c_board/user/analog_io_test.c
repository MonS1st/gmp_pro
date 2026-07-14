#include "analog_io_test.h"

#include "power_app.h"
#include "power_hal.h"

#include <board.h>
#include <ctrl_settings.h>
#include <driverlib.h>
#include <xplt.peripheral.h>

#define PSU_ANALOG_IO_TEST_ADC_FULL_SCALE_CODE (4095.0f)

volatile uint16_t g_adc_test_vout_raw = 0U;
volatile uint16_t g_adc_test_iout_raw = 0U;
volatile float g_adc_test_vout_pin_v = 0.0f;
volatile float g_adc_test_iout_pin_v = 0.0f;
volatile float g_adc_test_vout_value_v = 0.0f;
volatile float g_adc_test_iout_value_ma = 0.0f;
volatile uint16_t g_adc_test_enabled = 0U;
volatile uint32_t g_adc_test_sample_count = 0U;

volatile uint16_t g_dac_test_arm = 0U;
volatile uint16_t g_dac_test_enable = 0U;
volatile uint16_t g_dac_test_voltage_code = 0U;
volatile uint16_t g_dac_test_current_code = 0U;
volatile uint16_t g_dac_test_voltage_applied_code = 0U;
volatile uint16_t g_dac_test_current_applied_code = 0U;
volatile uint16_t g_dac_test_command = PSU_DAC_TEST_COMMAND_NONE;
volatile uint32_t g_dac_test_apply_count = 0U;
volatile uint32_t g_dac_test_reject_count = 0U;
volatile float g_dac_test_voltage_expected_v = 0.0f;
volatile float g_dac_test_current_expected_v = 0.0f;
volatile uint16_t g_dac_test_follow_ui_enable = 0U;
volatile uint16_t g_dac_test_follow_ui_active = 0U;
volatile uint16_t g_dac_test_last_voltage_mv = 0U;
volatile uint16_t g_dac_test_last_current_ma = 0U;
volatile uint32_t g_dac_test_follow_update_count = 0U;

static uint16_t s_dac_test_outputs_zeroed = 0U;
static uint16_t s_dac_test_follow_ui_was_active = 0U;

static float analog_io_test_dac_expected_v(uint16_t code)
{
    return ((float)code * PSU_ADC_VREF_V) /
           PSU_ANALOG_IO_TEST_ADC_FULL_SCALE_CODE;
}

static uint16_t analog_io_test_limit_code(uint16_t code, uint16_t limit)
{
    return (code > limit) ? limit : code;
}

static void analog_io_test_force_dac_zero(void)
{
    if ((s_dac_test_outputs_zeroed == 0U) ||
        (g_dac_test_voltage_applied_code != 0U) ||
        (g_dac_test_current_applied_code != 0U) ||
        (DAC_getShadowValue(IRIS_DACA_BASE) != 0U) ||
        (DAC_getShadowValue(IRIS_DACB_BASE) != 0U))
    {
        DAC_setShadowValue(IRIS_DACA_BASE, 0U);
        DAC_setShadowValue(IRIS_DACB_BASE, 0U);
    }

    g_dac_test_voltage_applied_code = 0U;
    g_dac_test_current_applied_code = 0U;
    g_dac_test_voltage_expected_v = 0.0f;
    g_dac_test_current_expected_v = 0.0f;
    g_dac_test_follow_ui_active = 0U;
    s_dac_test_outputs_zeroed = 1U;
    s_dac_test_follow_ui_was_active = 0U;
}

static void analog_io_test_copy_adc_diagnostics(void)
{
    uint16_t vout_raw = g_adc_vout_raw;
    uint16_t iout_raw = g_adc_iout_raw;
    float vout_pin_v;
    float iout_pin_v;

    vout_pin_v = ((float)vout_raw * PSU_ADC_VREF_V) /
                 PSU_ANALOG_IO_TEST_ADC_FULL_SCALE_CODE;
    iout_pin_v = ((float)iout_raw * PSU_ADC_VREF_V) /
                 PSU_ANALOG_IO_TEST_ADC_FULL_SCALE_CODE;

    g_adc_test_vout_raw = vout_raw;
    g_adc_test_iout_raw = iout_raw;
    g_adc_test_vout_pin_v = vout_pin_v;
    g_adc_test_iout_pin_v = iout_pin_v;
    g_adc_test_vout_value_v = vout_pin_v / PSU_VOUT_SENSOR_GAIN;
    g_adc_test_iout_value_ma =
        (iout_pin_v / PSU_IOUT_SENSOR_GAIN_V_PER_A) * 1000.0f;
    ++g_adc_test_sample_count;
}

static void analog_io_test_reject_invalid_state(void)
{
    ++g_dac_test_reject_count;
    g_dac_test_arm = 0U;
    g_dac_test_enable = 0U;
    g_dac_test_follow_ui_enable = 0U;
    g_dac_test_command = PSU_DAC_TEST_COMMAND_NONE;
    analog_io_test_force_dac_zero();
}

static void analog_io_test_process_follow_ui(void)
{
    uint16_t voltage_mv = power_app_get_voltage_mv();
    uint16_t current_ma = power_app_get_current_ma();
    uint16_t voltage_code;
    uint16_t current_code;

    // Synchronize once on mode entry, then write only when a UI setpoint changes.
    if ((s_dac_test_follow_ui_was_active == 0U) ||
        (voltage_mv != g_dac_test_last_voltage_mv) ||
        (current_ma != g_dac_test_last_current_ma))
    {
        voltage_code = power_voltage_mv_to_dac(voltage_mv);
        current_code = power_current_ma_to_dac(current_ma);

        DAC_setShadowValue(IRIS_DACA_BASE, voltage_code);
        DAC_setShadowValue(IRIS_DACB_BASE, current_code);

        g_dac_test_voltage_applied_code = voltage_code;
        g_dac_test_current_applied_code = current_code;
        g_dac_test_voltage_expected_v =
            analog_io_test_dac_expected_v(voltage_code);
        g_dac_test_current_expected_v =
            analog_io_test_dac_expected_v(current_code);
        g_dac_test_last_voltage_mv = voltage_mv;
        g_dac_test_last_current_ma = current_ma;
        s_dac_test_outputs_zeroed =
            ((voltage_code == 0U) && (current_code == 0U)) ? 1U : 0U;
        ++g_dac_test_follow_update_count;
    }

    s_dac_test_follow_ui_was_active = 1U;
    g_dac_test_follow_ui_active = 1U;
}

static void analog_io_test_process_dac_command(void)
{
    uint16_t command = g_dac_test_command;
    uint16_t voltage_code;
    uint16_t current_code;

    if (command > PSU_DAC_TEST_COMMAND_CLEAR_AND_DISARM)
    {
        analog_io_test_reject_invalid_state();
        return;
    }

    if (command == PSU_DAC_TEST_COMMAND_CLEAR_AND_DISARM)
    {
        analog_io_test_force_dac_zero();
        g_dac_test_arm = 0U;
        g_dac_test_follow_ui_enable = 0U;
        g_dac_test_follow_ui_active = 0U;
        g_dac_test_command = PSU_DAC_TEST_COMMAND_NONE;
        ++g_dac_test_apply_count;
        return;
    }

    if ((g_dac_test_enable > 1U) ||
        (g_dac_test_follow_ui_enable > 1U) ||
        ((g_dac_test_arm != 0U) &&
         (g_dac_test_arm != PSU_DAC_TEST_ARM_KEY)))
    {
        analog_io_test_reject_invalid_state();
        return;
    }

    if ((g_dac_test_enable != 1U) ||
        (g_dac_test_arm != PSU_DAC_TEST_ARM_KEY))
    {
        analog_io_test_force_dac_zero();
        if (command != PSU_DAC_TEST_COMMAND_NONE)
        {
            ++g_dac_test_reject_count;
        }
        g_dac_test_command = PSU_DAC_TEST_COMMAND_NONE;
        return;
    }

    if (g_dac_test_follow_ui_enable == 1U)
    {
        if (command != PSU_DAC_TEST_COMMAND_NONE)
        {
            ++g_dac_test_reject_count;
            g_dac_test_command = PSU_DAC_TEST_COMMAND_NONE;
        }
        analog_io_test_process_follow_ui();
        return;
    }

    g_dac_test_follow_ui_active = 0U;
    s_dac_test_follow_ui_was_active = 0U;

    if (command == PSU_DAC_TEST_COMMAND_NONE)
    {
        return;
    }

    voltage_code = analog_io_test_limit_code(
        g_dac_test_voltage_code, PSU_DAC_TEST_VOLTAGE_MAX_CODE);
    current_code = analog_io_test_limit_code(
        g_dac_test_current_code, PSU_DAC_TEST_CURRENT_MAX_CODE);

    if ((command == PSU_DAC_TEST_COMMAND_WRITE_DACA) ||
        (command == PSU_DAC_TEST_COMMAND_WRITE_BOTH))
    {
        DAC_setShadowValue(IRIS_DACA_BASE, voltage_code);
        g_dac_test_voltage_applied_code = voltage_code;
        g_dac_test_voltage_expected_v =
            analog_io_test_dac_expected_v(voltage_code);
    }

    if ((command == PSU_DAC_TEST_COMMAND_WRITE_DACB) ||
        (command == PSU_DAC_TEST_COMMAND_WRITE_BOTH))
    {
        DAC_setShadowValue(IRIS_DACB_BASE, current_code);
        g_dac_test_current_applied_code = current_code;
        g_dac_test_current_expected_v =
            analog_io_test_dac_expected_v(current_code);
    }

    s_dac_test_outputs_zeroed =
        ((g_dac_test_voltage_applied_code == 0U) &&
         (g_dac_test_current_applied_code == 0U)) ? 1U : 0U;
    ++g_dac_test_apply_count;
    g_dac_test_command = PSU_DAC_TEST_COMMAND_NONE;
}

void analog_io_test_init(void)
{
    g_adc_test_vout_raw = 0U;
    g_adc_test_iout_raw = 0U;
    g_adc_test_vout_pin_v = 0.0f;
    g_adc_test_iout_pin_v = 0.0f;
    g_adc_test_vout_value_v = 0.0f;
    g_adc_test_iout_value_ma = 0.0f;
    g_adc_test_sample_count = 0U;

    g_dac_test_arm = 0U;
    g_dac_test_enable = 0U;
    g_dac_test_voltage_code = 0U;
    g_dac_test_current_code = 0U;
    g_dac_test_command = PSU_DAC_TEST_COMMAND_NONE;
    g_dac_test_apply_count = 0U;
    g_dac_test_reject_count = 0U;
    g_dac_test_follow_ui_enable = 0U;
    g_dac_test_follow_ui_active = 0U;
    g_dac_test_last_voltage_mv = 0U;
    g_dac_test_last_current_ma = 0U;
    g_dac_test_follow_update_count = 0U;
    s_dac_test_outputs_zeroed = 0U;
    s_dac_test_follow_ui_was_active = 0U;
    analog_io_test_force_dac_zero();

#if PSU_ENABLE_ANALOG_IO_TEST
    g_adc_test_enabled = 1U;
#else
    g_adc_test_enabled = 0U;
#endif
}

gmp_task_status_t analog_io_test_task(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

#if PSU_ENABLE_ANALOG_IO_TEST
    analog_io_test_copy_adc_diagnostics();
    analog_io_test_process_dac_command();
#else
    g_adc_test_enabled = 0U;
    analog_io_test_force_dac_zero();
    if (g_dac_test_command != PSU_DAC_TEST_COMMAND_NONE)
    {
        ++g_dac_test_reject_count;
        g_dac_test_command = PSU_DAC_TEST_COMMAND_NONE;
    }
#endif

    return GMP_TASK_DONE;
}

gmp_task_t task_analog_io_test = {
    "analog_io_test",
    analog_io_test_task,
    PSU_ANALOG_IO_TEST_TASK_PERIOD_MS,
    0U,
    PSU_ENABLE_ANALOG_IO_TEST,
    NULL
};

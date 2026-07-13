#include "power_app.h"

#include "power_hal.h"
#include "power_protection.h"
#include "power_self_test.h"

#include <ctrl_settings.h>
#include <gmp_core.h>
#if !PSU_SOFT_TEST_MODE
#include <xplt.peripheral.h>
#endif

volatile power_app_t g_power_app;
static uint16_t s_last_voltage_set_mv;
static uint16_t s_last_current_set_ma;

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

static void power_app_clear_mode_confirmation(void)
{
    g_power_app.cc_confirm_count = 0U;
    g_power_app.cv_confirm_count = 0U;
}

static bool power_app_check_command_change(void)
{
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;

    voltage_set_mv = g_power_app.voltage_set_mv;
    current_set_ma = g_power_app.current_set_ma;

    if ((voltage_set_mv != s_last_voltage_set_mv) ||
        (current_set_ma != s_last_current_set_ma))
    {
        s_last_voltage_set_mv = voltage_set_mv;
        s_last_current_set_ma = current_set_ma;
        power_app_clear_mode_confirmation();
        return true;
    }

    return false;
}

static void power_app_update_mode(void)
{
    uint16_t voltage_meas_mv;
    uint16_t current_meas_ma;
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    bool cc_candidate;
    bool cv_candidate;

    if (!g_power_app.output_requested || !g_power_app.output_enabled ||
        g_power_app.fault_latched ||
        ((g_power_app.state != POWER_STATE_CV) && (g_power_app.state != POWER_STATE_CC)))
    {
        power_app_clear_mode_confirmation();
        return;
    }

    // Snapshot volatile measurements and commands before evaluating a mode transition.
    voltage_meas_mv = g_power_app.voltage_meas_mv;
    current_meas_ma = g_power_app.current_meas_ma;
    voltage_set_mv = g_power_app.voltage_set_mv;
    current_set_ma = g_power_app.current_set_ma;

    cc_candidate = (((uint32_t)current_meas_ma + PSU_CC_ENTER_CURRENT_MARGIN_MA) >=
                    (uint32_t)current_set_ma) &&
                   (((uint32_t)voltage_meas_mv + PSU_CC_ENTER_VOLTAGE_DROP_MV) <
                    (uint32_t)voltage_set_mv);
    cv_candidate = (((uint32_t)current_meas_ma + PSU_CV_RETURN_CURRENT_MARGIN_MA) <
                    (uint32_t)current_set_ma) ||
                   (((uint32_t)voltage_meas_mv + PSU_CV_RETURN_VOLTAGE_MARGIN_MV) >=
                    (uint32_t)voltage_set_mv);

    if (g_power_app.state == POWER_STATE_CV)
    {
        g_power_app.cv_confirm_count = 0U;
        if (cc_candidate)
        {
            if (g_power_app.cc_confirm_count < PSU_MODE_CONFIRM_COUNT)
            {
                ++g_power_app.cc_confirm_count;
            }
            if (g_power_app.cc_confirm_count >= PSU_MODE_CONFIRM_COUNT)
            {
                g_power_app.state = POWER_STATE_CC;
                power_app_clear_mode_confirmation();
            }
        }
        else
        {
            g_power_app.cc_confirm_count = 0U;
        }
    }
    else
    {
        g_power_app.cc_confirm_count = 0U;
        if (cv_candidate)
        {
            if (g_power_app.cv_confirm_count < PSU_MODE_CONFIRM_COUNT)
            {
                ++g_power_app.cv_confirm_count;
            }
            if (g_power_app.cv_confirm_count >= PSU_MODE_CONFIRM_COUNT)
            {
                g_power_app.state = POWER_STATE_CV;
                power_app_clear_mode_confirmation();
            }
        }
        else
        {
            g_power_app.cv_confirm_count = 0U;
        }
    }
}

static void power_app_set_output_off(power_state_t state)
{
    power_output_hw_set(false);
    power_dac_set_zero();
    g_power_app.dac_voltage_code = 0U;
    g_power_app.dac_current_code = 0U;
    g_power_app.output_enabled = false;
    g_power_app.state = state;
    power_app_clear_mode_confirmation();
}

static void power_app_trip(power_fault_t fault,
                           uint16_t voltage_mv,
                           uint16_t current_ma)
{
    if (!g_power_app.fault_latched)
    {
        g_power_app.fault = fault;
        g_power_app.fault_latched = true;
        g_power_app.trip_voltage_mv = voltage_mv;
        g_power_app.trip_current_ma = current_ma;
    }

    g_power_app.output_requested = false;
    g_power_app.fault_reset_requested = false;
    power_app_set_output_off(POWER_STATE_FAULT);
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
    g_power_app.trip_voltage_mv = 0U;
    g_power_app.trip_current_ma = 0U;
    g_power_app.dac_voltage_code = 0U;
    g_power_app.dac_current_code = 0U;
    g_power_app.cc_confirm_count = 0U;
    g_power_app.cv_confirm_count = 0U;
    g_power_app.state = POWER_STATE_OFF;
    g_power_app.fault = POWER_FAULT_NONE;
    g_power_app.output_requested = false;
    g_power_app.output_enabled = false;
    g_power_app.fault_latched = false;
    g_power_app.fault_reset_requested = false;
    s_last_voltage_set_mv = g_power_app.voltage_set_mv;
    s_last_current_set_ma = g_power_app.current_set_ma;

    power_output_hw_set(false);
    power_dac_set_zero();
    power_protection_init();
}

void power_app_set_voltage_mv(uint16_t voltage_mv)
{
    g_power_app.voltage_set_mv = (voltage_mv > PSU_VOLTAGE_CMD_MAX_MV) ?
                                     PSU_VOLTAGE_CMD_MAX_MV : voltage_mv;
}

void power_app_set_current_ma(uint16_t current_ma)
{
    g_power_app.current_set_ma = (current_ma > PSU_CURRENT_CMD_MAX_MA) ?
                                     PSU_CURRENT_CMD_MAX_MA : current_ma;
}

uint16_t power_app_get_voltage_mv(void)
{
    return g_power_app.voltage_set_mv;
}

uint16_t power_app_get_current_ma(void)
{
    return g_power_app.current_set_ma;
}

void power_app_request_output(bool enable)
{
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_OUTPUT_REQUEST
    if (enable)
    {
        gmp_base_print("SAFE_BRINGUP: physical output command blocked\r\n");
    }
    g_power_app.output_requested = false;
    return;
#else
    if (enable)
    {
        // Write first, then re-check the ISR-owned latch. Whichever side wins
        // the race leaves the request disabled after a fault is latched.
        g_power_app.output_requested = true;
        if (g_power_app.fault_latched)
        {
            g_power_app.output_requested = false;
        }
    }
    else
    {
        g_power_app.output_requested = false;
    }
#endif
}

void power_app_reset_fault(void)
{
    g_power_app.fault_reset_requested = true;
}

void power_app_fast_step(void)
{
    bool command_changed;
    bool protection_enabled;
    bool update_mode = false;
    uint16_t voltage_meas_mv;
    uint16_t current_meas_ma;
    power_protection_result_t protection_result;
#if PSU_SOFT_TEST_MODE
    uint16_t virtual_voltage_mv;
    uint16_t virtual_current_ma;
#endif

    power_app_limit_commands();
    command_changed = power_app_check_command_change();

#if PSU_SOFT_TEST_MODE
    if (power_self_test_get_measurement(&virtual_voltage_mv, &virtual_current_ma))
    {
        g_power_app.voltage_meas_mv = virtual_voltage_mv;
        g_power_app.current_meas_ma = virtual_current_ma;
    }
#else
    g_power_app.voltage_meas_mv = power_float_to_u16(g_vout_meas_v * 1000.0f);
    g_power_app.current_meas_ma = power_float_to_u16(g_iout_meas_ma);
#endif

    if (PSU_SAFE_BRINGUP || !PSU_ALLOW_OUTPUT_REQUEST ||
        !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE)
    {
        // This barrier is evaluated in every fast control step. It sanitizes
        // even direct debugger or memory writes before the application can
        // enter an output state or apply a physical command.
        power_output_hw_set(false);
        power_dac_set_zero();
        g_power_app.output_requested = false;
        g_power_app.output_enabled = false;
        g_power_app.fault_reset_requested = false;
        g_power_app.state = POWER_STATE_OFF;
        g_power_app.dac_voltage_code = power_voltage_mv_to_dac(g_power_app.voltage_set_mv);
        g_power_app.dac_current_code = power_current_ma_to_dac(g_power_app.current_set_ma);
        power_app_clear_mode_confirmation();
        (void)power_protection_step(g_power_app.voltage_meas_mv,
                                    g_power_app.current_meas_ma,
                                    false);
    }
    else
    {
        voltage_meas_mv = g_power_app.voltage_meas_mv;
        current_meas_ma = g_power_app.current_meas_ma;

    if (g_power_app.fault_latched)
    {
        power_app_set_output_off(POWER_STATE_FAULT);

        if (!g_power_app.fault_reset_requested)
        {
            return;
        }

        g_power_app.fault_reset_requested = false;
        if (!g_power_app.output_requested &&
            power_protection_release_safe(voltage_meas_mv, current_meas_ma))
        {
            g_power_app.fault = POWER_FAULT_NONE;
            g_power_app.fault_latched = false;
            g_power_app.trip_voltage_mv = 0U;
            g_power_app.trip_current_ma = 0U;
            power_protection_reset();
            power_app_set_output_off(POWER_STATE_OFF);
        }
        return;
    }

    // A reset request made without a latched fault must not arm a future reset.
    g_power_app.fault_reset_requested = false;

    if (!g_power_app.output_requested)
    {
        power_protection_reset();
        power_app_set_output_off(POWER_STATE_OFF);
        return;
    }

    protection_enabled = (g_power_app.state == POWER_STATE_STARTING) ||
                         (g_power_app.state == POWER_STATE_CV) ||
                         (g_power_app.state == POWER_STATE_CC);
    protection_result = power_protection_step(voltage_meas_mv,
                                              current_meas_ma,
                                              protection_enabled);

    if (protection_result == POWER_PROTECTION_RESULT_OVERVOLTAGE)
    {
        power_app_trip(POWER_FAULT_OVERVOLTAGE, voltage_meas_mv, current_meas_ma);
        return;
    }
    if (protection_result == POWER_PROTECTION_RESULT_OVERCURRENT)
    {
        power_app_trip(POWER_FAULT_OVERCURRENT, voltage_meas_mv, current_meas_ma);
        return;
    }

    switch (g_power_app.state)
    {
    case POWER_STATE_OFF:
        power_app_set_output_off(POWER_STATE_STARTING);
        break;

    case POWER_STATE_STARTING:
        power_app_clear_mode_confirmation();
        power_app_apply_commands();
        power_output_hw_set(true);
        g_power_app.output_enabled = power_output_hw_get();
        if (g_power_app.output_enabled)
        {
            g_power_app.state = POWER_STATE_CV;
            power_app_clear_mode_confirmation();
        }
        break;

    case POWER_STATE_CV:
    case POWER_STATE_CC:
        power_app_apply_commands();
        power_output_hw_set(true);
        g_power_app.output_enabled = power_output_hw_get();
        if (g_power_app.output_enabled)
        {
            update_mode = true;
        }
        else
        {
            power_app_set_output_off(POWER_STATE_STARTING);
        }
        break;

    case POWER_STATE_FAULT:
    default:
        power_app_set_output_off(POWER_STATE_FAULT);
        break;
    }

        if (update_mode && !command_changed)
        {
            power_app_update_mode();
        }
    }
}

void power_app_slow_step(void)
{
#if PSU_SOFT_TEST_MODE
    power_self_test_step();
#endif
}

#include "power_app.h"

#include "analog_io_test.h"
#include "power_control_policy.h"
#include "power_hal.h"
#include "power_protection.h"
#include "power_self_test.h"

#include <ctrl_settings.h>
#include <gmp_core.h>
#if !PSU_SOFT_TEST_MODE || \
    (PSU_ENABLE_ANALOG_BOARD_BRINGUP && PSU_ANALOG_BOARD_USE_REAL_ADC && \
     PSU_REAL_FEEDBACK_CONNECTED)
#include <xplt.peripheral.h>
#endif

volatile power_app_t g_power_app;
volatile uint32_t g_blocked_output_request_count = 0U;
volatile uint16_t g_analog_board_voltage_meas_mv = 0U;
volatile uint16_t g_analog_board_current_meas_ma = 0U;
volatile uint16_t g_analog_board_real_feedback_active = 0U;
volatile uint32_t g_analog_board_feedback_update_count = 0U;
volatile uint16_t g_analog_board_ovp_confirm_count = 0U;
volatile uint16_t g_analog_board_ocp_confirm_count = 0U;
volatile uint32_t g_analog_board_ovp_transient_count = 0U;
volatile uint32_t g_analog_board_ocp_transient_count = 0U;
volatile uint16_t g_output_switch_requested = 0U;
volatile uint16_t g_output_switch_active = 0U;
volatile uint16_t g_output_switch_precharge_active = 0U;
volatile uint16_t g_output_switch_precharge_complete = 0U;
volatile uint16_t g_output_switch_dac_gate_active = 0U;
volatile uint16_t g_output_switch_physical_relay_available = 0U;
volatile uint16_t g_output_switch_relay_command = 0U;
volatile uint32_t g_output_switch_toggle_count = 0U;
volatile uint32_t g_output_switch_enable_count = 0U;
volatile uint32_t g_output_switch_disable_count = 0U;
volatile uint32_t g_output_switch_reject_count = 0U;
volatile uint32_t g_output_switch_fault_shutdown_count = 0U;
volatile uint32_t g_output_switch_precharge_count = 0U;
volatile time_gt g_output_switch_precharge_start_tick = 0U;
static uint16_t s_last_voltage_set_mv;
static uint16_t s_last_current_set_ma;

#if !PSU_SOFT_TEST_MODE || \
    (PSU_ENABLE_ANALOG_BOARD_BRINGUP && PSU_ANALOG_BOARD_USE_REAL_ADC && \
     PSU_REAL_FEEDBACK_CONNECTED)
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

#if PSU_ENABLE_ANALOG_BOARD_BRINGUP && PSU_ANALOG_BOARD_USE_REAL_ADC && \
    PSU_REAL_FEEDBACK_CONNECTED
static power_protection_result_t power_app_update_analog_board_protection(
    uint16_t voltage_mv,
    uint16_t current_ma)
{
    if ((g_analog_board_iset_precharge_complete != 1U) ||
        (g_analog_board_protection_grace_active != 0U))
    {
        g_analog_board_ovp_confirm_count = 0U;
        g_analog_board_ocp_confirm_count = 0U;
        if (g_analog_board_protection_grace_active != 0U)
        {
            if (voltage_mv >= PSU_OVP_TRIP_MV)
            {
                ++g_analog_board_ovp_transient_count;
            }
            if (current_ma >= PSU_OCP_TRIP_MA)
            {
                ++g_analog_board_ocp_transient_count;
            }
        }
        return POWER_PROTECTION_RESULT_NONE;
    }

    if (voltage_mv >= PSU_OVP_TRIP_MV)
    {
        if (g_analog_board_ovp_confirm_count <
            PSU_ANALOG_BOARD_OVP_CONFIRM_COUNT)
        {
            ++g_analog_board_ovp_confirm_count;
        }
        if (g_analog_board_ovp_confirm_count <
            PSU_ANALOG_BOARD_OVP_CONFIRM_COUNT)
        {
            ++g_analog_board_ovp_transient_count;
        }
    }
    else
    {
        g_analog_board_ovp_confirm_count = 0U;
    }

    if (current_ma >= PSU_OCP_TRIP_MA)
    {
        if (g_analog_board_ocp_confirm_count <
            PSU_ANALOG_BOARD_OCP_CONFIRM_COUNT)
        {
            ++g_analog_board_ocp_confirm_count;
        }
        if (g_analog_board_ocp_confirm_count <
            PSU_ANALOG_BOARD_OCP_CONFIRM_COUNT)
        {
            ++g_analog_board_ocp_transient_count;
        }
    }
    else
    {
        g_analog_board_ocp_confirm_count = 0U;
    }

    // Match the public protection priority when both confirm together.
    if (g_analog_board_ovp_confirm_count >=
        PSU_ANALOG_BOARD_OVP_CONFIRM_COUNT)
    {
        return POWER_PROTECTION_RESULT_OVERVOLTAGE;
    }
    if (g_analog_board_ocp_confirm_count >=
        PSU_ANALOG_BOARD_OCP_CONFIRM_COUNT)
    {
        return POWER_PROTECTION_RESULT_OVERCURRENT;
    }
    return POWER_PROTECTION_RESULT_NONE;
}
#endif

static void power_app_limit_commands(void)
{
    if (g_power_app.voltage_set_mv > PSU_COMMAND_VOLTAGE_LIMIT_MV)
    {
        g_power_app.voltage_set_mv = PSU_COMMAND_VOLTAGE_LIMIT_MV;
    }
    if (g_power_app.current_set_ma > PSU_COMMAND_CURRENT_LIMIT_MA)
    {
        g_power_app.current_set_ma = PSU_COMMAND_CURRENT_LIMIT_MA;
    }
}

static void power_app_clear_mode_confirmation(void)
{
    g_power_app.cc_confirm_count = 0U;
    g_power_app.cv_confirm_count = 0U;
}

static bool power_app_physical_output_forbidden(void)
{
    return PSU_SAFE_BRINGUP || !PSU_ALLOW_OUTPUT_REQUEST ||
           !PSU_ALLOW_PHYSICAL_OUTPUT_ENABLE ||
           !PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE;
}

static bool power_app_output_switch_fault_active(void)
{
    return g_power_app.fault_latched ||
           (g_analog_board_feedback_fault != 0U) ||
           (g_analog_board_fault_hold_active != 0U) ||
           (g_analog_board_fault_shutdown_active != 0U) ||
           power_control_policy_fault_active() ||
           (g_power_app.state == POWER_STATE_FAULT);
}

static bool power_app_output_switch_ready(void)
{
    if (power_app_output_switch_fault_active() ||
        (g_analog_board_protection_grace_active != 0U) ||
        (g_analog_board_iset_precharge_complete != 1U) ||
        (g_dac_test_auto_follow_completed != 1U))
    {
        return false;
    }

#if PSU_OUTPUT_SWITCH_REQUIRE_SETTLED && PSU_REAL_FEEDBACK_CONNECTED
    if (g_analog_board_feedback_settled != 1U)
    {
        return false;
    }
#endif

    return true;
}

static void power_app_output_switch_set_off(power_state_t state)
{
    g_output_switch_dac_gate_active = 0U;
    g_output_switch_active = 0U;
    g_output_switch_precharge_active = 0U;
    g_output_switch_precharge_complete = 0U;
    g_output_switch_requested = 0U;
    g_output_switch_precharge_start_tick = 0U;
    g_output_switch_physical_relay_available =
        PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE ? 1U : 0U;
    g_output_switch_relay_command = 0U;

    g_power_app.output_requested = false;
    g_power_app.output_enabled = false;
    g_power_app.state = state;
    power_output_hw_set(false);
    power_app_clear_mode_confirmation();
}

void power_app_output_switch_fault_shutdown(void)
{
#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
    if ((g_power_app.state != POWER_STATE_FAULT) ||
        (g_output_switch_requested != 0U) ||
        (g_output_switch_active != 0U) ||
        (g_output_switch_precharge_active != 0U) ||
        (g_output_switch_dac_gate_active != 0U))
    {
        ++g_output_switch_fault_shutdown_count;
    }

    if (!g_power_app.fault_latched &&
        ((g_analog_board_feedback_fault != 0U) ||
         (g_analog_board_fault_hold_active != 0U)))
    {
        g_power_app.fault = POWER_FAULT_NONE;
        g_power_app.fault_latched = true;
        g_power_app.trip_voltage_mv = g_power_app.voltage_meas_mv;
        g_power_app.trip_current_ma = g_power_app.current_meas_ma;
    }

    g_power_app.fault_reset_requested = false;
    power_app_output_switch_set_off(POWER_STATE_FAULT);
#endif
}

void power_app_output_switch_policy_shutdown(void)
{
#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
    g_power_app.fault_reset_requested = false;
    power_app_output_switch_set_off(POWER_STATE_FAULT);
    analog_io_test_force_safe_outputs();
#endif
}

void power_app_output_switch_policy_reset(void)
{
#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
    g_power_app.fault_reset_requested = false;
    power_app_output_switch_set_off(POWER_STATE_OFF);
    analog_io_test_force_safe_outputs();
#endif
}

static void power_app_update_output_switch(void)
{
#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
    g_output_switch_physical_relay_available =
        PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE ? 1U : 0U;

    if (power_app_output_switch_fault_active())
    {
        power_app_output_switch_fault_shutdown();
        analog_io_test_force_safe_outputs();
        return;
    }

    if (g_output_switch_precharge_active != 0U)
    {
        g_output_switch_relay_command = 0U;
        power_output_hw_set(false);

        if (g_output_switch_requested == 0U)
        {
            power_app_output_switch_set_off(POWER_STATE_OFF);
            analog_io_test_force_safe_outputs();
            return;
        }

        if (gmp_base_is_delay_elapsed(g_output_switch_precharge_start_tick,
                                      PSU_OUTPUT_SWITCH_PRECHARGE_MS))
        {
            g_output_switch_precharge_active = 0U;
            g_output_switch_precharge_complete = 1U;
            g_output_switch_dac_gate_active = 1U;
            g_power_app.output_requested = true;
            power_output_hw_set(true);
            power_output_hw_service();

            if (power_app_output_switch_fault_active())
            {
                power_app_output_switch_fault_shutdown();
                analog_io_test_force_safe_outputs();
                return;
            }

            if (!power_output_hw_get())
            {
                ++g_output_switch_reject_count;
                power_app_output_switch_set_off(POWER_STATE_OFF);
                analog_io_test_force_safe_outputs();
                return;
            }

            g_output_switch_relay_command = 1U;
            g_power_app.output_enabled = true;
            g_power_app.state = POWER_STATE_CV;
            g_output_switch_active = 1U;
            ++g_output_switch_enable_count;
            power_app_clear_mode_confirmation();
        }
        return;
    }

    if (g_output_switch_active != 0U)
    {
        if (g_output_switch_requested == 0U)
        {
            power_app_output_switch_set_off(POWER_STATE_OFF);
            analog_io_test_force_safe_outputs();
            return;
        }

        if (!power_output_hw_get())
        {
            ++g_output_switch_reject_count;
            power_app_output_switch_set_off(POWER_STATE_OFF);
            analog_io_test_force_safe_outputs();
            return;
        }

        g_output_switch_relay_command = 1U;
        g_power_app.output_requested = true;
        g_power_app.output_enabled = true;
        return;
    }

    if (g_output_switch_requested == 0U)
    {
        g_output_switch_relay_command = 0U;
        power_output_hw_set(false);
        g_power_app.output_requested = false;
        g_power_app.output_enabled = false;
        g_power_app.state = POWER_STATE_OFF;
        return;
    }

    if (!power_app_output_switch_ready())
    {
        ++g_output_switch_reject_count;
        power_app_output_switch_set_off(POWER_STATE_OFF);
        analog_io_test_force_safe_outputs();
        return;
    }

    g_output_switch_precharge_active = 1U;
    g_output_switch_precharge_complete = 0U;
    g_output_switch_dac_gate_active = 0U;
    g_output_switch_active = 0U;
    g_output_switch_relay_command = 0U;
    power_output_hw_set(false);
    g_power_app.output_requested = false;
    g_power_app.output_enabled = false;
    g_power_app.state = POWER_STATE_STARTING;
    g_output_switch_precharge_start_tick = gmp_base_get_system_tick();
    ++g_output_switch_precharge_count;
    power_app_clear_mode_confirmation();
#endif
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

#if PSU_REAL_FEEDBACK_CONNECTED
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
#endif

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
#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
    power_app_output_switch_fault_shutdown();
    analog_io_test_force_safe_outputs();
#else
    power_app_set_output_off(POWER_STATE_FAULT);
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
#if PSU_ENABLE_LOW_RANGE_BRINGUP_LIMITS
    g_power_app.current_set_ma = PSU_ANALOG_BOARD_MIN_CURRENT_MA;
#else
    g_power_app.current_set_ma = 0U;
#endif
    g_power_app.voltage_meas_mv = 0U;
    g_power_app.current_meas_ma = 0U;
    g_power_app.trip_voltage_mv = 0U;
    g_power_app.trip_current_ma = 0U;
    g_power_app.dac_voltage_code = 0U;
    g_power_app.dac_current_code =
        power_current_ma_to_dac(g_power_app.current_set_ma);
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
    g_analog_board_voltage_meas_mv = 0U;
    g_analog_board_current_meas_ma = 0U;
    g_analog_board_real_feedback_active = 0U;
    g_analog_board_feedback_update_count = 0U;
    g_analog_board_ovp_confirm_count = 0U;
    g_analog_board_ocp_confirm_count = 0U;
    g_analog_board_ovp_transient_count = 0U;
    g_analog_board_ocp_transient_count = 0U;
    g_output_switch_requested = 0U;
    g_output_switch_active = 0U;
    g_output_switch_precharge_active = 0U;
    g_output_switch_precharge_complete = 0U;
    g_output_switch_dac_gate_active = 0U;
    g_output_switch_physical_relay_available =
        PSU_OUTPUT_SWITCH_PHYSICAL_RELAY_ENABLE ? 1U : 0U;
    g_output_switch_relay_command = 0U;
    g_output_switch_toggle_count = 0U;
    g_output_switch_enable_count = 0U;
    g_output_switch_disable_count = 0U;
    g_output_switch_reject_count = 0U;
    g_output_switch_fault_shutdown_count = 0U;
    g_output_switch_precharge_count = 0U;
    g_output_switch_precharge_start_tick = 0U;

    power_output_hw_set(false);
    power_dac_set_zero();
    power_protection_init();
}

void power_app_set_voltage_mv(uint16_t voltage_mv)
{
    g_power_app.voltage_set_mv =
        (voltage_mv > PSU_COMMAND_VOLTAGE_LIMIT_MV) ?
            PSU_COMMAND_VOLTAGE_LIMIT_MV : voltage_mv;
}

void power_app_set_current_ma(uint16_t current_ma)
{
    g_power_app.current_set_ma =
        (current_ma > PSU_COMMAND_CURRENT_LIMIT_MA) ?
            PSU_COMMAND_CURRENT_LIMIT_MA : current_ma;
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
        ++g_blocked_output_request_count;
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

void power_app_request_logical_output(bool enable)
{
#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
    ++g_output_switch_toggle_count;

    if (!enable)
    {
        if ((g_output_switch_requested != 0U) ||
            (g_output_switch_active != 0U) ||
            (g_output_switch_precharge_active != 0U))
        {
            ++g_output_switch_disable_count;
        }
        power_app_output_switch_set_off(POWER_STATE_OFF);
        analog_io_test_force_safe_outputs();
        return;
    }

    if (power_app_output_switch_fault_active())
    {
        ++g_output_switch_reject_count;
        power_app_output_switch_fault_shutdown();
        analog_io_test_force_safe_outputs();
        return;
    }

    g_output_switch_requested = 1U;
    g_power_app.output_requested = false;
#else
    power_app_request_output(enable);
#endif
}

void power_app_reset_fault(void)
{
    g_power_app.fault_reset_requested = true;
}

void power_app_fast_step(void)
{
    bool command_changed;
#if PSU_REAL_FEEDBACK_CONNECTED
    bool protection_enabled;
#endif
#if PSU_REAL_FEEDBACK_CONNECTED
    bool update_mode = false;
#endif
    uint16_t voltage_meas_mv;
    uint16_t current_meas_ma;
    power_protection_result_t protection_result;
#if PSU_SOFT_TEST_MODE && \
    !(PSU_ENABLE_ANALOG_BOARD_BRINGUP && PSU_ANALOG_BOARD_USE_REAL_ADC && \
      PSU_REAL_FEEDBACK_CONNECTED)
    uint16_t virtual_voltage_mv;
    uint16_t virtual_current_ma;
#endif

    power_app_limit_commands();
    command_changed = power_app_check_command_change();

#if PSU_ENABLE_ANALOG_BOARD_BRINGUP && PSU_ANALOG_BOARD_USE_REAL_ADC && \
    PSU_REAL_FEEDBACK_CONNECTED
    g_power_app.voltage_meas_mv =
        power_float_to_u16(g_vout_meas_v * 1000.0f);
    g_power_app.current_meas_ma = power_float_to_u16(g_iout_meas_ma);
    g_analog_board_voltage_meas_mv = g_power_app.voltage_meas_mv;
    g_analog_board_current_meas_ma = g_power_app.current_meas_ma;
    g_analog_board_real_feedback_active = 1U;
    ++g_analog_board_feedback_update_count;
#elif PSU_SOFT_TEST_MODE
    g_analog_board_real_feedback_active = 0U;
    if (power_self_test_get_measurement(&virtual_voltage_mv, &virtual_current_ma))
    {
        g_power_app.voltage_meas_mv = virtual_voltage_mv;
        g_power_app.current_meas_ma = virtual_current_ma;
    }
#else
    g_analog_board_real_feedback_active = 0U;
    g_power_app.voltage_meas_mv = power_float_to_u16(g_vout_meas_v * 1000.0f);
    g_power_app.current_meas_ma = power_float_to_u16(g_iout_meas_ma);
#endif

#if PSU_ENABLE_ANALOG_BOARD_BRINGUP && PSU_ANALOG_BOARD_USE_REAL_ADC && \
    PSU_REAL_FEEDBACK_CONNECTED
    voltage_meas_mv = g_power_app.voltage_meas_mv;
    current_meas_ma = g_power_app.current_meas_ma;
    if (!g_power_app.fault_latched)
    {
        protection_result = power_app_update_analog_board_protection(
            voltage_meas_mv,
            current_meas_ma);
        if (protection_result == POWER_PROTECTION_RESULT_OVERVOLTAGE)
        {
            power_app_trip(POWER_FAULT_OVERVOLTAGE,
                           voltage_meas_mv,
                           current_meas_ma);
            return;
        }
        if (protection_result == POWER_PROTECTION_RESULT_OVERCURRENT)
        {
            power_app_trip(POWER_FAULT_OVERCURRENT,
                           voltage_meas_mv,
                           current_meas_ma);
            return;
        }
    }
#endif

#if PSU_ENABLE_ANALOG_BOARD_BRINGUP && PSU_REAL_FEEDBACK_CONNECTED
    if (g_analog_board_feedback_settled != 1U)
    {
        voltage_meas_mv = g_power_app.voltage_meas_mv;
        current_meas_ma = g_power_app.current_meas_ma;
        protection_enabled = false;
        (void)power_protection_step(voltage_meas_mv,
                                    current_meas_ma,
                                    protection_enabled);
        power_output_hw_set(false);
        power_dac_set_zero();
        g_power_app.output_requested = false;
        g_power_app.output_enabled = false;
        g_power_app.fault_reset_requested = false;
#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
        if (g_power_app.fault_latched ||
            (g_analog_board_feedback_fault != 0U) ||
            (g_analog_board_fault_hold_active != 0U))
        {
            power_app_output_switch_fault_shutdown();
            analog_io_test_force_safe_outputs();
        }
        else if (power_control_policy_fault_active())
        {
            power_app_output_switch_policy_shutdown();
        }
        else if ((g_output_switch_active != 0U) ||
                 (g_output_switch_precharge_active != 0U) ||
                 (g_output_switch_dac_gate_active != 0U))
        {
            ++g_output_switch_reject_count;
            power_app_output_switch_set_off(POWER_STATE_OFF);
            analog_io_test_force_safe_outputs();
        }
        else
        {
            g_power_app.state = POWER_STATE_OFF;
            power_app_clear_mode_confirmation();
        }
#else
        g_power_app.state = g_power_app.fault_latched ?
                                POWER_STATE_FAULT : POWER_STATE_OFF;
        power_app_clear_mode_confirmation();
#endif
        return;
    }
#endif

#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
    if (power_app_physical_output_forbidden())
    {
        power_output_hw_set(false);
        g_output_switch_physical_relay_available = 0U;
        g_output_switch_relay_command = 0U;
        g_power_app.dac_voltage_code = g_dac_test_voltage_applied_code;
        g_power_app.dac_current_code = g_dac_test_current_applied_code;

        if (g_power_app.fault_latched ||
            (g_analog_board_feedback_fault != 0U) ||
            (g_analog_board_fault_hold_active != 0U))
        {
            power_app_output_switch_fault_shutdown();
            analog_io_test_force_safe_outputs();
            return;
        }

        if (power_control_policy_fault_active())
        {
            power_app_output_switch_policy_shutdown();
            return;
        }

        g_power_app.fault_reset_requested = false;
        g_power_app.output_requested = (g_output_switch_active != 0U);
        g_power_app.output_enabled = (g_output_switch_active != 0U);

        if (g_output_switch_active != 0U)
        {
            if ((g_power_app.state != POWER_STATE_CV) &&
                (g_power_app.state != POWER_STATE_CC))
            {
                g_power_app.state = POWER_STATE_CV;
                power_app_clear_mode_confirmation();
            }
            if (!command_changed)
            {
#if PSU_REAL_FEEDBACK_CONNECTED
                power_app_update_mode();
#endif
            }
        }
        else if (g_output_switch_precharge_active != 0U)
        {
            g_power_app.state = POWER_STATE_STARTING;
            power_app_clear_mode_confirmation();
        }
        else
        {
            g_power_app.state = POWER_STATE_OFF;
            power_app_clear_mode_confirmation();
        }
        return;
    }
#endif

#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
    if (power_control_policy_fault_active())
    {
        power_app_output_switch_policy_shutdown();
        return;
    }
#endif

    if (power_app_physical_output_forbidden())
    {
        // Keep every physical path off while the software model, measurement
        // override, protection counters, fault latch, and reset remain active.
        power_output_hw_set(false);
        power_dac_set_zero();
        g_power_app.output_requested = false;
        g_power_app.output_enabled = false;
        g_power_app.dac_voltage_code = power_voltage_mv_to_dac(g_power_app.voltage_set_mv);
        g_power_app.dac_current_code = power_current_ma_to_dac(g_power_app.current_set_ma);

        voltage_meas_mv = g_power_app.voltage_meas_mv;
        current_meas_ma = g_power_app.current_meas_ma;

        if (g_power_app.fault_latched)
        {
#if PSU_ENABLE_ANALOG_BOARD_BRINGUP
            // Bring-up faults remain latched; no reset may restart DAC follow.
            g_power_app.fault_reset_requested = false;
#else
            if (g_power_app.fault_reset_requested &&
                power_protection_release_safe(voltage_meas_mv, current_meas_ma))
            {
                g_power_app.fault = POWER_FAULT_NONE;
                g_power_app.fault_latched = false;
                g_power_app.trip_voltage_mv = 0U;
                g_power_app.trip_current_ma = 0U;
                power_protection_reset();
            }
            g_power_app.fault_reset_requested = false;
#endif
        }
        else
        {
            g_power_app.fault_reset_requested = false;
#if PSU_REAL_FEEDBACK_CONNECTED && PSU_ENABLE_ANALOG_BOARD_BRINGUP && \
    PSU_ANALOG_BOARD_USE_REAL_ADC
            // The dedicated 20 kHz bring-up filter already ran above.
            protection_result = POWER_PROTECTION_RESULT_NONE;
#elif PSU_REAL_FEEDBACK_CONNECTED
            protection_result = power_protection_step(voltage_meas_mv,
                                                      current_meas_ma,
                                                      true);
#else
            protection_result = POWER_PROTECTION_RESULT_NONE;
#endif
            if (protection_result == POWER_PROTECTION_RESULT_OVERVOLTAGE)
            {
                g_power_app.fault = POWER_FAULT_OVERVOLTAGE;
                g_power_app.fault_latched = true;
                g_power_app.trip_voltage_mv = voltage_meas_mv;
                g_power_app.trip_current_ma = current_meas_ma;
            }
            else if (protection_result == POWER_PROTECTION_RESULT_OVERCURRENT)
            {
                g_power_app.fault = POWER_FAULT_OVERCURRENT;
                g_power_app.fault_latched = true;
                g_power_app.trip_voltage_mv = voltage_meas_mv;
                g_power_app.trip_current_ma = current_meas_ma;
            }
        }

        g_power_app.state = g_power_app.fault_latched ?
                                POWER_STATE_FAULT : POWER_STATE_OFF;
        power_app_clear_mode_confirmation();
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

#if PSU_REAL_FEEDBACK_CONNECTED
    protection_enabled = (g_power_app.state == POWER_STATE_STARTING) ||
                         (g_power_app.state == POWER_STATE_CV) ||
                         (g_power_app.state == POWER_STATE_CC);
#endif
#if PSU_REAL_FEEDBACK_CONNECTED && PSU_ENABLE_ANALOG_BOARD_BRINGUP && \
    PSU_ANALOG_BOARD_USE_REAL_ADC
    // The dedicated 20 kHz bring-up filter already ran above.
    protection_result = POWER_PROTECTION_RESULT_NONE;
#elif PSU_REAL_FEEDBACK_CONNECTED
    protection_result = power_protection_step(voltage_meas_mv,
                                              current_meas_ma,
                                              protection_enabled);
#else
    protection_result = POWER_PROTECTION_RESULT_NONE;
#endif

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
#if PSU_REAL_FEEDBACK_CONNECTED
            update_mode = true;
#endif
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

#if PSU_REAL_FEEDBACK_CONNECTED
        if (update_mode && !command_changed)
        {
            power_app_update_mode();
        }
#endif
    }
}

void power_app_slow_step(void)
{
    power_output_hw_service();

#if PSU_SOFT_TEST_MODE
    power_self_test_step();
#endif

    power_app_update_output_switch();
}

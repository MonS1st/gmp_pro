
// GMP basic core header
#include <gmp_core.h>
#include <ctrl_settings.h>
#include <xplt.peripheral.h>

// user main header
#include "ctl_main.h"
#include "analog_io_test.h"
#include "power_app.h"
#include "power_control_policy.h"
#include "power_mode_monitor.h"
#include "power_settings_store.h"
#include "user_main.h"
#include <stdio.h>
#include <stdlib.h>

// peripheral
#include <core/dev/display/ht16k33.h>
#include <core/dev/gpio/pca9555.h>
#include <core/dev/sensor/hdc1080.h>

#include <oled_driver.h>

//=================================================================================================
// BEEP control function



gpio_halt gpio_beep;
gpio_halt gpio_fault_led;

void beep_on()
{
#if PSU_ENABLE_BEEP && !PSU_SAFE_BRINGUP
    gmp_hal_gpio_write(gpio_beep, PSU_BUZZER_ACTIVE_LEVEL);
#endif
}

void beep_off()
{
#if PSU_ENABLE_BEEP && !PSU_SAFE_BRINGUP
    gmp_hal_gpio_write(gpio_beep, PSU_BUZZER_INACTIVE_LEVEL);
#endif
}

void fault_led_on(void)
{
    gmp_hal_gpio_write(gpio_fault_led, 1U);
}

void fault_led_off(void)
{
    gmp_hal_gpio_write(gpio_fault_led, 0U);
}

//=================================================================================================
// LED control function

// devices
iic_halt iic_bus;
ht16k33_dev_t ht16k33;
hdc1080_dev_t hdc1080;

// Global volatile state remains directly observable in CCS Expressions.
volatile uint16_t s_last_key_id = 0U;
volatile uint16_t s_key_release_count = 0U;
static uint16_t s_key_vote_id = 0U;
static uint16_t s_key_vote_count = 0U;
static uint16_t s_key_vote_age = 0U;
#if PSU_ENABLE_HT16K33_DISPLAY
static uint16_t s_led_last_voltage_mv = 0xFFFFU;
static uint16_t s_led_last_current_ma = 0xFFFFU;
static time_gt s_ht16k33_display_test_start_tick = 0U;
#endif
static uint16_t s_oled_last_voltage_mv = 0xFFFFU;
static uint16_t s_oled_last_current_ma = 0xFFFFU;
static uint16_t s_oled_last_voltage_meas_mv = 0xFFFFU;
static uint16_t s_oled_last_current_meas_ma = 0xFFFFU;
static uint16_t s_oled_last_output_state = 0xFFFFU;
static uint16_t s_oled_last_output_fault = 0xFFFFU;
static uint16_t s_oled_last_feedback_fault = 0xFFFFU;
static uint16_t s_oled_last_mode = 0xFFFFU;
static uint16_t s_oled_last_feedback_valid = 0xFFFFU;
static uint16_t s_oled_last_using_test_data = 0xFFFFU;
static uint16_t s_oled_last_control_strategy = 0xFFFFU;
static uint16_t s_oled_last_active_preset_index = 0xFFFFU;
static uint16_t s_oled_last_policy_fault = 0xFFFFU;
static uint16_t s_oled_last_policy_fault_latched = 0xFFFFU;
static uint16_t s_oled_last_settings_result = 0xFFFFU;
static uint16_t s_key_consecutive_timeout_count = 0U;
static uint16_t s_i2c_recovery_ht_stage = 0U;

#define PSU_I2C_RECOVERY_HT_INIT_STAGE       (0U)
#define PSU_I2C_RECOVERY_HT_OSC_STAGE        (1U)
#define PSU_I2C_RECOVERY_HT_BRIGHTNESS_STAGE (2U)
#define PSU_I2C_RECOVERY_HT_DISPLAY_STAGE    (3U)

#define OLED_CLEAR_PAGE_COUNT       (8U)
#define OLED_CLEAR_PAGE_BYTES       (128U)
#define OLED_INIT_RETRY_FAST_MS     (200U)
#define OLED_INIT_RETRY_SLOW_MS     (1000U)
#define OLED_KEY_DEFER_MS            (100U)
#define HT16K33_DISPLAY_TEST_HOLD_MS (1000U)

static data_gt s_oled_blank_page[OLED_CLEAR_PAGE_BYTES] = {0U};
static data_gt s_oled_probe_command = 0xAEU;

static bool power_ui_tick_due(time_gt deadline)
{
    time_gt now = gmp_base_get_system_tick();

    return ((int32_t)(now - deadline) >= 0);
}

static bool power_ui_oled_action_due(void)
{
    // Signed subtraction is wrap-safe for these sub-second deadlines.
    return power_ui_tick_due(g_oled_next_action_tick);
}

static bool power_ui_shared_i2c_ready(void)
{
    return (g_i2c_fault_active == 0U) &&
           (g_i2c_recovery_state == PSU_I2C_RECOVERY_IDLE) &&
           (g_key_scan_ready == 1U) &&
           (g_key_consecutive_ok_count >= 4U);
}

static uint32_t power_ui_oled_retry_delay_ms(void)
{
    return (g_oled_init_retry_count <= 3U) ?
               OLED_INIT_RETRY_FAST_MS : OLED_INIT_RETRY_SLOW_MS;
}

static bool power_ui_defer_oled_for_key(void)
{
    if ((g_last_raw_key_id == 0U) &&
        (s_last_key_id == 0U) &&
        (g_key_candidate_id == 0U) &&
        (g_key_candidate_count == 0U) &&
        (s_key_release_count == 0U))
    {
        return false;
    }

    ++g_oled_deferred_for_key_count;
    g_oled_next_action_tick =
        gmp_base_get_system_tick() + (time_gt)OLED_KEY_DEFER_MS;
    return true;
}

static bool power_ui_oled_i2c_step_ready(void)
{
    if (!power_ui_shared_i2c_ready())
    {
        return false;
    }

    if (!power_ui_oled_action_due())
    {
        return false;
    }

    return !power_ui_defer_oled_for_key();
}

static void power_ui_record_oled_action_result(ec_gt result)
{
    g_oled_probe_result = result;
    g_oled_last_result = result;
    // A complete checked operation can contain multiple FIFO-safe transfers.
    // Protect the key reader once, and only after the whole operation succeeds.
    if (result == GMP_EC_OK)
    {
        g_key_i2c_holdoff_count = 1U;
    }
}

static void power_ui_schedule_oled_retry(uint32_t delay_ms)
{
    g_oled_dynamic_update_enabled = 0U;
    g_oled_pending_mask = 0U;
    g_oled_clear_page_index = 0U;
    g_oled_selected_address = 0U;
    oled_set_active_address(0U);
    g_oled_init_state = OLED_INIT_RETRY_WAIT;
    g_oled_next_action_tick =
        gmp_base_get_system_tick() + (time_gt)delay_ms;
    s_oled_last_voltage_mv = 0xFFFFU;
    s_oled_last_current_ma = 0xFFFFU;
    s_oled_last_voltage_meas_mv = 0xFFFFU;
    s_oled_last_current_meas_ma = 0xFFFFU;
    s_oled_last_output_state = 0xFFFFU;
    s_oled_last_output_fault = 0xFFFFU;
    s_oled_last_feedback_fault = 0xFFFFU;
    s_oled_last_mode = 0xFFFFU;
    s_oled_last_feedback_valid = 0xFFFFU;
    s_oled_last_using_test_data = 0xFFFFU;
    s_oled_last_active_preset_index = 0xFFFFU;
}

static void power_ui_handle_oled_failure_with_delay(ec_gt result,
                                                    uint32_t delay_ms)
{
    ++g_oled_probe_error_count;
    ++g_oled_init_failure_count;
    if (g_oled_init_retry_count < 0xFFFFU)
    {
        ++g_oled_init_retry_count;
    }

    power_ui_schedule_oled_retry(delay_ms);
}

static void power_ui_handle_oled_failure(ec_gt result)
{
    power_ui_handle_oled_failure_with_delay(
        result, power_ui_oled_retry_delay_ms());
}

static ec_gt power_ui_probe_oled_address(uint16_t address)
{
    ec_gt result;

    g_oled_last_slave_address = address;
    g_oled_last_control_byte = 0x00U;
    g_oled_last_payload_length = 1U;
    g_oled_last_addr_length = 1U;
    result = gmp_hal_iic_write_mem(
        iic_bus, address, 0x00U, 1U,
        &s_oled_probe_command, 1U, (time_gt)OLED_I2C_TIMEOUT_TICKS);
    if (result == GMP_EC_TIMEOUT)
    {
        ++g_oled_timeout_count;
    }

    return result;
}

static void power_ui_prepare_oled_address_scan(void)
{
    g_oled_selected_address = 0U;
    g_oled_probe_3c_result = GMP_EC_NOT_READY;
    g_oled_probe_3d_result = GMP_EC_NOT_READY;
    g_oled_position_result = GMP_EC_NOT_READY;
    g_oled_data_result = GMP_EC_NOT_READY;
    g_oled_fail_stage = OLED_FAIL_STAGE_NONE;
    g_oled_fail_x = 0U;
    g_oled_fail_page = 0U;
    g_oled_fail_length = 0U;
    g_oled_clear_page_index = 0U;
    g_oled_pending_mask = 0U;
    oled_set_active_address(0U);
    oled_reset_init_sequence();
}

static void power_ui_run_oled_probe_3c(void)
{
    ec_gt result;

    power_ui_prepare_oled_address_scan();
    g_oled_init_state = OLED_INIT_PROBE_3C;
    ++g_oled_address_scan_count;
    result = power_ui_probe_oled_address(OLED_IIC_7BIT_ADDR_3C);
    g_oled_probe_3c_result = result;
    power_ui_record_oled_action_result(result);

    if (result == GMP_EC_OK)
    {
        g_oled_selected_address = OLED_IIC_7BIT_ADDR_3C;
        oled_set_active_address(OLED_IIC_7BIT_ADDR_3C);
        oled_reset_init_sequence();
        ++g_oled_init_attempt_count;
        g_oled_probe_result = GMP_EC_OK;
        g_oled_last_result = GMP_EC_OK;
        g_oled_init_state = OLED_INIT_COMMANDS;
        return;
    }

    g_oled_init_state = OLED_INIT_PROBE_3D;
}

static void power_ui_run_oled_probe_3d(void)
{
    ec_gt result;

    g_oled_init_state = OLED_INIT_PROBE_3D;
    result = power_ui_probe_oled_address(OLED_IIC_7BIT_ADDR_3D);
    g_oled_probe_3d_result = result;
    power_ui_record_oled_action_result(result);

    if (result == GMP_EC_OK)
    {
        g_oled_selected_address = OLED_IIC_7BIT_ADDR_3D;
        oled_set_active_address(OLED_IIC_7BIT_ADDR_3D);
        oled_reset_init_sequence();
        ++g_oled_init_attempt_count;
        g_oled_probe_result = GMP_EC_OK;
        g_oled_last_result = GMP_EC_OK;
        g_oled_init_state = OLED_INIT_COMMANDS;
        return;
    }

    g_oled_probe_result = result;
    g_oled_last_result = result;
    ++g_oled_no_device_count;
    power_ui_handle_oled_failure_with_delay(
        result, OLED_INIT_RETRY_SLOW_MS);
}

static void power_ui_run_oled_commands(void)
{
    ec_gt result;

    g_oled_init_state = OLED_INIT_COMMANDS;
    result = oled_init_checked();
    power_ui_record_oled_action_result(result);
    if (result != GMP_EC_OK)
    {
        power_ui_handle_oled_failure(result);
    }
    else if (g_oled_init_command_index >= OLED_INIT_COMMAND_COUNT)
    {
        g_oled_init_state = OLED_INIT_TEST;
    }
}

static void power_ui_render_led_setpoints(bool force_update);
static void power_ui_request_led_setpoint_update(void);

static void power_ui_publish_key_vote(void)
{
    g_key_vote_id = s_key_vote_id;
    g_key_vote_count = s_key_vote_count;
    g_key_vote_age = s_key_vote_age;

    // Preserve the existing board diagnostics and OLED key-activity guard.
    g_key_candidate_id = s_key_vote_id;
    g_key_candidate_count = s_key_vote_count;
}

static void power_ui_reset_key_candidate(void)
{
    s_key_vote_id = 0U;
    s_key_vote_count = 0U;
    s_key_vote_age = 0U;
    power_ui_publish_key_vote();
}

static void power_ui_start_key_vote(uint16_t key_id)
{
    s_key_vote_id = key_id;
    s_key_vote_count = 1U;
    s_key_vote_age = 1U;
    power_ui_publish_key_vote();
}

static void power_ui_handle_key_read_error(void)
{
    // A failed read is neither a vote-window sample nor a release sample.
    s_key_release_count = 0U;
}

static void power_ui_schedule_i2c_recovery_retry(void)
{
    ++g_i2c_recovery_failure_count;
    g_i2c_recovery_verify_ok_count = 0U;
    g_key_consecutive_ok_count = 0U;
    s_i2c_recovery_ht_stage = PSU_I2C_RECOVERY_HT_INIT_STAGE;
    g_i2c_recovery_next_tick =
        gmp_base_get_system_tick() + (time_gt)PSU_I2C_RECOVERY_RETRY_MS;
    g_i2c_recovery_state = PSU_I2C_RECOVERY_WAIT;
}

static void power_ui_schedule_i2c_clear_retry(void)
{
    ++g_i2c_clear_failure_count;
    g_i2c_recovery_verify_ok_count = 0U;
    g_key_consecutive_ok_count = 0U;
    s_i2c_recovery_ht_stage = PSU_I2C_RECOVERY_HT_INIT_STAGE;
    g_i2c_recovery_next_tick =
        gmp_base_get_system_tick() + (time_gt)PSU_I2C_CLEAR_RETRY_MS;
    g_i2c_recovery_state = PSU_I2C_RECOVERY_WAIT;
}

static void power_ui_begin_i2c_recovery(void)
{
    g_i2c_sda_level_before = board_i2c_read_sda_level();
    g_i2c_scl_level_before = board_i2c_read_scl_level();
    g_i2c_fault_active = 1U;
    g_i2c_recovery_state = PSU_I2C_RECOVERY_WAIT;
    g_i2c_recovery_next_tick =
        gmp_base_get_system_tick() +
        (time_gt)PSU_I2C_RECOVERY_INITIAL_DELAY_MS;
    g_i2c_recovery_ht_init_result = GMP_EC_NOT_READY;
    g_i2c_recovery_verify_result = GMP_EC_NOT_READY;
    g_i2c_recovery_verify_ok_count = 0U;
    g_key_scan_ready = 0U;
    g_key_consecutive_ok_count = 0U;
    g_oled_dynamic_update_enabled = 0U;
    s_i2c_recovery_ht_stage = PSU_I2C_RECOVERY_HT_INIT_STAGE;
    s_last_key_id = 0U;
    s_key_release_count = 0U;
    power_ui_reset_key_candidate();
}

static void power_ui_advance_i2c_recovery(ht16k33_dev_t* dev)
{
    ec_gt result;
    fast_gt key_id = 0U;
    board_i2c_clear_result_t clear_result;

    switch (g_i2c_recovery_state)
    {
    case PSU_I2C_RECOVERY_WAIT:
        if (power_ui_tick_due(g_i2c_recovery_next_tick))
        {
            board_i2c_bus_clear_begin();
            g_i2c_recovery_state = PSU_I2C_RECOVERY_BUS_CLEAR;
        }
        return;

    case PSU_I2C_RECOVERY_BUS_CLEAR:
        clear_result = board_i2c_bus_clear_step();
        if (clear_result == BOARD_I2C_CLEAR_IN_PROGRESS)
        {
            return;
        }

        if ((clear_result == BOARD_I2C_CLEAR_RELEASED) &&
            (g_i2c_clear_sda_final != 0U) &&
            (g_i2c_clear_scl_final != 0U) &&
            (g_i2c_clear_stop_generated != 0U) &&
            (g_i2c_clear_pinmux_restored != 0U))
        {
            ++g_i2c_clear_success_count;
            g_i2c_recovery_next_tick = gmp_base_get_system_tick();
            g_i2c_recovery_state = PSU_I2C_RECOVERY_RESET_MODULE;
            return;
        }

        power_ui_schedule_i2c_clear_retry();
        return;

    case PSU_I2C_RECOVERY_RESET_MODULE:
        if (!power_ui_tick_due(g_i2c_recovery_next_tick))
        {
            return;
        }

        ++g_i2c_recovery_attempt_count;
        board_i2c_controller_reinit();
        g_i2c_sda_level_after = board_i2c_read_sda_level();
        g_i2c_scl_level_after = board_i2c_read_scl_level();
        g_i2c_recovery_next_tick =
            gmp_base_get_system_tick() +
            (time_gt)PSU_I2C_RECOVERY_SETTLE_MS;
        s_i2c_recovery_ht_stage = PSU_I2C_RECOVERY_HT_INIT_STAGE;
        g_i2c_recovery_state = PSU_I2C_RECOVERY_REINIT_HT;
        return;

    case PSU_I2C_RECOVERY_REINIT_HT:
        if (!power_ui_tick_due(g_i2c_recovery_next_tick))
        {
            return;
        }

        if (s_i2c_recovery_ht_stage == PSU_I2C_RECOVERY_HT_INIT_STAGE)
        {
            ht16k33_init_t init_cfg = {
                .brightness = 15,
                .blink_rate = 0,
                .int_enable = 0,
                .int_act_high = 0
            };

            result = ht16k33_init(
                dev, iic_bus, HT16K33_DEFAULT_DEV_ADDR, &init_cfg);
            g_i2c_recovery_ht_init_result = result;
            if (result != GMP_EC_OK)
            {
                power_ui_schedule_i2c_recovery_retry();
                return;
            }
            s_i2c_recovery_ht_stage = PSU_I2C_RECOVERY_HT_OSC_STAGE;
            return;
        }

        if (s_i2c_recovery_ht_stage == PSU_I2C_RECOVERY_HT_OSC_STAGE)
        {
            result = gmp_hal_iic_write_cmd(
                dev->bus, dev->dev_addr,
                HT16K33_CMD_OSC_ON, 1U, HT16K33_CFG_TIMEOUT);
            g_ht16k33_osc_on_result = result;
            if (result != GMP_EC_OK)
            {
                power_ui_schedule_i2c_recovery_retry();
                return;
            }
            s_i2c_recovery_ht_stage = PSU_I2C_RECOVERY_HT_BRIGHTNESS_STAGE;
            return;
        }

        if (s_i2c_recovery_ht_stage == PSU_I2C_RECOVERY_HT_BRIGHTNESS_STAGE)
        {
            result = gmp_hal_iic_write_cmd(
                dev->bus, dev->dev_addr,
                HT16K33_REG_BRIGHTNESS | 0x0FU, 1U,
                HT16K33_CFG_TIMEOUT);
            g_ht16k33_brightness_result = result;
            if (result != GMP_EC_OK)
            {
                power_ui_schedule_i2c_recovery_retry();
                return;
            }
            s_i2c_recovery_ht_stage = PSU_I2C_RECOVERY_HT_DISPLAY_STAGE;
            return;
        }

        result = gmp_hal_iic_write_cmd(
            dev->bus, dev->dev_addr,
            HT16K33_CMD_DISPLAY_ON, 1U, HT16K33_CFG_TIMEOUT);
        g_ht16k33_display_on_result = result;
        if (result != GMP_EC_OK)
        {
            power_ui_schedule_i2c_recovery_retry();
            return;
        }

        dev->last_key = 0U;
        g_key_scan_ready = 0U;
        s_last_key_id = 0U;
        s_key_release_count = 0U;
        g_key_ignore_scan_count = 0U;
        power_ui_reset_key_candidate();
        g_i2c_recovery_verify_result = GMP_EC_NOT_READY;
        g_i2c_recovery_verify_ok_count = 0U;
        g_key_consecutive_ok_count = 0U;
        g_i2c_recovery_next_tick =
            gmp_base_get_system_tick() +
            (time_gt)PSU_I2C_RECOVERY_VERIFY_PERIOD_MS;
        g_i2c_recovery_state = PSU_I2C_RECOVERY_VERIFY;
        return;

    case PSU_I2C_RECOVERY_VERIFY:
        if (!power_ui_tick_due(g_i2c_recovery_next_tick))
        {
            return;
        }

        dev->last_key = 0U;
        result = ht16k33_read_keys(dev, &key_id);
        g_i2c_recovery_verify_result = result;
        if (result != GMP_EC_OK)
        {
            power_ui_schedule_i2c_recovery_retry();
            return;
        }

        if (g_key_consecutive_ok_count < 0xFFFFU)
        {
            ++g_key_consecutive_ok_count;
        }

        if (key_id == 0U)
        {
            if (g_i2c_recovery_verify_ok_count < 0xFFFFU)
            {
                ++g_i2c_recovery_verify_ok_count;
            }
        }
        else
        {
            g_i2c_recovery_verify_ok_count = 0U;
        }

        if ((key_id == 0U) &&
            (g_i2c_recovery_verify_ok_count >=
             PSU_I2C_RECOVERY_VERIFY_OK_COUNT))
        {
            g_last_raw_key_id = 0U;
            g_key_consecutive_error_count = 0U;
            s_key_consecutive_timeout_count = 0U;
            g_key_scan_ready = 1U;
            g_i2c_fault_active = 0U;
            g_i2c_recovery_state = PSU_I2C_RECOVERY_COMPLETE;
            ++g_i2c_recovery_success_count;
            return;
        }

        g_i2c_recovery_next_tick =
            gmp_base_get_system_tick() +
            (time_gt)PSU_I2C_RECOVERY_VERIFY_PERIOD_MS;
        return;

    case PSU_I2C_RECOVERY_COMPLETE:
        power_ui_render_led_setpoints(true);
        dev->is_dirty = 1U;
        g_ht16k33_display_test_state = HT16K33_DISPLAY_TEST_NORMAL;
        g_i2c_recovery_state = PSU_I2C_RECOVERY_IDLE;
        return;

    case PSU_I2C_RECOVERY_IDLE:
    default:
        return;
    }
}

static bool power_ui_is_mapped_key(uint16_t key_id)
{
    if (((PSU_KEY_PRESET_1_ID != 0U) &&
         (key_id == PSU_KEY_PRESET_1_ID)) ||
        ((PSU_KEY_PRESET_2_ID != 0U) &&
         (key_id == PSU_KEY_PRESET_2_ID)) ||
        ((PSU_KEY_PRESET_3_ID != 0U) &&
         (key_id == PSU_KEY_PRESET_3_ID)))
    {
        return true;
    }

    switch (key_id)
    {
    case PSU_KEY_VOLTAGE_UP_ID:
    case PSU_KEY_VOLTAGE_DOWN_ID:
    case PSU_KEY_CURRENT_UP_ID:
    case PSU_KEY_CURRENT_DOWN_ID:
    case PSU_KEY_OUTPUT_TOGGLE_ID:
    case PSU_KEY_FAULT_RESET_ID:
    case PSU_KEY_CONTROL_STRATEGY_ID:
        return true;

    default:
        return false;
    }
}

static bool power_ui_setpoint_change_allowed(void)
{
    return !g_power_app.fault_latched
#if PSU_REAL_FEEDBACK_CONNECTED
           &&
           (g_analog_board_feedback_fault == 0U) &&
           (g_analog_board_fault_hold_active == 0U) &&
           (g_analog_board_feedback_settled == 1U)
#endif
           ;
}

static uint16_t power_ui_voltage_limit_mv(void)
{
    return PSU_COMMAND_VOLTAGE_LIMIT_MV;
}

static uint16_t power_ui_current_limit_ma(void)
{
    return PSU_COMMAND_CURRENT_LIMIT_MA;
}

static bool power_ui_execute_key_action(uint16_t key_id)
{
    uint16_t current;
    uint16_t limit;
    uint16_t minimum;
    uint32_t next;
    psu_control_strategy_t strategy;
    psu_control_strategy_t next_strategy;
    bool action_executed = true;

    g_settings_result = PSU_SETTINGS_RESULT_IDLE;

    if ((PSU_KEY_PRESET_1_ID != 0U) &&
        (key_id == PSU_KEY_PRESET_1_ID))
    {
        action_executed = power_presets_select(0U);
        if (action_executed)
        {
            power_ui_request_led_setpoint_update();
        }
        return action_executed;
    }
    if ((PSU_KEY_PRESET_2_ID != 0U) &&
        (key_id == PSU_KEY_PRESET_2_ID))
    {
        action_executed = power_presets_select(1U);
        if (action_executed)
        {
            power_ui_request_led_setpoint_update();
        }
        return action_executed;
    }
    if ((PSU_KEY_PRESET_3_ID != 0U) &&
        (key_id == PSU_KEY_PRESET_3_ID))
    {
        action_executed = power_presets_select(2U);
        if (action_executed)
        {
            power_ui_request_led_setpoint_update();
        }
        return action_executed;
    }

    if (((key_id == PSU_KEY_VOLTAGE_UP_ID) ||
         (key_id == PSU_KEY_VOLTAGE_DOWN_ID)) &&
        (g_voltage_adjust_locked != 0U))
    {
        ++g_locked_adjust_reject_count;
        return false;
    }

    if (((key_id == PSU_KEY_CURRENT_UP_ID) ||
         (key_id == PSU_KEY_CURRENT_DOWN_ID)) &&
        (g_current_adjust_locked != 0U))
    {
        ++g_locked_adjust_reject_count;
        return false;
    }

    if (((key_id == PSU_KEY_VOLTAGE_UP_ID) ||
         (key_id == PSU_KEY_VOLTAGE_DOWN_ID) ||
         (key_id == PSU_KEY_CURRENT_UP_ID) ||
         (key_id == PSU_KEY_CURRENT_DOWN_ID)) &&
        !power_ui_setpoint_change_allowed())
    {
        return false;
    }

    switch (key_id)
    {
    case PSU_KEY_VOLTAGE_UP_ID:
        current = power_app_get_voltage_mv();
        limit = power_ui_voltage_limit_mv();
        next = (uint32_t)current + PSU_VOLTAGE_STEP_MV;
        if (next > limit)
        {
            next = limit;
        }
        power_app_set_voltage_mv((uint16_t)next);
        power_ui_request_led_setpoint_update();
        break;

    case PSU_KEY_VOLTAGE_DOWN_ID:
        current = power_app_get_voltage_mv();
        power_app_set_voltage_mv((current < PSU_VOLTAGE_STEP_MV) ?
                                     0U : (uint16_t)(current - PSU_VOLTAGE_STEP_MV));
        power_ui_request_led_setpoint_update();
        break;

    case PSU_KEY_CURRENT_UP_ID:
        current = power_app_get_current_ma();
        limit = power_ui_current_limit_ma();
        next = (uint32_t)current + PSU_CURRENT_STEP_MA;
        if (next > limit)
        {
            next = limit;
        }
        power_app_set_current_ma((uint16_t)next);
        power_ui_request_led_setpoint_update();
        break;

    case PSU_KEY_CURRENT_DOWN_ID:
        current = power_app_get_current_ma();
        minimum = 0U;
#if PSU_ENABLE_LOW_RANGE_BRINGUP_LIMITS
        if (power_app_get_voltage_mv() > 0U)
        {
            minimum = PSU_ANALOG_BOARD_MIN_CURRENT_MA;
        }
#endif
        if (current < minimum)
        {
            current = minimum;
        }
        power_app_set_current_ma(
            ((uint32_t)PSU_CURRENT_STEP_MA >
             ((uint32_t)current - (uint32_t)minimum)) ?
                minimum : (uint16_t)(current - PSU_CURRENT_STEP_MA));
        power_ui_request_led_setpoint_update();
        break;

    case PSU_KEY_OUTPUT_TOGGLE_ID:
#if PSU_ENABLE_LOGICAL_OUTPUT_SWITCH
        power_app_request_logical_output(g_output_switch_requested == 0U);
#elif PSU_SAFE_BRINGUP || !PSU_ALLOW_OUTPUT_REQUEST
        power_app_request_output(true);
#else
        if (!g_power_app.fault_latched)
        {
            power_app_request_output(!g_power_app.output_requested);
        }
#endif
        break;

    case PSU_KEY_FAULT_RESET_ID:
        power_app_reset_fault();
        power_control_policy_request_reset();
        break;

    case PSU_KEY_CONTROL_STRATEGY_ID:
        strategy = power_control_policy_get_strategy();
        if (strategy == PSU_CONTROL_STRATEGY_AUTO)
        {
            next_strategy = PSU_CONTROL_STRATEGY_CV_ONLY;
        }
        else if (strategy == PSU_CONTROL_STRATEGY_CV_ONLY)
        {
            next_strategy = PSU_CONTROL_STRATEGY_CC_ONLY;
        }
        else
        {
            next_strategy = PSU_CONTROL_STRATEGY_AUTO;
        }
        (void)power_control_policy_set_strategy(next_strategy);
        break;

    default:
        action_executed = false;
        break;
    }

    return action_executed;
}

static bool power_ui_process_key_id(uint16_t key_id)
{
    bool action_executed;
    bool mapped_key = false;

    if (key_id != 0U)
    {
        mapped_key = power_ui_is_mapped_key(key_id);
        if (!mapped_key)
        {
            g_last_unmapped_key_id = key_id;
            ++g_unmapped_confirmed_count;
        }
    }

    // Ignore every nonzero report until the configured consecutive idle scans
    // prove that the shared I2C bus and key matrix started cleanly.
    if (g_key_scan_ready == 0U)
    {
        power_ui_reset_key_candidate();

        if (key_id == 0U)
        {
            if (s_key_release_count < PSU_KEY_RELEASE_FILTER_COUNT)
            {
                ++s_key_release_count;
            }
            if (s_key_release_count >= PSU_KEY_RELEASE_FILTER_COUNT)
            {
                s_key_release_count = 0U;
                s_last_key_id = 0U;
                g_key_scan_ready = 1U;
                power_ui_request_led_setpoint_update();
            }
        }
        else
        {
            s_key_release_count = 0U;
        }

        return false;
    }

    // A confirmed key remains latched until the configured consecutive zero
    // reports. Other nonzero IDs cannot bypass the release requirement.
    if (s_last_key_id != 0U)
    {
        power_ui_reset_key_candidate();

        if (key_id == 0U)
        {
            if (s_key_release_count < PSU_KEY_RELEASE_FILTER_COUNT)
            {
                ++s_key_release_count;
            }
            if (s_key_release_count >= PSU_KEY_RELEASE_FILTER_COUNT)
            {
                s_last_key_id = 0U;
                s_key_release_count = 0U;
            }
        }
        else
        {
            s_key_release_count = 0U;
        }

        return false;
    }

    s_key_release_count = 0U;

    if (s_key_vote_id == 0U)
    {
        if ((key_id != 0U) && mapped_key)
        {
            power_ui_start_key_vote(key_id);
        }
        return false;
    }

    // A different valid ID starts a fresh three-scan window. Zero and
    // unmapped samples leave the valid candidate in place.
    if ((key_id != 0U) && mapped_key && (key_id != s_key_vote_id))
    {
        power_ui_start_key_vote(key_id);
        return false;
    }

    if (s_key_vote_age < PSU_KEY_VOTE_WINDOW_COUNT)
    {
        ++s_key_vote_age;
    }

    if ((key_id == s_key_vote_id) &&
        (s_key_vote_count < PSU_KEY_VOTE_REQUIRED_COUNT))
    {
        ++s_key_vote_count;
    }

    if (s_key_vote_count >= PSU_KEY_VOTE_REQUIRED_COUNT)
    {
        key_id = s_key_vote_id;
        s_last_key_id = key_id;
        g_last_confirmed_key_id = key_id;
        power_ui_reset_key_candidate();
        ++g_key_confirmed_count;

        action_executed = power_ui_execute_key_action(key_id);
        if (action_executed)
        {
            ++g_key_action_count;
            ++g_key_first_sample_action_count;
        }

        return action_executed;
    }

    if (s_key_vote_age >= PSU_KEY_VOTE_WINDOW_COUNT)
    {
        ++g_key_vote_timeout_count;
        power_ui_reset_key_candidate();
        return false;
    }

    power_ui_publish_key_vote();
    return false;
}

bool power_ui_safe_bringup_self_test(void)
{
#if PSU_SAFE_BRINGUP
    uint16_t key_id;
    bool passed = true;

    for (key_id = 1U; key_id <= 39U; ++key_id)
    {
        g_power_app.output_requested = false;
        (void)power_ui_execute_key_action(key_id);
        if (g_power_app.output_requested)
        {
            passed = false;
        }
    }

    power_app_set_voltage_mv(0U);
    power_app_set_current_ma(0U);
    g_power_app.output_requested = false;
    g_power_app.fault_reset_requested = false;
    s_last_key_id = 0U;
    s_key_release_count = 0U;
    power_ui_reset_key_candidate();
    g_key_scan_ready = 0U;
    g_key_ignore_scan_count = 0U;
#if PSU_ENABLE_HT16K33_DISPLAY
    s_led_last_voltage_mv = 0xFFFFU;
    s_led_last_current_ma = 0xFFFFU;
#endif
    return passed;
#else
    return true;
#endif
}

#if !PSU_SAFE_BRINGUP
static const char *power_ui_state_text(power_state_t state)
{
    switch (state)
    {
    case POWER_STATE_OFF:
        return "OFF";
    case POWER_STATE_STARTING:
        return "START";
    case POWER_STATE_CV:
        return "CV";
    case POWER_STATE_CC:
        return "CC";
    case POWER_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static const char *power_ui_fault_text(power_fault_t fault)
{
    switch (fault)
    {
    case POWER_FAULT_NONE:
        return "NONE";
    case POWER_FAULT_OVERVOLTAGE:
        return "OVP";
    case POWER_FAULT_OVERCURRENT:
        return "OCP";
    default:
        return "UNK";
    }
}

#endif

static uint16_t power_ui_display_current(uint16_t current_ma)
{
    return (current_ma > 999U) ? 999U : current_ma;
}

//
// Common cathode digital tube segment code table
//
const unsigned char led_lut[] = {
    0x3F, // 0  (a,b,c,d,e,f)
    0x06, // 1  (b,c)
    0x5B, // 2  (a,b,d,e,g)
    0x4F, // 3  (a,b,c,d,g)
    0x66, // 4  (b,c,f,g)
    0x6D, // 5  (a,c,d,f,g)
    0x7D, // 6  (a,c,d,e,f,g)
    0x07, // 7  (a,b,c)
    0x7F, // 8  (a,b,c,d,e,f,g)
    0x6F, // 9  (a,b,c,d,f,g)
    0x77, // A  (a,b,c,e,f,g)
    0x7C, // b  (c,d,e,f,g)
    0x39, // C  (a,d,e,f)
    0x5E, // d  (b,c,d,e,g)
    0x79, // E  (a,d,e,f,g)
    0x71, // F  (a,e,f,g)
    0x76, // H  (b,c,e,f,g)
    0x38, // L  (d,e,f)
    0x73, // P  (a,b,e,f,g)
    0x3E, // U  (b,c,d,e,f)
    0x40, // -  (g) - dash
    0x80, // .  (dp) - dot
    0x00  // close all
};

void update_led_content_8byte(ht16k33_dev_t* dev, uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5,
                               uint16_t ch6, uint16_t ch7, uint16_t ch8)
{
    uint16_t i;

    // Normal rendering owns the complete 16-byte shadow. In particular, clear
    // the odd RAM addresses left set by the startup all-on test before writing
    // the eight board-specific digit bytes.
    for (i = 0U; i < HT16K33_CFG_DISP_RAM_SIZE; ++i)
    {
        dev->display_ram[i] = 0U;
    }

    dev->display_ram[0] = ch1;
    dev->display_ram[2] = ch2;
    dev->display_ram[4] = ch3;
    dev->display_ram[6] = ch4;
    dev->display_ram[8] = ch5;
    dev->display_ram[10] = ch6;
    dev->display_ram[12] = ch7;
    dev->display_ram[14] = ch8;
    dev->is_dirty = 1;
}

static void power_ui_render_led_setpoints(bool force_update)
{
#if !PSU_ENABLE_HT16K33_DISPLAY
    GMP_UNUSED_VAR(force_update);
    return;
#else
    uint16_t voltage_set_mv = g_power_app.voltage_set_mv;
    uint16_t current_set_ma = g_power_app.current_set_ma;
    uint32_t display_voltage_mv = voltage_set_mv;
    uint16_t display_current_ma = current_set_ma;

    if ((!force_update) &&
        (voltage_set_mv == s_led_last_voltage_mv) &&
        (current_set_ma == s_led_last_current_ma))
    {
        return;
    }

    if (display_voltage_mv > 99999U)
    {
        display_voltage_mv = 99999U;
    }
    if (display_current_ma > 999U)
    {
        display_current_ma = 999U;
    }

    // Fixed-width setpoint display: five voltage digits followed by three
    // current digits. Each led_lut index is constrained to the decimal range.
    update_led_content_8byte(
        &ht16k33,
        led_lut[(display_voltage_mv / 10000U) % 10U],
        led_lut[(display_voltage_mv / 1000U) % 10U],
        led_lut[(display_voltage_mv / 100U) % 10U],
        led_lut[(display_voltage_mv / 10U) % 10U],
        led_lut[display_voltage_mv % 10U],
        led_lut[(display_current_ma / 100U) % 10U],
        led_lut[(display_current_ma / 10U) % 10U],
        led_lut[display_current_ma % 10U]);

    s_led_last_voltage_mv = voltage_set_mv;
    s_led_last_current_ma = current_set_ma;
#endif
}

static void power_ui_request_led_setpoint_update(void)
{
    power_ui_render_led_setpoints(false);
}

void power_ui_request_led_setpoint_update_from_command(void)
{
    power_ui_request_led_setpoint_update();
}

gmp_task_status_t tsk_LED_flush(gmp_task_t* tsk)
{
#if !PSU_ENABLE_HT16K33_DISPLAY
    GMP_UNUSED_VAR(tsk);
    return GMP_TASK_DONE;
#else
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;

    if(flag_init_cmpt)
    {
        uint16_t display_test_state;
        uint16_t i;
        ec_gt ret;

        if ((g_i2c_fault_active != 0U) ||
            (g_i2c_recovery_state != PSU_I2C_RECOVERY_IDLE) ||
            (g_key_scan_ready == 0U))
        {
            return GMP_TASK_DONE;
        }

        display_test_state = g_ht16k33_display_test_state;

        if (display_test_state == HT16K33_DISPLAY_TEST_PREPARE_ALL)
        {
            for (i = 0U; i < HT16K33_CFG_DISP_RAM_SIZE; ++i)
            {
                dev->display_ram[i] = 0xFFU;
            }
            dev->is_dirty = 1;
        }
        else if (display_test_state == HT16K33_DISPLAY_TEST_HOLD_ALL)
        {
            if ((time_gt)(gmp_base_get_system_tick() -
                          s_ht16k33_display_test_start_tick) <
                (time_gt)HT16K33_DISPLAY_TEST_HOLD_MS)
            {
                return GMP_TASK_DONE;
            }

            g_ht16k33_display_test_state = HT16K33_DISPLAY_TEST_RESTORE;
            display_test_state = HT16K33_DISPLAY_TEST_RESTORE;
        }

        if (display_test_state == HT16K33_DISPLAY_TEST_RESTORE)
        {
            power_ui_render_led_setpoints(true);
        }
        else if (display_test_state == HT16K33_DISPLAY_TEST_NORMAL)
        {
            power_ui_request_led_setpoint_update();
        }
        else if (display_test_state != HT16K33_DISPLAY_TEST_PREPARE_ALL)
        {
            return GMP_TASK_DONE;
        }

        if (dev->is_dirty == 0)
        {
            return GMP_TASK_DONE;
        }

        ret = ht16k33_update_display(dev);
        g_led_update_result = ret;

        if (ret != GMP_EC_OK)
        {
            dev->is_dirty = 1;
            if (display_test_state == HT16K33_DISPLAY_TEST_PREPARE_ALL)
            {
                g_ht16k33_all_on_result = ret;
            }
            return GMP_TASK_DONE;
        }

        g_key_ignore_scan_count = 1U;
        ++g_led_update_count;

        if (display_test_state == HT16K33_DISPLAY_TEST_PREPARE_ALL)
        {
            g_ht16k33_all_on_result = GMP_EC_OK;
            ++g_ht16k33_display_test_count;
            s_ht16k33_display_test_start_tick =
                gmp_base_get_system_tick();
            g_ht16k33_display_test_state =
                HT16K33_DISPLAY_TEST_HOLD_ALL;
        }
        else if (display_test_state == HT16K33_DISPLAY_TEST_RESTORE)
        {
            g_ht16k33_display_test_state =
                HT16K33_DISPLAY_TEST_NORMAL;
        }
    }

    return GMP_TASK_DONE;
#endif
}

gmp_task_status_t tsk_key_flush(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;
    fast_gt key_id = 0;
    bool action_executed;

    if(flag_init_cmpt)
    {
        ec_gt ret;

        if ((g_i2c_fault_active != 0U) ||
            (g_i2c_recovery_state != PSU_I2C_RECOVERY_IDLE))
        {
            power_ui_advance_i2c_recovery(dev);
            ++g_i2c_key_backoff_skip_count;
            return GMP_TASK_DONE;
        }

        // A successful OLED step may contain several FIFO-safe transactions.
        // Skip one 20 ms key period before accessing HT16K33 again.
        if (g_key_i2c_holdoff_count != 0U)
        {
            --g_key_i2c_holdoff_count;
            ++g_key_i2c_holdoff_skip_count;
            power_ui_reset_key_candidate();
            return GMP_TASK_DONE;
        }

        // The common driver suppresses repeated reports of the same key for
        // 120 ms. This board-level state machine needs the current matrix
        // state on every scheduled scan, so disable only that event cache;
        // confirmation and repeat suppression are handled below.
        dev->last_key = 0;
        ret = ht16k33_read_keys(dev, &key_id);

        g_last_raw_key_id = (uint16_t)key_id;
        g_key_read_result = ret;

        if (ret != GMP_EC_OK)
        {
            ++g_key_read_error_count;
            ++g_key_consecutive_error_count;
            g_key_consecutive_ok_count = 0U;

            if (ret == GMP_EC_TIMEOUT)
            {
                if (s_key_consecutive_timeout_count < 0xFFFFU)
                {
                    ++s_key_consecutive_timeout_count;
                }
            }
            else
            {
                s_key_consecutive_timeout_count = 0U;
            }

            power_ui_handle_key_read_error();

            if (s_key_consecutive_timeout_count >=
                PSU_I2C_TIMEOUT_TRIGGER_COUNT)
            {
                ++g_oled_disabled_due_i2c_count;
                power_ui_begin_i2c_recovery();
            }

            return GMP_TASK_DONE;
        }

        ++g_key_read_ok_count;
        g_key_consecutive_error_count = 0U;
        s_key_consecutive_timeout_count = 0U;

        if (g_key_consecutive_ok_count < 0xFFFFU)
        {
            ++g_key_consecutive_ok_count;
        }
        if (g_key_ignore_scan_count != 0U)
        {
            power_ui_reset_key_candidate();
            --g_key_ignore_scan_count;
            return GMP_TASK_DONE;
        }

        action_executed = power_ui_process_key_id((uint16_t)key_id);

#if PSU_ENABLE_KEY_EVENT_LOG
        if (action_executed && (key_id != 0U))
        {
            gmp_base_print("KEY %u V=%u I=%u\r\n",
                           (unsigned int)key_id,
                           (unsigned int)g_power_app.voltage_set_mv,
                           (unsigned int)g_power_app.current_set_ma);
        }
#endif
    }

    return GMP_TASK_DONE;
}

gmp_task_status_t oled_show_task(gmp_task_t* tsk)
{
#if PSU_USE_INCREMENTAL_OLED_UI
    char line1[20];
    char line2[20];
    char line3[20];
    char line4[20];
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    uint16_t voltage_meas_mv;
    uint16_t current_meas_ma;
    uint16_t output_state;
    uint16_t output_fault;
    uint16_t feedback_fault;
    uint16_t display_mode;
    uint16_t feedback_valid;
    uint16_t using_test_data;
    uint16_t control_strategy;
    uint16_t active_preset_index;
    uint16_t policy_fault;
    uint16_t policy_fault_latched;
    uint16_t settings_result;
    uint16_t hard_fault_active;
    ec_gt ret;
#endif
    GMP_UNUSED_VAR(tsk);

    if ((flag_init_cmpt != 1U) || !power_ui_shared_i2c_ready())
    {
        return GMP_TASK_DONE;
    }

#if PSU_USE_INCREMENTAL_OLED_UI
    switch (g_oled_init_state)
    {
    case OLED_INIT_WAIT_POWER:
        if (power_ui_oled_i2c_step_ready())
        {
            power_ui_run_oled_probe_3c();
        }
        return GMP_TASK_DONE;

    case OLED_INIT_PROBE_3C:
        if (power_ui_oled_i2c_step_ready())
        {
            power_ui_run_oled_probe_3c();
        }
        return GMP_TASK_DONE;

    case OLED_INIT_PROBE_3D:
        if (power_ui_oled_i2c_step_ready())
        {
            power_ui_run_oled_probe_3d();
        }
        return GMP_TASK_DONE;

    case OLED_INIT_COMMANDS:
        if (power_ui_oled_i2c_step_ready())
        {
            power_ui_run_oled_commands();
        }
        return GMP_TASK_DONE;

    case OLED_INIT_TEST:
        if (!power_ui_oled_i2c_step_ready())
        {
            return GMP_TASK_DONE;
        }
        ret = oled_show_line_checked(0U, 0U, "TEST");
        power_ui_record_oled_action_result(ret);
        if (ret == GMP_EC_OK)
        {
            g_oled_clear_page_index = 0U;
            g_oled_init_state = OLED_INIT_CLEAR_PAGE;
        }
        else
        {
            power_ui_handle_oled_failure(ret);
        }
        return GMP_TASK_DONE;

    case OLED_INIT_CLEAR_PAGE:
        if (g_oled_clear_page_index >= OLED_CLEAR_PAGE_COUNT)
        {
            g_oled_init_state = OLED_INIT_TITLE;
            return GMP_TASK_DONE;
        }

        if (!power_ui_oled_i2c_step_ready())
        {
            return GMP_TASK_DONE;
        }

        ret = oled_write_page_checked(
            0U, (uint8_t)g_oled_clear_page_index,
            s_oled_blank_page, OLED_CLEAR_PAGE_BYTES);
        power_ui_record_oled_action_result(ret);
        if (ret != GMP_EC_OK)
        {
            power_ui_handle_oled_failure(ret);
            return GMP_TASK_DONE;
        }

        ++g_oled_clear_page_index;
        if (g_oled_clear_page_index >= OLED_CLEAR_PAGE_COUNT)
        {
            g_oled_init_state = OLED_INIT_TITLE;
        }
        return GMP_TASK_DONE;

    case OLED_INIT_TITLE:
        if (!power_ui_oled_i2c_step_ready())
        {
            return GMP_TASK_DONE;
        }
        ret = oled_show_line_checked(0U, 0U, "PSU TEST");
        power_ui_record_oled_action_result(ret);
        if (ret != GMP_EC_OK)
        {
            power_ui_handle_oled_failure(ret);
            return GMP_TASK_DONE;
        }

        g_oled_dynamic_update_enabled = 1U;
        ++g_oled_probe_ok_count;
        ++g_oled_init_success_count;
        ++g_oled_line_update_count;
        g_oled_pending_mask = OLED_PENDING_VOLTAGE |
                              OLED_PENDING_OUTPUT_STATE |
                              OLED_PENDING_MODE_STATUS |
                              OLED_PENDING_FEEDBACK_STATUS;
        g_oled_init_state = OLED_INIT_READY;
        return GMP_TASK_DONE;

    case OLED_INIT_RETRY_WAIT:
        if (power_ui_oled_i2c_step_ready())
        {
            power_ui_run_oled_probe_3c();
        }
        return GMP_TASK_DONE;

    case OLED_INIT_READY:
        g_oled_dynamic_update_enabled = 1U;
        break;

    default:
        g_oled_dynamic_update_enabled = 0U;
        g_oled_pending_mask = 0U;
        g_oled_clear_page_index = 0U;
        g_oled_selected_address = 0U;
        oled_set_active_address(0U);
        g_oled_init_state = OLED_INIT_WAIT_POWER;
        g_oled_next_action_tick =
            gmp_base_get_system_tick() + (time_gt)100U;
        return GMP_TASK_DONE;
    }

    voltage_set_mv = g_power_app.voltage_set_mv;
    current_set_ma = g_power_app.current_set_ma;
    feedback_valid = g_mode_monitor_feedback_valid;
    using_test_data = g_mode_monitor_using_test_data;
    display_mode = g_mode_monitor_display_mode;
    if ((feedback_valid != 0U) && (using_test_data != 0U))
    {
        voltage_meas_mv = g_mode_monitor_test_voltage_mv;
        current_meas_ma = g_mode_monitor_test_current_ma;
    }
    else
    {
        voltage_meas_mv = g_power_app.voltage_meas_mv;
        current_meas_ma = g_power_app.current_meas_ma;
    }
    if (g_output_switch_precharge_active != 0U)
    {
        output_state = 1U;
    }
    else if (g_output_switch_active != 0U)
    {
        output_state = 2U;
    }
    else
    {
        output_state = 0U;
    }
    output_fault = (uint16_t)g_power_app.fault;
    feedback_fault = ((g_analog_board_feedback_fault != 0U) ||
                      (g_analog_board_fault_hold_active != 0U) ||
                      (g_analog_board_fault_shutdown_active != 0U)) ? 1U : 0U;
    hard_fault_active = (g_power_app.fault_latched ||
                         (g_power_app.fault != POWER_FAULT_NONE) ||
                         (feedback_fault != 0U)) ? 1U : 0U;
    control_strategy = g_control_strategy;
    active_preset_index = g_active_preset_index;
    policy_fault = g_control_policy_fault;
    policy_fault_latched = g_control_policy_fault_latched;
    settings_result = g_settings_result;

    if ((voltage_set_mv != s_oled_last_voltage_mv) ||
        (current_set_ma != s_oled_last_current_ma))
    {
        g_oled_pending_mask |= OLED_PENDING_VOLTAGE;
    }

    if ((feedback_valid != s_oled_last_feedback_valid) ||
        (using_test_data != s_oled_last_using_test_data) ||
        ((feedback_valid != 0U) &&
         ((voltage_meas_mv != s_oled_last_voltage_meas_mv) ||
          (current_meas_ma != s_oled_last_current_meas_ma))))
    {
        g_oled_pending_mask |= OLED_PENDING_FEEDBACK_STATUS;
    }

    if ((display_mode != s_oled_last_mode) ||
        (using_test_data != s_oled_last_using_test_data) ||
        (control_strategy != s_oled_last_control_strategy) ||
        (active_preset_index != s_oled_last_active_preset_index))
    {
        g_oled_pending_mask |= OLED_PENDING_MODE_STATUS;
    }

    if (output_state != s_oled_last_output_state)
    {
        g_oled_pending_mask |= OLED_PENDING_MODE_STATUS |
                               OLED_PENDING_OUTPUT_STATE;
    }

    if ((output_state != s_oled_last_output_state) ||
        (output_fault != s_oled_last_output_fault) ||
        (feedback_fault != s_oled_last_feedback_fault) ||
        (policy_fault != s_oled_last_policy_fault) ||
        (policy_fault_latched != s_oled_last_policy_fault_latched) ||
        (settings_result != s_oled_last_settings_result))
    {
        g_oled_pending_mask |= OLED_PENDING_OUTPUT_STATE;
    }

    if (g_oled_pending_mask == 0U)
    {
        return GMP_TASK_DONE;
    }

    if (!power_ui_oled_i2c_step_ready())
    {
        return GMP_TASK_DONE;
    }

    // Commit at most one pending line per scheduled OLED task.
    if ((g_oled_pending_mask & OLED_PENDING_OUTPUT_STATE) != 0U)
    {
        if (hard_fault_active != 0U)
        {
            if (output_fault == (uint16_t)POWER_FAULT_OVERVOLTAGE)
            {
                (void)snprintf(line4, sizeof(line4), "FAULT OVP       ");
            }
            else if (output_fault == (uint16_t)POWER_FAULT_OVERCURRENT)
            {
                (void)snprintf(line4, sizeof(line4), "FAULT OCP       ");
            }
            else if (feedback_fault != 0U)
            {
                (void)snprintf(line4, sizeof(line4), "FAULT ADC       ");
            }
            else
            {
                (void)snprintf(line4, sizeof(line4), "FAULT           ");
            }
        }
        else if (policy_fault_latched != 0U)
        {
            if (policy_fault ==
                (uint16_t)PSU_POLICY_FAULT_CV_REGULATION_LOST)
            {
                (void)snprintf(line4, sizeof(line4), "CV REG LOST     ");
            }
            else if (policy_fault ==
                     (uint16_t)PSU_POLICY_FAULT_CC_REGULATION_LOST)
            {
                (void)snprintf(line4, sizeof(line4), "CC REG LOST     ");
            }
            else
            {
                (void)snprintf(line4, sizeof(line4), "POLICY FAULT    ");
            }
        }
        else if (settings_result == PSU_SETTINGS_RESULT_SAVE_OK)
        {
            (void)snprintf(line4, sizeof(line4), "SETTINGS SAVED  ");
        }
        else if (settings_result == PSU_SETTINGS_RESULT_FLASH_ERROR)
        {
            (void)snprintf(line4, sizeof(line4), "SAVE ERROR      ");
        }
        else if (settings_result == PSU_SETTINGS_RESULT_SAVE_REJECTED)
        {
            (void)snprintf(line4, sizeof(line4), "SAVE OFF ONLY   ");
        }
        else
        {
            (void)snprintf(line4, sizeof(line4), "READY           ");
        }

        ret = oled_show_line_checked(0U, 6U, line4);
        if (ret == GMP_EC_OK)
        {
            s_oled_last_output_state = output_state;
            s_oled_last_output_fault = output_fault;
            s_oled_last_feedback_fault = feedback_fault;
            s_oled_last_policy_fault = policy_fault;
            s_oled_last_policy_fault_latched = policy_fault_latched;
            s_oled_last_settings_result = settings_result;
            g_oled_pending_mask &=
                (uint16_t)(~OLED_PENDING_OUTPUT_STATE);
        }
    }
    else if ((g_oled_pending_mask & OLED_PENDING_MODE_STATUS) != 0U)
    {
        const char *actual_text;
        const char *strategy_text;
        const char *test_mark;

        strategy_text = power_control_policy_strategy_text(
            (psu_control_strategy_t)control_strategy);
        if (display_mode == (uint16_t)PSU_MODE_DISPLAY_CV)
        {
            actual_text = "CV";
        }
        else if (display_mode == (uint16_t)PSU_MODE_DISPLAY_CC)
        {
            actual_text = "CC";
        }
        else
        {
            actual_text = "--";
        }
        test_mark = ((using_test_data != 0U) &&
                     ((display_mode == (uint16_t)PSU_MODE_DISPLAY_CV) ||
                      (display_mode == (uint16_t)PSU_MODE_DISPLAY_CC))) ?
                        "*" : " ";

        if (output_state == 1U)
        {
            (void)snprintf(line3, sizeof(line3),
                           "M%u %-4s START --",
                           (unsigned int)(active_preset_index + 1U),
                           strategy_text);
        }
        else
        {
            (void)snprintf(line3, sizeof(line3),
                           "M%u %-4s %-3s %s%s",
                           (unsigned int)(active_preset_index + 1U),
                           strategy_text,
                           (output_state == 2U) ? "ON" : "OFF",
                           actual_text,
                           test_mark);
        }

        ret = oled_show_line_checked(0U, 4U, line3);
        if (ret == GMP_EC_OK)
        {
            s_oled_last_mode = display_mode;
            s_oled_last_using_test_data = using_test_data;
            s_oled_last_control_strategy = control_strategy;
            s_oled_last_active_preset_index = active_preset_index;
            g_oled_pending_mask &= (uint16_t)(~OLED_PENDING_MODE_STATUS);
        }
    }
    else if ((g_oled_pending_mask & OLED_PENDING_FEEDBACK_STATUS) != 0U)
    {
        if (feedback_valid == 0U)
        {
            (void)snprintf(line2, sizeof(line2), "VM--.-V IM---mA");
        }
        else
        {
            (void)snprintf(line2, sizeof(line2),
                           "VM%02u.%02uV IM%03umA",
                           (unsigned int)(voltage_meas_mv / 1000U),
                           (unsigned int)((voltage_meas_mv % 1000U) / 10U),
                           (unsigned int)power_ui_display_current(
                               current_meas_ma));
        }

        ret = oled_show_line_checked(0U, 2U, line2);
        if (ret == GMP_EC_OK)
        {
            s_oled_last_feedback_valid = feedback_valid;
            s_oled_last_using_test_data = using_test_data;
            s_oled_last_voltage_meas_mv = voltage_meas_mv;
            s_oled_last_current_meas_ma = current_meas_ma;
            g_oled_pending_mask &=
                (uint16_t)(~OLED_PENDING_FEEDBACK_STATUS);
        }
    }
    else if ((g_oled_pending_mask &
              (OLED_PENDING_VOLTAGE | OLED_PENDING_CURRENT)) != 0U)
    {
        (void)snprintf(line1, sizeof(line1), "VS%02u.%1uV IS%03umA ",
                       (unsigned int)(voltage_set_mv / 1000U),
                       (unsigned int)((voltage_set_mv % 1000U) / 100U),
                       (unsigned int)power_ui_display_current(current_set_ma));
        ret = oled_show_line_checked(0U, 0U, line1);
        if (ret == GMP_EC_OK)
        {
            s_oled_last_voltage_mv = voltage_set_mv;
            s_oled_last_current_ma = current_set_ma;
            g_oled_pending_mask &=
                (uint16_t)(~(OLED_PENDING_VOLTAGE |
                             OLED_PENDING_CURRENT));
        }
    }
    power_ui_record_oled_action_result(ret);
    if (ret != GMP_EC_OK)
    {
        ++g_oled_update_error_count;
        power_ui_handle_oled_failure(ret);
        return GMP_TASK_DONE;
    }

    ++g_oled_update_ok_count;
    ++g_oled_line_update_count;
    return GMP_TASK_DONE;
#else
    static bool s_page_initialized = false;
    static bool s_last_fault_page = false;
    char line1[32];
    char line2[32];
    char line3[32];
    char line4[32];
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    uint16_t voltage_meas_mv;
    uint16_t current_meas_ma;
    uint16_t trip_voltage_mv;
    uint16_t trip_current_ma;
    power_state_t state;
    power_fault_t fault;
    bool output_requested;
    bool output_enabled;
    bool fault_latched;
    bool fault_page;

    if (flag_init_cmpt == 1)
    {
        voltage_set_mv = g_power_app.voltage_set_mv;
        current_set_ma = g_power_app.current_set_ma;
        voltage_meas_mv = g_power_app.voltage_meas_mv;
        current_meas_ma = g_power_app.current_meas_ma;
        trip_voltage_mv = g_power_app.trip_voltage_mv;
        trip_current_ma = g_power_app.trip_current_ma;
        state = g_power_app.state;
        fault = g_power_app.fault;
        output_requested = g_power_app.output_requested;
        output_enabled = g_power_app.output_enabled;
        fault_latched = g_power_app.fault_latched;
        fault_page = fault_latched || (state == POWER_STATE_FAULT);

        if (!s_page_initialized)
        {
            oled_clear();
            s_page_initialized = true;
            s_last_fault_page = fault_page;
        }
        else if (fault_page != s_last_fault_page)
        {
            oled_clear();
            s_last_fault_page = fault_page;
        }

        (void)snprintf(line1, sizeof(line1), "SET %02u.%1uV %03umA ",
                       (unsigned int)(voltage_set_mv / 1000U),
                       (unsigned int)((voltage_set_mv % 1000U) / 100U),
                       (unsigned int)power_ui_display_current(current_set_ma));
        (void)snprintf(line2, sizeof(line2), "OUT%02u.%03uV %03umA",
                       (unsigned int)(voltage_meas_mv / 1000U),
                       (unsigned int)(voltage_meas_mv % 1000U),
                       (unsigned int)power_ui_display_current(current_meas_ma));
        (void)snprintf(line3, sizeof(line3), "%-7s OUT:%-3s ",
                       power_ui_state_text(state), output_enabled ? "ON" : "OFF");

        if (!fault_page)
        {
            (void)snprintf(line4, sizeof(line4), "READY REQ:%-3s   ",
                           output_requested ? "ON" : "OFF");
        }
        else if (fault == POWER_FAULT_OVERVOLTAGE)
        {
            (void)snprintf(line4, sizeof(line4), "FLT %s %02u.%03uV ",
                           power_ui_fault_text(fault),
                           (unsigned int)(trip_voltage_mv / 1000U),
                           (unsigned int)(trip_voltage_mv % 1000U));
        }
        else if (fault == POWER_FAULT_OVERCURRENT)
        {
            (void)snprintf(line4, sizeof(line4), "FLT %s %03umA   ",
                           power_ui_fault_text(fault),
                           (unsigned int)power_ui_display_current(trip_current_ma));
        }
        else
        {
            (void)snprintf(line4, sizeof(line4), "FAULT %-9s ",
                           power_ui_fault_text(fault));
        }

        oled_show_str(0U, 0U, line1);
        oled_show_str(0U, 2U, line2);
        oled_show_str(0U, 4U, line3);
        oled_show_str(0U, 6U, line4);
    }

    return GMP_TASK_DONE;
#endif
}

//=================================================================================================
// FPGA control function

gmp_task_status_t fpga_test_task(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    static fast_gt led_stat = 0;
    if (led_stat == 0)
    {
        led_stat = 1;
        SPI_writeReg(0x01U, 0x0003U);
    }
    else
    {
        led_stat = 0;
        SPI_writeReg(0x01U, 0x0000U);
    }

    SPI_writeReg(0x03, 0x00FF);

    // trigger ADC
    SPI_writeReg(0x05, 0x8000);
    SPI_writeReg(0x06, 0xA000);
    SPI_writeReg(0x07, 0xF000);

    uint16_t adc_result;
    adc_result = SPI_readReg(0x08);

    SPI_writeReg(0x04, adc_result);

    return GMP_TASK_DONE;
}

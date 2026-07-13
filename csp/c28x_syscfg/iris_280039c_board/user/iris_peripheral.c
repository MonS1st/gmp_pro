
// GMP basic core header
#include <gmp_core.h>
#include <ctrl_settings.h>

// user main header
#include "ctl_main.h"
#include "power_app.h"
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

void beep_on()
{
#if PSU_ENABLE_BEEP && !PSU_SAFE_BRINGUP
    gmp_hal_gpio_write(gpio_beep, 1);
#endif
}

void beep_off()
{
#if PSU_ENABLE_BEEP && !PSU_SAFE_BRINGUP
    gmp_hal_gpio_write(gpio_beep, 0);
#endif
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
#if PSU_ENABLE_HT16K33_DISPLAY
static uint16_t s_led_last_voltage_mv = 0xFFFFU;
static uint16_t s_led_last_current_ma = 0xFFFFU;
#endif
static uint16_t s_oled_last_voltage_mv = 0xFFFFU;
static uint16_t s_oled_last_current_ma = 0xFFFFU;
static uint16_t s_oled_last_key_id = 0xFFFFU;
static uint16_t s_key_consecutive_timeout_count = 0U;

static void power_ui_request_led_setpoint_update(void);

static void power_ui_reset_key_candidate(void)
{
    g_key_candidate_id = 0U;
    g_key_candidate_count = 0U;
}

static void power_ui_handle_key_read_error(void)
{
    power_ui_reset_key_candidate();
    s_key_release_count = 0U;
}

static bool power_ui_execute_key_action(uint16_t key_id)
{
    uint16_t current;
    uint32_t next;
    bool action_executed = true;

    switch (key_id)
    {
    case PSU_KEY_VOLTAGE_UP_ID:
        current = power_app_get_voltage_mv();
        next = (uint32_t)current + PSU_VOLTAGE_STEP_MV;
        if (next > PSU_VOLTAGE_CMD_MAX_MV)
        {
            next = PSU_VOLTAGE_CMD_MAX_MV;
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
        next = (uint32_t)current + PSU_CURRENT_STEP_MA;
        if (next > PSU_CURRENT_CMD_MAX_MA)
        {
            next = PSU_CURRENT_CMD_MAX_MA;
        }
        power_app_set_current_ma((uint16_t)next);
        power_ui_request_led_setpoint_update();
        break;

    case PSU_KEY_CURRENT_DOWN_ID:
        current = power_app_get_current_ma();
        power_app_set_current_ma((current < PSU_CURRENT_STEP_MA) ?
                                     0U : (uint16_t)(current - PSU_CURRENT_STEP_MA));
        power_ui_request_led_setpoint_update();
        break;

    case PSU_KEY_OUTPUT_TOGGLE_ID:
#if PSU_SAFE_BRINGUP || !PSU_ALLOW_OUTPUT_REQUEST
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

    // Ignore every nonzero report until four consecutive successful idle
    // scans prove that the shared I2C bus and key matrix started cleanly.
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

    // A confirmed key remains latched until four consecutive successful zero
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

    if (key_id == 0U)
    {
        power_ui_reset_key_candidate();
        return false;
    }

    if (g_key_candidate_id != key_id)
    {
        g_key_candidate_id = key_id;
        g_key_candidate_count = 1U;
    }
    else if (g_key_candidate_count < PSU_KEY_PRESS_CONFIRM_COUNT)
    {
        ++g_key_candidate_count;
    }

    if (g_key_candidate_count < PSU_KEY_PRESS_CONFIRM_COUNT)
    {
        return false;
    }

    s_last_key_id = key_id;
    power_ui_reset_key_candidate();
    ++g_key_confirmed_count;

    action_executed = power_ui_execute_key_action(key_id);
    if (action_executed)
    {
        ++g_key_action_count;
    }

    return action_executed;
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

static uint16_t power_ui_display_current(uint16_t current_ma)
{
    return (current_ma > 999U) ? 999U : current_ma;
}
#endif

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

static void power_ui_request_led_setpoint_update(void)
{
#if !PSU_ENABLE_HT16K33_DISPLAY
    return;
#else
    uint16_t voltage_set_mv = g_power_app.voltage_set_mv;
    uint16_t current_set_ma = g_power_app.current_set_ma;

    if (g_key_scan_ready == 0U)
    {
        return;
    }

    if ((voltage_set_mv == s_led_last_voltage_mv) &&
        (current_set_ma == s_led_last_current_ma))
    {
        return;
    }

    // First-stage board test format from master: 2026-XX-, where XX is
    // the last key accepted by the board-level three-scan confirmation.
    update_led_content_8byte(
        &ht16k33,
        led_lut[2], led_lut[0], led_lut[2], led_lut[6],
        led_lut[20], led_lut[s_last_key_id / 10U],
        led_lut[s_last_key_id % 10U], led_lut[20]);

    s_led_last_voltage_mv = voltage_set_mv;
    s_led_last_current_ma = current_set_ma;
#endif
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
        bool update_requested;
        ec_gt ret;

        if (g_key_scan_ready == 0U)
        {
            return GMP_TASK_DONE;
        }

        update_requested = (dev->is_dirty != 0);
        ret = ht16k33_update_display(dev);
        g_led_update_result = ret;

        // If the display peripheral fails, stop only this background UI task.
        if (ret != GMP_EC_OK)
        {
            tsk->is_enabled = 0;
        }
        else if (update_requested)
        {
            g_key_ignore_scan_count = 2U;
            ++g_led_update_count;
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

        // An OLED line consists of many back-to-back I2C transactions. Leave
        // two complete 50 ms key periods idle before accessing HT16K33 again.
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

            if ((s_key_consecutive_timeout_count >= 3U) &&
                (g_oled_dynamic_update_enabled != 0U))
            {
                g_oled_dynamic_update_enabled = 0U;
                g_oled_pending_mask = 0U;
                ++g_oled_disabled_due_i2c_count;
            }

            power_ui_handle_key_read_error();
            return GMP_TASK_DONE;
        }

        ++g_key_read_ok_count;
        g_key_consecutive_error_count = 0U;
        s_key_consecutive_timeout_count = 0U;

        if (g_key_consecutive_ok_count < 0xFFFFU)
        {
            ++g_key_consecutive_ok_count;
        }
        // Ten good key reads may recover a key-timeout suspension, but an
        // OLED write failure keeps dynamic updates latched off for diagnosis.
        if ((g_oled_dynamic_update_enabled == 0U) &&
            (g_oled_update_error_count == 0U) &&
            (g_key_consecutive_ok_count >= 10U))
        {
            g_oled_dynamic_update_enabled = 1U;
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
    static bool s_page_initialized = false;
#if PSU_SAFE_BRINGUP
    char line2[16];
    char line3[16];
    char line4[16];
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    uint16_t key_id;
    ec_gt ret;
#endif
    GMP_UNUSED_VAR(tsk);

#if PSU_SAFE_BRINGUP
    if (flag_init_cmpt == 1U)
    {
        voltage_set_mv = g_power_app.voltage_set_mv;
        current_set_ma = g_power_app.current_set_ma;
        key_id = g_last_raw_key_id;

        if (!s_page_initialized)
        {
            oled_clear();
            oled_show_str(0U, 0U, "BOARD TEST");
            s_page_initialized = true;
            g_oled_pending_mask = OLED_PENDING_VOLTAGE |
                                  OLED_PENDING_CURRENT |
                                  OLED_PENDING_KEY;
            g_key_i2c_holdoff_count = 2U;
            ++g_oled_line_update_count;
            return GMP_TASK_DONE;
        }

        if (g_oled_dynamic_update_enabled == 0U)
        {
            g_oled_pending_mask = 0U;
            return GMP_TASK_DONE;
        }

        if (voltage_set_mv != s_oled_last_voltage_mv)
        {
            g_oled_pending_mask |= OLED_PENDING_VOLTAGE;
        }

        if (current_set_ma != s_oled_last_current_ma)
        {
            g_oled_pending_mask |= OLED_PENDING_CURRENT;
        }

        if (key_id != s_oled_last_key_id)
        {
            g_oled_pending_mask |= OLED_PENDING_KEY;
        }

        // Commit at most one pending line per scheduled OLED task.
        if ((g_oled_pending_mask & OLED_PENDING_VOLTAGE) != 0U)
        {
            (void)snprintf(line2, sizeof(line2), "V %-5u mV    ",
                           (unsigned int)voltage_set_mv);
            ret = oled_show_line_checked(0U, 2U, line2);
            if (ret == GMP_EC_OK)
            {
                s_oled_last_voltage_mv = voltage_set_mv;
                g_oled_pending_mask &= (uint16_t)(~OLED_PENDING_VOLTAGE);
            }
        }
        else if ((g_oled_pending_mask & OLED_PENDING_CURRENT) != 0U)
        {
            (void)snprintf(line3, sizeof(line3), "I %-3u mA      ",
                           (unsigned int)current_set_ma);
            ret = oled_show_line_checked(0U, 4U, line3);
            if (ret == GMP_EC_OK)
            {
                s_oled_last_current_ma = current_set_ma;
                g_oled_pending_mask &= (uint16_t)(~OLED_PENDING_CURRENT);
            }
        }
        else if ((g_oled_pending_mask & OLED_PENDING_KEY) != 0U)
        {
            (void)snprintf(line4, sizeof(line4), "KEY %-2u       ",
                           (unsigned int)key_id);
            ret = oled_show_line_checked(0U, 6U, line4);
            if (ret == GMP_EC_OK)
            {
                s_oled_last_key_id = key_id;
                g_oled_pending_mask &= (uint16_t)(~OLED_PENDING_KEY);
            }
        }
        else
        {
            return GMP_TASK_DONE;
        }

        g_oled_last_result = ret;
        if (ret != GMP_EC_OK)
        {
            ++g_oled_update_error_count;
            g_oled_dynamic_update_enabled = 0U;
            g_oled_pending_mask = 0U;
            return GMP_TASK_DONE;
        }

        ++g_oled_update_ok_count;
        g_key_i2c_holdoff_count = 2U;
        ++g_oled_line_update_count;
    }

    return GMP_TASK_DONE;
#else
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
        SPI_writeReg(0x01, 0x0003);
    }
    else
    {
        led_stat = 0;
        SPI_writeReg(0x01, 0x0000);
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


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
    gmp_hal_gpio_write(gpio_beep, 1);
}

void beep_off()
{
    gmp_hal_gpio_write(gpio_beep, 0);
}

//=================================================================================================
// LED control function

// devices
iic_halt iic_bus;
ht16k33_dev_t ht16k33;
hdc1080_dev_t hdc1080;

// File-local variables remain visible to CCS for key scan debugging.
static volatile uint16_t s_last_key_id = 0U;
static volatile uint16_t s_key_release_count = 0U;

static bool power_ui_key_supports_repeat(uint16_t key_id)
{
    return (key_id == PSU_KEY_VOLTAGE_UP_ID) ||
           (key_id == PSU_KEY_VOLTAGE_DOWN_ID) ||
           (key_id == PSU_KEY_CURRENT_UP_ID) ||
           (key_id == PSU_KEY_CURRENT_DOWN_ID);
}

static void power_ui_execute_key_action(uint16_t key_id)
{
    uint16_t current;
    uint32_t next;

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
        break;

    case PSU_KEY_VOLTAGE_DOWN_ID:
        current = power_app_get_voltage_mv();
        power_app_set_voltage_mv((current < PSU_VOLTAGE_STEP_MV) ?
                                     0U : (uint16_t)(current - PSU_VOLTAGE_STEP_MV));
        break;

    case PSU_KEY_CURRENT_UP_ID:
        current = power_app_get_current_ma();
        next = (uint32_t)current + PSU_CURRENT_STEP_MA;
        if (next > PSU_CURRENT_CMD_MAX_MA)
        {
            next = PSU_CURRENT_CMD_MAX_MA;
        }
        power_app_set_current_ma((uint16_t)next);
        break;

    case PSU_KEY_CURRENT_DOWN_ID:
        current = power_app_get_current_ma();
        power_app_set_current_ma((current < PSU_CURRENT_STEP_MA) ?
                                     0U : (uint16_t)(current - PSU_CURRENT_STEP_MA));
        break;

    case PSU_KEY_OUTPUT_TOGGLE_ID:
        if (!g_power_app.fault_latched)
        {
            power_app_request_output(!g_power_app.output_requested);
        }
        break;

    case PSU_KEY_FAULT_RESET_ID:
        power_app_reset_fault();
        break;

    default:
        break;
    }
}

static bool power_ui_process_key_id(uint16_t key_id)
{
    if (key_id != 0U)
    {
        s_key_release_count = 0U;

        if (key_id != s_last_key_id)
        {
            s_last_key_id = key_id;
            power_ui_execute_key_action(key_id);
            return true;
        }

        if (power_ui_key_supports_repeat(key_id))
        {
            power_ui_execute_key_action(key_id);
            return true;
        }

        return false;
    }

    if (s_last_key_id == 0U)
    {
        s_key_release_count = 0U;
        return false;
    }

    if (s_key_release_count < PSU_KEY_RELEASE_FILTER_COUNT)
    {
        ++s_key_release_count;
    }

    if (s_key_release_count >= PSU_KEY_RELEASE_FILTER_COUNT)
    {
        s_last_key_id = 0U;
        s_key_release_count = 0U;
    }

    return false;
}

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

gmp_task_status_t tsk_LED_flush(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;

    if(flag_init_cmpt)
    {
    // fresh LED buffer here.
    ec_gt ret = ht16k33_update_display(dev);

    // if meets error, close this task
    if (ret != GMP_EC_OK)
    {
        tsk->is_enabled = 0;
    }
    }

    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_key_flush(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;
    fast_gt key_id = 0;
    bool action_executed;

    if(flag_init_cmpt)
    {
        ec_gt ret = ht16k33_read_keys(dev, &key_id);

        // If the key peripheral fails, stop only this background UI task.
        if (ret != GMP_EC_OK)
        {
            tsk->is_enabled = 0;
            return GMP_TASK_DONE;
        }

        action_executed = power_ui_process_key_id((uint16_t)key_id);

        if (action_executed)
        {
            gmp_base_print("Power key action, id=%d\r\n", key_id);
        }
    }

    return GMP_TASK_DONE;
}

gmp_task_status_t oled_show_task(gmp_task_t* tsk)
{
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

    GMP_UNUSED_VAR(tsk);

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

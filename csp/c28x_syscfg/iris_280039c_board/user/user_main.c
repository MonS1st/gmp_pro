// This is the example of user main.

// GMP basic core header
#include <gmp_core.h>
#include <ctrl_settings.h>

// user main header
#include "ctl_main.h"
#include "user_main.h"
#include <stdlib.h>

#include <core/dev/mem_presp.h>
#include <core/dev/pil_core.h>
#include <core/dev/tunable.h>

#include <oled_driver.h>

#include "power_app.h"
#include "power_debug.h"

ctrl_gt kp, ki, kd;

ctrl_gt ram_range[512];

//=================================================================================================
// Datalink protocol online Debug module

gmp_datalink_t dl;

//
// PIL (processor in loop module)
//
gmp_pil_sim_t pil;

//
// Tunable Dictionary
//
const gmp_param_item_t dict_m1[] = {
    {&kp, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&ki, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},
    {&kd, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},
};
const uint16_t var_tunable_count = sizeof(dict_m1) / sizeof(dict_m1[0]);
gmp_param_tunable_t tunable;

//
// Memory perspective Dictionary
//
const gmp_mem_region_t mem_regions[] = {
    {.base_addr = &ram_range, .byte_length = sizeof(ram_range) * GMP_PORT_DATA_SIZE_PER_BYTES, .perm = GMP_MEM_PERM_RW},
};
const uint16_t mem_regions_count = sizeof(mem_regions) / sizeof(mem_regions[0]);
gmp_mem_persp_t mem_persp_server;

//
// Datalink protocol stack task
//
gmp_task_status_t tsk_dl_debug_device(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    flush_dl_rx_buffer();

    // In PC simulation environment the DL protocol module is disabled.
#ifndef SPECIFY_PC_ENVIRONMENT

    gmp_dl_event_t e = gmp_dev_dl_loop_cb(&dl);

    switch (e)
    {
    //
    // if TX data is ready, do transmit
    //
    case GMP_DL_EVENT_TX_RDY:

        // send tx buffer message
        flush_dl_tx_buffer();

        // ack TX state machine.
        gmp_dev_dl_tx_state_done(&dl);
        break;

    case GMP_DL_EVENT_RX_OK:

        //
        // Ack PIL simulation message
        //
        if (gmp_pil_sim_rx_cb(&pil))
            break;

        //
        // Ack parameter tunable message
        //
        if (gmp_param_tunable_rx_cb(&tunable))
            break;

        //
        // Ack memory perspective message
        //
        if (gmp_mem_persp_rx_cb(&mem_persp_server))
            break;

        //
        // Echo Command
        //
        if (dl.rx_head.cmd == 0x99)
        {
            // echo payload_buf
            gmp_dev_dl_tx_request(&dl, dl.rx_head.seq_id, GMP_DL_CMD_ECHO, dl.expected_payload_len, dl.payload_buf);

            // ack this message
            gmp_dev_dl_msg_handled(&dl);

            break;
        }

        // default handler
        gmp_dev_dl_default_rx_handler(&dl);

        break;
    }

#endif // SPECIFY_PC_ENVIRONMENT

    return GMP_TASK_DONE;
}

//=================================================================================================
// task manager

// GPIO
gpio_halt user_led;
volatile uint16_t flag_init_cmpt = 0;

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    static fast_gt led_stat = 0;
    if (led_stat == 0)
    {
        led_stat = 1;
        gmp_hal_gpio_write(user_led, 0);
    }
    else
    {
        led_stat = 0;
        gmp_hal_gpio_write(user_led, 1);
    }

    return GMP_TASK_DONE;
}

//
// Non-blocking task scheduler
//
gmp_scheduler_t sched;

static gmp_task_status_t tsk_power_app(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    power_app_slow_step();
    return GMP_TASK_DONE;
}

static gmp_task_status_t tsk_power_debug_command(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    power_debug_process_command();
    return GMP_TASK_DONE;
}

static gmp_task_status_t tsk_power_console_status(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    power_debug_print_status();
    return GMP_TASK_DONE;
}

// Keep named task objects so startup/error handling never depends on a
// scheduler-list index. All tasks must be non-blocking.
static gmp_task_t task_dl_online =
    {"dl_online", tsk_dl_debug_device, 2, 0, 1, NULL};
static gmp_task_t task_flush_key =
    {"flush_key", tsk_key_flush, PSU_KEY_TASK_PERIOD_MS, 10,
     PSU_ENABLE_HT16K33_KEY, (void*)&ht16k33};
static gmp_task_t task_oled_show =
    {"oled_show", oled_show_task, 100, 500,
     PSU_ENABLE_OLED_DISPLAY, NULL};
static gmp_task_t task_flush_led =
    {"flush_led", tsk_LED_flush, 500, 200,
     PSU_ENABLE_HT16K33_DISPLAY, (void*)&ht16k33};
static gmp_task_t task_fpga_test =
    {"fpga_test", fpga_test_task, 1000, 600, 0, NULL};
static gmp_task_t task_blink_led =
    {"blink_led", tsk_blink, 1000, 100, 1, NULL};
static gmp_task_t task_startup =
    {"startup", tsk_startup, 250, 0, 1, NULL};
static gmp_task_t task_power_app =
    {"power_app", tsk_power_app, 1, 0, 1, NULL};
static gmp_task_t task_power_command =
    {"power_cmd", tsk_power_debug_command, 10, 0,
     PSU_ENABLE_MANUAL_COMMAND, NULL};
static gmp_task_t task_power_console =
    {"power_console", tsk_power_console_status, 500, 500,
     PSU_ENABLE_CONSOLE_UI, NULL};

static gmp_task_t *const tasks[] = {
    &task_dl_online,
    &task_flush_key,
    &task_oled_show,
    &task_flush_led,
    &task_fpga_test,
    &task_blink_led,
    &task_startup,
    &task_power_app,
    &task_power_command,
    &task_power_console,
};

//=================================================================================================
// initialize routine

GMP_NO_OPT_PREFIX
void init(void) GMP_NO_OPT_SUFFIX
{
    int i;

    // init scheduler
    gmp_scheduler_init(&sched);

    for (i = 0; i < sizeof(tasks) / sizeof(tasks[0]); ++i)
        gmp_scheduler_add_task(&sched, tasks[i]);

    // init datalink protocol
    gmp_dev_dl_init(&dl);

    // enable PIL simulation environment
    gmp_pil_sim_init(&pil, &dl, 0x10);

    // Band DL module with tunable and persp module.
    gmp_param_tunable_init(&tunable, &dl, 0x30, dict_m1, var_tunable_count);
    gmp_mem_persp_init(&mem_persp_server, &dl, 0x50, mem_regions, mem_regions_count);
}

// Initialization tasks after all peripherals have been initialized
gmp_task_status_t tsk_startup(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    static uint16_t beep_counter = 0;

    if (beep_counter == 0)
        beep_on();
    else if (beep_counter == 1)
        beep_off();
    else if (beep_counter == 2)
        beep_on();
    else if (beep_counter == 3)
        beep_off();

    beep_counter += 1;

    // if program is complete, init all the peripherals, and close this routine.
    if (beep_counter >= 4)
    {
#if PSU_ENABLE_HT16K33_KEY || PSU_ENABLE_HT16K33_DISPLAY
        ht16k33_init_t ht16k33_init_struct = {.brightness = 15, .blink_rate = 0, .int_enable = 0, .int_act_high = 0};

        ec_gt ec = ht16k33_init(&ht16k33, iic_bus, HT16K33_DEFAULT_DEV_ADDR, &ht16k33_init_struct);

        if (ec == GMP_EC_OK)
        {
#if PSU_ENABLE_HT16K33_DISPLAY
            update_led_content_8byte(&ht16k33, led_lut[2], led_lut[0], led_lut[2], led_lut[6], led_lut[20], led_lut[7],
                                             led_lut[7], led_lut[20]);
#endif
        }
        else
        {
#if PSU_ENABLE_HT16K33_KEY
            task_flush_key.is_enabled = 0;
#endif
#if PSU_ENABLE_HT16K33_DISPLAY
            task_flush_led.is_enabled = 0;
#endif
        }
#endif

        // init and test the oled.
#if PSU_ENABLE_OLED_DISPLAY
        oled_init();
#endif

#if PSU_ENABLE_HT16K33_KEY || PSU_ENABLE_HT16K33_DISPLAY
        if (ec == GMP_EC_OK)
        {
            gmp_base_print("UI INIT HT16K33 ret=%lu key=%u led=%u oled=%u\r\n",
                           (unsigned long)ec,
                           (unsigned int)task_flush_key.is_enabled,
                           (unsigned int)task_flush_led.is_enabled,
                           (unsigned int)task_oled_show.is_enabled);
        }
        else
        {
            gmp_base_print("UI INIT HT16K33 FAILED ret=%lu\r\n", (unsigned long)ec);
        }
#endif

        //        hdc1080_config_reg_t hdc1080_cfg = {.all = 0};
        //        hdc1080_cfg.bits.mode = 1; // continuous acquisition data
        //
        //        hdc1080_init(&hdc1080, iic_bus, HDC1080_I2C_ADDR_DEFAULT, hdc1080_cfg);
        //        hdc1080_trigger_temp_hum_sequence(&hdc1080);

        flag_init_cmpt = 1;

        // startup process is complete.
        tsk->is_enabled = 0;
    }

    return GMP_TASK_DONE;
}

//=================================================================================================
// endless loop routine

GMP_NO_OPT_PREFIX
void mainloop(void) GMP_NO_OPT_SUFFIX
{
    // run task scheduler
    gmp_scheduler_dispatch(&sched);
}

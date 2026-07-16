// This is the example of user main.

// GMP basic core header
#include <gmp_core.h>
#include <ctrl_settings.h>

// user main header
#include "ctl_main.h"
#include "analog_io_test.h"
#include "power_control_policy.h"
#include "power_mode_monitor.h"
#include "power_settings_store.h"
#include "rotary_encoder_ui.h"
#include "user_main.h"
#include <stdlib.h>

#include <core/dev/mem_presp.h>
#include <core/dev/pil_core.h>
#include <core/dev/tunable.h>

#include <oled_driver.h>

#include "power_app.h"
#include "power_debug.h"
#include "power_hal.h"
#include <xplt.ctl_interface.h>
#include <xplt.peripheral.h>

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
volatile uint16_t g_fpga_reset_level = 0U;
volatile uint16_t g_fpga_spi_r1_readback = 0xFFFFU;
volatile uint16_t g_power_safe_bringup_self_test_failures = 0U;
volatile uint32_t g_main_isr_count = 0U;
volatile uint32_t g_scheduler_loop_count = 0U;
volatile uint16_t g_ui_init_stage = 0U;
volatile uint16_t g_ui_init_result = 0xFFFFU;
volatile uint16_t g_last_raw_key_id = 0U;
volatile ec_gt g_key_read_result = GMP_EC_OK;
volatile uint32_t g_key_read_ok_count = 0U;
volatile uint32_t g_key_read_error_count = 0U;
volatile uint16_t g_key_consecutive_error_count = 0U;
volatile uint16_t g_key_candidate_id = 0U;
volatile uint16_t g_key_candidate_count = 0U;
volatile uint16_t g_key_vote_id = 0U;
volatile uint16_t g_key_vote_count = 0U;
volatile uint16_t g_key_vote_age = 0U;
volatile uint32_t g_key_vote_timeout_count = 0U;
volatile uint16_t g_key_scan_ready = 0U;
volatile uint32_t g_key_confirmed_count = 0U;
volatile uint32_t g_key_action_count = 0U;
volatile uint16_t g_last_confirmed_key_id = 0U;
volatile uint16_t g_last_unmapped_key_id = 0U;
volatile uint32_t g_unmapped_confirmed_count = 0U;
volatile uint32_t g_key_first_sample_action_count = 0U;
volatile uint16_t g_key_ignore_scan_count = 0U;
volatile uint32_t g_led_update_count = 0U;
volatile ec_gt g_led_update_result = GMP_EC_OK;
volatile ec_gt g_ht16k33_clear_result = GMP_EC_NOT_READY;
volatile uint32_t g_ht16k33_clear_count = 0U;
volatile ec_gt g_ht16k33_display_off_result = GMP_EC_NOT_READY;
volatile ec_gt g_ht16k33_osc_on_result = GMP_EC_NOT_READY;
volatile ec_gt g_ht16k33_brightness_result = GMP_EC_NOT_READY;
volatile ec_gt g_ht16k33_display_on_result = GMP_EC_NOT_READY;
volatile ec_gt g_ht16k33_all_on_result = GMP_EC_NOT_READY;
volatile uint16_t g_ht16k33_display_test_state = HT16K33_DISPLAY_TEST_WAIT_INIT;
volatile uint32_t g_ht16k33_display_test_count = 0U;
volatile uint16_t g_oled_pending_mask = 0U;
volatile uint32_t g_oled_line_update_count = 0U;
volatile uint16_t g_key_i2c_holdoff_count = 0U;
volatile uint32_t g_key_i2c_holdoff_skip_count = 0U;
volatile uint16_t g_oled_dynamic_update_enabled = 0U;
volatile uint32_t g_oled_disabled_due_i2c_count = 0U;
volatile uint16_t g_key_consecutive_ok_count = 0U;
volatile uint16_t g_i2c_fault_active = 0U;
volatile uint16_t g_i2c_recovery_state = PSU_I2C_RECOVERY_IDLE;
volatile uint32_t g_i2c_recovery_attempt_count = 0U;
volatile uint32_t g_i2c_recovery_success_count = 0U;
volatile uint32_t g_i2c_recovery_failure_count = 0U;
volatile ec_gt g_i2c_recovery_ht_init_result = GMP_EC_NOT_READY;
volatile ec_gt g_i2c_recovery_verify_result = GMP_EC_NOT_READY;
volatile uint16_t g_i2c_recovery_verify_ok_count = 0U;
volatile uint16_t g_i2c_sda_level_before = 0U;
volatile uint16_t g_i2c_scl_level_before = 0U;
volatile uint16_t g_i2c_sda_level_after = 0U;
volatile uint16_t g_i2c_scl_level_after = 0U;
volatile uint32_t g_i2c_key_backoff_skip_count = 0U;
volatile time_gt g_i2c_recovery_next_tick = 0U;
volatile uint16_t g_i2c_clear_state = 0U;
volatile uint16_t g_i2c_clear_clock_count = 0U;
volatile uint32_t g_i2c_clear_attempt_count = 0U;
volatile uint32_t g_i2c_clear_success_count = 0U;
volatile uint32_t g_i2c_clear_failure_count = 0U;
volatile uint16_t g_i2c_clear_sda_initial = 0U;
volatile uint16_t g_i2c_clear_sda_final = 0U;
volatile uint16_t g_i2c_clear_scl_final = 0U;
volatile uint16_t g_i2c_clear_stop_generated = 0U;
volatile uint16_t g_i2c_clear_pinmux_restored = 0U;
volatile ec_gt g_oled_last_result = GMP_EC_NOT_READY;
volatile uint32_t g_oled_update_ok_count = 0U;
volatile uint32_t g_oled_update_error_count = 0U;
volatile uint16_t g_oled_init_state = OLED_INIT_WAIT_POWER;
volatile uint16_t g_oled_init_retry_count = 0U;
volatile uint32_t g_oled_init_attempt_count = 0U;
volatile uint32_t g_oled_init_success_count = 0U;
volatile uint32_t g_oled_init_failure_count = 0U;
volatile uint16_t g_oled_clear_page_index = 0U;
volatile time_gt g_oled_next_action_tick = 0U;
volatile uint32_t g_oled_deferred_for_key_count = 0U;
volatile ec_gt g_oled_probe_3c_result = GMP_EC_NOT_READY;
volatile ec_gt g_oled_probe_3d_result = GMP_EC_NOT_READY;
volatile uint16_t g_oled_selected_address = 0U;
volatile uint32_t g_oled_address_scan_count = 0U;
volatile uint32_t g_oled_no_device_count = 0U;
volatile uint16_t g_oled_reset_control_available = 0U;
volatile uint16_t g_oled_power_control_available = 0U;
volatile uint32_t g_oled_reset_count = 0U;

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

#if PSU_SAFE_BRINGUP && PSU_ENABLE_SAFE_SELF_TEST
#define PSU_SAFE_TEST_HAL_OUTPUTS  (1U << 0)
#define PSU_SAFE_TEST_APP_REQUEST  (1U << 1)
#define PSU_SAFE_TEST_DEBUG_TOGGLE (1U << 2)
#define PSU_SAFE_TEST_KEY_ACTIONS  (1U << 3)
#define PSU_SAFE_TEST_STATE_GUARD  (1U << 4)
#define PSU_SAFE_TEST_PWM_TRIP     (1U << 5)

static void power_safe_bringup_run_self_test(void)
{
    uint16_t failures = 0U;

    if (!power_hal_safe_bringup_self_test())
    {
        failures |= PSU_SAFE_TEST_HAL_OUTPUTS;
    }

    power_app_request_output(true);
    if (g_power_app.output_requested)
    {
        failures |= PSU_SAFE_TEST_APP_REQUEST;
    }

    if (!power_debug_safe_bringup_self_test())
    {
        failures |= PSU_SAFE_TEST_DEBUG_TOGGLE;
    }

    if (!power_ui_safe_bringup_self_test())
    {
        failures |= PSU_SAFE_TEST_KEY_ACTIONS;
    }

    // Simulate hostile debugger/memory writes and verify that one fast step
    // sanitizes them before any output path can consume the values.
    g_power_app.output_requested = true;
    g_power_app.output_enabled = true;
    g_power_app.state = POWER_STATE_CC;
    power_app_fast_step();
    if (g_power_app.output_requested || g_power_app.output_enabled ||
        (g_power_app.state != POWER_STATE_OFF) || power_output_hw_get())
    {
        failures |= PSU_SAFE_TEST_STATE_GUARD;
    }

    ctl_fast_enable_output();
    if (!ctl_safe_bringup_pwm_all_tripped())
    {
        failures |= PSU_SAFE_TEST_PWM_TRIP;
    }

    // Restore a deterministic zero/off application state after destructive
    // invariant tests, then reinforce the PWM trip one more time.
    power_app_init();
    ctl_fast_disable_output();
    g_power_safe_bringup_self_test_failures = failures;
}
#endif

// Keep named task objects so startup/error handling never depends on a
// scheduler-list index. All tasks must be non-blocking.
static gmp_task_t task_dl_online =
    {"dl_online", tsk_dl_debug_device, 2, 0, 1, NULL};
static gmp_task_t task_flush_key =
    {"flush_key", tsk_key_flush,
     PSU_KEY_TASK_PERIOD_MS, PSU_KEY_TASK_INITIAL_DELAY_MS,
     PSU_ENABLE_HT16K33_KEY, (void*)&ht16k33};
static gmp_task_t task_oled_show =
    {"oled_show", oled_show_task,
     PSU_OLED_TASK_PERIOD_MS, PSU_OLED_TASK_INITIAL_DELAY_MS,
     PSU_ENABLE_OLED_DISPLAY, NULL};
static gmp_task_t task_flush_led =
    {"flush_led", tsk_LED_flush,
     PSU_HT16K33_DISPLAY_PERIOD_MS,
     PSU_HT16K33_DISPLAY_INITIAL_DELAY_MS,
     PSU_ENABLE_HT16K33_DISPLAY, (void*)&ht16k33};
static gmp_task_t task_fpga_test =
    {"fpga_test", fpga_test_task, 1000, 600, 0, NULL};
static gmp_task_t task_blink_led =
    {"blink_led", tsk_blink, 1000, 100, 1, NULL};
static gmp_task_t task_startup =
    {"startup", tsk_startup, 250, 0, 1, NULL};
static gmp_task_t task_power_app =
    {"power_app", tsk_power_app, 1, 0, 1, NULL};
static gmp_task_t task_power_mode_monitor =
    {"mode_monitor", power_mode_monitor_task,
     PSU_MODE_MONITOR_TASK_PERIOD_MS, 0,
     PSU_MODE_MONITOR_ENABLE, NULL};
static gmp_task_t task_power_control_policy =
{
    "control_policy",
    power_control_policy_task,
    PSU_CONTROL_POLICY_TASK_PERIOD_MS,
    0,
    PSU_CONTROL_POLICY_ENABLE,
    NULL
};
static gmp_task_t task_power_settings_store =
{
    "settings_store",
    power_settings_store_task,
    20U,
    0,
    1,
    NULL
};
static gmp_task_t task_power_command =
    {"power_cmd", tsk_power_debug_command, 10, 0,
     PSU_ENABLE_MANUAL_COMMAND, NULL};
static gmp_task_t task_power_console =
    {"power_console", tsk_power_console_status,
     PSU_PERIODIC_STATUS_PERIOD_MS, PSU_PERIODIC_STATUS_PERIOD_MS,
     PSU_ENABLE_CONSOLE_UI && PSU_ENABLE_PERIODIC_STATUS_LOG, NULL};

static gmp_task_t *const tasks[] = {
    &task_dl_online,
    &task_flush_key,
    &task_oled_show,
    &task_flush_led,
    &task_fpga_test,
    &task_blink_led,
    &task_startup,
    &task_analog_io_test,
    &task_rotary_encoder_ui,
    &task_power_app,
    &task_power_mode_monitor,
    &task_power_control_policy,
    &task_power_settings_store,
    &task_power_command,
    &task_power_console,
};

//=================================================================================================
// initialize routine

GMP_NO_OPT_PREFIX
void init(void) GMP_NO_OPT_SUFFIX
{
    int i;

    // SysConfig released the active-low reset during Board_init(). Wait once
    // before any FPGA SPI access, then capture the GPIO55 level for debug.
    DEVICE_DELAY_US(10000U);
    g_fpga_reset_level = (uint16_t)GPIO_readPin(IRIS_GPIO_SPI_RST);

    SPI_writeReg(0x01U, 0x000AU);
    DEVICE_DELAY_US(1000U);
    g_fpga_spi_r1_readback = SPI_readReg(0x01U);

    analog_io_test_init();
    rotary_encoder_ui_init();
    power_mode_monitor_init();
    power_control_policy_init();
    power_settings_store_init();

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
#if PSU_ENABLE_HT16K33_KEY && !PSU_ENABLE_HT16K33_DISPLAY
static void power_ui_clear_and_disable_ht16k33_display(void)
{
    uint16_t i;

    // Clear the controller's local shadow and the HT16K33 display RAM once.
    // Do not mark the shadow dirty: periodic display flushing is disabled.
    for (i = 0U; i < HT16K33_CFG_DISP_RAM_SIZE; ++i)
    {
        ht16k33.display_ram[i] = 0U;
    }

    ++g_ht16k33_clear_count;
    g_ht16k33_clear_result = gmp_hal_iic_write_mem(
        ht16k33.bus, ht16k33.dev_addr, 0x00U, 1U,
        ht16k33.display_ram, HT16K33_CFG_DISP_RAM_SIZE,
        HT16K33_CFG_TIMEOUT);

    // Fail dark: attempt DISPLAY OFF even if the preceding clear reports an
    // error. The oscillator remains enabled for key-matrix scanning.
    g_ht16k33_display_off_result = gmp_hal_iic_write_cmd(
        ht16k33.bus, ht16k33.dev_addr,
        HT16K33_REG_DISPLAY_SETUP, 1U, HT16K33_CFG_TIMEOUT);
    ht16k33.is_dirty = 0;
}
#endif

#if PSU_ENABLE_HT16K33_DISPLAY
static void power_ui_reassert_ht16k33_display_control(void)
{
    // Keep the raw result of each independent display-control transaction.
    g_ht16k33_osc_on_result = gmp_hal_iic_write_cmd(
        ht16k33.bus, ht16k33.dev_addr,
        HT16K33_CMD_OSC_ON, 1U, HT16K33_CFG_TIMEOUT);

    g_ht16k33_brightness_result = gmp_hal_iic_write_cmd(
        ht16k33.bus, ht16k33.dev_addr,
        HT16K33_REG_BRIGHTNESS | 0x0FU, 1U,
        HT16K33_CFG_TIMEOUT);

    g_ht16k33_display_on_result = gmp_hal_iic_write_cmd(
        ht16k33.bus, ht16k33.dev_addr,
        HT16K33_CMD_DISPLAY_ON, 1U, HT16K33_CFG_TIMEOUT);
}
#endif

gmp_task_status_t tsk_startup(gmp_task_t* tsk)
{
    ec_gt ec = GMP_EC_OK;

    g_ui_init_stage = 1U;
    g_ui_init_result = 0xFFFFU;

#if PSU_ENABLE_BEEP && !PSU_SAFE_BRINGUP
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

    if (beep_counter < 4)
    {
        return GMP_TASK_DONE;
    }
#endif

#if PSU_SAFE_BRINGUP && PSU_ENABLE_SAFE_SELF_TEST
    power_safe_bringup_run_self_test();
#endif

#if PSU_ENABLE_HT16K33_KEY || PSU_ENABLE_HT16K33_DISPLAY
    ht16k33_init_t ht16k33_init_struct = {.brightness = 15, .blink_rate = 0, .int_enable = 0, .int_act_high = 0};

    ec = ht16k33_init(&ht16k33, iic_bus, HT16K33_DEFAULT_DEV_ADDR, &ht16k33_init_struct);
#if PSU_ENABLE_HT16K33_KEY && !PSU_ENABLE_HT16K33_DISPLAY
    if (ec == GMP_EC_OK)
    {
        power_ui_clear_and_disable_ht16k33_display();
    }
    else
    {
        ht16k33.is_dirty = 0;
    }
#else
    if (ec == GMP_EC_OK)
    {
        power_ui_reassert_ht16k33_display_control();
        g_ht16k33_display_test_state = HT16K33_DISPLAY_TEST_PREPARE_ALL;
    }
    else
    {
        ht16k33.is_dirty = 0;
    }
#endif
    g_ui_init_stage = 2U;
    g_ui_init_result = (uint16_t)ec;

    if (ec != GMP_EC_OK)
    {
#if PSU_ENABLE_HT16K33_KEY
        task_flush_key.is_enabled = 0;
#endif
#if PSU_ENABLE_HT16K33_DISPLAY
        task_flush_led.is_enabled = 0;
#endif
    }
#endif

#if PSU_ENABLE_OLED_DISPLAY
    // The periodic OLED task owns all OLED I2C traffic. Startup only arms its
    // non-blocking power-wait state after the other peripherals are ready.
    g_oled_dynamic_update_enabled = 0U;
    g_oled_pending_mask = 0U;
    g_oled_probe_result = GMP_EC_NOT_READY;
    g_oled_last_result = GMP_EC_NOT_READY;
    g_oled_probe_ok_count = 0U;
    g_oled_probe_error_count = 0U;
    g_oled_init_state = OLED_INIT_WAIT_POWER;
    g_oled_init_retry_count = 0U;
    g_oled_init_attempt_count = 0U;
    g_oled_init_success_count = 0U;
    g_oled_init_failure_count = 0U;
    g_oled_clear_page_index = 0U;
    g_oled_deferred_for_key_count = 0U;
    g_oled_probe_3c_result = GMP_EC_NOT_READY;
    g_oled_probe_3d_result = GMP_EC_NOT_READY;
    g_oled_selected_address = 0U;
    g_oled_address_scan_count = 0U;
    g_oled_no_device_count = 0U;
    g_oled_reset_control_available = 0U;
    g_oled_power_control_available = 0U;
    g_oled_reset_count = 0U;
    g_oled_last_slave_address = 0U;
    g_oled_init_command_index = 0U;
    g_oled_init_command_result = GMP_EC_NOT_READY;
    g_oled_last_failed_command = 0U;
    g_oled_command_ok_count = 0U;
    g_oled_command_error_count = 0U;
    oled_set_active_address(0U);
    oled_reset_init_sequence();
    g_oled_next_action_tick =
        gmp_base_get_system_tick() + (time_gt)100U;
    g_key_i2c_holdoff_count = 0U;
#endif

    //        hdc1080_config_reg_t hdc1080_cfg = {.all = 0};
    //        hdc1080_cfg.bits.mode = 1; // continuous acquisition data
    //
    //        hdc1080_init(&hdc1080, iic_bus, HDC1080_I2C_ADDR_DEFAULT, hdc1080_cfg);
    //        hdc1080_trigger_temp_hum_sequence(&hdc1080);

    flag_init_cmpt = 1U;
    g_ui_init_stage = 3U;

#if PSU_ENABLE_STARTUP_LOG
    gmp_base_print(
        "UI ht=%u key=%u led=%u oled=%u\r\n",
        (unsigned int)g_ui_init_result,
        (unsigned int)task_flush_key.is_enabled,
        (unsigned int)task_flush_led.is_enabled,
        (unsigned int)task_oled_show.is_enabled);
#endif

    // startup process is complete.
    tsk->is_enabled = 0;

    return GMP_TASK_DONE;
}

//=================================================================================================
// endless loop routine

GMP_NO_OPT_PREFIX
void mainloop(void) GMP_NO_OPT_SUFFIX
{
    ++g_scheduler_loop_count;

    // run task scheduler
    gmp_scheduler_dispatch(&sched);
}

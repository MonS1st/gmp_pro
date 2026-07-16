//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of user objects in this file.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <core/dev/at_device.h>
#include <core/dev/datalink.h>

#include <core/pm/function_scheduler.h>

// peripheral
#include <core/dev/display/ht16k33.h>
#include <core/dev/gpio/pca9555.h>
#include <core/dev/sensor/hdc1080.h>

#include <stdbool.h>

#ifndef _FILE_USER_MAIN_H_
#define _FILE_USER_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

//=================================================================================================
// global variables

#define HT16K33_DISPLAY_TEST_WAIT_INIT     (0U)
#define HT16K33_DISPLAY_TEST_PREPARE_ALL   (1U)
#define HT16K33_DISPLAY_TEST_HOLD_ALL      (2U)
#define HT16K33_DISPLAY_TEST_RESTORE       (3U)
#define HT16K33_DISPLAY_TEST_NORMAL        (4U)

#define HT16K33_MAPPING_SCAN_COMMAND_NONE   (0U)
#define HT16K33_MAPPING_SCAN_COMMAND_START  (1U)
#define HT16K33_MAPPING_SCAN_COMMAND_ABORT  (2U)

#define HT16K33_MAPPING_SCAN_IDLE            (0U)
#define HT16K33_MAPPING_SCAN_POINT_PENDING   (1U)
#define HT16K33_MAPPING_SCAN_HOLD_POINT      (2U)
#define HT16K33_MAPPING_SCAN_RESTORE_PENDING (3U)

#define HT16K33_MAPPING_SCAN_POINT_COUNT   (128U)
#define HT16K33_MAPPING_SCAN_POINT_INVALID (0xFFFFU)

#define PSU_I2C_RECOVERY_IDLE               (0U)
#define PSU_I2C_RECOVERY_WAIT               (1U)
#define PSU_I2C_RECOVERY_BUS_CLEAR          (2U)
#define PSU_I2C_RECOVERY_RESET_MODULE       (3U)
#define PSU_I2C_RECOVERY_REINIT_HT          (4U)
#define PSU_I2C_RECOVERY_VERIFY             (5U)
#define PSU_I2C_RECOVERY_COMPLETE           (6U)

#ifndef SPECIFY_PC_TEST_ENV

#endif // SPECIFY_PC_TEST_ENV

extern const unsigned char led_lut[];

extern iic_halt iic_bus;
extern ht16k33_dev_t ht16k33;
extern hdc1080_dev_t hdc1080;

extern volatile uint16_t flag_init_cmpt;
extern volatile uint16_t g_power_safe_bringup_self_test_failures;
extern volatile uint32_t g_main_isr_count;
extern volatile uint32_t g_scheduler_loop_count;
extern volatile uint16_t g_ui_init_stage;
extern volatile uint16_t g_ui_init_result;
extern volatile uint16_t g_last_raw_key_id;
extern volatile ec_gt g_key_read_result;
extern volatile uint32_t g_key_read_ok_count;
extern volatile uint32_t g_key_read_error_count;
extern volatile uint16_t g_key_consecutive_error_count;
extern volatile uint16_t g_key_candidate_id;
extern volatile uint16_t g_key_candidate_count;
extern volatile uint16_t g_key_vote_id;
extern volatile uint16_t g_key_vote_count;
extern volatile uint16_t g_key_vote_age;
extern volatile uint32_t g_key_vote_timeout_count;
extern volatile uint16_t g_key_scan_ready;
extern volatile uint32_t g_key_confirmed_count;
extern volatile uint32_t g_key_action_count;
extern volatile uint16_t g_last_confirmed_key_id;
extern volatile uint16_t g_last_unmapped_key_id;
extern volatile uint32_t g_unmapped_confirmed_count;
extern volatile uint32_t g_key_first_sample_action_count;
extern volatile uint16_t g_key_ignore_scan_count;
extern volatile uint32_t g_led_update_count;
extern volatile ec_gt g_led_update_result;
extern volatile ec_gt g_ht16k33_clear_result;
extern volatile uint32_t g_ht16k33_clear_count;
extern volatile ec_gt g_ht16k33_display_off_result;
extern volatile ec_gt g_ht16k33_osc_on_result;
extern volatile ec_gt g_ht16k33_brightness_result;
extern volatile ec_gt g_ht16k33_display_on_result;
extern volatile ec_gt g_ht16k33_all_on_result;
extern volatile uint16_t g_ht16k33_display_test_state;
extern volatile uint32_t g_ht16k33_display_test_count;
extern volatile uint16_t g_ht16k33_mapping_scan_command;
extern volatile uint16_t g_ht16k33_mapping_scan_state;
extern volatile uint16_t g_ht16k33_mapping_scan_point;
extern volatile uint16_t g_ht16k33_mapping_scan_last_point;
extern volatile uint16_t g_ht16k33_mapping_scan_ram_address;
extern volatile uint16_t g_ht16k33_mapping_scan_bit_mask;
extern volatile uint32_t g_ht16k33_mapping_scan_step_ms;
extern volatile ec_gt g_ht16k33_mapping_scan_result;
extern volatile uint32_t g_ht16k33_mapping_scan_start_count;
extern volatile uint32_t g_ht16k33_mapping_scan_complete_count;
extern volatile uint32_t g_ht16k33_mapping_scan_abort_count;
extern volatile uint32_t g_ht16k33_mapping_scan_reject_count;
extern volatile uint32_t g_ht16k33_mapping_scan_update_count;
extern volatile uint32_t g_ht16k33_mapping_scan_restore_count;
extern volatile uint16_t g_oled_pending_mask;
extern volatile uint32_t g_oled_line_update_count;
extern volatile uint16_t g_key_i2c_holdoff_count;
extern volatile uint32_t g_key_i2c_holdoff_skip_count;
extern volatile uint16_t g_oled_dynamic_update_enabled;
extern volatile uint32_t g_oled_disabled_due_i2c_count;
extern volatile uint16_t g_key_consecutive_ok_count;
extern volatile uint16_t g_i2c_fault_active;
extern volatile uint16_t g_i2c_recovery_state;
extern volatile uint32_t g_i2c_recovery_attempt_count;
extern volatile uint32_t g_i2c_recovery_success_count;
extern volatile uint32_t g_i2c_recovery_failure_count;
extern volatile ec_gt g_i2c_recovery_ht_init_result;
extern volatile ec_gt g_i2c_recovery_verify_result;
extern volatile uint16_t g_i2c_recovery_verify_ok_count;
extern volatile uint16_t g_i2c_sda_level_before;
extern volatile uint16_t g_i2c_scl_level_before;
extern volatile uint16_t g_i2c_sda_level_after;
extern volatile uint16_t g_i2c_scl_level_after;
extern volatile uint32_t g_i2c_key_backoff_skip_count;
extern volatile time_gt g_i2c_recovery_next_tick;
extern volatile uint16_t g_i2c_clear_state;
extern volatile uint16_t g_i2c_clear_clock_count;
extern volatile uint32_t g_i2c_clear_attempt_count;
extern volatile uint32_t g_i2c_clear_success_count;
extern volatile uint32_t g_i2c_clear_failure_count;
extern volatile uint16_t g_i2c_clear_sda_initial;
extern volatile uint16_t g_i2c_clear_sda_final;
extern volatile uint16_t g_i2c_clear_scl_final;
extern volatile uint16_t g_i2c_clear_stop_generated;
extern volatile uint16_t g_i2c_clear_pinmux_restored;
extern volatile ec_gt g_oled_last_result;
extern volatile uint32_t g_oled_update_ok_count;
extern volatile uint32_t g_oled_update_error_count;
extern volatile uint16_t g_oled_init_state;
extern volatile uint16_t g_oled_init_retry_count;
extern volatile uint32_t g_oled_init_attempt_count;
extern volatile uint32_t g_oled_init_success_count;
extern volatile uint32_t g_oled_init_failure_count;
extern volatile uint16_t g_oled_clear_page_index;
extern volatile time_gt g_oled_next_action_tick;
extern volatile uint32_t g_oled_deferred_for_key_count;
extern volatile ec_gt g_oled_probe_3c_result;
extern volatile ec_gt g_oled_probe_3d_result;
extern volatile uint16_t g_oled_selected_address;
extern volatile uint32_t g_oled_address_scan_count;
extern volatile uint32_t g_oled_no_device_count;
extern volatile uint16_t g_oled_reset_control_available;
extern volatile uint16_t g_oled_power_control_available;
extern volatile uint32_t g_oled_reset_count;
extern volatile uint32_t g_blocked_output_request_count;
extern volatile uint16_t g_analog_board_voltage_meas_mv;
extern volatile uint16_t g_analog_board_current_meas_ma;
extern volatile uint16_t g_analog_board_real_feedback_active;
extern volatile uint32_t g_analog_board_feedback_update_count;
extern volatile uint16_t s_last_key_id;
extern volatile uint16_t s_key_release_count;

#define OLED_PENDING_VOLTAGE (1U << 0)
#define OLED_PENDING_CURRENT (1U << 1)
#define OLED_PENDING_KEY     (1U << 2)
#define OLED_PENDING_OUTPUT_STATE (1U << 3)
#define OLED_PENDING_MODE_STATUS (1U << 4)
#define OLED_PENDING_FEEDBACK_STATUS (1U << 5)

#define OLED_INIT_WAIT_POWER  (0U)
#define OLED_INIT_PROBE_3C    (1U)
#define OLED_INIT_PROBE_3D    (2U)
#define OLED_INIT_COMMANDS    (3U)
#define OLED_INIT_TEST        (4U)
#define OLED_INIT_CLEAR_PAGE  (5U)
#define OLED_INIT_TITLE       (6U)
#define OLED_INIT_READY       (7U)
#define OLED_INIT_RETRY_WAIT  (8U)

//=================================================================================================
// global functions

//
// User should implement this 3 functions at least
//
void init(void);
void mainloop(void);
void setup_peripheral(void);
void flush_dl_tx_buffer(void);
void flush_dl_rx_buffer(void);

//
// For Controller projects user should implement the following functions
//
void ctl_init(void);
void ctl_mainloop(void);

gmp_task_status_t tsk_startup(gmp_task_t* tsk);
gmp_task_status_t tsk_key_flush(gmp_task_t* tsk);
gmp_task_status_t tsk_LED_flush(gmp_task_t* tsk);
gmp_task_status_t fpga_test_task(gmp_task_t* tsk);
gmp_task_status_t oled_show_task(gmp_task_t* tsk);
bool power_ui_safe_bringup_self_test(void);
void power_ui_request_led_setpoint_update_from_command(void);

void update_led_content_8byte(ht16k33_dev_t* dev,
                              uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4,
                              uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8);


// peripheral function
void beep_on();
void beep_off();



#ifdef __cplusplus
}
#endif

#endif // _FILE_USER_MAIN_H_

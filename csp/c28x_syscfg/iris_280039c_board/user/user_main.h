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
extern volatile uint16_t g_key_scan_ready;
extern volatile uint32_t g_key_confirmed_count;
extern volatile uint32_t g_key_action_count;
extern volatile uint16_t g_key_ignore_scan_count;
extern volatile uint32_t g_led_update_count;
extern volatile ec_gt g_led_update_result;
extern volatile uint32_t g_blocked_output_request_count;
extern volatile uint16_t s_last_key_id;
extern volatile uint16_t s_key_release_count;

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

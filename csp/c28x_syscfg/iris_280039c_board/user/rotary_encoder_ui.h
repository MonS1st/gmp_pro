#ifndef ROTARY_ENCODER_UI_H
#define ROTARY_ENCODER_UI_H

#include <gmp_core.h>
#include <core/pm/function_scheduler.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile uint16_t g_encoder_ui_enabled;
extern volatile uint16_t g_encoder_ui_mode;

extern volatile uint32_t g_encoder_raw_position;
extern volatile int32_t g_encoder_last_delta;
extern volatile int32_t g_encoder_detent_accumulator;

extern volatile uint32_t g_encoder_clockwise_count;
extern volatile uint32_t g_encoder_counterclockwise_count;
extern volatile uint32_t g_encoder_invalid_transition_count;

extern volatile uint16_t g_encoder_button_raw;
extern volatile uint16_t g_encoder_button_pressed;
extern volatile uint16_t g_encoder_button_debounce_count;
extern volatile uint32_t g_encoder_button_press_count;
extern volatile uint32_t g_encoder_mode_switch_count;
extern volatile uint32_t g_encoder_fault_reset_request_count;
extern volatile uint32_t g_encoder_fault_reset_success_count;
extern volatile uint32_t g_encoder_fault_reset_reject_count;
extern volatile uint16_t g_encoder_fault_reset_pending;

extern volatile uint32_t g_encoder_voltage_update_count;
extern volatile uint32_t g_encoder_current_update_count;
extern volatile uint32_t g_encoder_reject_count;

extern gmp_task_t task_rotary_encoder_ui;

void rotary_encoder_ui_init(void);
gmp_task_status_t rotary_encoder_ui_task(gmp_task_t* tsk);

#ifdef __cplusplus
}
#endif

#endif // ROTARY_ENCODER_UI_H

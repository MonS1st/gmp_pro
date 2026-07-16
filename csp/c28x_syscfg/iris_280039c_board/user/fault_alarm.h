#ifndef FAULT_ALARM_H
#define FAULT_ALARM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    PSU_FAULT_ALARM_REASON_NONE = 0U,
    PSU_FAULT_ALARM_REASON_OVP = 1U,
    PSU_FAULT_ALARM_REASON_OCP = 2U,
    PSU_FAULT_ALARM_REASON_OVP_OCP = 3U
} psu_fault_alarm_reason_t;

extern volatile uint16_t g_fault_alarm_led_active;
extern volatile uint16_t g_fault_alarm_buzzer_active;
extern volatile uint16_t g_fault_alarm_reason;
extern volatile uint16_t g_fault_alarm_event_issued;
extern volatile uint32_t g_fault_alarm_pulse_count;
extern volatile uint32_t g_fault_alarm_clear_count;
extern volatile uint32_t g_fault_alarm_start_tick;
extern volatile uint32_t g_fault_alarm_last_duration_ms;

extern volatile uint16_t g_fault_alarm_hardware_ready;
extern volatile uint16_t g_fault_alarm_led_hardware_ready;
extern volatile uint16_t g_fault_alarm_buzzer_hardware_ready;
extern volatile uint16_t g_fault_alarm_selected_led_number;
extern volatile uint16_t g_fault_alarm_led_gpio_number;
extern volatile uint16_t g_fault_alarm_buzzer_type;

extern volatile uint16_t g_fault_alarm_led_apply_pending;
extern volatile uint16_t g_fault_alarm_buzzer_start_pending;
extern volatile uint16_t g_fault_alarm_buzzer_stop_pending;

extern volatile uint16_t g_fault_alarm_led_test_command;
extern volatile uint32_t g_fault_alarm_led_test_count;
extern volatile uint16_t g_fault_alarm_buzzer_test_command;
extern volatile uint32_t g_fault_alarm_buzzer_test_count;

void fault_alarm_init(void);
void fault_alarm_on_fault_latched(psu_fault_alarm_reason_t reason);
void fault_alarm_on_fault_cleared(void);
void fault_alarm_task(uint32_t current_tick);

#ifdef __cplusplus
}
#endif

#endif // FAULT_ALARM_H

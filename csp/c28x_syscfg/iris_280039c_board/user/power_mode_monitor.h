#ifndef POWER_MODE_MONITOR_H
#define POWER_MODE_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#include <gmp_core.h>
#include <core/pm/function_scheduler.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    PSU_MODE_DISPLAY_OFF = 0,
    PSU_MODE_DISPLAY_START,
    PSU_MODE_DISPLAY_NO_FEEDBACK,
    PSU_MODE_DISPLAY_CV,
    PSU_MODE_DISPLAY_CC,
    PSU_MODE_DISPLAY_FAULT
} psu_mode_display_t;

extern volatile uint16_t g_mode_monitor_enabled;
extern volatile uint16_t g_mode_monitor_feedback_valid;
extern volatile uint16_t g_mode_monitor_display_mode;

extern volatile uint16_t g_mode_monitor_test_enable;
extern volatile uint16_t g_mode_monitor_test_voltage_mv;
extern volatile uint16_t g_mode_monitor_test_current_ma;
extern volatile uint16_t g_mode_monitor_using_test_data;

extern volatile uint16_t g_mode_monitor_voltage_candidate;
extern volatile uint16_t g_mode_monitor_current_candidate;
extern volatile uint16_t g_mode_monitor_cv_confirm_count;
extern volatile uint16_t g_mode_monitor_cc_confirm_count;

extern volatile uint32_t g_mode_monitor_update_count;
extern volatile uint32_t g_mode_monitor_cv_enter_count;
extern volatile uint32_t g_mode_monitor_cc_enter_count;
extern volatile uint32_t g_mode_monitor_no_feedback_count;
extern volatile uint32_t g_mode_monitor_reset_count;

void power_mode_monitor_init(void);
gmp_task_status_t power_mode_monitor_task(gmp_task_t *tsk);
const char *power_mode_monitor_text(psu_mode_display_t mode);
bool power_mode_monitor_feedback_valid(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_MODE_MONITOR_H

#ifndef POWER_CONTROL_POLICY_H
#define POWER_CONTROL_POLICY_H

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
    PSU_CONTROL_STRATEGY_AUTO = 0,
    PSU_CONTROL_STRATEGY_CV_ONLY,
    PSU_CONTROL_STRATEGY_CC_ONLY
} psu_control_strategy_t;

typedef enum
{
    PSU_POLICY_FAULT_NONE = 0,
    PSU_POLICY_FAULT_CV_REGULATION_LOST,
    PSU_POLICY_FAULT_CC_REGULATION_LOST
} psu_policy_fault_t;

extern volatile uint16_t g_control_policy_enabled;
extern volatile uint16_t g_control_strategy;

extern volatile uint16_t g_control_policy_fault;
extern volatile uint16_t g_control_policy_fault_latched;

extern volatile uint16_t g_control_policy_mismatch_candidate;
extern volatile uint16_t g_control_policy_mismatch_confirm_count;

extern volatile uint16_t g_control_policy_target_active;
extern volatile uint16_t g_control_policy_using_test_mode;

extern volatile uint16_t g_control_policy_reset_requested;

extern volatile uint16_t g_control_policy_test_trip_enable;

extern volatile uint16_t g_control_policy_trip_actual_mode;
extern volatile uint16_t g_control_policy_trip_strategy;

extern volatile uint16_t g_control_policy_trip_voltage_set_mv;
extern volatile uint16_t g_control_policy_trip_current_set_ma;
extern volatile uint16_t g_control_policy_trip_voltage_meas_mv;
extern volatile uint16_t g_control_policy_trip_current_meas_ma;

extern volatile uint32_t g_control_policy_strategy_change_count;
extern volatile uint32_t g_control_policy_strategy_reject_count;

extern volatile uint32_t g_control_policy_mismatch_count;
extern volatile uint32_t g_control_policy_cv_lost_count;
extern volatile uint32_t g_control_policy_cc_lost_count;

extern volatile uint32_t g_control_policy_fault_shutdown_count;
extern volatile uint32_t g_control_policy_reset_count;
extern volatile uint32_t g_control_policy_reset_reject_count;

void power_control_policy_init(void);
gmp_task_status_t power_control_policy_task(gmp_task_t *tsk);

bool power_control_policy_set_strategy(psu_control_strategy_t strategy);
psu_control_strategy_t power_control_policy_get_strategy(void);

const char *power_control_policy_strategy_text(
    psu_control_strategy_t strategy);
const char *power_control_policy_fault_text(psu_policy_fault_t fault);

bool power_control_policy_fault_active(void);
bool power_control_policy_output_start_allowed(void);
void power_control_policy_request_reset(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_CONTROL_POLICY_H

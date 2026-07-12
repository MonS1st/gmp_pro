#ifndef POWER_APP_H
#define POWER_APP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    POWER_STATE_OFF = 0,
    POWER_STATE_STARTING,
    POWER_STATE_CV,
    POWER_STATE_CC,
    POWER_STATE_FAULT
} power_state_t;

typedef enum
{
    POWER_FAULT_NONE = 0,
    POWER_FAULT_OVERVOLTAGE,
    POWER_FAULT_OVERCURRENT
} power_fault_t;

typedef struct
{
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    uint16_t voltage_meas_mv;
    uint16_t current_meas_ma;
    uint16_t dac_voltage_code;
    uint16_t dac_current_code;
    uint16_t cc_confirm_count;
    uint16_t cv_confirm_count;
    power_state_t state;
    power_fault_t fault;
    bool output_requested;
    bool output_enabled;
    bool fault_latched;
} power_app_t;

extern volatile power_app_t g_power_app;

void power_app_init(void);
// UI and communication code should use these APIs instead of writing g_power_app directly.
void power_app_set_voltage_mv(uint16_t voltage_mv);
void power_app_set_current_ma(uint16_t current_ma);
uint16_t power_app_get_voltage_mv(void);
uint16_t power_app_get_current_ma(void);
void power_app_request_output(bool enable);
void power_app_reset_fault(void);
void power_app_fast_step(void);
void power_app_slow_step(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_APP_H

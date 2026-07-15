#ifndef POWER_SETTINGS_STORE_H
#define POWER_SETTINGS_STORE_H

#include <stdint.h>

#include <gmp_core.h>
#include <core/pm/function_scheduler.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    uint16_t control_strategy;
    uint16_t checksum;
} psu_saved_settings_t;

typedef enum
{
    PSU_SETTINGS_RESULT_IDLE = 0,
    PSU_SETTINGS_RESULT_LOAD_OK,
    PSU_SETTINGS_RESULT_SAVE_OK,
    PSU_SETTINGS_RESULT_INVALID_DATA,
    PSU_SETTINGS_RESULT_FLASH_ERROR,
    PSU_SETTINGS_RESULT_SAVE_REJECTED
} psu_settings_result_t;

extern volatile uint16_t g_settings_valid;
extern volatile uint16_t g_settings_save_request;
extern volatile uint16_t g_settings_result;
extern volatile uint16_t g_settings_loaded;

void power_settings_store_init(void);
void power_settings_store_request_save(void);
gmp_task_status_t power_settings_store_task(gmp_task_t *tsk);

#ifdef __cplusplus
}
#endif

#endif // POWER_SETTINGS_STORE_H

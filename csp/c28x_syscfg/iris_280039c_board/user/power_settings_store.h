#ifndef POWER_SETTINGS_STORE_H
#define POWER_SETTINGS_STORE_H

#include <stdbool.h>
#include <stdint.h>

#include <gmp_core.h>
#include <core/pm/function_scheduler.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define PSU_PRESET_COUNT (3U)

typedef struct
{
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    uint16_t control_strategy;
} psu_preset_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t active_preset_index;
    psu_preset_t presets[PSU_PRESET_COUNT];
    uint16_t checksum;
    uint16_t reserved[2];
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
extern volatile uint16_t g_settings_result;
extern volatile uint16_t g_settings_loaded;

extern volatile uint16_t g_active_preset_index;
extern volatile uint16_t g_preset_1_voltage_mv;
extern volatile uint16_t g_preset_1_current_ma;
extern volatile uint16_t g_preset_1_strategy;
extern volatile uint16_t g_preset_2_voltage_mv;
extern volatile uint16_t g_preset_2_current_ma;
extern volatile uint16_t g_preset_2_strategy;
extern volatile uint16_t g_preset_3_voltage_mv;
extern volatile uint16_t g_preset_3_current_ma;
extern volatile uint16_t g_preset_3_strategy;
extern volatile uint32_t g_preset_switch_count;
extern volatile uint32_t g_preset_switch_reject_count;
extern volatile uint32_t g_preset_capture_count;
extern volatile uint16_t g_presets_dirty;
extern volatile uint32_t g_preset_auto_save_count;
extern volatile uint32_t g_preset_auto_save_failure_count;
extern volatile uint32_t g_preset_auto_save_deferred_count;

void power_settings_store_init(void);
// Called only after a user setter has made a real value change.
void power_presets_capture_user_change(void);
bool power_presets_select(uint16_t preset_index);
gmp_task_status_t power_settings_store_task(gmp_task_t *tsk);

#ifdef __cplusplus
}
#endif

#endif // POWER_SETTINGS_STORE_H

#include "power_settings_store.h"

#include <stdbool.h>

#include <ctrl_settings.h>
#include <device.h>
#include <driverlib.h>
#include <F021_F28003x_C28x.h>

#include "analog_io_test.h"
#include "power_app.h"
#include "power_control_policy.h"
#include "power_hal.h"

#define PSU_SETTINGS_MAGIC                       (0x50535531UL)
#define PSU_SETTINGS_VERSION                     (2U)
#define PSU_SETTINGS_VERSION_V1                  (1U)
#define PSU_SETTINGS_FLASH_ADDRESS               (0x000AE000UL)
#define PSU_SETTINGS_FLASH_SECTOR_16BIT_WORDS    (0x00001000UL)
#define PSU_SETTINGS_FLASH_BLANK_CHECK_32BIT_WORDS (0x00000800UL)
#define PSU_SETTINGS_PROGRAM_WORDS               (16U)
#define PSU_SETTINGS_PROGRAM_CHUNK_WORDS         (8U)
#define PSU_PRESET_AUTO_SAVE_RETRY_DELAY_MS      (2000U)

#if PSU_SETTINGS_PROGRAM_WORDS > PSU_SETTINGS_FLASH_SECTOR_16BIT_WORDS
#error "PSU settings record exceeds the reserved Flash sector"
#endif

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    uint16_t control_strategy;
    uint16_t checksum;
} psu_saved_settings_v1_t;

volatile uint16_t g_settings_valid = 0U;
volatile uint16_t g_settings_result = PSU_SETTINGS_RESULT_IDLE;
volatile uint16_t g_settings_loaded = 0U;

volatile uint16_t g_active_preset_index = 0U;
volatile uint16_t g_preset_1_voltage_mv = 0U;
volatile uint16_t g_preset_1_current_ma = 0U;
volatile uint16_t g_preset_1_strategy = PSU_CONTROL_STRATEGY_AUTO;
volatile uint16_t g_preset_2_voltage_mv = 0U;
volatile uint16_t g_preset_2_current_ma = 0U;
volatile uint16_t g_preset_2_strategy = PSU_CONTROL_STRATEGY_AUTO;
volatile uint16_t g_preset_3_voltage_mv = 0U;
volatile uint16_t g_preset_3_current_ma = 0U;
volatile uint16_t g_preset_3_strategy = PSU_CONTROL_STRATEGY_AUTO;
volatile uint32_t g_preset_switch_count = 0U;
volatile uint32_t g_preset_switch_reject_count = 0U;
volatile uint32_t g_preset_capture_count = 0U;
volatile uint16_t g_presets_dirty = 0U;
volatile uint32_t g_preset_auto_save_count = 0U;
volatile uint32_t g_preset_auto_save_failure_count = 0U;
volatile uint32_t g_preset_auto_save_deferred_count = 0U;

static psu_preset_t s_presets[PSU_PRESET_COUNT];
static uint16_t s_active_preset_index;
static bool s_preset_apply_in_progress;
static bool s_flash_operation_in_progress;
static bool s_auto_save_deferred_recorded;
static bool s_auto_save_retry_pending;
static time_gt s_preset_last_change_tick;
static time_gt s_auto_save_retry_tick;

static void power_presets_sync_diagnostics(void)
{
    g_active_preset_index = s_active_preset_index;
    g_preset_1_voltage_mv = s_presets[0].voltage_set_mv;
    g_preset_1_current_ma = s_presets[0].current_set_ma;
    g_preset_1_strategy = s_presets[0].control_strategy;
    g_preset_2_voltage_mv = s_presets[1].voltage_set_mv;
    g_preset_2_current_ma = s_presets[1].current_set_ma;
    g_preset_2_strategy = s_presets[1].control_strategy;
    g_preset_3_voltage_mv = s_presets[2].voltage_set_mv;
    g_preset_3_current_ma = s_presets[2].current_set_ma;
    g_preset_3_strategy = s_presets[2].control_strategy;
}

static void power_presets_mark_dirty(void)
{
    g_presets_dirty = 1U;
    g_settings_result = PSU_SETTINGS_RESULT_IDLE;
    s_preset_last_change_tick = gmp_base_get_system_tick();
    s_auto_save_deferred_recorded = false;
    s_auto_save_retry_pending = false;
}

static void power_presets_capture_active(void)
{
    psu_preset_t *preset;
    uint16_t voltage_set_mv;
    uint16_t current_set_ma;
    uint16_t control_strategy;

    if (s_preset_apply_in_progress ||
        (s_active_preset_index >= PSU_PRESET_COUNT))
    {
        return;
    }

    preset = &s_presets[s_active_preset_index];
    voltage_set_mv = g_user_voltage_set_mv;
    current_set_ma = g_user_current_set_ma;
    control_strategy = g_control_strategy;
    if ((preset->voltage_set_mv == voltage_set_mv) &&
        (preset->current_set_ma == current_set_ma) &&
        (preset->control_strategy == control_strategy))
    {
        return;
    }

    preset->voltage_set_mv = voltage_set_mv;
    preset->current_set_ma = current_set_ma;
    preset->control_strategy = control_strategy;
    power_presets_sync_diagnostics();
    power_presets_mark_dirty();
    ++g_preset_capture_count;
}

void power_presets_capture_user_change(void)
{
    power_presets_capture_active();
}

static uint16_t power_settings_checksum_v1(
    const psu_saved_settings_v1_t *settings)
{
    uint16_t checksum = 0xA55AU;

    checksum ^= (uint16_t)(settings->magic & 0xFFFFUL);
    checksum ^= (uint16_t)(settings->magic >> 16U);
    checksum ^= settings->version;
    checksum ^= settings->voltage_set_mv;
    checksum ^= settings->current_set_ma;
    checksum ^= settings->control_strategy;
    return checksum;
}

static uint16_t power_settings_checksum_v2(
    const psu_saved_settings_t *settings)
{
    uint16_t checksum = 0xA55AU;
    uint16_t i;

    checksum ^= (uint16_t)(settings->magic & 0xFFFFUL);
    checksum ^= (uint16_t)(settings->magic >> 16U);
    checksum ^= settings->version;
    checksum ^= settings->active_preset_index;
    for (i = 0U; i < PSU_PRESET_COUNT; ++i)
    {
        checksum ^= settings->presets[i].voltage_set_mv;
        checksum ^= settings->presets[i].current_set_ma;
        checksum ^= settings->presets[i].control_strategy;
    }
    return checksum;
}

static void power_settings_read_v1(psu_saved_settings_v1_t *settings)
{
    volatile const uint16_t *flash_words =
        (volatile const uint16_t *)PSU_SETTINGS_FLASH_ADDRESS;

    settings->magic = (uint32_t)flash_words[0] |
                      ((uint32_t)flash_words[1] << 16U);
    settings->version = flash_words[2];
    settings->voltage_set_mv = flash_words[3];
    settings->current_set_ma = flash_words[4];
    settings->control_strategy = flash_words[5];
    settings->checksum = flash_words[6];
}

static void power_settings_read_v2(psu_saved_settings_t *settings)
{
    volatile const uint16_t *flash_words =
        (volatile const uint16_t *)PSU_SETTINGS_FLASH_ADDRESS;
    uint16_t i;
    uint16_t word_index = 4U;

    settings->magic = (uint32_t)flash_words[0] |
                      ((uint32_t)flash_words[1] << 16U);
    settings->version = flash_words[2];
    settings->active_preset_index = flash_words[3];
    for (i = 0U; i < PSU_PRESET_COUNT; ++i)
    {
        settings->presets[i].voltage_set_mv = flash_words[word_index++];
        settings->presets[i].current_set_ma = flash_words[word_index++];
        settings->presets[i].control_strategy = flash_words[word_index++];
    }
    settings->checksum = flash_words[13];
    settings->reserved[0] = flash_words[14];
    settings->reserved[1] = flash_words[15];
}

static bool power_settings_preset_valid(const psu_preset_t *preset)
{
    return (preset->voltage_set_mv <= PSU_COMMAND_VOLTAGE_LIMIT_MV) &&
           (preset->current_set_ma <= PSU_COMMAND_CURRENT_LIMIT_MA) &&
           (preset->control_strategy <=
            (uint16_t)PSU_CONTROL_STRATEGY_CC_ONLY);
}

static bool power_settings_v1_valid(
    const psu_saved_settings_v1_t *settings)
{
    return (settings->magic == PSU_SETTINGS_MAGIC) &&
           (settings->version == PSU_SETTINGS_VERSION_V1) &&
           (settings->checksum == power_settings_checksum_v1(settings)) &&
           (settings->voltage_set_mv <= PSU_COMMAND_VOLTAGE_LIMIT_MV) &&
           (settings->current_set_ma <= PSU_COMMAND_CURRENT_LIMIT_MA) &&
           (settings->control_strategy <=
            (uint16_t)PSU_CONTROL_STRATEGY_CC_ONLY);
}

static bool power_settings_v2_valid(const psu_saved_settings_t *settings)
{
    uint16_t i;

    if ((settings->magic != PSU_SETTINGS_MAGIC) ||
        (settings->version != PSU_SETTINGS_VERSION) ||
        (settings->active_preset_index >= PSU_PRESET_COUNT) ||
        (settings->checksum != power_settings_checksum_v2(settings)))
    {
        return false;
    }

    for (i = 0U; i < PSU_PRESET_COUNT; ++i)
    {
        if (!power_settings_preset_valid(&settings->presets[i]))
        {
            return false;
        }
    }
    return true;
}

static void power_settings_force_output_off(void)
{
    g_output_switch_requested = 0U;
    g_output_switch_active = 0U;
    g_output_switch_precharge_active = 0U;
    g_output_switch_precharge_complete = 0U;
    g_output_switch_dac_gate_active = 0U;
    g_output_switch_relay_command = 0U;
    g_power_app.output_requested = false;
    g_power_app.output_enabled = false;
    g_power_app.state = POWER_STATE_OFF;
    power_output_hw_set(false);
    power_relay_cutoff_epwm_set(true);
    analog_io_test_force_safe_outputs();
}

static bool power_settings_safe_state(void)
{
    return (g_power_app.state == POWER_STATE_OFF) &&
           !g_power_app.output_requested &&
           !g_power_app.output_enabled &&
           !g_power_app.fault_latched &&
           (g_power_app.fault == POWER_FAULT_NONE) &&
           (g_analog_board_feedback_fault == 0U) &&
           (g_analog_board_fault_hold_active == 0U) &&
           (g_analog_board_fault_shutdown_active == 0U) &&
           !power_control_policy_fault_active() &&
           (g_output_switch_requested == 0U) &&
           (g_output_switch_active == 0U) &&
           (g_output_switch_precharge_active == 0U) &&
           (g_analog_board_iset_precharge_active == 0U) &&
           (g_output_switch_dac_gate_active == 0U) &&
           (g_output_switch_relay_command == 0U);
}

static bool power_settings_save_allowed(void)
{
    return !s_flash_operation_in_progress && power_settings_safe_state();
}

static bool power_presets_apply(uint16_t preset_index)
{
    uint16_t old_preset_index = s_active_preset_index;
    uint16_t old_voltage_set_mv = g_user_voltage_set_mv;
    uint16_t old_current_set_ma = g_user_current_set_ma;
    psu_control_strategy_t old_strategy =
        power_control_policy_get_strategy();
    const psu_preset_t *preset;

    if (preset_index >= PSU_PRESET_COUNT)
    {
        return false;
    }

    preset = &s_presets[preset_index];
    s_preset_apply_in_progress = true;
    s_active_preset_index = preset_index;
    power_app_restore_user_setpoints(preset->voltage_set_mv,
                                     preset->current_set_ma);
    if (!power_control_policy_set_strategy(
            (psu_control_strategy_t)preset->control_strategy))
    {
        s_active_preset_index = old_preset_index;
        power_app_restore_user_setpoints(old_voltage_set_mv,
                                         old_current_set_ma);
        (void)power_control_policy_set_strategy(old_strategy);
        s_preset_apply_in_progress = false;
        power_presets_sync_diagnostics();
        return false;
    }
    power_app_update_effective_setpoints();
    s_preset_apply_in_progress = false;
    power_presets_sync_diagnostics();
    return true;
}

bool power_presets_select(uint16_t preset_index)
{
    if ((preset_index >= PSU_PRESET_COUNT) ||
        !power_settings_safe_state())
    {
        ++g_preset_switch_reject_count;
        return false;
    }

    power_presets_capture_active();
    if (preset_index == s_active_preset_index)
    {
        return true;
    }

    power_settings_force_output_off();
    if (!power_presets_apply(preset_index))
    {
        ++g_preset_switch_reject_count;
        power_settings_force_output_off();
        return false;
    }

    power_settings_force_output_off();
    power_presets_mark_dirty();
    ++g_preset_switch_count;
    return true;
}

#ifndef __cplusplus
#pragma CODE_SECTION(power_settings_flash_write, ".TI.ramfunc");
#endif
static bool power_settings_flash_write(const uint16_t *program_words)
{
    Fapi_StatusType status;
    Fapi_FlashStatusWordType flash_status_word;
    bool interrupts_were_disabled;
    bool success = true;
    uint16_t offset;

    interrupts_were_disabled = Interrupt_disableGlobal();

    status = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS,
                                DEVICE_SYSCLK_FREQ / 1000000UL);
    if (status != Fapi_Status_Success)
    {
        success = false;
    }

    if (success)
    {
        Flash_disablePrefetch(FLASH0CTRL_BASE);
        FLASH_DELAY_CONFIG;
        status = Fapi_setActiveFlashBank(Fapi_FlashBank2);
        Flash_enablePrefetch(FLASH0CTRL_BASE);
        FLASH_DELAY_CONFIG;
        if (status != Fapi_Status_Success)
        {
            success = false;
        }
    }

    if (success)
    {
        status = Fapi_issueAsyncCommand(Fapi_ClearMore);
        while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
        {
        }
        if ((status != Fapi_Status_Success) ||
            (Fapi_getFsmStatus() != 0U))
        {
            success = false;
        }
    }

    if (success)
    {
        status = Fapi_issueAsyncCommandWithAddress(
            Fapi_EraseSector, (uint32 *)PSU_SETTINGS_FLASH_ADDRESS);
        while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
        {
        }
        if ((status != Fapi_Status_Success) ||
            (Fapi_getFsmStatus() != 0U))
        {
            success = false;
        }
    }

    if (success)
    {
        status = Fapi_doBlankCheck(
            (uint32 *)PSU_SETTINGS_FLASH_ADDRESS,
            PSU_SETTINGS_FLASH_BLANK_CHECK_32BIT_WORDS,
            &flash_status_word);
        if (status != Fapi_Status_Success)
        {
            success = false;
        }
    }

    for (offset = 0U;
         success && (offset < PSU_SETTINGS_PROGRAM_WORDS);
         offset += PSU_SETTINGS_PROGRAM_CHUNK_WORDS)
    {
        status = Fapi_issueProgrammingCommand(
            (uint32 *)(PSU_SETTINGS_FLASH_ADDRESS + (uint32_t)offset),
            (uint16 *)&program_words[offset],
            PSU_SETTINGS_PROGRAM_CHUNK_WORDS,
            0,
            0U,
            Fapi_AutoEccGeneration);
        while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
        {
        }
        if ((status != Fapi_Status_Success) ||
            (Fapi_getFsmStatus() != 0U))
        {
            success = false;
        }
    }

    if (success)
    {
        status = Fapi_doVerifyBy16bits(
            (uint16 *)PSU_SETTINGS_FLASH_ADDRESS,
            PSU_SETTINGS_PROGRAM_WORDS,
            (uint16 *)program_words,
            &flash_status_word);
        if (status != Fapi_Status_Success)
        {
            success = false;
        }
    }

    if (!interrupts_were_disabled)
    {
        (void)Interrupt_enableGlobal();
    }
    return success;
}

static void power_presets_load_defaults(void)
{
    psu_preset_t defaults;
    uint16_t i;

    defaults.voltage_set_mv = g_user_voltage_set_mv;
    defaults.current_set_ma = g_user_current_set_ma;
    defaults.control_strategy = g_control_strategy;
    for (i = 0U; i < PSU_PRESET_COUNT; ++i)
    {
        s_presets[i] = defaults;
    }
    s_active_preset_index = 0U;
    power_presets_sync_diagnostics();
}

static void power_presets_load_v2(const psu_saved_settings_t *settings)
{
    uint16_t i;

    for (i = 0U; i < PSU_PRESET_COUNT; ++i)
    {
        s_presets[i] = settings->presets[i];
    }
    s_active_preset_index = settings->active_preset_index;
    power_presets_sync_diagnostics();
}

static void power_presets_import_v1(
    const psu_saved_settings_v1_t *settings)
{
    power_presets_load_defaults();
    s_presets[0].voltage_set_mv = settings->voltage_set_mv;
    s_presets[0].current_set_ma = settings->current_set_ma;
    s_presets[0].control_strategy = settings->control_strategy;
    s_active_preset_index = 0U;
    power_presets_sync_diagnostics();
}

void power_settings_store_init(void)
{
    psu_saved_settings_t settings_v2;
    psu_saved_settings_v1_t settings_v1;

    g_settings_valid = 0U;
    g_settings_loaded = 0U;
    g_settings_result = PSU_SETTINGS_RESULT_INVALID_DATA;
    g_presets_dirty = 0U;
    g_preset_switch_count = 0U;
    g_preset_switch_reject_count = 0U;
    g_preset_capture_count = 0U;
    g_preset_auto_save_count = 0U;
    g_preset_auto_save_failure_count = 0U;
    g_preset_auto_save_deferred_count = 0U;
    s_preset_apply_in_progress = false;
    s_flash_operation_in_progress = false;
    s_auto_save_deferred_recorded = false;
    s_auto_save_retry_pending = false;
    s_preset_last_change_tick = gmp_base_get_system_tick();
    s_auto_save_retry_tick = s_preset_last_change_tick;

    power_settings_force_output_off();
    power_presets_load_defaults();
    power_settings_read_v2(&settings_v2);
    if (power_settings_v2_valid(&settings_v2))
    {
        power_presets_load_v2(&settings_v2);
        if (power_presets_apply(settings_v2.active_preset_index))
        {
            g_settings_valid = 1U;
            g_settings_loaded = 1U;
            g_settings_result = PSU_SETTINGS_RESULT_LOAD_OK;
        }
        else
        {
            power_presets_load_defaults();
        }
    }
    else
    {
        power_settings_read_v1(&settings_v1);
        if (power_settings_v1_valid(&settings_v1))
        {
            power_presets_import_v1(&settings_v1);
            if (power_presets_apply(0U))
            {
                g_settings_valid = 1U;
                g_settings_loaded = 1U;
                g_settings_result = PSU_SETTINGS_RESULT_LOAD_OK;
                power_presets_mark_dirty();
            }
        }
    }

    power_presets_sync_diagnostics();
    power_settings_force_output_off();
}

static void power_settings_make_v2_record(psu_saved_settings_t *settings)
{
    uint16_t i;

    settings->magic = PSU_SETTINGS_MAGIC;
    settings->version = PSU_SETTINGS_VERSION;
    settings->active_preset_index = s_active_preset_index;
    for (i = 0U; i < PSU_PRESET_COUNT; ++i)
    {
        settings->presets[i] = s_presets[i];
    }
    settings->reserved[0] = 0xFFFFU;
    settings->reserved[1] = 0xFFFFU;
    settings->checksum = power_settings_checksum_v2(settings);
}

static void power_settings_serialize_v2(
    const psu_saved_settings_t *settings,
    uint16_t *program_words)
{
    uint16_t i;
    uint16_t word_index = 4U;

    program_words[0] = (uint16_t)(settings->magic & 0xFFFFUL);
    program_words[1] = (uint16_t)(settings->magic >> 16U);
    program_words[2] = settings->version;
    program_words[3] = settings->active_preset_index;
    for (i = 0U; i < PSU_PRESET_COUNT; ++i)
    {
        program_words[word_index++] = settings->presets[i].voltage_set_mv;
        program_words[word_index++] = settings->presets[i].current_set_ma;
        program_words[word_index++] = settings->presets[i].control_strategy;
    }
    program_words[13] = settings->checksum;
    program_words[14] = settings->reserved[0];
    program_words[15] = settings->reserved[1];
}

gmp_task_status_t power_settings_store_task(gmp_task_t *tsk)
{
    psu_saved_settings_t settings;
    uint16_t program_words[PSU_SETTINGS_PROGRAM_WORDS];

    GMP_UNUSED_VAR(tsk);

    if ((g_presets_dirty == 0U) ||
        !gmp_base_is_delay_elapsed(s_preset_last_change_tick,
                                   PSU_PRESET_AUTO_SAVE_DELAY_MS))
    {
        return GMP_TASK_DONE;
    }

    if (s_auto_save_retry_pending &&
        !gmp_base_is_delay_elapsed(s_auto_save_retry_tick,
                                   PSU_PRESET_AUTO_SAVE_RETRY_DELAY_MS))
    {
        return GMP_TASK_DONE;
    }

    if (!power_settings_save_allowed())
    {
        if (!s_auto_save_deferred_recorded)
        {
            ++g_preset_auto_save_deferred_count;
            s_auto_save_deferred_recorded = true;
        }
        return GMP_TASK_DONE;
    }

    power_settings_make_v2_record(&settings);
    power_settings_serialize_v2(&settings, program_words);
    s_flash_operation_in_progress = true;
    power_settings_force_output_off();
    if (power_settings_flash_write(program_words))
    {
        g_settings_valid = 1U;
        g_settings_result = PSU_SETTINGS_RESULT_SAVE_OK;
        g_presets_dirty = 0U;
        s_auto_save_deferred_recorded = false;
        s_auto_save_retry_pending = false;
        ++g_preset_auto_save_count;
    }
    else
    {
        g_settings_valid = 0U;
        g_settings_result = PSU_SETTINGS_RESULT_FLASH_ERROR;
        g_presets_dirty = 1U;
        s_auto_save_retry_tick = gmp_base_get_system_tick();
        s_auto_save_retry_pending = true;
        ++g_preset_auto_save_failure_count;
    }
    power_settings_force_output_off();
    s_flash_operation_in_progress = false;
    return GMP_TASK_DONE;
}

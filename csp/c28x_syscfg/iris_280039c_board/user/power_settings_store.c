#include "power_settings_store.h"

#include <stdbool.h>

#include <ctrl_settings.h>
#include <device.h>
#include <driverlib.h>
#include <F021_F28003x_C28x.h>

#include "analog_io_test.h"
#include "power_app.h"
#include "power_control_policy.h"

#define PSU_SETTINGS_MAGIC              (0x50535531UL)
#define PSU_SETTINGS_VERSION            (1U)
#define PSU_SETTINGS_FLASH_ADDRESS      (0x000AE000UL)
#define PSU_SETTINGS_FLASH_SECTOR_WORDS (0x00000800UL)
#define PSU_SETTINGS_PROGRAM_WORDS      (8U)

volatile uint16_t g_settings_valid = 0U;
volatile uint16_t g_settings_save_request = 0U;
volatile uint16_t g_settings_result = PSU_SETTINGS_RESULT_IDLE;
volatile uint16_t g_settings_loaded = 0U;

static uint16_t power_settings_checksum(const psu_saved_settings_t *settings)
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

static void power_settings_read_record(psu_saved_settings_t *settings)
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

static bool power_settings_record_valid(
    const psu_saved_settings_t *settings)
{
    return (settings->magic == PSU_SETTINGS_MAGIC) &&
           (settings->version == PSU_SETTINGS_VERSION) &&
           (settings->checksum == power_settings_checksum(settings)) &&
           (settings->voltage_set_mv <= PSU_COMMAND_VOLTAGE_LIMIT_MV) &&
           (settings->current_set_ma <= PSU_COMMAND_CURRENT_LIMIT_MA) &&
           (settings->control_strategy <=
            (uint16_t)PSU_CONTROL_STRATEGY_CC_ONLY);
}

static void power_settings_force_output_off(void)
{
    g_output_switch_requested = 0U;
    g_output_switch_active = 0U;
    g_output_switch_precharge_active = 0U;
    g_output_switch_precharge_complete = 0U;
    g_output_switch_dac_gate_active = 0U;
    g_power_app.output_requested = false;
    g_power_app.output_enabled = false;
    g_power_app.state = POWER_STATE_OFF;
    analog_io_test_force_safe_outputs();
}

static bool power_settings_save_allowed(void)
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
           (g_output_switch_dac_gate_active == 0U);
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
            PSU_SETTINGS_FLASH_SECTOR_WORDS,
            &flash_status_word);
        if (status != Fapi_Status_Success)
        {
            success = false;
        }
    }

    if (success)
    {
        status = Fapi_issueProgrammingCommand(
            (uint32 *)PSU_SETTINGS_FLASH_ADDRESS,
            (uint16 *)program_words,
            PSU_SETTINGS_PROGRAM_WORDS,
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

void power_settings_store_init(void)
{
    psu_saved_settings_t settings;

    g_settings_valid = 0U;
    g_settings_save_request = 0U;
    g_settings_loaded = 0U;
    g_settings_result = PSU_SETTINGS_RESULT_INVALID_DATA;

    power_settings_read_record(&settings);
    if (power_settings_record_valid(&settings))
    {
        power_app_set_voltage_mv(settings.voltage_set_mv);
        power_app_set_current_ma(settings.current_set_ma);
        (void)power_control_policy_set_strategy(
            (psu_control_strategy_t)settings.control_strategy);
        g_settings_valid = 1U;
        g_settings_loaded = 1U;
        g_settings_result = PSU_SETTINGS_RESULT_LOAD_OK;
    }

    power_settings_force_output_off();
}

void power_settings_store_request_save(void)
{
    if (g_settings_save_request == 0U)
    {
        g_settings_result = PSU_SETTINGS_RESULT_IDLE;
        g_settings_save_request = 1U;
    }
}

gmp_task_status_t power_settings_store_task(gmp_task_t *tsk)
{
    psu_saved_settings_t settings;
    uint16_t program_words[PSU_SETTINGS_PROGRAM_WORDS];

    GMP_UNUSED_VAR(tsk);

    if (g_settings_save_request == 0U)
    {
        return GMP_TASK_DONE;
    }

    g_settings_save_request = 0U;
    if (!power_settings_save_allowed())
    {
        g_settings_result = PSU_SETTINGS_RESULT_SAVE_REJECTED;
        return GMP_TASK_DONE;
    }

    settings.magic = PSU_SETTINGS_MAGIC;
    settings.version = PSU_SETTINGS_VERSION;
    settings.voltage_set_mv = g_power_app.voltage_set_mv;
    settings.current_set_ma = g_power_app.current_set_ma;
    settings.control_strategy = g_control_strategy;
    settings.checksum = power_settings_checksum(&settings);

    program_words[0] = (uint16_t)(settings.magic & 0xFFFFUL);
    program_words[1] = (uint16_t)(settings.magic >> 16U);
    program_words[2] = settings.version;
    program_words[3] = settings.voltage_set_mv;
    program_words[4] = settings.current_set_ma;
    program_words[5] = settings.control_strategy;
    program_words[6] = settings.checksum;
    program_words[7] = 0xFFFFU;

    power_settings_force_output_off();
    if (power_settings_flash_write(program_words))
    {
        g_settings_valid = 1U;
        g_settings_result = PSU_SETTINGS_RESULT_SAVE_OK;
    }
    else
    {
        g_settings_valid = 0U;
        g_settings_result = PSU_SETTINGS_RESULT_FLASH_ERROR;
    }
    power_settings_force_output_off();
    return GMP_TASK_DONE;
}

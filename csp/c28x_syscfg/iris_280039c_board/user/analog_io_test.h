#ifndef ANALOG_IO_TEST_H
#define ANALOG_IO_TEST_H

#include <gmp_core.h>
#include <core/pm/function_scheduler.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define PSU_ANALOG_IO_TEST_TASK_PERIOD_MS       (10U)

#define PSU_DAC_TEST_ARM_KEY                    (0xA55AU)
#define PSU_DAC_TEST_VOLTAGE_MAX_CODE           (3103U)
#define PSU_DAC_TEST_CURRENT_MAX_CODE           (2731U)

#define PSU_DAC_TEST_COMMAND_NONE               (0U)
#define PSU_DAC_TEST_COMMAND_WRITE_DACA         (1U)
#define PSU_DAC_TEST_COMMAND_WRITE_DACB         (2U)
#define PSU_DAC_TEST_COMMAND_WRITE_BOTH         (3U)
#define PSU_DAC_TEST_COMMAND_CLEAR_AND_DISARM   (4U)

extern volatile uint16_t g_adc_test_vout_raw;
extern volatile uint16_t g_adc_test_iout_raw;
extern volatile float g_adc_test_vout_pin_v;
extern volatile float g_adc_test_iout_pin_v;
extern volatile float g_adc_test_vout_value_v;
extern volatile float g_adc_test_iout_value_ma;
extern volatile uint16_t g_adc_test_enabled;
extern volatile uint32_t g_adc_test_sample_count;

extern volatile uint16_t g_dac_test_arm;
extern volatile uint16_t g_dac_test_enable;
extern volatile uint16_t g_dac_test_voltage_code;
extern volatile uint16_t g_dac_test_current_code;
extern volatile uint16_t g_dac_test_voltage_applied_code;
extern volatile uint16_t g_dac_test_current_applied_code;
extern volatile uint16_t g_dac_test_command;
extern volatile uint32_t g_dac_test_apply_count;
extern volatile uint32_t g_dac_test_reject_count;
extern volatile float g_dac_test_voltage_expected_v;
extern volatile float g_dac_test_current_expected_v;
extern volatile uint16_t g_dac_test_follow_ui_enable;
extern volatile uint16_t g_dac_test_follow_ui_active;
extern volatile uint16_t g_dac_test_last_voltage_mv;
extern volatile uint16_t g_dac_test_last_current_ma;
extern volatile uint32_t g_dac_test_follow_update_count;
extern volatile uint16_t g_dac_test_auto_follow_pending;
extern volatile uint16_t g_dac_test_auto_follow_completed;
extern volatile uint32_t g_dac_test_auto_follow_count;
extern volatile time_gt g_dac_test_auto_follow_start_tick;
extern volatile uint32_t g_analog_board_voltage_clamp_count;
extern volatile uint32_t g_analog_board_current_clamp_count;
extern volatile uint32_t g_analog_board_min_current_clamp_count;
extern volatile uint16_t g_analog_board_applied_voltage_mv;
extern volatile uint16_t g_analog_board_applied_current_ma;
extern volatile uint16_t g_analog_board_iset_precharge_active;
extern volatile uint16_t g_analog_board_iset_precharge_complete;
extern volatile uint32_t g_analog_board_iset_precharge_count;
extern volatile uint16_t g_analog_board_protection_grace_active;
extern volatile uint32_t g_analog_board_protection_grace_skip_count;
extern volatile time_gt g_analog_board_protection_grace_start_tick;
extern volatile uint32_t g_analog_board_fault_shutdown_count;
extern volatile uint16_t g_analog_board_fault_shutdown_active;
extern volatile uint16_t g_analog_board_fault_hold_current_ma;
extern volatile uint16_t g_analog_board_fault_hold_active;
extern volatile uint32_t g_analog_board_fault_hold_count;
extern volatile uint16_t g_analog_board_feedback_fault;
extern volatile uint32_t g_analog_board_feedback_fault_count;
extern volatile uint16_t g_analog_board_adc_fault_confirm_count;
extern volatile uint32_t g_analog_board_adc_transient_count;
extern volatile uint16_t g_analog_board_last_vout_raw;
extern volatile uint16_t g_analog_board_last_iout_raw;
extern volatile uint16_t g_analog_board_feedback_settled;
extern volatile uint16_t g_analog_board_feedback_valid_count;
extern volatile uint32_t g_analog_board_feedback_settle_skip_count;
extern volatile time_gt g_analog_board_feedback_start_tick;
extern volatile uint16_t g_analog_board_ovp_confirm_count;
extern volatile uint16_t g_analog_board_ocp_confirm_count;
extern volatile uint32_t g_analog_board_ovp_transient_count;
extern volatile uint32_t g_analog_board_ocp_transient_count;

extern gmp_task_t task_analog_io_test;

void analog_io_test_init(void);
void analog_io_test_force_safe_outputs(void);
gmp_task_status_t analog_io_test_task(gmp_task_t* tsk);

#ifdef __cplusplus
}
#endif

#endif // ANALOG_IO_TEST_H

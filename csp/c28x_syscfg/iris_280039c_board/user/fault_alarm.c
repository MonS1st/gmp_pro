#include "fault_alarm.h"

#include <stdbool.h>
#include <ctrl_settings.h>
#include <gmp_core.h>
#include <xplt.peripheral.h>

#include "power_app.h"
#include "power_hal.h"
#include "user_main.h"

#if !PSU_FAULT_LED_BACKEND_FPGA
#error "The fault LED must use the FPGA backend"
#endif
#if PSU_FAULT_FPGA_LED_BIT > 3U
#error "The fault LED must select one of FPGA led[3:0]"
#endif
#if (PSU_FAULT_FPGA_LED_ACTIVE_HIGH != 0U) && \
    (PSU_FAULT_FPGA_LED_ACTIVE_HIGH != 1U)
#error "FPGA fault LED polarity must be 0 or 1"
#endif
#if PSU_FAULT_BUZZER_TYPE != PSU_FAULT_BUZZER_TYPE_ACTIVE
#error "GPIO58 is an active beeper and must not use PWM"
#endif
#if PSU_BUZZER_ACTIVE_LEVEL == PSU_BUZZER_INACTIVE_LEVEL
#error "Beeper active and inactive levels must differ"
#endif

typedef enum
{
    FAULT_LED_TEST_IDLE = 0U,
    FAULT_LED_TEST_CLEAR_PENDING,
    FAULT_LED_TEST_ON_PENDING,
    FAULT_LED_TEST_ACTIVE,
    FAULT_LED_TEST_OFF_PENDING
} fault_led_test_state_t;

volatile uint16_t g_fault_alarm_led_active = 0U;
volatile uint16_t g_fault_alarm_buzzer_active = 0U;
volatile uint16_t g_fault_alarm_reason = PSU_FAULT_ALARM_REASON_NONE;
volatile uint16_t g_fault_alarm_event_issued = 0U;
volatile uint32_t g_fault_alarm_pulse_count = 0U;
volatile uint32_t g_fault_alarm_clear_count = 0U;
volatile uint32_t g_fault_alarm_start_tick = 0U;
volatile uint32_t g_fault_alarm_last_duration_ms = 0U;

volatile uint16_t g_fault_alarm_hardware_ready = 0U;
volatile uint16_t g_fault_alarm_led_hardware_ready = 0U;
volatile uint16_t g_fault_alarm_buzzer_hardware_ready = 0U;
volatile uint16_t g_fault_alarm_selected_led_number = 0U;
volatile uint16_t g_fault_alarm_buzzer_type = PSU_FAULT_BUZZER_TYPE_UNKNOWN;

volatile uint16_t g_fault_alarm_fpga_led_bit = 0U;
volatile uint16_t g_fault_alarm_fpga_led_mask = 0U;
volatile uint16_t g_fault_alarm_fpga_led_shadow = 0U;
volatile uint16_t g_fault_alarm_led_mapping_verified = 0U;

volatile uint32_t g_fault_alarm_fpga_write_count = 0U;
volatile uint32_t g_fault_alarm_fpga_write_error_count = 0U;
volatile uint32_t g_fault_alarm_fpga_write_timeout_count = 0U;

volatile uint16_t g_fault_alarm_led_apply_pending = 0U;
volatile uint16_t g_fault_alarm_buzzer_start_pending = 0U;
volatile uint16_t g_fault_alarm_buzzer_stop_pending = 0U;

volatile uint16_t g_fault_alarm_led_test_command = 0U;
volatile uint16_t g_fault_alarm_led_test_active_bit = 0xFFFFU;
volatile uint32_t g_fault_alarm_led_test_count = 0U;
volatile uint16_t g_fault_alarm_buzzer_test_command = 0U;
volatile uint32_t g_fault_alarm_buzzer_test_count = 0U;

static volatile uint16_t s_fault_led_desired = 0U;
static fault_led_test_state_t s_led_test_state = FAULT_LED_TEST_IDLE;
static uint16_t s_led_test_bit = 0U;
static uint32_t s_led_test_start_tick = 0U;
static uint16_t s_fault_buzzer_pulse_active = 0U;
static uint16_t s_buzzer_test_active = 0U;
static uint32_t s_buzzer_test_start_tick = 0U;

static uint16_t s_led_retry_armed = 0U;
static uint16_t s_led_last_attempt_mask = 0U;
static uint16_t s_led_last_attempt_active = 0U;
static uint32_t s_led_last_attempt_tick = 0U;

static bool fault_alarm_manual_test_safe(void)
{
    return (g_output_switch_requested == 0U) &&
           (g_output_switch_active == 0U) &&
           (g_output_switch_precharge_active == 0U) &&
           (g_output_switch_dac_gate_active == 0U) &&
           !g_power_app.output_requested &&
           !g_power_app.output_enabled &&
           !power_output_hw_get() &&
           (g_power_app.state == POWER_STATE_OFF) &&
           !g_power_app.fault_latched &&
           (g_power_app.fault == POWER_FAULT_NONE) &&
           power_relay_cutoff_epwm_get();
}

static void fault_alarm_buzzer_set(bool active)
{
#if PSU_ENABLE_BEEP && PSU_ENABLE_FAULT_BUZZER && !PSU_SAFE_BRINGUP
    if (active)
    {
        beep_on();
    }
    else
    {
        beep_off();
    }
#else
    (void)active;
#endif
}

static bool fault_alarm_fpga_led_write(uint16_t led_mask, bool active)
{
    bool success;

    ++g_fault_alarm_fpga_write_count;
    success = board_fpga_led_set(led_mask, active);
    g_fault_alarm_fpga_led_shadow = board_fpga_led_get_shadow();
    if (!success)
    {
        ++g_fault_alarm_fpga_write_error_count;
        ++g_fault_alarm_fpga_write_timeout_count;
    }
    return success;
}

static bool fault_alarm_fpga_led_write_due(uint16_t led_mask,
                                           bool active,
                                           uint32_t current_tick,
                                           bool *attempted)
{
    *attempted = false;
    if ((s_led_retry_armed != 0U) &&
        (s_led_last_attempt_mask == led_mask) &&
        (s_led_last_attempt_active == (active ? 1U : 0U)) &&
        ((current_tick - s_led_last_attempt_tick) <
         PSU_FAULT_FPGA_LED_RETRY_MS))
    {
        return false;
    }

    s_led_retry_armed = 1U;
    s_led_last_attempt_mask = led_mask;
    s_led_last_attempt_active = active ? 1U : 0U;
    s_led_last_attempt_tick = current_tick;
    *attempted = true;
    return fault_alarm_fpga_led_write(led_mask, active);
}

static bool fault_alarm_fault_led_level(uint16_t desired_active)
{
#if PSU_FAULT_FPGA_LED_ACTIVE_HIGH
    return desired_active != 0U;
#else
    return desired_active == 0U;
#endif
}

static void fault_alarm_apply_led_pending(uint32_t current_tick)
{
    uint16_t desired_active;
    bool attempted;
    bool success;

    if (g_fault_alarm_led_apply_pending == 0U)
    {
        return;
    }

    desired_active = s_fault_led_desired;
    success = fault_alarm_fpga_led_write_due(
        PSU_FAULT_FPGA_LED_MASK,
        fault_alarm_fault_led_level(desired_active),
        current_tick,
        &attempted);
    if (!attempted || !success)
    {
        return;
    }

    g_fault_alarm_led_active = desired_active;
    if (s_fault_led_desired == desired_active)
    {
        g_fault_alarm_led_apply_pending = 0U;
    }
}

static void fault_alarm_led_test_finish(void)
{
    s_led_test_state = FAULT_LED_TEST_IDLE;
    s_led_test_start_tick = 0U;
    g_fault_alarm_led_test_active_bit = 0xFFFFU;
    g_fault_alarm_led_test_command = 0U;
}

static void fault_alarm_led_test_abort_before_fault(uint32_t current_tick)
{
    bool attempted;

    if (g_fault_alarm_event_issued == 0U)
    {
        return;
    }

    if (s_led_test_state != FAULT_LED_TEST_IDLE)
    {
        // Update the shared shadow before applying the fault bit. Even if this
        // write times out, the following fault write carries the corrected
        // complete R1 value and cannot preserve a stale mapping-test LED.
        (void)fault_alarm_fpga_led_write_due(
            PSU_FAULT_FPGA_LED_LOW4_MASK, false, current_tick, &attempted);
    }
    fault_alarm_led_test_finish();
}

static void fault_alarm_led_test_service(uint32_t current_tick)
{
    uint16_t command;
    uint16_t selected_bit;
    uint16_t selected_mask;
    bool attempted;
    bool success;

    if (g_fault_alarm_event_issued != 0U)
    {
        g_fault_alarm_led_test_command = 0U;
        return;
    }

    if (s_led_test_state != FAULT_LED_TEST_IDLE)
    {
        if (!fault_alarm_manual_test_safe())
        {
            s_led_test_state = FAULT_LED_TEST_OFF_PENDING;
        }

        if ((s_led_test_state == FAULT_LED_TEST_ACTIVE) &&
            ((current_tick - s_led_test_start_tick) >=
             PSU_FAULT_LED_TEST_DURATION_MS))
        {
            s_led_test_state = FAULT_LED_TEST_OFF_PENDING;
        }

        if (s_led_test_state == FAULT_LED_TEST_OFF_PENDING)
        {
            success = fault_alarm_fpga_led_write_due(
                PSU_FAULT_FPGA_LED_LOW4_MASK,
                false,
                current_tick,
                &attempted);
            if (attempted && success)
            {
                fault_alarm_led_test_finish();
            }
            return;
        }

        if (s_led_test_state == FAULT_LED_TEST_CLEAR_PENDING)
        {
            success = fault_alarm_fpga_led_write_due(
                PSU_FAULT_FPGA_LED_LOW4_MASK,
                false,
                current_tick,
                &attempted);
            if (!attempted || !success)
            {
                return;
            }
            s_led_test_state = FAULT_LED_TEST_ON_PENDING;
        }

        if (s_led_test_state == FAULT_LED_TEST_ON_PENDING)
        {
            if (!fault_alarm_manual_test_safe() ||
                (g_fault_alarm_event_issued != 0U))
            {
                s_led_test_state = FAULT_LED_TEST_OFF_PENDING;
                return;
            }

            selected_mask = (uint16_t)(1U << s_led_test_bit);
            success = fault_alarm_fpga_led_write_due(
                selected_mask, true, current_tick, &attempted);
            if (!attempted || !success)
            {
                return;
            }
            s_led_test_start_tick = current_tick;
            s_led_test_state = FAULT_LED_TEST_ACTIVE;
            ++g_fault_alarm_led_test_count;
        }
        return;
    }

    command = g_fault_alarm_led_test_command;
    if (command == 0U)
    {
        return;
    }
    if ((command > 5U) || !fault_alarm_manual_test_safe())
    {
        g_fault_alarm_led_test_command = 0U;
        return;
    }

    if (command == 5U)
    {
        g_fault_alarm_led_test_active_bit = 0xFFFFU;
        s_led_test_state = FAULT_LED_TEST_OFF_PENDING;
        return;
    }

    selected_bit = command - 1U;
    s_led_test_bit = selected_bit;
    g_fault_alarm_led_test_active_bit = selected_bit;
    s_led_test_state = FAULT_LED_TEST_CLEAR_PENDING;
}

static void fault_alarm_stop_buzzer_pending(uint32_t current_tick)
{
    if (g_fault_alarm_buzzer_stop_pending == 0U)
    {
        return;
    }

    g_fault_alarm_buzzer_stop_pending = 0U;
    fault_alarm_buzzer_set(false);
    g_fault_alarm_buzzer_active = 0U;

    if (s_fault_buzzer_pulse_active != 0U)
    {
        g_fault_alarm_last_duration_ms =
            current_tick - g_fault_alarm_start_tick;
    }
    s_fault_buzzer_pulse_active = 0U;

    if (s_buzzer_test_active != 0U)
    {
        s_buzzer_test_active = 0U;
        g_fault_alarm_buzzer_test_command = 0U;
    }
}

static void fault_alarm_start_buzzer_pending(uint32_t current_tick)
{
    uint32_t pulse_count;

    if (g_fault_alarm_buzzer_start_pending == 0U)
    {
        return;
    }

    pulse_count = g_fault_alarm_pulse_count;
    g_fault_alarm_buzzer_start_pending = 0U;
    if ((g_fault_alarm_event_issued == 0U) ||
        (g_fault_alarm_buzzer_hardware_ready == 0U))
    {
        return;
    }

    fault_alarm_buzzer_set(true);
    g_fault_alarm_buzzer_active = 1U;
    g_fault_alarm_start_tick = current_tick;
    g_fault_alarm_last_duration_ms = 0U;
    s_fault_buzzer_pulse_active = 1U;

    // If the fast path cleared or replaced this event while the background
    // task was starting the GPIO, stop immediately and leave any newer start
    // pending for the next 1 ms invocation.
    if ((g_fault_alarm_event_issued == 0U) ||
        (g_fault_alarm_pulse_count != pulse_count) ||
        (g_fault_alarm_buzzer_stop_pending != 0U))
    {
        fault_alarm_buzzer_set(false);
        g_fault_alarm_buzzer_active = 0U;
        s_fault_buzzer_pulse_active = 0U;
    }
}

static void fault_alarm_fault_buzzer_timeout(uint32_t current_tick)
{
    uint32_t elapsed_ms;

    if (s_fault_buzzer_pulse_active == 0U)
    {
        return;
    }

    elapsed_ms = current_tick - g_fault_alarm_start_tick;
    if (elapsed_ms < PSU_FAULT_BUZZER_PULSE_MS)
    {
        return;
    }

    fault_alarm_buzzer_set(false);
    g_fault_alarm_buzzer_active = 0U;
    g_fault_alarm_last_duration_ms = elapsed_ms;
    s_fault_buzzer_pulse_active = 0U;
}

static void fault_alarm_buzzer_test_service(uint32_t current_tick)
{
    if (s_buzzer_test_active != 0U)
    {
        if (!fault_alarm_manual_test_safe() ||
            (g_fault_alarm_event_issued != 0U) ||
            ((current_tick - s_buzzer_test_start_tick) >=
             PSU_FAULT_BUZZER_PULSE_MS))
        {
            fault_alarm_buzzer_set(false);
            g_fault_alarm_buzzer_active = 0U;
            s_buzzer_test_active = 0U;
            g_fault_alarm_buzzer_test_command = 0U;
        }
        return;
    }

    if (g_fault_alarm_buzzer_test_command == 0U)
    {
        return;
    }
    if ((g_fault_alarm_buzzer_test_command != 1U) ||
        !fault_alarm_manual_test_safe() ||
        (g_fault_alarm_event_issued != 0U) ||
        (g_fault_alarm_buzzer_active != 0U) ||
        (g_fault_alarm_buzzer_start_pending != 0U) ||
        (g_fault_alarm_buzzer_stop_pending != 0U))
    {
        g_fault_alarm_buzzer_test_command = 0U;
        return;
    }

    fault_alarm_buzzer_set(true);
    g_fault_alarm_buzzer_active = 1U;
    s_buzzer_test_active = 1U;
    s_buzzer_test_start_tick = current_tick;
    ++g_fault_alarm_buzzer_test_count;
}

void fault_alarm_init(void)
{
    g_fault_alarm_led_active = 0U;
    g_fault_alarm_buzzer_active = 0U;
    g_fault_alarm_reason = PSU_FAULT_ALARM_REASON_NONE;
    g_fault_alarm_event_issued = 0U;
    g_fault_alarm_pulse_count = 0U;
    g_fault_alarm_clear_count = 0U;
    g_fault_alarm_start_tick = 0U;
    g_fault_alarm_last_duration_ms = 0U;

    g_fault_alarm_led_hardware_ready =
        PSU_FAULT_LED_BACKEND_FPGA ? 1U : 0U;
#if PSU_ENABLE_BEEP && PSU_ENABLE_FAULT_BUZZER && !PSU_SAFE_BRINGUP
    g_fault_alarm_buzzer_hardware_ready = 1U;
#else
    g_fault_alarm_buzzer_hardware_ready = 0U;
#endif
    g_fault_alarm_hardware_ready =
        ((g_fault_alarm_led_hardware_ready != 0U) &&
         (g_fault_alarm_buzzer_hardware_ready != 0U)) ? 1U : 0U;
    g_fault_alarm_selected_led_number = PSU_FAULT_LED_SELECTED_NUMBER;
    g_fault_alarm_buzzer_type = PSU_FAULT_BUZZER_TYPE;

    g_fault_alarm_fpga_led_bit = PSU_FAULT_FPGA_LED_BIT;
    g_fault_alarm_fpga_led_mask = PSU_FAULT_FPGA_LED_MASK;
    g_fault_alarm_fpga_led_shadow = board_fpga_led_get_shadow();
    g_fault_alarm_led_mapping_verified = 0U;
    g_fault_alarm_fpga_write_count = 0U;
    g_fault_alarm_fpga_write_error_count = 0U;
    g_fault_alarm_fpga_write_timeout_count = 0U;

    s_fault_led_desired = 0U;
    g_fault_alarm_led_apply_pending = 1U;
    g_fault_alarm_buzzer_start_pending = 0U;
    g_fault_alarm_buzzer_stop_pending = 1U;

    g_fault_alarm_led_test_command = 0U;
    g_fault_alarm_led_test_active_bit = 0xFFFFU;
    g_fault_alarm_led_test_count = 0U;
    g_fault_alarm_buzzer_test_command = 0U;
    g_fault_alarm_buzzer_test_count = 0U;

    s_led_test_state = FAULT_LED_TEST_IDLE;
    s_led_test_bit = 0U;
    s_led_test_start_tick = 0U;
    s_fault_buzzer_pulse_active = 0U;
    s_buzzer_test_active = 0U;
    s_buzzer_test_start_tick = 0U;
    s_led_retry_armed = 0U;
    s_led_last_attempt_mask = 0U;
    s_led_last_attempt_active = 0U;
    s_led_last_attempt_tick = 0U;
}

void fault_alarm_on_fault_latched(psu_fault_alarm_reason_t reason)
{
    uint16_t combined_reason;

    if ((reason == PSU_FAULT_ALARM_REASON_NONE) ||
        (reason > PSU_FAULT_ALARM_REASON_OVP_OCP))
    {
        return;
    }

    if (g_fault_alarm_event_issued != 0U)
    {
        combined_reason = g_fault_alarm_reason | (uint16_t)reason;
        if (combined_reason <= (uint16_t)PSU_FAULT_ALARM_REASON_OVP_OCP)
        {
            g_fault_alarm_reason = combined_reason;
        }
        return;
    }

    g_fault_alarm_reason = (uint16_t)reason;
    g_fault_alarm_event_issued = 1U;
    s_fault_led_desired = 1U;
    g_fault_alarm_led_apply_pending = 1U;
    g_fault_alarm_buzzer_start_pending = 1U;
    g_fault_alarm_last_duration_ms = 0U;
    ++g_fault_alarm_pulse_count;
}

void fault_alarm_on_fault_cleared(void)
{
    s_fault_led_desired = 0U;
    g_fault_alarm_led_apply_pending = 1U;
    g_fault_alarm_buzzer_start_pending = 0U;
    g_fault_alarm_buzzer_stop_pending = 1U;
    g_fault_alarm_reason = PSU_FAULT_ALARM_REASON_NONE;
    g_fault_alarm_event_issued = 0U;
    ++g_fault_alarm_clear_count;
}

void fault_alarm_task(uint32_t current_tick)
{
    g_fault_alarm_fpga_led_shadow = board_fpga_led_get_shadow();

    if ((s_buzzer_test_active != 0U) &&
        ((g_fault_alarm_event_issued != 0U) ||
         !fault_alarm_manual_test_safe()))
    {
        g_fault_alarm_buzzer_stop_pending = 1U;
        g_fault_alarm_buzzer_test_command = 0U;
    }

    // All physical alarm operations run only in this 1 ms background task.
    fault_alarm_stop_buzzer_pending(current_tick);
    fault_alarm_led_test_abort_before_fault(current_tick);
    fault_alarm_apply_led_pending(current_tick);
    fault_alarm_start_buzzer_pending(current_tick);
    fault_alarm_fault_buzzer_timeout(current_tick);
    fault_alarm_led_test_service(current_tick);
    fault_alarm_buzzer_test_service(current_tick);
}

#include "fault_alarm.h"

#include <stdbool.h>
#include <ctrl_settings.h>
#include <gmp_core.h>

#include "power_app.h"
#include "power_hal.h"
#include "user_main.h"

#if PSU_FAULT_BUZZER_TYPE != PSU_FAULT_BUZZER_TYPE_ACTIVE
#error "GPIO58 is an active beeper and must not use PWM"
#endif
#if PSU_BUZZER_ACTIVE_LEVEL == PSU_BUZZER_INACTIVE_LEVEL
#error "Beeper active and inactive levels must differ"
#endif

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
volatile uint16_t g_fault_alarm_led_gpio_number = 0U;
volatile uint16_t g_fault_alarm_buzzer_type = PSU_FAULT_BUZZER_TYPE_UNKNOWN;

volatile uint16_t g_fault_alarm_led_apply_pending = 0U;
volatile uint16_t g_fault_alarm_buzzer_start_pending = 0U;
volatile uint16_t g_fault_alarm_buzzer_stop_pending = 0U;

volatile uint16_t g_fault_alarm_led_test_command = 0U;
volatile uint32_t g_fault_alarm_led_test_count = 0U;
volatile uint16_t g_fault_alarm_buzzer_test_command = 0U;
volatile uint32_t g_fault_alarm_buzzer_test_count = 0U;

static volatile uint16_t s_fault_led_desired = 0U;
static uint16_t s_led_test_active = 0U;
static uint32_t s_led_test_start_tick = 0U;
static uint32_t s_led_test_start_pulse_count = 0U;
static uint16_t s_fault_buzzer_pulse_active = 0U;
static uint16_t s_buzzer_test_active = 0U;
static uint32_t s_buzzer_test_start_tick = 0U;

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

static void fault_alarm_apply_led_pending(void)
{
    uint16_t desired_active;

    if (g_fault_alarm_led_apply_pending == 0U)
    {
        return;
    }

    desired_active = s_fault_led_desired;
    if (desired_active != 0U)
    {
        fault_led_on();
    }
    else
    {
        fault_led_off();
    }
    g_fault_alarm_led_active = desired_active;

    if (s_fault_led_desired == desired_active)
    {
        g_fault_alarm_led_apply_pending = 0U;
    }
}

static void fault_alarm_led_test_finish(void)
{
    s_led_test_active = 0U;
    s_led_test_start_tick = 0U;
    g_fault_alarm_led_test_command = 0U;

    // A real fault owns the LED once latched. Otherwise a rejected, aborted,
    // or completed manual test always returns GPIO44 to its safe LOW state.
    if (g_fault_alarm_event_issued == 0U)
    {
        fault_led_off();
        g_fault_alarm_led_active = 0U;
    }
}

static void fault_alarm_led_test_service(uint32_t current_tick)
{
    if (s_led_test_active != 0U)
    {
        if ((g_fault_alarm_led_test_command != 1U) ||
            (g_fault_alarm_pulse_count != s_led_test_start_pulse_count) ||
            (g_fault_alarm_event_issued != 0U) ||
            !fault_alarm_manual_test_safe() ||
            ((current_tick - s_led_test_start_tick) >=
             PSU_FAULT_LED_TEST_DURATION_MS))
        {
            fault_alarm_led_test_finish();
        }
        return;
    }

    if (g_fault_alarm_led_test_command == 0U)
    {
        return;
    }
    if ((g_fault_alarm_led_test_command != 1U) ||
        (g_fault_alarm_led_hardware_ready == 0U) ||
        (g_fault_alarm_event_issued != 0U) ||
        !fault_alarm_manual_test_safe())
    {
        g_fault_alarm_led_test_command = 0U;
        return;
    }

    fault_led_on();
    g_fault_alarm_led_active = 1U;
    s_led_test_active = 1U;
    s_led_test_start_tick = current_tick;
    s_led_test_start_pulse_count = g_fault_alarm_pulse_count;
    ++g_fault_alarm_led_test_count;
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

    // If the event changed while the background task was starting the GPIO,
    // stop immediately. A newer start request remains pending for the next run.
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

    g_fault_alarm_led_hardware_ready = 1U;
#if PSU_ENABLE_BEEP && PSU_ENABLE_FAULT_BUZZER && !PSU_SAFE_BRINGUP
    g_fault_alarm_buzzer_hardware_ready = 1U;
#else
    g_fault_alarm_buzzer_hardware_ready = 0U;
#endif
    g_fault_alarm_hardware_ready =
        g_fault_alarm_buzzer_hardware_ready &&
        g_fault_alarm_led_hardware_ready;
    g_fault_alarm_selected_led_number = PSU_FAULT_LED_SELECTED_NUMBER;
    g_fault_alarm_led_gpio_number = PSU_FAULT_LED_GPIO_NUMBER;
    g_fault_alarm_buzzer_type = PSU_FAULT_BUZZER_TYPE;

    s_fault_led_desired = 0U;
    g_fault_alarm_led_apply_pending = 1U;
    g_fault_alarm_buzzer_start_pending = 0U;
    g_fault_alarm_buzzer_stop_pending = 1U;
    g_fault_alarm_led_test_command = 0U;
    g_fault_alarm_led_test_count = 0U;
    g_fault_alarm_buzzer_test_command = 0U;
    g_fault_alarm_buzzer_test_count = 0U;

    s_led_test_active = 0U;
    s_led_test_start_tick = 0U;
    s_led_test_start_pulse_count = 0U;
    s_fault_buzzer_pulse_active = 0U;
    s_buzzer_test_active = 0U;
    s_buzzer_test_start_tick = 0U;
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
    if ((s_buzzer_test_active != 0U) &&
        ((g_fault_alarm_event_issued != 0U) ||
         !fault_alarm_manual_test_safe()))
    {
        g_fault_alarm_buzzer_stop_pending = 1U;
        g_fault_alarm_buzzer_test_command = 0U;
    }

    // Physical GPIO operations are serialized in this 1 ms background task.
    fault_alarm_apply_led_pending();
    fault_alarm_led_test_service(current_tick);
    fault_alarm_stop_buzzer_pending(current_tick);
    fault_alarm_start_buzzer_pending(current_tick);
    fault_alarm_fault_buzzer_timeout(current_tick);
    fault_alarm_buzzer_test_service(current_tick);
}

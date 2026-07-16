#include "fault_alarm.h"

#include <ctrl_settings.h>
#include <gmp_core.h>

#if PSU_FAULT_ALARM_HARDWARE_READY
#include <board.h>
#include <driverlib.h>
#endif

#if PSU_FAULT_ALARM_HARDWARE_READY
#if (PSU_FAULT_LED_SELECTED_NUMBER != 6U) && \
    (PSU_FAULT_LED_SELECTED_NUMBER != 7U) && \
    (PSU_FAULT_LED_SELECTED_NUMBER != 8U)
#error "A hardware-ready fault alarm must select LED6, LED7, or LED8"
#endif
#if PSU_FAULT_LED_ACTIVE_LEVEL == PSU_FAULT_LED_INACTIVE_LEVEL
#error "Confirmed fault LED active and inactive levels must differ"
#endif
#if PSU_FAULT_BUZZER_TYPE != PSU_FAULT_BUZZER_TYPE_ACTIVE
#error "Only a confirmed active beeper may use the GPIO alarm driver"
#endif
#if PSU_BUZZER_ACTIVE_LEVEL == PSU_BUZZER_INACTIVE_LEVEL
#error "Confirmed beeper active and inactive levels must differ"
#endif
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
volatile uint16_t g_fault_alarm_selected_led_number = 0U;
volatile uint16_t g_fault_alarm_buzzer_type = PSU_FAULT_BUZZER_TYPE_UNKNOWN;

static void fault_alarm_hw_init(void)
{
#if PSU_FAULT_ALARM_HARDWARE_READY
    // Write the safe levels before enabling either GPIO output.
    GPIO_writePin(PSU_FAULT_LED_GPIO, PSU_FAULT_LED_INACTIVE_LEVEL);
    GPIO_setPadConfig(PSU_FAULT_LED_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(PSU_FAULT_LED_GPIO, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(PSU_FAULT_LED_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setControllerCore(PSU_FAULT_LED_GPIO, GPIO_CORE_CPU1);

    GPIO_writePin(PSU_BUZZER_GPIO, PSU_BUZZER_INACTIVE_LEVEL);
    GPIO_setPadConfig(PSU_BUZZER_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(PSU_BUZZER_GPIO, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(PSU_BUZZER_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setControllerCore(PSU_BUZZER_GPIO, GPIO_CORE_CPU1);
#endif
}

static void fault_alarm_hw_set_led(uint16_t active)
{
#if PSU_FAULT_ALARM_HARDWARE_READY
    GPIO_writePin(PSU_FAULT_LED_GPIO,
                  (active != 0U) ? PSU_FAULT_LED_ACTIVE_LEVEL :
                                   PSU_FAULT_LED_INACTIVE_LEVEL);
#else
    (void)active;
#endif
}

static void fault_alarm_hw_set_buzzer(uint16_t active)
{
#if PSU_FAULT_ALARM_HARDWARE_READY
    // Hardware-ready builds are compile-time limited to a confirmed active
    // beeper. An unconfirmed/passive beeper never starts a guessed PWM.
    GPIO_writePin(PSU_BUZZER_GPIO,
                  (active != 0U) ? PSU_BUZZER_ACTIVE_LEVEL :
                                   PSU_BUZZER_INACTIVE_LEVEL);
#else
    (void)active;
#endif
}

static uint32_t fault_alarm_elapsed(uint32_t current_tick)
{
    // Unsigned subtraction remains correct across the 32-bit tick wrap.
    return current_tick - g_fault_alarm_start_tick;
}

void fault_alarm_init(void)
{
    fault_alarm_hw_init();
    fault_alarm_hw_set_buzzer(0U);
    fault_alarm_hw_set_led(0U);

    g_fault_alarm_led_active = 0U;
    g_fault_alarm_buzzer_active = 0U;
    g_fault_alarm_reason = PSU_FAULT_ALARM_REASON_NONE;
    g_fault_alarm_event_issued = 0U;
    g_fault_alarm_pulse_count = 0U;
    g_fault_alarm_clear_count = 0U;
    g_fault_alarm_start_tick = 0U;
    g_fault_alarm_last_duration_ms = 0U;
    g_fault_alarm_hardware_ready = PSU_FAULT_ALARM_HARDWARE_READY;
    g_fault_alarm_selected_led_number = PSU_FAULT_LED_SELECTED_NUMBER;
    g_fault_alarm_buzzer_type = PSU_FAULT_BUZZER_TYPE;
}

void fault_alarm_on_fault_latched(psu_fault_alarm_reason_t reason,
                                  uint32_t current_tick)
{
    uint16_t combined_reason;

    if ((reason == PSU_FAULT_ALARM_REASON_NONE) ||
        (reason > PSU_FAULT_ALARM_REASON_OVP_OCP))
    {
        return;
    }

    if (g_fault_alarm_event_issued != 0U)
    {
        // OVP and OCP can confirm in the same fault cycle. Upgrade the reason
        // without restarting the 300 ms pulse or incrementing its count.
        combined_reason = g_fault_alarm_reason | (uint16_t)reason;
        if (combined_reason <= (uint16_t)PSU_FAULT_ALARM_REASON_OVP_OCP)
        {
            g_fault_alarm_reason = combined_reason;
        }
        return;
    }

    g_fault_alarm_reason = (uint16_t)reason;
    g_fault_alarm_event_issued = 1U;
    g_fault_alarm_led_active = 1U;
    g_fault_alarm_buzzer_active = 1U;
    g_fault_alarm_start_tick = current_tick;
    g_fault_alarm_last_duration_ms = 0U;
    ++g_fault_alarm_pulse_count;

    fault_alarm_hw_set_led(1U);
    fault_alarm_hw_set_buzzer(1U);
}

void fault_alarm_on_fault_cleared(void)
{
    if (g_fault_alarm_buzzer_active != 0U)
    {
        g_fault_alarm_last_duration_ms =
            fault_alarm_elapsed((uint32_t)gmp_base_get_system_tick());
    }

    fault_alarm_hw_set_buzzer(0U);
    fault_alarm_hw_set_led(0U);

    g_fault_alarm_buzzer_active = 0U;
    g_fault_alarm_led_active = 0U;
    g_fault_alarm_reason = PSU_FAULT_ALARM_REASON_NONE;
    g_fault_alarm_event_issued = 0U;
    g_fault_alarm_start_tick = 0U;
    ++g_fault_alarm_clear_count;
}

void fault_alarm_task(uint32_t current_tick)
{
    uint32_t elapsed_ms;
    uint32_t pulse_count;
    uint32_t start_tick;

    if (g_fault_alarm_buzzer_active == 0U)
    {
        return;
    }

    // Snapshot the pulse generation so an ISR-side clear/new-latch transition
    // cannot make this background invocation expire a newer pulse.
    pulse_count = g_fault_alarm_pulse_count;
    start_tick = g_fault_alarm_start_tick;
    elapsed_ms = current_tick - start_tick;
    if (elapsed_ms < PSU_FAULT_BUZZER_PULSE_MS)
    {
        return;
    }

    if ((g_fault_alarm_buzzer_active == 0U) ||
        (g_fault_alarm_pulse_count != pulse_count) ||
        (g_fault_alarm_start_tick != start_tick))
    {
        return;
    }

    fault_alarm_hw_set_buzzer(0U);
    g_fault_alarm_buzzer_active = 0U;
    g_fault_alarm_last_duration_ms = elapsed_ms;
}

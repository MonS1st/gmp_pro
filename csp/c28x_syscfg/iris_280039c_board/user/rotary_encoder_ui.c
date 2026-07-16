#include "rotary_encoder_ui.h"

#include "analog_io_test.h"
#include "power_app.h"

#include <board.h>
#include <ctrl_settings.h>
#include <driverlib.h>

#define ROTARY_ENCODER_MAX_EVENTS_PER_TASK (4U)

// The GPIO SysConfig module exports the pin as IRIS_ENCODER_SW. Keep the
// hardware-facing name explicit at the read site requested by the UI design.
#ifndef IRIS_ENCODER_SW_GPIO
#define IRIS_ENCODER_SW_GPIO (IRIS_ENCODER_SW)
#endif

volatile uint16_t g_encoder_ui_enabled = 0U;
volatile uint16_t g_encoder_ui_mode = PSU_ENCODER_MODE_VOLTAGE;

volatile uint32_t g_encoder_raw_position = 0U;
volatile int32_t g_encoder_last_delta = 0;
volatile int32_t g_encoder_detent_accumulator = 0;

volatile uint32_t g_encoder_clockwise_count = 0U;
volatile uint32_t g_encoder_counterclockwise_count = 0U;
volatile uint32_t g_encoder_invalid_transition_count = 0U;

volatile uint16_t g_encoder_button_raw = 1U;
volatile uint16_t g_encoder_button_pressed = 0U;
volatile uint16_t g_encoder_button_debounce_count = 0U;
volatile uint32_t g_encoder_button_press_count = 0U;
volatile uint32_t g_encoder_mode_switch_count = 0U;
volatile uint32_t g_encoder_fault_reset_request_count = 0U;
volatile uint32_t g_encoder_fault_reset_success_count = 0U;
volatile uint32_t g_encoder_fault_reset_reject_count = 0U;
volatile uint16_t g_encoder_fault_reset_pending = 0U;

volatile uint32_t g_encoder_voltage_update_count = 0U;
volatile uint32_t g_encoder_current_update_count = 0U;
volatile uint32_t g_encoder_reject_count = 0U;

static uint32_t s_encoder_previous_position = 0U;

static bool rotary_encoder_resettable_power_fault_active(void)
{
    return (g_power_app.fault_latched != 0U) &&
           ((g_power_app.fault == POWER_FAULT_OVERVOLTAGE) ||
            (g_power_app.fault == POWER_FAULT_OVERCURRENT));
}

static void rotary_encoder_update_fault_reset_result(void)
{
    if (g_encoder_fault_reset_pending == 0U)
    {
        return;
    }

    if (g_power_app.fault_latched == 0U)
    {
        ++g_encoder_fault_reset_success_count;
        g_encoder_fault_reset_pending = 0U;
    }
    else if (!g_power_app.fault_reset_requested)
    {
        ++g_encoder_fault_reset_reject_count;
        g_encoder_fault_reset_pending = 0U;
    }
}

static uint16_t rotary_encoder_voltage_limit_mv(void)
{
    return PSU_COMMAND_VOLTAGE_LIMIT_MV;
}

static uint16_t rotary_encoder_current_limit_ma(void)
{
    return PSU_COMMAND_CURRENT_LIMIT_MA;
}

static bool rotary_encoder_setpoint_change_allowed(void)
{
    return ((g_power_app.fault_latched == 0)
#if PSU_REAL_FEEDBACK_CONNECTED
            &&
            (g_analog_board_feedback_fault == 0U) &&
            (g_analog_board_fault_hold_active == 0U) &&
            (g_analog_board_feedback_settled == 1U)
#endif
            );
}

static void rotary_encoder_update_voltage(bool clockwise)
{
    uint16_t current = power_app_get_voltage_mv();
    uint16_t limit = rotary_encoder_voltage_limit_mv();
    uint16_t next;

    if (current > limit)
    {
        current = limit;
    }

    if (clockwise)
    {
        next = ((uint32_t)PSU_VOLTAGE_STEP_MV >
                ((uint32_t)limit - (uint32_t)current)) ?
                   limit :
                   (uint16_t)(current + PSU_VOLTAGE_STEP_MV);
    }
    else
    {
        next = (current < PSU_VOLTAGE_STEP_MV) ?
                   0U : (uint16_t)(current - PSU_VOLTAGE_STEP_MV);
    }

    if (next == power_app_get_voltage_mv())
    {
        ++g_encoder_reject_count;
        return;
    }

    power_app_set_voltage_mv(next);
    ++g_encoder_voltage_update_count;
}

static void rotary_encoder_update_current(bool clockwise)
{
    uint16_t current = power_app_get_current_ma();
    uint16_t limit = rotary_encoder_current_limit_ma();
    uint16_t minimum = 0U;
    uint16_t next;

#if PSU_ENABLE_LOW_RANGE_BRINGUP_LIMITS
    if (power_app_get_voltage_mv() > 0U)
    {
        minimum = PSU_ANALOG_BOARD_MIN_CURRENT_MA;
    }
#endif

    if (current > limit)
    {
        current = limit;
    }
    if (current < minimum)
    {
        current = minimum;
    }

    if (clockwise)
    {
        next = ((uint32_t)PSU_CURRENT_STEP_MA >
                ((uint32_t)limit - (uint32_t)current)) ?
                   limit :
                   (uint16_t)(current + PSU_CURRENT_STEP_MA);
    }
    else
    {
        next = ((uint32_t)PSU_CURRENT_STEP_MA >
                ((uint32_t)current - (uint32_t)minimum)) ?
                   minimum :
                   (uint16_t)(current - PSU_CURRENT_STEP_MA);
    }

    if (next == power_app_get_current_ma())
    {
        ++g_encoder_reject_count;
        return;
    }

    power_app_set_current_ma(next);
    ++g_encoder_current_update_count;
}

static void rotary_encoder_apply_detent(bool clockwise)
{
    if (clockwise)
    {
        ++g_encoder_clockwise_count;
    }
    else
    {
        ++g_encoder_counterclockwise_count;
    }

    if (!rotary_encoder_setpoint_change_allowed())
    {
        ++g_encoder_reject_count;
        return;
    }

    if (g_encoder_ui_mode == PSU_ENCODER_MODE_VOLTAGE)
    {
        rotary_encoder_update_voltage(clockwise);
    }
    else if (g_encoder_ui_mode == PSU_ENCODER_MODE_CURRENT)
    {
        rotary_encoder_update_current(clockwise);
    }
    else
    {
        g_encoder_ui_mode = PSU_ENCODER_MODE_VOLTAGE;
        ++g_encoder_reject_count;
    }
}

static int32_t rotary_encoder_position_delta(uint32_t current,
                                             uint32_t previous)
{
    const int32_t position_span =
        (int32_t)(PSU_ENCODER_POSITION_COUNTER_MAX + 1UL);
    const int32_t half_span = position_span / 2;
    int32_t delta = (int32_t)current - (int32_t)previous;

    if (delta > half_span)
    {
        delta -= position_span;
    }
    else if (delta < -half_span)
    {
        delta += position_span;
    }

    return delta;
}

static void rotary_encoder_process_position(void)
{
    uint16_t events_processed = 0U;
    uint16_t interrupt_status;
    int32_t counts_per_detent = (int32_t)PSU_ENCODER_COUNTS_PER_DETENT;
    int32_t delta;
    uint32_t position = EQEP_getPosition(IRIS_EQEP2_BASE);

    g_encoder_raw_position = position;
    delta = rotary_encoder_position_delta(position,
                                          s_encoder_previous_position);
    s_encoder_previous_position = position;
    g_encoder_last_delta = delta;

    interrupt_status = EQEP_getInterruptStatus(IRIS_EQEP2_BASE);
    interrupt_status &= (EQEP_INT_PHASE_ERROR | EQEP_INT_POS_CNT_ERROR);
    if (interrupt_status != 0U)
    {
        ++g_encoder_invalid_transition_count;
        EQEP_clearInterruptStatus(IRIS_EQEP2_BASE, interrupt_status);
    }

    if (counts_per_detent <= 0)
    {
        g_encoder_detent_accumulator = 0;
        ++g_encoder_invalid_transition_count;
        return;
    }

    g_encoder_detent_accumulator += delta;

    for (events_processed = 0U;
         events_processed < ROTARY_ENCODER_MAX_EVENTS_PER_TASK;
         ++events_processed)
    {
        if (g_encoder_detent_accumulator >= counts_per_detent)
        {
            g_encoder_detent_accumulator -= counts_per_detent;
            rotary_encoder_apply_detent(true);
        }
        else if (g_encoder_detent_accumulator <= -counts_per_detent)
        {
            g_encoder_detent_accumulator += counts_per_detent;
            rotary_encoder_apply_detent(false);
        }
        else
        {
            break;
        }
    }

    // A mechanical knob cannot legitimately exceed this per-2 ms budget.
    // Drop any whole-detent backlog so a corrupted count cannot drain into
    // many later setpoint changes; retain only a sub-detent residual.
    if (g_encoder_detent_accumulator >= counts_per_detent)
    {
        g_encoder_detent_accumulator = counts_per_detent - 1;
        ++g_encoder_invalid_transition_count;
    }
    else if (g_encoder_detent_accumulator <= -counts_per_detent)
    {
        g_encoder_detent_accumulator = 1 - counts_per_detent;
        ++g_encoder_invalid_transition_count;
    }
}

static void rotary_encoder_process_button(void)
{
    uint16_t raw = (uint16_t)GPIO_readPin(IRIS_ENCODER_SW_GPIO);

    g_encoder_button_raw = raw;

    if (g_encoder_button_pressed == 0U)
    {
        if (raw == 0U)
        {
            if (g_encoder_button_debounce_count <
                PSU_ENCODER_BUTTON_DEBOUNCE_COUNT)
            {
                ++g_encoder_button_debounce_count;
            }

            if (g_encoder_button_debounce_count >=
                PSU_ENCODER_BUTTON_DEBOUNCE_COUNT)
            {
                g_encoder_button_pressed = 1U;
                g_encoder_button_debounce_count = 0U;
                ++g_encoder_button_press_count;

                if (rotary_encoder_resettable_power_fault_active())
                {
                    power_app_reset_fault();
                    ++g_encoder_fault_reset_request_count;
                    g_encoder_fault_reset_pending = 1U;
                }
                else
                {
                    g_encoder_ui_mode =
                        (g_encoder_ui_mode == PSU_ENCODER_MODE_VOLTAGE) ?
                            PSU_ENCODER_MODE_CURRENT :
                            PSU_ENCODER_MODE_VOLTAGE;
                    ++g_encoder_mode_switch_count;
                }
            }
        }
        else
        {
            g_encoder_button_debounce_count = 0U;
        }
    }
    else
    {
        if (raw != 0U)
        {
            if (g_encoder_button_debounce_count <
                PSU_ENCODER_BUTTON_DEBOUNCE_COUNT)
            {
                ++g_encoder_button_debounce_count;
            }

            if (g_encoder_button_debounce_count >=
                PSU_ENCODER_BUTTON_DEBOUNCE_COUNT)
            {
                g_encoder_button_pressed = 0U;
                g_encoder_button_debounce_count = 0U;
            }
        }
        else
        {
            g_encoder_button_debounce_count = 0U;
        }
    }
}

void rotary_encoder_ui_init(void)
{
    g_encoder_ui_enabled = PSU_ENABLE_ROTARY_ENCODER_UI ? 1U : 0U;
    g_encoder_ui_mode = PSU_ENCODER_MODE_VOLTAGE;

    g_encoder_raw_position = EQEP_getPosition(IRIS_EQEP2_BASE);
    s_encoder_previous_position = g_encoder_raw_position;
    g_encoder_last_delta = 0;
    g_encoder_detent_accumulator = 0;

    g_encoder_clockwise_count = 0U;
    g_encoder_counterclockwise_count = 0U;
    g_encoder_invalid_transition_count = 0U;

    g_encoder_button_raw = (uint16_t)GPIO_readPin(IRIS_ENCODER_SW_GPIO);
    g_encoder_button_pressed = 0U;
    g_encoder_button_debounce_count = 0U;
    g_encoder_button_press_count = 0U;
    g_encoder_mode_switch_count = 0U;
    g_encoder_fault_reset_request_count = 0U;
    g_encoder_fault_reset_success_count = 0U;
    g_encoder_fault_reset_reject_count = 0U;
    g_encoder_fault_reset_pending = 0U;

    g_encoder_voltage_update_count = 0U;
    g_encoder_current_update_count = 0U;
    g_encoder_reject_count = 0U;
}

gmp_task_status_t rotary_encoder_ui_task(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    if (g_encoder_ui_enabled == 0U)
    {
        return GMP_TASK_DONE;
    }

    rotary_encoder_update_fault_reset_result();
    rotary_encoder_process_position();
    rotary_encoder_process_button();

    return GMP_TASK_DONE;
}

gmp_task_t task_rotary_encoder_ui = {
    "rotary_encoder_ui",
    rotary_encoder_ui_task,
    PSU_ENCODER_TASK_PERIOD_MS,
    0U,
    PSU_ENABLE_ROTARY_ENCODER_UI,
    NULL,
};

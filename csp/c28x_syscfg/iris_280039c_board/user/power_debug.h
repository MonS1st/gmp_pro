#ifndef POWER_DEBUG_H
#define POWER_DEBUG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    POWER_DEBUG_CMD_NONE = 0,
    POWER_DEBUG_CMD_VOLTAGE_UP,
    POWER_DEBUG_CMD_VOLTAGE_DOWN,
    POWER_DEBUG_CMD_CURRENT_UP,
    POWER_DEBUG_CMD_CURRENT_DOWN,
    POWER_DEBUG_CMD_OUTPUT_TOGGLE,
    POWER_DEBUG_CMD_FAULT_RESET,
    POWER_DEBUG_CMD_SET_VOLTAGE_MV,
    POWER_DEBUG_CMD_SET_CURRENT_MA,
    POWER_DEBUG_CMD_SET_LOAD_OHM,
    POWER_DEBUG_CMD_SET_MEASUREMENT_OVERRIDE,
    POWER_DEBUG_CMD_CLEAR_MEASUREMENT_OVERRIDE
} power_debug_command_t;

// Write arguments and command first in CCS Expressions, then increment
// command_count. The background task acknowledges exactly one execution by
// copying that count to ack_count.
extern volatile uint16_t g_power_debug_command;
extern volatile uint16_t g_power_debug_arg0;
extern volatile uint16_t g_power_debug_arg1;
extern volatile uint16_t g_power_debug_command_count;
extern volatile uint16_t g_power_debug_ack_count;

void power_debug_process_command(void);
void power_debug_print_status(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_DEBUG_H

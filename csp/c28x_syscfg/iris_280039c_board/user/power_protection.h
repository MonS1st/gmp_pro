#ifndef POWER_PROTECTION_H
#define POWER_PROTECTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    POWER_PROTECTION_RESULT_NONE = 0,
    POWER_PROTECTION_RESULT_OVERVOLTAGE,
    POWER_PROTECTION_RESULT_OVERCURRENT
} power_protection_result_t;

typedef struct
{
    uint16_t ovp_confirm_count;
    uint16_t ocp_confirm_count;
    uint16_t last_voltage_mv;
    uint16_t last_current_ma;
} power_protection_t;

extern volatile power_protection_t g_power_protection;

void power_protection_init(void);
void power_protection_reset(void);
power_protection_result_t power_protection_step(uint16_t voltage_mv,
                                                uint16_t current_ma,
                                                bool enabled);
bool power_protection_release_safe(uint16_t voltage_mv, uint16_t current_ma);

#ifdef __cplusplus
}
#endif

#endif // POWER_PROTECTION_H

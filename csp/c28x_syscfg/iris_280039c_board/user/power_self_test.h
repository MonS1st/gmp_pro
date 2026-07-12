#ifndef POWER_SELF_TEST_H
#define POWER_SELF_TEST_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile float g_virtual_load_ohm;
extern volatile uint16_t g_virtual_voltage_mv;
extern volatile uint16_t g_virtual_current_ma;
extern volatile uint16_t g_virtual_measurement_seq;

void power_self_test_step(void);
bool power_self_test_get_measurement(uint16_t *voltage_mv, uint16_t *current_ma);

#ifdef __cplusplus
}
#endif

#endif // POWER_SELF_TEST_H

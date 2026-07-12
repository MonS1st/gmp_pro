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
extern volatile bool g_virtual_measurement_override_enable;
extern volatile uint16_t g_virtual_override_voltage_mv;
extern volatile uint16_t g_virtual_override_current_ma;

void power_self_test_step(void);
bool power_self_test_get_measurement(uint16_t *voltage_mv, uint16_t *current_ma);
void power_self_test_set_load_ohm(uint16_t load_ohm);
uint16_t power_self_test_get_load_ohm(void);
void power_self_test_set_measurement_override(uint16_t voltage_mv,
                                              uint16_t current_ma);
void power_self_test_clear_measurement_override(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_SELF_TEST_H

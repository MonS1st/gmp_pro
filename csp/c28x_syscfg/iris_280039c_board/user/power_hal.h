#ifndef POWER_HAL_H
#define POWER_HAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// DACA drives Vset; DACB drives V_ISET.
uint16_t power_voltage_mv_to_dac(uint16_t voltage_mv);
uint16_t power_current_ma_to_dac(uint16_t current_ma);

void power_dac_set_voltage_mv(uint16_t voltage_mv);
void power_dac_set_current_ma(uint16_t current_ma);
void power_dac_set_zero(void);

#define POWER_FPGA_R1_LED_MASK  (0x000FU)

bool power_fpga_r1_write_masked(uint16_t mask, uint16_t value);
void power_output_hw_set(bool enable);
void power_output_hw_service(void);
bool power_output_hw_get(void);
bool power_hal_safe_bringup_self_test(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_HAL_H

#ifndef POWER_HAL_H
#define POWER_HAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// DACA drives Vset; DACB drives V_ISET.
extern volatile uint16_t g_vs_5v_dac_code;
extern volatile uint16_t g_vs_103v_dac_code;
extern volatile uint16_t g_vs_11v_dac_code;
uint16_t power_voltage_mv_to_dac(uint16_t voltage_mv);
uint16_t power_current_ma_to_dac(uint16_t current_ma);

void power_dac_set_voltage_mv(uint16_t voltage_mv);
void power_dac_set_current_ma(uint16_t current_ma);
void power_dac_set_zero(void);

// Active-high relay cutoff: ePWM4A on GPIO22. HIGH disconnects the load;
// LOW permits relay connection only after the application safety gate passes.
extern volatile uint16_t g_relay_cutoff_command;
extern volatile uint16_t g_relay_cutoff_level;
extern volatile uint32_t g_relay_cutoff_assert_count;
extern volatile uint32_t g_relay_cutoff_release_count;
void power_relay_cutoff_epwm_init(void);
void power_relay_cutoff_epwm_set(bool cutoff_active);
bool power_relay_cutoff_epwm_get(void);

// Software-only logical Output state; it does not drive ePWM4A.
void power_output_hw_set(bool enable);
void power_output_hw_service(void);
bool power_output_hw_get(void);
bool power_hal_safe_bringup_self_test(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_HAL_H

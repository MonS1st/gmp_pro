/** @file xplt.peripheral.cpp @brief Simulated SINV ADC and host peripheral setup. */
#include <gmp_core.hpp>
#include "user_main.h"
#include <xplt.peripheral.h>

extern "C" {
extern gpio_halt user_led;
adc_channel_t adc_v_grid;
adc_channel_t adc_i_ac;
adc_channel_t adc_v_bus;
#ifdef SINV_2023A_SINGLE_MODE_ACTIVE
adc_channel_t adc_i_load;
#endif
fast_gt g_sinv_sim_enable_pending = 0;

void setup_peripheral(void)
{
    user_led = nullptr;
    ctl_init_adc_channel(&adc_v_grid,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_AC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_AC_VOLTAGE_BIAS), CTRL_ADC_RESOLUTION, 24);
    ctl_init_adc_channel(&adc_i_ac,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_AC_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_AC_CURRENT_BIAS), CTRL_ADC_RESOLUTION, 24);
#ifdef SINV_2023A_SINGLE_MODE_ACTIVE
    /* The model's IG shunt reports bridge-N minus load-N voltage, opposite
       to positive load current from AC_BUS_L through the load to AC_BUS_N.
       A negative ADC gain maps channel 5 to positive load-current peak PU. */
    ctl_init_adc_channel(&adc_i_load,
        -ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_AC_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_AC_CURRENT_BIAS), CTRL_ADC_RESOLUTION, 24);
#endif
    ctl_init_adc_channel(&adc_v_bus,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_BIAS), CTRL_ADC_RESOLUTION, 24);
}

void flush_dl_rx_buffer(void) { }
void send_monitor_data(void) { }
}

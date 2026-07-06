//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of controller objects in this file.
//
// User should implement the Main ISR of the controller tasks.
//
// User should ensure that all the controller codes here is platform-independent.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <xplt.peripheral.h>
#include "ctl_main.h"

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// Controller interface

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    g_adc3_daca_raw = ADC_readResult(ADC_CH3_RESULT_BASE, ADC_CH3);
    g_adc3_sine_pu = ctl_step_adc_channel(&g_adc3_daca_channel, g_adc3_daca_raw);
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{

    EPWM_setCounterCompareValue(IRIS_EPWM1_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                g_epwm1_cmpa);
    EPWM_setCounterCompareValue(IRIS_EPWM1_BASE,
                                EPWM_COUNTER_COMPARE_B,
                                g_epwm1_cmpb);

    g_daca_sine_code = ctl_pu_to_dac_code(g_daca_sine_value,
                                          DACA_SINE_DAC_AMP_CODE_F);
    DAC_setShadowValue(IRIS_DACA_BASE, g_daca_sine_code);

    g_dacb_lead_code = ctl_pu_to_dac_code(g_lead_output_pu,
                                          DACB_LEAD_DAC_AMP_CODE_F);
    DAC_setShadowValue(IRIS_DACB_BASE, g_dacb_lead_code);

}

// function prototype
void GPIO_WritePin(uint16_t gpioNumber, uint16_t outVal);

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_fast_enable_output()
{
    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

// Disable Output
GMP_STATIC_INLINE void ctl_fast_disable_output()
{
    // Disables the PWM device
    EPWM_forceTripZoneEvent(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_

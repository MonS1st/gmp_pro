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

#include <ctl/component/interface/adc_channel.h>//2

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// Controller interface



extern adc_channel_t input_wave_adc;
extern pwm_channel_t output_pwm_iris_1;//5
extern ctrl_gt comp_out;//5

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{


    ctl_step_adc_channel(&input_wave_adc,ADC_readResult(INPUT_WAVE_RESULT_REG, INPUT_WAVE));//2
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    static uint32_t tick = 0;

    tick += 1;

    ctrl_gt output_signal = (tick % 20000) / 20000.0f * 100;

    //EPWM_setCounterCompareValue(IRIS_EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 1500);
    //1.65V + 1.65/2 sin(100*2/pi *t )

    DAC_setShadowValue(IRIS_DACA_BASE, ctl_sin(output_signal) * 1024 + 2048);//1
    //DAC_setShadowValue(IRIS_DACB_BASE, iuvw.control_port.value.dat[phase_C] * 2048 + 2048);

    //展示移动相位后的输出，与adcA相差45度
    DAC_setShadowValue(IRIS_DACB_BASE, comp_out * 1024 + 2048);

    //死区
    ctl_step_pwm_channel(&output_pwm_iris_1, (comp_out + 1.0f)/2);
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

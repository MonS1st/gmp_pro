/**
 * @file ctl_main.cpp
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <xplt.peripheral.h>

//=================================================================================================
// include Necessary control modules

#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/interface/pwm_channel.h>
#include <ctl/component/interface/spwm_modulator.h>
#include <ctl/component/intrinsic/discrete/lead_lag.h>

#include <ctl/framework/cia402_state_machine.h>

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// controller modules with extern
#define CONTROL_UPDATE_HZ             (20000.0f)
#define CONTROL_EPWM_CMP_HALF_TICKS   (750U)

#define DACA_SINE_FREQ_HZ             (100.0f)
#define DACA_SINE_PHASE_STEP_PU       (DACA_SINE_FREQ_HZ / CONTROL_UPDATE_HZ)

#define ADC_RESOLUTION_BITS           (12)
#define ADC_IQN                       (24)
#define ADC3_DACA_BIAS_PU             (0.5f)
#define ADC3_DACA_GAIN_PU             (4.0f)

#define LEAD_FREQ_HZ                  (100.0f)
#define LEAD_ANGLE_RAD                (0.7853981633974483f)

#define DACA_SINE_DAC_AMP_CODE_F      (1024.0f)
#define DACB_LEAD_DAC_AMP_CODE_F      (1024.0f)

extern ctrl_gt g_daca_sine_phase_pu;
extern ctrl_gt g_daca_sine_value;
extern uint16_t g_daca_sine_code;

extern adc_channel_t g_adc3_daca_channel;
extern adc_gt g_adc3_daca_raw;
extern ctrl_gt g_adc3_sine_pu;

extern ctl_lead_t g_sine_lead;
extern ctrl_gt g_lead_output_gain;
extern float g_lead_mag_at_freq;
extern ctrl_gt g_lead_output_pu;
extern uint16_t g_dacb_lead_code;

GMP_STATIC_INLINE uint16_t ctl_pu_to_dac_code(ctrl_gt value_pu, float amp_code)
{
    float code = 2048.0f + amp_code * (float)value_pu + 0.5f;

    if (code < 0.0f)
    {
        code = 0.0f;
    }

    if (code > 4095.0f)
    {
        code = 4095.0f;
    }

    return (uint16_t)code;
}



//=================================================================================================
// function prototype

void ctl_init(void);
void ctl_mainloop(void);

void clear_all_controllers();

//=================================================================================================
// controller process

// periodic callback function things.

GMP_STATIC_INLINE void ctl_dispatch(void)
{
    g_daca_sine_value = ctl_sin(g_daca_sine_phase_pu);

    g_lead_output_pu = ctl_mul(g_lead_output_gain,
                               ctl_step_lead(&g_sine_lead, g_adc3_sine_pu));

    g_daca_sine_phase_pu += float2ctrl(DACA_SINE_PHASE_STEP_PU);
    if (g_daca_sine_phase_pu >= float2ctrl(1.0f))
    {
        g_daca_sine_phase_pu -= float2ctrl(1.0f);
    }
}


#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_

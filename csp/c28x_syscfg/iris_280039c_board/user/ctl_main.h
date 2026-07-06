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

#include <ctl/framework/cia402_state_machine.h>

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// controller modules with extern
#define DACA_SINE_FREQ_HZ        (100.0f)
#define DACA_SINE_UPDATE_HZ      (10000.0f)
#define DACA_SINE_PHASE_STEP_PU  (DACA_SINE_FREQ_HZ / DACA_SINE_UPDATE_HZ)

extern ctrl_gt g_daca_sine_phase_pu;
extern ctrl_gt g_daca_sine_value;
extern uint16_t g_daca_sine_code;



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

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
#include <ctl/component/intrinsic/discrete/lead_lag.h>//lead_lag

#include <ctl/framework/cia402_state_machine.h>

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// controller modules with extern


//=================================================================================================
// function prototype

void ctl_init(void);
void ctl_mainloop(void);

void clear_all_controllers();

//=================================================================================================
// controller process

//定义lead_comp
extern ctl_lead_t lead_comp;//3
extern ctrl_gt comp_out;//4
extern ctl_lead_t lead_comp2;//7
extern uint16_t current_valid_lead_comp;

// periodic callback function things.
GMP_STATIC_INLINE void ctl_dispatch(void)
{

    if(current_valid_lead_comp == 1){
        //拿着adc的结果进行超前补偿
        comp_out = ctl_step_lead(&lead_comp,input_wave_adc.control_port.value);//3
    }
    else
    {
        comp_out = ctl_step_lead(&lead_comp2,input_wave_adc.control_port.value);//7
    }

}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_

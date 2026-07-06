
//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should define your own controller objects,
// and initilize them.
//
// User should implement a ctl loop function, this
// function would be called every main loop.
//
// User should implement a state machine if you are using
// Controller Nanon framework.
//

#include <gmp_core.h>

#include <ctrl_settings.h>

#include "ctl_main.h"

#include <xplt.peripheral.h>

#include <core/pm/function_scheduler.h>

#include <core/dev/pil_core.h>

//=================================================================================================
// global controller variables

ctl_lead_t lead_comp;//2
ctl_lead_t lead_comp2;
ctrl_gt comp_out = 0.0f;
uint16_t current_valid_lead_comp = 1;


//=================================================================================================
// CTL initialize routine

void ctl_init()
{
    ctl_init_lead_form3(&lead_comp, 3.1415926f / 4.0f, 100.0f, CONTROLLER_FREQUENCY);
    ctl_init_lead_form3(&lead_comp2, 3.1415926f / 4.0f, 100.0f, CONTROLLER_FREQUENCY);

    comp_out = 0.0f;
    current_valid_lead_comp = 1;
}

//=================================================================================================
// CTL endless loop routine

void ctl_mainloop(void)
{
    return;
}


void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{

}

//=================================================================================================
// Controller Tasks

//=================================================================================================
// CiA402 default callback routine


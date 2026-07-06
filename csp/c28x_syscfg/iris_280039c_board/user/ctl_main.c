
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

#include <math.h>

#include "ctl_main.h"

#include <xplt.peripheral.h>

#include <core/pm/function_scheduler.h>

#include <core/dev/pil_core.h>

//=================================================================================================
// global controller variables
ctrl_gt g_daca_sine_phase_pu = float2ctrl(0.0f);
ctrl_gt g_daca_sine_value = float2ctrl(0.0f);
uint16_t g_daca_sine_code = 2048U;

adc_channel_t g_adc3_daca_channel;
adc_gt g_adc3_daca_raw = 0U;
ctrl_gt g_adc3_sine_pu = float2ctrl(0.0f);

ctl_lead_t g_sine_lead;
ctrl_gt g_lead_output_gain = float2ctrl(1.0f);
float g_lead_mag_at_freq = 1.0f;
ctrl_gt g_lead_output_pu = float2ctrl(0.0f);
uint16_t g_dacb_lead_code = 2048U;


//=================================================================================================
// CTL initialize routine

void ctl_init()
{
    ctl_init_adc_channel(&g_adc3_daca_channel,
                         float2ctrl(ADC3_DACA_GAIN_PU),
                         float2ctrl(ADC3_DACA_BIAS_PU),
                         ADC_RESOLUTION_BITS,
                         ADC_IQN);

    ctl_init_lead_form3(&g_sine_lead,
                        LEAD_ANGLE_RAD,
                        LEAD_FREQ_HZ,
                        CONTROL_UPDATE_HZ);

    {
        float omega = 2.0f * 3.14159265358979323846f * LEAD_FREQ_HZ / CONTROL_UPDATE_HZ;
        float cos_omega = cosf(omega);
        float sin_omega = sinf(omega);
        float b0 = (float)g_sine_lead.b0;
        float b1 = (float)g_sine_lead.b1;
        float a1 = (float)g_sine_lead.a1;
        float num_re = b0 + b1 * cos_omega;
        float num_im = -b1 * sin_omega;
        float den_re = 1.0f - a1 * cos_omega;
        float den_im = a1 * sin_omega;
        float num_mag_sq = num_re * num_re + num_im * num_im;
        float den_mag_sq = den_re * den_re + den_im * den_im;

        if (den_mag_sq > 1.0e-12f)
        {
            g_lead_mag_at_freq = sqrtf(num_mag_sq / den_mag_sq);
        }
        else
        {
            g_lead_mag_at_freq = 1.0f;
        }

        if (g_lead_mag_at_freq > 1.0e-6f)
        {
            g_lead_output_gain = float2ctrl(1.0f / g_lead_mag_at_freq);
        }
        else
        {
            g_lead_output_gain = float2ctrl(1.0f);
        }
    }
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


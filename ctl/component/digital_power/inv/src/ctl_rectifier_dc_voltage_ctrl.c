/**
 * @file ctl_rectifier_dc_voltage_ctrl.c
 * @brief Implementation of the grid PWM rectifier DC-link voltage loop.
 */

#include <gmp_core.h>

#include <ctl/component/digital_power/inv/rectifier_dc_voltage_ctrl.h>

static ctrl_gt rectifier_safe_limit(ctrl_gt limit)
{
    if (limit <= float2ctrl(0.000001f))
        return float2ctrl(0.001f);
    return limit;
}

void ctl_init_rectifier_dc_voltage_ctrl(rectifier_dc_voltage_ctrl_t* ctrl, parameter_gt kp, parameter_gt ki,
                                        parameter_gt loop_frequency_hz, parameter_gt voltage_base_v,
                                        parameter_gt current_limit_pu, int id_polarity)
{
    gmp_base_assert(ctrl);
    gmp_base_assert(loop_frequency_hz > 0.0f);
    gmp_base_assert(voltage_base_v > 0.0f);

    ctrl->voltage_base_v = float2ctrl(voltage_base_v);
    ctrl->current_limit_pu = rectifier_safe_limit(float2ctrl(current_limit_pu));
    ctl_set_rectifier_id_polarity(ctrl, id_polarity);

    ctl_init_pi(&ctrl->voltage_pi, kp, ki, loop_frequency_hz);
    ctl_set_pi_limit(&ctrl->voltage_pi, ctrl->current_limit_pu, -ctrl->current_limit_pu);
    ctl_set_pi_int_limit(&ctrl->voltage_pi, ctrl->current_limit_pu, -ctrl->current_limit_pu);

    ctrl->udc_ref_v = float2ctrl(0.0f);
    ctrl->udc_ref_pu = float2ctrl(0.0f);
    ctrl->udc_meas_pu = float2ctrl(0.0f);
    ctrl->udc_error_pu = float2ctrl(0.0f);
    ctrl->id_ref_raw = float2ctrl(0.0f);
    ctrl->id_ref_out = float2ctrl(0.0f);
    ctrl->enable = 0;
    ctrl->saturated = 0;
}

void ctl_enable_rectifier_dc_voltage_ctrl(rectifier_dc_voltage_ctrl_t* ctrl)
{
    gmp_base_assert(ctrl);
    ctrl->enable = 1;
}

void ctl_disable_rectifier_dc_voltage_ctrl(rectifier_dc_voltage_ctrl_t* ctrl)
{
    gmp_base_assert(ctrl);
    ctrl->enable = 0;
    ctl_clear_rectifier_dc_voltage_ctrl(ctrl);
}

void ctl_clear_rectifier_dc_voltage_ctrl(rectifier_dc_voltage_ctrl_t* ctrl)
{
    gmp_base_assert(ctrl);
    ctl_clear_pi(&ctrl->voltage_pi);
    ctrl->udc_ref_pu = ctl_div(ctrl->udc_ref_v, ctrl->voltage_base_v);
    ctrl->udc_meas_pu = float2ctrl(0.0f);
    ctrl->udc_error_pu = float2ctrl(0.0f);
    ctrl->id_ref_raw = float2ctrl(0.0f);
    ctrl->id_ref_out = float2ctrl(0.0f);
    ctrl->saturated = 0;
}

void ctl_set_rectifier_udc_ref(rectifier_dc_voltage_ctrl_t* ctrl, ctrl_gt udc_ref_v)
{
    gmp_base_assert(ctrl);
    ctrl->udc_ref_v = udc_ref_v;
    ctrl->udc_ref_pu = ctl_div(udc_ref_v, ctrl->voltage_base_v);
}

void ctl_set_rectifier_current_limit(rectifier_dc_voltage_ctrl_t* ctrl, ctrl_gt current_limit_pu)
{
    gmp_base_assert(ctrl);
    ctrl->current_limit_pu = rectifier_safe_limit(current_limit_pu);
    ctl_set_pi_limit(&ctrl->voltage_pi, ctrl->current_limit_pu, -ctrl->current_limit_pu);
    ctl_set_pi_int_limit(&ctrl->voltage_pi, ctrl->current_limit_pu, -ctrl->current_limit_pu);
}

void ctl_set_rectifier_id_polarity(rectifier_dc_voltage_ctrl_t* ctrl, int id_polarity)
{
    gmp_base_assert(ctrl);
    ctrl->id_polarity = (id_polarity < 0) ? float2ctrl(-1.0f) : float2ctrl(1.0f);
}

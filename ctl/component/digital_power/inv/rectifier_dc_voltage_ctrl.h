/**
 * @file rectifier_dc_voltage_ctrl.h
 * @brief Minimal DC-link voltage outer loop for a grid-following PWM rectifier.
 *
 * The controller consumes the measured DC-link voltage in controller
 * per-unit, accepts the user reference in volts, and produces a signed
 * d-axis current reference in per-unit.  The inner positive-sequence dq
 * current controller remains owned by the GFL core.
 */

#ifndef _FILE_RECTIFIER_DC_VOLTAGE_CTRL_H_
#define _FILE_RECTIFIER_DC_VOLTAGE_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <ctl/component/intrinsic/continuous/continuous_pi.h>

typedef struct _tag_rectifier_dc_voltage_ctrl_t
{
    /* Engineering-unit command and feedback. */
    ctrl_gt udc_ref_v;
    ctrl_gt udc_meas_pu;
    ctrl_gt udc_error_pu;

    /* Controller-domain references and output. */
    ctrl_gt udc_ref_pu;
    ctrl_gt id_ref_raw;
    ctrl_gt id_ref_out;

    /* Configuration. */
    ctrl_gt voltage_base_v;
    ctrl_gt id_polarity;
    ctrl_gt current_limit_pu;

    /* PI and state flags. */
    ctl_pi_t voltage_pi;
    fast_gt enable;
    fast_gt saturated;
} rectifier_dc_voltage_ctrl_t;

void ctl_init_rectifier_dc_voltage_ctrl(rectifier_dc_voltage_ctrl_t* ctrl, parameter_gt kp, parameter_gt ki,
                                        parameter_gt loop_frequency_hz, parameter_gt voltage_base_v,
                                        parameter_gt current_limit_pu, int id_polarity);

void ctl_enable_rectifier_dc_voltage_ctrl(rectifier_dc_voltage_ctrl_t* ctrl);
void ctl_disable_rectifier_dc_voltage_ctrl(rectifier_dc_voltage_ctrl_t* ctrl);
void ctl_clear_rectifier_dc_voltage_ctrl(rectifier_dc_voltage_ctrl_t* ctrl);

void ctl_set_rectifier_udc_ref(rectifier_dc_voltage_ctrl_t* ctrl, ctrl_gt udc_ref_v);
void ctl_set_rectifier_current_limit(rectifier_dc_voltage_ctrl_t* ctrl, ctrl_gt current_limit_pu);
void ctl_set_rectifier_id_polarity(rectifier_dc_voltage_ctrl_t* ctrl, int id_polarity);

/**
 * @brief Execute one outer-loop sample.
 * @param ctrl Controller instance.
 * @param udc_meas_pu Measured DC-link voltage in controller per-unit.
 * @return Signed d-axis current reference in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_rectifier_dc_voltage_ctrl(rectifier_dc_voltage_ctrl_t* ctrl,
                                                               ctrl_gt udc_meas_pu)
{
    ctrl->udc_meas_pu = udc_meas_pu;

    if (!ctrl->enable)
    {
        ctrl->udc_error_pu = 0;
        ctrl->id_ref_raw = 0;
        ctrl->id_ref_out = 0;
        ctrl->saturated = 0;
        return ctrl->id_ref_out;
    }

    ctrl->udc_error_pu = ctrl->udc_ref_pu - ctrl->udc_meas_pu;
    ctrl->id_ref_raw = ctl_step_pi_ser(&ctrl->voltage_pi, ctrl->udc_error_pu);
    ctrl->id_ref_out = ctl_mul(ctrl->id_ref_raw, ctrl->id_polarity);

    ctrl->saturated =
        (ctl_abs(ctrl->id_ref_raw) >= (ctrl->current_limit_pu - float2ctrl(0.000001f))) ? 1 : 0;

    return ctrl->id_ref_out;
}

#ifdef __cplusplus
}
#endif

#endif /* _FILE_RECTIFIER_DC_VOLTAGE_CTRL_H_ */

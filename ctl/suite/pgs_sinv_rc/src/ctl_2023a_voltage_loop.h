/**
 * @file ctl_2023a_voltage_loop.h
 * @brief Header-only suite-local 2023A standalone output-voltage QPR outer loop.
 *
 * The loop consumes PU RL-terminal voltage feedback and produces a limited
 * PU peak inductor-current reference for the validated SINV current loop.
 * It is intentionally suite-local and does not change the reusable
 * sinv_rc_core component.
 */
#ifndef _CTL_2023A_VOLTAGE_LOOP_H_
#define _CTL_2023A_VOLTAGE_LOOP_H_

#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _tag_ctl_2023a_voltage_loop
{
    ctl_qpr_t qpr;

    ctrl_gt voltage_ref_peak;
    ctrl_gt voltage_ref_peak_target;
    ctrl_gt voltage_ref_peak_step;
    ctrl_gt voltage_ref_inst;
    ctrl_gt voltage_feedback;
    ctrl_gt voltage_error;

    ctrl_gt current_ref_unsat;
    ctrl_gt current_ref_controller_limited;
    ctrl_gt load_current_feedback;
    ctrl_gt load_current_ff;
    ctrl_gt capacitor_current_ff;
    ctrl_gt current_ref_total_unsat;
    ctrl_gt current_ref;
    ctrl_gt voltage_loop_output_limit;
    ctrl_gt current_ref_limit;
    ctrl_gt current_ref_effective_limit;
    ctrl_gt load_current_ff_gain;
    ctrl_gt cap_current_ff_gain;
    ctrl_gt cap_ff_scale_pu;
    fast_gt flag_enable_load_current_ff;
    fast_gt flag_enable_cap_current_ff;
    fast_gt flag_saturated;
} ctl_2023a_voltage_loop_t;

GMP_STATIC_INLINE void ctl_clear_2023a_voltage_loop(
    ctl_2023a_voltage_loop_t* loop)
{
    ctl_clear_qpr_controller(&loop->qpr);
    loop->voltage_ref_peak = float2ctrl(0.0f);
    loop->voltage_ref_inst = float2ctrl(0.0f);
    loop->voltage_feedback = float2ctrl(0.0f);
    loop->voltage_error = float2ctrl(0.0f);
    loop->current_ref_unsat = float2ctrl(0.0f);
    loop->current_ref_controller_limited = float2ctrl(0.0f);
    loop->load_current_feedback = float2ctrl(0.0f);
    loop->load_current_ff = float2ctrl(0.0f);
    loop->capacitor_current_ff = float2ctrl(0.0f);
    loop->current_ref_total_unsat = float2ctrl(0.0f);
    loop->current_ref = float2ctrl(0.0f);
    loop->flag_saturated = 0;
}

GMP_STATIC_INLINE void ctl_init_2023a_voltage_loop(
    ctl_2023a_voltage_loop_t* loop,
    parameter_gt voltage_ref_rms_v,
    parameter_gt voltage_base_v,
    parameter_gt current_base_a,
    parameter_gt filter_capacitance_f,
    parameter_gt output_frequency_hz,
    parameter_gt soft_start_time_s,
    parameter_gt qpr_kp,
    parameter_gt qpr_kr,
    parameter_gt qpr_wi_hz,
    parameter_gt current_ref_limit_peak_pu,
    parameter_gt voltage_loop_output_limit_pu,
    parameter_gt load_current_ff_gain,
    parameter_gt cap_current_ff_gain,
    fast_gt enable_load_current_ff,
    fast_gt enable_cap_current_ff,
    parameter_gt controller_frequency_hz)
{
    parameter_gt voltage_ref_peak_pu;
    parameter_gt soft_start_samples;
    parameter_gt effective_limit;

    gmp_base_assert(voltage_ref_rms_v >= 0.0f);
    gmp_base_assert(voltage_base_v > 0.0f);
    gmp_base_assert(current_base_a > 0.0f);
    gmp_base_assert(filter_capacitance_f > 0.0f);
    gmp_base_assert(output_frequency_hz > 0.0f);
    gmp_base_assert(controller_frequency_hz > 2.0f * output_frequency_hz);
    gmp_base_assert(soft_start_time_s > 0.0f);
    gmp_base_assert(qpr_kp >= 0.0f);
    gmp_base_assert(qpr_kr >= 0.0f);
    gmp_base_assert(qpr_wi_hz > 0.0f);
    gmp_base_assert(current_ref_limit_peak_pu > 0.0f);
    gmp_base_assert(voltage_loop_output_limit_pu > 0.0f);
    gmp_base_assert(load_current_ff_gain >= 0.0f);
    gmp_base_assert(cap_current_ff_gain >= 0.0f);
    gmp_base_assert(enable_load_current_ff == 0 ||
        enable_load_current_ff == 1);
    gmp_base_assert(enable_cap_current_ff == 0 ||
        enable_cap_current_ff == 1);

    ctl_init_qpr_controller(&loop->qpr, qpr_kp, qpr_kr,
        output_frequency_hz, qpr_wi_hz, controller_frequency_hz);

    voltage_ref_peak_pu =
        1.4142135623730951f * voltage_ref_rms_v / voltage_base_v;
    soft_start_samples = soft_start_time_s * controller_frequency_hz;

    loop->voltage_ref_peak_target = float2ctrl(voltage_ref_peak_pu);
    if (soft_start_samples > 1.0f)
    {
        loop->voltage_ref_peak_step =
            float2ctrl(voltage_ref_peak_pu / soft_start_samples);
    }
    else
    {
        loop->voltage_ref_peak_step = loop->voltage_ref_peak_target;
    }

    effective_limit = voltage_loop_output_limit_pu;
    if (current_ref_limit_peak_pu < effective_limit)
    {
        effective_limit = current_ref_limit_peak_pu;
    }

    loop->voltage_loop_output_limit =
        float2ctrl(voltage_loop_output_limit_pu);
    loop->current_ref_limit = float2ctrl(current_ref_limit_peak_pu);
    loop->current_ref_effective_limit = float2ctrl(effective_limit);
    loop->load_current_ff_gain = float2ctrl(load_current_ff_gain);
    loop->cap_current_ff_gain = float2ctrl(cap_current_ff_gain);
    loop->cap_ff_scale_pu = float2ctrl(
        6.2831853071795865f * output_frequency_hz *
        filter_capacitance_f * voltage_base_v / current_base_a);
    loop->flag_enable_load_current_ff = enable_load_current_ff;
    loop->flag_enable_cap_current_ff = enable_cap_current_ff;

    ctl_clear_2023a_voltage_loop(loop);
}

/**
 * @brief Execute the voltage outer loop once at the current-loop rate.
 * @param loop Voltage-loop object.
 * @param sin_theta Fixed-frequency internal phase-generator sine output.
 * @param cos_theta Fixed-frequency internal phase-generator cosine output.
 * @param voltage_feedback_pu Filtered/sample-held/quantized RL voltage in PU.
 * @param load_current_feedback_pu Load current in CTRL_CURRENT_BASE peak PU.
 * @return Limited instantaneous IL reference in PU peak units.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_2023a_voltage_loop(
    ctl_2023a_voltage_loop_t* loop,
    ctrl_gt sin_theta,
    ctrl_gt cos_theta,
    ctrl_gt voltage_feedback_pu,
    ctrl_gt load_current_feedback_pu)
{
    ctl_qpr_t qpr_before_step;
    ctrl_gt zero = float2ctrl(0.0f);

    if (loop->voltage_ref_peak < loop->voltage_ref_peak_target)
    {
        loop->voltage_ref_peak += loop->voltage_ref_peak_step;
        if (loop->voltage_ref_peak > loop->voltage_ref_peak_target)
            loop->voltage_ref_peak = loop->voltage_ref_peak_target;
    }

    loop->voltage_ref_inst = ctl_mul(loop->voltage_ref_peak, sin_theta);
    loop->voltage_feedback = voltage_feedback_pu;
    loop->voltage_error = loop->voltage_ref_inst - loop->voltage_feedback;

    /* Preserve the resonant state so saturation can freeze updates that
       would push the QPR farther into the active current-reference limit. */
    qpr_before_step = loop->qpr;
    loop->current_ref_unsat = ctl_step_qpr_controller(
        &loop->qpr, loop->voltage_error);

    loop->current_ref_controller_limited = ctl_sat(
        loop->current_ref_unsat, loop->voltage_loop_output_limit,
        -loop->voltage_loop_output_limit);

    loop->load_current_feedback = load_current_feedback_pu;
    if (loop->flag_enable_load_current_ff)
    {
        loop->load_current_ff = ctl_mul(
            loop->load_current_ff_gain, loop->load_current_feedback);
    }
    else
    {
        loop->load_current_ff = zero;
    }
    if (loop->flag_enable_cap_current_ff)
    {
        loop->capacitor_current_ff = ctl_mul(
            ctl_mul(loop->cap_current_ff_gain, loop->cap_ff_scale_pu),
            ctl_mul(loop->voltage_ref_peak, cos_theta));
    }
    else
    {
        loop->capacitor_current_ff = zero;
    }

    loop->current_ref_total_unsat =
        loop->current_ref_controller_limited +
        loop->load_current_ff + loop->capacitor_current_ff;
    loop->current_ref = ctl_sat(loop->current_ref_total_unsat,
        loop->current_ref_limit, -loop->current_ref_limit);
    loop->flag_saturated =
        loop->current_ref != loop->current_ref_total_unsat;

    if (loop->flag_saturated &&
        ((loop->current_ref_total_unsat > loop->current_ref_limit &&
          loop->voltage_error > zero) ||
         (loop->current_ref_total_unsat < -loop->current_ref_limit &&
          loop->voltage_error < zero)))
    {
        loop->qpr = qpr_before_step;
    }

    return loop->current_ref;
}

#ifdef __cplusplus
}
#endif

#endif // _CTL_2023A_VOLTAGE_LOOP_H_


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

#include "ctl_settings_defaults.h"

#include "ctl_main.h"

#include <xplt.peripheral.h>

#include <core/dev/pil_core.h>

//=================================================================================================
// global controller variables

// System framework
cia402_sm_t cia402_sm;

// Control Law Core
// Current controller, Power controller / Voltage controller
gfl_pq_ctrl_t pq_ctrl;
inv_neg_ctrl_init_t gfl_neg_init;
inv_neg_ctrl_t neg_current_ctrl;
gfl_inv_ctrl_init_t gfl_init;
gfl_inv_ctrl_t inv_ctrl;

// Input channel

// Output channel: SPWM modulator / SVPWM modulator / NPC modulator
#if defined USING_NPC_MODULATOR
npc_modulator_t spwm;
#else
spwm_modulator_t spwm;
#endif // USING_NPC_MODULATOR

// Protection module

// ADC Calibrator
adc_bias_calibrator_t adc_calibrator;
#if defined SPECIFY_ENABLE_ADC_CALIBRATE
volatile fast_gt flag_enable_adc_calibrator = 1;
#else
volatile fast_gt flag_enable_adc_calibrator = 0;
#endif
volatile fast_gt index_adc_calibrator = 0;
uint32_t pq_loop_tick = 0;
ctl_vector2_t gfl_current_ref_target;
ctl_slope_limiter_t gfl_current_ref_ramp[2];
rectifier_dc_voltage_ctrl_t rectifier_dc_voltage_ctrl;
rectifier_cmd_t rectifier_cmd;
rectifier_status_t rectifier_status;

#if BUILD_LEVEL == 6
static uint32_t rectifier_pll_lock_ticks = 0;
static uint32_t rectifier_pll_loss_ticks = 0;
static uint32_t rectifier_state_ticks = 0;
static uint32_t rectifier_voltage_loop_tick = 0;
static ctrl_gt rectifier_udc_ramp_ref_v = 0;
static fast_gt rectifier_cia402_ready = 0;
static fast_gt rectifier_manual_active = 0;
#endif

// User commands

//=================================================================================================
// BUILD_LEVEL lifecycle helpers

static void ctl_reset_control_state(void)
{
    // Keep every stop path deterministic: no stale integrator, reference, or
    // compensation state may survive the next commissioning attempt.
    ctl_disable_gfl_inv(&inv_ctrl);
    ctl_disable_gfl_inv_pll(&inv_ctrl);
    ctl_disable_gfl_inv_decouple(&inv_ctrl);
    ctl_disable_gfl_inv_active_damp(&inv_ctrl);
    ctl_disable_gfl_inv_lead_compensator(&inv_ctrl);
    ctl_disable_neg_inv(&neg_current_ctrl);
    ctl_disable_gfl_pq_ctrl(&pq_ctrl);

    ctl_clear_gfl_inv_with_PLL(&inv_ctrl);
    ctl_clear_neg_inv(&neg_current_ctrl);
    ctl_clear_gfl_pq(&pq_ctrl);

    inv_ctrl.flag_enable_current_ctrl = 0;
    ctl_set_gfl_inv_current(&inv_ctrl, 0, 0);
    ctl_vector2_clear(&neg_current_ctrl.idqn_set);
    ctl_vector2_clear(&neg_current_ctrl.vdqn_set);
    ctl_set_gfl_pq_ref(&pq_ctrl, 0, 0);
    ctl_vector2_clear(&gfl_current_ref_target);
    ctl_clear_slope_limiter(&gfl_current_ref_ramp[phase_d]);
    ctl_clear_slope_limiter(&gfl_current_ref_ramp[phase_q]);
    pq_loop_tick = 0;
    ctl_disable_rectifier_dc_voltage_ctrl(&rectifier_dc_voltage_ctrl);
}

static void ctl_set_gfl_current_target(ctrl_gt id, ctrl_gt iq)
{
    gfl_current_ref_target.dat[phase_d] = id;
    gfl_current_ref_target.dat[phase_q] = iq;
}

static void ctl_configure_build_level(fast_gt enable_algorithms)
{
    // PLL synchronisation is prepared before the final output gate.  Current,
    // negative-sequence, compensation and P/Q algorithms remain stopped until
    // ctl_enable_pwm() calls this helper with enable_algorithms=1.
#if BUILD_LEVEL == 0
    // PLL-only commissioning: update Vd/Vq and PLL diagnostics, never output.
    ctl_set_gfl_inv_openloop_mode(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);
    ctl_enable_gfl_inv_pll(&inv_ctrl);

#elif BUILD_LEVEL == 1
    ctl_set_gfl_inv_openloop_mode(&inv_ctrl);
    ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, float2ctrl(GFL_OPEN_LOOP_VD_PU),
                                     float2ctrl(GFL_OPEN_LOOP_VQ_PU));

#elif BUILD_LEVEL == 2
    // Basic PLL-synchronised positive-sequence current loop.
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_current_target(float2ctrl(GFL_CURRENT_LEVEL2_ID_PU),
                               float2ctrl(GFL_CURRENT_LEVEL2_IQ_PU));
    ctl_enable_gfl_inv_pll(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);

#elif BUILD_LEVEL == 3
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_current_target(float2ctrl(GFL_CURRENT_LEVEL3_ID_PU),
                               float2ctrl(GFL_CURRENT_LEVEL3_IQ_PU));
    ctl_enable_gfl_inv_pll(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);

#elif BUILD_LEVEL == 4
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_current_target(float2ctrl(GFL_CURRENT_LEVEL4_ID_PU),
                               float2ctrl(GFL_CURRENT_LEVEL4_IQ_PU));
    ctl_enable_gfl_inv_pll(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);

#elif BUILD_LEVEL == 5
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_current_target(0, 0);
    ctl_enable_gfl_inv_pll(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);

#elif BUILD_LEVEL == 6
    // BL6 reuses the PLL and positive-sequence current loop.  The DC-link
    // outer loop is started by the explicit rectifier state machine only
    // after the PLL-lock and zero-current holds have completed.
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_current_target(0, 0);
    ctl_enable_gfl_inv_pll(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);
#endif // BUILD_LEVEL

    if (!enable_algorithms)
    {
        inv_ctrl.flag_enable_system = 0;
        inv_ctrl.flag_enable_current_ctrl = 0;
        ctl_disable_neg_inv(&neg_current_ctrl);
        ctl_disable_gfl_pq_ctrl(&pq_ctrl);
        ctl_disable_gfl_inv_decouple(&inv_ctrl);
        ctl_disable_gfl_inv_active_damp(&inv_ctrl);
        ctl_disable_gfl_inv_lead_compensator(&inv_ctrl);
        ctl_set_gfl_inv_current(&inv_ctrl, 0, 0);
        ctl_vector2_clear(&gfl_current_ref_target);
    }
    else
    {
#if BUILD_LEVEL >= 1
        ctl_enable_gfl_inv(&inv_ctrl);
#endif
#if BUILD_LEVEL >= 2
        inv_ctrl.flag_enable_current_ctrl = 1;
#endif
#if BUILD_LEVEL >= 3 && GMP_GFL_ENABLE_NEGATIVE_SEQUENCE
        ctl_enable_neg_current_inv(&neg_current_ctrl);
#endif
#if BUILD_LEVEL >= 4 && GMP_GFL_ENABLE_DECOUPLING
        ctl_enable_gfl_inv_decouple(&inv_ctrl);
#endif
#if BUILD_LEVEL >= 4 && GMP_GFL_ENABLE_ACTIVE_DAMPING
        ctl_enable_gfl_inv_active_damp(&inv_ctrl);
#endif
#if BUILD_LEVEL >= 4 && GMP_GFL_ENABLE_LEAD_COMPENSATOR
        ctl_enable_gfl_inv_lead_compensator(&inv_ctrl);
#endif
#if BUILD_LEVEL == 5
        ctl_enable_gfl_pq_ctrl(&pq_ctrl);
        ctl_set_gfl_pq_ref(&pq_ctrl, float2ctrl(GFL_ACTIVE_POWER_REF_PU),
                           float2ctrl(GFL_REACTIVE_POWER_REF_PU));
#endif
    }
}

//=================================================================================================
// CTL initialize routine

void ctl_init()
{
    //
    // stop here and wait for user start the motor controller
    //
    ctl_fast_disable_output();

    //
    // GFL inverter init objects
    //
    gfl_init.fs = CONTROLLER_FREQUENCY;
    gfl_init.v_base = CTRL_VOLTAGE_BASE;
    gfl_init.v_grid = GFL_GRID_VOLTAGE_PU;
    gfl_init.i_base = CTRL_CURRENT_BASE;
    gfl_init.freq_base = GFL_GRID_FREQUENCY_HZ;

    gfl_init.grid_filter_L = GFL_GRID_FILTER_INDUCTANCE_H;
    gfl_init.grid_filter_C = GFL_GRID_FILTER_CAPACITANCE_F;

    ctl_auto_tuning_gfl_inv(&gfl_init);
    ctl_init_gfl_inv(&inv_ctrl, &gfl_init);
    // gfl_core initializes the AC feedback filters but leaves the attached
    // DC-link channels cleared.  BL6 consumes the filtered Udc signal, so
    // initialize both DC feedback filters explicitly at the same cutoffs.
    ctl_init_filter_iir1_lpf(&inv_ctrl.filter_idc, gfl_init.fs, gfl_init.current_adc_fc);
    ctl_init_filter_iir1_lpf(&inv_ctrl.filter_udc, gfl_init.fs, gfl_init.voltage_adc_fc);

    ctl_auto_tuning_neg_inv(&gfl_neg_init, &gfl_init);
    ctl_init_neg_inv(&neg_current_ctrl, &gfl_neg_init);
    ctl_attach_neg_inv_to_gfl(&neg_current_ctrl, &inv_ctrl);

    //
    // init SPWM modulator
    //
#if defined USING_NPC_MODULATOR
    ctl_init_npc_modulator(&spwm, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP, &inv_ctrl.adc_iabc->value, float2ctrl(0.02),
                           float2ctrl(0.005));
#else
    ctl_init_spwm_modulator(&spwm, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP, &inv_ctrl.adc_iabc->value, float2ctrl(0.02),
                            float2ctrl(0.005));
#endif // USING_NPC_MODULATOR

    //
    // Power controller
    //
    ctl_init_gfl_pq(&pq_ctrl, GFL_PQ_ACTIVE_KP, GFL_PQ_ACTIVE_KI, GFL_PQ_REACTIVE_KP, GFL_PQ_REACTIVE_KI,
                    GFL_PQ_CURRENT_LIMIT_PU, GFL_PQ_LOOP_FREQUENCY_HZ);
    ctl_attach_gfl_pq_to_core(&pq_ctrl, &inv_ctrl);
#if BUILD_LEVEL == 6
    ctl_init_slope_limiter(&gfl_current_ref_ramp[phase_d], RECTIFIER_ID_REF_SLEW_PU_S,
                           -RECTIFIER_ID_REF_SLEW_PU_S, CONTROLLER_FREQUENCY);
    ctl_init_slope_limiter(&gfl_current_ref_ramp[phase_q], RECTIFIER_ID_REF_SLEW_PU_S,
                           -RECTIFIER_ID_REF_SLEW_PU_S, CONTROLLER_FREQUENCY);
#else
    ctl_init_slope_limiter(&gfl_current_ref_ramp[phase_d], GFL_CURRENT_REF_SLEW_PU_S,
                           -GFL_CURRENT_REF_SLEW_PU_S, CONTROLLER_FREQUENCY);
    ctl_init_slope_limiter(&gfl_current_ref_ramp[phase_q], GFL_CURRENT_REF_SLEW_PU_S,
                           -GFL_CURRENT_REF_SLEW_PU_S, CONTROLLER_FREQUENCY);
#endif
    ctl_vector2_clear(&gfl_current_ref_target);
    pq_loop_tick = 0;

    ctl_init_rectifier_dc_voltage_ctrl(&rectifier_dc_voltage_ctrl, RECTIFIER_DC_VOLTAGE_KP,
                                       RECTIFIER_DC_VOLTAGE_KI,
                                       RECTIFIER_DC_VOLTAGE_LOOP_FREQUENCY_HZ, CTRL_VOLTAGE_BASE,
                                       RECTIFIER_CURRENT_LIMIT_PU, RECTIFIER_ID_POLARITY);
    ctl_set_rectifier_udc_ref(&rectifier_dc_voltage_ctrl, float2ctrl(RECTIFIER_DC_VOLTAGE_REF_V));
    rectifier_cmd.enable = 0;
    rectifier_cmd.clear_fault = 0;
    rectifier_cmd.udc_ref_v = float2ctrl(RECTIFIER_DC_VOLTAGE_REF_V);
    rectifier_cmd.iq_ref_pu = float2ctrl(RECTIFIER_IQ_REF_PU);
    rectifier_cmd.current_limit_pu = float2ctrl(RECTIFIER_CURRENT_LIMIT_PU);
    rectifier_cmd.manual_current_mode = 0;
    rectifier_cmd.manual_id_ref_pu = 0;
    rectifier_cmd.manual_iq_ref_pu = 0;
    rectifier_status.state = RECTIFIER_STATE_IDLE;
    rectifier_status.fault_bits = RECTIFIER_FAULT_NONE;
    rectifier_status.udc_ref_v = rectifier_cmd.udc_ref_v;
    rectifier_status.udc_meas_v = 0;
    rectifier_status.udc_error_v = 0;
    rectifier_status.id_ref_pu = 0;
    rectifier_status.iq_ref_pu = 0;
    rectifier_status.id_meas_pu = 0;
    rectifier_status.iq_meas_pu = 0;
    rectifier_status.pll_freq_pu = 0;
    rectifier_status.pll_error = 0;
    rectifier_status.pll_locked = 0;
    rectifier_status.pwm_enabled = 0;
    rectifier_status.current_limited = 0;
    rectifier_status.voltage_pi_saturated = 0;
#if BUILD_LEVEL == 6
    rectifier_pll_lock_ticks = 0;
    rectifier_pll_loss_ticks = 0;
    rectifier_state_ticks = 0;
    rectifier_voltage_loop_tick = 0;
    rectifier_udc_ramp_ref_v = float2ctrl(0.0f);
    rectifier_cia402_ready = 0;
    rectifier_manual_active = 0;
#endif

    // All levels start from one safe baseline, then select only the
    // algorithms assigned to the requested BUILD_LEVEL.
    ctl_reset_control_state();
    ctl_configure_build_level(0);

    //
    // init and config CiA402 standard state machine
    //
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = GFL_CIA402_OPERATION_ENABLE_DELAY_MS;

#if defined SPECIFY_PC_ENVIRONMENT
    cia402_sm.flag_enable_control_word = 0;
    cia402_sm.current_cmd = CIA402_CMD_ENABLE_OPERATION;
#endif // SPECIFY_PC_ENVIRONMENT

#if BUILD_LEVEL >= 3

    // NOTICE:
    // if grid connect is request disable switch delay from CIA402_SM_SWITCH_ON_DISABLED to CIA402_SM_SWITCHED_ON
    // or a longer judgment time can lead to failure to connect to the grid.
    cia402_sm.minimum_transit_delay[CIA402_SM_READY_TO_SWITCH_ON] = 0;
    cia402_sm.minimum_transit_delay[CIA402_SM_SWITCHED_ON] = 0;

#endif // BUILD_LEVEL

    //
    // init ADC Calibrator
    //
    ctl_init_adc_calibrator(&adc_calibrator, GFL_ADC_CALIBRATOR_FC_HZ, GFL_ADC_CALIBRATOR_Q, CONTROLLER_FREQUENCY);

    if (flag_enable_adc_calibrator)
    {
        ctl_enable_adc_calibrator(&adc_calibrator);
    }
}

//=================================================================================================
// CTL endless loop routine

void ctl_mainloop(void)
{
    cia402_dispatch(&cia402_sm);

    return;
}

void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{
#if defined ENABLE_GMP_DL_PIL_SIM
    ctl_input_callback_pil(rx);

    ctl_dispatch();

    ctl_output_callback_pil(tx);
#endif // defined ENABLE_GMP_DL_PIL_SIM
}

#if defined ENABLE_GMP_DL_PIL_SIM
time_gt gmp_base_get_ctrl_tick(void)
{
    return inv_ctrl.isr_tick / ((uint32_t)CONTROLLER_FREQUENCY / 1000);
}
#endif // defined ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
// CiA402 default callback routine

void ctl_enable_pwm()
{
#if BUILD_LEVEL == 0
    // BL0 is a measurement/PLL-only level.  Refuse the final PWM enable
    // request even if CiA402 reaches Operation Enabled automatically in SIL.
    ctl_fast_disable_output();
    ctl_disable_gfl_inv(&inv_ctrl);
#elif BUILD_LEVEL == 6
    // CiA402 calls this once on entry to Operation Enabled.  BL6 keeps the
    // physical gate disabled until its own PLL-lock and zero-current holds
    // have completed.
    rectifier_cia402_ready = 1;
    ctl_fast_disable_output();
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);
    ctl_enable_gfl_inv_pll(&inv_ctrl);
#else
    ctl_configure_build_level(1);
    ctl_fast_enable_output();
#endif // BUILD_LEVEL
}

void ctl_disable_pwm()
{
#if BUILD_LEVEL == 6
    rectifier_cia402_ready = 0;
    ctl_fast_disable_output();
    rectifier_status.pwm_enabled = 0;
    rectifier_status.state = RECTIFIER_STATE_IDLE;
    rectifier_status.fault_bits = RECTIFIER_FAULT_NONE;
    ctl_disable_rectifier_dc_voltage_ctrl(&rectifier_dc_voltage_ctrl);
    ctl_clear_pid(&inv_ctrl.pid_idq[phase_d]);
    ctl_clear_pid(&inv_ctrl.pid_idq[phase_q]);
    ctl_set_gfl_inv_current(&inv_ctrl, 0, 0);
    ctl_vector2_clear(&gfl_current_ref_target);
    ctl_clear_slope_limiter(&gfl_current_ref_ramp[phase_d]);
    ctl_clear_slope_limiter(&gfl_current_ref_ramp[phase_q]);
#else
    ctl_fast_disable_output();

    // Disable every algorithm independently of the physical output gate,
    // then clear all dynamic state and references.
    ctl_reset_control_state();

    // Keep the synchronising PLL alive while the power stage is stopped so
    // the next CiA402 enable request can still wait for a real lock event.
#if BUILD_LEVEL == 0 || BUILD_LEVEL >= 2
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);
    ctl_enable_gfl_inv_pll(&inv_ctrl);
#endif
#endif
}

fast_gt ctl_check_pll_locked(void)
{
    ctrl_gt pll_erro = ctl_abs(ctl_get_gfl_pll_error(&inv_ctrl));

    // grid connected, judge if PLL is ready.
    if (ctl_is_gfl_grid_connected(&inv_ctrl))
    {
        if (pll_erro < CTRL_SPLL_EPSILON)
            return 1;
        else
            return 0;
    }

    // not connect to gird
    else
        return 1;
}

fast_gt ctl_exec_dc_voltage_ready(void)
{
#if BUILD_LEVEL == 6
    ctrl_gt udc_v = ctl_mul(inv_ctrl.filter_udc.out, float2ctrl(CTRL_VOLTAGE_BASE));
    float udc_v_float = ctrl2float(udc_v);
    if ((udc_v_float != udc_v_float) || udc_v_float > 3.402823e38f || udc_v_float < -3.402823e38f)
        return 0;
    return (udc_v_float >= RECTIFIER_DC_UNDERVOLTAGE_V && udc_v_float <= RECTIFIER_DC_OVERVOLTAGE_V) ? 1 : 0;
#else
    return 1;
#endif
}

#if BUILD_LEVEL == 6
static fast_gt rectifier_ctrl_value_valid(ctrl_gt value)
{
    float converted = ctrl2float(value);
    return (converted == converted) && converted <= 3.402823e38f && converted >= -3.402823e38f;
}

static ctrl_gt rectifier_measured_udc_v(void)
{
    return ctl_mul(inv_ctrl.filter_udc.out, float2ctrl(CTRL_VOLTAGE_BASE));
}

static ctrl_gt rectifier_command_current_limit(void)
{
    ctrl_gt limit = rectifier_cmd.current_limit_pu;
    if (!rectifier_ctrl_value_valid(limit) || limit <= 0)
        limit = float2ctrl(RECTIFIER_CURRENT_LIMIT_PU);
    return ctl_sat(limit, float2ctrl(RECTIFIER_CURRENT_LIMIT_PU), float2ctrl(0.001f));
}

static ctrl_gt rectifier_command_udc_ref(void)
{
    ctrl_gt ref = rectifier_cmd.udc_ref_v;
    if (!rectifier_ctrl_value_valid(ref) || ref <= float2ctrl(0.0f))
        ref = float2ctrl(RECTIFIER_DC_VOLTAGE_REF_V);
    return ctl_sat(ref, float2ctrl(RECTIFIER_DC_OVERVOLTAGE_V - 1.0f),
                   float2ctrl(RECTIFIER_DC_UNDERVOLTAGE_V));
}

static ctrl_gt rectifier_safe_current_command(ctrl_gt value)
{
    ctrl_gt limit = rectifier_command_current_limit();
    if (!rectifier_ctrl_value_valid(value))
        return float2ctrl(0.0f);
    return ctl_sat(value, limit, -limit);
}

static void rectifier_clear_actuation(void)
{
    ctl_fast_disable_output();
    rectifier_status.pwm_enabled = 0;
    ctl_disable_gfl_inv(&inv_ctrl);
    inv_ctrl.flag_enable_current_ctrl = 0;
    ctl_disable_rectifier_dc_voltage_ctrl(&rectifier_dc_voltage_ctrl);
    ctl_clear_pid(&inv_ctrl.pid_idq[phase_d]);
    ctl_clear_pid(&inv_ctrl.pid_idq[phase_q]);
    ctl_set_gfl_inv_current(&inv_ctrl, 0, 0);
    ctl_vector2_clear(&gfl_current_ref_target);
    ctl_clear_slope_limiter(&gfl_current_ref_ramp[phase_d]);
    ctl_clear_slope_limiter(&gfl_current_ref_ramp[phase_q]);
    rectifier_voltage_loop_tick = 0;
    rectifier_state_ticks = 0;
}

static void rectifier_enter_fault(uint32_t fault_bits)
{
    rectifier_status.fault_bits |= fault_bits;
    rectifier_clear_actuation();
    rectifier_status.state = RECTIFIER_STATE_FAULT;
    rectifier_pll_lock_ticks = 0;
    rectifier_pll_loss_ticks = 0;
}

static uint32_t rectifier_basic_faults(fast_gt active)
{
    uint32_t faults = RECTIFIER_FAULT_NONE;
    ctrl_gt udc_pu = inv_ctrl.filter_udc.out;
    ctrl_gt udc_v = rectifier_measured_udc_v();

    if (!rectifier_ctrl_value_valid(udc_pu) || !rectifier_ctrl_value_valid(udc_v) ||
        !rectifier_ctrl_value_valid(inv_ctrl.idq.dat[phase_d]) ||
        !rectifier_ctrl_value_valid(inv_ctrl.idq.dat[phase_q]))
    {
        faults |= RECTIFIER_FAULT_ADC_INVALID;
        return faults;
    }

    ctrl_gt current_limit = float2ctrl(RECTIFIER_SOFTWARE_OVERCURRENT_PU);
    if (ctl_abs(inv_ctrl.iabc.dat[phase_A]) > current_limit ||
        ctl_abs(inv_ctrl.iabc.dat[phase_B]) > current_limit ||
        ctl_abs(inv_ctrl.iabc.dat[phase_C]) > current_limit)
        faults |= RECTIFIER_FAULT_SOFTWARE_OVERCURRENT;

    if (udc_v > float2ctrl(RECTIFIER_DC_OVERVOLTAGE_V))
        faults |= RECTIFIER_FAULT_DC_OVERVOLTAGE;

    if (active && udc_v < float2ctrl(RECTIFIER_DC_UNDERVOLTAGE_V))
        faults |= RECTIFIER_FAULT_DC_UNDERVOLTAGE;

    return faults;
}

static void rectifier_apply_current_target(ctrl_gt id_ref, ctrl_gt iq_ref)
{
    ctrl_gt current_limit = rectifier_command_current_limit();
    ctrl_gt id_sq = ctl_mul(id_ref, id_ref);
    ctrl_gt iq_sq = ctl_mul(iq_ref, iq_ref);
    ctrl_gt mag_sq = id_sq + iq_sq;
    ctrl_gt limit_sq = ctl_mul(current_limit, current_limit);
    rectifier_status.current_limited = 0;

    if (mag_sq > limit_sq)
    {
        ctrl_gt scale = ctl_sqrt(ctl_div(limit_sq, mag_sq));
        id_ref = ctl_mul(id_ref, scale);
        iq_ref = ctl_mul(iq_ref, scale);
        rectifier_status.current_limited = 1;
    }

    rectifier_status.id_ref_pu = id_ref;
    rectifier_status.iq_ref_pu = iq_ref;
    gfl_current_ref_target.dat[phase_d] = id_ref;
    gfl_current_ref_target.dat[phase_q] = iq_ref;
}

static void rectifier_seed_zero_current_voltage(void)
{
#if GMP_GFL_ENABLE_EXTERNAL_FEEDFORWARD
    ctl_set_pid_integrator(&inv_ctrl.pid_idq[phase_d], 0);
    ctl_set_pid_integrator(&inv_ctrl.pid_idq[phase_q], 0);
#else
    // With feed-forward disabled, preload the current PI with the measured
    // grid voltage so enabling the bridge is bumpless at zero current.
    ctl_set_pid_integrator(&inv_ctrl.pid_idq[phase_d], inv_ctrl.vdq.dat[phase_d]);
    ctl_set_pid_integrator(&inv_ctrl.pid_idq[phase_q], inv_ctrl.vdq.dat[phase_q]);
#endif
    ctl_vector2_copy(&inv_ctrl.vdq_out, &inv_ctrl.vdq);
    ctl_vector2_copy(&inv_ctrl.vdq_out_comp, &inv_ctrl.vdq);
    ctl_ct_ipark2(&inv_ctrl.vdq_out_comp, &inv_ctrl.phasor, &inv_ctrl.vab_pos);
    inv_ctrl.vab0_out.dat[phase_alpha] = inv_ctrl.vab_pos.dat[phase_alpha];
    inv_ctrl.vab0_out.dat[phase_beta] = inv_ctrl.vab_pos.dat[phase_beta];
    inv_ctrl.vab0_out.dat[phase_0] = 0;
}

static void rectifier_set_inner_enabled(void)
{
    ctl_configure_build_level(1);
    rectifier_seed_zero_current_voltage();
    ctl_enable_gfl_inv(&inv_ctrl);
    inv_ctrl.flag_enable_current_ctrl = 1;
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);
    ctl_enable_gfl_inv_pll(&inv_ctrl);
    ctl_fast_enable_output();
    rectifier_status.pwm_enabled = 1;
}

static void rectifier_update_status(void)
{
    ctrl_gt udc_v = rectifier_measured_udc_v();
#ifdef USING_DSOGI_PLL
    rectifier_status.pll_freq_pu = inv_ctrl.pll.srf_pll.freq_pu;
#else
    rectifier_status.pll_freq_pu = inv_ctrl.pll.freq_pu;
#endif
    rectifier_status.udc_ref_v = rectifier_command_udc_ref();
    rectifier_status.udc_meas_v = udc_v;
    rectifier_status.udc_error_v = rectifier_status.udc_ref_v - udc_v;
    rectifier_status.id_meas_pu = inv_ctrl.idq.dat[phase_d];
    rectifier_status.iq_meas_pu = inv_ctrl.idq.dat[phase_q];
    rectifier_status.pll_error = ctl_get_gfl_pll_error(&inv_ctrl);
    rectifier_status.pll_locked = ctl_check_pll_locked();
    rectifier_status.voltage_pi_saturated = rectifier_dc_voltage_ctrl.saturated;
}

static void rectifier_step_voltage_outer_loop(void)
{
    ctrl_gt target_v = rectifier_command_udc_ref();
    ctrl_gt step_v = float2ctrl(RECTIFIER_UDC_REF_SLEW_V_S / RECTIFIER_DC_VOLTAGE_LOOP_FREQUENCY_HZ);
    if (rectifier_udc_ramp_ref_v < target_v)
    {
        rectifier_udc_ramp_ref_v += step_v;
        if (rectifier_udc_ramp_ref_v > target_v)
            rectifier_udc_ramp_ref_v = target_v;
    }
    else
    {
        rectifier_udc_ramp_ref_v -= step_v;
        if (rectifier_udc_ramp_ref_v < target_v)
            rectifier_udc_ramp_ref_v = target_v;
    }

    ctl_set_rectifier_current_limit(&rectifier_dc_voltage_ctrl, rectifier_command_current_limit());
    ctl_set_rectifier_udc_ref(&rectifier_dc_voltage_ctrl, rectifier_udc_ramp_ref_v);
    ctl_enable_rectifier_dc_voltage_ctrl(&rectifier_dc_voltage_ctrl);
    ctrl_gt id_ref = ctl_step_rectifier_dc_voltage_ctrl(&rectifier_dc_voltage_ctrl, inv_ctrl.filter_udc.out);
    ctrl_gt iq_ref = rectifier_cmd.manual_current_mode ?
                         rectifier_safe_current_command(rectifier_cmd.manual_iq_ref_pu) :
                         rectifier_safe_current_command(rectifier_cmd.iq_ref_pu);
    rectifier_apply_current_target(id_ref, iq_ref);
}

void ctl_step_rectifier_bl6(void)
{
    uint32_t basic_faults;
    fast_gt pll_locked;

    rectifier_update_status();
    basic_faults = rectifier_basic_faults(rectifier_status.state >= RECTIFIER_STATE_ZERO_CURRENT);

    if (rectifier_status.state != RECTIFIER_STATE_FAULT &&
        (rectifier_cmd.enable || rectifier_status.state != RECTIFIER_STATE_IDLE) && basic_faults)
    {
        rectifier_enter_fault(basic_faults);
    }

    if (rectifier_status.state == RECTIFIER_STATE_FAULT)
    {
        rectifier_update_status();
        if (rectifier_cmd.clear_fault &&
            rectifier_basic_faults(0) == RECTIFIER_FAULT_NONE &&
            ctl_exec_dc_voltage_ready())
        {
            rectifier_cmd.clear_fault = 0;
            rectifier_status.fault_bits = RECTIFIER_FAULT_NONE;
            rectifier_status.state = RECTIFIER_STATE_IDLE;
            rectifier_clear_actuation();
        }
        return;
    }

    if (!rectifier_cmd.enable || !rectifier_cia402_ready)
    {
        rectifier_clear_actuation();
        rectifier_status.state = RECTIFIER_STATE_IDLE;
        rectifier_update_status();
        return;
    }

    pll_locked = ctl_check_pll_locked();
    if (rectifier_status.state == RECTIFIER_STATE_IDLE)
    {
        rectifier_status.state = RECTIFIER_STATE_WAIT_PLL;
        rectifier_pll_lock_ticks = 0;
        rectifier_state_ticks = 0;
        rectifier_clear_actuation();
    }

    if (rectifier_status.state == RECTIFIER_STATE_WAIT_PLL)
    {
        rectifier_apply_current_target(0, 0);
        if (pll_locked)
            ++rectifier_pll_lock_ticks;
        else
            rectifier_pll_lock_ticks = 0;

        if (rectifier_pll_lock_ticks >=
            (uint32_t)((CONTROLLER_FREQUENCY * RECTIFIER_PLL_LOCK_HOLD_MS) / 1000))
        {
            rectifier_set_inner_enabled();
            rectifier_udc_ramp_ref_v = rectifier_measured_udc_v();
            rectifier_status.state = RECTIFIER_STATE_ZERO_CURRENT;
            rectifier_state_ticks = 0;
        }
    }
    else
    {
        if (pll_locked)
            rectifier_pll_loss_ticks = 0;
        else
            ++rectifier_pll_loss_ticks;
        if (rectifier_pll_loss_ticks >=
            (uint32_t)((CONTROLLER_FREQUENCY * RECTIFIER_PLL_LOCK_HOLD_MS) / 1000))
        {
            rectifier_enter_fault(RECTIFIER_FAULT_PLL_LOSS);
            return;
        }

        ++rectifier_state_ticks;
        if (rectifier_cmd.manual_current_mode)
        {
            ctl_disable_rectifier_dc_voltage_ctrl(&rectifier_dc_voltage_ctrl);
            rectifier_manual_active = 1;
            rectifier_apply_current_target(rectifier_safe_current_command(rectifier_cmd.manual_id_ref_pu),
                                           rectifier_safe_current_command(rectifier_cmd.manual_iq_ref_pu));
            if (rectifier_status.state == RECTIFIER_STATE_ZERO_CURRENT)
                rectifier_status.state = RECTIFIER_STATE_RUN;
        }
        else
        {
            if (rectifier_manual_active)
            {
                rectifier_manual_active = 0;
                rectifier_udc_ramp_ref_v = rectifier_measured_udc_v();
                rectifier_status.state = RECTIFIER_STATE_VOLTAGE_RAMP;
                rectifier_state_ticks = 0;
            }
            if (rectifier_status.state == RECTIFIER_STATE_ZERO_CURRENT)
            {
                rectifier_apply_current_target(0, 0);
                ctl_disable_rectifier_dc_voltage_ctrl(&rectifier_dc_voltage_ctrl);
            }
            else if (++rectifier_voltage_loop_tick >= RECTIFIER_DC_VOLTAGE_LOOP_DIVIDER)
            {
                rectifier_voltage_loop_tick = 0;
                rectifier_step_voltage_outer_loop();
            }
            if (rectifier_status.state == RECTIFIER_STATE_ZERO_CURRENT &&
                rectifier_state_ticks >=
                    (uint32_t)((CONTROLLER_FREQUENCY * RECTIFIER_ZERO_CURRENT_HOLD_MS) / 1000))
            {
                rectifier_status.state = RECTIFIER_STATE_VOLTAGE_RAMP;
                rectifier_state_ticks = 0;
            }
            else if (rectifier_status.state == RECTIFIER_STATE_VOLTAGE_RAMP &&
                     ctl_abs(rectifier_udc_ramp_ref_v - rectifier_command_udc_ref()) <
                         float2ctrl(RECTIFIER_UDC_REF_SLEW_V_S / RECTIFIER_DC_VOLTAGE_LOOP_FREQUENCY_HZ))
            {
                rectifier_status.state = RECTIFIER_STATE_RUN;
            }
        }
    }

    rectifier_update_status();
}
#endif

fast_gt ctl_exec_adc_calibration(void)
{
    //
    // 1. ADC Auto calibrate
    //
    if (flag_enable_adc_calibrator)
    {
        if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) && ctl_is_adc_calibrator_result_valid(&adc_calibrator))
        {

            // index_adc_calibrator == 13, for Ibus
            if (index_adc_calibrator == 13)
            {
                // vbus get result
                idc.bias = idc.bias + ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), idc.gain);

                // move to next position
                index_adc_calibrator += 1;

                // adc calibrate process done.
                flag_enable_adc_calibrator = 0;

                // clear INV controller
                ctl_clear_gfl_inv_with_PLL(&inv_ctrl);

                // ADC Calibrator complete here.
                //ctl_enable_gfl_inv(&inv_ctrl);
            }

            // index_adc_calibrator == 12, for Vbus
            else if (index_adc_calibrator == 12)
            {
                // vbus get result
                //udc.bias = udc.bias + ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), udc.gain);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            // index_adc_calibrator == 11 ~ 9, for Vuvw
            else if (index_adc_calibrator <= 11 && index_adc_calibrator >= 9)
            {
                // vuvw get result
                uuvw.bias[index_adc_calibrator - 9] =
                    uuvw.bias[index_adc_calibrator - 9] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), uuvw.gain[index_adc_calibrator - 9]);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            // index_adc_calibrator == 8 ~ 6, for Vabc
            else if (index_adc_calibrator <= 8 && index_adc_calibrator >= 6)
            {
                // vabc get result
                vabc.bias[index_adc_calibrator - 6] =
                    vabc.bias[index_adc_calibrator - 6] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), vabc.gain[index_adc_calibrator - 6]);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            // index_adc_calibrator == 5 ~ 3, for Iabc
            else if (index_adc_calibrator <= 5 && index_adc_calibrator >= 3)
            {

                // iabc get result
                iabc.bias[index_adc_calibrator - 3] =
                    iabc.bias[index_adc_calibrator - 3] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), iabc.gain[index_adc_calibrator - 3]);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            // index_adc_calibrator == 2 ~ 0, for Iuvw
            else if (index_adc_calibrator <= 2)
            {
                // iuvw get result
                iuvw.bias[index_adc_calibrator] =
                    iuvw.bias[index_adc_calibrator] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), iuvw.gain[index_adc_calibrator]);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            // over-range protection
            if (index_adc_calibrator > 13)
                flag_enable_adc_calibrator = 0;
        }

        // ADC calibrate is not complete
        return 0;
    }

    // skip calibrate routine
    return 1;
}

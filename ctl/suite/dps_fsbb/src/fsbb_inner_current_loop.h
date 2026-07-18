#ifndef FSBB_INNER_CURRENT_LOOP_H_
#define FSBB_INNER_CURRENT_LOOP_H_

#include <ctl/component/digital_power/dcdc/dcdc_core.h>

GMP_STATIC_INLINE ctrl_gt ctl_step_fsbb_inductor_current_loop(ctl_dcdc_core_t* core, ctrl_gt i_L_ref_cmd,
                                                              ctrl_gt i_L_ref_max, ctrl_gt i_L_ref_min)
{
    /* 1. 限制外环或测试程序给出的电感电流指令 */
    ctrl_gt i_L_ref_limited = ctl_sat(i_L_ref_cmd, i_L_ref_max, i_L_ref_min);

    /* 2. 对电感电流参考进行斜坡处理 */
    core->i_ramp_ref = ctl_step_slope_limiter(&core->ramp_i, i_L_ref_limited);

    /* 3. 计算电感电流误差 */
    ctrl_gt error_i_L = core->i_ramp_ref - core->filter_i_L.out;

    /* 4. 电感电流PI产生等效电压命令 */
    core->v_out_formal = ctl_step_pid_ser(&core->current_pid, error_i_L);

    /* 5. 限制最终送给调制器的电压命令 */
    core->v_out_formal = ctl_sat(core->v_out_formal, core->out_max, core->out_min);



    return core->v_out_formal;
}

#endif // FSBB_INNER_CURRENT_LOOP_H_

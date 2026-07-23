%% SDPE MATLAB 初始化脚本
% 项目：PGS 单相逆变器通用控制
% 由 tools/SDPE_v2 生成。请勿直接编辑生成的变量。

% 说明：
%   适用于所有 pgs_sinv_rc 项目的平台无关控制契约。

%% 项目元数据
PGS_SINV_RC_COMMON_SDPE_PROJECT_ID = 'pgs_sinv_rc_common';  % 项目唯一标识符

PGS_SINV_RC_COMMON_SDPE_PROJECT_SUITE = 'pgs_sinv_rc';      % 所属项目套件

PGS_SINV_RC_COMMON_SDPE_PROJECT_VERSION = '1.0.0';          % 项目版本号

PGS_SINV_RC_COMMON_SDPE_PROJECT_UPDATED_AT = '2026-07-15';  % 项目最后更新日期

%% 控制特性开关
% 启用频率自适应重复控制器的延迟插入功能。
SINV_ENABLE_REPETITIVE_CONTROL = true;

% 为闭环电流环构建等级启用电网电压前馈。
SINV_ENABLE_GRID_VOLTAGE_FEEDFORWARD = true;

%% 运行时配置
% 允许 ENABLE_OPERATION 完成完整的 CiA402 启动序列。
CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH = true;

%% 需求参数绑定
% 电网额定频率，单位：Hz。
CTRL_GRID_FREQUENCY = 50.0;

% SOGI 锁相环的比例增益。
CTRL_PLL_KP = 10.0;

% SOGI 锁相环的积分时间常数，单位：秒。
CTRL_PLL_TI = 0.02;

% 锁相环误差滤波器的截止频率，单位：Hz。
CTRL_PLL_LPF_FC = 20.0;

% 锁相环频率误差锁定的阈值，单位：标幺值 (PU)。
CTRL_SPLL_EPSILON = 0.005;

% 功率测量低通滤波器的截止频率，单位：Hz。
CTRL_PQ_LPF_FC = 200.0;

% 峰值电流指令限制，单位：标幺值 (PU)。
CTRL_CURRENT_LIMIT_PU = 0.9;

% P/Q 参考生成器使用的最小电压幅值，单位：标幺值 (PU)。
CTRL_GRID_VMIN_PU = 0.1;

% 有功功率指令的压摆率限制，单位：PU/s。
CTRL_P_SLEW_PU_S = 5.0;

% 无功功率指令的压摆率限制，单位：PU/s。
CTRL_Q_SLEW_PU_S = 5.0;

% PWM 死区补偿使用的电流死区，单位：标幺值 (PU)。
CTRL_CURRENT_DB_PU = 0.01;

% QPR 电流环的穿越频率目标，单位：Hz。
SINV_CURRENT_LOOP_BANDWIDTH_HZ = 600.0;

% FDRC（频率自适应重复控制）跟踪的最低基波频率，单位：Hz。
CTRL_FDRC_MIN_FREQ = 45.0;

% 重复控制开始学习前的稳定等待时间，单位：毫秒。
SINV_FDRC_ENABLE_DELAY_MS = 300;

% 重复控制的学习增益。
SINV_FDRC_LEARNING_GAIN = 0.10;

% FDRC 鲁棒性滤波器的截止频率，单位：Hz。
SINV_FDRC_Q_FILTER_HZ = 1000.0;

% 控制器采样周期中的 plant 延迟补偿步数。
SINV_FDRC_LEAD_STEPS = 3.0;

% 电流误差阈值，超过此值时重复控制学习将被冻结，单位：标幺值 (PU)。
SINV_FDRC_FREEZE_ERROR_PU = 0.05;

% BUILD_LEVEL 1：正弦 H 桥的电压幅值参考，单位：标幺值 (PU)。
SINV_LEVEL1_VOLTAGE_REF_PU = 0.35;

% BUILD_LEVEL 2：阻性负载下的峰值电流指令，单位：标幺值 (PU)。
SINV_LEVEL2_CURRENT_REF_PEAK_PU = 0.20;

% BUILD_LEVEL 3：带符号的电网有功功率指令；正值代表向电网馈电（输出功率）。
SINV_LEVEL3_ACTIVE_POWER_REF_PU = 0.10;

% BUILD_LEVEL 3：电网无功功率指令。
SINV_LEVEL3_REACTIVE_POWER_REF_PU = 0.0;

% BUILD_LEVEL 4：实测有功功率的闭环控制目标值，单位：标幺值 (PU)。
SINV_LEVEL4_ACTIVE_POWER_REF_PU = 0.15;

% 有功功率外环的比例增益。
SINV_POWER_LOOP_KP = 0.6;

% 有功功率外环的积分增益，单位：每秒。
SINV_POWER_LOOP_KI = 8.0;

% BUILD_LEVEL 5：物理直流母线电压目标值，单位：伏特 (V)。
SINV_DC_BUS_REF_V = 60.0;

% 直流母线电压外环的比例增益。
SINV_DC_BUS_LOOP_KP = 0.8;

% 直流母线电压外环的积分增益，单位：每秒。
SINV_DC_BUS_LOOP_KI = 12.0;

% 外环有功功率指令的对称限幅，单位：标幺值 (PU)。
SINV_OUTER_LOOP_POWER_LIMIT_PU = 0.65;

% 功率环和直流母线电压外环的执行频率，单位：Hz。
SINV_OUTER_LOOP_FREQUENCY_HZ = 1000.0;

% 操作使能的最小过渡延迟时间，单位：毫秒。
SINV_CIA402_OPERATION_ENABLE_DELAY_MS = 100;

%% 本地辅助函数
function value = sdpe_select(condition, true_value, false_value)
% SDPE 选择函数：根据条件返回对应的值
if condition
    value = true_value;
else
    value = false_value;
end
end
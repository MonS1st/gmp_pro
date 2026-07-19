function results = run_fsbb_build4_cvcc_load_test()
%RUN_FSBB_BUILD4_CVCC_LOAD_TEST Automated 20/10/20-ohm Build 4 UDP SIL test.

root = fileparts(mfilename('fullpath'));
source_model = 'MCS_STD_FSBB_MODEL_2022b';
test_model = 'MCS_STD_FSBB_MODEL_2022b_cvcc_test';
stop_time = 2.5;
exe = fullfile(root, 'x64', 'Debug', 'Digital_Power_Suite_FSBB_SIL_Env.exe');
result_dir = fullfile(root, 'validation');

assert_test_configuration(root);
if ~isfile(exe)
    error('FSBB:SILExecutableMissing', 'Build Debug|x64 first: %s', exe);
end
if ~isfolder(result_dir)
    mkdir(result_dir);
end

sdpe_dir = fullfile(root, 'sdpe_mgr');
mex_dir = fullfile(root, '..', '..', '..', '..', '..', 'tools', ...
    'gmp_sil', 'udp_helper_v2', 'mdl_asio_helper', 'bin', 'x64', 'Debug');
addpath(sdpe_dir);
if isfolder(mex_dir)
    addpath(mex_dir);
end
evalin('base', 'sdpe_dps_fsbb_simulate_settings_matlab_init;');

temp_root = tempname;
mkdir(temp_root);
temp_model_file = fullfile(temp_root, [test_model '.slx']);
copyfile(fullfile(root, [source_model '.slx']), temp_model_file);
load_system(temp_model_file);
model_cleanup = onCleanup(@() close_test_model(test_model, temp_root)); %#ok<NASGU>

if isfolder(mex_dir)
    init_callback = sprintf(['addpath(''%s''); addpath(''%s''); ' ...
        'evalin(''base'',''sdpe_dps_fsbb_simulate_settings_matlab_init;'');'], ...
        slash_path(sdpe_dir), slash_path(mex_dir));
else
    init_callback = sprintf(['addpath(''%s''); ' ...
        'evalin(''base'',''sdpe_dps_fsbb_simulate_settings_matlab_init;'');'], ...
        slash_path(sdpe_dir));
end
set_param(test_model, 'PreLoadFcn', init_callback, 'InitFcn', init_callback, ...
    'StopTime', num2str(stop_time, 17));

set_param([test_model '/Constant'], 'Value', '24');
original_load = [test_model '/Series RLC Branch'];
set_param(original_load, 'BranchType', 'R', 'Resistance', '20');
add_parallel_load_branch(test_model, original_load);
add_monitor_loggers(test_model);
set_param(test_model, 'SimulationCommand', 'update');

start_info = System.Diagnostics.ProcessStartInfo;
start_info.FileName = exe;
start_info.WorkingDirectory = root;
start_info.UseShellExecute = false;
start_info.CreateNoWindow = true;
controller = System.Diagnostics.Process.Start(start_info);
controller_cleanup = onCleanup(@() stop_controller(controller)); %#ok<NASGU>
pause(0.5);
if controller.HasExited
    error('FSBB:SILStartFailed', 'SIL executable exited with code %d.', controller.ExitCode);
end

sim_in = Simulink.SimulationInput(test_model);
sim_in = sim_in.setModelParameter('StopTime', num2str(stop_time, 17), ...
    'ReturnWorkspaceOutputs', 'on');
sim_out = sim(sim_in);

channels = cell(1, 11);
for k = 1:11
    channels{k} = sim_out.get(sprintf('fsbb_ch%d', k));
    fprintf('CH%d samples=%d time=[%.9g, %.9g]\n', k, numel(channels{k}.Time), ...
        double(channels{k}.Time(1)), double(channels{k}.Time(end)));
end
load_command = sim_out.get('fsbb_load_command');
save(fullfile(result_dir, 'build4_cvcc_load_switch_raw.mat'), ...
    'channels', 'load_command');

results = evaluate_results(channels, load_command);
save_results(result_dir, results, channels, load_command);
plot_results(result_dir, results, channels);
print_summary(results);
end

function assert_test_configuration(root)
header = fileread(fullfile(root, 'sdpe_mgr', 'sdpe_dps_fsbb_simulate_settings.h'));
required = {
    '#define\s+BUILD_LEVEL\s+\(4\)', 'BUILD_LEVEL=4';
    '#define\s+FSBB_BUILD4_SELF_TEST_ENABLE\s+\(0\)', 'FSBB_BUILD4_SELF_TEST_ENABLE=0';
    '#define\s+FSBB_INPUT_VOLTAGE_NOMINAL\s+\(24\.0f\)', 'Vin=24 V';
    '#define\s+FSBB_DEFAULT_OUTPUT_VOLTAGE\s+\(12\.0f\)', 'Vref=12 V';
    '#define\s+FSBB_DEFAULT_CURRENT_LIMIT\s+\(0\.8f\)', 'Ilimit=0.8 A'};
for k = 1:size(required, 1)
    if isempty(regexp(header, required{k, 1}, 'once'))
        error('FSBB:TestConfigurationMismatch', 'Generated settings do not select %s.', required{k, 2});
    end
end
end

function add_parallel_load_branch(model, original_load)
load_switch = [model '/CVCC Test Ideal Switch'];
parallel_load = [model '/CVCC Parallel 20 Ohm Load'];
step_on = [model '/CVCC Load Step On'];
step_off = [model '/CVCC Load Step Off'];
sum_block = [model '/CVCC Load Command'];
command_logger = [model '/CVCC Load Command Logger'];

% The SPS Breaker waits for a current zero before opening and therefore
% cannot reliably remove a resistive branch carrying DC.  Ideal Switch is
% the matching SPS device intended for direct 0/1 control in this circuit.
add_block('spsIdealSwitchLib/Ideal Switch', load_switch, ...
    'Ron', '0.01', 'Lon', '0', 'IC', '0', ...
    'Rs', '1e6', 'Cs', 'inf', 'Measurements', 'off', ...
    'Position', [1190 665 1245 735]);
add_block(original_load, parallel_load, 'Position', [1335 665 1385 753]);
set_param(parallel_load, 'BranchType', 'R', 'Resistance', '20');

add_block('simulink/Sources/Step', step_on, ...
    'Time', '0.8', 'Before', '0', 'After', '1', ...
    'Position', [900 650 955 680]);
add_block('simulink/Sources/Step', step_off, ...
    'Time', '1.6', 'Before', '0', 'After', '-1', ...
    'Position', [900 705 955 735]);
add_block('simulink/Math Operations/Add', sum_block, ...
    'Inputs', '++', 'Position', [1020 670 1050 720]);
add_block('simulink/Sinks/To Workspace', command_logger, ...
    'VariableName', 'fsbb_load_command', 'SaveFormat', 'Timeseries', ...
    'Position', [1090 745 1230 775]);

on_ph = get_param(step_on, 'PortHandles');
off_ph = get_param(step_off, 'PortHandles');
sum_ph = get_param(sum_block, 'PortHandles');
switch_ph = get_param(load_switch, 'PortHandles');
logger_ph = get_param(command_logger, 'PortHandles');
load_ph = get_param(original_load, 'PortHandles');
parallel_ph = get_param(parallel_load, 'PortHandles');

add_line(model, on_ph.Outport(1), sum_ph.Inport(1), 'autorouting', 'on');
add_line(model, off_ph.Outport(1), sum_ph.Inport(2), 'autorouting', 'on');
add_line(model, sum_ph.Outport(1), switch_ph.Inport(1), 'autorouting', 'on');
add_line(model, sum_ph.Outport(1), logger_ph.Inport(1), 'autorouting', 'on');

% Branch from the existing output and common-negative electrical nets.
add_line(model, load_ph.LConn(1), switch_ph.LConn(1), 'autorouting', 'on');
add_line(model, switch_ph.RConn(1), parallel_ph.LConn(1), 'autorouting', 'on');
add_line(model, parallel_ph.RConn(1), load_ph.RConn(1), 'autorouting', 'on');
end

function add_monitor_loggers(model)
selectors = [2 2 3 3 4 4 5 5 6 6 7];
ports = [1 2 1 2 1 2 1 2 1 2 1];
for k = 1:11
    source = sprintf('%s/Bus Selector%d', model, selectors(k));
    logger = sprintf('%s/CVCC Logger CH%d', model, k);
    add_block('simulink/Sinks/To Workspace', logger, ...
        'VariableName', sprintf('fsbb_ch%d', k), ...
        'SaveFormat', 'Timeseries', 'Decimation', '10', ...
        'MaxDataPoints', '1000000', ...
        'Position', [1510 25 + 32 * k 1640 47 + 32 * k]);
    src_ph = get_param(source, 'PortHandles');
    dst_ph = get_param(logger, 'PortHandles');
    add_line(model, src_ph.Outport(ports(k)), dst_ph.Inport(1), 'autorouting', 'on');
end
end

function results = evaluate_results(ch, load_command)
stage_windows = [0.74 0.79; 1.35 1.55; 2.20 2.45];
expected = [12.0 0.6 0; 8.0 0.8 1; 12.0 0.6 0];
stages = repmat(struct('window_s', [], 'vout_v', 0, 'iout_a', 0, ...
    'mode', 0, 'mode_fraction', 0), 1, 3);
for k = 1:3
    stages(k).window_s = stage_windows(k, :);
    stages(k).vout_v = window_mean(ch{2}, stage_windows(k, :));
    stages(k).iout_a = window_mean(ch{4}, stage_windows(k, :));
    [stages(k).mode, stages(k).mode_fraction] = ...
        window_mode(ch{8}, stage_windows(k, :), expected(k, 3));
end

active = ch{11}.Time >= 0.5;
min_error = abs(double(ch{7}.Data(:)) - ...
    min(double(ch{5}.Data(:)), double(ch{6}.Data(:))));
il = abs(double(ch{3}.Data(:)));

checks = struct;
checks.stage1 = abs(stages(1).vout_v - expected(1, 1)) <= 0.75 && ...
    abs(stages(1).iout_a - expected(1, 2)) <= 0.08 && stages(1).mode_fraction >= 0.95;
checks.stage2 = abs(stages(2).vout_v - expected(2, 1)) <= 0.75 && ...
    abs(stages(2).iout_a - expected(2, 2)) <= 0.08 && stages(2).mode_fraction >= 0.95;
checks.stage3 = abs(stages(3).vout_v - expected(3, 1)) <= 0.75 && ...
    abs(stages(3).iout_a - expected(3, 2)) <= 0.08 && stages(3).mode_fraction >= 0.95;
checks.command_is_min = max(min_error) <= 0.01;
checks.no_fault = all(abs(double(ch{10}.Data(:))) < 0.5);
checks.pwm_enabled = all(double(ch{11}.Data(active)) > 0.5);
checks.current_bounded = max(il) <= 1.15 && mean(il >= 0.98) <= 0.05;
load_command_stages = [window_mean(load_command, [0.20 0.75]), ...
    window_mean(load_command, [0.90 1.50]), ...
    window_mean(load_command, [1.80 2.40])];
checks.load_sequence = max(abs(load_command_stages - [0 1 0])) <= 0.01;

results = struct;
results.test = 'DPS FSBB Build 4 CV-CC-CV load switch';
results.stage_windows_s = stage_windows;
results.stages = stages;
results.max_command_min_error_a = max(min_error);
results.max_abs_inductor_current_a = max(il);
results.inductor_limit_fraction = mean(il >= 0.98);
results.load_command_stages = load_command_stages;
results.checks = checks;
results.passed = all(cell2mat(struct2cell(checks)));
end

function value = window_mean(series, window)
mask = series.Time >= window(1) & series.Time <= window(2);
if ~any(mask)
    error('FSBB:MissingSamples', 'No logged samples in [%.3f, %.3f] s.', window(1), window(2));
end
value = mean(double(series.Data(mask)), 'all');
end

function [mode, fraction] = window_mode(series, window, expected)
mask = series.Time >= window(1) & series.Time <= window(2);
values = round(double(series.Data(mask)));
mode = median(values, 'all');
fraction = mean(values == expected, 'all');
end

function save_results(result_dir, results, channels, load_command)
mat_file = fullfile(result_dir, 'build4_cvcc_load_switch_results.mat');
save(mat_file, 'results', 'channels', 'load_command');

text_file = fullfile(result_dir, 'build4_cvcc_load_switch_results.txt');
fid = fopen(text_file, 'w');
if fid < 0
    error('FSBB:ResultFileOpenFailed', 'Cannot open %s.', text_file);
end
file_cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'DPS FSBB Build 4 CV-CC-CV load switch\n');
for k = 1:3
    s = results.stages(k);
    fprintf(fid, 'stage%d [%.2f, %.2f] s: Vout=%.6f V, Iout=%.6f A, CH8=%.0f, mode_fraction=%.6f\n', ...
        k, s.window_s(1), s.window_s(2), s.vout_v, s.iout_a, s.mode, s.mode_fraction);
end
fprintf(fid, 'max |CH7-min(CH5,CH6)| = %.9f A\n', results.max_command_min_error_a);
fprintf(fid, 'max |IL| = %.6f A\n', results.max_abs_inductor_current_a);
fprintf(fid, 'IL >= 0.98 A fraction = %.9f\n', results.inductor_limit_fraction);
fprintf(fid, 'load command stages = [%.6f %.6f %.6f]\n', ...
    results.load_command_stages);
names = fieldnames(results.checks);
for k = 1:numel(names)
    fprintf(fid, '%s = %d\n', names{k}, results.checks.(names{k}));
end
fprintf(fid, 'OVERALL = %s\n', pass_text(results.passed));
end

function plot_results(result_dir, results, ch)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1250 950]);
tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(ch{1}.Time, ch{1}.Data, '-', ch{2}.Time, ch{2}.Data, '-', 'LineWidth', 1.0);
grid on; ylabel('Voltage (V)'); legend('Vin', 'Vout', 'Location', 'best');
title(sprintf('Build 4 CV-CC-CV load switch - %s', pass_text(results.passed)));

nexttile;
plot(ch{3}.Time, ch{3}.Data, '-', ch{4}.Time, ch{4}.Data, '-', 'LineWidth', 1.0);
grid on; ylabel('Current (A)'); legend('IL', 'Iout', 'Location', 'best');

nexttile;
plot(ch{5}.Time, ch{5}.Data, '-', ch{6}.Time, ch{6}.Data, '-', ...
    ch{7}.Time, ch{7}.Data, '--', 'LineWidth', 1.0);
grid on; ylabel('Reference (A)'); legend('iL ref CV', 'iL ref CC', 'iL ref cmd', 'Location', 'best');

nexttile;
stairs(ch{8}.Time, ch{8}.Data, '-', 'LineWidth', 1.0); hold on;
stairs(ch{9}.Time, ch{9}.Data, '-');
stairs(ch{10}.Time, ch{10}.Data, '-');
stairs(ch{11}.Time, ch{11}.Data, '-');
grid on; ylabel('State'); xlabel('Time (s)');
legend('CC dominant', 'CiA402 state', 'Fault code', 'PWM enable', 'Location', 'best');

for ax = findall(fig, 'Type', 'axes')'
    xline(ax, 0.8, ':', 'HandleVisibility', 'off');
    xline(ax, 1.6, ':', 'HandleVisibility', 'off');
    xlim(ax, [0 2.5]);
end
exportgraphics(fig, fullfile(result_dir, 'build4_cvcc_load_switch_waveforms.png'), 'Resolution', 170);
close(fig);
end

function print_summary(results)
for k = 1:3
    s = results.stages(k);
    fprintf('STAGE%d Vout=%.6f V Iout=%.6f A CH8=%.0f mode_fraction=%.6f\n', ...
        k, s.vout_v, s.iout_a, s.mode, s.mode_fraction);
end
fprintf('CVCC_TEST_OVERALL=%s\n', pass_text(results.passed));
end

function text = pass_text(passed)
if passed
    text = 'PASS';
else
    text = 'FAIL';
end
end

function stop_controller(controller)
try
    if ~isempty(controller) && ~controller.HasExited
        controller.Kill;
        controller.WaitForExit(2000);
    end
catch
end
end

function close_test_model(model, temp_root)
try
    close_system(model, 0);
catch
end
try
    if isfolder(temp_root) && startsWith(temp_root, tempdir)
        rmdir(temp_root, 's');
    end
catch
end
end

function path = slash_path(path)
path = strrep(path, '\', '/');
end

function metrics = run_2023a_voltage_validation(stop_time)
%RUN_2023A_VOLTAGE_VALIDATION Validate the 2023A standalone voltage loop.
%
% This function is intentionally fixed to
% PGS_2023A_SINGLE_INV_RLOAD_2022b. It does not resolve or fall back to a
% standard model. It requires generated BUILD_LEVEL=2, SINV_APP_MODE=1,
% and an existing Debug/x64 SIL executable. Results are written only after
% the simulation has completed and the exact controller process is stopped.

arguments
    stop_time (1,1) double {mustBeFinite, mustBePositive} = 3.0
end

expected_build_level = 2;
expected_app_mode = 1;
uo_target_rms_v = 24.0;
uo_tolerance_v = 0.2;
nominal_frequency_hz = 50.0;
frequency_tolerance_hz = 0.2;
io_tolerance_a = 0.05;
maximum_uo_thd_percent = 2.0;
maximum_saturation_fraction = 0.01;
maximum_saturation_duration_s = 0.1 / nominal_frequency_hz;
steady_fraction = 0.70;
minimum_analysis_cycles = 20;
maximum_analysis_cycles = 40;
samples_per_cycle = 2000;

root = fileparts(mfilename('fullpath'));
model = 'PGS_2023A_SINGLE_INV_RLOAD_2022b';
model_file = fullfile(root, [model '.slx']);
exe = fullfile(root, 'x64', 'Debug', ...
    'Motor_Control_Suite_SIL_Env.exe');
header_file = fullfile(root, 'sdpe_mgr', ...
    'sdpe_pgs_sinv_rc_simulate_settings.h');
matlab_init_file = fullfile(root, 'sdpe_mgr', ...
    'sdpe_pgs_sinv_rc_simulate_settings_matlab_init.m');

try
    if ~isfile(model_file)
        error('SINV:2023A:ModelMissing', ...
            'The fixed 2023A voltage-loop model does not exist: %s', ...
            model_file);
    end
    if ~isfile(exe)
        error('SINV:2023A:SILExecutableMissing', ...
            'The Debug/x64 SIL controller does not exist: %s', exe);
    end

    config = assert_generated_configuration(header_file, matlab_init_file, ...
        expected_build_level, expected_app_mode);
    if abs(config.SINV_2023A_UO_REF_RMS_V - uo_target_rms_v) > 1e-12
        error('SINV:2023A:VoltageTargetMismatch', ...
            ['Generated SINV_2023A_UO_REF_RMS_V must be %.6g V, ' ...
             'but it is %.17g V.'], ...
            uo_target_rms_v, config.SINV_2023A_UO_REF_RMS_V);
    end

    analysis_start_floor = max(steady_fraction * stop_time, ...
        config.SINV_2023A_SOFTSTART_TIME_S + 0.20);
    minimum_end_time = analysis_start_floor + ...
        minimum_analysis_cycles / nominal_frequency_hz;
    if stop_time < minimum_end_time
        error('SINV:2023A:StopTimeTooShort', ...
            ['Stop time %.6g s leaves fewer than %d complete steady ' ...
             'cycles after soft start and startup exclusion.'], ...
            stop_time, minimum_analysis_cycles);
    end

    if bdIsLoaded(model)
        error('SINV:2023A:ModelAlreadyLoaded', ...
            ['Close the fixed validation model before running this function. ' ...
             'No loaded model will be modified or closed automatically: %s'], ...
            model_file);
    end

    model_info_before = dir(model_file);
    load_system(model_file);
    model_cleanup = onCleanup(@() close_model_without_saving(model)); %#ok<NASGU>

    assert_physical_logger(model, '2023A Uo Physical Logger', 'uo_phys');
    assert_physical_logger(model, '2023A IG Io Physical Logger', 'io_phys');
    assert_physical_logger(model, '2023A IL Physical Logger', 'il_phys');

    % Monitor ports follow xplt.ctl_interface.h and the established model
    % bus-selector mapping. These loggers exist only in memory.
    monitor_spec = {
        'Bus Selector3', 2, 'iref'
        'Bus Selector8', 1, 'cia402_state'
        'Bus Selector9', 1, 'active_errors'
        'Bus Selector9', 2, 'diverge_fault_value'
        'From',          1, 'output_enable'
        };
    for k = 1:size(monitor_spec, 1)
        add_monitor_logger(model, monitor_spec{k, 1}, monitor_spec{k, 2}, ...
            monitor_spec{k, 3}, k);
    end

    fprintf('\n2023A standalone voltage-loop validation\n');
    fprintf('  Model: %s\n', model_file);
    fprintf('  Controller: %s\n', exe);
    fprintf('  Generated BUILD_LEVEL/SINV_APP_MODE: %d/%d\n', ...
        config.BUILD_LEVEL, config.SINV_APP_MODE);
    fprintf('  Stop time: %.6g s\n', stop_time);

    controller = start_controller(exe, root);
    controller_cleanup = onCleanup( ...
        @() stop_controller_quietly(controller)); %#ok<NASGU>
    pause(0.25);
    if controller.HasExited
        error('SINV:2023A:ControllerExitedEarly', ...
            'The Debug/x64 SIL controller exited before simulation started.');
    end

    out = sim(model, 'StopTime', num2str(stop_time, 17), ...
        'ReturnWorkspaceOutputs', 'on');

    % Stop the exact process launched above before analysis or file output.
    stop_controller(controller);

    uo = require_scalar_series(out, 'uo_phys');
    io = require_scalar_series(out, 'io_phys');
    il = require_scalar_series(out, 'il_phys');
    iref = require_scalar_series(out, 'iref');
    output_enable = require_scalar_series(out, 'output_enable');
    cia402_state = require_scalar_series(out, 'cia402_state');
    active_errors = require_scalar_series(out, 'active_errors');
    diverge_fault = require_scalar_series(out, 'diverge_fault_value');

    waveform_series = {uo, io, il, iref};
    analysis_start_floor = max(analysis_start_floor, ...
        max(cellfun(@(s) s.time(1), waveform_series)));
    analysis_end = min(cellfun(@(s) s.time(end), waveform_series));
    output_frequency_hz = estimate_frequency(uo, analysis_start_floor, ...
        analysis_end, nominal_frequency_hz);

    available_cycles = floor((analysis_end - analysis_start_floor) * ...
        output_frequency_hz);
    analysis_cycles = min(maximum_analysis_cycles, available_cycles);
    if analysis_cycles < minimum_analysis_cycles
        error('SINV:2023A:SteadyWindowTooShort', ...
            ['Only %d complete steady cycles are available after %.6g s; ' ...
             'at least %d are required.'], ...
            analysis_cycles, analysis_start_floor, minimum_analysis_cycles);
    end

    analysis_start = analysis_end - analysis_cycles / output_frequency_hz;
    analysis_fs_hz = samples_per_cycle * output_frequency_hz;
    sample_count = analysis_cycles * samples_per_cycle;
    analysis_time = analysis_start + (0:sample_count-1)' / analysis_fs_hz;

    uo_data = interpolate_series(uo, analysis_time, 'linear');
    io_data = interpolate_series(io, analysis_time, 'linear');
    il_data = interpolate_series(il, analysis_time, 'linear');
    iref_data = interpolate_series(iref, analysis_time, 'linear');

    uo_rms_v = waveform_rms(uo_data);
    io_rms_a = waveform_rms(io_data);
    il_rms_a = waveform_rms(il_data);
    current_ref_peak_a = max(abs(iref_data));
    current_ref_rms_a = waveform_rms(iref_data);
    [uo_thd_percent, harmonic_orders, uo_harmonic_peak] = ...
        harmonic_thd(uo_data, analysis_cycles, samples_per_cycle, 40);

    current_ref_limit_peak_a = min( ...
        config.SINV_2023A_CURRENT_REF_LIMIT_PEAK_PU, ...
        config.SINV_2023A_VOLTAGE_LOOP_OUTPUT_LIMIT_PU) * ...
        config.CTRL_CURRENT_BASE;
    [current_ref_saturation_fraction, saturation_duration_s] = ...
        saturation_metrics(iref_data, 1 / analysis_fs_hz, ...
        current_ref_limit_peak_a);

    enable_data = window_values(output_enable, analysis_start, analysis_end);
    state_data = window_values(cia402_state, analysis_start, analysis_end);
    error_data = window_values(active_errors, analysis_start, analysis_end);
    divergence_data = window_values(diverge_fault, analysis_start, analysis_end);

    output_enable_final = double(output_enable.data(end));
    cia402_state_final = double(cia402_state.data(end));
    active_errors_final = double(active_errors.data(end));
    divergence_final = double(diverge_fault.data(end));
    expected_io_rms_a = config.SINV_2023A_UO_REF_RMS_V / ...
        config.SINV_RLOAD_OHM;

    checks = struct( ...
        'uo_rms_within_24_v_plus_minus_0p2_v', ...
            abs(uo_rms_v - uo_target_rms_v) <= uo_tolerance_v, ...
        'frequency_within_50_hz_plus_minus_0p2_hz', ...
            abs(output_frequency_hz - nominal_frequency_hz) <= ...
                frequency_tolerance_hz, ...
        'io_rms_is_about_2_a', ...
            abs(io_rms_a - expected_io_rms_a) <= io_tolerance_a, ...
        'uo_thd_is_at_most_2_percent', ...
            uo_thd_percent <= maximum_uo_thd_percent, ...
        'output_enable_is_one_throughout_steady_window', ...
            all(enable_data == 1), ...
        'cia402_state_is_four_throughout_steady_window', ...
            all(state_data == 4), ...
        'active_errors_are_zero_throughout_steady_window', ...
            all(error_data == 0), ...
        'divergence_is_zero_throughout_steady_window', ...
            all(divergence_data == 0), ...
        'current_reference_is_not_persistently_limited', ...
            current_ref_saturation_fraction <= ...
                maximum_saturation_fraction && ...
            saturation_duration_s <= maximum_saturation_duration_s);
    passed = all(structfun(@(value) logical(value), checks));

    metrics = struct( ...
        'model', model, ...
        'build_level', config.BUILD_LEVEL, ...
        'sinv_app_mode', config.SINV_APP_MODE, ...
        'stop_time_s', stop_time, ...
        'steady_start_s', analysis_start, ...
        'steady_end_s', analysis_end, ...
        'steady_cycle_count', analysis_cycles, ...
        'uo_phys_rms_v', uo_rms_v, ...
        'io_phys_rms_a', io_rms_a, ...
        'il_phys_rms_a', il_rms_a, ...
        'output_frequency_hz', output_frequency_hz, ...
        'uo_phys_thd_percent', uo_thd_percent, ...
        'current_ref_peak_a', current_ref_peak_a, ...
        'current_ref_rms_a', current_ref_rms_a, ...
        'current_ref_limit_peak_a', current_ref_limit_peak_a, ...
        'current_ref_saturation_fraction', ...
            current_ref_saturation_fraction, ...
        'current_ref_max_saturation_duration_s', saturation_duration_s, ...
        'output_enable_final', output_enable_final, ...
        'cia402_state_final', cia402_state_final, ...
        'active_errors_final', active_errors_final, ...
        'divergence_final', divergence_final, ...
        'pass_checks', checks, ...
        'passed', passed);

    % Discard temporary monitor blocks without saving the target model.
    close_system(model, 0);
    model_info_after = dir(model_file);
    if model_info_after.bytes ~= model_info_before.bytes || ...
            model_info_after.datenum ~= model_info_before.datenum
        error('SINV:2023A:ModelFileChanged', ...
            ['The fixed target model changed during validation. Results were ' ...
             'not written; inspect the model immediately: %s'], model_file);
    end

    result_dir = fullfile(root, 'validation');
    if ~isfolder(result_dir)
        mkdir(result_dir);
    end
    json_file = fullfile(result_dir, '2023a_voltage_metrics.json');
    png_file = fullfile(result_dir, '2023a_voltage_waveforms.png');
    write_waveform_figure(png_file, analysis_time, uo_data, io_data, ...
        il_data, iref_data, current_ref_limit_peak_a, ...
        output_frequency_hz, harmonic_orders, uo_harmonic_peak, ...
        uo_thd_percent);
    write_json(json_file, metrics);

    fprintf('\nSteady window: %.6f to %.6f s (%d cycles)\n', ...
        analysis_start, analysis_end, analysis_cycles);
    fprintf('  Uo_phys: %.6f Vrms, %.6f Hz, THD %.6f %%\n', ...
        uo_rms_v, output_frequency_hz, uo_thd_percent);
    fprintf('  Io_phys/IL_phys: %.6f / %.6f Arms\n', ...
        io_rms_a, il_rms_a);
    fprintf('  Iref peak/RMS/limit: %.6f / %.6f / %.6f A\n', ...
        current_ref_peak_a, current_ref_rms_a, current_ref_limit_peak_a);
    fprintf('  Iref saturation fraction/max duration: %.8f / %.8f s\n', ...
        current_ref_saturation_fraction, saturation_duration_s);
    fprintf('  Enable/state/errors/divergence: %g / %g / %g / %g\n', ...
        output_enable_final, cia402_state_final, active_errors_final, ...
        divergence_final);
    fprintf('  JSON: %s\n', json_file);
    fprintf('  PNG:  %s\n', png_file);

    if passed
        fprintf('2023A standalone voltage-loop validation: PASS\n');
    else
        failed = failed_check_names(checks);
        fprintf(2, '2023A standalone voltage-loop validation: FAIL\n');
        fprintf(2, '  Failed checks: %s\n', strjoin(failed, ', '));
        error('SINV:2023A:ValidationFailed', ...
            'Voltage-loop validation failed; see JSON and PNG outputs.');
    end
catch ME
    fprintf(2, '\n2023A voltage-loop validation stopped: %s\n', ME.message);
    for k = 1:numel(ME.stack)
        fprintf(2, '  at %s, line %d\n', ...
            ME.stack(k).name, ME.stack(k).line);
    end
    rethrow(ME);
end
end


function config = assert_generated_configuration( ...
        header_file, matlab_init_file, expected_build_level, expected_app_mode)
if ~isfile(header_file) || ~isfile(matlab_init_file)
    error('SINV:2023A:GeneratedSettingsMissing', ...
        'Both generated C and MATLAB settings files are required.');
end

names = {
    'BUILD_LEVEL'
    'SINV_APP_MODE'
    'SINV_APP_MODE_STANDARD_BL2'
    'SINV_APP_MODE_2023A_SINGLE'
    'SINV_2023A_UO_REF_RMS_V'
    'SINV_2023A_OUTPUT_FREQ_HZ'
    'SINV_2023A_SOFTSTART_TIME_S'
    'SINV_2023A_VOLTAGE_LOOP_KP'
    'SINV_2023A_VOLTAGE_LOOP_KR'
    'SINV_2023A_VOLTAGE_LOOP_WI_HZ'
    'SINV_2023A_CURRENT_REF_LIMIT_PEAK_PU'
    'SINV_2023A_VOLTAGE_LOOP_OUTPUT_LIMIT_PU'
    'CTRL_CURRENT_BASE'
    'SINV_RLOAD_OHM'
    };

header_text = fileread(header_file);
matlab_text = fileread(matlab_init_file);
config = struct;
for k = 1:numel(names)
    name = names{k};
    c_value = read_c_numeric_macro(header_text, name);
    matlab_value = read_matlab_numeric_assignment(matlab_text, name);
    tolerance = 1e-12 * max(1, max(abs([c_value, matlab_value])));
    if abs(c_value - matlab_value) > tolerance
        error('SINV:2023A:GeneratedSettingsDisagree', ...
            'Generated C and MATLAB values disagree for %s.', name);
    end
    config.(name) = c_value;
end

if config.BUILD_LEVEL ~= expected_build_level || ...
        config.SINV_APP_MODE ~= expected_app_mode || ...
        config.SINV_APP_MODE_STANDARD_BL2 ~= 0 || ...
        config.SINV_APP_MODE_2023A_SINGLE ~= expected_app_mode
    error('SINV:2023A:GeneratedModeMismatch', ...
        ['Generated settings must select BUILD_LEVEL=%d and ' ...
         'SINV_APP_MODE=%d with mode constants 0/1.'], ...
        expected_build_level, expected_app_mode);
end
end


function value = read_c_numeric_macro(text, name)
number = '([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)';
expression = ['(?m)^\s*#define\s+' name '\s+\(\s*' ...
    number '[fFlL]?\s*\)\s*$'];
tokens = regexp(text, expression, 'tokens');
if numel(tokens) ~= 1
    error('SINV:2023A:GeneratedMacroMissingOrDuplicate', ...
        'Expected exactly one generated C macro named %s.', name);
end
value = str2double(tokens{1}{1});
if ~isfinite(value)
    error('SINV:2023A:GeneratedMacroInvalid', ...
        'Generated C macro %s is not finite.', name);
end
end


function value = read_matlab_numeric_assignment(text, name)
number = '([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)';
expression = ['(?m)^\s*' name '\s*=\s*' number '\s*;\s*$'];
tokens = regexp(text, expression, 'tokens');
if numel(tokens) ~= 1
    error('SINV:2023A:GeneratedVariableMissingOrDuplicate', ...
        'Expected exactly one generated MATLAB variable named %s.', name);
end
value = str2double(tokens{1}{1});
if ~isfinite(value)
    error('SINV:2023A:GeneratedVariableInvalid', ...
        'Generated MATLAB variable %s is not finite.', name);
end
end


function assert_physical_logger(model, block_name, variable_name)
blocks = find_system(model, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'Type', 'Block', ...
    'Name', block_name);
if numel(blocks) ~= 1
    error('SINV:2023A:PhysicalLoggerMissing', ...
        'Expected one physical logger named %s, found %d.', ...
        block_name, numel(blocks));
end
if ~strcmp(get_param(blocks{1}, 'BlockType'), 'ToWorkspace') || ...
        ~strcmp(get_param(blocks{1}, 'VariableName'), variable_name) || ...
        ~strcmpi(get_param(blocks{1}, 'SaveFormat'), 'Timeseries')
    error('SINV:2023A:PhysicalLoggerInvalid', ...
        ['Logger "%s" must be a To Workspace block writing %s as a ' ...
         'timeseries.'], block_name, variable_name);
end
end


function add_monitor_logger(model, source_name, source_port, variable, index)
source = [model '/' source_name];
if getSimulinkBlockHandle(source) == -1
    error('SINV:2023A:MonitorSourceMissing', ...
        'Required controller monitor source is missing: %s', source);
end

block = sprintf('%s/2023A Voltage Validation Logger %02d', model, index);
if getSimulinkBlockHandle(block) ~= -1
    error('SINV:2023A:TemporaryLoggerExists', ...
        'Temporary logger name is already present: %s', block);
end

source_handles = get_param(source, 'PortHandles');
if numel(source_handles.Outport) < source_port
    error('SINV:2023A:MonitorPortMissing', ...
        'Monitor %s does not have output port %d.', source, source_port);
end

position = [1120, 30 + index * 28, 1300, 50 + index * 28];
add_block('simulink/Sinks/To Workspace', block, ...
    'VariableName', variable, ...
    'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', '2000000', ...
    'Decimation', '1', ...
    'Position', position);
logger_handles = get_param(block, 'PortHandles');
add_line(model, source_handles.Outport(source_port), ...
    logger_handles.Inport(1), 'autorouting', 'on');
end


function process = start_controller(exe, root)
info = System.Diagnostics.ProcessStartInfo;
info.FileName = exe;
info.WorkingDirectory = root;
info.UseShellExecute = false;
info.CreateNoWindow = true;
process = System.Diagnostics.Process.Start(info);
if isempty(process)
    error('SINV:2023A:ControllerStartFailed', ...
        'Could not start the Debug/x64 SIL controller: %s', exe);
end
end


function stop_controller(process)
if isempty(process)
    return;
end
if ~process.HasExited
    process.Kill;
end
if ~process.WaitForExit(5000)
    error('SINV:2023A:ControllerStopTimeout', ...
        'The SIL controller did not exit within 5 seconds.');
end
end


function stop_controller_quietly(process)
try
    stop_controller(process);
catch ME
    warning('SINV:2023A:ControllerCleanupFailed', ...
        'Could not confirm controller shutdown: %s', ME.message);
end
end


function close_model_without_saving(model)
try
    if bdIsLoaded(model)
        close_system(model, 0);
    end
catch ME
    warning('SINV:2023A:ModelCleanupFailed', ...
        'Could not close the unsaved validation model: %s', ME.message);
end
end


function series = require_scalar_series(out, name)
try
    raw = out.get(name);
catch ME
    error('SINV:2023A:SignalMissing', ...
        'SimulationOutput does not contain %s: %s', name, ME.message);
end
if isempty(raw)
    error('SINV:2023A:SignalEmpty', ...
        'SimulationOutput signal %s is empty.', name);
end

if isa(raw, 'Simulink.SimulationData.Signal')
    raw = raw.Values;
end
if isa(raw, 'timeseries')
    time = double(raw.Time(:));
    data = double(raw.Data);
elseif istimetable(raw)
    time = seconds(rowtimes(raw) - rowtimes(raw(1, :)));
    data = double(raw.Variables);
elseif isstruct(raw) && isfield(raw, 'time') && isfield(raw, 'signals')
    time = double(raw.time(:));
    data = double(raw.signals.values);
elseif isstruct(raw) && isfield(raw, 'Time') && isfield(raw, 'Data')
    time = double(raw.Time(:));
    data = double(raw.Data);
else
    error('SINV:2023A:SignalFormatUnsupported', ...
        'Signal %s has unsupported class %s.', name, class(raw));
end

if isempty(time) || isempty(data) || numel(data) ~= numel(time)
    error('SINV:2023A:SignalNotScalar', ...
        'Signal %s must contain one scalar value per time sample.', name);
end
data = data(:);
if any(~isfinite(time)) || any(~isfinite(data))
    error('SINV:2023A:SignalNonFinite', ...
        'Signal %s contains NaN or Inf.', name);
end

[time, order] = sort(time);
data = data(order);
[time, unique_index] = unique(time, 'last');
data = data(unique_index);
if numel(time) < 2 || time(end) <= time(1)
    error('SINV:2023A:SignalTimeInvalid', ...
        'Signal %s does not contain a usable time interval.', name);
end
series = struct('name', name, 'time', time, 'data', data);
end


function frequency_hz = estimate_frequency(series, start_time, end_time, nominal)
if end_time <= start_time
    error('SINV:2023A:FrequencyWindowInvalid', ...
        'The Uo_phys frequency-estimation window is empty.');
end
sample_rate_hz = 20000;
sample_count = floor((end_time - start_time) * sample_rate_hz) + 1;
query_time = start_time + (0:sample_count-1)' / sample_rate_hz;
data = interpolate_series(series, query_time, 'linear');
data = data - median(data);
data = movmean(data, max(3, round(sample_rate_hz / 1000)));

crossing_index = find(data(1:end-1) <= 0 & data(2:end) > 0);
if numel(crossing_index) < 3
    error('SINV:2023A:FrequencyEstimateFailed', ...
        'Uo_phys does not contain enough rising zero crossings.');
end
y1 = data(crossing_index);
y2 = data(crossing_index + 1);
t1 = query_time(crossing_index);
crossing_time = t1 - y1 .* (1 / sample_rate_hz) ./ (y2 - y1);
period = diff(crossing_time);
period = period(period >= 1 / (nominal + 10) & ...
    period <= 1 / (nominal - 10));
if isempty(period)
    error('SINV:2023A:FrequencyEstimateFailed', ...
        'Uo_phys has no valid 40-60 Hz periods.');
end
frequency_hz = 1 / median(period);
end


function data = interpolate_series(series, query_time, method)
tolerance = 100 * eps(max(abs([series.time(1), series.time(end), ...
    query_time(1), query_time(end)])));
if query_time(1) < series.time(1) - tolerance || ...
        query_time(end) > series.time(end) + tolerance
    error('SINV:2023A:SignalWindowUnavailable', ...
        'Signal %s does not cover the requested steady window.', series.name);
end
data = interp1(series.time, series.data, query_time, method);
if any(~isfinite(data))
    error('SINV:2023A:InterpolationFailed', ...
        'Signal %s produced NaN or Inf during steady resampling.', ...
        series.name);
end
end


function data = window_values(series, start_time, end_time)
index = series.time >= start_time & series.time <= end_time;
data = series.data(index);
if isempty(data)
    error('SINV:2023A:StatusWindowUnavailable', ...
        'Signal %s has no samples in the steady window.', series.name);
end
end


function value = waveform_rms(data)
value = sqrt(mean(double(data(:)).^2));
end


function [thd_percent, orders, peak_amplitude] = ...
        harmonic_thd(data, cycle_count, samples_per_cycle, maximum_harmonic)
data = double(data(:)) - mean(double(data(:)));
sample_count = numel(data);
if sample_count ~= cycle_count * samples_per_cycle
    error('SINV:2023A:HarmonicWindowInvalid', ...
        'Harmonic window does not contain an integer number of cycles.');
end

maximum_harmonic = min(maximum_harmonic, ...
    floor((samples_per_cycle - 1) / 2));
orders = (1:maximum_harmonic)';
sample_index = (0:sample_count-1)';
peak_amplitude = zeros(size(orders));
for k = 1:numel(orders)
    basis = exp(-1i * 2 * pi * orders(k) * cycle_count * ...
        sample_index / sample_count);
    peak_amplitude(k) = 2 * abs(sum(data .* basis)) / sample_count;
end
if peak_amplitude(1) <= eps(max(abs(data)))
    error('SINV:2023A:FundamentalMissing', ...
        'Uo_phys does not contain a usable fundamental.');
end
thd_percent = 100 * sqrt(sum(peak_amplitude(2:end).^2)) / ...
    peak_amplitude(1);
end


function [fraction, maximum_duration_s] = ...
        saturation_metrics(data, sample_period_s, limit_peak_a)
if ~isfinite(limit_peak_a) || limit_peak_a <= 0
    error('SINV:2023A:CurrentLimitInvalid', ...
        'The generated current-reference limit must be positive and finite.');
end
tolerance = max(1e-6, 1e-3 * limit_peak_a);
saturated = abs(double(data(:))) >= limit_peak_a - tolerance;
fraction = mean(saturated);
edges = diff([false; saturated; false]);
run_starts = find(edges == 1);
run_ends = find(edges == -1) - 1;
if isempty(run_starts)
    maximum_duration_s = 0.0;
else
    maximum_duration_s = max(run_ends - run_starts + 1) * sample_period_s;
end
end


function write_waveform_figure(filename, time, uo, io, il, iref, ...
        iref_limit, frequency_hz, harmonic_orders, harmonic_peak, ...
        uo_thd_percent)
fig = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1400, 1000]);
figure_cleanup = onCleanup(@() close_figure(fig)); %#ok<NASGU>
layout = tiledlayout(fig, 3, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
plot_stride = max(1, floor(numel(time) / 12000));
plot_index = 1:plot_stride:numel(time);

nexttile(layout, [1, 2]);
plot(time(plot_index), uo(plot_index), 'LineWidth', 0.9);
grid on;
xlabel('Time (s)'); ylabel('Uo_{phys} (V)');
title(sprintf('RL-terminal voltage, %.4f Hz', frequency_hz));

nexttile(layout);
plot(time(plot_index), io(plot_index), ...
    time(plot_index), il(plot_index), 'LineWidth', 0.9);
grid on;
xlabel('Time (s)'); ylabel('Current (A)');
legend('Io_{phys}', 'IL_{phys}', 'Location', 'best');
title('Physical load and filter-inductor currents');

nexttile(layout);
plot(time(plot_index), iref(plot_index), 'LineWidth', 0.9);
hold on;
yline(iref_limit, '--r');
yline(-iref_limit, '--r');
hold off;
grid on;
xlabel('Time (s)'); ylabel('Iref (A)');
title('Controller current reference and generated limits');

nexttile(layout, [1, 2]);
harmonic_percent = 100 * harmonic_peak / harmonic_peak(1);
bar(harmonic_orders, max(harmonic_percent, 1e-6), 0.8);
set(gca, 'YScale', 'log');
grid on;
xlabel('Harmonic order');
ylabel('Peak amplitude (% of fundamental)');
title(sprintf('Uo_{phys} harmonics, THD = %.4f %%', ...
    uo_thd_percent));
xlim([0.5, harmonic_orders(end) + 0.5]);
ylim([1e-4, max(200, 2 * max(harmonic_percent))]);

title(layout, '2023A standalone voltage-loop steady-state validation');
exportgraphics(fig, filename, 'Resolution', 160);
end


function close_figure(fig)
try
    if isgraphics(fig)
        close(fig);
    end
catch
end
end


function write_json(filename, metrics)
fid = fopen(filename, 'w');
if fid < 0
    error('SINV:2023A:JSONOpenFailed', ...
        'Could not create metrics file: %s', filename);
end
file_cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', jsonencode(metrics, PrettyPrint=true));
end


function names = failed_check_names(checks)
fields = fieldnames(checks);
failed = ~structfun(@(value) logical(value), checks);
names = fields(failed);
end

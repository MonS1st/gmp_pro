function metrics = run_2023a_measurement_validation(stop_time)
%RUN_2023A_MEASUREMENT_VALIDATION Validate the 2023A physical log channels.
%
% This validation is intentionally fixed to the measurement-only R2022b
% model and the existing BUILD_LEVEL 2 SIL executable. It does not resolve
% to a standard model, save a Simulink model, generate SDPE, or build code.

arguments
    stop_time (1,1) double {mustBeFinite, mustBePositive} = 2.0
end

build_level = 2;
nominal_frequency_hz = 50.0;
frequency_tolerance_hz = 1.0;
steady_fraction = 0.60;
minimum_analysis_cycles = 10;
maximum_analysis_cycles = 40;
samples_per_cycle = 4000;

root = fileparts(mfilename('fullpath'));
model = 'PGS_2023A_SINGLE_INV_RLOAD_2022b';
model_file = fullfile(root, [model '.slx']);
exe = fullfile(root, 'x64', 'Debug', ...
    'Motor_Control_Suite_SIL_Env.exe');
header_file = fullfile(root, 'sdpe_mgr', ...
    'sdpe_pgs_sinv_rc_simulate_settings.h');
matlab_init_file = fullfile(root, 'sdpe_mgr', ...
    'sdpe_pgs_sinv_rc_simulate_settings_matlab_init.m');

if ~isfile(model_file)
    error('SINV:2023A:ModelMissing', ...
        'The fixed 2023A measurement model does not exist: %s', model_file);
end
if ~isfile(exe)
    error('SINV:2023A:SILExecutableMissing', ...
        'The existing BL2 SIL controller does not exist: %s', exe);
end
assert_generated_build_level(header_file, matlab_init_file, build_level);

if stop_time * nominal_frequency_hz < minimum_analysis_cycles / ...
        (1.0 - steady_fraction)
    error('SINV:2023A:StopTimeTooShort', ...
        ['Stop time %.6g s is too short for a %.0f%% startup exclusion ' ...
         'and at least %d steady 50 Hz cycles.'], ...
        stop_time, 100 * steady_fraction, minimum_analysis_cycles);
end

if bdIsLoaded(model) && strcmp(get_param(model, 'Dirty'), 'on')
    error('SINV:2023A:ModelDirty', ...
        ['The target model is already loaded with unsaved changes. ' ...
         'Validation stopped without changing or closing it: %s'], model_file);
end

model_info_before = dir(model_file);
load_system(model_file);
model_cleanup = onCleanup(@() close_model_without_saving(model)); %#ok<NASGU>

assert_physical_logger(model, '2023A Uo Physical Logger', 'uo_phys');
assert_physical_logger(model, '2023A IG Io Physical Logger', 'io_phys');
assert_physical_logger(model, '2023A IL Physical Logger', 'il_phys');

% Reuse the controller-monitor mapping from run_sinv_validation.m. These
% logger blocks exist only in the loaded diagram and are never saved.
monitor_spec = {
    'Bus Selector2', 1, 'vac'
    'Bus Selector2', 2, 'iac'
    'Bus Selector3', 1, 'vbus'
    'Bus Selector3', 2, 'iref'
    'Bus Selector4', 2, 'pll_hz'
    'Bus Selector8', 1, 'cia402_state'
    'Bus Selector9', 1, 'active_errors'
    'Bus Selector9', 2, 'diverge_fault_value'
    'From',          1, 'output_enable'
    };
for k = 1:size(monitor_spec, 1)
    add_monitor_logger(model, monitor_spec{k, 1}, monitor_spec{k, 2}, ...
        monitor_spec{k, 3}, k);
end

plant = [model ...
    '/Single Phase DC//AC Full Bridge Inverter, LC Filter (GMP STD MDL)'];
ig_shunt_resistance_ohm = resolve_mask_scalar(plant, ...
    'GRID_CURRENT_SENSOR_R');
adc_bits = resolve_mask_scalar(plant, 'ADC_Resolution');
adc_reference_v = resolve_mask_scalar(plant, ...
    'AC_VOLTAGE_ADC_REFERENCE');
voltage_sensitivity_v_per_v = resolve_mask_scalar(plant, ...
    'AC_Voltage_Gain');
current_sensitivity_v_per_a = resolve_mask_scalar(plant, ...
    'AC_Current_Gain');
voltage_lsb_v = adc_reference_v / ((2^adc_bits - 1) * ...
    voltage_sensitivity_v_per_v);
current_lsb_a = adc_reference_v / ((2^adc_bits - 1) * ...
    current_sensitivity_v_per_a);

fprintf('\n2023A measurement baseline validation\n');
fprintf('  Model: %s\n', model_file);
fprintf('  Controller: %s\n', exe);
fprintf('  Generated BUILD_LEVEL: %d\n', build_level);
fprintf('  Stop time: %.6g s\n', stop_time);

controller = start_controller(exe, root);
controller_cleanup = onCleanup(@() stop_controller_quietly(controller)); %#ok<NASGU>
pause(0.25);
if controller.HasExited
    error('SINV:2023A:ControllerExitedEarly', ...
        'The BL2 SIL controller exited before simulation started.');
end

out = sim(model, 'StopTime', num2str(stop_time, 17), ...
    'ReturnWorkspaceOutputs', 'on');

% Stop the exact process started above before doing analysis or file I/O.
stop_controller(controller);

uo = require_scalar_series(out, 'uo_phys');
io = require_scalar_series(out, 'io_phys');
il = require_scalar_series(out, 'il_phys');
vac = require_scalar_series(out, 'vac');
iac = require_scalar_series(out, 'iac');
iref = require_scalar_series(out, 'iref');
vbus = require_scalar_series(out, 'vbus');
pll_hz = require_scalar_series(out, 'pll_hz');
output_enable = require_scalar_series(out, 'output_enable');
cia402_state = require_scalar_series(out, 'cia402_state');
active_errors = require_scalar_series(out, 'active_errors');
diverge_fault = require_scalar_series(out, 'diverge_fault_value');

waveform_series = {uo, io, il, vac, iac, iref, vbus, pll_hz};
analysis_start_floor = steady_fraction * stop_time;
analysis_start_floor = max(analysis_start_floor, ...
    max(cellfun(@(s) s.time(1), waveform_series)));
analysis_end = min(cellfun(@(s) s.time(end), waveform_series));
uo_frequency_hz = estimate_frequency(uo, analysis_start_floor, ...
    analysis_end, nominal_frequency_hz);

available_cycles = floor((analysis_end - analysis_start_floor) * ...
    uo_frequency_hz);
analysis_cycles = min(maximum_analysis_cycles, available_cycles);
if analysis_cycles < minimum_analysis_cycles
    error('SINV:2023A:SteadyWindowTooShort', ...
        ['Only %d complete steady cycles are available after %.6g s; ' ...
         'at least %d are required.'], ...
        analysis_cycles, analysis_start_floor, minimum_analysis_cycles);
end
analysis_start = analysis_end - analysis_cycles / uo_frequency_hz;
analysis_fs_hz = samples_per_cycle * uo_frequency_hz;
sample_count = analysis_cycles * samples_per_cycle;
analysis_time = analysis_start + (0:sample_count-1)' / analysis_fs_hz;

uo_data = interpolate_series(uo, analysis_time);
io_data = interpolate_series(io, analysis_time);
il_data = interpolate_series(il, analysis_time);
vac_data = interpolate_series(vac, analysis_time);
iac_data = interpolate_series(iac, analysis_time);
iref_data = interpolate_series(iref, analysis_time);
vbus_data = interpolate_series(vbus, analysis_time);
pll_data = interpolate_series(pll_hz, analysis_time);

uo_rms_v = waveform_rms(uo_data);
io_rms_a = waveform_rms(io_data);
il_rms_a = waveform_rms(il_data);
vac_rms_v = waveform_rms(vac_data);
iac_rms_a = waveform_rms(iac_data);
iref_rms_a = waveform_rms(iref_data);
vbus_mean_v = mean(vbus_data);
pll_frequency_hz = mean(pll_data);

% These harmonic calculations use the unfiltered, unquantized physical
% uo_phys waveform. Vac/Iac are not substituted for the required signals.
[uo_thd_percent, harmonic_orders, uo_harmonic_peak] = ...
    harmonic_thd(uo_data, analysis_cycles, samples_per_cycle, 40);
[io_thd_percent, ~, ~] = ...
    harmonic_thd(io_data, analysis_cycles, samples_per_cycle, 40);
[il_thd_percent, ~, ~] = ...
    harmonic_thd(il_data, analysis_cycles, samples_per_cycle, 40);

output_enable_final = final_value(output_enable);
cia402_state_final = final_value(cia402_state);
active_errors_final = final_value(active_errors);
diverge_fault_value = final_value(diverge_fault);

vac_minus_uo_rms_v = vac_rms_v - uo_rms_v;
iac_minus_il_rms_a = iac_rms_a - il_rms_a;
il_minus_io_rms_a = il_rms_a - io_rms_a;

% Derive the expected Vac/Uo separation from the actual model shunt and
% measured load current. The expected numerical result is not hard-coded.
expected_ig_shunt_drop_rms_v = ...
    io_rms_a * ig_shunt_resistance_ohm;
vac_uo_tolerance_v = max(1.5 * voltage_lsb_v, ...
    0.35 * expected_ig_shunt_drop_rms_v);
minimum_detectable_drop_v = max(10 * eps(max(vac_rms_v, uo_rms_v)), ...
    0.10 * expected_ig_shunt_drop_rms_v);
vac_uo_shunt_relation_ok = ...
    abs(vac_minus_uo_rms_v) > minimum_detectable_drop_v && ...
    abs(abs(vac_minus_uo_rms_v) - expected_ig_shunt_drop_rms_v) <= ...
        vac_uo_tolerance_v;

iac_il_tolerance_a = max(5 * current_lsb_a, 0.05 * il_rms_a);
checks = struct( ...
    'output_enable_is_one', output_enable_final == 1, ...
    'cia402_state_is_operation_enabled', cia402_state_final == 4, ...
    'active_errors_are_zero', active_errors_final == 0, ...
    'divergence_fault_is_zero', diverge_fault_value == 0, ...
    'uo_phys_exists_nonempty_finite', true, ...
    'io_phys_exists_nonempty_finite', true, ...
    'il_phys_exists_nonempty_finite', true, ...
    'uo_phys_rms_is_positive', uo_rms_v > 0, ...
    'io_phys_rms_is_positive', io_rms_a > 0, ...
    'il_phys_rms_is_positive', il_rms_a > 0, ...
    'uo_phys_frequency_is_near_50_hz', ...
        abs(uo_frequency_hz - nominal_frequency_hz) <= ...
            frequency_tolerance_hz, ...
    'iac_rms_matches_il_phys_rms', ...
        abs(iac_minus_il_rms_a) <= iac_il_tolerance_a, ...
    'vac_uo_difference_matches_ig_shunt', vac_uo_shunt_relation_ok);
passed = all(structfun(@(value) logical(value), checks));

metrics = struct( ...
    'build_level', build_level, ...
    'model', model, ...
    'stop_time_s', stop_time, ...
    'output_enable_final', output_enable_final, ...
    'cia402_state_final', cia402_state_final, ...
    'active_errors_final', active_errors_final, ...
    'diverge_fault_value', diverge_fault_value, ...
    'uo_phys_rms_v', uo_rms_v, ...
    'io_phys_rms_a', io_rms_a, ...
    'il_phys_rms_a', il_rms_a, ...
    'uo_phys_frequency_hz', uo_frequency_hz, ...
    'uo_phys_thd_percent', uo_thd_percent, ...
    'io_phys_thd_percent', io_thd_percent, ...
    'il_phys_thd_percent', il_thd_percent, ...
    'vac_rms_v', vac_rms_v, ...
    'iac_rms_a', iac_rms_a, ...
    'iref_rms_a', iref_rms_a, ...
    'vbus_mean_v', vbus_mean_v, ...
    'vac_minus_uo_rms_v', vac_minus_uo_rms_v, ...
    'iac_minus_il_rms_a', iac_minus_il_rms_a, ...
    'il_minus_io_rms_a', il_minus_io_rms_a, ...
    'pll_frequency_hz', pll_frequency_hz, ...
    'steady_start_s', analysis_start, ...
    'steady_end_s', analysis_end, ...
    'steady_cycle_count', analysis_cycles, ...
    'ig_shunt_resistance_ohm', ig_shunt_resistance_ohm, ...
    'expected_ig_shunt_drop_rms_v', expected_ig_shunt_drop_rms_v, ...
    'voltage_adc_lsb_equivalent_v', voltage_lsb_v, ...
    'current_adc_lsb_equivalent_a', current_lsb_a, ...
    'vac_uo_shunt_tolerance_v', vac_uo_tolerance_v, ...
    'iac_il_tolerance_a', iac_il_tolerance_a, ...
    'pass_checks', checks, ...
    'passed', passed);

% Discard all temporary monitor blocks without saving the model file.
close_system(model, 0);
model_info_after = dir(model_file);
if model_info_after.bytes ~= model_info_before.bytes || ...
        model_info_after.datenum ~= model_info_before.datenum
    error('SINV:2023A:ModelFileChanged', ...
        ['The target model file changed during validation. Results were not ' ...
         'written; inspect the model immediately: %s'], model_file);
end

result_dir = fullfile(root, 'validation');
if ~isfolder(result_dir)
    mkdir(result_dir);
end
json_file = fullfile(result_dir, ...
    '2023a_measurement_baseline_metrics.json');
png_file = fullfile(result_dir, ...
    '2023a_measurement_baseline_waveforms.png');

write_waveform_figure(png_file, analysis_time, uo_data, io_data, il_data, ...
    vac_data, iac_data, uo_frequency_hz, analysis_cycles, ...
    samples_per_cycle, harmonic_orders, uo_harmonic_peak, ...
    uo_thd_percent);
write_json(json_file, metrics);

fprintf('\nSteady window: %.6f to %.6f s (%d cycles)\n', ...
    analysis_start, analysis_end, analysis_cycles);
fprintf('  Uo_phys: %.6f Vrms, %.6f Hz, THD %.6f %%\n', ...
    uo_rms_v, uo_frequency_hz, uo_thd_percent);
fprintf('  Io_phys: %.6f Arms, THD %.6f %%\n', ...
    io_rms_a, io_thd_percent);
fprintf('  IL_phys: %.6f Arms, THD %.6f %%\n', ...
    il_rms_a, il_thd_percent);
fprintf('  Vac - Uo RMS: %.6f V (IG shunt estimate %.6f V)\n', ...
    vac_minus_uo_rms_v, expected_ig_shunt_drop_rms_v);
fprintf('  Iac - IL RMS: %.6f A; IL - Io RMS: %.6f A\n', ...
    iac_minus_il_rms_a, il_minus_io_rms_a);
fprintf('  Enable/state/errors/divergence: %g / %g / %g / %g\n', ...
    output_enable_final, cia402_state_final, active_errors_final, ...
    diverge_fault_value);
fprintf('  JSON: %s\n', json_file);
fprintf('  PNG:  %s\n', png_file);

if passed
    fprintf('2023A measurement baseline validation: PASS\n');
else
    failed = failed_check_names(checks);
    fprintf(2, '2023A measurement baseline validation: FAIL\n');
    fprintf(2, '  Failed checks: %s\n', strjoin(failed, ', '));
    error('SINV:2023A:MeasurementValidationFailed', ...
        'Measurement baseline validation failed; see JSON and PNG outputs.');
end
end


function assert_generated_build_level(header_file, matlab_init_file, expected)
if ~isfile(header_file) || ~isfile(matlab_init_file)
    error('SINV:2023A:GeneratedSettingsMissing', ...
        'Generated BL2 settings are missing under sdpe_mgr.');
end

header = fileread(header_file);
header_token = regexp(header, ...
    '#define\s+BUILD_LEVEL\s+\(\s*(\d+)\s*\)', 'tokens', 'once');
matlab_init = fileread(matlab_init_file);
matlab_token = regexp(matlab_init, ...
    '(?m)^\s*BUILD_LEVEL\s*=\s*(\d+)\s*;', 'tokens', 'once');

if isempty(header_token) || isempty(matlab_token) || ...
        str2double(header_token{1}) ~= expected || ...
        str2double(matlab_token{1}) ~= expected
    error('SINV:2023A:BuildLevelMismatch', ...
        ['Both generated C and MATLAB settings must select BUILD_LEVEL=%d. ' ...
         'Do not regenerate or rebuild for this validation.'], expected);
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
        'Expected exactly one "%s" block, found %d. Model was not changed.', ...
        block_name, numel(blocks));
end
if ~strcmp(get_param(blocks{1}, 'BlockType'), 'ToWorkspace') || ...
        ~strcmp(get_param(blocks{1}, 'VariableName'), variable_name) || ...
        ~strcmpi(get_param(blocks{1}, 'SaveFormat'), 'Timeseries')
    error('SINV:2023A:PhysicalLoggerInvalid', ...
        ['Logger "%s" must be a To Workspace block writing %s as a ' ...
         'timeseries. Model was not changed.'], block_name, variable_name);
end
end


function add_monitor_logger(model, source_name, source_port, variable, index)
source = [model '/' source_name];
if getSimulinkBlockHandle(source) == -1
    error('SINV:2023A:MonitorSourceMissing', ...
        'Required controller monitor source is missing: %s', source);
end

block = sprintf('%s/2023A Measurement Validation Logger %02d', ...
    model, index);
if getSimulinkBlockHandle(block) ~= -1
    error('SINV:2023A:TemporaryLoggerExists', ...
        'Temporary logger name is already present: %s', block);
end

source_handles = get_param(source, 'PortHandles');
if numel(source_handles.Outport) < source_port
    error('SINV:2023A:MonitorPortMissing', ...
        'Monitor %s does not have output port %d.', source, source_port);
end

position = [1120, 20 + index * 26, 1300, 40 + index * 26];
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


function value = resolve_mask_scalar(block, parameter)
if getSimulinkBlockHandle(block) == -1
    error('SINV:2023A:PlantMissing', ...
        'Required plant block is missing: %s', block);
end
expression = get_param(block, parameter);
try
    value = double(slResolve(expression, block));
catch ME
    error('SINV:2023A:MaskParameterResolutionFailed', ...
        'Could not resolve %s=%s on %s: %s', ...
        parameter, expression, block, ME.message);
end
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('SINV:2023A:InvalidMaskParameter', ...
        '%s on %s must resolve to a positive finite scalar.', ...
        parameter, block);
end
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
        'Could not start the BL2 SIL controller: %s', exe);
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
        'The BL2 SIL controller did not exit within 5 seconds.');
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

if isempty(time) || isempty(data)
    error('SINV:2023A:SignalEmpty', 'Signal %s is empty.', name);
end
if numel(data) ~= numel(time)
    error('SINV:2023A:SignalNotScalar', ...
        'Signal %s must contain exactly one scalar value per time sample.', ...
        name);
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
data = interpolate_series(series, query_time);
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


function data = interpolate_series(series, query_time)
tolerance = 100 * eps(max(abs([series.time(1), series.time(end), ...
    query_time(1), query_time(end)])));
if query_time(1) < series.time(1) - tolerance || ...
        query_time(end) > series.time(end) + tolerance
    error('SINV:2023A:SignalWindowUnavailable', ...
        'Signal %s does not cover the requested steady window.', series.name);
end
data = interp1(series.time, series.data, query_time, 'linear');
if any(~isfinite(data))
    error('SINV:2023A:InterpolationFailed', ...
        'Signal %s produced NaN or Inf during steady resampling.', ...
        series.name);
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
        'The physical waveform does not contain a usable fundamental.');
end
thd_percent = 100 * sqrt(sum(peak_amplitude(2:end).^2)) / ...
    peak_amplitude(1);
end


function value = final_value(series)
value = double(series.data(end));
end


function write_waveform_figure(filename, time, uo, io, il, vac, iac, ...
        frequency_hz, cycle_count, samples_per_cycle, harmonic_orders, ...
        harmonic_peak, uo_thd_percent)
fig = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1400, 1000]);
figure_cleanup = onCleanup(@() close_figure(fig)); %#ok<NASGU>
layout = tiledlayout(fig, 3, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
plot_stride = max(1, floor(samples_per_cycle / 400));
plot_index = 1:plot_stride:numel(time);

nexttile(layout);
plot(time(plot_index), uo(plot_index), ...
    time(plot_index), vac(plot_index), 'LineWidth', 0.9);
grid on;
xlabel('Time (s)'); ylabel('Voltage (V)');
legend('Uo_{phys}', 'Vac', 'Location', 'best');
title('Physical load-terminal voltage vs controller Vac');

nexttile(layout);
plot(time(plot_index), io(plot_index), ...
    time(plot_index), il(plot_index), 'LineWidth', 0.9);
grid on;
xlabel('Time (s)'); ylabel('Current (A)');
legend('Io_{phys}', 'IL_{phys}', 'Location', 'best');
title('Physical load and filter-inductor currents');

nexttile(layout);
plot(time(plot_index), il(plot_index), ...
    time(plot_index), iac(plot_index), 'LineWidth', 0.9);
grid on;
xlabel('Time (s)'); ylabel('Current (A)');
legend('IL_{phys}', 'Iac', 'Location', 'best');
title('Physical inductor current vs controller Iac');

nexttile(layout);
local_cycle_count = min(3, cycle_count);
local_count = local_cycle_count * samples_per_cycle;
local_index = numel(time) - local_count + 1:numel(time);
plot(time(local_index), uo(local_index), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)'); ylabel('Uo_{phys} (V)');
title(sprintf('Last %d Uo_{phys} cycles at %.4f Hz', ...
    local_cycle_count, frequency_hz));

nexttile(layout, [1, 2]);
harmonic_percent = 100 * harmonic_peak / harmonic_peak(1);
bar(harmonic_orders, max(harmonic_percent, 1e-6), 0.8);
set(gca, 'YScale', 'log');
grid on;
xlabel('Harmonic order');
ylabel('Peak amplitude (% of fundamental)');
title(sprintf(['Uo_{phys} direct physical harmonics, ' ...
    'THD = %.4f %%'], uo_thd_percent));
xlim([0.5, harmonic_orders(end) + 0.5]);
ylim([1e-4, max(200, 2 * max(harmonic_percent))]);

title(layout, sprintf(['2023A measurement baseline, BL2, ' ...
    '%d-cycle steady window'], cycle_count));
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

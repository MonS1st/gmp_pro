function results = run_2023a_load_regulation_validation(stop_time)
%RUN_2023A_LOAD_REGULATION_VALIDATION Validate the 0-2 A load regulation.
%
% The sweep is intentionally fixed to PGS_2023A_SINGLE_INV_RLOAD_2022b and
% the existing Debug/x64 SIL executable. Every load point uses an isolated
% SimulationInput variable override and a separately launched controller
% process. The loaded model is never saved.

arguments
    stop_time (1,1) double {mustBeFinite, mustBePositive} = 3.0
end

expected_build_level = 2;
expected_app_mode = 1;
% MATLAB message-identifier fields must begin with a letter, so use R2023A.
target_voltage_rms_v = 24.0;
nominal_frequency_hz = 50.0;
target_current_a = [0, 0.5, 1.0, 1.5, 2.0];
load_resistance_ohm = [1e6, 48, 24, 16, 12];
approximate_no_load = [true, false, false, false, false];

minimum_uo_rms_v = 23.8;
maximum_uo_rms_v = 24.2;
minimum_frequency_hz = 49.8;
maximum_frequency_hz = 50.2;
maximum_uo_thd_percent = 2.0;
maximum_saturation_fraction = 0.01;
maximum_saturation_duration_s = 0.002;
maximum_load_regulation_percent = 0.2;
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
model_info_before = [];
owns_model_cleanup = false;

try
    if ~isfile(model_file)
        error('SINV:R2023A:LoadRegulationModelMissing', ...
            'The fixed 2023A load-regulation model does not exist: %s', ...
            model_file);
    end
    if ~isfile(exe)
        error('SINV:R2023A:LoadRegulationExecutableMissing', ...
            'The Debug/x64 SIL controller does not exist: %s', exe);
    end

    config = assert_generated_configuration(header_file, matlab_init_file, ...
        expected_build_level, expected_app_mode);
    if abs(config.SINV_2023A_UO_REF_RMS_V - target_voltage_rms_v) > 1e-12
        error('SINV:R2023A:LoadRegulationVoltageTargetMismatch', ...
            ['Generated SINV_2023A_UO_REF_RMS_V must be %.6g V, ' ...
             'but it is %.17g V.'], ...
            target_voltage_rms_v, config.SINV_2023A_UO_REF_RMS_V);
    end

    analysis_start_floor = max(steady_fraction * stop_time, ...
        config.SINV_2023A_SOFTSTART_TIME_S + 0.20);
    minimum_end_time = analysis_start_floor + ...
        minimum_analysis_cycles / nominal_frequency_hz;
    if stop_time < minimum_end_time
        error('SINV:R2023A:LoadRegulationStopTimeTooShort', ...
            ['Stop time %.6g s leaves fewer than %d complete steady ' ...
             'cycles after soft start and startup exclusion.'], ...
            stop_time, minimum_analysis_cycles);
    end

    if bdIsLoaded(model)
        error('SINV:R2023A:LoadRegulationModelAlreadyLoaded', ...
            ['Close the fixed validation model before running this function. ' ...
             'No loaded model will be changed or closed automatically: %s'], ...
            model_file);
    end
    assert_no_existing_controller_process(exe);

    model_info_before = dir(model_file);
    model_cleanup = onCleanup(@() close_model_without_saving(model)); %#ok<NASGU>
    owns_model_cleanup = true;
    load_system(model_file);

    assert_load_block_uses_variable(model);
    assert_physical_logger(model, '2023A Uo Physical Logger', 'uo_phys');
    assert_physical_logger(model, '2023A IG Io Physical Logger', 'io_phys');
    assert_physical_logger(model, '2023A IL Physical Logger', 'il_phys');

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

    % This temporary signal records the value compiled from SINV_RLOAD_OHM
    % in the same simulation context as the Load block. It therefore checks
    % that SimulationInput.setVariable remains effective after model InitFcn
    % callbacks. The probe exists only in memory and is never saved.
    add_load_override_probe(model);

    fprintf('\n2023A 0-2 A load-regulation validation\n');
    fprintf('  Model: %s\n', model_file);
    fprintf('  Controller: %s\n', exe);
    fprintf('  Generated BUILD_LEVEL/SINV_APP_MODE: %d/%d\n', ...
        config.BUILD_LEVEL, config.SINV_APP_MODE);
    fprintf('  Voltage-loop Kr: %.9g\n', ...
        config.SINV_2023A_VOLTAGE_LOOP_KR);
    fprintf('  Stop time: %.6g s\n', stop_time);

    point_count = numel(target_current_a);
    point_template = empty_load_point_result();
    point_results = repmat(point_template, point_count, 1);
    expected_point_fields = fieldnames(point_template);
    for point_index = 1:point_count
        requested_resistance = load_resistance_ohm(point_index);
        fprintf('\nLoad point %d/%d\n', point_index, point_count);
        fprintf('  Target current: %.6g A\n', ...
            target_current_a(point_index));
        fprintf('  Load resistance: %.9g Ohm\n', requested_resistance);
        fprintf('  Approximate no-load: %s\n', ...
            logical_text(approximate_no_load(point_index)));

        simulation_input = Simulink.SimulationInput(model);
        simulation_input = simulation_input.setVariable( ...
            'SINV_RLOAD_OHM', requested_resistance);
        simulation_input = simulation_input.setModelParameter( ...
            'StopTime', num2str(stop_time, 17), ...
            'ReturnWorkspaceOutputs', 'on');

        [out, process_id] = run_one_load_point( ...
            simulation_input, exe, root);

        applied_load = require_scalar_series(out, ...
            'applied_rload_ohm');
        applied_resistance = median(applied_load.data);
        override_tolerance = max(1e-9, ...
            1e-9 * abs(requested_resistance));
        override_confirmed = ...
            all(abs(applied_load.data - requested_resistance) <= ...
                override_tolerance);
        if ~override_confirmed
            error('SINV:R2023A:LoadOverrideIneffective', ...
                ['SimulationInput requested SINV_RLOAD_OHM=%.17g Ohm, ' ...
                 'but the compiled in-simulation probe recorded %.17g Ohm. ' ...
                 'The sweep stopped without saving the model.'], ...
                requested_resistance, applied_resistance);
        end

        point_result = analyze_load_point( ...
            out, target_current_a(point_index), requested_resistance, ...
            approximate_no_load(point_index), applied_resistance, ...
            process_id, config, analysis_start_floor, ...
            nominal_frequency_hz, minimum_analysis_cycles, ...
            maximum_analysis_cycles, samples_per_cycle, ...
            minimum_uo_rms_v, maximum_uo_rms_v, ...
            minimum_frequency_hz, maximum_frequency_hz, ...
            maximum_uo_thd_percent, maximum_saturation_fraction, ...
            maximum_saturation_duration_s);

        actual_point_fields = fieldnames(point_result);
        missing_point_fields = setdiff( ...
            expected_point_fields, actual_point_fields, 'stable');
        extra_point_fields = setdiff( ...
            actual_point_fields, expected_point_fields, 'stable');
        if ~isempty(missing_point_fields) || ~isempty(extra_point_fields)
            error('SINV:R2023A:LoadRegulationPointSchemaMismatch', ...
                ['Load point %d returned a noncanonical result structure. ' ...
                 'Missing fields: %s. Extra fields: %s.'], ...
                point_index, format_field_names(missing_point_fields), ...
                format_field_names(extra_point_fields));
        end
        point_result = orderfields(point_result, point_template);
        point_results(point_index) = point_result;
    end

    assert_load_block_uses_variable(model);
    close_system(model, 0);

    model_info_after = dir(model_file);
    model_file_unchanged = ...
        model_info_after.bytes == model_info_before.bytes && ...
        model_info_after.datenum == model_info_before.datenum;
    if ~model_file_unchanged
        error('SINV:R2023A:LoadRegulationModelFileChanged', ...
            ['The fixed target model changed during validation. Results were ' ...
             'not written; inspect the model immediately: %s'], model_file);
    end

    all_controller_processes_stopped = ...
        controller_process_count(exe) == 0;
    if ~all_controller_processes_stopped
        error('SINV:R2023A:LoadRegulationControllerStillRunning', ...
            ['A controller process remains after the sweep. Results were not ' ...
             'written; stop the process and inspect the environment.']);
    end

    all_uo_rms_v = [point_results.uo_phys_rms_v];
    u_no_load = point_results(1).uo_phys_rms_v;
    u_full_load = point_results(end).uo_phys_rms_v;
    endpoint_load_regulation_percent = ...
        abs(u_no_load - u_full_load) / u_full_load * 100;
    voltage_span_v = max(all_uo_rms_v) - min(all_uo_rms_v);
    span_load_regulation_percent = ...
        voltage_span_v / target_voltage_rms_v * 100;

    overall_checks = struct( ...
        'all_five_load_points_completed', ...
            numel(point_results) == point_count, ...
        'all_point_checks_passed', all([point_results.passed]), ...
        'endpoint_load_regulation_is_at_most_0p2_percent', ...
            endpoint_load_regulation_percent <= ...
                maximum_load_regulation_percent, ...
        'span_load_regulation_is_at_most_0p2_percent', ...
            span_load_regulation_percent <= ...
                maximum_load_regulation_percent, ...
        'all_points_have_zero_errors', ...
            all([point_results.active_errors_final] == 0), ...
        'all_points_have_zero_divergence', ...
            all([point_results.divergence_final] == 0), ...
        'all_points_avoid_persistent_current_limiting', ...
            all([point_results.current_ref_saturation_fraction] <= ...
                maximum_saturation_fraction) && ...
            all([point_results.current_ref_max_saturation_duration_s] <= ...
                maximum_saturation_duration_s), ...
        'model_file_is_unchanged', model_file_unchanged, ...
        'all_controller_processes_are_stopped', ...
            all_controller_processes_stopped);
    passed = all(structfun(@(value) logical(value), overall_checks));

    results = struct( ...
        'model', model, ...
        'stop_time_s', stop_time, ...
        'build_level', config.BUILD_LEVEL, ...
        'sinv_app_mode', config.SINV_APP_MODE, ...
        'voltage_loop_kr', config.SINV_2023A_VOLTAGE_LOOP_KR, ...
        'target_voltage_rms_v', target_voltage_rms_v, ...
        'endpoint_load_regulation_percent', ...
            endpoint_load_regulation_percent, ...
        'span_load_regulation_percent', ...
            span_load_regulation_percent, ...
        'voltage_span_v', voltage_span_v, ...
        'point_results', point_results, ...
        'pass_checks', overall_checks, ...
        'passed', passed);

    result_dir = fullfile(root, 'validation');
    if ~isfolder(result_dir)
        mkdir(result_dir);
    end
    json_file = fullfile(result_dir, ...
        '2023a_load_regulation_metrics.json');
    csv_file = fullfile(result_dir, ...
        '2023a_load_regulation_points.csv');
    png_file = fullfile(result_dir, ...
        '2023a_load_regulation_curve.png');
    harmonics_csv_file = fullfile(result_dir, ...
        '2023a_load_regulation_harmonics.csv');
    harmonic_diagnostics_png_file = fullfile(result_dir, ...
        '2023a_load_regulation_harmonic_diagnostics.png');

    write_load_regulation_figure(png_file, point_results, ...
        target_voltage_rms_v, minimum_uo_rms_v, maximum_uo_rms_v, ...
        maximum_uo_thd_percent, endpoint_load_regulation_percent, ...
        span_load_regulation_percent);
    write_points_csv(csv_file, point_results);
    write_harmonics_csv(harmonics_csv_file, point_results);
    write_harmonic_diagnostics_figure( ...
        harmonic_diagnostics_png_file, point_results);
    write_json(json_file, results);

    fprintf('\nNo-load voltage: %.9f Vrms\n', u_no_load);
    fprintf('Full-load voltage: %.9f Vrms\n', u_full_load);
    fprintf('Endpoint load regulation: %.9f %%\n', ...
        endpoint_load_regulation_percent);
    fprintf('Maximum voltage span: %.9f V\n', voltage_span_v);
    fprintf('Span load regulation: %.9f %%\n', ...
        span_load_regulation_percent);
    fprintf('Overall %s\n', pass_text(passed));
    fprintf('JSON: %s\n', json_file);
    fprintf('CSV:  %s\n', csv_file);
    fprintf('PNG:  %s\n', png_file);
    fprintf('Harmonics CSV: %s\n', harmonics_csv_file);
    fprintf('Harmonic diagnostics PNG: %s\n', ...
        harmonic_diagnostics_png_file);

    print_point_summary(point_results);
    print_harmonic_diagnostics(point_results);
    if ~passed
        print_failures(point_results, overall_checks, ...
            endpoint_load_regulation_percent, ...
            span_load_regulation_percent);
        error('SINV:R2023A:LoadRegulationValidationFailed', ...
            'Load-regulation validation failed; see JSON, CSV and PNG outputs.');
    end
catch ME
    % The cleanup object closes the model again when this function unwinds.
    % Close it here as well so the cleanup state is observable before the
    % original exception is reported, without ever saving the dirty model.
    if owns_model_cleanup
        close_model_without_saving(model);
    end
    report_exception_cleanup_state(model_file, model_info_before, exe);
    report_caught_exception(ME);
    rethrow(ME);
end
end


function result = empty_load_point_result()
result = struct( ...
    'target_current_a', NaN, ...
    'load_resistance_ohm', NaN, ...
    'approximate_no_load', false, ...
    'applied_load_resistance_ohm', NaN, ...
    'load_override_confirmed', false, ...
    'controller_process_id', NaN, ...
    'uo_phys_rms_v', NaN, ...
    'io_phys_rms_a', NaN, ...
    'il_phys_rms_a', NaN, ...
    'output_frequency_hz', NaN, ...
    'uo_phys_thd_percent', NaN, ...
    'fundamental_peak_v', NaN, ...
    'fundamental_rms_v', NaN, ...
    'harmonic_peak_v_h2_to_h15', NaN(1, 14), ...
    'harmonic_percent_h2_to_h15', NaN(1, 14), ...
    'h2_percent', NaN, ...
    'h3_percent', NaN, ...
    'h4_percent', NaN, ...
    'h5_percent', NaN, ...
    'h6_percent', NaN, ...
    'h7_percent', NaN, ...
    'h9_percent', NaN, ...
    'h11_percent', NaN, ...
    'h13_percent', NaN, ...
    'h15_percent', NaN, ...
    'low_order_odd_thd_percent', NaN, ...
    'low_order_even_thd_percent', NaN, ...
    'dominant_harmonic_order_1', NaN, ...
    'dominant_harmonic_percent_1', NaN, ...
    'dominant_harmonic_order_2', NaN, ...
    'dominant_harmonic_percent_2', NaN, ...
    'dominant_harmonic_order_3', NaN, ...
    'dominant_harmonic_percent_3', NaN, ...
    'uo_dc_offset_v', NaN, ...
    'uo_positive_peak_v', NaN, ...
    'uo_negative_peak_abs_v', NaN, ...
    'peak_asymmetry_percent', NaN, ...
    'uo_positive_half_rms_v', NaN, ...
    'uo_negative_half_rms_v', NaN, ...
    'half_cycle_rms_asymmetry_percent', NaN, ...
    'uo_crest_factor', NaN, ...
    'light_load', false, ...
    'capacitor_current_expected_rms_a', NaN, ...
    'inductor_current_excess_rms_a', NaN, ...
    'diagnostic_flags', empty_diagnostic_flags(), ...
    'current_ref_peak_a', NaN, ...
    'current_ref_rms_a', NaN, ...
    'current_ref_limit_peak_a', NaN, ...
    'current_ref_saturation_fraction', NaN, ...
    'current_ref_max_saturation_duration_s', NaN, ...
    'output_enable_final', NaN, ...
    'cia402_state_final', NaN, ...
    'active_errors_final', NaN, ...
    'divergence_final', NaN, ...
    'steady_start_s', NaN, ...
    'steady_end_s', NaN, ...
    'steady_cycle_count', NaN, ...
    'expected_current_from_measured_voltage_a', NaN, ...
    'current_consistency_error_a', NaN, ...
    'current_consistency_tolerance_a', NaN, ...
    'pass_checks', empty_load_point_pass_checks(), ...
    'failed_checks', {cell(0, 1)}, ...
    'passed', false);
end


function flags = empty_diagnostic_flags()
flags = struct( ...
    'dominant_low_order_odd_harmonics', false, ...
    'significant_even_harmonics', false, ...
    'significant_dc_offset', false, ...
    'significant_peak_asymmetry', false, ...
    'significant_half_cycle_asymmetry', false, ...
    'light_load_thd_failure', false, ...
    'possible_zero_crossing_or_deadtime_nonlinearity', false, ...
    'possible_waveform_asymmetry', false, ...
    'possible_lc_light_load_resonance', false);
end


function checks = empty_load_point_pass_checks()
checks = struct( ...
    'load_override_matches_requested_resistance', false, ...
    'uo_rms_is_between_23p8_and_24p2_v', false, ...
    'frequency_is_between_49p8_and_50p2_hz', false, ...
    'uo_thd_is_at_most_2_percent', false, ...
    'current_ref_saturation_fraction_is_at_most_0p01', false, ...
    'current_ref_max_saturation_duration_is_at_most_0p002_s', false, ...
    'output_enable_is_one_throughout_steady_window', false, ...
    'cia402_state_is_four_throughout_steady_window', false, ...
    'active_errors_are_zero_throughout_steady_window', false, ...
    'divergence_is_zero_throughout_steady_window', false, ...
    'load_current_consistency_passed', false);
end


function text = format_field_names(fields)
if isempty(fields)
    text = '<none>';
else
    text = strjoin(fields, ', ');
end
end


function point = analyze_load_point(out, target_current_a, ...
        load_resistance_ohm, approximate_no_load, ...
        applied_load_resistance_ohm, controller_process_id, config, ...
        analysis_start_floor, nominal_frequency_hz, ...
        minimum_analysis_cycles, maximum_analysis_cycles, ...
        samples_per_cycle, minimum_uo_rms_v, maximum_uo_rms_v, ...
        minimum_frequency_hz, maximum_frequency_hz, ...
        maximum_uo_thd_percent, maximum_saturation_fraction, ...
        maximum_saturation_duration_s)
point = empty_load_point_result();
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
    error('SINV:R2023A:LoadRegulationSteadyWindowTooShort', ...
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

% Waveform symmetry is evaluated on the raw integer-cycle window before
% harmonic_thd removes its DC component.
uo_dc_offset_v = mean(double(uo_data(:)));
uo_positive_peak_v = max(double(uo_data(:)));
uo_negative_peak_abs_v = abs(min(double(uo_data(:))));
peak_asymmetry_percent = abs( ...
    uo_positive_peak_v - uo_negative_peak_abs_v) / ...
    max(eps, 0.5 * (uo_positive_peak_v + uo_negative_peak_abs_v)) * 100;
positive_half_data = uo_data(uo_data >= 0);
negative_half_data = uo_data(uo_data < 0);
if isempty(positive_half_data) || isempty(negative_half_data)
    error('SINV:R2023A:LoadRegulationWaveformPolarityMissing', ...
        'Uo_phys must contain both positive and negative half cycles.');
end
uo_positive_half_rms_v = waveform_rms(positive_half_data);
uo_negative_half_rms_v = waveform_rms(negative_half_data);
half_cycle_rms_asymmetry_percent = abs( ...
    uo_positive_half_rms_v - uo_negative_half_rms_v) / ...
    max(eps, 0.5 * (uo_positive_half_rms_v + ...
        uo_negative_half_rms_v)) * 100;
uo_crest_factor = max(abs(double(uo_data(:)))) / uo_rms_v;

[uo_thd_percent, harmonic_orders, harmonic_peak_v] = harmonic_thd( ...
    uo_data, analysis_cycles, samples_per_cycle, 40);
if numel(harmonic_orders) < 15 || ...
        any(harmonic_orders(1:15) ~= (1:15)')
    error('SINV:R2023A:LoadRegulationHarmonicRangeInvalid', ...
        'The harmonic analysis must provide contiguous orders H1 through H15.');
end
harmonic_peak_v_h1_to_h15 = reshape(harmonic_peak_v(1:15), 1, []);
fundamental_peak_v = harmonic_peak_v_h1_to_h15(1);
fundamental_rms_v = fundamental_peak_v / sqrt(2);
harmonic_peak_v_h2_to_h15 = harmonic_peak_v_h1_to_h15(2:15);
harmonic_percent_h2_to_h15 = ...
    harmonic_peak_v_h2_to_h15 / fundamental_peak_v * 100;
h2_percent = harmonic_percent_h2_to_h15(1);
h3_percent = harmonic_percent_h2_to_h15(2);
h4_percent = harmonic_percent_h2_to_h15(3);
h5_percent = harmonic_percent_h2_to_h15(4);
h6_percent = harmonic_percent_h2_to_h15(5);
h7_percent = harmonic_percent_h2_to_h15(6);
h9_percent = harmonic_percent_h2_to_h15(8);
h11_percent = harmonic_percent_h2_to_h15(10);
h13_percent = harmonic_percent_h2_to_h15(12);
h15_percent = harmonic_percent_h2_to_h15(14);
low_order_odd_thd_percent = sqrt(sum( ...
    harmonic_peak_v_h1_to_h15([3, 5, 7, 9]).^2)) / ...
    fundamental_peak_v * 100;
low_order_even_thd_percent = sqrt(sum( ...
    harmonic_peak_v_h1_to_h15([2, 4, 6, 8]).^2)) / ...
    fundamental_peak_v * 100;
[dominant_percent, dominant_index] = sort( ...
    harmonic_percent_h2_to_h15, 'descend');
dominant_order = dominant_index + 1;

light_load = io_rms_a < 0.75;
capacitor_current_expected_rms_a = 2 * pi * output_frequency_hz * ...
    config.SINV_FILTER_CAPACITANCE_F * uo_rms_v;
inductor_current_excess_rms_a = ...
    il_rms_a - capacitor_current_expected_rms_a;

% Diagnostic flags are heuristic indications, not proven root causes.
diagnostic_flags = empty_diagnostic_flags();
diagnostic_flags.dominant_low_order_odd_harmonics = ...
    low_order_odd_thd_percent >= 1.0;
diagnostic_flags.significant_even_harmonics = ...
    low_order_even_thd_percent >= 0.5;
diagnostic_flags.significant_dc_offset = abs(uo_dc_offset_v) >= 0.05;
diagnostic_flags.significant_peak_asymmetry = ...
    peak_asymmetry_percent >= 0.5;
diagnostic_flags.significant_half_cycle_asymmetry = ...
    half_cycle_rms_asymmetry_percent >= 0.5;
diagnostic_flags.light_load_thd_failure = ...
    light_load && uo_thd_percent > 2.0;
diagnostic_flags.possible_zero_crossing_or_deadtime_nonlinearity = ...
    light_load && low_order_odd_thd_percent >= 1.0 && ...
    low_order_even_thd_percent < 0.5 && abs(uo_dc_offset_v) < 0.05;
diagnostic_flags.possible_waveform_asymmetry = ...
    diagnostic_flags.significant_even_harmonics || ...
    diagnostic_flags.significant_dc_offset || ...
    diagnostic_flags.significant_peak_asymmetry || ...
    diagnostic_flags.significant_half_cycle_asymmetry;
diagnostic_flags.possible_lc_light_load_resonance = ...
    light_load && uo_thd_percent > 2.0 && ...
    low_order_odd_thd_percent < 1.0 && ...
    low_order_even_thd_percent < 0.5;

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

expected_current_from_measured_voltage_a = ...
    uo_rms_v / load_resistance_ohm;
current_consistency_error_a = abs( ...
    io_rms_a - expected_current_from_measured_voltage_a);
if approximate_no_load
    load_current_consistency_passed = io_rms_a <= 0.01;
    current_consistency_tolerance_a = 0.01;
else
    current_consistency_tolerance_a = max(0.01, ...
        0.01 * expected_current_from_measured_voltage_a);
    load_current_consistency_passed = ...
        current_consistency_error_a <= current_consistency_tolerance_a;
end

checks = empty_load_point_pass_checks();
checks.load_override_matches_requested_resistance = ...
    abs(applied_load_resistance_ohm - load_resistance_ohm) <= ...
        max(1e-9, 1e-9 * abs(load_resistance_ohm));
checks.uo_rms_is_between_23p8_and_24p2_v = ...
    uo_rms_v >= minimum_uo_rms_v && uo_rms_v <= maximum_uo_rms_v;
checks.frequency_is_between_49p8_and_50p2_hz = ...
    output_frequency_hz >= minimum_frequency_hz && ...
        output_frequency_hz <= maximum_frequency_hz;
checks.uo_thd_is_at_most_2_percent = ...
    uo_thd_percent <= maximum_uo_thd_percent;
checks.current_ref_saturation_fraction_is_at_most_0p01 = ...
    current_ref_saturation_fraction <= maximum_saturation_fraction;
checks.current_ref_max_saturation_duration_is_at_most_0p002_s = ...
    saturation_duration_s <= maximum_saturation_duration_s;
checks.output_enable_is_one_throughout_steady_window = ...
    all(enable_data == 1);
checks.cia402_state_is_four_throughout_steady_window = ...
    all(state_data == 4);
checks.active_errors_are_zero_throughout_steady_window = ...
    all(error_data == 0);
checks.divergence_is_zero_throughout_steady_window = ...
    all(divergence_data == 0);
checks.load_current_consistency_passed = ...
    load_current_consistency_passed;
passed = all(structfun(@(value) logical(value), checks));

point.target_current_a = target_current_a;
point.load_resistance_ohm = load_resistance_ohm;
point.approximate_no_load = logical(approximate_no_load);
point.applied_load_resistance_ohm = applied_load_resistance_ohm;
point.load_override_confirmed = true;
point.controller_process_id = double(controller_process_id);
point.uo_phys_rms_v = uo_rms_v;
point.io_phys_rms_a = io_rms_a;
point.il_phys_rms_a = il_rms_a;
point.output_frequency_hz = output_frequency_hz;
point.uo_phys_thd_percent = uo_thd_percent;
point.fundamental_peak_v = fundamental_peak_v;
point.fundamental_rms_v = fundamental_rms_v;
point.harmonic_peak_v_h2_to_h15 = harmonic_peak_v_h2_to_h15;
point.harmonic_percent_h2_to_h15 = harmonic_percent_h2_to_h15;
point.h2_percent = h2_percent;
point.h3_percent = h3_percent;
point.h4_percent = h4_percent;
point.h5_percent = h5_percent;
point.h6_percent = h6_percent;
point.h7_percent = h7_percent;
point.h9_percent = h9_percent;
point.h11_percent = h11_percent;
point.h13_percent = h13_percent;
point.h15_percent = h15_percent;
point.low_order_odd_thd_percent = low_order_odd_thd_percent;
point.low_order_even_thd_percent = low_order_even_thd_percent;
point.dominant_harmonic_order_1 = dominant_order(1);
point.dominant_harmonic_percent_1 = dominant_percent(1);
point.dominant_harmonic_order_2 = dominant_order(2);
point.dominant_harmonic_percent_2 = dominant_percent(2);
point.dominant_harmonic_order_3 = dominant_order(3);
point.dominant_harmonic_percent_3 = dominant_percent(3);
point.uo_dc_offset_v = uo_dc_offset_v;
point.uo_positive_peak_v = uo_positive_peak_v;
point.uo_negative_peak_abs_v = uo_negative_peak_abs_v;
point.peak_asymmetry_percent = peak_asymmetry_percent;
point.uo_positive_half_rms_v = uo_positive_half_rms_v;
point.uo_negative_half_rms_v = uo_negative_half_rms_v;
point.half_cycle_rms_asymmetry_percent = ...
    half_cycle_rms_asymmetry_percent;
point.uo_crest_factor = uo_crest_factor;
point.light_load = logical(light_load);
point.capacitor_current_expected_rms_a = ...
    capacitor_current_expected_rms_a;
point.inductor_current_excess_rms_a = inductor_current_excess_rms_a;
point.diagnostic_flags = diagnostic_flags;
point.current_ref_peak_a = current_ref_peak_a;
point.current_ref_rms_a = current_ref_rms_a;
point.current_ref_limit_peak_a = current_ref_limit_peak_a;
point.current_ref_saturation_fraction = current_ref_saturation_fraction;
point.current_ref_max_saturation_duration_s = saturation_duration_s;
point.output_enable_final = output_enable_final;
point.cia402_state_final = cia402_state_final;
point.active_errors_final = active_errors_final;
point.divergence_final = divergence_final;
point.steady_start_s = analysis_start;
point.steady_end_s = analysis_end;
point.steady_cycle_count = analysis_cycles;
point.expected_current_from_measured_voltage_a = ...
    expected_current_from_measured_voltage_a;
point.current_consistency_error_a = current_consistency_error_a;
point.current_consistency_tolerance_a = current_consistency_tolerance_a;
point.pass_checks = checks;
point.failed_checks = failed_check_names(checks);
point.passed = passed;
end


function [out, process_id] = run_one_load_point(simulation_input, exe, root)
assert_no_existing_controller_process(exe);
controller = start_controller(exe, root);
controller_cleanup = onCleanup( ...
    @() stop_controller_quietly(controller)); %#ok<NASGU>
process_id = double(controller.Id);
fprintf('  Controller process: PID %d\n', process_id);
pause(0.25);
if controller.HasExited
    error('SINV:R2023A:LoadRegulationControllerExitedEarly', ...
        'The Debug/x64 SIL controller exited before simulation started.');
end

fprintf('  Simulation status: running\n');
out = sim(simulation_input);

% Stop the exact process before any caller-side signal analysis.
stop_controller(controller);
fprintf('  Simulation status: completed; controller stopped\n');
end


function config = assert_generated_configuration( ...
        header_file, matlab_init_file, expected_build_level, expected_app_mode)
if ~isfile(header_file) || ~isfile(matlab_init_file)
    error('SINV:R2023A:LoadRegulationGeneratedSettingsMissing', ...
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
    'SINV_FILTER_CAPACITANCE_F'
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
        error('SINV:R2023A:LoadRegulationGeneratedSettingsDisagree', ...
            'Generated C and MATLAB values disagree for %s.', name);
    end
    config.(name) = c_value;
end

if config.BUILD_LEVEL ~= expected_build_level || ...
        config.SINV_APP_MODE ~= expected_app_mode || ...
        config.SINV_APP_MODE_STANDARD_BL2 ~= 0 || ...
        config.SINV_APP_MODE_2023A_SINGLE ~= expected_app_mode
    error('SINV:R2023A:LoadRegulationGeneratedModeMismatch', ...
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
    error('SINV:R2023A:LoadRegulationMacroMissingOrDuplicate', ...
        'Expected exactly one generated C macro named %s.', name);
end
value = str2double(tokens{1}{1});
if ~isfinite(value)
    error('SINV:R2023A:LoadRegulationMacroInvalid', ...
        'Generated C macro %s is not finite.', name);
end
end


function value = read_matlab_numeric_assignment(text, name)
number = '([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)';
expression = ['(?m)^\s*' name '\s*=\s*' number '\s*;\s*$'];
tokens = regexp(text, expression, 'tokens');
if numel(tokens) ~= 1
    error('SINV:R2023A:LoadRegulationVariableMissingOrDuplicate', ...
        'Expected exactly one generated MATLAB variable named %s.', name);
end
value = str2double(tokens{1}{1});
if ~isfinite(value)
    error('SINV:R2023A:LoadRegulationVariableInvalid', ...
        'Generated MATLAB variable %s is not finite.', name);
end
end


function assert_load_block_uses_variable(model)
load_block = [model '/Load'];
if getSimulinkBlockHandle(load_block) == -1
    error('SINV:R2023A:LoadRegulationLoadBlockMissing', ...
        'Required top-level Load block is missing: %s', load_block);
end
resistance_expression = strtrim(get_param(load_block, 'Resistance'));
if ~strcmp(resistance_expression, 'SINV_RLOAD_OHM')
    error('SINV:R2023A:LoadRegulationLoadExpressionMismatch', ...
        ['The Load block must retain Resistance=SINV_RLOAD_OHM; ' ...
         'current expression is %s.'], resistance_expression);
end
end


function assert_physical_logger(model, block_name, variable_name)
blocks = find_system(model, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'Type', 'Block', ...
    'Name', block_name);
if numel(blocks) ~= 1
    error('SINV:R2023A:LoadRegulationPhysicalLoggerMissing', ...
        'Expected one physical logger named %s, found %d.', ...
        block_name, numel(blocks));
end
if ~strcmp(get_param(blocks{1}, 'BlockType'), 'ToWorkspace') || ...
        ~strcmp(get_param(blocks{1}, 'VariableName'), variable_name) || ...
        ~strcmpi(get_param(blocks{1}, 'SaveFormat'), 'Timeseries')
    error('SINV:R2023A:LoadRegulationPhysicalLoggerInvalid', ...
        ['Logger "%s" must be a To Workspace block writing %s as a ' ...
         'timeseries.'], block_name, variable_name);
end
end


function add_monitor_logger(model, source_name, source_port, variable, index)
source = [model '/' source_name];
if getSimulinkBlockHandle(source) == -1
    error('SINV:R2023A:LoadRegulationMonitorSourceMissing', ...
        'Required controller monitor source is missing: %s', source);
end

block = sprintf('%s/2023A Load Regulation Logger %02d', model, index);
if getSimulinkBlockHandle(block) ~= -1
    error('SINV:R2023A:LoadRegulationTemporaryLoggerExists', ...
        'Temporary logger name is already present: %s', block);
end

source_handles = get_param(source, 'PortHandles');
if numel(source_handles.Outport) < source_port
    error('SINV:R2023A:LoadRegulationMonitorPortMissing', ...
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


function add_load_override_probe(model)
constant_block = [model '/2023A Applied Rload Probe'];
logger_block = [model '/2023A Applied Rload Logger'];
if getSimulinkBlockHandle(constant_block) ~= -1 || ...
        getSimulinkBlockHandle(logger_block) ~= -1
    error('SINV:R2023A:LoadRegulationProbeExists', ...
        'A temporary load-override probe already exists in the model.');
end

add_block('simulink/Sources/Constant', constant_block, ...
    'Value', 'SINV_RLOAD_OHM', ...
    'SampleTime', '0.001', ...
    'Position', [1120, 220, 1240, 250]);
add_block('simulink/Sinks/To Workspace', logger_block, ...
    'VariableName', 'applied_rload_ohm', ...
    'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', '10000', ...
    'Decimation', '1', ...
    'Position', [1300, 220, 1480, 250]);
constant_handles = get_param(constant_block, 'PortHandles');
logger_handles = get_param(logger_block, 'PortHandles');
add_line(model, constant_handles.Outport(1), logger_handles.Inport(1), ...
    'autorouting', 'on');
end


function process = start_controller(exe, root)
info = System.Diagnostics.ProcessStartInfo;
info.FileName = exe;
info.WorkingDirectory = root;
info.UseShellExecute = false;
info.CreateNoWindow = true;
process = System.Diagnostics.Process.Start(info);
if isempty(process)
    error('SINV:R2023A:LoadRegulationControllerStartFailed', ...
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
    error('SINV:R2023A:LoadRegulationControllerStopTimeout', ...
        'The SIL controller did not exit within 5 seconds.');
end
end


function stop_controller_quietly(process)
try
    stop_controller(process);
catch ME
    emit_cleanup_warning( ...
        'SINV:R2023A:LoadRegulationControllerCleanupFailed', ...
        sprintf('Could not confirm controller shutdown: %s', ME.message));
end
end


function count = controller_process_count(exe)
[~, process_name] = fileparts(exe);
process_list = enumerate_named_processes(process_name);
count = numel(process_list);
end


function assert_no_existing_controller_process(exe)
[~, process_name] = fileparts(exe);
process_list = enumerate_named_processes(process_name);
if isempty(process_list)
    return;
end

error('SINV:R2023A:LoadRegulationControllerAlreadyRunning', ...
    ['Existing SIL controller process detected.\n' ...
     'Process name: %s\nPID(s): %s\nMatches: %s\n' ...
     'Stop stale controllers manually before retrying.'], ...
    process_name, format_process_ids(process_list), ...
    format_process_descriptions(process_list));
end


function process_list = enumerate_named_processes(process_name)
% Return only serializable MATLAB values; never return a .NET Process.
processes = System.Diagnostics.Process.GetProcessesByName(process_name);
process_count = double(processes.Length);
process_template = struct( ...
    'pid', NaN, ...
    'name', char(process_name), ...
    'has_exited', false, ...
    'pid_available', false, ...
    'name_available', false, ...
    'has_exited_available', false);
process_list = repmat(process_template, 1, process_count);

for index = 1:process_count
    process = processes.GetValue(index - 1);

    try
        process_list(index).pid = double(process.Id);
        process_list(index).pid_available = true;
    catch
    end
    try
        process_list(index).name = char(process.ProcessName);
        process_list(index).name_available = true;
    catch
    end
    try
        process_list(index).has_exited = logical(process.HasExited);
        process_list(index).has_exited_available = true;
    catch
    end
end
end


function text = format_process_ids(process_list)
values = cell(1, numel(process_list));
for index = 1:numel(process_list)
    if process_list(index).pid_available
        values{index} = sprintf('%.0f', process_list(index).pid);
    else
        values{index} = sprintf('unavailable[%d]', index);
    end
end
text = strjoin(values, ', ');
end


function text = format_process_descriptions(process_list)
values = cell(1, numel(process_list));
for index = 1:numel(process_list)
    if process_list(index).has_exited_available
        if process_list(index).has_exited
            has_exited = 'true';
        else
            has_exited = 'false';
        end
    else
        has_exited = 'unknown';
    end
    values{index} = sprintf('%s (PID %s, HasExited=%s)', ...
        process_list(index).name, ...
        format_process_ids(process_list(index)), has_exited);
end
text = strjoin(values, '; ');
end


function close_model_without_saving(model)
try
    if bdIsLoaded(model)
        close_system(model, 0);
    end
catch ME
    emit_cleanup_warning('SINV:R2023A:LoadRegulationModelCleanupFailed', ...
        sprintf('Could not close the unsaved validation model: %s', ...
        ME.message));
end
end


function report_exception_cleanup_state(model_file, model_info_before, exe)
try
    report_exception_cleanup_state_impl( ...
        model_file, model_info_before, exe);
catch unexpected_error
    emit_cleanup_warning( ...
        'SINV:R2023A:LoadRegulationCleanupReportFailedSafely', ...
        sprintf('Cleanup-state reporting failed safely: %s', ...
        unexpected_error.message));
end
end


function report_exception_cleanup_state_impl( ...
        model_file, model_info_before, exe)
try
    if isfile(model_file) && ~isempty(model_info_before)
        model_info_after = dir(model_file);
        if model_info_after.bytes ~= model_info_before.bytes || ...
                model_info_after.datenum ~= model_info_before.datenum
            emit_cleanup_warning( ...
                'SINV:R2023A:LoadRegulationModelFileChangedDuringError', ...
                sprintf(['The target model size or timestamp changed before ' ...
                'validation stopped: %s'], model_file));
        end
    end
catch model_error
    emit_cleanup_warning( ...
        'SINV:R2023A:LoadRegulationModelCleanupStateUnavailable', ...
        sprintf('Could not inspect the model-file cleanup state: %s', ...
        model_error.message));
end

[~, process_name] = fileparts(exe);
try
    process_list = enumerate_named_processes(process_name);
catch enumeration_error
    emit_cleanup_warning( ...
        'SINV:R2023A:LoadRegulationProcessEnumerationFailed', ...
        sprintf(['Could not enumerate controller processes named %s; ' ...
        'no conclusion about residual processes was made. Reason: %s'], ...
        process_name, enumeration_error.message));
    return;
end

if isempty(process_list)
    emit_cleanup_info(sprintf( ...
        'Cleanup check: no %s process remains.', process_name));
else
    emit_cleanup_warning( ...
        'SINV:R2023A:LoadRegulationControllerRemainsDuringError', ...
        sprintf(['Controller cleanup detected matching process(es).\n' ...
        'Process name: %s\nPID(s): %s\nMatches: %s'], ...
        process_name, format_process_ids(process_list), ...
        format_process_descriptions(process_list)));
end
end


function emit_cleanup_info(message)
try
    fprintf('  %s\n', message);
catch
end
end


function emit_cleanup_warning(identifier, message)
try
    warning(identifier, '%s', message);
catch
    try
        fprintf(2, 'Warning [%s]: %s\n', identifier, message);
    catch
    end
end
end


function report_caught_exception(ME)
try
    fprintf(2, '\n2023A load-regulation validation stopped: %s\n', ...
        ME.message);
    for k = 1:numel(ME.stack)
        fprintf(2, '  at %s, line %d\n', ...
            ME.stack(k).name, ME.stack(k).line);
    end
catch
end
end


function series = require_scalar_series(out, name)
raw = out.get(name);
if isempty(raw)
    error('SINV:R2023A:LoadRegulationSignalEmpty', ...
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
    error('SINV:R2023A:LoadRegulationSignalFormatUnsupported', ...
        'Signal %s has unsupported class %s.', name, class(raw));
end

if isempty(time) || isempty(data) || numel(data) ~= numel(time)
    error('SINV:R2023A:LoadRegulationSignalNotScalar', ...
        'Signal %s must contain one scalar value per time sample.', name);
end
data = data(:);
if any(~isfinite(time)) || any(~isfinite(data))
    error('SINV:R2023A:LoadRegulationSignalNonFinite', ...
        'Signal %s contains NaN or Inf.', name);
end

[time, order] = sort(time);
data = data(order);
[time, unique_index] = unique(time, 'last');
data = data(unique_index);
if numel(time) < 2 || time(end) <= time(1)
    error('SINV:R2023A:LoadRegulationSignalTimeInvalid', ...
        'Signal %s does not contain a usable time interval.', name);
end
series = struct('name', name, 'time', time, 'data', data);
end


function frequency_hz = estimate_frequency(series, start_time, end_time, nominal)
if end_time <= start_time
    error('SINV:R2023A:LoadRegulationFrequencyWindowInvalid', ...
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
    error('SINV:R2023A:LoadRegulationFrequencyEstimateFailed', ...
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
    error('SINV:R2023A:LoadRegulationFrequencyEstimateFailed', ...
        'Uo_phys has no valid 40-60 Hz periods.');
end
frequency_hz = 1 / median(period);
end


function data = interpolate_series(series, query_time, method)
tolerance = 100 * eps(max(abs([series.time(1), series.time(end), ...
    query_time(1), query_time(end)])));
if query_time(1) < series.time(1) - tolerance || ...
        query_time(end) > series.time(end) + tolerance
    error('SINV:R2023A:LoadRegulationSignalWindowUnavailable', ...
        'Signal %s does not cover the requested steady window.', series.name);
end
data = interp1(series.time, series.data, query_time, method);
if any(~isfinite(data))
    error('SINV:R2023A:LoadRegulationInterpolationFailed', ...
        'Signal %s produced NaN or Inf during steady resampling.', ...
        series.name);
end
end


function data = window_values(series, start_time, end_time)
sample_period_s = median(diff(series.time));
coverage_tolerance_s = max(sample_period_s, ...
    100 * eps(max(abs([series.time(1), series.time(end), ...
    start_time, end_time]))));
if series.time(1) > start_time + coverage_tolerance_s || ...
        series.time(end) < end_time - coverage_tolerance_s
    error('SINV:R2023A:LoadRegulationStatusWindowIncomplete', ...
        'Signal %s does not cover the complete steady window.', series.name);
end
index = series.time >= start_time & series.time <= end_time;
data = series.data(index);
if isempty(data)
    error('SINV:R2023A:LoadRegulationStatusWindowUnavailable', ...
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
    error('SINV:R2023A:LoadRegulationHarmonicWindowInvalid', ...
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
    error('SINV:R2023A:LoadRegulationFundamentalMissing', ...
        'Uo_phys does not contain a usable fundamental.');
end
thd_percent = 100 * sqrt(sum(peak_amplitude(2:end).^2)) / ...
    peak_amplitude(1);
end


function [fraction, maximum_duration_s] = ...
        saturation_metrics(data, sample_period_s, limit_peak_a)
if ~isfinite(limit_peak_a) || limit_peak_a <= 0
    error('SINV:R2023A:LoadRegulationCurrentLimitInvalid', ...
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


function write_points_csv(filename, points)
failed_checks = cell(numel(points), 1);
for k = 1:numel(points)
    failed_checks{k} = strjoin(points(k).failed_checks, '|');
end

table_data = table( ...
    [points.target_current_a]', ...
    [points.load_resistance_ohm]', ...
    [points.approximate_no_load]', ...
    [points.applied_load_resistance_ohm]', ...
    [points.uo_phys_rms_v]', ...
    [points.io_phys_rms_a]', ...
    [points.il_phys_rms_a]', ...
    [points.output_frequency_hz]', ...
    [points.uo_phys_thd_percent]', ...
    [points.current_ref_peak_a]', ...
    [points.current_ref_rms_a]', ...
    [points.current_ref_limit_peak_a]', ...
    [points.current_ref_saturation_fraction]', ...
    [points.current_ref_max_saturation_duration_s]', ...
    [points.output_enable_final]', ...
    [points.cia402_state_final]', ...
    [points.active_errors_final]', ...
    [points.divergence_final]', ...
    [points.expected_current_from_measured_voltage_a]', ...
    [points.current_consistency_error_a]', ...
    [points.steady_start_s]', ...
    [points.steady_end_s]', ...
    [points.steady_cycle_count]', ...
    [points.passed]', ...
    failed_checks, ...
    'VariableNames', { ...
        'TargetCurrentA', ...
        'LoadResistanceOhm', ...
        'ApproximateNoLoad', ...
        'AppliedLoadResistanceOhm', ...
        'UoPhysRmsV', ...
        'IoPhysRmsA', ...
        'IlPhysRmsA', ...
        'OutputFrequencyHz', ...
        'UoPhysThdPercent', ...
        'CurrentRefPeakA', ...
        'CurrentRefRmsA', ...
        'CurrentRefLimitPeakA', ...
        'CurrentRefSaturationFraction', ...
        'CurrentRefMaxSaturationDurationS', ...
        'OutputEnableFinal', ...
        'Cia402StateFinal', ...
        'ActiveErrorsFinal', ...
        'DivergenceFinal', ...
        'ExpectedCurrentFromMeasuredVoltageA', ...
        'CurrentConsistencyErrorA', ...
        'SteadyStartS', ...
        'SteadyEndS', ...
        'SteadyCycleCount', ...
        'PointPassed', ...
        'FailedChecks'});
diagnostic_flags = [points.diagnostic_flags];
table_data.FundamentalPeakV = [points.fundamental_peak_v]';
table_data.FundamentalRmsV = [points.fundamental_rms_v]';
table_data.H2Percent = [points.h2_percent]';
table_data.H3Percent = [points.h3_percent]';
table_data.H4Percent = [points.h4_percent]';
table_data.H5Percent = [points.h5_percent]';
table_data.H6Percent = [points.h6_percent]';
table_data.H7Percent = [points.h7_percent]';
table_data.H9Percent = [points.h9_percent]';
table_data.H11Percent = [points.h11_percent]';
table_data.H13Percent = [points.h13_percent]';
table_data.H15Percent = [points.h15_percent]';
table_data.LowOrderOddThdPercent = ...
    [points.low_order_odd_thd_percent]';
table_data.LowOrderEvenThdPercent = ...
    [points.low_order_even_thd_percent]';
table_data.DominantHarmonicOrder1 = ...
    [points.dominant_harmonic_order_1]';
table_data.DominantHarmonicPercent1 = ...
    [points.dominant_harmonic_percent_1]';
table_data.DominantHarmonicOrder2 = ...
    [points.dominant_harmonic_order_2]';
table_data.DominantHarmonicPercent2 = ...
    [points.dominant_harmonic_percent_2]';
table_data.DominantHarmonicOrder3 = ...
    [points.dominant_harmonic_order_3]';
table_data.DominantHarmonicPercent3 = ...
    [points.dominant_harmonic_percent_3]';
table_data.UoDcOffsetV = [points.uo_dc_offset_v]';
table_data.UoPositivePeakV = [points.uo_positive_peak_v]';
table_data.UoNegativePeakAbsV = [points.uo_negative_peak_abs_v]';
table_data.PeakAsymmetryPercent = [points.peak_asymmetry_percent]';
table_data.UoPositiveHalfRmsV = [points.uo_positive_half_rms_v]';
table_data.UoNegativeHalfRmsV = [points.uo_negative_half_rms_v]';
table_data.HalfCycleRmsAsymmetryPercent = ...
    [points.half_cycle_rms_asymmetry_percent]';
table_data.UoCrestFactor = [points.uo_crest_factor]';
table_data.LightLoad = [points.light_load]';
table_data.CapacitorCurrentExpectedRmsA = ...
    [points.capacitor_current_expected_rms_a]';
table_data.InductorCurrentExcessRmsA = ...
    [points.inductor_current_excess_rms_a]';
table_data.DominantLowOrderOddHarmonics = ...
    [diagnostic_flags.dominant_low_order_odd_harmonics]';
table_data.SignificantEvenHarmonics = ...
    [diagnostic_flags.significant_even_harmonics]';
table_data.SignificantDcOffset = ...
    [diagnostic_flags.significant_dc_offset]';
table_data.SignificantPeakAsymmetry = ...
    [diagnostic_flags.significant_peak_asymmetry]';
table_data.SignificantHalfCycleAsymmetry = ...
    [diagnostic_flags.significant_half_cycle_asymmetry]';
table_data.LightLoadThdFailure = ...
    [diagnostic_flags.light_load_thd_failure]';
table_data.PossibleZeroCrossingOrDeadtimeNonlinearity = ...
    [diagnostic_flags.possible_zero_crossing_or_deadtime_nonlinearity]';
table_data.PossibleWaveformAsymmetry = ...
    [diagnostic_flags.possible_waveform_asymmetry]';
table_data.PossibleLcLightLoadResonance = ...
    [diagnostic_flags.possible_lc_light_load_resonance]';
writetable(table_data, filename);
end


function write_harmonics_csv(filename, points)
harmonic_count = 15;
row_count = numel(points) * harmonic_count;
target_current_a = zeros(row_count, 1);
measured_io_rms_a = zeros(row_count, 1);
load_resistance_ohm = zeros(row_count, 1);
harmonic_order = zeros(row_count, 1);
harmonic_peak_v = zeros(row_count, 1);
harmonic_percent_of_fundamental = zeros(row_count, 1);

for k = 1:numel(points)
    rows = ((k - 1) * harmonic_count + (1:harmonic_count))';
    point_harmonic_peak_v = [points(k).fundamental_peak_v, ...
        points(k).harmonic_peak_v_h2_to_h15];
    point_harmonic_percent = [100, ...
        points(k).harmonic_percent_h2_to_h15];
    target_current_a(rows) = points(k).target_current_a;
    measured_io_rms_a(rows) = points(k).io_phys_rms_a;
    load_resistance_ohm(rows) = points(k).load_resistance_ohm;
    harmonic_order(rows) = (1:harmonic_count)';
    harmonic_peak_v(rows) = point_harmonic_peak_v(:);
    harmonic_percent_of_fundamental(rows) = ...
        point_harmonic_percent(:);
end

table_data = table(target_current_a, measured_io_rms_a, ...
    load_resistance_ohm, harmonic_order, harmonic_peak_v, ...
    harmonic_percent_of_fundamental);
writetable(table_data, filename);
end


function write_load_regulation_figure(filename, points, target_voltage, ...
        minimum_voltage, maximum_voltage, maximum_thd, ...
        endpoint_regulation, span_regulation)
measured_io = [points.io_phys_rms_a];
uo_rms = [points.uo_phys_rms_v];
uo_thd = [points.uo_phys_thd_percent];
iref_peak = [points.current_ref_peak_a];
iref_limit = [points.current_ref_limit_peak_a];

fig = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1200, 1100]);
figure_cleanup = onCleanup(@() close_figure(fig)); %#ok<NASGU>
layout = tiledlayout(fig, 3, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(layout);
plot(measured_io, uo_rms, '-o', 'LineWidth', 1.2, ...
    'MarkerFaceColor', [0.0, 0.45, 0.74]);
hold on;
yline(target_voltage, '-k', '24.0 V target');
yline(minimum_voltage, '--r', '23.8 V lower bound');
yline(maximum_voltage, '--r', '24.2 V upper bound');
hold off;
grid on;
xlabel('Measured Io RMS (A)');
ylabel('Uo RMS (V)');
title('Output-voltage load-regulation curve');

nexttile(layout);
plot(measured_io, uo_thd, '-o', 'LineWidth', 1.2, ...
    'MarkerFaceColor', [0.47, 0.67, 0.19]);
hold on;
yline(maximum_thd, '--r', '2% THD boundary');
hold off;
grid on;
xlabel('Measured Io RMS (A)');
ylabel('Uo THD (%)');
title('Output-voltage THD over load');

nexttile(layout);
plot(measured_io, iref_peak, '-o', 'LineWidth', 1.2, ...
    'MarkerFaceColor', [0.85, 0.33, 0.10]);
hold on;
plot(measured_io, iref_limit, '--k', 'LineWidth', 1.1);
hold off;
grid on;
xlabel('Measured Io RMS (A)');
ylabel('Current reference peak (A)');
legend('Iref peak', 'Iref limit', 'Location', 'best');
title('Current-reference peak versus configured limit');

title(layout, sprintf([ ...
    '2023A load regulation: endpoint %.6f %%, span %.6f %%'], ...
    endpoint_regulation, span_regulation));
exportgraphics(fig, filename, 'Resolution', 160);
end


function write_harmonic_diagnostics_figure(filename, points)
measured_io = [points.io_phys_rms_a];
near_no_load_index = find([points.approximate_no_load], 1, 'first');
two_amp_index = find(abs([points.target_current_a] - 2.0) <= 1e-12, ...
    1, 'first');
if isempty(near_no_load_index) || isempty(two_amp_index)
    error('SINV:R2023A:LoadRegulationDiagnosticPointMissing', ...
        'Harmonic diagnostics require approximate no-load and 2 A points.');
end

fig = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1400, 950]);
figure_cleanup = onCleanup(@() close_figure(fig)); %#ok<NASGU>
layout = tiledlayout(fig, 2, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(layout);
plot(measured_io, [points.h3_percent], '-o', 'LineWidth', 1.2);
hold on;
plot(measured_io, [points.h5_percent], '-s', 'LineWidth', 1.2);
plot(measured_io, [points.h7_percent], '-^', 'LineWidth', 1.2);
plot(measured_io, [points.h9_percent], '-d', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Measured Io RMS (A)');
ylabel('Percent of fundamental (%)');
legend('H3', 'H5', 'H7', 'H9', 'Location', 'best');
title('Low-order odd harmonics over load');

nexttile(layout);
plot(measured_io, [points.low_order_odd_thd_percent], ...
    '-o', 'LineWidth', 1.2);
hold on;
plot(measured_io, [points.low_order_even_thd_percent], ...
    '-s', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Measured Io RMS (A)');
ylabel('Composite harmonic content (%)');
legend('H3/H5/H7/H9', 'H2/H4/H6/H8', 'Location', 'best');
title('Low-order odd/even composite content');

nexttile(layout);
yyaxis left;
plot(measured_io, [points.uo_dc_offset_v], ...
    '-o', 'LineWidth', 1.2);
ylabel('DC offset (V)');
yyaxis right;
plot(measured_io, [points.peak_asymmetry_percent], ...
    '-s', 'LineWidth', 1.2);
hold on;
plot(measured_io, [points.half_cycle_rms_asymmetry_percent], ...
    '-^', 'LineWidth', 1.2);
hold off;
ylabel('Asymmetry (%)');
grid on;
xlabel('Measured Io RMS (A)');
legend('DC offset', 'Peak asymmetry', 'Half-cycle RMS asymmetry', ...
    'Location', 'best');
title('Waveform symmetry diagnostics');

nexttile(layout);
harmonic_orders = 2:15;
comparison_percent = [ ...
    points(near_no_load_index).harmonic_percent_h2_to_h15; ...
    points(two_amp_index).harmonic_percent_h2_to_h15]';
bar(harmonic_orders, comparison_percent, 'grouped');
grid on;
xlabel('Harmonic order');
ylabel('Percent of fundamental (%)');
legend('Approximate no-load', '2 A target', 'Location', 'best');
title('H2-H15 comparison');
xticks(harmonic_orders);

title(layout, '2023A load-regulation harmonic diagnostics');
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


function close_file_quietly(fid)
try
    fclose(fid);
catch
end
end


function write_json(filename, results)
fid = fopen(filename, 'w');
if fid < 0
    error('SINV:R2023A:LoadRegulationJSONOpenFailed', ...
        'Could not create metrics file: %s', filename);
end
file_cleanup = onCleanup(@() close_file_quietly(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', jsonencode(results, PrettyPrint=true));
end


function print_point_summary(points)
fprintf('\n');
fprintf(['Target I   Rload       Measured Io   Measured Uo   Frequency   ' ...
    'THD       Iref peak   Saturation   Point PASS\n']);
for k = 1:numel(points)
    fprintf('%7.3f  %10.3f  %11.6f  %11.6f  %9.6f  %7.4f  %10.6f  %10.6f  %s\n', ...
        points(k).target_current_a, points(k).load_resistance_ohm, ...
        points(k).io_phys_rms_a, points(k).uo_phys_rms_v, ...
        points(k).output_frequency_hz, points(k).uo_phys_thd_percent, ...
        points(k).current_ref_peak_a, ...
        points(k).current_ref_saturation_fraction, ...
        pass_text(points(k).passed));
end
end


function print_harmonic_diagnostics(points)
fprintf('\nDetailed harmonic diagnostics\n');
fprintf(['Diagnostic flags are heuristic indications, ' ...
    'not proven root causes.\n']);
for k = 1:numel(points)
    point = points(k);
    fprintf('\nLoad %.6g A harmonic diagnostics:\n', ...
        point.target_current_a);
    fprintf('  Target current: %.9g A\n', point.target_current_a);
    fprintf('  Measured Io: %.9g A\n', point.io_phys_rms_a);
    fprintf('  THD: %.9g %%\n', point.uo_phys_thd_percent);
    fprintf('  H3/H5/H7/H9: %.9g/%.9g/%.9g/%.9g %%\n', ...
        point.h3_percent, point.h5_percent, point.h7_percent, ...
        point.h9_percent);
    fprintf('  Low-order odd/even: %.9g/%.9g %%\n', ...
        point.low_order_odd_thd_percent, ...
        point.low_order_even_thd_percent);
    fprintf('  DC offset: %.9g V\n', point.uo_dc_offset_v);
    fprintf('  Peak asymmetry: %.9g %%\n', ...
        point.peak_asymmetry_percent);
    fprintf('  Half-cycle RMS asymmetry: %.9g %%\n', ...
        point.half_cycle_rms_asymmetry_percent);
    fprintf('  Expected capacitor current: %.9g A\n', ...
        point.capacitor_current_expected_rms_a);
    fprintf('  Measured IL: %.9g A\n', point.il_phys_rms_a);
    fprintf('  IL excess: %.9g A\n', ...
        point.inductor_current_excess_rms_a);
    fprintf(['  Dominant harmonics: H%.0f %.9g %%, H%.0f %.9g %%, ' ...
        'H%.0f %.9g %%\n'], ...
        point.dominant_harmonic_order_1, ...
        point.dominant_harmonic_percent_1, ...
        point.dominant_harmonic_order_2, ...
        point.dominant_harmonic_percent_2, ...
        point.dominant_harmonic_order_3, ...
        point.dominant_harmonic_percent_3);

    flags = point.diagnostic_flags;
    flag_names = fieldnames(flags);
    active_flags = flag_names(structfun(@(value) logical(value), flags));
    if isempty(active_flags)
        indications = 'none';
    else
        indications = strjoin(active_flags, ', ');
    end
    fprintf('  Diagnostic indications: %s\n', indications);
end
end


function print_failures(points, overall_checks, endpoint_regulation, ...
        span_regulation)
for k = 1:numel(points)
    if ~points(k).passed
        fprintf(2, 'Failed load point %.6g A: %s\n', ...
            points(k).target_current_a, ...
            strjoin(points(k).failed_checks, ', '));
    end
end
overall_failed = failed_check_names(overall_checks);
if ~isempty(overall_failed)
    fprintf(2, 'Failed overall checks: %s\n', ...
        strjoin(overall_failed, ', '));
end
fprintf(2, 'Endpoint load regulation: %.9f %%\n', ...
    endpoint_regulation);
fprintf(2, 'Span load regulation: %.9f %%\n', span_regulation);
end


function value = logical_text(flag)
if flag
    value = 'true (approximate no-load)';
else
    value = 'false';
end
end


function value = pass_text(flag)
if flag
    value = 'PASS';
else
    value = 'FAIL';
end
end


function names = failed_check_names(checks)
fields = fieldnames(checks);
failed = ~structfun(@(value) logical(value), checks);
names = fields(failed);
end

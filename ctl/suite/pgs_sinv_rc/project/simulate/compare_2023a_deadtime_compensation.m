function comparison = compare_2023a_deadtime_compensation()
%COMPARE_2023A_DEADTIME_COMPENSATION Compare saved dead-time A/B results.
%
% This function never starts a simulation or writes files. It reads the
% dbcomp_off baseline and dbcomp_on candidate JSON/CSV artifacts already
% saved by run_2023a_load_regulation_validation.

root = fileparts(mfilename('fullpath'));
validation_dir = fullfile(root, 'validation');
baseline = load_saved_dataset(validation_dir, 'dbcomp_off');
candidate = load_saved_dataset(validation_dir, 'dbcomp_on');

assert_matching_load_points(baseline.points, candidate.points);
target_voltage_rms_v = require_matching_target_voltage( ...
    baseline.metrics, candidate.metrics);

point_specs = {
    'uo_rms_v', 'Uo RMS', 'uo_phys_rms_v', 'target'
    'thd_percent', 'THD', 'uo_phys_thd_percent', 'lower'
    'h5_percent', 'H5', 'h5_percent', 'lower'
    'h7_percent', 'H7', 'h7_percent', 'lower'
    'h9_percent', 'H9', 'h9_percent', 'lower'
    'low_order_odd_percent', 'Low-order odd', ...
        'low_order_odd_thd_percent', 'lower'
    'low_order_even_percent', 'Low-order even', ...
        'low_order_even_thd_percent', 'lower'
    'dc_offset_v', 'DC offset', 'uo_dc_offset_v', 'absolute_lower'
    'peak_asymmetry_percent', 'Peak asymmetry', ...
        'peak_asymmetry_percent', 'lower'
    'half_cycle_rms_asymmetry_percent', ...
        'Half-cycle RMS asymmetry', ...
        'half_cycle_rms_asymmetry_percent', 'lower'
    'current_ref_peak_a', 'Current-ref peak', ...
        'current_ref_peak_a', 'lower'
    'saturation_fraction', 'Saturation', ...
        'current_ref_saturation_fraction', 'lower'
    'point_pass', 'Point PASS', 'passed', 'boolean'
    };

point_count = numel(baseline.points);
point_template = struct( ...
    'target_current_a', NaN, ...
    'baseline', struct(), ...
    'candidate', struct(), ...
    'delta', struct(), ...
    'relative_improvement_percent', struct());
point_results = repmat(point_template, point_count, 1);
for point_index = 1:point_count
    point_results(point_index).target_current_a = ...
        baseline.points(point_index).target_current_a;
    for metric_index = 1:size(point_specs, 1)
        output_name = point_specs{metric_index, 1};
        source_name = point_specs{metric_index, 3};
        direction = point_specs{metric_index, 4};
        baseline_value = numeric_scalar( ...
            baseline.points(point_index), source_name);
        candidate_value = numeric_scalar( ...
            candidate.points(point_index), source_name);
        point_results(point_index).baseline.(output_name) = baseline_value;
        point_results(point_index).candidate.(output_name) = candidate_value;
        point_results(point_index).delta.(output_name) = ...
            candidate_value - baseline_value;
        point_results(point_index).relative_improvement_percent.(output_name) = ...
            relative_improvement(baseline_value, candidate_value, ...
                direction, target_voltage_rms_v);
    end
end

overall_specs = {
    'endpoint_load_regulation_percent', 'Endpoint load regulation', ...
        'endpoint_load_regulation_percent'
    'span_load_regulation_percent', 'Span load regulation', ...
        'span_load_regulation_percent'
    };
overall = struct('baseline', struct(), 'candidate', struct(), ...
    'delta', struct(), 'relative_improvement_percent', struct());
for metric_index = 1:size(overall_specs, 1)
    output_name = overall_specs{metric_index, 1};
    source_name = overall_specs{metric_index, 3};
    baseline_value = numeric_scalar(baseline.metrics, source_name);
    candidate_value = numeric_scalar(candidate.metrics, source_name);
    overall.baseline.(output_name) = baseline_value;
    overall.candidate.(output_name) = candidate_value;
    overall.delta.(output_name) = candidate_value - baseline_value;
    overall.relative_improvement_percent.(output_name) = ...
        relative_improvement(baseline_value, candidate_value, ...
            'lower', target_voltage_rms_v);
end

thd_specs = {
    'no_load_thd_percent', 'No-load THD', 0.0
    'half_amp_thd_percent', '0.5 A THD', 0.5
    'full_load_thd_percent', 'Full-load THD', ...
        max([baseline.points.target_current_a])
    };
for metric_index = 1:size(thd_specs, 1)
    output_name = thd_specs{metric_index, 1};
    target_current_a = thd_specs{metric_index, 3};
    baseline_point = point_at_current(baseline.points, target_current_a);
    candidate_point = point_at_current(candidate.points, target_current_a);
    baseline_value = numeric_scalar(baseline_point, ...
        'uo_phys_thd_percent');
    candidate_value = numeric_scalar(candidate_point, ...
        'uo_phys_thd_percent');
    overall.baseline.(output_name) = baseline_value;
    overall.candidate.(output_name) = candidate_value;
    overall.delta.(output_name) = candidate_value - baseline_value;
    overall.relative_improvement_percent.(output_name) = ...
        relative_improvement(baseline_value, candidate_value, ...
            'lower', target_voltage_rms_v);
end

comparison = struct( ...
    'baseline_label', 'dbcomp_off', ...
    'candidate_label', 'dbcomp_on', ...
    'validation_directory', validation_dir, ...
    'point_results', point_results, ...
    'overall', overall);

print_point_comparison(point_results, point_specs);
print_overall_comparison(overall, overall_specs, thd_specs);
end


function dataset = load_saved_dataset(validation_dir, suffix)
json_file = fullfile(validation_dir, sprintf( ...
    '2023a_load_regulation_metrics_%s.json', suffix));
points_file = fullfile(validation_dir, sprintf( ...
    '2023a_load_regulation_points_%s.csv', suffix));
harmonics_file = fullfile(validation_dir, sprintf( ...
    '2023a_load_regulation_harmonics_%s.csv', suffix));
required_files = {json_file, points_file, harmonics_file};
missing = required_files(~cellfun(@isfile, required_files));
if ~isempty(missing)
    error('SINV:R2023A:DeadtimeComparisonDatasetMissing', ...
        'The %s dataset is incomplete. Missing: %s', ...
        suffix, strjoin(missing, ', '));
end

try
    metrics = jsondecode(fileread(json_file));
catch ME
    error('SINV:R2023A:DeadtimeComparisonJSONInvalid', ...
        'Could not parse %s: %s', json_file, ME.message);
end
if ~isstruct(metrics) || ~isfield(metrics, 'point_results') || ...
        ~isstruct(metrics.point_results) || isempty(metrics.point_results)
    error('SINV:R2023A:DeadtimeComparisonJSONSchemaInvalid', ...
        'Metrics JSON has no usable point_results array: %s', json_file);
end
points = metrics.point_results(:);
[~, point_order] = sort([points.target_current_a]);
points = points(point_order);

points_table = readtable(points_file, 'VariableNamingRule', 'preserve');
harmonics_table = readtable(harmonics_file, ...
    'VariableNamingRule', 'preserve');
harmonics_table = normalize_harmonics_table_columns( ...
    harmonics_table, harmonics_file);
validate_points_csv(points, points_table, points_file);
validate_harmonics_csv(points, harmonics_table, harmonics_file);

dataset = struct('metrics', metrics, 'points', points);
end


function table_data = normalize_harmonics_table_columns( ...
        table_data, filename)
column_aliases = {
    'TargetCurrentA', ...
        {'TargetCurrentA', 'target_current_a'}
    'MeasuredIoRmsA', ...
        {'MeasuredIoRmsA', 'measured_io_rms_a'}
    'LoadResistanceOhm', ...
        {'LoadResistanceOhm', 'load_resistance_ohm'}
    'HarmonicOrder', ...
        {'HarmonicOrder', 'harmonic_order'}
    'HarmonicPeakV', ...
        {'HarmonicPeakV', 'harmonic_peak_v'}
    'HarmonicPercentOfFundamental', ...
        {'HarmonicPercentOfFundamental', ...
         'harmonic_percent_of_fundamental'}
    };

resolved_names = cell(size(column_aliases, 1), 1);
for column_index = 1:size(column_aliases, 1)
    canonical_name = column_aliases{column_index, 1};
    candidates = column_aliases{column_index, 2};
    resolved_names{column_index} = resolve_table_column( ...
        table_data, candidates, canonical_name, filename);
end

variable_names = table_data.Properties.VariableNames;
for column_index = 1:size(column_aliases, 1)
    canonical_name = column_aliases{column_index, 1};
    source_name = resolved_names{column_index};
    if ~strcmp(source_name, canonical_name)
        source_index = find(strcmp(variable_names, source_name));
        variable_names{source_index} = canonical_name;
    end
end
table_data.Properties.VariableNames = variable_names;
end


function column_name = resolve_table_column( ...
        table_data, candidates, canonical_name, filename)
available_columns = table_data.Properties.VariableNames;
matched_candidates = candidates(ismember(candidates, available_columns));
if isempty(matched_candidates)
    error('SINV:R2023A:DeadtimeComparisonCSVColumnMissing', ...
        ['Harmonics CSV %s has no exact alias for %s. Expected one of: ' ...
         '%s. Available columns: %s'], ...
        filename, canonical_name, strjoin(candidates, ', '), ...
        format_available_columns(available_columns));
end
if numel(matched_candidates) > 1
    error('SINV:R2023A:DeadtimeComparisonCSVColumnAmbiguous', ...
        ['Harmonics CSV %s has ambiguous aliases for %s: %s. ' ...
         'Available columns: %s'], ...
        filename, canonical_name, strjoin(matched_candidates, ', '), ...
        format_available_columns(available_columns));
end
column_name = matched_candidates{1};
end


function formatted = format_available_columns(available_columns)
if isempty(available_columns)
    formatted = '<none>';
else
    formatted = strjoin(available_columns, ', ');
end
end


function validate_points_csv(points, csv_data, filename)
required_columns = {'TargetCurrentA', 'UoPhysRmsV', ...
    'UoPhysThdPercent', 'CurrentRefPeakA', ...
    'CurrentRefSaturationFraction', 'PointPassed'};
assert_table_columns(csv_data, required_columns, filename);
csv_data = sortrows(csv_data, 'TargetCurrentA');
if height(csv_data) ~= numel(points)
    error('SINV:R2023A:DeadtimeComparisonPointsCSVCountMismatch', ...
        'JSON/points CSV row counts disagree: %s', filename);
end
assert_close([points.target_current_a]', csv_data.TargetCurrentA, ...
    'target current', filename);
assert_close([points.uo_phys_rms_v]', csv_data.UoPhysRmsV, ...
    'Uo RMS', filename);
assert_close([points.uo_phys_thd_percent]', csv_data.UoPhysThdPercent, ...
    'THD', filename);
assert_close([points.current_ref_peak_a]', csv_data.CurrentRefPeakA, ...
    'current-ref peak', filename);
assert_close([points.current_ref_saturation_fraction]', ...
    csv_data.CurrentRefSaturationFraction, 'saturation', filename);
assert_close(double([points.passed]'), double(csv_data.PointPassed), ...
    'point PASS', filename);
end


function validate_harmonics_csv(points, csv_data, filename)
required_columns = {'TargetCurrentA', 'HarmonicOrder', ...
    'HarmonicPercentOfFundamental'};
assert_table_columns(csv_data, required_columns, filename);
for point_index = 1:numel(points)
    target_current_a = points(point_index).target_current_a;
    for harmonic_order = [5, 7, 9]
        row = abs(csv_data.TargetCurrentA - target_current_a) <= 1e-12 & ...
            csv_data.HarmonicOrder == harmonic_order;
        if nnz(row) ~= 1
            error('SINV:R2023A:DeadtimeComparisonHarmonicRowMissing', ...
                ['Expected one %.6g A H%d row in harmonics CSV: %s'], ...
                target_current_a, harmonic_order, filename);
        end
        json_field = sprintf('h%d_percent', harmonic_order);
        assert_close(points(point_index).(json_field), ...
            csv_data.HarmonicPercentOfFundamental(row), ...
            sprintf('H%d', harmonic_order), filename);
    end
end
end


function assert_table_columns(table_data, required_columns, filename)
missing = setdiff(required_columns, table_data.Properties.VariableNames, ...
    'stable');
if ~isempty(missing)
    error('SINV:R2023A:DeadtimeComparisonCSVSchemaInvalid', ...
        'CSV is missing columns %s: %s', strjoin(missing, ', '), filename);
end
end


function assert_close(json_value, csv_value, label, filename)
json_value = double(json_value(:));
csv_value = double(csv_value(:));
tolerance = 1e-10 * max(1, max(abs([json_value; csv_value])));
if numel(json_value) ~= numel(csv_value) || ...
        any(~isfinite(json_value)) || any(~isfinite(csv_value)) || ...
        any(abs(json_value - csv_value) > tolerance)
    error('SINV:R2023A:DeadtimeComparisonJSONCSVDisagree', ...
        'JSON and CSV disagree for %s: %s', label, filename);
end
end


function assert_matching_load_points(baseline, candidate)
baseline_current = [baseline.target_current_a];
candidate_current = [candidate.target_current_a];
if numel(baseline_current) ~= numel(candidate_current) || ...
        any(abs(baseline_current - candidate_current) > 1e-12)
    error('SINV:R2023A:DeadtimeComparisonLoadPointsMismatch', ...
        'dbcomp_off and dbcomp_on load points do not match.');
end
end


function target_voltage = require_matching_target_voltage( ...
        baseline_metrics, candidate_metrics)
target_voltage = numeric_scalar(baseline_metrics, ...
    'target_voltage_rms_v');
candidate_target = numeric_scalar(candidate_metrics, ...
    'target_voltage_rms_v');
if abs(target_voltage - candidate_target) > 1e-12
    error('SINV:R2023A:DeadtimeComparisonVoltageTargetMismatch', ...
        'dbcomp_off and dbcomp_on voltage targets do not match.');
end
end


function point = point_at_current(points, target_current_a)
index = find(abs([points.target_current_a] - target_current_a) <= 1e-12);
if numel(index) ~= 1
    error('SINV:R2023A:DeadtimeComparisonLoadPointMissing', ...
        'Expected exactly one %.6g A load point.', target_current_a);
end
point = points(index);
end


function value = numeric_scalar(data, field_name)
if ~isfield(data, field_name)
    error('SINV:R2023A:DeadtimeComparisonMetricMissing', ...
        'Required metric is missing: %s', field_name);
end
value = double(data.(field_name));
if ~isscalar(value) || ~isfinite(value)
    error('SINV:R2023A:DeadtimeComparisonMetricInvalid', ...
        'Metric must be a finite scalar: %s', field_name);
end
end


function improvement = relative_improvement( ...
        baseline, candidate, direction, target)
switch direction
    case 'lower'
        improvement_amount = baseline - candidate;
        reference = abs(baseline);
    case 'absolute_lower'
        improvement_amount = abs(baseline) - abs(candidate);
        reference = abs(baseline);
    case 'target'
        baseline_error = abs(baseline - target);
        candidate_error = abs(candidate - target);
        improvement_amount = baseline_error - candidate_error;
        reference = baseline_error;
    case 'boolean'
        improvement = 100 * (candidate - baseline);
        return;
    otherwise
        error('SINV:R2023A:DeadtimeComparisonDirectionInvalid', ...
            'Unsupported improvement direction: %s', direction);
end
if reference <= eps(max(1, abs(baseline)))
    if abs(improvement_amount) <= eps(max(1, abs(candidate)))
        improvement = 0.0;
    else
        improvement = NaN;
    end
else
    improvement = 100 * improvement_amount / reference;
end
end


function print_point_comparison(points, specs)
fprintf('\n2023A dead-time compensation A/B comparison\n');
fprintf('Candidate - baseline; positive improvement means better.\n');
fprintf('%6s  %-29s %12s %12s %12s %11s\n', ...
    'Load', 'Metric', 'Off', 'On', 'Delta', 'Improve %');
for point_index = 1:numel(points)
    for metric_index = 1:size(specs, 1)
        field_name = specs{metric_index, 1};
        fprintf('%5.1fA  %-29s %12.6g %12.6g %12.6g %11.5g\n', ...
            points(point_index).target_current_a, specs{metric_index, 2}, ...
            points(point_index).baseline.(field_name), ...
            points(point_index).candidate.(field_name), ...
            points(point_index).delta.(field_name), ...
            points(point_index).relative_improvement_percent.(field_name));
    end
end
end


function print_overall_comparison(overall, overall_specs, thd_specs)
fprintf('\nOverall metrics\n');
fprintf('%-29s %12s %12s %12s %11s\n', ...
    'Metric', 'Off', 'On', 'Delta', 'Improve %');
all_specs = [overall_specs(:, 1:2); thd_specs(:, 1:2)];
for metric_index = 1:size(all_specs, 1)
    field_name = all_specs{metric_index, 1};
    fprintf('%-29s %12.6g %12.6g %12.6g %11.5g\n', ...
        all_specs{metric_index, 2}, ...
        overall.baseline.(field_name), ...
        overall.candidate.(field_name), ...
        overall.delta.(field_name), ...
        overall.relative_improvement_percent.(field_name));
end
end

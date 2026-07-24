function stats = analyze_bl_result(resultInput, varargin)
%ANALYZE_BL_RESULT Analyze one common 16-channel GFL SIL result.
%
% The monitor layout is:
%   Ia Ib Ic I0 Id Iq Id_ref Iq_ref Vd_grid Vq_grid Vd_cmd Vq_cmd
%   PLL_error PLL_frequency level_specific_0 level_specific_1
%
% Level-specific channels:
%   BL0 angle/PLL-lock, BL1 angle/system-enable,
%   BL2 PLL-lock/system-enable, BL3 negative Id/Iq,
%   BL4 external/decoupling feed-forward d, BL5 measured P/Q.

    p = inputParser;
    addParameter(p, 'Thresholds', bl_validation_thresholds(), @isstruct);
    addParameter(p, 'OutputDirectory', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'CaseName', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'WritePlot', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
    parse(p, varargin{:});
    thresholds = p.Results.Thresholds;

    if ischar(resultInput) || isstring(resultInput)
        loaded = load(char(resultInput), 'result');
        result = loaded.result;
    else
        result = resultInput;
    end
    assert(isstruct(result), 'Input must be a result struct or MAT file.');

    scenario = field_or(result, 'Scenario', sprintf('BL%d', result.BuildLevel));
    stats = struct( ...
        'Status', result.Status, ...
        'Scenario', char(string(scenario)), ...
        'BuildLevel', double(result.BuildLevel), ...
        'StopTime', double(result.StopTime), ...
        'Pass', false, ...
        'FailureReason', '');
    if ~strcmp(result.Status, 'ok')
        stats.FailureReason = field_or(result, 'ErrorMessage', 'Simulation failed.');
        return
    end

    [monitor, monitorTime] = get_result_signal(result, 'gfl_monitor');
    [enable, ~] = get_result_signal(result, 'gfl_enable');
    [pwm, pwmTime] = get_result_signal(result, 'gfl_pwm');
    values = normalize_columns(monitor, 16);
    enable = normalize_columns(enable, []);
    pwm = normalize_columns(pwm, []);

    n = size(values, 1);
    tailStart = max(1, floor((1 - thresholds.TailFraction) * n));
    tail = tailStart:n;
    currentSum = sum(values(:, 1:3), 2);
    idError = values(:, 7) - values(:, 5);
    iqError = values(:, 8) - values(:, 6);

    stats.SampleCount = n;
    stats.IdMean = mean(values(tail, 5), 'omitnan');
    stats.IqMean = mean(values(tail, 6), 'omitnan');
    stats.IdRefMean = mean(values(tail, 7), 'omitnan');
    stats.IqRefMean = mean(values(tail, 8), 'omitnan');
    stats.CurrentTrackingRms = sqrt(mean(idError(tail).^2 + iqError(tail).^2, 'omitnan'));
    stats.PLLFrequencyMean = mean(values(tail, 14), 'omitnan');
    stats.PLLFrequencyErrorMax = max(abs(values(tail, 14) - 1), [], 'omitnan');
    stats.PLLVqMean = mean(values(tail, 10), 'omitnan');
    stats.PLLVqAbsMean = mean(abs(values(tail, 10)), 'omitnan');
    stats.CurrentPeak = max(abs(values(:, 1:3)), [], 'all', 'omitnan');
    stats.ThreeWireResidualPeak = max(abs(currentSum), [], 'omitnan');
    stats.ThreeWireResidualRms = sqrt(mean(currentSum.^2, 'omitnan'));
    stats.PWMEnableMean = mean(double(enable(:)), 'omitnan');
    stats.PWMFinite = all(isfinite(double(pwm(:))));
    [stats.PWMMin, stats.PWMMax, stats.PWMBoundaryFraction] = ...
        pwm_metrics(pwm, enable, thresholds.PWMCompareMax);
    stats.Diverged = any(~isfinite(values), 'all') || ...
        stats.CurrentPeak > thresholds.DivergenceCurrentPeak;
    stats.LevelSpecificTail0 = mean(values(tail, 15), 'omitnan');
    stats.LevelSpecificTail1 = mean(values(tail, 16), 'omitnan');
    stats.NegativeSequenceCurrentRms = NaN;
    stats.PMeasuredMean = NaN;
    stats.QMeasuredMean = NaN;
    stats.PReferenceMean = NaN;
    stats.QReferenceMean = NaN;
    stats.PTrackingError = NaN;
    stats.QTrackingError = NaN;
    stats.NegativeSequenceImprovement = NaN;
    stats.NegativeSequenceControllerRms = NaN;

    if result.BuildLevel == 3
        frequencyHz = 50;
        if isfield(result, 'PlantConfig') && isfield(result.PlantConfig, 'GridFrequencyHz')
            frequencyHz = result.PlantConfig.GridFrequencyHz;
        end
        stats.NegativeSequenceCurrentRms = fundamental_negative_sequence_rms( ...
            monitorTime(tail), values(tail, 1:3), frequencyHz);
        stats.NegativeSequenceControllerRms = ...
            sqrt(mean(values(tail, 15).^2 + values(tail, 16).^2, 'omitnan'));
    elseif result.BuildLevel == 5
        stats.PMeasuredMean = mean(values(tail, 15), 'omitnan');
        stats.QMeasuredMean = mean(values(tail, 16), 'omitnan');
        pProfile = field_or(result, 'PRefProfile', zeros(0, 2));
        qProfile = field_or(result, 'QRefProfile', zeros(0, 2));
        pRef = evaluate_profile(monitorTime, pProfile);
        qRef = evaluate_profile(monitorTime, qProfile);
        stats.PReferenceMean = mean(pRef(tail), 'omitnan');
        stats.QReferenceMean = mean(qRef(tail), 'omitnan');
        stats.PTrackingError = abs(stats.PMeasuredMean - stats.PReferenceMean);
        stats.QTrackingError = abs(stats.QMeasuredMean - stats.QReferenceMean);
    end

    stats.FeatureFlags = field_or(result, 'FeatureFlags', struct());
    stats.PlantConfig = field_or(result, 'PlantConfig', struct());
    stats.MonitorLayout = [ ...
        'Ia Ib Ic I0 Id Iq Id_ref Iq_ref Vd_grid Vq_grid Vd_cmd Vq_cmd ' ...
        'PLL_error PLL_frequency level_specific_0 level_specific_1'];

    failures = {};
    if stats.Diverged
        failures{end + 1} = 'non-finite signal or divergent current'; %#ok<AGROW>
    end
    if ~stats.PWMFinite
        failures{end + 1} = 'non-finite PWM'; %#ok<AGROW>
    end
    if stats.CurrentPeak > thresholds.CurrentPeakMax
        failures{end + 1} = sprintf('current peak %.4g > %.4g', ...
            stats.CurrentPeak, thresholds.CurrentPeakMax); %#ok<AGROW>
    end
    if stats.ThreeWireResidualRms > thresholds.ThreeWireResidualRmsMax
        failures{end + 1} = sprintf('three-wire residual RMS %.4g > %.4g', ...
            stats.ThreeWireResidualRms, thresholds.ThreeWireResidualRmsMax); %#ok<AGROW>
    end
    if result.BuildLevel ~= 1 && plant_grid_connected(stats.PlantConfig)
        if stats.PLLFrequencyErrorMax > thresholds.PLLFrequencyErrorMax
            failures{end + 1} = sprintf('PLL frequency error %.4g > %.4g', ...
                stats.PLLFrequencyErrorMax, thresholds.PLLFrequencyErrorMax); %#ok<AGROW>
        end
        if stats.PLLVqAbsMean > thresholds.PLLVqAbsMeanMax
            failures{end + 1} = sprintf('|Vq| mean %.4g > %.4g', ...
                stats.PLLVqAbsMean, thresholds.PLLVqAbsMeanMax); %#ok<AGROW>
        end
    end
    if result.BuildLevel == 0
        if stats.PWMEnableMean > thresholds.PWMDisabledMeanMax
            failures{end + 1} = 'BL0 PWM was not hard-disabled'; %#ok<AGROW>
        end
    else
        if stats.PWMBoundaryFraction > thresholds.PWMBoundaryFractionMax
            failures{end + 1} = sprintf('PWM boundary fraction %.4g > %.4g', ...
                stats.PWMBoundaryFraction, thresholds.PWMBoundaryFractionMax); %#ok<AGROW>
        end
    end
    if result.BuildLevel == 1
        if plant_grid_connected(stats.PlantConfig)
            failures{end + 1} = 'BL1 requires the independent grid breaker open'; %#ok<AGROW>
        end
    elseif result.BuildLevel >= 2 && result.BuildLevel <= 4
        if stats.CurrentTrackingRms > thresholds.CurrentTrackingRmsMax
            failures{end + 1} = sprintf('dq tracking RMS %.4g > %.4g', ...
                stats.CurrentTrackingRms, thresholds.CurrentTrackingRmsMax); %#ok<AGROW>
        end
    elseif result.BuildLevel == 5
        if stats.PTrackingError > thresholds.PowerPErrorMax
            failures{end + 1} = sprintf('P error %.4g > %.4g', ...
                stats.PTrackingError, thresholds.PowerPErrorMax); %#ok<AGROW>
        end
        if stats.QTrackingError > thresholds.PowerQErrorMax
            failures{end + 1} = sprintf('Q error %.4g > %.4g', ...
                stats.QTrackingError, thresholds.PowerQErrorMax); %#ok<AGROW>
        end
    end

    stats.Pass = isempty(failures);
    stats.FailureReason = strjoin(failures, '; ');

    outputDirectory = char(string(p.Results.OutputDirectory));
    if logical(p.Results.WritePlot) && ~isempty(outputDirectory)
        if ~isfolder(outputDirectory)
            mkdir(outputDirectory);
        end
        caseName = char(string(p.Results.CaseName));
        if isempty(caseName)
            caseName = char(string(scenario));
        end
        stats.PlotFile = fullfile(outputDirectory, [safe_name(caseName) '.png']);
        write_plot(stats.PlotFile, monitorTime, values, pwmTime, pwm, enable, stats);
    else
        stats.PlotFile = '';
    end
end

function [minimum, maximum, boundaryFraction] = pwm_metrics(pwm, enable, compareMax)
    activePwm = double(pwm(:, 1:min(3, size(pwm, 2))));
    enabled = double(enable(:, 1)) > 0;
    sampleCount = min(size(activePwm, 1), numel(enabled));
    activePwm = activePwm(1:sampleCount, :);
    enabled = enabled(1:sampleCount);
    minimum = 0;
    maximum = 0;
    boundaryFraction = 0;
    if any(enabled)
        enabledPwm = activePwm(enabled, :);
        minimum = min(enabledPwm, [], 'all');
        maximum = max(enabledPwm, [], 'all');
        boundaryFraction = mean(any(enabledPwm <= 0 | enabledPwm >= compareMax, 2));
    end
end

function value = field_or(s, field, fallback)
    if isfield(s, field)
        value = s.(field);
    else
        value = fallback;
    end
end

function tf = plant_grid_connected(plant)
    tf = true;
    if isstruct(plant) && isfield(plant, 'GridConnected')
        tf = logical(plant.GridConnected);
    end
end

function values = evaluate_profile(time, profile)
    time = double(time(:));
    profile = double(profile);
    values = zeros(size(time));
    if isempty(profile)
        return
    end
    for k = 1:size(profile, 1)
        values(time >= profile(k, 1)) = profile(k, 2);
    end
end

function values = normalize_columns(values, expectedColumns)
    values = double(values);
    if ndims(values) > 2
        values = squeeze(values);
    end
    if isvector(values)
        values = values(:);
    end
    if ~isempty(expectedColumns) && size(values, 2) ~= expectedColumns && ...
            size(values, 1) == expectedColumns
        values = values.';
    end
    if ~isempty(expectedColumns)
        assert(size(values, 2) == expectedColumns, ...
            'Expected %d columns, received %d.', expectedColumns, size(values, 2));
    end
end

function [signal, time] = get_signal(logsout, name)
    assert(~isempty(logsout), 'logsout is empty; enable line logging before sim().');
    element = logsout.get(name);
    assert(~isempty(element), 'Signal %s is not present in logsout.', name);
    values = element.Values;
    if isstruct(values)
        fields = fieldnames(values);
        data = cellfun(@(field) values.(field).Data(:), fields, 'UniformOutput', false);
        signal = [data{:}];
        time = values.(fields{1}).Time(:);
    else
        signal = values.Data;
        time = values.Time(:);
    end
end

function [signal, time] = get_result_signal(result, name)
    if isfield(result, 'Signals') && isfield(result.Signals, name)
        item = result.Signals.(name);
        signal = item.Data;
        time = item.Time;
    else
        assert(isfield(result, 'Logsout'), ...
            'Result contains neither compact Signals nor legacy Logsout.');
        [signal, time] = get_signal(result.Logsout, name);
    end
end

function rmsValue = fundamental_negative_sequence_rms(time, currents, frequencyHz)
    time = double(time(:));
    currents = double(currents);
    if numel(time) < 8 || size(currents, 2) ~= 3
        rmsValue = NaN;
        return
    end
    theta = 2 * pi * frequencyHz * time;
    design = [cos(theta) sin(theta) ones(size(theta))];
    coefficients = design \ currents;
    phasePhasors = coefficients(1, :) - 1i * coefficients(2, :);
    rotation = exp(1i * 2 * pi / 3);
    negativePhasor = (phasePhasors(1) + rotation^2 * phasePhasors(2) + ...
        rotation * phasePhasors(3)) / 3;
    rmsValue = abs(negativePhasor) / sqrt(2);
end

function write_plot(fileName, monitorTime, values, pwmTime, pwm, enable, stats)
    figureHandle = figure('Visible', 'off', 'Color', 'white', 'Position', [100 100 1200 850]);
    cleanup = onCleanup(@() close(figureHandle));
    tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(monitorTime, values(:, 5:8), 'LineWidth', 1);
    grid on;
    ylabel('dq current (pu)');
    legend('Id', 'Iq', 'Id*', 'Iq*', 'Location', 'best');
    title(sprintf('%s — BL%d — %s', stats.Scenario, stats.BuildLevel, pass_word(stats.Pass)), ...
        'Interpreter', 'none');

    nexttile;
    yyaxis left;
    plot(monitorTime, values(:, 14), 'LineWidth', 1);
    ylabel('PLL freq (pu)');
    yyaxis right;
    plot(monitorTime, values(:, 10), 'LineWidth', 1);
    ylabel('Vq (pu)');
    grid on;

    nexttile;
    plot(monitorTime, values(:, 1:3), 'LineWidth', 0.9);
    grid on;
    ylabel('phase current (pu)');
    legend('Ia', 'Ib', 'Ic', 'Location', 'best');

    nexttile;
    sampleCount = min([numel(pwmTime), size(pwm, 1), size(enable, 1)]);
    plot(pwmTime(1:sampleCount), pwm(1:sampleCount, 1:min(3, size(pwm, 2))), ...
        'LineWidth', 0.8);
    hold on;
    plot(pwmTime(1:sampleCount), double(enable(1:sampleCount, 1)) * 3000, ...
        'k--', 'LineWidth', 0.8);
    grid on;
    ylabel('PWM compare');
    xlabel('time (s)');

    exportgraphics(figureHandle, fileName, 'Resolution', 140);
    clear cleanup
end

function value = pass_word(pass)
    if pass
        value = 'PASS';
    else
        value = 'FAIL';
    end
end

function value = safe_name(value)
    value = regexprep(char(string(value)), '[^A-Za-z0-9_.-]+', '_');
end

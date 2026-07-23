function stats = analyze_bl_result(resultInput)
%ANALYZE_BL_RESULT Summarize the common 16-channel GFL SIL monitor.
%
% The monitor layout is:
%   Ia Ib Ic I0 Id Iq Id_ref Iq_ref Vd_grid Vq_grid Vd_cmd Vq_cmd
%   PLL_error PLL_frequency level_specific_0 level_specific_1
%
% For BL0 the last two channels are angle/PLL-lock; BL1 uses
% angle/system-enable.  BL2 uses PLL-lock/system-enable; BL3 uses negative
% Id/Iq; BL4 uses external
% and decoupling feed-forward d terms; BL5 uses measured P/Q.

    if ischar(resultInput) || isstring(resultInput)
        loaded = load(char(resultInput), 'result');
        result = loaded.result;
    else
        result = resultInput;
    end
    assert(isstruct(result), 'Input must be a result struct or MAT file.');
    stats = struct('Status', result.Status, 'BuildLevel', result.BuildLevel, ...
        'StopTime', result.StopTime);
    if ~strcmp(result.Status, 'ok')
        stats.Message = result.ErrorMessage;
        disp(stats);
        return;
    end

    logsout = result.Logsout;
    monitor = get_signal(logsout, 'gfl_monitor');
    enable = get_signal(logsout, 'gfl_enable');
    pwm = get_signal(logsout, 'gfl_pwm');
    values = double(monitor);
    if ndims(values) > 2
        values = squeeze(values);
    end
    if size(values, 2) ~= 16 && size(values, 1) == 16
        values = values.';
    end
    values = reshape(values, size(values, 1), []);
    n = size(values, 1);
    tail = max(1, floor(0.8 * n)):n;
    stats.SampleCount = n;
    stats.IdMean = mean(values(tail, 5), 'omitnan');
    stats.IqMean = mean(values(tail, 6), 'omitnan');
    stats.IdRefMean = mean(values(tail, 7), 'omitnan');
    stats.IqRefMean = mean(values(tail, 8), 'omitnan');
    stats.PLLFrequencyMean = mean(values(tail, 14), 'omitnan');
    stats.PLLVqMean = mean(values(tail, 10), 'omitnan');
    stats.CurrentPeak = max(abs(values(:, 1:3)), [], 'all', 'omitnan');
    stats.ThreeWireResidualPeak = max(abs(values(:, 1) + values(:, 2) + values(:, 3)), [], ...
        'omitnan');
    stats.PWMEnableMean = mean(double(enable(:)), 'omitnan');
    stats.PWMFinite = all(isfinite(double(pwm(:))));
    activePwm = double(pwm(:, 1:min(3, size(pwm, 2))));
    enabled = double(enable(:)) > 0;
    sampleCount = min(size(activePwm, 1), numel(enabled));
    activePwm = activePwm(1:sampleCount, :);
    enabled = enabled(1:sampleCount);
    stats.PWMBoundaryFraction = 0;
    stats.PWMMin = 0;
    stats.PWMMax = 0;
    if any(enabled)
        enabledPwm = activePwm(enabled, :);
        stats.PWMMin = min(enabledPwm, [], 'all');
        stats.PWMMax = max(enabledPwm, [], 'all');
        % CTRL_PWM_CMP_MAX is 3000 in the generated PC target settings.
        stats.PWMBoundaryFraction = mean(any(enabledPwm <= 0 | enabledPwm >= 3000, 2));
    end
    stats.Diverged = any(~isfinite(values), 'all') || stats.CurrentPeak > 10;
    stats.LevelSpecificTail = mean(values(tail, 15:16), 1, 'omitnan');
    stats.MonitorLayout = 'Ia Ib Ic I0 Id Iq Id_ref Iq_ref Vd_grid Vq_grid Vd_cmd Vq_cmd PLL_error PLL_frequency level_specific_0 level_specific_1';
    disp(stats);
end

function signal = get_signal(logsout, name)
    assert(~isempty(logsout), 'logsout is empty; enable line logging before sim().');
    element = logsout.get(name);
    assert(~isempty(element), 'Signal %s is not present in logsout.', name);
    values = element.Values;
    if isstruct(values)
        fields = fieldnames(values);
        data = cellfun(@(field) values.(field).Data(:), fields, 'UniformOutput', false);
        signal = [data{:}];
    else
        signal = values.Data;
    end
end

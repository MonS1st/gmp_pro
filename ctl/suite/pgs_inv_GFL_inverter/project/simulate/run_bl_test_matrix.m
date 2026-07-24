function report = run_bl_test_matrix(varargin)
%RUN_BL_TEST_MATRIX Build and execute the BL0-BL5 SIL validation matrix.
%
% Artifacts are written to validation_results/<timestamp>/:
%   summary.csv, summary.md, matrix_results.mat, matrix_config.json,
%   models/, mat/, plots/, config/, and build/<case>/.

    p = inputParser;
    addParameter(p, 'OutputRoot', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'OriginalModel', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'ExistingDirectory', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'StopOnFailure', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
    addParameter(p, 'MSBuild', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});

    thisDir = fileparts(mfilename('fullpath'));
    repoRoot = fileparts(fileparts(fileparts(fileparts(fileparts(thisDir)))));
    existingDirectory = char(string(p.Results.ExistingDirectory));
    if ~isempty(existingDirectory)
        report = reanalyze_existing(existingDirectory);
        return
    end
    outputRoot = char(string(p.Results.OutputRoot));
    if isempty(outputRoot)
        outputRoot = fullfile(repoRoot, 'validation_results');
    end
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(outputRoot, timestamp);
    folders = struct( ...
        'Models', fullfile(outputDir, 'models'), ...
        'Mat', fullfile(outputDir, 'mat'), ...
        'Plots', fullfile(outputDir, 'plots'), ...
        'Config', fullfile(outputDir, 'config'), ...
        'Build', fullfile(outputDir, 'build'));
    mkdir(outputDir);
    names = fieldnames(folders);
    for i = 1:numel(names)
        mkdir(folders.(names{i}));
    end

    originalModel = char(string(p.Results.OriginalModel));
    if isempty(originalModel)
        originalModel = fullfile(thisDir, 'DP_STD_MDL_DCAC_3ph_2level_gridconn.slx');
    end
    validationModel = fullfile(folders.Models, ...
        'DP_STD_MDL_DCAC_3ph_2level_gridconn_validation.slx');
    configure_bl_validation_model(originalModel, validationModel, ...
        'BreakerSwitchTimes', 0.30, ...
        'DeltaSwitchTimes', 0.75, ...
        'Zab', 8, 'Zbc', 8, 'Zca', 8, 'ZbcAdditional', 2);

    thresholds = bl_validation_thresholds();
    cases = validation_cases();
    stats = repmat(empty_stats(), numel(cases), 1);
    builds = cell(numel(cases), 1);
    results = cell(numel(cases), 1);

    for i = 1:numel(cases)
        c = cases(i);
        fprintf('\n[%d/%d] %s\n', i, numel(cases), c.Name);
        caseBuildDir = fullfile(folders.Build, c.Name);
        resultFile = fullfile(folders.Mat, [c.Name '.mat']);
        configFile = fullfile(folders.Config, [c.Name '.json']);
        write_text(configFile, jsonencode(case_snapshot(c), PrettyPrint=true));
        try
            builds{i} = build_bl_controller(c.BuildLevel, caseBuildDir, ...
                'NegSequenceEnabled', c.NegSequenceEnabled, ...
                'ExternalFeedforwardEnabled', c.ExternalFeedforwardEnabled, ...
                'DecouplingEnabled', c.DecouplingEnabled, ...
                'ActiveDampingEnabled', c.ActiveDampingEnabled, ...
                'LeadCompensatorEnabled', c.LeadCompensatorEnabled, ...
                'PQProfileEnabled', c.PQProfileEnabled, ...
                'MSBuild', p.Results.MSBuild);

            ports = 14000 + (i - 1) * 10 + (0:3);
            results{i} = run_bl_simulation(c.BuildLevel, c.StopTime, ...
                'Scenario', c.Name, ...
                'ModelFile', validationModel, ...
                'Executable', builds{i}.Executable, ...
                'Ports', ports, ...
                'ResultFile', resultFile, ...
                'GridConnected', c.GridConnected, ...
                'GridBreakerCloseTime', c.GridBreakerCloseTime, ...
                'UnbalancedLoadEnabled', c.UnbalancedLoadEnabled, ...
                'UnbalanceSwitchTime', c.UnbalanceSwitchTime, ...
                'Zab', c.Zab, 'Zbc', c.Zbc, 'Zca', c.Zca, ...
                'ZbcAdditional', c.ZbcAdditional, ...
                'NegSequenceEnabled', c.NegSequenceEnabled, ...
                'ExternalFeedforwardEnabled', c.ExternalFeedforwardEnabled, ...
                'DecouplingEnabled', c.DecouplingEnabled, ...
                'ActiveDampingEnabled', c.ActiveDampingEnabled, ...
                'LeadCompensatorEnabled', c.LeadCompensatorEnabled, ...
                'PQProfileEnabled', c.PQProfileEnabled, ...
                'PRefProfile', c.PRefProfile, ...
                'QRefProfile', c.QRefProfile);
            stats(i) = analyze_bl_result(results{i}, ...
                'Thresholds', thresholds, ...
                'OutputDirectory', folders.Plots, ...
                'CaseName', c.Name, ...
                'WritePlot', true);
        catch err
            stats(i) = empty_stats();
            stats(i).Status = 'error';
            stats(i).Scenario = c.Name;
            stats(i).BuildLevel = c.BuildLevel;
            stats(i).StopTime = c.StopTime;
            stats(i).Pass = false;
            stats(i).FailureReason = sprintf('%s: %s', err.identifier, err.message);
            fprintf(2, '%s failed: %s\n', c.Name, err.message);
            if logical(p.Results.StopOnFailure)
                rethrow(err);
            end
        end
    end

    [stats, comparison] = apply_pair_gates(stats, thresholds);
    summary = stats_table(cases, stats);
    summaryCsv = fullfile(outputDir, 'summary.csv');
    writetable(summary, summaryCsv);
    summaryMd = fullfile(outputDir, 'summary.md');
    write_summary(summaryMd, timestamp, originalModel, validationModel, ...
        thresholds, comparison, summary);
    write_build_summary(fullfile(outputDir, 'build_summary.md'), cases, builds);

    matrixConfig = struct( ...
        'Timestamp', timestamp, ...
        'OriginalModel', originalModel, ...
        'ValidationModel', validationModel, ...
        'Thresholds', thresholds, ...
        'Cases', {arrayfun(@case_snapshot, cases, 'UniformOutput', false)});
    write_text(fullfile(outputDir, 'matrix_config.json'), ...
        jsonencode(matrixConfig, PrettyPrint=true));
    save(fullfile(outputDir, 'matrix_results.mat'), ...
        'cases', 'stats', 'results', 'builds', 'thresholds', 'comparison', '-v7.3');

    report = struct( ...
        'OutputDirectory', outputDir, ...
        'SummaryCsv', summaryCsv, ...
        'SummaryMarkdown', summaryMd, ...
        'ValidationModel', validationModel, ...
        'Table', summary, ...
        'Stats', stats, ...
        'Comparison', comparison, ...
        'AllPassed', all([stats.Pass]));
    fprintf('\nValidation artifacts: %s\n', outputDir);
end

function report = reanalyze_existing(outputDir)
    resultFile = fullfile(outputDir, 'matrix_results.mat');
    assert(isfile(resultFile), 'Existing matrix result does not exist: %s', resultFile);
    loaded = load(resultFile, 'cases', 'results', 'builds');
    cases = loaded.cases;
    results = loaded.results;
    builds = loaded.builds;
    thresholds = bl_validation_thresholds();
    plotsDir = fullfile(outputDir, 'plots');
    if ~isfolder(plotsDir)
        mkdir(plotsDir);
    end
    stats = repmat(empty_stats(), numel(cases), 1);
    for i = 1:numel(cases)
        if isempty(results{i})
            stats(i).Status = 'error';
            stats(i).Scenario = cases(i).Name;
            stats(i).BuildLevel = cases(i).BuildLevel;
            stats(i).StopTime = cases(i).StopTime;
            stats(i).FailureReason = 'Saved simulation result is unavailable.';
        else
            stats(i) = analyze_bl_result(results{i}, ...
                'Thresholds', thresholds, ...
                'OutputDirectory', plotsDir, ...
                'CaseName', cases(i).Name, ...
                'WritePlot', true);
        end
    end
    [stats, comparison] = apply_pair_gates(stats, thresholds);
    summary = stats_table(cases, stats);
    summaryCsv = fullfile(outputDir, 'summary.csv');
    writetable(summary, summaryCsv);

    configFile = fullfile(outputDir, 'matrix_config.json');
    config = jsondecode(fileread(configFile));
    config.Thresholds = thresholds;
    write_text(configFile, jsonencode(config, PrettyPrint=true));
    [~, timestamp] = fileparts(outputDir);
    summaryMd = fullfile(outputDir, 'summary.md');
    write_summary(summaryMd, timestamp, config.OriginalModel, ...
        config.ValidationModel, thresholds, comparison, summary);
    write_build_summary(fullfile(outputDir, 'build_summary.md'), cases, builds);
    save(resultFile, 'cases', 'stats', 'results', 'builds', ...
        'thresholds', 'comparison', '-v7.3');

    report = struct( ...
        'OutputDirectory', outputDir, ...
        'SummaryCsv', summaryCsv, ...
        'SummaryMarkdown', summaryMd, ...
        'ValidationModel', config.ValidationModel, ...
        'Table', summary, ...
        'Stats', stats, ...
        'Comparison', comparison, ...
        'AllPassed', all([stats.Pass]));
end

function cases = validation_cases()
    base = make_case();
    cases = repmat(base, 11, 1);

    cases(1) = set_case(base, 'bl0_pll_only', 0, 0.35);

    cases(2) = set_case(base, 'bl1_open_loop_grid_open', 1, 0.40);
    cases(2).GridConnected = false;

    cases(3) = set_case(base, 'bl2_breaker_close_current_ramp', 2, 0.65);
    cases(3).GridBreakerCloseTime = 0.05;

    cases(4) = set_case(base, 'bl3_unbalance_neg_off', 3, 0.95);
    cases(4).NegSequenceEnabled = false;
    cases(4).UnbalancedLoadEnabled = true;
    cases(4).UnbalanceSwitchTime = 0.55;

    cases(5) = cases(4);
    cases(5).Name = 'bl3_unbalance_neg_on';
    cases(5).NegSequenceEnabled = true;

    cases(6) = set_case(base, 'bl4_base', 4, 0.65);
    cases(6).NegSequenceEnabled = true;

    cases(7) = cases(6);
    cases(7).Name = 'bl4_external_ff';
    cases(7).ExternalFeedforwardEnabled = true;

    cases(8) = cases(7);
    cases(8).Name = 'bl4_ff_decoupling';
    cases(8).DecouplingEnabled = true;

    cases(9) = cases(8);
    cases(9).Name = 'bl4_ff_decoupling_damping';
    cases(9).ActiveDampingEnabled = true;

    cases(10) = cases(9);
    cases(10).Name = 'bl4_all_compensation';
    cases(10).LeadCompensatorEnabled = true;

    cases(11) = set_case(base, 'bl5_pq_profile_unbalance', 5, 0.95);
    cases(11).NegSequenceEnabled = true;
    cases(11).ExternalFeedforwardEnabled = true;
    cases(11).DecouplingEnabled = true;
    cases(11).ActiveDampingEnabled = true;
    cases(11).LeadCompensatorEnabled = true;
    cases(11).PQProfileEnabled = true;
    cases(11).UnbalancedLoadEnabled = true;
    cases(11).UnbalanceSwitchTime = 0.75;
    cases(11).PRefProfile = [0 0; 0.30 0.05];
    cases(11).QRefProfile = [0 0; 0.55 0.02];
end

function c = make_case()
    c = struct( ...
        'Name', '', 'BuildLevel', 0, 'StopTime', 0.4, ...
        'GridConnected', true, 'GridBreakerCloseTime', 0, ...
        'UnbalancedLoadEnabled', false, 'UnbalanceSwitchTime', 0.75, ...
        'Zab', 8, 'Zbc', 8, 'Zca', 8, 'ZbcAdditional', 2, ...
        'NegSequenceEnabled', false, ...
        'ExternalFeedforwardEnabled', false, ...
        'DecouplingEnabled', false, ...
        'ActiveDampingEnabled', false, ...
        'LeadCompensatorEnabled', false, ...
        'PQProfileEnabled', false, ...
        'PRefProfile', zeros(0, 2), ...
        'QRefProfile', zeros(0, 2));
end

function c = set_case(base, name, buildLevel, stopTime)
    c = base;
    c.Name = name;
    c.BuildLevel = buildLevel;
    c.StopTime = stopTime;
end

function snapshot = case_snapshot(c)
    snapshot = c;
    snapshot.Zab = impedance_snapshot(c.Zab);
    snapshot.Zbc = impedance_snapshot(c.Zbc);
    snapshot.Zca = impedance_snapshot(c.Zca);
    snapshot.ZbcAdditional = impedance_snapshot(c.ZbcAdditional);
end

function value = impedance_snapshot(z)
    value = struct('Real', real(z), 'Imag', imag(z));
end

function s = empty_stats()
    s = struct( ...
        'Status', '', 'Scenario', '', 'BuildLevel', NaN, 'StopTime', NaN, ...
        'Pass', false, 'FailureReason', '', 'SampleCount', NaN, ...
        'IdMean', NaN, 'IqMean', NaN, 'IdRefMean', NaN, 'IqRefMean', NaN, ...
        'CurrentTrackingRms', NaN, 'PLLFrequencyMean', NaN, ...
        'PLLFrequencyErrorMax', NaN, 'PLLVqMean', NaN, 'PLLVqAbsMean', NaN, ...
        'CurrentPeak', NaN, 'ThreeWireResidualPeak', NaN, ...
        'ThreeWireResidualRms', NaN, 'PWMEnableMean', NaN, ...
        'PWMFinite', false, 'PWMMin', NaN, 'PWMMax', NaN, ...
        'PWMBoundaryFraction', NaN, 'Diverged', true, ...
        'LevelSpecificTail0', NaN, 'LevelSpecificTail1', NaN, ...
        'NegativeSequenceCurrentRms', NaN, ...
        'PMeasuredMean', NaN, 'QMeasuredMean', NaN, ...
        'PReferenceMean', NaN, 'QReferenceMean', NaN, ...
        'PTrackingError', NaN, 'QTrackingError', NaN, ...
        'NegativeSequenceImprovement', NaN, ...
        'NegativeSequenceControllerRms', NaN, ...
        'FeatureFlags', struct(), 'PlantConfig', struct(), ...
        'MonitorLayout', '', 'PlotFile', '');
end

function [stats, comparison] = apply_pair_gates(stats, thresholds)
    comparison = struct('Available', false, 'OffRms', NaN, 'OnRms', NaN, ...
        'Improvement', NaN, 'Passed', false, 'Reason', '');
    offIndex = find(strcmp({stats.Scenario}, 'bl3_unbalance_neg_off'), 1);
    onIndex = find(strcmp({stats.Scenario}, 'bl3_unbalance_neg_on'), 1);
    if isempty(offIndex) || isempty(onIndex) || ...
            ~isfinite(stats(offIndex).NegativeSequenceCurrentRms) || ...
            ~isfinite(stats(onIndex).NegativeSequenceCurrentRms)
        comparison.Reason = 'BL3 paired negative-sequence metrics are unavailable.';
        return
    end
    comparison.Available = true;
    comparison.OffRms = stats(offIndex).NegativeSequenceCurrentRms;
    comparison.OnRms = stats(onIndex).NegativeSequenceCurrentRms;
    comparison.Improvement = (comparison.OffRms - comparison.OnRms) / ...
        max(comparison.OffRms, eps);
    comparison.Passed = comparison.Improvement >= thresholds.NegativeSequenceImprovementMin;
    stats(offIndex).NegativeSequenceImprovement = comparison.Improvement;
    stats(onIndex).NegativeSequenceImprovement = comparison.Improvement;
    if ~comparison.Passed
        comparison.Reason = sprintf('negative-sequence RMS improvement %.2f%% < %.2f%%', ...
            100 * comparison.Improvement, 100 * thresholds.NegativeSequenceImprovementMin);
        stats(onIndex).Pass = false;
        stats(onIndex).FailureReason = append_reason(stats(onIndex).FailureReason, comparison.Reason);
    end
end

function value = append_reason(value, extra)
    if isempty(value)
        value = extra;
    else
        value = [value '; ' extra];
    end
end

function summary = stats_table(cases, stats)
    n = numel(stats);
    Case = strings(n, 1);
    BL = zeros(n, 1);
    Status = strings(n, 1);
    Pass = false(n, 1);
    GridConnected = false(n, 1);
    Unbalance = false(n, 1);
    NegSequence = false(n, 1);
    ExternalFF = false(n, 1);
    Decoupling = false(n, 1);
    ActiveDamping = false(n, 1);
    LeadCompensator = false(n, 1);
    CurrentPeak = nan(n, 1);
    ThreeWireResidualRms = nan(n, 1);
    PLLFrequencyErrorMax = nan(n, 1);
    PLLVqAbsMean = nan(n, 1);
    CurrentTrackingRms = nan(n, 1);
    NegativeSequenceRms = nan(n, 1);
    NegativeSequenceImprovement = nan(n, 1);
    PError = nan(n, 1);
    QError = nan(n, 1);
    PWMBoundaryFraction = nan(n, 1);
    FailureReason = strings(n, 1);
    for i = 1:n
        Case(i) = string(cases(i).Name);
        BL(i) = cases(i).BuildLevel;
        Status(i) = string(stats(i).Status);
        Pass(i) = stats(i).Pass;
        GridConnected(i) = cases(i).GridConnected;
        Unbalance(i) = cases(i).UnbalancedLoadEnabled;
        NegSequence(i) = cases(i).NegSequenceEnabled;
        ExternalFF(i) = cases(i).ExternalFeedforwardEnabled;
        Decoupling(i) = cases(i).DecouplingEnabled;
        ActiveDamping(i) = cases(i).ActiveDampingEnabled;
        LeadCompensator(i) = cases(i).LeadCompensatorEnabled;
        CurrentPeak(i) = stats(i).CurrentPeak;
        ThreeWireResidualRms(i) = stats(i).ThreeWireResidualRms;
        PLLFrequencyErrorMax(i) = stats(i).PLLFrequencyErrorMax;
        PLLVqAbsMean(i) = stats(i).PLLVqAbsMean;
        CurrentTrackingRms(i) = stats(i).CurrentTrackingRms;
        NegativeSequenceRms(i) = stats(i).NegativeSequenceCurrentRms;
        NegativeSequenceImprovement(i) = stats(i).NegativeSequenceImprovement;
        PError(i) = stats(i).PTrackingError;
        QError(i) = stats(i).QTrackingError;
        PWMBoundaryFraction(i) = stats(i).PWMBoundaryFraction;
        FailureReason(i) = string(stats(i).FailureReason);
    end
    summary = table(Case, BL, Status, Pass, GridConnected, Unbalance, ...
        NegSequence, ExternalFF, Decoupling, ActiveDamping, LeadCompensator, ...
        CurrentPeak, ThreeWireResidualRms, PLLFrequencyErrorMax, PLLVqAbsMean, ...
        CurrentTrackingRms, NegativeSequenceRms, NegativeSequenceImprovement, ...
        PError, QError, PWMBoundaryFraction, FailureReason);
end

function write_summary(fileName, timestamp, originalModel, validationModel, thresholds, comparison, summary)
    lines = strings(0, 1);
    lines(end + 1) = "# GFL BL0-BL5 SIL validation";
    lines(end + 1) = "";
    lines(end + 1) = "- Timestamp: `" + timestamp + "`";
    lines(end + 1) = "- Original model (not saved): `" + originalModel + "`";
    lines(end + 1) = "- Validation copy: `" + validationModel + "`";
    lines(end + 1) = "- Scope: PC SIL commissioning evidence only; no hardware/grid-code certification.";
    lines(end + 1) = "";
    lines(end + 1) = "## Matrix";
    lines(end + 1) = "";
    lines(end + 1) = "| Case | BL | Result | Ipk | sum(I) RMS | PLL df | |Vq| | dq RMS | neg RMS | P err | Q err | Reason |";
    lines(end + 1) = "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---|";
    for i = 1:height(summary)
        outcome = "FAIL";
        if summary.Pass(i)
            outcome = "PASS";
        end
        lines(end + 1) = sprintf('| %s | %d | %s | %.4g | %.4g | %.4g | %.4g | %.4g | %.4g | %.4g | %.4g | %s |', ...
            summary.Case(i), summary.BL(i), outcome, summary.CurrentPeak(i), ...
            summary.ThreeWireResidualRms(i), summary.PLLFrequencyErrorMax(i), ...
            summary.PLLVqAbsMean(i), summary.CurrentTrackingRms(i), ...
            summary.NegativeSequenceRms(i), summary.PError(i), summary.QError(i), ...
            replace(summary.FailureReason(i), "|", "/"));
    end
    lines(end + 1) = "";
    lines(end + 1) = "## BL3 paired comparison";
    lines(end + 1) = "";
    if comparison.Available
        lines(end + 1) = sprintf('- Negative-sequence RMS: off %.6g, on %.6g, improvement %.2f%% (%s).', ...
            comparison.OffRms, comparison.OnRms, 100 * comparison.Improvement, ...
            pass_word(comparison.Passed));
    else
        lines(end + 1) = "- " + comparison.Reason;
    end
    lines(end + 1) = "";
    lines(end + 1) = "## Threshold snapshot";
    lines(end + 1) = "";
    lines(end + 1) = "```json";
    lines(end + 1) = string(jsonencode(thresholds, PrettyPrint=true));
    lines(end + 1) = "```";
    write_text(fileName, strjoin(lines, newline));
end

function write_build_summary(fileName, cases, builds)
    lines = ["# Build log summary"; ""; "| Case | Result | Log |"; "|---|---|---|"];
    for i = 1:numel(cases)
        outcome = "FAIL";
        logFile = "";
        if ~isempty(builds{i})
            logFile = string(builds{i}.LogFile);
            if builds{i}.Succeeded
                outcome = "PASS";
            end
        end
        lines(end + 1) = "| " + cases(i).Name + " | " + outcome + " | `" + logFile + "` |"; %#ok<AGROW>
    end
    write_text(fileName, strjoin(lines, newline));
end

function value = pass_word(pass)
    if pass
        value = 'PASS';
    else
        value = 'FAIL';
    end
end

function write_text(fileName, value)
    fid = fopen(fileName, 'w', 'n', 'UTF-8');
    assert(fid >= 0, 'Cannot write %s', fileName);
    cleanup = onCleanup(@() fclose(fid));
    fwrite(fid, char(value), 'char');
    clear cleanup
end

function result = run_bl_simulation(buildLevel, stopTime, varargin)
%RUN_BL_SIMULATION Run one GMP SIL validation case without saving the model.
%
% The controller executable is started in a private temporary directory with
% private UDP ports.  The existing model is loaded by its real path and the
% actual GMP SIL mask is configured by its mask parameter names.  No model
% save operation is performed.
%
% Example:
%   result = run_bl_simulation(0, 0.30);
%   stats = analyze_bl_result(result.ResultFile);

    parser = inputParser;
    addRequired(parser, 'buildLevel', @(x) isnumeric(x) && isscalar(x) && ...
        isfinite(x) && x == fix(x) && x >= 0 && x <= 6);
    addRequired(parser, 'stopTime', @(x) isnumeric(x) && isscalar(x) && ...
        isfinite(x) && x > 0);
    addParameter(parser, 'ModelFile', '', @(x) ischar(x) || isstring(x));
    addParameter(parser, 'Executable', '', @(x) ischar(x) || isstring(x));
    addParameter(parser, 'Ports', [13500 13501 13502 13503], ...
        @(x) isnumeric(x) && numel(x) == 4 && all(isfinite(x)));
    addParameter(parser, 'ResultFile', '', @(x) ischar(x) || isstring(x));
    addParameter(parser, 'Scenario', sprintf('BL%d', buildLevel), ...
        @(x) ischar(x) || isstring(x));
    addParameter(parser, 'GridConnected', true, @is_logical_scalar);
    addParameter(parser, 'GridBreakerCloseTime', 0, @is_nonnegative_scalar);
    addParameter(parser, 'UnbalancedLoadEnabled', false, @is_logical_scalar);
    addParameter(parser, 'UnbalanceSwitchTime', 0.75, @is_nonnegative_scalar);
    addParameter(parser, 'Zab', 8 + 0j, @is_impedance_scalar);
    addParameter(parser, 'Zbc', 8 + 0j, @is_impedance_scalar);
    addParameter(parser, 'Zca', 8 + 0j, @is_impedance_scalar);
    addParameter(parser, 'ZbcAdditional', 2 + 0j, @is_impedance_scalar);
    addParameter(parser, 'GridFrequencyHz', 50, ...
        @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
    addParameter(parser, 'NegSequenceEnabled', buildLevel >= 3 && buildLevel <= 5, @is_logical_scalar);
    addParameter(parser, 'ExternalFeedforwardEnabled', buildLevel >= 4 && buildLevel <= 5, @is_logical_scalar);
    addParameter(parser, 'DecouplingEnabled', buildLevel >= 4 && buildLevel <= 5, @is_logical_scalar);
    addParameter(parser, 'ActiveDampingEnabled', buildLevel >= 4 && buildLevel <= 5, @is_logical_scalar);
    addParameter(parser, 'LeadCompensatorEnabled', buildLevel >= 4 && buildLevel <= 5, @is_logical_scalar);
    addParameter(parser, 'PQProfileEnabled', false, @is_logical_scalar);
    addParameter(parser, 'PRefProfile', zeros(0, 2), @is_profile);
    addParameter(parser, 'QRefProfile', zeros(0, 2), @is_profile);
    addParameter(parser, 'DcInitialVoltageV', 30, @is_nonnegative_scalar);
    addParameter(parser, 'DcCapacitanceF', 5e-3, @is_positive_scalar);
    addParameter(parser, 'DcLoadOhm', 500, @is_positive_scalar);
    addParameter(parser, 'DcLoadStepOhm', 500, @is_positive_scalar);
    addParameter(parser, 'DcLoadStepTime', 1e6, @is_nonnegative_scalar);
    addParameter(parser, 'GridVoltageProfile', zeros(0, 2), @is_profile);
    addParameter(parser, 'RectifierPanelProfile', zeros(0, 17), @is_panel_profile);
    addParameter(parser, 'MaxSavedSamples', 100000, ...
        @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 1000 && x == fix(x));
    parse(parser, buildLevel, stopTime, varargin{:});

    buildLevel = double(parser.Results.buildLevel);
    stopTime = double(parser.Results.stopTime);
    thisDir = fileparts(mfilename('fullpath'));

    if strlength(string(parser.Results.ModelFile)) == 0
        modelFile = fullfile(thisDir, 'DP_STD_MDL_DCAC_3ph_2level_gridconn.slx');
    else
        modelFile = char(parser.Results.ModelFile);
    end
    if strlength(string(parser.Results.Executable)) == 0
        executable = fullfile(thisDir, 'validation_build', sprintf('bl%d', buildLevel), ...
            'Digital_Power_simulink.exe');
    else
        executable = char(parser.Results.Executable);
    end
    if strlength(string(parser.Results.ResultFile)) == 0
        resultFile = fullfile(tempdir, sprintf('gmp_gfl_bl%d_result.mat', buildLevel));
    else
        resultFile = char(parser.Results.ResultFile);
    end
    resultFolder = fileparts(resultFile);
    if ~isempty(resultFolder) && ~isfolder(resultFolder)
        mkdir(resultFolder);
    end

    assert(isfile(modelFile), 'Model file does not exist: %s', modelFile);
    assert(isfile(executable), 'SIL executable does not exist: %s', executable);

    ports = round(double(parser.Results.Ports(:).'));
    runtimeDir = fullfile(tempdir, sprintf('gmp_gfl_bl%d_%s', buildLevel, ...
        char(java.util.UUID.randomUUID())));
    mkdir(runtimeDir);
    networkFile = fullfile(runtimeDir, 'network.json');
    network = struct('command_recv_port', ports(3), ...
        'command_trans_port', ports(4), 'receive_port', ports(1), ...
        'target_address', '127.0.0.1', 'transmit_port', ports(2));
    fid = fopen(networkFile, 'w');
    assert(fid >= 0, 'Cannot create temporary network file: %s', networkFile);
    fwrite(fid, jsonencode(network), 'char');
    fclose(fid);

    controller = [];
    modelName = '';
    modelWasLoaded = false;
    previousDirectory = pwd;
    cd(runtimeDir);
    cleanupDirectory = onCleanup(@() cd(previousDirectory));

    try
        controller = start_controller(executable, runtimeDir);

        [~, modelName, ~] = fileparts(modelFile);
        load_system(modelFile);
        modelWasLoaded = true;
        set_param(modelName, 'StopTime', num2str(stopTime, '%.12g'));
        plantConfig = configure_plant(modelName, parser.Results);

        silBlock = find_system(modelName, 'LookUnderMasks', 'all', ...
            'FollowLinks', 'on', 'Name', 'GMP SIL Core Module');
        assert(numel(silBlock) == 1, ...
            'Expected one GMP SIL Core Module, found %d.', numel(silBlock));
        silBlock = silBlock{1};
        set_param(silBlock, 'MaskMsgTxPort', num2str(ports(1)), ...
            'MaskMsgRxPort', num2str(ports(2)), ...
            'MaskCmdTxPort', num2str(ports(3)), ...
            'MaskCmdRxPort', num2str(ports(4)));

        configure_logging(modelName, 'Monitor', 'gfl_monitor');
        configure_logging(modelName, 'EnableTag', 'gfl_enable');
        configure_logging(modelName, 'PWMTag', 'gfl_pwm');

        simOut = sim(modelName, 'ReturnWorkspaceOutputs', 'on');
        logsout = [];
        try
            logsout = simOut.logsout;
        catch
            try
                logsout = simOut.get('logsout');
            catch
            end
        end

        result = struct();
        result.Status = 'ok';
        result.BuildLevel = buildLevel;
        result.StopTime = stopTime;
        result.Scenario = char(string(parser.Results.Scenario));
        result.ModelFile = modelFile;
        result.Executable = executable;
        result.RuntimeDirectory = runtimeDir;
        result.Ports = ports;
        result.PlantConfig = plantConfig;
        result.FeatureFlags = feature_flags(parser.Results);
        result.PRefProfile = double(parser.Results.PRefProfile);
        result.QRefProfile = double(parser.Results.QRefProfile);
        result.GridVoltageProfile = plantConfig.GridVoltageProfile;
        result.RectifierPanelProfile = plantConfig.RectifierPanelProfile;
        if buildLevel == 6
            result.MonitorLayout = 'BL6_RECTIFIER_V1';
        else
            result.MonitorLayout = 'GFL_COMMON_V1';
        end
        result.Signals = compact_logsout(logsout, parser.Results.MaxSavedSamples);
        result.MaxSavedSamples = double(parser.Results.MaxSavedSamples);
        result.ResultFile = resultFile;
        save(resultFile, 'result', '-v7.3');
        stop_controller(controller);
        close_model(modelName, modelWasLoaded);
        clear cleanupDirectory
        fprintf('SIL simulation completed: BL%d, StopTime=%g s\n', buildLevel, stopTime);
        fprintf('Result saved to %s\n', resultFile);
    catch err
        stop_controller(controller);
        close_model(modelName, modelWasLoaded);
        clear cleanupDirectory
        result = struct('Status', 'error', 'BuildLevel', buildLevel, ...
            'StopTime', stopTime, 'ModelFile', modelFile, ...
            'Executable', executable, 'RuntimeDirectory', runtimeDir, ...
            'Ports', ports, 'ResultFile', resultFile, ...
            'Scenario', char(string(parser.Results.Scenario)), ...
            'FeatureFlags', feature_flags(parser.Results), ...
            'ErrorIdentifier', err.identifier, 'ErrorMessage', err.message);
        save(resultFile, 'result', '-v7.3');
        fprintf(2, 'SIL simulation failed for BL%d: %s\n', buildLevel, err.message);
        rethrow(err);
    end
end

function signals = compact_logsout(logsout, maxSamples)
    names = {'gfl_monitor', 'gfl_enable', 'gfl_pwm'};
    signals = struct();
    for i = 1:numel(names)
        element = logsout.get(names{i});
        assert(~isempty(element), 'Signal %s is not present in logsout.', names{i});
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
        if numel(time) > maxSamples
            keep = unique(round(linspace(1, numel(time), maxSamples)));
            time = time(keep);
            signal = signal(keep, :);
        end
        signals.(names{i}) = struct('Time', double(time), 'Data', double(signal));
    end
end

function plant = configure_plant(modelName, cfg)
    gridBreaker = find_system(modelName, 'SearchDepth', 1, ...
        'Name', 'Grid Breaker', 'MaskType', 'Three-Phase Breaker');
    deltaBranches = cell(1, 3);
    deltaSwitches = cell(1, 3);
    pairNames = {'Zab', 'Zbc', 'Zca'};
    for k = 1:3
        deltaBranches{k} = find_system(modelName, 'SearchDepth', 1, ...
            'Name', ['Floating Delta ' pairNames{k}], 'MaskType', 'Series RLC Branch');
        deltaSwitches{k} = find_system(modelName, 'SearchDepth', 1, ...
            'Name', ['Delta Switch ' pairNames{k}], 'MaskType', 'Breaker');
    end
    hasDelta = all(cellfun(@(x) numel(x) == 1, deltaBranches)) && ...
        all(cellfun(@(x) numel(x) == 1, deltaSwitches));
    hasBreaker = numel(gridBreaker) == 1;
    dcCapacitor = find_system(modelName, 'SearchDepth', 1, ...
        'Name', 'BL6 DC Link Capacitor', 'MaskType', 'Series RLC Branch');
    dcLoad = find_system(modelName, 'SearchDepth', 1, ...
        'Name', 'BL6 DC Load Base', 'MaskType', 'Series RLC Branch');
    dcLoadStep = find_system(modelName, 'SearchDepth', 1, ...
        'Name', 'BL6 DC Load Step', 'MaskType', 'Series RLC Branch');
    dcLoadSwitch = find_system(modelName, 'SearchDepth', 1, ...
        'Name', 'BL6 DC Load Step Switch', 'MaskType', 'Breaker');
    panelSource = find_system(modelName, 'SearchDepth', 1, ...
        'Name', 'BL6 Panel Commands', 'BlockType', 'FromWorkspace');
    gridSource = find_system(modelName, 'SearchDepth', 1, ...
        'Name', 'BL6 Grid Voltage Command', 'BlockType', 'FromWorkspace');
    hasRectifierPlant = numel(dcCapacitor) == 1 && numel(dcLoad) == 1 && ...
        numel(dcLoadStep) == 1 && numel(dcLoadSwitch) == 1 && ...
        numel(panelSource) == 1 && numel(gridSource) == 1;

    if ~logical(cfg.GridConnected) && ~hasBreaker
        error('run_bl_simulation:MissingGridBreaker', ...
            'GridConnected=false requires a validation model copy with Grid Breaker.');
    end
    if logical(cfg.UnbalancedLoadEnabled) && ~hasDelta
        error('run_bl_simulation:MissingDeltaLoad', ...
            'UnbalancedLoadEnabled=true requires the floating-delta validation model copy.');
    end
    if cfg.buildLevel == 6 && ~hasRectifierPlant
        error('run_bl_simulation:MissingRectifierPlant', ...
            'BUILD_LEVEL 6 requires a non-destructive BL6 rectifier model copy.');
    end

    if hasBreaker
        if logical(cfg.GridConnected)
            if cfg.GridBreakerCloseTime == 0
                set_param(gridBreaker{1}, 'InitialState', 'closed', ...
                    'SwitchTimes', '[1000000 1000001]');
            else
                set_param(gridBreaker{1}, 'InitialState', 'open', ...
                    'SwitchTimes', mat2str([cfg.GridBreakerCloseTime 1e6], 15));
            end
        else
            set_param(gridBreaker{1}, 'InitialState', 'open', ...
                'SwitchTimes', '[1000000 1000001]');
        end
    end

    zValues = [cfg.Zab, cfg.Zbc + cfg.ZbcAdditional, cfg.Zca];
    if hasDelta
        for k = 1:3
            configure_impedance(deltaBranches{k}{1}, zValues(k), cfg.GridFrequencyHz);
            if logical(cfg.UnbalancedLoadEnabled)
                if cfg.UnbalanceSwitchTime == 0
                    set_param(deltaSwitches{k}{1}, 'InitialState', '1', ...
                        'SwitchingTimes', '[1000000 1000001]');
                else
                    set_param(deltaSwitches{k}{1}, 'InitialState', '0', ...
                        'SwitchingTimes', mat2str([cfg.UnbalanceSwitchTime 1e6], 15));
                end
            else
                set_param(deltaSwitches{k}{1}, 'InitialState', '0', ...
                    'SwitchingTimes', '[1000000 1000001]');
            end
        end
    end

    gridProfile = double(cfg.GridVoltageProfile);
    if isempty(gridProfile)
        gridProfile = [0 16];
    end
    panelProfile = double(cfg.RectifierPanelProfile);
    if isempty(panelProfile)
        panelProfile = default_rectifier_panel_profile();
    end

    if hasRectifierPlant
        set_param(dcCapacitor{1}, 'BranchType', 'C', ...
            'Capacitance', num2str(cfg.DcCapacitanceF, 17), ...
            'Setx0', 'on', 'InitialVoltage', num2str(cfg.DcInitialVoltageV, 17));
        set_param(dcLoad{1}, 'BranchType', 'R', ...
            'Resistance', num2str(cfg.DcLoadOhm, 17));
        set_param(dcLoadStep{1}, 'BranchType', 'R', ...
            'Resistance', num2str(cfg.DcLoadStepOhm, 17));
        if cfg.DcLoadStepTime == 0
            set_param(dcLoadSwitch{1}, 'InitialState', '1', ...
                'SwitchingTimes', '[1000000 1000001]');
        elseif cfg.DcLoadStepTime >= 1e6
            set_param(dcLoadSwitch{1}, 'InitialState', '0', ...
                'SwitchingTimes', '[1000000 1000001]');
        else
            set_param(dcLoadSwitch{1}, 'InitialState', '0', ...
                'SwitchingTimes', mat2str([cfg.DcLoadStepTime 1e6], 15));
        end
        mws = get_param(modelName, 'ModelWorkspace');
        assignin(mws, 'bl6_grid_voltage_profile', gridProfile);
        assignin(mws, 'bl6_panel_commands', panelProfile);
    end

    plant = struct( ...
        'HasGridBreaker', hasBreaker, ...
        'HasFloatingDeltaLoad', hasDelta, ...
        'GridConnected', logical(cfg.GridConnected), ...
        'GridBreakerCloseTime', double(cfg.GridBreakerCloseTime), ...
        'UnbalancedLoadEnabled', logical(cfg.UnbalancedLoadEnabled), ...
        'UnbalanceSwitchTime', double(cfg.UnbalanceSwitchTime), ...
        'Zab', cfg.Zab, 'Zbc', cfg.Zbc, 'Zca', cfg.Zca, ...
        'ZbcAdditional', cfg.ZbcAdditional, ...
        'EffectiveZbc', cfg.Zbc + cfg.ZbcAdditional, ...
        'GridFrequencyHz', double(cfg.GridFrequencyHz), ...
        'HasRectifierPlant', hasRectifierPlant, ...
        'DcInitialVoltageV', double(cfg.DcInitialVoltageV), ...
        'DcCapacitanceF', double(cfg.DcCapacitanceF), ...
        'DcLoadOhm', double(cfg.DcLoadOhm), ...
        'DcLoadStepOhm', double(cfg.DcLoadStepOhm), ...
        'DcLoadStepTime', double(cfg.DcLoadStepTime), ...
        'GridVoltageProfile', gridProfile, ...
        'RectifierPanelProfile', panelProfile);
end

function profile = default_rectifier_panel_profile()
    profile = zeros(1, 17);
    profile(1) = 0;
    profile(2) = 1;
    profile(4) = 36;
    profile(6) = 0.20;
end

function configure_impedance(block, z, frequencyHz)
    r = real(z);
    x = imag(z);
    assert(r >= 0, 'Delta branch resistance must be non-negative.');
    if abs(x) < 1e-12
        set_param(block, 'BranchType', 'R', 'Resistance', num2str(r, 17));
    elseif x > 0
        set_param(block, 'BranchType', 'RL', ...
            'Resistance', num2str(r, 17), ...
            'Inductance', num2str(x / (2 * pi * frequencyHz), 17));
    else
        set_param(block, 'BranchType', 'RC', ...
            'Resistance', num2str(r, 17), ...
            'Capacitance', num2str(-1 / (2 * pi * frequencyHz * x), 17));
    end
end

function flags = feature_flags(cfg)
    flags = struct( ...
        'NegSequenceEnabled', logical(cfg.NegSequenceEnabled), ...
        'ExternalFeedforwardEnabled', logical(cfg.ExternalFeedforwardEnabled), ...
        'DecouplingEnabled', logical(cfg.DecouplingEnabled), ...
        'ActiveDampingEnabled', logical(cfg.ActiveDampingEnabled), ...
        'LeadCompensatorEnabled', logical(cfg.LeadCompensatorEnabled), ...
        'PQProfileEnabled', logical(cfg.PQProfileEnabled));
end

function process = start_controller(executable, runtimeDir)
    startInfo = System.Diagnostics.ProcessStartInfo(executable);
    startInfo.WorkingDirectory = runtimeDir;
    startInfo.UseShellExecute = false;
    startInfo.CreateNoWindow = true;
    process = System.Diagnostics.Process();
    process.StartInfo = startInfo;
    assert(process.Start(), 'Unable to start SIL executable: %s', executable);
    pause(0.25);
    assert(~process.HasExited, 'SIL executable exited before Simulink started.');
end

function configure_logging(modelName, gotoTag, loggingName)
    blocks = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'Goto', ...
        'GotoTag', gotoTag);
    assert(numel(blocks) == 1, ...
        'Expected one top-level Goto block for tag %s, found %d.', gotoTag, numel(blocks));
    handles = get_param(blocks{1}, 'PortHandles');
    line = get_param(handles.Inport, 'Line');
    assert(line ~= -1, 'Goto block %s has no incoming line.', gotoTag);
    sourcePort = get_param(line, 'SrcPortHandle');
    set_param(sourcePort, 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', loggingName);
end

function stop_controller(process)
    if isempty(process)
        return;
    end
    try
        if ~process.HasExited
            process.Kill();
            process.WaitForExit(2000);
        end
    catch
    end
end

function close_model(modelName, modelWasLoaded)
    if modelWasLoaded && ~isempty(modelName)
        try
            close_system(modelName, 0);
        catch
        end
    end
end

function tf = is_logical_scalar(value)
    tf = (islogical(value) || isnumeric(value)) && isscalar(value) && isfinite(double(value));
end

function tf = is_nonnegative_scalar(value)
    tf = isnumeric(value) && isscalar(value) && isfinite(value) && value >= 0;
end

function tf = is_positive_scalar(value)
    tf = isnumeric(value) && isscalar(value) && isfinite(value) && value > 0;
end

function tf = is_impedance_scalar(value)
    tf = isnumeric(value) && isscalar(value) && all(isfinite([real(value) imag(value)])) && real(value) >= 0;
end

function tf = is_profile(value)
    tf = isnumeric(value) && ismatrix(value) && size(value, 2) == 2 && ...
        all(isfinite(value), 'all') && (isempty(value) || all(diff(value(:, 1)) >= 0));
end

function tf = is_panel_profile(value)
    tf = isnumeric(value) && ismatrix(value) && size(value, 2) == 17 && ...
        all(isfinite(value), 'all') && (isempty(value) || all(diff(value(:, 1)) >= 0));
end

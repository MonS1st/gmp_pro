function result = run_bl_simulation(buildLevel, stopTime, varargin)
%RUN_BL_SIMULATION Run one GMP SIL build level without changing the model.
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
        isfinite(x) && x == fix(x) && x >= 0 && x <= 5);
    addRequired(parser, 'stopTime', @(x) isnumeric(x) && isscalar(x) && ...
        isfinite(x) && x > 0);
    addParameter(parser, 'ModelFile', '', @(x) ischar(x) || isstring(x));
    addParameter(parser, 'Executable', '', @(x) ischar(x) || isstring(x));
    addParameter(parser, 'Ports', [13500 13501 13502 13503], ...
        @(x) isnumeric(x) && numel(x) == 4 && all(isfinite(x)));
    addParameter(parser, 'ResultFile', '', @(x) ischar(x) || isstring(x));
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

    try
        controller = start_controller(executable, runtimeDir);

        [~, modelName, ~] = fileparts(modelFile);
        load_system(modelFile);
        modelWasLoaded = true;
        set_param(modelName, 'StopTime', num2str(stopTime, '%.12g'));

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
        result.ModelFile = modelFile;
        result.Executable = executable;
        result.RuntimeDirectory = runtimeDir;
        result.Ports = ports;
        result.Logsout = logsout;
        result.SimulationOutput = simOut;
        result.ResultFile = resultFile;
        save(resultFile, 'result', '-v7.3');
        stop_controller(controller);
        close_model(modelName, modelWasLoaded);
        fprintf('SIL simulation completed: BL%d, StopTime=%g s\n', buildLevel, stopTime);
        fprintf('Result saved to %s\n', resultFile);
    catch err
        stop_controller(controller);
        close_model(modelName, modelWasLoaded);
        result = struct('Status', 'error', 'BuildLevel', buildLevel, ...
            'StopTime', stopTime, 'ModelFile', modelFile, ...
            'Executable', executable, 'RuntimeDirectory', runtimeDir, ...
            'Ports', ports, 'ResultFile', resultFile, ...
            'ErrorIdentifier', err.identifier, 'ErrorMessage', err.message);
        save(resultFile, 'result', '-v7.3');
        fprintf(2, 'SIL simulation failed for BL%d: %s\n', buildLevel, err.message);
        rethrow(err);
    end
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

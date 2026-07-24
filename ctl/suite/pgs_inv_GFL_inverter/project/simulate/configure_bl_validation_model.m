function modelFile = configure_bl_validation_model(originalFile, modelFile, varargin)
%CONFIGURE_BL_VALIDATION_MODEL Create a non-destructive SIL plant copy.
%
% The checked-in gridconn model is intentionally left untouched.  This helper
% copies it to MODELFILE and adds:
%   * one independent three-phase breaker between the filter current sensors
%     and the three controlled grid sources; and
%   * three floating Series RLC branches in a delta connection on the
%     inverter side of the breaker, each behind a time-controlled delta
%     switch so the unbalance can be applied at a repeatable time.
%
% The breaker is time controlled by the Three-Phase Breaker mask and the
% delta impedances are supplied as complex values in ohms.  A positive
% imaginary part is represented by an RL branch and a negative imaginary part
% by an RC branch.  ZbcAdditional is added to Zbc so BL3/BL5 can reuse the
% same model copy with an explicit unbalance injection.
%
% This function deliberately refuses to overwrite ORIGINALFILE or an
% existing MODELFILE.  Delete a generated copy manually before recreating it.

    p = inputParser;
    addParameter(p, 'BreakerSwitchTimes', [0.30], @(x) isnumeric(x) && isvector(x));
    addParameter(p, 'InitialState', 'open', @(x) ischar(x) || isstring(x));
    addParameter(p, 'DeltaSwitchTimes', [0.75], @(x) isnumeric(x) && isvector(x));
    addParameter(p, 'DeltaInitialState', 'open', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Zab', 8 + 0j, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Zbc', 8 + 0j, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Zca', 8 + 0j, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'ZbcAdditional', 0 + 0j, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'GridFrequencyHz', 50, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, varargin{:});
    cfg = p.Results;

    originalFile = char(string(originalFile));
    modelFile = char(string(modelFile));
    if ~isfile(originalFile)
        error('configure_bl_validation_model:MissingOriginal', ...
            'Original model does not exist: %s', originalFile);
    end
    if strcmpi(string(originalFile), string(modelFile))
        error('configure_bl_validation_model:OverwriteOriginal', ...
            'The validation copy must be different from the original model.');
    end
    if isfile(modelFile)
        error('configure_bl_validation_model:CopyExists', ...
            'Refusing to overwrite existing validation model: %s', modelFile);
    end

    copyFolder = fileparts(modelFile);
    if ~isempty(copyFolder) && ~isfolder(copyFolder)
        mkdir(copyFolder);
    end
    [~, modelName] = fileparts(modelFile);

    [copied, copyMessage] = copyfile(originalFile, modelFile);
    assert(copied, 'Could not copy validation model: %s', copyMessage);

    load_system(modelFile);
    cleanupCopy = onCleanup(@() close_if_loaded(modelFile));
    root = modelName;

    % The source/current-sensor endpoints are discovered from port handles,
    % rather than relying on line coordinates or generated numeric handles.
    sourceBlocks = { ...
        [root '/Grid Source(A)'], ...
        [root '/Grid Source(B)'], ...
        [root '/Grid Source(B)1']};
    currentBlocks = { ...
        [root '/Current Measurement'], ...
        [root '/Current Measurement1'], ...
        [root '/Current Measurement2']};

    breaker = [root '/Grid Breaker'];
    add_block('spsThreePhaseBreakerLib/Three-Phase Breaker', breaker, ...
        'Position', [2260 315 2325 390]);
    set_param(breaker, ...
        'InitialState', char(string(cfg.InitialState)), ...
        'SwitchA', 'on', 'SwitchB', 'on', 'SwitchC', 'on', ...
        'External', 'off', ...
        'SwitchTimes', mat2str(normalize_switch_times(cfg.BreakerSwitchTimes), 15), ...
        'Measurements', 'None');

    for k = 1:3
        remove_electrical_line(root, currentBlocks{k}, sourceBlocks{k});
        currentPorts = get_param(currentBlocks{k}, 'PortHandles');
        sourcePorts = get_param(sourceBlocks{k}, 'PortHandles');
        breakerPorts = get_param(breaker, 'PortHandles');
        add_line(root, currentPorts.RConn, breakerPorts.LConn(k), 'autorouting', 'on');
        % In the existing Controlled Voltage Source orientation, RConn is
        % the phase terminal and LConn is the shared neutral terminal.
        add_line(root, breakerPorts.RConn(k), sourcePorts.RConn, 'autorouting', 'on');
    end

    % Store the values in the model workspace as a configuration snapshot.
    % The branch masks are populated with the same values below, so the model
    % remains self-contained when opened outside this MATLAB session.
    mws = get_param(root, 'ModelWorkspace');
    assignin(mws, 'Zab', cfg.Zab);
    assignin(mws, 'Zbc', cfg.Zbc + cfg.ZbcAdditional);
    assignin(mws, 'Zca', cfg.Zca);
    assignin(mws, 'grid_frequency_hz', cfg.GridFrequencyHz);
    assignin(mws, 'grid_breaker_switch_times', cfg.BreakerSwitchTimes);
    assignin(mws, 'delta_switch_times', cfg.DeltaSwitchTimes);

    phaseNodes = { ...
        get_param(currentBlocks{1}, 'PortHandles').LConn, ...
        get_param(currentBlocks{2}, 'PortHandles').LConn, ...
        get_param(currentBlocks{3}, 'PortHandles').LConn};
    zValues = [cfg.Zab, cfg.Zbc + cfg.ZbcAdditional, cfg.Zca];
    pairNames = {'Zab', 'Zbc', 'Zca'};
    pairPorts = {[1 2], [2 3], [3 1]};
    y0 = 465;
    for k = 1:3
        branch = [root '/Floating Delta ' pairNames{k}];
        add_block('spsSeriesRLCBranchLib/Series RLC Branch', branch, ...
            'Position', [2260 y0 + (k - 1) * 60 2365 y0 + 25 + (k - 1) * 60]);
        configure_branch(branch, zValues(k), cfg.GridFrequencyHz);
        deltaSwitch = [root '/Delta Switch ' pairNames{k}];
        add_block('spsBreakerLib/Breaker', deltaSwitch, ...
            'Position', [2395 y0 + (k - 1) * 60 2445 y0 + 25 + (k - 1) * 60]);
        set_param(deltaSwitch, ...
            'InitialState', breaker_state_number(cfg.DeltaInitialState), ...
            'SwitchingTimes', mat2str(normalize_switch_times(cfg.DeltaSwitchTimes), 15), ...
            'External', 'off', 'Measurements', 'None');
        bh = get_param(branch, 'PortHandles');
        sh = get_param(deltaSwitch, 'PortHandles');
        add_line(root, phaseNodes{pairPorts{k}(1)}, sh.LConn, 'autorouting', 'on');
        add_line(root, sh.RConn, bh.LConn, 'autorouting', 'on');
        add_line(root, bh.RConn, phaseNodes{pairPorts{k}(2)}, 'autorouting', 'on');
    end

    set_param(root, 'StopTime', get_param(root, 'StopTime'));
    save_system(root, modelFile);
    clear cleanupCopy
end

function remove_electrical_line(root, leftBlock, rightBlock)
    leftHandle = get_param(leftBlock, 'Handle');
    rightHandle = get_param(rightBlock, 'Handle');
    lines = find_system(root, 'FindAll', 'on', 'Type', 'line');
    for i = 1:numel(lines)
        src = get_param(lines(i), 'SrcBlockHandle');
        dst = get_param(lines(i), 'DstBlockHandle');
        if same_handle(src, leftHandle) && same_handle(dst, rightHandle)
            delete_line(lines(i));
            return
        end
        if same_handle(src, rightHandle) && same_handle(dst, leftHandle)
            delete_line(lines(i));
            return
        end
    end
    error('configure_bl_validation_model:MissingConnection', ...
        'Could not find electrical connection %s <-> %s.', leftBlock, rightBlock);
end

function tf = same_handle(a, b)
    tf = ~isempty(a) && ~isempty(b) && all(double(a(:)) == double(b(:)));
end

function configure_branch(block, z, frequencyHz)
    r = real(z);
    x = imag(z);
    if r < 0
        error('configure_bl_validation_model:NegativeResistance', ...
            'Delta branch resistance must be non-negative, got %g.', r);
    end
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

function value = breaker_state_number(state)
    state = lower(string(state));
    if state == "open"
        value = '0';
    elseif state == "closed"
        value = '1';
    else
        error('configure_bl_validation_model:DeltaInitialState', ...
            'DeltaInitialState must be open or closed.');
    end
end

function times = normalize_switch_times(times)
    times = double(times(:).');
    times(~isfinite(times)) = 1e6;
    if isempty(times)
        times = [1e6 1000001];
    elseif numel(times) == 1
        if times(1) >= 1e6
            times = [1e6 1000001];
        else
            times = [times(1) 1e6];
        end
    end
end

function close_if_loaded(fileName)
    [~, name] = fileparts(fileName);
    if bdIsLoaded(name)
        close_system(name, 0);
    end
end

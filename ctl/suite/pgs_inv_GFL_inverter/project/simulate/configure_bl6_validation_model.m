function modelFile = configure_bl6_validation_model(originalFile, modelFile, varargin)
%CONFIGURE_BL6_VALIDATION_MODEL Create a non-destructive PWM-rectifier plant.
%
% The helper first creates the existing breaker/floating-delta validation
% copy, then replaces the ideal DC voltage source in that copy with a
% capacitor, a base resistive load, and a switched resistive load.  It also
% adds From Workspace sources for the BL6 panel command vector and grid
% voltage profile.  The checked-in original model is never saved.

    p = inputParser;
    addParameter(p, 'DcInitialVoltageV', 30, @is_nonnegative_scalar);
    addParameter(p, 'DcCapacitanceF', 5e-3, @is_positive_scalar);
    addParameter(p, 'DcLoadOhm', 500, @is_positive_scalar);
    addParameter(p, 'DcLoadStepOhm', 500, @is_positive_scalar);
    addParameter(p, 'DcLoadStepTime', 1e6, @is_nonnegative_scalar);
    addParameter(p, 'GridVoltageProfile', [0 16], @is_profile);
    addParameter(p, 'RectifierPanelProfile', default_panel_profile(), @is_panel_profile);
    parse(p, varargin{:});
    cfg = p.Results;

    configure_bl_validation_model(originalFile, modelFile, ...
        'BreakerSwitchTimes', 0, 'InitialState', 'closed', ...
        'DeltaSwitchTimes', 1e6, 'DeltaInitialState', 'open');

    [~, modelName] = fileparts(modelFile);
    load_system(modelFile);
    cleanup = onCleanup(@() close_if_loaded(modelName));
    root = modelName;

    bridge = [root '/Three Phase DC//AC Full Bridge Inverter, LC Filter (GMP STD MDL)'];
    dcSource = [root '/Controlled Voltage Source1'];
    sourceCommand = [root '/Volage Source'];
    assert(getSimulinkBlockHandle(bridge) ~= -1, 'BL6 bridge block was not found.');
    assert(getSimulinkBlockHandle(dcSource) ~= -1, 'Original DC source was not found in the model copy.');

    bridgePorts = get_param(bridge, 'PortHandles');
    sourcePorts = get_param(dcSource, 'PortHandles');
    delete_connected_line(sourcePorts.LConn);
    delete_connected_line(sourcePorts.RConn);
    delete_connected_line(get_param(sourceCommand, 'PortHandles').Outport);
    delete_block(dcSource);
    delete_block(sourceCommand);

    capacitor = [root '/BL6 DC Link Capacitor'];
    add_block('spsSeriesRLCBranchLib/Series RLC Branch', capacitor, ...
        'Position', [1370 205 1470 235]);
    set_param(capacitor, 'BranchType', 'C', ...
        'Capacitance', num2str(cfg.DcCapacitanceF, 17), ...
        'Setx0', 'on', 'InitialVoltage', num2str(cfg.DcInitialVoltageV, 17), ...
        'Measurements', 'None');

    baseLoad = [root '/BL6 DC Load Base'];
    add_block('spsSeriesRLCBranchLib/Series RLC Branch', baseLoad, ...
        'Position', [1370 255 1470 285]);
    set_param(baseLoad, 'BranchType', 'R', ...
        'Resistance', num2str(cfg.DcLoadOhm, 17), 'Measurements', 'None');

    loadSwitch = [root '/BL6 DC Load Step Switch'];
    add_block('spsBreakerLib/Breaker', loadSwitch, ...
        'Position', [1370 310 1420 340]);
    set_param(loadSwitch, 'InitialState', '0', 'External', 'off', ...
        'SwitchingTimes', switch_times(cfg.DcLoadStepTime), 'Measurements', 'None');

    stepLoad = [root '/BL6 DC Load Step'];
    add_block('spsSeriesRLCBranchLib/Series RLC Branch', stepLoad, ...
        'Position', [1450 310 1550 340]);
    set_param(stepLoad, 'BranchType', 'R', ...
        'Resistance', num2str(cfg.DcLoadStepOhm, 17), 'Measurements', 'None');

    capPorts = get_param(capacitor, 'PortHandles');
    basePorts = get_param(baseLoad, 'PortHandles');
    switchPorts = get_param(loadSwitch, 'PortHandles');
    stepPorts = get_param(stepLoad, 'PortHandles');
    dcPositive = bridgePorts.LConn(1);
    dcNegative = bridgePorts.LConn(2);
    add_line(root, dcPositive, capPorts.LConn, 'autorouting', 'on');
    add_line(root, capPorts.RConn, dcNegative, 'autorouting', 'on');
    add_line(root, dcPositive, basePorts.LConn, 'autorouting', 'on');
    add_line(root, basePorts.RConn, dcNegative, 'autorouting', 'on');
    add_line(root, dcPositive, switchPorts.LConn, 'autorouting', 'on');
    add_line(root, switchPorts.RConn, stepPorts.LConn, 'autorouting', 'on');
    add_line(root, stepPorts.RConn, dcNegative, 'autorouting', 'on');

    replace_grid_voltage_constant(root);
    add_panel_command_source(root);

    mws = get_param(root, 'ModelWorkspace');
    assignin(mws, 'bl6_grid_voltage_profile', double(cfg.GridVoltageProfile));
    assignin(mws, 'bl6_panel_commands', double(cfg.RectifierPanelProfile));
    assignin(mws, 'bl6_dc_initial_voltage_v', double(cfg.DcInitialVoltageV));
    assignin(mws, 'bl6_dc_capacitance_f', double(cfg.DcCapacitanceF));
    assignin(mws, 'bl6_dc_load_ohm', double(cfg.DcLoadOhm));
    assignin(mws, 'bl6_dc_load_step_ohm', double(cfg.DcLoadStepOhm));
    assignin(mws, 'bl6_dc_load_step_time', double(cfg.DcLoadStepTime));

    save_system(root, modelFile);
    clear cleanup
end

function replace_grid_voltage_constant(root)
    oldBlock = [root '/Constant'];
    oldPorts = get_param(oldBlock, 'PortHandles');
    line = get_param(oldPorts.Outport, 'Line');
    assert(line ~= -1, 'Grid voltage Constant is not connected.');
    destination = get_param(line, 'DstPortHandle');
    position = get_param(oldBlock, 'Position');
    delete_line(line);
    delete_block(oldBlock);

    source = [root '/BL6 Grid Voltage Command'];
    add_block('simulink/Sources/From Workspace', source, ...
        'Position', position, ...
        'VariableName', 'bl6_grid_voltage_profile', ...
        'Interpolate', 'off', ...
        'OutputAfterFinalValue', 'Holding final value');
    sourcePorts = get_param(source, 'PortHandles');
    add_line(root, sourcePorts.Outport, destination(1), 'autorouting', 'on');
end

function add_panel_command_source(root)
    source = [root '/BL6 Panel Commands'];
    add_block('simulink/Sources/From Workspace', source, ...
        'Position', [650 335 765 365], ...
        'VariableName', 'bl6_panel_commands', ...
        'Interpolate', 'off', ...
        'OutputAfterFinalValue', 'Holding final value');
    panelGoto = [root '/BL6 PanelTag'];
    add_block('simulink/Signal Routing/Goto', panelGoto, ...
        'Position', [795 335 875 365], ...
        'GotoTag', 'PanelTag', 'TagVisibility', 'global');
    add_line(root, get_param(source, 'PortHandles').Outport, ...
        get_param(panelGoto, 'PortHandles').Inport, 'autorouting', 'on');
end

function delete_connected_line(port)
    line = get_param(port, 'Line');
    if line ~= -1
        delete_line(line);
    end
end

function value = switch_times(time)
    if time == 0
        value = '[0 1000000]';
    elseif time >= 1e6
        value = '[1000000 1000001]';
    else
        value = mat2str([time 1e6], 15);
    end
end

function profile = default_panel_profile()
    profile = zeros(1, 17);
    profile(1) = 0;
    profile(2) = 1;
    profile(4) = 36;
    profile(6) = 0.20;
end

function close_if_loaded(modelName)
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
end

function tf = is_nonnegative_scalar(value)
    tf = isnumeric(value) && isscalar(value) && isfinite(value) && value >= 0;
end

function tf = is_positive_scalar(value)
    tf = isnumeric(value) && isscalar(value) && isfinite(value) && value > 0;
end

function tf = is_profile(value)
    tf = isnumeric(value) && ismatrix(value) && size(value, 2) == 2 && ...
        all(isfinite(value), 'all') && ~isempty(value) && all(diff(value(:, 1)) >= 0);
end

function tf = is_panel_profile(value)
    tf = isnumeric(value) && ismatrix(value) && size(value, 2) == 17 && ...
        all(isfinite(value), 'all') && ~isempty(value) && all(diff(value(:, 1)) >= 0);
end

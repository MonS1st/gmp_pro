function upgrade_2023a_single_inv_voltage_feedback()
%UPGRADE_2023A_SINGLE_INV_VOLTAGE_FEEDBACK Route sensed RL voltage to ADC4.
%
% This one-shot upgrade modifies only PGS_2023A_SINGLE_INV_RLOAD_2022b.slx.
% It keeps the existing ADC bus and UDP ABI unchanged. The existing AC bus
% voltage measurement module is reused, including its configured sensitivity,
% 1500 Hz low-pass filter, PWM-triggered sample/hold, and ADC quantization.
% Only the module's negative physical terminal is moved from the converter
% side of the IG shunt to AC_BUS_N, so Sinv_VC/ADC4 represents the voltage
% directly across the final RL load terminals.
%
% The uo_phys logger remains connected to its independent ideal physical
% measurement and is not used as a controller input.

root = fileparts(mfilename('fullpath'));
model = 'PGS_2023A_SINGLE_INV_RLOAD_2022b';
model_file = fullfile(root, [model '.slx']);
marker_text = ...
    '2023A ADC4 feedback = RL-terminal voltage via existing sensor chain';

if ~isfile(model_file)
    error('SINV:2023A:ModelMissing', ...
        'The dedicated 2023A model does not exist: %s', model_file);
end
if bdIsLoaded(model)
    error('SINV:2023A:ModelAlreadyLoaded', ...
        ['Close the dedicated model before running this one-shot upgrade. ' ...
         'No model was changed: %s'], model_file);
end

target_saved = false;
try
    load_system(model_file);

    plant = [model ...
        '/Single Phase DC//AC Full Bridge Inverter, LC Filter (GMP STD MDL)'];
    load_block = [model '/Load'];
    uo_sensor = [model '/2023A Uo Physical Measurement'];
    uo_logger = [model '/2023A Uo Physical Logger'];

    require_block(plant, '2023A inverter plant');
    require_block(load_block, 'final RL load');
    require_block(uo_sensor, 'independent Uo physical measurement');
    require_block(uo_logger, 'independent Uo physical logger');
    assert_logger(uo_logger, 'uo_phys');
    assert_signal_source(uo_logger, uo_sensor);

    if annotation_exists(model, marker_text)
        error('SINV:2023A:AlreadyUpgraded', ...
            ['The voltage-feedback upgrade marker already exists. ' ...
             'The model was not saved again: %s'], model_file);
    end

    % Sinv_VC is the unchanged fifth ADC-bus element. Discover its source
    % rather than relying on the displayed subsystem spelling.
    voltage_module = source_block_for_tag(plant, 'Sinv_VC');
    require_mask_expression(voltage_module, 'ADC_GAIN', 'AC_Voltage_Gain');
    require_mask_expression(voltage_module, 'ADC_BIT', 'ADC_Resolution');
    require_mask_expression(voltage_module, 'ADC_REFERENCE', ...
        'AC_VOLTAGE_ADC_REFERENCE');
    require_mask_expression(voltage_module, 'ADC_BIAS', ...
        'AC_VOLTAGE_ADC_BIAS');
    require_mask_expression(voltage_module, 'cut_frequency', ...
        'AC_VOLTAGE_ADC_CUT_FREQ');
    require_resolved_scalar(plant, 'AC_VOLTAGE_ADC_CUT_FREQ', 1500.0);

    voltage_ports = get_param(voltage_module, 'PortHandles');
    if numel(voltage_ports.LConn) ~= 2
        error('SINV:2023A:VoltageModulePorts', ...
            'The Sinv_VC source must expose exactly two physical terminals.');
    end
    assert_connected(voltage_ports.LConn(1), ...
        'existing AC voltage sensor positive terminal');
    assert_connected(voltage_ports.LConn(2), ...
        'existing AC voltage sensor negative terminal');

    ac_bus_n = [plant '/AC_BUS_N'];
    require_block(ac_bus_n, 'plant AC_BUS_N physical port');
    ac_bus_n_terminal = single_physical_terminal(ac_bus_n);
    assert_connected(ac_bus_n_terminal, 'plant AC_BUS_N terminal');

    % Remember every connected top-level physical terminal under the plant.
    % If deleting the old sensor branch were ever to remove a shared physical
    % network instead of only that branch, the check below aborts before save.
    connected_before = connected_physical_ports(plant);
    old_line = get_param(voltage_ports.LConn(2), 'Line');
    if isempty(old_line) || any(old_line == -1)
        error('SINV:2023A:OldVoltageBranchMissing', ...
            'The existing Sinv_VC negative-terminal branch is missing.');
    end

    delete_line(old_line);
    if is_port_connected(voltage_ports.LConn(2))
        error('SINV:2023A:OldVoltageBranchStillConnected', ...
            'The old Sinv_VC negative-terminal branch was not removed.');
    end

    branch_physical_connection(plant, ac_bus_n_terminal, ...
        voltage_ports.LConn(2), 'AC_BUS_N to Sinv_VC negative terminal');
    assert_connected(voltage_ports.LConn(2), ...
        'upgraded Sinv_VC negative terminal');

    connected_after = connected_physical_ports(plant);
    missing_ports = setdiff(connected_before, connected_after);
    if ~isempty(missing_ports)
        error('SINV:2023A:PhysicalNetworkDamaged', ...
            ['Rewiring disconnected %d previously connected physical ' ...
             'terminal(s). The model will be closed without saving.'], ...
            numel(missing_ports));
    end

    % Keep acceptance measurement independent from the sensed/quantized path.
    uo_ports = get_param(uo_sensor, 'PortHandles');
    if numel(uo_ports.LConn) ~= 2 || isempty(uo_ports.Outport)
        error('SINV:2023A:UoPhysicalPorts', ...
            'The independent uo_phys measurement has unexpected ports.');
    end
    assert_connected(uo_ports.LConn(1), 'uo_phys positive terminal');
    assert_connected(uo_ports.LConn(2), 'uo_phys negative terminal');
    assert_connected(uo_ports.Outport(1), 'uo_phys logging signal');

    note = Simulink.Annotation(model, marker_text);
    note.Position = [1420, 20, 1880, 60];

    save_system(model, model_file);
    target_saved = true;
    if ~strcmp(get_param(model, 'Dirty'), 'off')
        error('SINV:2023A:ModelStillDirty', ...
            'The upgraded model remained dirty after save: %s', model_file);
    end
    close_system(model, 0);

    fprintf('\nUpgraded 2023A voltage feedback successfully:\n%s\n', ...
        model_file);
    fprintf(['ADC4/Sinv_VC now senses the final RL-terminal voltage through ' ...
        'the existing sensor, filter, sample/hold, and ADC chain.\n']);
    fprintf('uo_phys remains an independent ideal acceptance signal.\n');
    fprintf('No standard model, SDPE output, build, or simulation was changed.\n');

catch ME
    fprintf(2, '\n2023A voltage-feedback upgrade failed: %s\n', ME.message);
    for k = 1:numel(ME.stack)
        fprintf(2, '  at %s, line %d\n', ME.stack(k).name, ...
            ME.stack(k).line);
    end
    if bdIsLoaded(model)
        try
            close_system(model, 0);
        catch close_error
            fprintf(2, 'Could not close the unsaved target model: %s\n', ...
                close_error.message);
        end
    end
    if target_saved
        fprintf(2, ['The target had already been saved before the later ' ...
            'failure; inspect it manually: %s\n'], model_file);
    else
        fprintf(2, 'The target file was not saved.\n');
    end
    rethrow(ME);
end
end


function block = source_block_for_tag(system, tag)
goto_blocks = find_system(system, ...
    'SearchDepth', 1, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'BlockType', 'Goto', ...
    'GotoTag', tag);
if numel(goto_blocks) ~= 1
    error('SINV:2023A:GotoTagMismatch', ...
        'Expected exactly one Goto block for %s, found %d.', ...
        tag, numel(goto_blocks));
end

connectivity = get_param(goto_blocks{1}, 'PortConnectivity');
source_handles = [];
for k = 1:numel(connectivity)
    handles = connectivity(k).SrcBlock;
    source_handles = [source_handles, handles(handles ~= -1)]; %#ok<AGROW>
end
source_handles = unique(source_handles);
if numel(source_handles) ~= 1
    error('SINV:2023A:GotoSourceMismatch', ...
        'Expected exactly one source block for %s, found %d.', ...
        tag, numel(source_handles));
end
block = getfullname(source_handles(1));
end


function require_mask_expression(block, parameter, expected)
actual = strtrim(get_param(block, parameter));
if ~strcmp(actual, expected)
    error('SINV:2023A:SensorParameterMismatch', ...
        '%s on %s must remain "%s"; found "%s".', ...
        parameter, block, expected, actual);
end
end


function require_resolved_scalar(block, parameter, expected)
expression = get_param(block, parameter);
try
    value = double(slResolve(expression, block));
catch ME
    error('SINV:2023A:SensorParameterUnresolved', ...
        'Could not resolve %s=%s on %s: %s', ...
        parameter, expression, block, ME.message);
end
if ~isscalar(value) || ~isfinite(value) || ...
        abs(value - expected) > max(1.0, abs(expected)) * 1e-12
    error('SINV:2023A:SensorParameterValue', ...
        '%s on %s must resolve to %.17g; found %.17g.', ...
        parameter, block, expected, value);
end
end


function terminal = single_physical_terminal(block)
ports = get_param(block, 'PortHandles');
terminals = [ports.LConn(:); ports.RConn(:)];
terminals = terminals(terminals ~= -1);
if numel(terminals) ~= 1
    error('SINV:2023A:PhysicalPortMismatch', ...
        'Expected one physical terminal on %s, found %d.', ...
        block, numel(terminals));
end
terminal = terminals(1);
end


function handles = connected_physical_ports(system)
blocks = find_system(system, ...
    'SearchDepth', 1, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'Type', 'Block');
handles = [];
for k = 1:numel(blocks)
    ports = get_param(blocks{k}, 'PortHandles');
    candidates = [ports.LConn(:); ports.RConn(:)];
    for p = 1:numel(candidates)
        if candidates(p) ~= -1 && is_port_connected(candidates(p))
            handles(end + 1) = candidates(p); %#ok<AGROW>
        end
    end
end
handles = unique(handles);
end


function branch_physical_connection(system, existing_terminal, new_terminal, label)
line_handle = get_param(existing_terminal, 'Line');
if isempty(line_handle) || any(line_handle == -1)
    error('SINV:2023A:PhysicalTerminalUnconnected', ...
        'Existing physical terminal is not connected: %s.', label);
end
source_port = get_param(line_handle, 'SrcPortHandle');
source_port = source_port(source_port ~= -1);
if isempty(source_port)
    error('SINV:2023A:PhysicalLineSourceMissing', ...
        'Could not find the existing physical line source: %s.', label);
end
try
    add_line(system, source_port(1), new_terminal, 'autorouting', 'on');
catch ME
    error('SINV:2023A:PhysicalBranchFailed', ...
        'Failed to branch %s: %s', label, ME.message);
end
end


function assert_logger(block, variable)
if ~strcmp(get_param(block, 'BlockType'), 'ToWorkspace') || ...
        ~strcmp(get_param(block, 'VariableName'), variable) || ...
        ~strcmpi(get_param(block, 'SaveFormat'), 'Timeseries')
    error('SINV:2023A:PhysicalLoggerInvalid', ...
        '%s must log %s as a timeseries.', block, variable);
end
end


function assert_signal_source(destination, expected_source)
ports = get_param(destination, 'PortHandles');
if isempty(ports.Inport)
    error('SINV:2023A:LoggerInputMissing', ...
        'Logger has no input port: %s', destination);
end
line_handle = get_param(ports.Inport(1), 'Line');
if isempty(line_handle) || any(line_handle == -1)
    error('SINV:2023A:LoggerDisconnected', ...
        'Logger is disconnected: %s', destination);
end
source_handle = get_param(line_handle, 'SrcBlockHandle');
if numel(source_handle) ~= 1 || ...
        source_handle ~= get_param(expected_source, 'Handle')
    error('SINV:2023A:LoggerSourceMismatch', ...
        '%s is not driven directly by %s.', destination, expected_source);
end
end


function present = annotation_exists(system, text)
annotations = find_system(system, ...
    'FindAll', 'on', ...
    'SearchDepth', 1, ...
    'Type', 'annotation');
present = false;
for k = 1:numel(annotations)
    if strcmp(get_param(annotations(k), 'Name'), text)
        present = true;
        return;
    end
end
end


function connected = is_port_connected(port)
line_handle = get_param(port, 'Line');
connected = ~isempty(line_handle) && all(line_handle ~= -1);
end


function assert_connected(port, description)
if ~is_port_connected(port)
    error('SINV:2023A:ConnectionMissing', ...
        'Required connection is missing: %s.', description);
end
end


function require_block(block, description)
if getSimulinkBlockHandle(block) == -1
    error('SINV:2023A:RequiredBlockMissing', ...
        'Required %s was not found: %s', description, block);
end
end

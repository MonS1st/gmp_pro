function create_2023a_single_inv_rload_model()
%CREATE_2023A_SINGLE_INV_RLOAD_MODEL Create the measurement-only 2023A model.
%
% This function copies PGS_STD_SINV_MODEL_RLOAD_2022b.slx and modifies only
% the copy. It adds three direct physical waveform records:
%   uo_phys - voltage directly across the top-level resistive load;
%   io_phys - load current derived from the existing IG shunt voltage;
%   il_phys - existing ideal filter-inductor current measurement.
%
% It does not run configure_sinv_models, generate SDPE, build native code,
% start the UDP controller, or run a simulation.

root = fileparts(mfilename('fullpath'));
source_model = 'PGS_STD_SINV_MODEL_RLOAD_2022b';
target_model = 'PGS_2023A_SINGLE_INV_RLOAD_2022b';
source_file = fullfile(root, [source_model '.slx']);
target_file = fullfile(root, [target_model '.slx']);

target_created = false;
target_saved = false;

if ~isfile(source_file)
    error('SINV:2023A:SourceMissing', ...
        'Source model does not exist: %s', source_file);
end

if isfile(target_file) || bdIsLoaded(target_model)
    error('SINV:2023A:TargetExists', ...
        ['Target model already exists or is loaded. No file was overwritten. ' ...
         'Resolve it manually before retrying: %s'], target_file);
end

if bdIsLoaded(source_model) && strcmp(get_param(source_model, 'Dirty'), 'on')
    error('SINV:2023A:SourceDirty', ...
        ['The source model is loaded with unsaved changes. Close it without ' ...
         'saving, or resolve those changes explicitly, before creating the copy.']);
end

try
    [copy_ok, copy_message] = copyfile(source_file, target_file);
    if ~copy_ok
        error('SINV:2023A:CopyFailed', ...
            'Could not copy the source model: %s', copy_message);
    end
    target_created = true;

    load_system(target_file);
    if ~bdIsLoaded(target_model)
        error('SINV:2023A:TargetLoadFailed', ...
            'The copied target model did not load: %s', target_file);
    end

    plant = [target_model ...
        '/Single Phase DC//AC Full Bridge Inverter, LC Filter (GMP STD MDL)'];
    load_block = [target_model '/Load'];

    require_block(plant, '2023A plant');
    require_block(load_block, 'top-level resistive load');

    % Discover the exact measurement modules through their existing Goto
    % tags. This avoids relying on the misspelled displayed block names.
    vc_module = source_block_for_tag(plant, 'Sinv_VC');
    il_module = source_block_for_tag(plant, 'Sinv_IL');
    ig_module = source_block_for_tag(plant, 'Sinv_IG');

    voltage_template = require_child_block(vc_module, 'Voltage Measurement');
    il_raw_sensor = require_child_block(il_module, 'Current Measurement');
    ig_raw_sensor = require_child_block(ig_module, 'Voltage Measurement');

    %% Direct voltage measurement across the two top-level RL terminals.
    uo_sensor = [target_model '/2023A Uo Physical Measurement'];
    uo_logger = [target_model '/2023A Uo Physical Logger'];
    ensure_block_absent(uo_sensor);
    ensure_block_absent(uo_logger);

    load_position = get_param(load_block, 'Position');
    sensor_position = [load_position(3) + 65, load_position(2) + 15, ...
                       load_position(3) + 100, load_position(2) + 65];
    logger_position = [sensor_position(3) + 70, sensor_position(2) + 5, ...
                       sensor_position(3) + 220, sensor_position(2) + 35];

    add_block(voltage_template, uo_sensor, 'Position', sensor_position);
    uo_ports = get_param(uo_sensor, 'PortHandles');
    load_ports = get_param(load_block, 'PortHandles');

    if numel(uo_ports.LConn) ~= 2 || isempty(load_ports.LConn) || ...
            isempty(load_ports.RConn)
        error('SINV:2023A:UnexpectedVoltagePorts', ...
            ['Expected two measurement conserving ports and two load ' ...
             'terminals while adding Uo_phys.']);
    end

    % Load LConn is connected to plant AC_BUS_L; Load RConn is connected to
    % plant AC_BUS_N. The measurement therefore excludes the converter-side
    % VC reference point and includes the IG shunt drop exactly as required.
    branch_physical_connection(target_model, load_ports.LConn(1), ...
        uo_ports.LConn(1), 'RL L terminal to Uo positive terminal');
    branch_physical_connection(target_model, load_ports.RConn(1), ...
        uo_ports.LConn(2), 'RL N terminal to Uo negative terminal');

    add_timeseries_logger(target_model, uo_ports.Outport(1), uo_logger, ...
        'uo_phys', logger_position);

    %% Direct filter-inductor current record from the existing ideal sensor.
    il_logger = [il_module '/2023A IL Physical Logger'];
    ensure_block_absent(il_logger);
    il_sensor_position = get_param(il_raw_sensor, 'Position');
    il_logger_position = [il_sensor_position(1) + 150, ...
                          il_sensor_position(2) + 170, ...
                          il_sensor_position(1) + 300, ...
                          il_sensor_position(2) + 200];
    il_sensor_ports = get_param(il_raw_sensor, 'PortHandles');
    add_timeseries_logger(il_module, il_sensor_ports.Outport(1), il_logger, ...
        'il_phys', il_logger_position);

    %% Direct load-current record from the existing IG shunt measurement.
    shunt_expression = get_param(plant, 'GRID_CURRENT_SENSOR_R');
    shunt_resistance = str2double(strtrim(shunt_expression));
    if ~isfinite(shunt_resistance) || shunt_resistance <= 0
        error('SINV:2023A:InvalidIGShunt', ...
            ['GRID_CURRENT_SENSOR_R must resolve to a positive numeric ' ...
             'literal for the measurement-only model. Current expression: %s'], ...
            shunt_expression);
    end

    ig_gain = [ig_module '/2023A IG Shunt To Current'];
    ig_logger = [ig_module '/2023A IG Io Physical Logger'];
    ensure_block_absent(ig_gain);
    ensure_block_absent(ig_logger);

    ig_sensor_position = get_param(ig_raw_sensor, 'Position');
    ig_gain_position = [ig_sensor_position(1) + 80, ...
                        ig_sensor_position(2) + 190, ...
                        ig_sensor_position(1) + 180, ...
                        ig_sensor_position(2) + 220];
    ig_logger_position = [ig_gain_position(3) + 60, ig_gain_position(2), ...
                          ig_gain_position(3) + 210, ig_gain_position(4)];

    add_block('simulink/Math Operations/Gain', ig_gain, ...
        'Gain', num2str(1.0 / shunt_resistance, 17), ...
        'Position', ig_gain_position);
    ig_raw_ports = get_param(ig_raw_sensor, 'PortHandles');
    ig_gain_ports = get_param(ig_gain, 'PortHandles');
    add_line(ig_module, ig_raw_ports.Outport(1), ig_gain_ports.Inport(1), ...
        'autorouting', 'on');
    add_timeseries_logger(ig_module, ig_gain_ports.Outport(1), ig_logger, ...
        'io_phys', ig_logger_position);

    %% Structural checks before saving.
    require_block(uo_sensor, 'direct RL terminal voltage measurement');
    require_block(uo_logger, 'Uo physical logger');
    require_block(il_logger, 'IL physical logger');
    require_block(ig_gain, 'IG shunt-to-current conversion');
    require_block(ig_logger, 'Io physical logger');

    assert_connected(uo_ports.LConn(1), 'Uo positive physical terminal');
    assert_connected(uo_ports.LConn(2), 'Uo negative physical terminal');
    assert_connected(uo_ports.Outport(1), 'Uo measurement output');
    assert_connected(il_sensor_ports.Outport(1), 'IL raw measurement output');
    assert_connected(ig_raw_ports.Outport(1), 'IG shunt-voltage output');
    assert_connected(ig_gain_ports.Outport(1), 'Io physical-current output');

    save_system(target_model, target_file);
    target_saved = true;

    if ~strcmp(get_param(target_model, 'Dirty'), 'off')
        error('SINV:2023A:TargetStillDirty', ...
            'Target model remained dirty after save: %s', target_file);
    end

    close_system(target_model, 0);

    fprintf('\nCreated 2023A measurement-only model successfully:\n%s\n', ...
        target_file);
    fprintf('Recorded variables: uo_phys, io_phys, il_phys\n');
    fprintf('Source model was not saved or modified.\n');
    fprintf('No SDPE generation, build, UDP launch, or simulation was run.\n');

catch ME
    fprintf(2, '\n2023A model creation failed: %s\n', ME.message);
    for k = 1:numel(ME.stack)
        fprintf(2, '  at %s, line %d\n', ME.stack(k).name, ME.stack(k).line);
    end

    if bdIsLoaded(target_model)
        try
            close_system(target_model, 0);
        catch close_error
            fprintf(2, 'Could not close unsaved target model: %s\n', ...
                close_error.message);
        end
    end

    % Remove only the exact target created by this invocation. Never touch
    % the source model or any *.slx.r2022b backup.
    if target_created && ~target_saved && isfile(target_file)
        try
            delete(target_file);
            fprintf(2, 'Removed incomplete target copy: %s\n', target_file);
        catch delete_error
            fprintf(2, 'Could not remove incomplete target copy: %s\n', ...
                delete_error.message);
        end
    end

    rethrow(ME);
end
end


function block = source_block_for_tag(plant, tag)
goto_blocks = find_system(plant, ...
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


function child = require_child_block(parent, name)
blocks = find_system(parent, ...
    'SearchDepth', 1, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'Type', 'Block', ...
    'Name', name);

if numel(blocks) ~= 1
    error('SINV:2023A:ChildBlockMismatch', ...
        'Expected one child named "%s" under "%s", found %d.', ...
        name, parent, numel(blocks));
end
child = blocks{1};
end


function branch_physical_connection(system, existing_terminal, new_terminal, label)
line_handle = get_param(existing_terminal, 'Line');
if isempty(line_handle) || any(line_handle == -1)
    error('SINV:2023A:PhysicalTerminalUnconnected', ...
        'Existing physical terminal is not connected: %s.', label);
end

source_port = get_param(line_handle, 'SrcPortHandle');
if isempty(source_port) || any(source_port == -1)
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


function add_timeseries_logger(parent, source_port, block, variable, position)
add_block('simulink/Sinks/To Workspace', block, ...
    'VariableName', variable, ...
    'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', '2000000', ...
    'Decimation', '1', ...
    'Position', position);

logger_ports = get_param(block, 'PortHandles');
add_line(parent, source_port, logger_ports.Inport(1), 'autorouting', 'on');
end


function assert_connected(port, description)
line_handle = get_param(port, 'Line');
if isempty(line_handle) || all(line_handle == -1)
    error('SINV:2023A:ConnectionMissing', ...
        'Required connection is missing: %s.', description);
end
end


function ensure_block_absent(block)
if getSimulinkBlockHandle(block) ~= -1
    error('SINV:2023A:BlockAlreadyExists', ...
        'Refusing to replace existing block: %s', block);
end
end


function require_block(block, description)
if getSimulinkBlockHandle(block) == -1
    error('SINV:2023A:RequiredBlockMissing', ...
        'Required %s was not found: %s', description, block);
end
end

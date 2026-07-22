% inspect_2023a_rload_exact_readonly.m
% Exact, read-only topology inspection for the 2023A RLOAD model.
% This script does not modify, save, or simulate the model.

script_path = mfilename('fullpath');
if isempty(script_path)
    error('Save this script in the simulate directory before running it.');
end

root = fileparts(script_path);
mdl = 'PGS_2023A_SINGLE_INV_RLOAD_2022b';
model_file = fullfile(root, [mdl '.slx']);

if ~isfile(model_file)
    error('Model not found: %s', model_file);
end

was_loaded = bdIsLoaded(mdl);
if ~was_loaded
    load_system(model_file);
end
cleanup = onCleanup(@() close_if_owned(mdl, was_loaded)); %#ok<NASGU>

plant = [mdl ...
    '/Single Phase DC//AC Full Bridge Inverter, LC Filter (GMP STD MDL)'];

fprintf('\nMODEL\n');
fprintf('  Name: %s\n', mdl);
fprintf('  File: %s\n', get_param(mdl, 'FileName'));
fprintf('  Dirty: %s\n', get_param(mdl, 'Dirty'));
print_parameter(mdl, 'PreLoadFcn');
print_parameter(mdl, 'PostLoadFcn');
print_parameter(mdl, 'InitFcn');

fprintf('\nTOP-LEVEL LOAD AND PLANT PORTS\n');
load_block = [mdl '/Load'];
dump_block(load_block);
print_parameter(load_block, 'BranchType');
print_parameter(load_block, 'Resistance');
print_parameter(load_block, 'Measurements');
dump_block(plant);

fprintf('\nEXACT PHYSICAL BLOCKS\n');
physical_names = {
    'AC_BUS_L'
    'AC_BUS_N'
    'SINV_Filter_L'
    'SINV_Filter_C'
    'Grid_Current_Sensor'
    'AC bus Voltage Measurement Module'
    'Capactor Current Measurement Module1'
};

for k = 1:numel(physical_names)
    block = [plant '/' physical_names{k}];
    dump_block(block);
end

grid_sensor = [plant '/Grid_Current_Sensor'];
print_parameter(grid_sensor, 'BranchType');
print_parameter(grid_sensor, 'Resistance');
print_parameter(plant, 'GRID_CURRENT_SENSOR_R');
print_parameter(plant, 'Grid_Current_Gain');

fprintf('\nEXACT GOTO/FROM CHAINS\n');
tags = {'Sinv_IL', 'Sinv_VC', 'Sinv_IG'};

for k = 1:numel(tags)
    tag = tags{k};
    fprintf('\n--- %s ---\n', tag);

    goto_blocks = find_system(plant, ...
        'SearchDepth', 1, ...
        'LookUnderMasks', 'all', ...
        'FollowLinks', 'on', ...
        'BlockType', 'Goto', ...
        'GotoTag', tag);

    from_blocks = find_system(plant, ...
        'SearchDepth', 1, ...
        'LookUnderMasks', 'all', ...
        'FollowLinks', 'on', ...
        'BlockType', 'From', ...
        'GotoTag', tag);

    fprintf('Goto count: %d\n', numel(goto_blocks));
    for n = 1:numel(goto_blocks)
        dump_block(goto_blocks{n});
        dump_source_blocks(goto_blocks{n});
    end

    fprintf('From count: %d\n', numel(from_blocks));
    for n = 1:numel(from_blocks)
        dump_block(from_blocks{n});
    end
end

fprintf('\nADC BUS CREATOR INPUT ORDER\n');
bus_creators = find_system(plant, ...
    'SearchDepth', 1, ...
    'BlockType', 'BusCreator');

if numel(bus_creators) ~= 1
    fprintf('Expected one BusCreator, found %d.\n', numel(bus_creators));
else
    dump_block(bus_creators{1});
end

fprintf('\nTOP-LEVEL ADC ROUTING\n');
top_adc_goto = find_system(mdl, ...
    'SearchDepth', 1, ...
    'BlockType', 'Goto', ...
    'GotoTag', 'ADC_PORT');

top_adc_from = find_system(mdl, ...
    'SearchDepth', 1, ...
    'BlockType', 'From', ...
    'GotoTag', 'ADC_PORT');

for k = 1:numel(top_adc_goto)
    dump_block(top_adc_goto{k});
end
for k = 1:numel(top_adc_from)
    dump_block(top_adc_from{k});
end

fprintf('\nInspection complete. Model Dirty state: %s\n', ...
    get_param(mdl, 'Dirty'));


function dump_source_blocks(block)
pc = get_param(block, 'PortConnectivity');

for k = 1:numel(pc)
    handles = pc(k).SrcBlock;
    handles = handles(handles ~= -1);

    for n = 1:numel(handles)
        source = getfullname(handles(n));
        fprintf('  Exact source block:\n    %s\n', source);
        dump_block(source);
    end
end
end


function dump_block(block)
fprintf('\nBLOCK: %s\n', block);
fprintf('  BlockType: %s\n', safe_text(block, 'BlockType'));
fprintf('  MaskType: %s\n', safe_text(block, 'MaskType'));

pc = get_param(block, 'PortConnectivity');

for k = 1:numel(pc)
    fprintf('  Port %d type: %s\n', k, value_text(pc(k).Type));
    print_handles('Source', pc(k).SrcBlock);
    print_handles('Destination', pc(k).DstBlock);
end
end


function print_handles(label, handles)
if isempty(handles)
    return;
end

handles = handles(handles ~= -1);

for k = 1:numel(handles)
    try
        fprintf('    %s: %s\n', label, getfullname(handles(k)));
    catch
        fprintf('    %s handle: %s\n', label, value_text(handles(k)));
    end
end
end


function print_parameter(object, parameter)
try
    value = get_param(object, parameter);
    fprintf('  %s = %s\n', parameter, value_text(value));
catch ME
    fprintf('  %s unavailable: %s\n', parameter, ME.message);
end
end


function text = safe_text(object, parameter)
try
    text = value_text(get_param(object, parameter));
catch
    text = '<unavailable>';
end
end


function text = value_text(value)
if ischar(value)
    text = value;
elseif isstring(value)
    text = char(value);
elseif isnumeric(value) || islogical(value)
    text = mat2str(value);
else
    text = '<non-text value>';
end

if isempty(text)
    text = '<empty>';
end
end


function close_if_owned(mdl, was_loaded)
if ~was_loaded && bdIsLoaded(mdl)
    close_system(mdl, 0);
end
end
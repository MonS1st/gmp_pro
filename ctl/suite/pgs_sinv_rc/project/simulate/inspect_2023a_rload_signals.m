function inspect_2023a_rload_signals()
%INSPECT_2023A_RLOAD_SIGNALS Write a read-only R-load signal map.

root = fileparts(mfilename('fullpath'));

mdl = 'PGS_STD_SINV_MODEL_RLOAD_2022b';
model_file = fullfile(root, [mdl '.slx']);

load_system(model_file);

out_dir = fullfile(root, 'validation');
if ~isfolder(out_dir)
    mkdir(out_dir);
end

out_file = fullfile(out_dir, '2023a_rload_signal_map.txt');
fid = fopen(out_file, 'w');

if fid < 0
    close_system(mdl, 0);
    error('Cannot create output file: %s', out_file);
end

file_cleanup = onCleanup(@() safe_close_file(fid));
model_cleanup = onCleanup(@() safe_close_model(mdl));

fprintf(fid, 'Model: %s\n', mdl);
fprintf(fid, 'Model file: %s\n', get_param(mdl, 'FileName'));
fprintf(fid, 'Generated: %s\n\n', datestr(now, 31));

terms = {'IL', 'VC', 'IG', 'IDC', 'VDC', 'RLOAD', 'RL'};

for k = 1:numel(terms)
    term = terms{k};
    escaped_term = regexptranslate('escape', term);

    fprintf(fid, '\n');
    fprintf(fid, '==================================================\n');
    fprintf(fid, 'SEARCH TERM: %s\n', term);
    fprintf(fid, '==================================================\n');

    %% 1. 搜索名称中含目标词的模块
    blocks = find_system( ...
        mdl, ...
        'LookUnderMasks', 'all', ...
        'FollowLinks', 'on', ...
        'RegExp', 'on', ...
        'Type', 'Block', ...
        'Name', ['(?i).*' escaped_term '.*']);

    fprintf(fid, '\nBlocks found: %d\n', numel(blocks));

    for n = 1:numel(blocks)
        blk = blocks{n};

        fprintf(fid, '\nBLOCK: %s\n', blk);
        fprintf(fid, '  Parent: %s\n', get_param(blk, 'Parent'));
        fprintf(fid, '  BlockType: %s\n', safe_get_param(blk, 'BlockType'));
        fprintf(fid, '  MaskType: %s\n', safe_get_param(blk, 'MaskType'));
        fprintf(fid, '  ReferenceBlock: %s\n', ...
            safe_get_param(blk, 'ReferenceBlock'));

        try
            pc = get_param(blk, 'PortConnectivity');

            for p = 1:numel(pc)
                fprintf(fid, '  Connectivity entry %d:\n', p);

                if isfield(pc, 'Type')
                    fprintf(fid, '    Port type: %s\n', ...
                        value_to_text(pc(p).Type));
                end

                src = pc(p).SrcBlock;

                if ~isempty(src)
                    src = src(src ~= -1);

                    for q = 1:numel(src)
                        try
                            fprintf(fid, '    Source: %s\n', ...
                                getfullname(src(q)));
                        catch
                            fprintf(fid, ...
                                '    Source handle: %s\n', ...
                                value_to_text(src(q)));
                        end
                    end
                end

                dst = pc(p).DstBlock;

                if ~isempty(dst)
                    dst = dst(dst ~= -1);

                    for q = 1:numel(dst)
                        try
                            fprintf(fid, '    Destination: %s\n', ...
                                getfullname(dst(q)));
                        catch
                            fprintf(fid, ...
                                '    Destination handle: %s\n', ...
                                value_to_text(dst(q)));
                        end
                    end
                end
            end
        catch ME
            fprintf(fid, '  Port inspection failed: %s\n', ME.message);
        end
    end

    %% 2. 搜索名称完全匹配的信号线
    line_handles = find_system( ...
        mdl, ...
        'FindAll', 'on', ...
        'RegExp', 'on', ...
        'Type', 'line', ...
        'Name', ['(?i)^' escaped_term '$']);

    fprintf(fid, '\nSignal lines found: %d\n', numel(line_handles));

    for n = 1:numel(line_handles)
        line_handle = line_handles(n);

        fprintf(fid, '\nSIGNAL LINE: %s\n', ...
            safe_get_param(line_handle, 'Name'));

        try
            src = get_param(line_handle, 'SrcBlockHandle');
            src_port = get_param(line_handle, 'SrcPortHandle');
            dst = get_param(line_handle, 'DstBlockHandle');
            dst_port = get_param(line_handle, 'DstPortHandle');

            if ~isempty(src) && src ~= -1
                fprintf(fid, '  Source block: %s\n', getfullname(src));
                fprintf(fid, '  Source port handle: %s\n', ...
                    value_to_text(src_port));
            end

            if ~isempty(dst)
                valid_dst = dst ~= -1;
                dst = dst(valid_dst);

                if isnumeric(dst_port) && ...
                        numel(dst_port) == numel(valid_dst)
                    dst_port = dst_port(valid_dst);
                end

                for q = 1:numel(dst)
                    fprintf(fid, '  Destination block: %s\n', ...
                        getfullname(dst(q)));

                    if isnumeric(dst_port) && q <= numel(dst_port)
                        fprintf(fid, ...
                            '  Destination port handle: %s\n', ...
                            value_to_text(dst_port(q)));
                    end
                end
            end
        catch ME
            fprintf(fid, '  Signal inspection failed: %s\n', ME.message);
        end
    end

    %% 3. 搜索 Goto 标签
    goto_blocks = find_system( ...
        mdl, ...
        'LookUnderMasks', 'all', ...
        'FollowLinks', 'on', ...
        'Type', 'Block', ...
        'BlockType', 'Goto', ...
        'GotoTag', term);

    fprintf(fid, '\nGoto blocks found: %d\n', numel(goto_blocks));

    for n = 1:numel(goto_blocks)
        fprintf(fid, '  %s\n', goto_blocks{n});
    end

    %% 4. 搜索 From 标签
    from_blocks = find_system( ...
        mdl, ...
        'LookUnderMasks', 'all', ...
        'FollowLinks', 'on', ...
        'Type', 'Block', ...
        'BlockType', 'From', ...
        'GotoTag', term);

    fprintf(fid, '\nFrom blocks found: %d\n', numel(from_blocks));

    for n = 1:numel(from_blocks)
        fprintf(fid, '  %s\n', from_blocks{n});
    end
end

%% 输出模型顶层模块
fprintf(fid, '\n\n');
fprintf(fid, '==================================================\n');
fprintf(fid, 'MODEL TOP-LEVEL BLOCKS\n');
fprintf(fid, '==================================================\n');

top_blocks = find_system( ...
    mdl, ...
    'SearchDepth', 1, ...
    'Type', 'Block');

for k = 1:numel(top_blocks)
    blk = top_blocks{k};

    fprintf(fid, '\nBLOCK: %s\n', blk);
    fprintf(fid, '  BlockType: %s\n', safe_get_param(blk, 'BlockType'));
    fprintf(fid, '  MaskType: %s\n', safe_get_param(blk, 'MaskType'));
end

%% 输出所有顶层信号线
fprintf(fid, '\n\n');
fprintf(fid, '==================================================\n');
fprintf(fid, 'TOP-LEVEL SIGNAL LINES\n');
fprintf(fid, '==================================================\n');

top_lines = find_system( ...
    mdl, ...
    'SearchDepth', 1, ...
    'FindAll', 'on', ...
    'Type', 'line');

for k = 1:numel(top_lines)
    line_handle = top_lines(k);
    line_name = safe_get_param(line_handle, 'Name');

    try
        src = get_param(line_handle, 'SrcBlockHandle');
        dst = get_param(line_handle, 'DstBlockHandle');

        fprintf(fid, '\nSignal name: %s\n', line_name);

        if ~isempty(src) && src ~= -1
            fprintf(fid, '  Source: %s\n', getfullname(src));
        end

        if ~isempty(dst)
            dst = dst(dst ~= -1);

            for q = 1:numel(dst)
                fprintf(fid, '  Destination: %s\n', ...
                    getfullname(dst(q)));
            end
        end
    catch ME
        fprintf(fid, '  Inspection failed: %s\n', ME.message);
    end
end

fclose(fid);
fid = -1;

close_system(mdl, 0);

fprintf('\nSignal map saved successfully:\n%s\n', out_file);
end


%% 本地辅助函数

function value = safe_get_param(object, parameter)
    try
        value = get_param(object, parameter);

        if isempty(value)
            value = '<empty>';
        elseif ~ischar(value) && ~isstring(value)
            value = value_to_text(value);
        end
    catch
        value = '<not available>';
    end
end

function text = value_to_text(value)
    try
        if ischar(value)
            text = value;
        elseif isstring(value)
            text = char(value);
        elseif isnumeric(value) || islogical(value)
            text = mat2str(value);
        else
            text = '<non-text value>';
        end
    catch
        text = '<conversion failed>';
    end
end

function safe_close_file(fid)
    if isnumeric(fid) && isscalar(fid) && fid > 0
        try
            fclose(fid);
        catch
        end
    end
end

function safe_close_model(mdl)
    try
        if bdIsLoaded(mdl)
            close_system(mdl, 0);
        end
    catch
    end
end

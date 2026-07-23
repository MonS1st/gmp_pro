function update_lead_requirement(file, source, lead)
    % 解析JSON
    data = jsondecode(source);
    
    % 在requirements数组中查找SINV_FDRC_LEAD_STEPS
    reqIdx = [];
    for i = 1:length(data.requirements)
        if strcmp(data.requirements(i).macro, 'SINV_FDRC_LEAD_STEPS')
            reqIdx = i;
            break;
        end
    end
    
    % 如果找不到，报错
    if isempty(reqIdx)
        error('SINV:LeadSettingMissing', 'Could not find SINV_FDRC_LEAD_STEPS in %s.', file);
    end
    
    % 更新binding中的float值
    data.requirements(reqIdx).binding.float = lead;
    
    % 将修改后的数据编码为JSON字符串
    updated = jsonencode(data, 'PrettyPrint', true);
    
    % 写入文件
    fid = fopen(file, 'w');
    if fid < 0
        error('SINV:WriteFailed', 'Cannot write %s.', file);
    end
    fprintf(fid, '%s', updated);
    fclose(fid);
    
    % 显示更新信息
    fprintf('已更新 SINV_FDRC_LEAD_STEPS 为 %.1f\n', lead);
end
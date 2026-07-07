function plot_allocation_table(parent, scenario, sequences, distances, title_str)
%PLOT_ALLOCATION_TABLE 显示任务分配结果表（含异构参数信息）
%   输入：
%      parent     — 父容器（figure、uipanel 或 axes）
%      scenario   — 场景结构体
%      sequences  — [N×1] cell 数组
%      distances  — [N×1] 每架无人机飞行距离
%      title_str  — 可选标题

    if nargin < 5
        title_str = '任务分配结果';
    end

    N = scenario.N;
    L_max = scenario.L_max;

    % ---- 构建表格数据 ----
    has_sensors = isfield(scenario, 'sensor_types');
    data = cell(N + 1, 6 + has_sensors);
    for i = 1:N
        seq = sequences{i};
        col = 1;
        data{i, col} = sprintf('UAV%d', i); col = col + 1;
        if has_sensors
            sensor_names = {'光学', '雷达', '多光谱'};
            data{i, col} = sensor_names{min(scenario.sensor_types(i), 3)}; col = col + 1;
        end
        if isfield(scenario, 'uav_speed')
            data{i, col} = scenario.uav_speed(i); col = col + 1;
        end
        if isempty(seq)
            data{i, col} = '---'; col = col + 1;
            data{i, col} = 0; col = col + 1;
            data{i, col} = L_max(i); col = col + 1;
            data{i, col} = '0.0%'; col = col + 1;
        else
            data{i, col} = mat2str(seq(:)'); col = col + 1;
            data{i, col} = round(distances(i), 1); col = col + 1;
            data{i, col} = L_max(i); col = col + 1;
            util = distances(i) / L_max(i) * 100;
            data{i, col} = sprintf('%.1f%%', util); col = col + 1;
        end
    end

    % 汇总行
    total_dist = sum(distances);
    total_cap  = sum(L_max);
    col = 1;
    data{N + 1, col} = '合计'; col = col + 1;
    if has_sensors, data{N + 1, col} = ''; col = col + 1; end
    if isfield(scenario, 'uav_speed'), data{N + 1, col} = ''; col = col + 1; end
    data{N + 1, col} = ''; col = col + 1;
    data{N + 1, col} = round(total_dist, 1); col = col + 1;
    data{N + 1, col} = total_cap; col = col + 1;
    data{N + 1, col} = sprintf('%.1f%%', total_dist / total_cap * 100);

    % 列名
    colnames = {'无人机', '传感器', '速度', '目标序列', '距离', '最大航程', '利用率'};
    if ~has_sensors
        colnames = {'无人机', '目标序列', '距离', '最大航程', '利用率'};
    elseif ~isfield(scenario, 'uav_speed')
        colnames = {'无人机', '传感器', '目标序列', '距离', '最大航程', '利用率'};
    end

    columnWidths = {60, 55, 50, 160, 60, 65, 60};
    if ~has_sensors
        columnWidths = {60, 160, 60, 65, 60};
    elseif ~isfield(scenario, 'uav_speed')
        columnWidths = {60, 55, 160, 60, 65, 60};
    end

    % ---- 判断父容器类型 ----
    if isa(parent, 'matlab.graphics.axis.Axes')
        % axes 模式 → 文本输出
        cla(parent);
        text_str = {title_str, ''};
        header = sprintf('%-8s %-10s %-10s %-8s %-6s %s', colnames{:});
        text_str{end+1} = header;
        text_str{end+1} = repmat('-', 1, length(header));
        for i = 1:size(data, 1)
            line_str = '';
            for j = 1:length(colnames)
                line_str = [line_str, sprintf('%-10s', num2str(data{i,j}))]; %#ok<AGROW>
            end
            text_str{end+1} = line_str;
        end
        % 添加统计信息
        text_str{end+1} = '';
        if has_sensors
            text_str{end+1} = '传感器类型: 1=光学  2=雷达  3=多光谱';
        end

        text(parent, 0.02, 0.98, strjoin(text_str, newline), ...
            'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
            'FontName', 'Consolas', 'FontSize', 8);
        axis(parent, 'off');

    else
        % uipanel / figure 模式 → uitable
        delete(findobj(parent, 'Type', 'uitable'));
        nrow = N + 1;
        uitable(parent, ...
            'Data', data, ...
            'ColumnName', colnames, ...
            'ColumnWidth', columnWidths, ...
            'RowName', [], ...
            'Position', [20, 20, sum(cell2mat(columnWidths)) + 20, 30 * nrow]);

        annotation(parent, 'textbox', ...
            'String', title_str, ...
            'Position', [0.3, 0.95, 0.4, 0.05], ...
            'HorizontalAlignment', 'center', ...
            'FontSize', 12, 'FontWeight', 'bold', ...
            'EdgeColor', 'none');
    end
end

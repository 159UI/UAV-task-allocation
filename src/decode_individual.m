function [sequences, distances, fitness] = decode_individual(individual, scenario)
%DECODE_INDIVIDUAL 解码优先级向量为分配方案（增强版少UAV适配）
%   输入：
%      individual — [1×M] 实数向量（优先级编码）
%      scenario   — 场景结构体
%   输出：
%      sequences  — [N×1] cell，sequences{i} 为无人机 i 访问的目标序号
%      distances  — [N×1] 每架无人机的实际飞行距离
%      fitness    — 标量，目标函数值 J
%
%   解码流程：
%     1. 按优先级从大到小排序目标
%     2. 贪婪分配：优先传感器匹配 → 选最小新增距离 → 留最大余量
%     3. 未分配目标→第二轮宽松匹配
%     4. 计算适应度

    M = scenario.M;
    N = scenario.N;
    L_max = scenario.L_max;
    base  = scenario.base;
    obstacles = scenario.obstacles;
    d_safe = scenario.d_safe;
    P_val  = scenario.P;

    % ---- 1. 按优先级降序 ----
    [~, order] = sort(individual, 'descend');

    % ---- 2. 初始化 ----
    sequences = cell(N, 1);
    for i = 1:N, sequences{i} = []; end
    distances = zeros(N, 1);
    assigned = false(M, 1);  % 标记是否已分配

    % ---- 传感器兼容 ----
    has_sensor = isfield(scenario, 'sensor_types') && isfield(scenario, 'target_sensors');
    if ~has_sensor
        scenario.sensor_types = ones(N, 1);
        scenario.target_sensors = ones(M, 1);
    end

    % ===== 第一轮：严格贪婪分配 =====
    for idx = 1:M
        t = order(idx);

        % 传感器匹配
        req = scenario.target_sensors(t);
        compat = find(scenario.sensor_types >= req);
        if isempty(compat), continue; end

        target_pos = scenario.targets(t, :);
        d_out = distance_penalty(base, target_pos, obstacles, d_safe, P_val);

        best_uav = -1;
        best_new = inf;
        best_rem = -inf;  % 剩余余量（越大越好）

        for ki = 1:length(compat)
            i = compat(ki);
            if isempty(sequences{i})
                new_dist = 2 * d_out;
            else
                last = scenario.targets(sequences{i}(end), :);
                d_extra = distance_penalty(last, target_pos, obstacles, d_safe, P_val);
                d_back = distance_penalty(target_pos, base, obstacles, d_safe, P_val);
                old_ret = distance_penalty(last, base, obstacles, d_safe, P_val);
                new_dist = distances(i) - old_ret + d_extra + d_back;
            end

            if new_dist <= L_max(i)
                remain = L_max(i) - new_dist;
                % 优先选新增距离小的，相同距离选余量大的
                if new_dist < best_new - 1e-6 || ...
                   (abs(new_dist - best_new) < 1e-6 && remain > best_rem)
                    best_new = new_dist;
                    best_uav = i;
                    best_rem = remain;
                end
            end
        end

        if best_uav > 0
            sequences{best_uav}(end+1) = t;
            distances(best_uav) = best_new;
            assigned(t) = true;
        end
    end

    % ===== 第二轮：对未分配目标尝试再次分配 =====
    unassigned = find(~assigned);
    if ~isempty(unassigned)
        % 按当前各 UAV 剩余航程从大到小处理
        remaining = L_max - distances;

        for u = 1:length(unassigned)
            t = unassigned(u);
            req = scenario.target_sensors(t);
            compat = find(scenario.sensor_types >= req);
            if isempty(compat), continue; end

            target_pos = scenario.targets(t, :);

            % 找剩余航程最大的兼容UAV
            best_uav = -1;
            best_new = inf;

            for ki = 1:length(compat)
                i = compat(ki);
                if remaining(i) < 1, continue; end  % 完全没余量了

                if isempty(sequences{i})
                    d_out = distance_penalty(base, target_pos, obstacles, d_safe, P_val);
                    new_dist = 2 * d_out;
                else
                    last = scenario.targets(sequences{i}(end), :);
                    d_extra = distance_penalty(last, target_pos, obstacles, d_safe, P_val);
                    d_back = distance_penalty(target_pos, base, obstacles, d_safe, P_val);
                    old_ret = distance_penalty(last, base, obstacles, d_safe, P_val);
                    new_dist = distances(i) - old_ret + d_extra + d_back;
                end

                if new_dist <= L_max(i) && new_dist < best_new
                    best_new = new_dist;
                    best_uav = i;
                end
            end

            if best_uav > 0
                sequences{best_uav}(end+1) = t;
                distances(best_uav) = best_new;
                remaining(best_uav) = L_max(best_uav) - best_new;
                assigned(t) = true;
            end
        end
    end

    % ===== 第三轮：尝试在序列中插入未分配目标（最优位置插入） =====
    changed = true;
    while changed
        changed = false;
        unassigned = find(~assigned);
        if isempty(unassigned), break; end

        for u = 1:length(unassigned)
            t = unassigned(u);
            req = scenario.target_sensors(t);
            compat = find(scenario.sensor_types >= req);
            if isempty(compat), continue; end

            target_pos = scenario.targets(t, :);

            for ki = 1:length(compat)
                i = compat(ki);
                current_seq = sequences{i};
                current_len = length(current_seq);

                % 尝试在序列的每个位置插入（从0到末尾）
                best_pos = -1;
                best_new_dist = inf;

                for pos = 0:current_len
                    % pos=0 → 插在最前（基地之后）
                    % pos=current_len → 插在最后（基地之前）
                    test_seq = [current_seq(1:pos), t, current_seq(pos+1:end)];

                    % 计算test_seq的总距离
                    test_dist = 0;
                    pts = [base; scenario.targets(test_seq, :); base];
                    ok = true;
                    for seg = 1:length(pts)-1
                        seg_d = distance_penalty(pts(seg,:), pts(seg+1,:), ...
                            obstacles, d_safe, P_val);
                        test_dist = test_dist + seg_d;
                        if test_dist > L_max(i)
                            ok = false;
                            break;
                        end
                    end

                    if ok && test_dist < best_new_dist
                        best_new_dist = test_dist;
                        best_pos = pos;
                    end
                end

                if best_pos >= 0
                    % 插入到最佳位置
                    sequences{i} = [current_seq(1:best_pos), t, current_seq(best_pos+1:end)];
                    distances(i) = best_new_dist;
                    assigned(t) = true;
                    changed = true;
                    break;  % 此目标已分配，换下一个
                end
            end
        end
    end

    % ===== 第四轮：最近邻重排序（少UAV时，用NN重构全路径） =====
    unassigned = find(~assigned);
    if N <= 3 && ~isempty(unassigned)
        for i = 1:N
            current_set = sequences{i};
            all_in_set = unique([current_set, unassigned']);
            if length(all_in_set) <= length(current_set), continue; end

            % 最近邻构造：从基地出发，每次找最近的未访问目标
            ordered = [];
            pos = base;
            remaining_set = all_in_set;
            total = 0;

            while ~isempty(remaining_set)
                % 找离当前位置最近的目标
                best_j = -1;
                best_d = inf;
                for j = 1:length(remaining_set)
                    tj = remaining_set(j);
                    d = distance_penalty(pos, scenario.targets(tj,:), obstacles, d_safe, P_val);
                    if d < best_d
                        best_d = d;
                        best_j = j;
                    end
                end
                tid = remaining_set(best_j);

                % 检查加上这个目标后，回基地的总距离是否超限
                return_to_base = distance_penalty(scenario.targets(tid,:), base, obstacles, d_safe, P_val);
                test_total = total + best_d + return_to_base;

                if test_total <= L_max(i)
                    ordered(end+1) = tid;
                    pos = scenario.targets(tid,:);
                    total = total + best_d;
                    remaining_set(best_j) = [];
                else
                    % 不能加这个目标，从候选集移除（但仍然可能加其他更近的）
                    remaining_set(best_j) = [];
                end
            end

            % 若NN找到更多目标，采用NN结果
            if length(ordered) > length(current_set)
                sequences{i} = ordered;
                % 重算距离
                total = 0;
                pos = base;
                for j = 1:length(ordered)
                    total = total + distance_penalty(pos, scenario.targets(ordered(j),:), obstacles, d_safe, P_val);
                    pos = scenario.targets(ordered(j),:);
                end
                total = total + distance_penalty(pos, base, obstacles, d_safe, P_val);
                distances(i) = total;
            end
        end
    end
    % 第四轮结束后统一更新assigned
    assigned(:) = false;
    for ii = 1:N
        for jj = 1:length(sequences{ii})
            assigned(sequences{ii}(jj)) = true;
        end
    end

    % ---- 计算适应度 ----
    fitness = calc_objective(scenario, sequences, distances);
end

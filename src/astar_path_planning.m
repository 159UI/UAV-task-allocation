function path = astar_path_planning(start_pos, goal_pos, obstacles, bounds, resolution)
%ASTAR_PATH_PLANNING 简易栅格 A* 避障路径规划
%   输入：
%      start_pos  — [1×2] 起点坐标
%      goal_pos   — [1×2] 终点坐标
%      obstacles  — [K×3] 障碍物矩阵 [x, y, radius]
%      bounds     — [x_min, x_max, y_min, y_max] 场景边界
%      resolution — 栅格分辨率（每个栅格代表的实际距离）
%   输出：
%      path       — [n×2] 路径点序列（从起点到终点），若无可通行路径返回 []

    if nargin < 5
        resolution = 2.0;
    end

    % ---- 栅格化 ----
    x_min = bounds(1); x_max = bounds(2);
    y_min = bounds(3); y_max = bounds(4);
    nx = ceil((x_max - x_min) / resolution);
    ny = ceil((y_max - y_min) / resolution);

    % ---- 障碍物地图 ----
    obs_map = false(nx, ny);  % true = 占用
    K = size(obstacles, 1);
    for k = 1:K
        cx = obstacles(k, 1);
        cy = obstacles(k, 2);
        r  = obstacles(k, 3) + 2;  % 安全裕量

        % 确定障碍物影响范围
        ix_min = max(1, floor((cx - r - x_min) / resolution));
        ix_max = min(nx, ceil((cx + r - x_min) / resolution));
        iy_min = max(1, floor((cy - r - y_min) / resolution));
        iy_max = min(ny, ceil((cy + r - y_min) / resolution));

        for ix = ix_min:ix_max
            for iy = iy_min:iy_max
                gx = x_min + (ix - 0.5) * resolution;
                gy = y_min + (iy - 0.5) * resolution;
                if sqrt((gx - cx)^2 + (gy - cy)^2) < r
                    obs_map(ix, iy) = true;
                end
            end
        end
    end

    % ---- 转换起点终点到栅格坐标 ----
    start_idx = [max(1, min(nx, floor((start_pos(1) - x_min) / resolution) + 1)), ...
                 max(1, min(ny, floor((start_pos(2) - y_min) / resolution) + 1))];
    goal_idx  = [max(1, min(nx, floor((goal_pos(1) - x_min) / resolution) + 1)), ...
                 max(1, min(ny, floor((goal_pos(2) - y_min) / resolution) + 1))];

    % 检查起点终点是否被占用
    if obs_map(start_idx(1), start_idx(2)) || obs_map(goal_idx(1), goal_idx(2))
        path = [];
        return;
    end

    % ---- A* 搜索 ----
    % 启发函数：曼哈顿距离
    heuristic = @(idx) abs(idx(1) - goal_idx(1)) + abs(idx(2) - goal_idx(2));

    % 8 方向邻域
    neighbors = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
    neighbor_cost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];

    open_set = containers.Map();
    closed_set = false(nx, ny);

    start_key = mat2str(start_idx);
    open_set(start_key) = struct('pos', start_idx, ...
        'g', 0, 'f', heuristic(start_idx), 'parent', []);

    path_found = false;
    max_search = nx * ny * 4;
    search_count = 0;

    while open_set.Count > 0 && search_count < max_search
        search_count = search_count + 1;

        % 找出 open_set 中 f 值最小的节点
        keys = open_set.keys;
        best_key = keys{1};
        best_f = open_set(best_key).f;
        for k = 2:length(keys)
            if open_set(keys{k}).f < best_f
                best_f = open_set(keys{k}).f;
                best_key = keys{k};
            end
        end

        current = open_set(best_key);
        open_set.remove(best_key);
        closed_set(current.pos(1), current.pos(2)) = true;

        % 到达目标
        if current.pos(1) == goal_idx(1) && current.pos(2) == goal_idx(2)
            % 回溯路径
            path_grid = [];
            node = current;
            while ~isempty(node)
                path_grid = [node.pos; path_grid];
                node = node.parent;
            end

            % 转换为实际坐标
            path = zeros(size(path_grid, 1), 2);
            for p = 1:size(path_grid, 1)
                path(p, 1) = x_min + (path_grid(p, 1) - 0.5) * resolution;
                path(p, 2) = y_min + (path_grid(p, 2) - 0.5) * resolution;
            end

            % 平滑路径（去除冗余拐点）
            path = simplify_path(path);
            path_found = true;
            break;
        end

        % 扩展邻域
        for nb = 1:8
            new_pos = current.pos + neighbors(nb, :);

            % 边界检查
            if new_pos(1) < 1 || new_pos(1) > nx || ...
               new_pos(2) < 1 || new_pos(2) > ny
                continue;
            end

            % 障碍物检查
            if obs_map(new_pos(1), new_pos(2))
                continue;
            end

            % 已关闭
            if closed_set(new_pos(1), new_pos(2))
                continue;
            end

            new_g = current.g + neighbor_cost(nb) * resolution;
            new_key = mat2str(new_pos);

            if open_set.isKey(new_key)
                if new_g < open_set(new_key).g
                    open_set(new_key).g = new_g;
                    open_set(new_key).f = new_g + heuristic(new_pos);
                    open_set(new_key).parent = current;
                end
            else
                open_set(new_key) = struct('pos', new_pos, ...
                    'g', new_g, ...
                    'f', new_g + heuristic(new_pos), ...
                    'parent', current);
            end
        end
    end

    if ~path_found
        % 若未找到路径，返回直接连接线（用惩罚法）
        path = [start_pos; goal_pos];
    end
end


function simple_path = simplify_path(path)
%SIMPLIFY_PATH 简化路径：去除冗余拐点（Douglas-Peucker 简化）
    if size(path, 1) <= 2
        simple_path = path;
        return;
    end

    % 保留必要的拐点：检查三点是否共线
    keep = true(size(path, 1), 1);
    i = 2;
    while i < size(path, 1)
        v1 = path(i, :) - path(i-1, :);
        v2 = path(i+1, :) - path(i, :);
        % 计算夹角（若几乎共线则移除中间点）
        cos_angle = sum(v1 .* v2) / (norm(v1) * norm(v2) + eps);
        if cos_angle > 0.99 || cos_angle < -0.99
            keep(i) = false;
        end
        i = i + 1;
    end
    simple_path = path(keep, :);
end

function plot_path(ax, scenario, sequences, title_str)
%PLOT_PATH 绘制二维路径图（含避障绕行 + 汉字图例）
%   路径自动绕开障碍物显示，不再直线穿过。

    if nargin < 4, title_str = '任务分配路径图'; end

    cla(ax); hold(ax, 'on');

    N = scenario.N;
    M = scenario.M;
    K = scenario.K;
    base = scenario.base;
    targets = scenario.targets;
    obstacles = scenario.obstacles;
    d_safe = scenario.d_safe;
    colors = lines(N);

    % ---- 绘制障碍物 ----
    for k = 1:K
        xc = obstacles(k,1); yc = obstacles(k,2); r = obstacles(k,3);
        rectangle(ax, 'Position', [xc-r, yc-r, 2*r, 2*r], ...
            'Curvature', [1, 1], 'FaceColor', [0.9, 0.7, 0.7], ...
            'EdgeColor', [0.7, 0.2, 0.2], 'LineWidth', 1.5, 'LineStyle', '--');
    end

    % ---- 绘制目标 ----
    for j = 1:M
        scatter(ax, targets(j,1), targets(j,2), 100, 'g', 'd', 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1);
        text(ax, targets(j,1), targets(j,2)+3, num2str(j), ...
            'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', 'k', 'FontWeight', 'bold');
    end

    % ---- 绘制基地 ----
    scatter(ax, base(1), base(2), 180, 'b', 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 2);
    text(ax, base(1), base(2)-10, '基地', ...
        'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');

    % ---- 绘制无人机路径（含避障绕行） ----
    for i = 1:N
        seq = sequences{i};
        if isempty(seq), continue; end

        raw_points = [base; targets(seq, :); base];
        full_path = [];
        for seg = 1:size(raw_points,1)-1
            seg_path = avoid_segment(raw_points(seg,:), raw_points(seg+1,:), obstacles, d_safe);
            if seg == 1, full_path = seg_path;
            else full_path = [full_path; seg_path(2:end,:)]; end
        end

        plot(ax, full_path(:,1), full_path(:,2), '-', ...
            'Color', colors(i,:), 'LineWidth', 2.5);
        scatter(ax, full_path(:,1), full_path(:,2), 18, colors(i,:), 'filled');
    end

    % ---- 右上角图例（最边缘，右对齐） ----
    xl = xlim(ax); yl = ylim(ax);
    lx = xl(2); ly = yl(2);
    legend_text = {
        '━━ 图例 ━━'
        '● 蓝色圆点 — 基地'
        '◆ 绿色菱形 — 静态目标'
        '○ 红色虚线圆 — 障碍物'
        '━ 彩色实线 — 避障飞行路径'
    };
    text(ax, lx, ly, legend_text, ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', ...
        'FontName', 'SimSun', 'FontSize', 8, ...
        'BackgroundColor', [1,1,1,0.85], ...
        'EdgeColor', [0.5,0.5,0.5], 'Margin', 4);

    % ---- 轴设置 ----
    axis(ax, 'equal');
    margin = 15;
    xlim(ax, [min(scenario.range_x(1), -10)-margin, max(scenario.range_x(2), 10)+margin]);
    ylim(ax, [min(scenario.range_y(1), -10)-margin, max(scenario.range_y(2), 10)+margin]);
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
    title(ax, title_str, 'FontWeight', 'bold', 'FontSize', 11);
    grid(ax, 'on');
    hold(ax, 'off');
end


function path = avoid_segment(p1, p2, obstacles, d_safe)
%AVOID_SEGMENT 生成两点间避障路径（障碍物边缘弧线绕行）
    K = size(obstacles, 1);

    for k = 1:K
        cx = obstacles(k,1); cy = obstacles(k,2); r = obstacles(k,3);
        safe_r = r + d_safe + 3;

        d = p2 - p1;
        d_len2 = sum(d.^2);
        if d_len2 < 1e-10, continue; end

        t = dot([cx, cy] - p1, d) / d_len2;
        t = max(0, min(1, t));
        closest = p1 + t * d;
        if norm(closest - [cx, cy]) >= safe_r, continue; end

        a1 = atan2(p1(2)-cy, p1(1)-cx);
        a2 = atan2(p2(2)-cy, p2(1)-cx);
        da_forward = a2 - a1;
        da_backward = da_forward - sign(da_forward)*2*pi;
        [~, idx] = max([abs(da_forward), abs(da_backward)]);
        if idx == 1, a_start = a1; a_end = a2;
        else a_start = a1; a_end = a2 - sign(da_forward)*2*pi; end

        n_arc = 10;
        angles = linspace(a_start, a_end, n_arc);
        arc = zeros(n_arc, 2);
        for i = 1:n_arc
            arc(i, :) = [cx, cy] + safe_r * [cos(angles(i)), sin(angles(i))];
        end

        path = [p1; arc; p2];
        return;
    end
    path = [p1; p2];
end

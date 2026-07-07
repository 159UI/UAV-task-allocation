function plot_convergence(ax, sa_history, cs_history, sa_runtime, cs_runtime, varargin)
%PLOT_CONVERGENCE 绘制收敛曲线（SA 和 CS 同图对比）
%   输入：
%      ax          — 坐标轴句柄
%      sa_history  — SA 适应度历史数组
%      cs_history  — CS 适应度历史数组
%      sa_runtime  — SA 运行时间（秒）
%      cs_runtime  — CS 运行时间（秒）
%    可选参数：
%      'title_str'  — 标题，默认 '收敛曲线对比'
%      'show_best'  — 是否标注最优值，默认 true

    p = inputParser;
    addParameter(p, 'title_str', '收敛曲线对比');
    addParameter(p, 'show_best', true);
    parse(p, varargin{:});
    opts = p.Results;

    cla(ax); hold(ax, 'on');

    % ---- SA 曲线 ----
    if ~isempty(sa_history)
        iter_sa = 1:length(sa_history);
        plot(ax, iter_sa, sa_history, 'b-', 'LineWidth', 2, ...
            'DisplayName', sprintf('SA (%.2fs)', sa_runtime));
    end

    % ---- CS 曲线 ----
    if ~isempty(cs_history)
        iter_cs = 1:length(cs_history);
        plot(ax, iter_cs, cs_history, 'r-', 'LineWidth', 2, ...
            'DisplayName', sprintf('CS (%.2fs)', cs_runtime));
    end

    % ---- 标注最优值 ----
    if opts.show_best
        if ~isempty(sa_history)
            [best_sa, idx_sa] = max(sa_history);
            scatter(ax, idx_sa, best_sa, 60, 'b', 'filled');
            text(ax, idx_sa, best_sa, sprintf(' SA最优: %.1f', best_sa), ...
                'FontSize', 9, 'Color', 'b');
        end
        if ~isempty(cs_history)
            [best_cs, idx_cs] = max(cs_history);
            scatter(ax, idx_cs, best_cs, 60, 'r', 'filled');
            text(ax, idx_cs, best_cs, sprintf(' CS最优: %.1f', best_cs), ...
                'FontSize', 9, 'Color', 'r');
        end
    end

    % ---- 轴设置 ----
    xlabel(ax, '迭代次数');
    ylabel(ax, '最优适应度 J');
    title(ax, opts.title_str, 'FontWeight', 'bold', 'FontSize', 11);
    legend(ax, 'Location', 'southeast', 'FontSize', 9);
    grid(ax, 'on');
    hold(ax, 'off');
end

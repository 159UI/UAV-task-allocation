function fitness = calc_objective(scenario, sequences, distances)
%CALC_OBJECTIVE 计算目标函数值
%   输入：
%      scenario   — 场景结构体
%      sequences  — [N×1] cell 数组，每架无人机的目标序号序列（可为空）
%      distances  — [N×1] 每架无人机的飞行距离
%   输出：
%      fitness    — 标量，目标函数值 J = sum(R_j) - ω·sum(L_i/L_max)
%
%   目标函数（最大化）：
%     J = 总访问收益 - ω·∑(L_i / L_max_i)
%
%   SA 和 CS 都通过此函数评估解的质量

    N = scenario.N;
    omega = scenario.omega;
    L_max = scenario.L_max;
    rewards = scenario.rewards;

    % ---- 第一项：总收益 ----
    visited_targets = [];
    for i = 1:N
        visited_targets = [visited_targets; sequences{i}(:)];
    end
    total_reward = sum(rewards(visited_targets));

    % ---- 第二项：航程惩罚 ----
    total_dist_ratio = 0;
    for i = 1:N
        total_dist_ratio = total_dist_ratio + distances(i) / L_max(i);
    end

    % ---- 适应度 ----
    fitness = total_reward - omega * total_dist_ratio;
end

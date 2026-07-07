function result = simulated_annealing(scenario, params)
%SIMULATED_ANNEALING 模拟退火算法主循环
%   输入：
%      scenario — 场景结构体（含 N, M, K, targets, rewards, obstacles, L_max 等）
%      params   — 结构体，包含 SA 参数：
%        .T0       — 初始温度
%        .alpha    — 降温速率 (0.8~0.99)
%        .T_end    — 终止温度
%        .max_iter — 最大迭代次数
%   输出：
%      result — 结构体：
%        .best_individual  — 最优解（优先级向量）
%        .best_sequence    — 最优分配方案 [N×1 cell]
%        .best_distances   — 最优飞行距离 [N×1]
%        .best_fitness     — 最优适应度
%        .fitness_history  — [max_iter×1] 收敛曲线
%        .temp_history     — [max_iter×1] 温度变化
%        .runtime          — 运行时间（秒）
%
%   算法步骤：
%     1. 随机生成初始解
%     2. 迭代：三选一邻域操作（Swap/Insert/Reverse）
%     3. Metropolis 准则决定是否接受
%     4. 指数降温 T_{k+1} = alpha * T_k
%     5. 终止条件：达最大迭代或温度低于 T_end

    t_start = tic;

    % ---- 参数 ----
    M = scenario.M;
    if M < 2
        error('目标数量 M 必须 ≥ 2');
    end

    T0      = params.T0;
    alpha   = params.alpha;
    T_end   = params.T_end;
    max_iter = params.max_iter;

    % ---- 1. 初始化解 ----
    current_indiv = 10 * (2 * rand(1, M) - 1);  % [-10, 10]
    [~, ~, current_fitness] = decode_individual(current_indiv, scenario);

    best_indiv = current_indiv;
    [best_seq, best_dist, best_fitness] = decode_individual(best_indiv, scenario);

    % ---- 2. 记录 ----
    fitness_history = zeros(max_iter, 1);
    temp_history   = zeros(max_iter, 1);
    fitness_history(1) = best_fitness;
    temp_history(1)    = T0;

    % ---- 3. SA 主循环 ----
    T = T0;
    no_improve_count = 0;

    for iter = 2:max_iter
        % ---- 生成邻域解 ----
        new_indiv = neighbor_operation(current_indiv);

        % ---- 评估新解 ----
        [~, ~, new_fitness] = decode_individual(new_indiv, scenario);
        delta = new_fitness - current_fitness;

        % ---- Metropolis 准则 ----
        if delta > 0 || (T > eps && rand() < exp(delta / T))
            current_indiv = new_indiv;
            current_fitness = new_fitness;

            % 更新全局最优
            if current_fitness > best_fitness
                best_indiv = current_indiv;
                [best_seq, best_dist, best_fitness] = decode_individual(best_indiv, scenario);
                no_improve_count = 0;
            else
                no_improve_count = no_improve_count + 1;
            end
        else
            no_improve_count = no_improve_count + 1;
        end

        % ---- 降温 ----
        T = alpha * T;
        temp_history(iter) = T;

        % ---- 记录历史 ----
        [~, ~, iter_best] = decode_individual(best_indiv, scenario);
        fitness_history(iter) = iter_best;

        % ---- 提前终止 ----
        if T < T_end
            fitness_history(iter+1:end) = [];
            temp_history(iter+1:end) = [];
            break;
        end
    end

    % ---- 整理输出 ----
    runtime = toc(t_start);
    result = struct();
    result.best_individual = best_indiv;
    result.best_sequence   = best_seq;      % [N×1] cell，sequences{i} 为 UAV i 的目标序列
    result.best_distances  = best_dist;
    result.best_fitness    = best_fitness;
    result.fitness_history = fitness_history;
    result.temp_history    = temp_history;
    result.runtime         = runtime;
    result.algorithm       = 'SA';
end


% ======================== 邻域操作 ========================
function new_indiv = neighbor_operation(indiv)
% 随机选择一种邻域操作：Swap / Insert / Reverse
    M = length(indiv);
    new_indiv = indiv;
    op = randi(3);

    switch op
        case 1  % Swap：交换两个随机位置
            idx = randperm(M, 2);
            new_indiv(idx([1,2])) = new_indiv(idx([2,1]));

        case 2  % Insert：取出一个插入到另一位置
            from = randi(M);
            to   = randi(M);
            val = new_indiv(from);
            new_indiv(from) = [];
            if to > length(new_indiv)
                new_indiv(end+1) = val;
            else
                new_indiv = [new_indiv(1:to-1), val, new_indiv(to:end)];
            end

        case 3  % Reverse：反转一段连续子序列
            seg = sort(randperm(M, 2));
            new_indiv(seg(1):seg(2)) = fliplr(new_indiv(seg(1):seg(2)));
    end
end

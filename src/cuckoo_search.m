function result = cuckoo_search(scenario, params)
%CUCKOO_SEARCH 布谷鸟搜索算法主循环
%   输入：
%      scenario — 场景结构体
%      params   — 结构体，包含 CS 参数：
%        .N_pop      — 种群规模（巢穴数量）
%        .Pa         — 发现概率 (0.1~0.5)
%        .alpha_step — 莱维飞行步长缩放因子
%        .max_iter   — 最大迭代次数
%   输出：
%      result — 结构体：
%        .best_individual  — 最优解（优先级向量）
%        .best_sequence    — 最优分配方案
%        .best_distances   — 最优飞行距离
%        .best_fitness     — 最优适应度
%        .fitness_history  — 收敛曲线
%        .runtime          — 运行时间（秒）
%
%   算法步骤：
%     1. 初始化 N_pop 个随机巢穴
%     2. 莱维飞行：全局探索，生成新巢穴
%     3. 发现与丢弃：以概率 Pa 丢弃劣质解，偏好随机游走生成新解
%     4. 终止条件：达最大迭代次数

    t_start = tic;

    % ---- 参数 ----
    M = scenario.M;
    if M < 2
        error('目标数量 M 必须 ≥ 2');
    end

    N_pop      = params.N_pop;
    Pa         = params.Pa;
    alpha_step = params.alpha_step;
    max_iter   = params.max_iter;

    % ---- 1. 初始化种群 ----
    nests = 10 * (2 * rand(N_pop, M) - 1);   % [-10, 10]
    fitness = zeros(N_pop, 1);

    for i = 1:N_pop
        [~, ~, fitness(i)] = decode_individual(nests(i, :), scenario);
    end

    [best_fitness, best_idx] = max(fitness);
    best_indiv = nests(best_idx, :);
    [best_seq, best_dist, ~] = decode_individual(best_indiv, scenario);

    % ---- 记录 ----
    fitness_history = zeros(max_iter, 1);
    fitness_history(1) = best_fitness;

    % ---- 2. CS 主循环 ----
    for iter = 2:max_iter
        % ========== 阶段一：莱维飞行（全局探索） ==========
        for i = 1:N_pop
            % 生成莱维步长
            levy_step = levy_flight(1.5, M);
            % 更新巢穴位置
            new_nest = nests(i, :) + alpha_step * levy_step .* (nests(i, :) - best_indiv);
            % 边界限制
            new_nest = max(min(new_nest, 10), -10);
            % 评估新解
            [~, ~, new_fitness] = decode_individual(new_nest, scenario);
            % 贪婪选择
            if new_fitness > fitness(i)
                nests(i, :) = new_nest;
                fitness(i)  = new_fitness;
            end
        end

        % ========== 阶段二：发现与丢弃（局部开发） ==========
        for i = 1:N_pop
            if rand() < Pa
                % 随机选择三个不同的巢穴
                idxs = randperm(N_pop, 3);
                while any(idxs(1) == idxs(2)) || any(idxs(1) == idxs(3)) || any(idxs(2) == idxs(3))
                    idxs = randperm(N_pop, 3);
                end
                % 偏好随机游走
                r = rand();
                new_nest = nests(idxs(1), :) + r * (nests(idxs(2), :) - nests(idxs(3), :));
                % 边界限制
                new_nest = max(min(new_nest, 10), -10);
                % 评估
                [~, ~, new_fitness] = decode_individual(new_nest, scenario);
                if new_fitness > fitness(i)
                    nests(i, :) = new_nest;
                    fitness(i)  = new_fitness;
                end
            end
        end

        % ========== 更新全局最优 ==========
        [current_best, best_idx] = max(fitness);
        if current_best > best_fitness
            best_fitness = current_best;
            best_indiv = nests(best_idx, :);
            [best_seq, best_dist, ~] = decode_individual(best_indiv, scenario);
        end

        fitness_history(iter) = best_fitness;
    end

    % ---- 整理输出 ----
    runtime = toc(t_start);
    result = struct();
    result.best_individual = best_indiv;
    result.best_sequence   = best_seq;      % [N×1] cell，sequences{i} 为 UAV i 的目标序列
    result.best_distances  = best_dist;
    result.best_fitness    = best_fitness;
    result.fitness_history = fitness_history;
    result.runtime         = runtime;
    result.algorithm       = 'CS';
end

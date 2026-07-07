function sim_result = simulate_dynamic(scenario, sequences, distances)
%SIMULATE_DYNAMIC 简易路径跟踪（静态场景）
%   对 SA/CS 求解出的分配方案，按规划路径记录轨迹点

    N = scenario.N;
    base = scenario.base;

    uav_trails = cell(N, 1);
    for i = 1:N
        uav_trails{i} = [base];
        seq = sequences{i};
        if isempty(seq), continue; end
        for j = 1:length(seq)
            uav_trails{i}(end+1,:) = scenario.targets(seq(j), :);
        end
        uav_trails{i}(end+1,:) = base;
    end

    actual_reward = 0;
    visited = false(scenario.M, 1);
    for i = 1:N
        for j = 1:length(sequences{i})
            t = sequences{i}(j);
            if ~visited(t)
                visited(t) = true;
                actual_reward = actual_reward + scenario.rewards(t);
            end
        end
    end

    cost_ratio = 0;
    for i = 1:N
        cost_ratio = cost_ratio + distances(i) / scenario.L_max(i);
    end

    sim_result = struct();
    sim_result.uav_trails = uav_trails;
    sim_result.actual_reward = actual_reward;
    sim_result.actual_fitness = actual_reward - scenario.omega * cost_ratio;
    sim_result.target_visited = visited;
    sim_result.total_steps = 1;
    sim_result.total_frames = 1;
    sim_result.history_uav_pos = [];
    sim_result.target_capture_time = [];
    sim_result.collisions = 0;
end

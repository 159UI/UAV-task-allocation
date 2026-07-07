function scenario = generate_scenario(N, M, K, varargin)
%GENERATE_SCENARIO 生成静态海事巡检场景
%   输入：
%      N  — 无人机数量
%      M  — 静态目标数量
%      K  — 静态障碍物数量
%    可选参数：
%      'L_max'        — 最大航程，标量或 [N×1]，默认 200
%      'uav_speed'    — 巡航速度，标量或 [N×1]，默认 8
%      'sensor_types' — 传感器类型 [N×1]，默认随机 1~3
%      'd_safe'       — 安全距离，默认 2
%      'P'            — 惩罚常数，默认 50
%      'omega'        — 航程权重，默认 0.3
%      'range_x'      — X 范围，默认 [-50, 50]
%      'range_y'      — Y 范围，默认 [0, 100]
%      'obs_radius'   — 障碍物半径 [min, max]，默认 [3, 8]
%      'reward_range' — 收益范围 [min, max]，默认 [1, 10]
%      'seed'         — 随机种子
%   输出：
%      scenario — 场景结构体（所有目标/障碍物均为静态）

    p = inputParser;
    addParameter(p, 'L_max', 200);
    addParameter(p, 'uav_speed', 8);
    addParameter(p, 'sensor_types', []);
    addParameter(p, 'd_safe', 2);
    addParameter(p, 'P', 50);
    addParameter(p, 'omega', 0.3);
    addParameter(p, 'range_x', [-50, 50]);
    addParameter(p, 'range_y', [0, 100]);
    addParameter(p, 'obs_radius', [3, 8]);
    addParameter(p, 'reward_range', [1, 10]);
    addParameter(p, 'seed', []);
    parse(p, varargin{:});
    opts = p.Results;

    if ~isempty(opts.seed), rng(opts.seed); else, rng('shuffle'); end

    % 基础参数
    scenario.N = N;
    scenario.M = M;
    scenario.K = K;
    scenario.base = [0, 0];
    scenario.d_safe = opts.d_safe;
    scenario.P = opts.P;
    scenario.omega = opts.omega;
    scenario.range_x = opts.range_x;
    scenario.range_y = opts.range_y;

    % 无人机参数
    if isscalar(opts.L_max)
        scenario.L_max = opts.L_max * ones(N, 1);
    else
        scenario.L_max = opts.L_max(:);
    end
    if isscalar(opts.uav_speed)
        scenario.uav_speed = opts.uav_speed * ones(N, 1);
    else
        scenario.uav_speed = opts.uav_speed(:);
    end
    if isempty(opts.sensor_types)
        scenario.sensor_types = randi([1, 3], N, 1);
    else
        scenario.sensor_types = opts.sensor_types(:);
    end

    rx = opts.range_x;
    ry = opts.range_y;
    rr = opts.obs_radius;

    % --- 生成障碍物（先放，目标靠后） ---
    all_centers = zeros(K, 2);
    scenario.obstacles = zeros(K, 3);
    for k = 1:K
        placed = false;
        for tries = 1:200
            cx = rx(1) + (rx(2)-rx(1))*rand();
            cy = ry(1) + (ry(2)-ry(1))*rand();
            r_obs = rr(1) + (rr(2)-rr(1))*rand();
            ok = true;
            if sqrt(cx^2+cy^2) < r_obs + 10, ok = false; end
            if ok && k > 1
                if any(sqrt(sum((all_centers(1:k-1,:)-[cx,cy]).^2,2)) < r_obs+rr(2)+opts.d_safe)
                    ok = false;
                end
            end
            if ok
                scenario.obstacles(k,:) = [cx, cy, r_obs];
                all_centers(k,:) = [cx, cy];
                placed = true; break;
            end
        end
        if ~placed
            scenario.obstacles(k,:) = [rx(1)+(rx(2)-rx(1))*rand(), ...
                ry(1)+(ry(2)-ry(1))*rand(), rr(1)+(rr(2)-rr(1))*rand()];
        end
    end

    % --- 生成目标（不与障碍物重叠） ---
    scenario.targets = zeros(M, 2);
    scenario.rewards = opts.reward_range(1) + (opts.reward_range(2)-opts.reward_range(1))*rand(M,1);
    scenario.target_sensors = randi([1,3], M, 1);

    for j = 1:M
        for tries = 1:200
            tx = rx(1) + (rx(2)-rx(1))*rand();
            ty = ry(1) + (ry(2)-ry(1))*rand();
            ok = true;
            if sqrt(tx^2+ty^2) < 10, ok = false; end
            if ok
                for k = 1:K
                    c = scenario.obstacles(k,:);
                    if sqrt((tx-c(1))^2 + (ty-c(2))^2) < c(3) + opts.d_safe + 3
                        ok = false; break;
                    end
                end
            end
            if ok
                scenario.targets(j,:) = [tx, ty];
                break;
            end
        end
        if scenario.targets(j,1) == 0 && scenario.targets(j,2) == 0
            scenario.targets(j,:) = [rx(1)+(rx(2)-rx(1))*rand(), ry(1)+(ry(2)-ry(1))*rand()];
        end
    end
end

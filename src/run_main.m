%RUN_MAIN 无GUI调试运行脚本（静态场景）
clc; clear; close all;
fprintf('=== UAV任务分配仿真（SA vs CS）===\n\n');

% 场景参数
N = 5; M = 12; K = 6;
L_max = 600;
uav_speed = [8;10;6;8;7];
sensor_types = [1;2;3;1;2];

% 权重
omega = 0.3; P_val = 50; d_safe = 2;

% SA 参数
sa_p.T0 = 100; sa_p.alpha = 0.95; sa_p.T_end = 1; sa_p.max_iter = 500;
% CS 参数
cs_p.N_pop = 25; cs_p.Pa = 0.25; cs_p.alpha_step = 0.01; cs_p.max_iter = 500;

% 生成场景
fprintf('生成场景: N=%d, M=%d, K=%d ... ', N, M, K);
sc = generate_scenario(N, M, K, ...
    'L_max', L_max, 'uav_speed', uav_speed, 'sensor_types', sensor_types, ...
    'omega', omega, 'P', P_val, 'd_safe', d_safe, 'seed', 42);
fprintf('完成\n');

% 运行 SA
fprintf('\n运行SA...');
sa_r = simulated_annealing(sc, sa_p);
fprintf('  J=%.4f, 耗时=%.3fs\n', sa_r.best_fitness, sa_r.runtime);

% 运行 CS
fprintf('运行CS...');
cs_r = cuckoo_search(sc, cs_p);
fprintf('  J=%.4f, 耗时=%.3fs\n', cs_r.best_fitness, cs_r.runtime);

% 对比
fprintf('\n对比:\n');
fprintf('  SA: J=%.4f, 耗时=%.3fs\n', sa_r.best_fitness, sa_r.runtime);
fprintf('  CS: J=%.4f, 耗时=%.3fs\n', cs_r.best_fitness, cs_r.runtime);

% 绘图
figure('Name','对比','Position',[100,100,1200,700]);
subplot(2,3,1);
plot_path(gca, sc, sa_r.best_sequence, sprintf('SA J=%.2f',sa_r.best_fitness));
subplot(2,3,2);
plot_path(gca, sc, cs_r.best_sequence, sprintf('CS J=%.2f',cs_r.best_fitness));
subplot(2,3,3);
plot_convergence(gca, sa_r.fitness_history, cs_r.fitness_history, sa_r.runtime, cs_r.runtime);
subplot(2,3,4);
plot_allocation_table(gca, sc, sa_r.best_sequence, sa_r.best_distances, 'SA');
subplot(2,3,5);
plot_allocation_table(gca, sc, cs_r.best_sequence, cs_r.best_distances, 'CS');
fprintf('\n完成\n');

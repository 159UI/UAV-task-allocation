function d = distance_penalty(p1, p2, obstacles, d_safe, P)
%DISTANCE_PENALTY 计算两点之间的惩罚距离（含避障惩罚）
%   输入：
%      p1, p2     — [1×2] 两点坐标
%      obstacles  — [K×3] 障碍物矩阵 [x, y, radius]
%      d_safe     — 标量，安全距离裕量
%      P          — 标量，障碍穿越惩罚常数
%   输出：
%      d          — 惩罚距离 = 欧氏距离 + P × 穿越障碍物个数
%
%   在 SA/CS 的解码过程中被反复调用，用于快速评估路径长度
%
%   参考：collision_check 进行碰撞检测

    % 欧氏距离
    euclidean_dist = sqrt(sum((p2 - p1).^2));

    % 碰撞检测
    num_cross = collision_check(p1, p2, obstacles, d_safe);

    % 惩罚距离
    d = euclidean_dist + P * num_cross;
end

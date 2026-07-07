function num_crossings = collision_check(p1, p2, obstacles, d_safe)
%COLLISION_CHECK 检测线段 AB 与所有圆形障碍物的碰撞情况
%   输入：
%      p1, p2     — [1×2] 线段端点坐标
%      obstacles  — [K×3] 障碍物矩阵，每行为 [x, y, radius]
%      d_safe     — 标量，安全距离裕量
%   输出：
%      num_crossings — 线段穿过的障碍物数量
%
%   方法：计算线段到障碍物中心的最短距离
%       若 dist_min < radius + d_safe，判定为碰撞

    num_crossings = 0;
    K = size(obstacles, 1);

    for k = 1:K
        cx = obstacles(k, 1);
        cy = obstacles(k, 2);
        r  = obstacles(k, 3);

        % 线段 AB 向量
        dx = p2(1) - p1(1);
        dy = p2(2) - p1(2);

        % 障碍物中心到线段起点的向量
        fx = p1(1) - cx;
        fy = p1(2) - cy;

        % 计算投影参数 t（限制在 [0,1] 范围内）
        t = -(dx * fx + dy * fy) / (dx^2 + dy^2 + eps);
        t = max(0, min(1, t));

        % 最近点坐标
        near_x = p1(1) + t * dx;
        near_y = p1(2) + t * dy;

        % 最近点到障碍物中心的距离
        dist = sqrt((near_x - cx)^2 + (near_y - cy)^2);

        if dist < (r + d_safe)
            num_crossings = num_crossings + 1;
        end
    end
end

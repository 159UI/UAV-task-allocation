function step = levy_flight(beta, dim)
%LEVY_FLIGHT 生成莱维飞行步长（Mantegna 算法）
%   输入：
%      beta — 莱维分布参数，默认 1.5（范围 0 < beta ≤ 2）
%      dim  — 向量维度
%   输出：
%      step — [1×dim] 莱维飞行步长向量
%
%   算法：Mantegna R.N., 1992
%   step = u / |v|^(1/beta)
%   其中 u ~ N(0, σ_u^2), v ~ N(0, 1)
%   σ_u = [Γ(1+beta)·sin(π·beta/2) / (Γ((1+beta)/2)·beta·2^((beta-1)/2))]^(1/beta)

    if nargin < 1 || isempty(beta)
        beta = 1.5;
    end
    if nargin < 2
        dim = 1;
    end

    % 计算 sigma_u
    numerator   = gamma(1 + beta) * sin(pi * beta / 2);
    denominator = gamma((1 + beta) / 2) * beta * 2^((beta - 1) / 2);
    sigma_u     = (numerator / denominator)^(1 / beta);

    % u ~ N(0, sigma_u^2), v ~ N(0, 1)
    u = randn(1, dim) * sigma_u;
    v = randn(1, dim);

    % 莱维步长
    step = u ./ (abs(v).^(1 / beta));
end

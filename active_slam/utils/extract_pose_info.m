function [Sigma_r, D_opt, T_opt, N_eff, mean_pose] = extract_pose_info(particles)
%EXTRACT_POSE_INFO 从FastSLAM粒子云提取位姿统计信息
%   输入:
%     particles — 粒子数组，每个粒子有 .xv (3x1位姿) 和 .w (权重)
%   输出:
%     Sigma_r   — 3x3 位姿协方差矩阵（从粒子云估计）
%     D_opt     — D-optimality = det(Sigma_r)^(1/3) = exp(mean(log(eigvals)))
%     T_opt     — T-optimality = trace(Sigma_r)/3 = mean(eigvals)
%     N_eff     — 有效粒子数
%     mean_pose — 加权平均位姿 [x; y; theta]

    N = length(particles);
    poses = reshape([particles.xv], 3, N);   % 3 x N
    weights = [particles.w];                  % 1 x N
    
    % 清洗NaN/Inf粒子
    valid = ~any(isnan(poses) | isinf(poses), 1) & ~isnan(weights) & ~isinf(weights);
    if ~all(valid)
        poses = poses(:, valid);
        weights = weights(valid);
        if isempty(weights)
            mean_pose = [0; 0; 0];
            Sigma_r = eye(3) * 0.01;
            D_opt = 0.01^(1/3);
            T_opt = 0.01;
            N_eff = 1;
            return;
        end
        weights = weights / sum(weights);
    end
    
    % 加权平均位姿
    mean_pose = sum(poses .* weights, 2);
    
    % 角度归一化（加权平均后可能超出 [-pi, pi]）
    mean_pose(3) = atan2(sin(mean_pose(3)), cos(mean_pose(3)));
    
    % 加权协方差矩阵
    centered = poses - mean_pose;
    % 角度差归一化到 [-pi, pi]
    centered(3, :) = atan2(sin(centered(3, :)), cos(centered(3, :)));
    
    Sigma_r = (centered .* weights) * centered' + 1e-6 * eye(3);
    
    % 确保对称正定
    Sigma_r = 0.5 * (Sigma_r + Sigma_r');
    
    % 特征值
    eigenvals = eig(Sigma_r);
    eigenvals = max(eigenvals, 1e-10);  % 防止非正或极小值
    
    % D-opt (几何平均的指数形式)
    D_opt = exp(mean(log(eigenvals)));
    
    % T-opt (算术平均 = 迹/3)
    T_opt = mean(eigenvals);
    
    % 有效粒子数
    N_eff = 1 / sum(weights.^2);
end

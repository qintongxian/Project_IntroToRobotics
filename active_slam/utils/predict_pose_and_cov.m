function [pred_pose, pred_cov, D_opt_pred] = predict_pose_and_cov(robot_pose, Sigma_r, goal, params)
%PREDICT_POSE_AND_COV 预测执行动作（移动到goal）后的位姿和协方差
%   输入:
%     robot_pose — [x; y; theta] 当前位姿
%     Sigma_r    — 3x3 当前位姿协方差
%     goal       — [x, y] 目标位置
%     params     — 结构体，包含 .k_pos, .k_ang (协方差增长系数)
%   输出:
%     pred_pose  — 预测位姿 [x; y; theta]
%     pred_cov   — 预测协方差 3x3
%     D_opt_pred — 预测D-opt

    if nargin < 4
        params = struct();
    end
    if ~isfield(params, 'k_pos'), params.k_pos = 0.05; end
    if ~isfield(params, 'k_ang'), params.k_ang = 0.02; end
    
    dx = goal(1) - robot_pose(1);
    dy = goal(2) - robot_pose(2);
    dist = sqrt(dx^2 + dy^2);
    target_theta = atan2(dy, dx);
    
    pred_pose = [goal(1); goal(2); target_theta];
    
    % 协方差增长模型：距离越远，不确定性越大
    % 位置不确定性随距离平方增长，角度随距离线性增长
    Q_motion = diag([params.k_pos * dist^2, params.k_pos * dist^2, params.k_ang * dist]);
    pred_cov = Sigma_r + Q_motion;
    
    % 确保正定
    pred_cov = 0.5 * (pred_cov + pred_cov');
    pred_cov = pred_cov + 1e-6 * eye(3);
    
    % 计算预测D-opt
    eigs = eig(pred_cov);
    eigs = max(eigs, 1e-10);
    D_opt_pred = exp(mean(log(eigs)));
end

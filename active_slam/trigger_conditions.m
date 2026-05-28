function [should_replan, trigger_params] = trigger_conditions(belief, state, trigger_params)
%TRIGGER_CONDITIONS 多维度触发条件判断
%   判断是否需要触发低频决策循环（重规划）
%
%   输入:
%     belief         — 结构体，包含 .pose_mean, .pose_cov, .Neff
%     state          — 状态机状态，包含 .current_goal
%     trigger_params — 结构体:
%         .N_period        — 时间触发周期 [步]
%         .time_counter    — 当前计数器
%         .spatial_threshold— 空间触发阈值 [m]
%         .sigma_threshold — 不确定性阈值（协方差行列式）
%         .alpha_decay     — 自适应阈值衰减系数 (默认 0.995)
%         .Neff_threshold  — 粒子退化阈值
%         .reason          — (输出) 触发原因字符串
%
%   输出:
%     should_replan  — true/false
%     trigger_params — 更新后的参数（计数器重置等）

    should_replan = false;
    
    % [1] 时间触发（兜底机制）
    % 注意: time_counter 由主循环负责递增
    if trigger_params.time_counter >= trigger_params.N_period
        should_replan = true;
        trigger_params.time_counter = 0;
        trigger_params.reason = 'time_periodic';
        return;
    end
    
    % [2] 空间触发：到达当前目标附近
    if isfield(state, 'current_goal') && ~isempty(state.current_goal)
        dist_to_goal = norm(belief.pose_mean(1:2) - state.current_goal(1:2));
        if dist_to_goal < trigger_params.spatial_threshold
            should_replan = true;
            trigger_params.time_counter = 0;
            trigger_params.reason = 'reached_goal';
            return;
        end
    end
    
    % [3] 不确定性触发（自适应阈值）
    if isfield(belief, 'pose_cov') && ~isempty(belief.pose_cov)
        pose_uncertainty = det(belief.pose_cov);
        adaptive_threshold = trigger_params.sigma_threshold * ...
                             (trigger_params.alpha_decay ^ trigger_params.time_counter);
        if pose_uncertainty > adaptive_threshold
            should_replan = true;
            trigger_params.time_counter = 0;
            trigger_params.reason = 'high_uncertainty';
            return;
        end
    end
    
    % [4] 事件触发：粒子退化
    if isfield(belief, 'Neff') && belief.Neff < trigger_params.Neff_threshold
        should_replan = true;
        trigger_params.time_counter = 0;
        trigger_params.reason = 'particle_depletion';
        return;
    end
end

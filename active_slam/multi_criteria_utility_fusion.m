function [utilities, best_idx, criteria] = multi_criteria_utility_fusion(particles, og_map, candidate_goals, robot_pose, params)
%MULTI_CRITERIA_UTILITY_FUSION 多准则效用函数融合框架 — 核心入口
%   基于《融合效用函数.md》四层框架：信息论(IT) + 最优设计(TOED) + 图论(Graph) + 几何(Geo)
%
%   输入:
%     particles        — FastSLAM粒子数组 (.xv, .w, .xf, .Pf)
%     og_map           — OccupancyGridMap 对象
%     candidate_goals  — K x 2 矩阵，候选目标世界坐标 [x, y]
%     robot_pose       — [x; y; theta] 当前位姿
%     params           — 结构体，包含所有超参数:
%         .sensor_range      — 传感器量程 [m] (默认 10)
%         .sensor_fov        — 视场角 [rad] (默认 2*pi)
%         .k_alpha           — Rényi映射系数 (默认 1.0)
%         .w_dist            — 距离权重 (默认 0.3)
%         .w_path            — 路径熵权重 (默认 0.2)
%         .w_turn            — 转向权重 (默认 0.05)
%         .D_opt_threshold   — D-opt阈值 (默认 1.0)
%         .N_particles       — 粒子数 (默认 100)
%
%   输出:
%     utilities — K x 1 效用值
%     best_idx  — 最优候选索引
%     criteria  — 结构体，保存各层原始值用于分析:
%         .IT, .TOED, .Graph, .Geo, .weights

    %% ========== 默认参数 ==========
    if nargin < 5
        params = struct();
    end
    if ~isfield(params, 'sensor_range'),    params.sensor_range = 10.0; end
    if ~isfield(params, 'sensor_fov'),      params.sensor_fov = 2*pi; end
    if ~isfield(params, 'k_alpha'),         params.k_alpha = 1.0; end
    if ~isfield(params, 'w_dist'),          params.w_dist = 0.3; end
    if ~isfield(params, 'w_path'),          params.w_path = 0.2; end
    if ~isfield(params, 'w_turn'),          params.w_turn = 0.05; end
    if ~isfield(params, 'D_opt_threshold'), params.D_opt_threshold = 1.0; end
    if ~isfield(params, 'N_particles'),     params.N_particles = 100; end
    
    K = size(candidate_goals, 1);
    utilities = zeros(K, 1);
    
    %% ========== 预计算：全局信息（只算一次） ==========
    [Sigma_r, D_opt_global, T_opt_global, N_eff, ~] = extract_pose_info(particles);
    
    % 从最优粒子提取landmarks用于图论层
    [~, best_particle_idx] = max([particles.w]);
    best_particle = particles(best_particle_idx);
    landmarks = best_particle.xf;
    covariances = best_particle.Pf;
    
    if ~isempty(landmarks)
        sensor_params_graph = struct();
        sensor_params_graph.range_max = params.sensor_range;
        sensor_params_graph.fov = params.sensor_fov;
        [lambda2, ~, ~] = build_landmark_graph(landmarks, covariances, robot_pose, sensor_params_graph);
    else
        lambda2 = 0;
    end
    
    R_unknown = og_map.unknownRatio();
    
    % 自适应权重
    [w_IT, w_TOED, w_Graph, w_Geo] = adaptive_weight_scheduler(...
        R_unknown, D_opt_global, N_eff, params);
    
    % 全局Rényi参数（基于当前位姿不确定性）
    alpha_global = 1 + params.k_alpha * log(1 + D_opt_global);
    
    %% ========== 对每个候选目标计算多层效用 ==========
    for i = 1:K
        goal = candidate_goals(i, :);
        
        % ===== 预测位姿和协方差 =====
        [pred_pose, pred_cov, D_opt_pred] = predict_pose_and_cov(robot_pose, Sigma_r, goal, params);
        alpha_pred = 1 + params.k_alpha * log(1 + D_opt_pred);
        
        % ===== 信息论层 U_IT =====
        H_map = compute_local_map_entropy(og_map, pred_pose, params.sensor_range);
        H_renyi = compute_renyi_entropy_local(og_map, pred_pose, params.sensor_range, alpha_pred);
        U_IT = H_map - H_renyi;
        
        % ===== TOED层 U_TOED =====
        if isnan(D_opt_pred) || D_opt_pred <= 0
            U_TOED = 0;  % 回退：不参与效用计算
        else
            U_TOED = -log(D_opt_pred + 1e-6);
        end
        
        % ===== 图论层 U_Graph =====
        % 简化：假设到达目标后图连通度与当前lambda2近似
        % 更精细的做法是模拟目标处可见的landmark并预测lambda2变化
        U_Graph = lambda2;
        
        % ===== 几何层 U_Geo =====
        dist_cost = norm(robot_pose(1:2) - goal');
        H_path = compute_path_entropy(og_map, robot_pose(1:2), goal);
        delta_theta = abs(atan2(goal(2)-robot_pose(2), goal(1)-robot_pose(1)) - robot_pose(3));
        delta_theta = min(delta_theta, 2*pi - delta_theta);
        
        U_Geo = -params.w_dist*dist_cost - params.w_path*H_path - params.w_turn*delta_theta;
        
        % ===== 融合 =====
        utilities(i) = w_IT*U_IT + w_TOED*U_TOED + w_Graph*U_Graph + w_Geo*U_Geo;
        
        % 如果仍有NaN（如IT层计算失败），只用几何层
        if isnan(utilities(i))
            utilities(i) = U_Geo;
        end
        
        % 保存各准则值
        criteria.IT(i) = U_IT;
        criteria.TOED(i) = U_TOED;
        criteria.Graph(i) = U_Graph;
        criteria.Geo(i) = U_Geo;
    end
    
    % 保存权重信息
    criteria.weights = [w_IT, w_TOED, w_Graph, w_Geo];
    criteria.D_opt_global = D_opt_global;
    criteria.N_eff = N_eff;
    criteria.R_unknown = R_unknown;
    criteria.lambda2 = lambda2;
    
    [~, best_idx] = max(utilities);
end

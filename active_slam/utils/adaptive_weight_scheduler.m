function [w_IT, w_TOED, w_Graph, w_Geo] = adaptive_weight_scheduler(R_unknown, D_opt, N_eff, params)
%ADAPTIVE_WEIGHT_SCHEDULER 自适应权重调度器
%   根据探索进度、位姿不确定性和粒子退化程度动态调整四层权重
%
%   输入:
%     R_unknown      — OG地图未知区域比例 [0,1]
%     D_opt          — 当前位姿D-optimality（越大越不确定）
%     N_eff          — 有效粒子数
%     params         — 结构体:
%         .N_particles     — 总粒子数
%         .D_opt_threshold — D-opt高不确定性阈值
%   输出:
%     w_IT, w_TOED, w_Graph, w_Geo — 四层归一化权重

    if nargin < 4
        params = struct();
    end
    if ~isfield(params, 'N_particles'),     params.N_particles = 100; end
    if ~isfield(params, 'D_opt_threshold'), params.D_opt_threshold = 1.0; end
    
    % ===== 基础权重（随探索进度） =====
    if R_unknown > 0.6
        base = [0.50, 0.15, 0.15, 0.20];  % 探索阶段: IT主导
    elseif R_unknown > 0.2
        base = [0.30, 0.25, 0.25, 0.20];  % 平衡阶段
    else
        base = [0.15, 0.40, 0.25, 0.20];  % 利用阶段: TOED+Graph主导
    end
    
    % ===== 粒子退化修正 =====
    % N_eff低时增加保守性（优先短路径、降低探索倾向）
    degradation_factor = min(1, N_eff / (params.N_particles * 0.3));
    if degradation_factor < 0.5
        base = base + [-0.15, 0.05, 0.05, 0.05];
    end
    
    % ===== 位姿不确定性修正 =====
    % D_opt大时（定位漂移风险高）增加Graph权重，降低IT权重
    uncertainty_factor = min(1, D_opt / params.D_opt_threshold);
    if uncertainty_factor > 0.7
        base = base + [-0.10, 0.05, 0.10, -0.05];
    end
    
    % 确保权重非负
    base = max(base, 0.01);
    
    % 归一化
    total = sum(base);
    w_IT   = base(1) / total;
    w_TOED = base(2) / total;
    w_Graph= base(3) / total;
    w_Geo  = base(4) / total;
end

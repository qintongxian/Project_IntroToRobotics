function [lambda2, tau_G, avg_degree] = build_landmark_graph(landmarks, covariances, robot_pose, sensor_params)
%BUILD_LANDMARK_GRAPH 从landmark共视图计算图论指标
%   输入:
%     landmarks   — 2 x M 矩阵，landmark世界坐标
%     covariances — 2 x 2 x M 矩阵（可选，若无则不使用权值）
%     robot_pose  — [x; y; theta]，用于判断共视
%     sensor_params — 包含 .range_max, .fov [rad]
%   输出:
%     lambda2     — 代数连通度（Fiedler值），越大图越连通
%     tau_G       — 生成树数量指标（简化计算）
%     avg_degree  — 平均节点度
%
%   注意: 当landmark数量很大时，会自动降采样以控制计算量

    if nargin < 3 || isempty(robot_pose)
        robot_pose = [0; 0; 0];
    end
    if nargin < 4
        sensor_params = struct();
    end
    if ~isfield(sensor_params, 'range_max'), sensor_params.range_max = 10.0; end
    if ~isfield(sensor_params, 'fov'),       sensor_params.fov = 2*pi; end
    
    M = size(landmarks, 2);
    if M == 0
        lambda2 = 0; tau_G = 0; avg_degree = 0;
        return;
    end
    
    % 降采样：如果landmark太多，只保留最近的一部分
    max_lm = 100;
    if M > max_lm
        % 按距机器人距离排序，保留最近的
        dists = sqrt(sum((landmarks - robot_pose(1:2)).^2, 1));
        [~, idx] = sort(dists, 'ascend');
        landmarks = landmarks(:, idx(1:max_lm));
        if nargin >= 2 && ~isempty(covariances) && ndims(covariances) == 3
            covariances = covariances(:, :, idx(1:max_lm));
        end
        M = max_lm;
    end
    
    W = zeros(M, M);  % 权重矩阵
    
    % 构建共视图：同时被同一帧观测到的landmark之间有边
    for i = 1:M
        for j = i+1:M
            v_i = landmarks(:, i) - robot_pose(1:2);
            v_j = landmarks(:, j) - robot_pose(1:2);
            d_i = norm(v_i);
            d_j = norm(v_j);
            
            % 检查是否都在传感器范围内
            if d_i < sensor_params.range_max && d_j < sensor_params.range_max
                angle_i = atan2(v_i(2), v_i(1));
                angle_j = atan2(v_j(2), v_j(1));
                angle_diff = abs(angle_i - angle_j);
                angle_diff = min(angle_diff, 2*pi - angle_diff);
                
                if angle_diff < sensor_params.fov / 2
                    % 共视：边权重与观测精度相关
                    if nargin >= 2 && ~isempty(covariances) && ndims(covariances) == 3
                        w_ij = 1 / (trace(covariances(:,:,i)) + trace(covariances(:,:,j)) + 1e-6);
                    else
                        w_ij = 1.0;
                    end
                    W(i, j) = w_ij;
                    W(j, i) = w_ij;
                end
            end
        end
    end
    
    % 度矩阵和拉普拉斯矩阵
    D = diag(sum(W, 2));
    L = D - W;
    
    % 代数连通度（第二小特征值）
    if M > 1
        eigenvals_L = sort(eig(L));
        lambda2 = eigenvals_L(2);
    else
        lambda2 = 0;
    end
    
    % 生成树数量（简化：用约化拉普拉斯行列式）
    if M > 1
        L_reduced = L(2:end, 2:end);
        tau_G = det(L_reduced + 1e-6*eye(M-1));
    else
        tau_G = 0;
    end
    
    % 平均节点度
    avg_degree = mean(sum(W > 0, 2));
end

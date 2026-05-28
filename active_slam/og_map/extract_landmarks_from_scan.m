function [zf, idf, zn, updated_landmarks] = extract_landmarks_from_scan(ranges, angles, robot_pose, existing_landmarks, sensor_params)
%EXTRACT_LANDMARKS_FROM_SCAN 将激光扫描转为landmark观测，并做数据关联
%   输入:
%     ranges           — N x 1 激光距离 [m]
%     angles           — N x 1 激光角度 [rad]，相对于机器人朝向
%     robot_pose       — [x; y; theta]
%     existing_landmarks — 2 x M 矩阵，已有landmark世界坐标 (若无则为 [])
%     sensor_params    — 结构体，包含 .range_max, .range_min, .assoc_threshold
%
%   输出:
%     zf — 2 x K 矩阵 [range; bearing]，已知landmark的观测
%     idf — 1 x K 索引，对应 existing_landmarks 的列号
%     zn — 2 x L 矩阵 [range; bearing]，新landmark的观测
%     updated_landmarks — 数据关联后的landmark列表（用于更新外部DA表）

    if nargin < 5
        sensor_params = struct();
    end
    if ~isfield(sensor_params, 'range_max'),       sensor_params.range_max = 10.0; end
    if ~isfield(sensor_params, 'range_min'),       sensor_params.range_min = 0.1; end
    if ~isfield(sensor_params, 'assoc_threshold'), sensor_params.assoc_threshold = 0.5; end
    
    x_r = robot_pose(1);
    y_r = robot_pose(2);
    theta_r = robot_pose(3);
    
    zf = [];
    idf = [];
    zn = [];
    new_landmarks = [];
    
    % === 第一轮：数据关联，只基于已有landmark（existing_landmarks）===
    % 注意：不能在循环中把新landmark加入关联池，否则idf会超出existing_landmarks范围
    for i = 1:length(ranges)
        r = ranges(i);
        phi = angles(i);
        
        % 过滤无效测量
        if r >= sensor_params.range_max || r <= sensor_params.range_min || isnan(r) || isinf(r)
            continue;
        end
        
        % landmark 世界坐标
        lm_x = x_r + r * cos(theta_r + phi);
        lm_y = y_r + r * sin(theta_r + phi);
        lm_world = [lm_x; lm_y];
        
        % 数据关联：最近邻（只基于existing_landmarks，不动态扩展）
        if isempty(existing_landmarks)
            is_new = true;
            assoc_idx = 0;
        else
            dists = sqrt(sum((existing_landmarks - lm_world).^2, 1));
            [min_dist, assoc_idx] = min(dists);
            is_new = (min_dist >= sensor_params.assoc_threshold);
        end
        
        if is_new
            % 新 landmark：观测为机器人坐标系下的 range-bearing
            zn = [zn, [r; phi]];
            new_landmarks = [new_landmarks, lm_world];
        else
            % 已有 landmark
            zf = [zf, [r; phi]];
            idf = [idf, assoc_idx];
        end
    end
    
    % 去重：同一landmark被多束激光命中时合并观测
    if ~isempty(idf)
        [unique_idf, ~, ic] = unique(idf, 'stable');
        zf_merged = zeros(2, length(unique_idf));
        for k = 1:length(unique_idf)
            vals = zf(:, ic == k);
            zf_merged(:, k) = mean(vals, 2);
        end
        zf = zf_merged;
        idf = unique_idf;
    end
    
    % 合并新landmark到updated_landmarks
    updated_landmarks = [existing_landmarks, new_landmarks];
end

function [zf, idf, zn, updated_landmarks] = extract_landmarks_from_scan(ranges, angles, robot_pose, existing_landmarks, sensor_params)
%EXTRACT_LANDMARKS_FROM_SCAN 将激光扫描转为几何特征 landmark（端点/不连续点）
%   策略：
%     1. 过滤无效测量
%     2. 提取相邻射线端点距离突变处（breakpoint）=> 墙角/端点
%     3. 对相邻 breakpoint 聚类取平均，得到稳定的几何特征
%     4. 与已有 landmark 做最近邻数据关联
%
%   输入:
%     ranges           — N x 1 激光距离 [m]
%     angles           — N x 1 激光角度 [rad]，相对于机器人朝向
%     robot_pose       — [x; y; theta]
%     existing_landmarks — 2 x M 矩阵，已有landmark世界坐标 (若无则为 [])
%     sensor_params    — 结构体
%
%   输出:
%     zf — 2 x K 矩阵 [range; bearing]，已知landmark的观测
%     idf — 1 x K 索引，对应 existing_landmarks 的列号
%     zn — 2 x L 矩阵 [range; bearing]，新landmark的观测
%     updated_landmarks — 数据关联后的landmark列表

    if nargin < 5
        sensor_params = struct();
    end
    if ~isfield(sensor_params, 'range_max'),       sensor_params.range_max = 10.0; end
    if ~isfield(sensor_params, 'range_min'),       sensor_params.range_min = 0.1; end
    if ~isfield(sensor_params, 'assoc_threshold'), sensor_params.assoc_threshold = 0.5; end
    if ~isfield(sensor_params, 'break_threshold'), sensor_params.break_threshold = 0.5; end
    if ~isfield(sensor_params, 'min_corners'),     sensor_params.min_corners = 2; end
    
    x_r = robot_pose(1);
    y_r = robot_pose(2);
    theta_r = robot_pose(3);
    
    zf = [];
    idf = [];
    zn = [];
    new_landmarks = [];
    
    %% ==============================================================
    %  Step 1: 过滤无效测量
    %% ==============================================================
    valid = ranges > sensor_params.range_min & ranges < sensor_params.range_max & ~isnan(ranges) & ~isinf(ranges);
    r_valid = ranges(valid);
    a_valid = angles(valid);
    n_valid = length(r_valid);
    
    if n_valid < 3
        updated_landmarks = existing_landmarks;
        return;
    end
    
    %% ==============================================================
    %  Step 2: 提取几何特征点（不连续点 / 端点）
    %% ==============================================================
    % 转为机器人局部笛卡尔坐标
    pts_local = [r_valid .* cos(a_valid), r_valid .* sin(a_valid)];  % N x 2
    
    % 计算相邻射线端点的欧氏距离（环形首尾相连）
    pts_next = circshift(pts_local, -1, 1);
    dists = sqrt(sum((pts_next - pts_local).^2, 2));  % N x 1
    
    % breakpoint 检测：相邻端点距离超过阈值 => 存在不连续（墙角/边缘）
    is_break = dists > sensor_params.break_threshold;
    
    % 端点标记：breakpoint 的两侧点都是端点候选
    % is_break(i) 表示 pts(i) 与 pts(i+1) 之间不连续
    corner_mask = is_break | circshift(is_break, 1, 1);
    
    % 提取候选索引
    corner_idx = find(corner_mask);
    candidates_local = [];
    
    if ~isempty(corner_idx)
        % 聚类：相邻的角点索引属于同一个物理角点，取平均得到稳定位置
        clusters = {};
        current_clust = corner_idx(1);
        
        for i = 2:length(corner_idx)
            prev = corner_idx(i-1);
            curr = corner_idx(i);
            % 判断是否相邻（考虑激光扫描的环形结构）
            is_adjacent = (curr == prev + 1) || (prev == n_valid && curr == 1);
            
            if is_adjacent
                current_clust = [current_clust, curr];
            else
                clusters{end+1} = current_clust; %#ok<AGROW>
                current_clust = curr;
            end
        end
        clusters{end+1} = current_clust; %#ok<AGROW>
        
        % 每个聚类取平均，得到最终候选点（局部坐标系）
        n_corners = length(clusters);
        candidates_local = zeros(n_corners, 2);
        for i = 1:n_corners
            candidates_local(i, :) = mean(pts_local(clusters{i}, :), 1);
        end
    end
    
    %% ==============================================================
    %  Step 3: 回退策略 — 若几何特征太少，做稀疏采样
    %% ==============================================================
    if size(candidates_local, 1) < sensor_params.min_corners
        % 环境过于简单（如直面一堵长墙），没有明显角点
        % 回退：每隔一定步长取一个稀疏采样点，避免完全无观测
        step = max(1, floor(n_valid / 8));  % 约保留 8 个采样点
        sample_idx = 1:step:n_valid;
        candidates_local = pts_local(sample_idx, :);
    end
    
    %% ==============================================================
    %  Step 4: 转为 range-bearing 观测并做数据关联
    %% ==============================================================
    cand_ranges   = sqrt(sum(candidates_local.^2, 2));
    cand_bearings = atan2(candidates_local(:,2), candidates_local(:,1));
    
    % 世界坐标（用于数据关联）
    lm_world_mat = [x_r + cand_ranges .* cos(theta_r + cand_bearings), ...
                    y_r + cand_ranges .* sin(theta_r + cand_bearings)];  % N x 2
    
    for i = 1:size(lm_world_mat, 1)
        lm_w = lm_world_mat(i, :)';
        
        % 数据关联：最近邻（欧氏距离）
        if isempty(existing_landmarks)
            is_new = true;
            assoc_idx = 0;
        else
            dists_to_existing = sqrt(sum((existing_landmarks - lm_w).^2, 1));
            [min_dist, assoc_idx] = min(dists_to_existing);
            is_new = (min_dist >= sensor_params.assoc_threshold);
        end
        
        if is_new
            % 新 landmark
            zn = [zn, [cand_ranges(i); cand_bearings(i)]];
            new_landmarks = [new_landmarks, lm_w];
        else
            % 已有 landmark
            zf = [zf, [cand_ranges(i); cand_bearings(i)]];
            idf = [idf, assoc_idx];
        end
    end
    
    %% ==============================================================
    %  Step 5: 去重 — 同一已有 landmark 被多个候选命中时合并观测
    %% ==============================================================
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
    
    % 合并新 landmark
    updated_landmarks = [existing_landmarks, new_landmarks];
end

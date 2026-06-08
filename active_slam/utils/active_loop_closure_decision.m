function [loop_goal, path] = active_loop_closure_decision(trajectory_history, current_pose, og_map, robot_radius, spatial_threshold)
%ACTIVE_LOOP_CLOSURE_DECISION 从历史轨迹中选择最佳回环目标
%   策略：在历史轨迹中采样候选点，优先选择距离适中、路径可达、
%         且历史较久的位姿作为回环目标，以最大化回环校正效果。
%
%   输入:
%     trajectory_history — 3 x T 历史轨迹矩阵（通常取最优粒子位姿历史）
%     current_pose       — [x; y; theta] 当前位姿
%     og_map             — OccupancyGridMap 对象
%     robot_radius       — 机器人碰撞半径 [m] (默认 0.3)
%     spatial_threshold  — 到达目标的空间阈值 [m] (默认 2.0)
%   输出:
%     loop_goal — [x; y; theta] 回环目标位姿（空表示无可行回环点）
%     path      — N x 2 路径路点世界坐标 [x, y]（空表示无可行路径）

    if nargin < 5 || isempty(spatial_threshold)
        spatial_threshold = 2.0;
    end
    if nargin < 4 || isempty(robot_radius)
        robot_radius = 0.3;
    end

    n_history = size(trajectory_history, 2);
    loop_goal = [];
    path = [];
    
    if n_history < 100
        return;
    end
    
    % 参数：采样间隔与最小时间/步数间隔
    sample_step = 40;
    min_step_gap = 100;  % 至少100步之前的历史点才考虑回环
    
    candidates = [];
    candidate_indices = [];
    
    % === 第一轮筛选：距离适中 + 直线路径无阻挡 ===
    for i = sample_step:sample_step:n_history-min_step_gap
        hist_pose = trajectory_history(:, i);
        dist = norm(hist_pose(1:2) - current_pose(1:2));
        
        if dist > 3.0 && dist < 15.0
            if ~is_path_blocked(og_map, current_pose(1:2)', hist_pose(1:2)')
                candidates = [candidates, hist_pose]; %#ok<AGROW>
                candidate_indices = [candidate_indices, i]; %#ok<AGROW>
            end
        end
    end
    
    % === 第二轮：若无直线路径可达候选，放宽到所有距离适中点 ===
    if isempty(candidates)
        for i = sample_step:sample_step:n_history-min_step_gap
            hist_pose = trajectory_history(:, i);
            dist = norm(hist_pose(1:2) - current_pose(1:2));
            if dist > 3.0 && dist < 15.0
                candidates = [candidates, hist_pose]; %#ok<AGROW>
                candidate_indices = [candidate_indices, i]; %#ok<AGROW>
            end
        end
    end
    
    % === 第三轮：若仍无，退回最老的一个历史点 ===
    if isempty(candidates) && n_history > min_step_gap
        candidates = trajectory_history(:, 1);
        candidate_indices = 1;
    end
    
    if isempty(candidates)
        return;
    end
    
    % === 评分选择最优回环点 ===
    % 距离当前位置越近越好（降低移动代价）
    % 历史越老越好（回环基线越长，校正效果越明显）
    n_cand = size(candidates, 2);
    scores = zeros(1, n_cand);
    
    for k = 1:n_cand
        dist_cost = norm(candidates(1:2, k) - current_pose(1:2));
        age_bonus = candidate_indices(k) / n_history;  % 归一化到 [0,1]
        scores(k) = -0.5 * dist_cost + 4.0 * age_bonus;
    end
    
    [~, best_idx] = max(scores);
    best_pose = candidates(:, best_idx);
    
    % 构建目标姿态：保持历史朝向
    loop_goal = best_pose;
    
    % === A* 路径规划 ===
    path_world = astar_og(og_map, current_pose(1:2)', best_pose(1:2)', robot_radius);
    
    if ~isempty(path_world)
        path = path_world;
    else
        % A* 失败时回退到直线路径（至少上层有路径可跟踪）
        path = [current_pose(1:2)'; best_pose(1:2)'];
    end
end

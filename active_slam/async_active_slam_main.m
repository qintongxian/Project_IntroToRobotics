function [trajectory, maps, entropy_history, state_log, criteria_log] = async_active_slam_main(sim_mode)
%ASYNC_ACTIVE_SLAM_MAIN 异步双循环主动SLAM主程序
%   高频FastSLAM 2.0后端 + 低频多准则融合决策
%
%   输入:
%     sim_mode — 'simulation' 使用仿真环境 (默认)
%                'ros'        连接ROS/Gazebo (需ROS Toolbox)
%   输出:
%     trajectory     — 3 x T 真实/估计轨迹
%     maps           — 地图历史快照 (cell数组)
%     entropy_history— 位姿熵历史
%     state_log      — 状态机状态历史
%     criteria_log   — 各层准则值历史 (用于消融实验)

    if nargin < 1
        sim_mode = 'simulation';
    end
    
    %% ========== 参数初始化 ==========
    T_max = 5000;               % 最大高频步数
    M = 100;                    % FastSLAM粒子数
    map_size = [20, 20];        % 地图尺寸 [m] (宽, 高)
    map_origin = [-10, -10];    % 左下角原点
    cell_size = 0.1;            % 栅格分辨率 [m]
    sensor_range = 10;          % 传感器最大量程 [m]
    sensor_fov = 2*pi;          % 视场角 [rad]
    dt_slam = 0.05;             % SLAM周期 (20Hz = 50ms)
    robot_radius = 0.3;         % 机器人碰撞半径 [m]
    
    % 触发条件参数
    trigger_params = struct();
    trigger_params.N_period = 50;               % 时间触发：每50步
    trigger_params.time_counter = 0;
    trigger_params.spatial_threshold = 2.0;     % 空间触发：距目标<2m
    trigger_params.sigma_threshold = 0.5;       % 不确定性阈值
    trigger_params.alpha_decay = 0.995;         % 自适应衰减
    trigger_params.Neff_threshold = M * 0.3;    % 粒子退化阈值
    trigger_params.reason = '';
    
    % 状态机初始化
    state = struct();
    state.mode = 'INIT';
    state.need_decision = false;
    state.current_goal = [0; 0; 0];
    state.current_path = [];
    state.current_wp_idx = 1;
    state.loop_closure_completed = false;
    state.exploration_complete = false;
    
    % 效用函数参数
    utility_params = struct();
    utility_params.sensor_range = sensor_range;
    utility_params.sensor_fov = sensor_fov;
    utility_params.k_alpha = 1.0;
    utility_params.w_dist = 0.3;
    utility_params.w_path = 0.2;
    utility_params.w_turn = 0.05;
    utility_params.D_opt_threshold = 1.0;
    utility_params.N_particles = M;
    
    % 传感器参数
    sensor_params = struct();
    sensor_params.range_max = sensor_range;
    sensor_params.range_min = 0.1;
    sensor_params.assoc_threshold = 0.5;
    sensor_params.P_occ = 0.6;
    sensor_params.P_free = 0.35;
    sensor_params.R_noise = diag([0.1^2; (1*pi/180)^2]);  % range-bearing观测噪声
    
    %% ========== FastSLAM 2.0 初始化 ==========
    % 复用现有 fastslam 的粒子结构: .xv, .w, .xf, .Pf, .Pv, .da
    particles = init_particles(M);
    x_true = [0; 0; 0];         % 仿真用真实位姿
    
    %% ========== OG地图初始化 ==========
    og_map = OccupancyGridMap(map_size(1), map_size(2), cell_size, map_origin);
    
    %% ========== 存储历史 ==========
    trajectory = zeros(3, T_max);
    best_pose_history = zeros(3, T_max);
    maps = {};
    entropy_history = zeros(1, T_max);
    state_log = cell(1, T_max);
    criteria_log = cell(1, T_max);  % 动态使用，预分配足够大
    map_idx = 1;
    criteria_idx = 1;
    
    %% ========== 可视化初始化 ==========
    enable_visualization = true;
    last_criteria = struct('IT', 0, 'TOED', 0, 'Graph', 0, 'Geo', 0, ...
                           'weights', [0.25, 0.25, 0.25, 0.25], ...
                           'D_opt_global', 0, 'N_eff', M, 'R_unknown', 1, 'lambda2', 0);
    last_frontiers = [];
    last_best_goal = [];
    if enable_visualization
        visualize_realtime('init');
    end
    
    %% ========== 仿真环境初始化 (仅仿真模式) ==========
    if strcmpi(sim_mode, 'simulation')
        % 线段墙体环境 [x1, y1, x2, y2]
        env_walls = generate_simple_environment();
    end
    
    %% ===================================================================
    %  T_max 高频主循环
    %% ===================================================================
    for t = 1:T_max
        
        %% ==============================================================
        %  [高频] Step 1: 获取传感器数据
        %% ==============================================================
        if strcmpi(sim_mode, 'simulation')
            x_true_prev = x_true;
            
            % 路径跟踪：如果有A*路径，跟踪当前路点而非直冲终点
            if ~isempty(state.current_path) && state.current_wp_idx <= size(state.current_path, 1)
                wp = state.current_path(state.current_wp_idx, :)';
                % 若已接近当前路点，切换到下一路点
                if norm(x_true(1:2) - wp(1:2)) < 0.5 && state.current_wp_idx < size(state.current_path, 1)
                    state.current_wp_idx = state.current_wp_idx + 1;
                    wp = state.current_path(state.current_wp_idx, :)';
                end
            else
                wp = state.current_goal(1:2);
            end
            
            [ranges, angles, odom_cmd] = sim_sensor_step(x_true, wp, ...
                env_walls, sensor_params, dt_slam);
            x_true = sim_motion_step(x_true, odom_cmd, dt_slam, env_walls, robot_radius);
            
            % 关键修复：FastSLAM 必须使用与实际运动一致的里程计。
            % 碰撞/卡墙时真实位姿停滞，若继续用理想控制指令 odom_cmd，
            % 粒子会前进而真值不动，导致严重定位漂移。
            delta_theta = atan2(sin(x_true(3) - x_true_prev(3)), ...
                                cos(x_true(3) - x_true_prev(3)));
            mid_theta = x_true_prev(3) + delta_theta / 2;
            actual_v = ((x_true(1) - x_true_prev(1)) * cos(mid_theta) + ...
                        (x_true(2) - x_true_prev(2)) * sin(mid_theta)) / dt_slam;
            actual_w = delta_theta / dt_slam;
            odom = [actual_v; actual_w];
        else
            % ROS模式：从话题读取
            error('ROS mode not yet implemented in this demo.');
        end
        
        %% ==============================================================
        %  [高频] Step 2: FastSLAM 2.0 更新
        %% ==============================================================
        % 运动预测（差速模型）
        Q_motion = diag([0.01, 0.005]);  % [v_noise^2, w_noise^2]
        for k = 1:M
            particles(k) = predict_diffdrive(particles(k), odom(1), odom(2), Q_motion, dt_slam, 1);
        end
        
        % 将激光扫描转为landmark观测，并做数据关联
        for k = 1:M
            [zf, idf, zn] = extract_landmarks_from_scan(ranges, angles, ...
                particles(k).xv, particles(k).xf, sensor_params);
            
            % 已知landmark：FastSLAM 2.0 最优提议 + EKF更新
            % 过滤过近的landmark（避免雅可比矩阵数值爆炸）
            if ~isempty(zf)
                min_dist_thresh = 0.3;  % 最小距离阈值 [m]
                valid_obs = zf(1, :) > min_dist_thresh;
                zf = zf(:, valid_obs);
                idf = idf(valid_obs);
                
                if ~isempty(zf)
                    particles(k) = sample_proposal(particles(k), zf, idf, sensor_params.R_noise);
                    particles(k) = feature_update(particles(k), zf, idf, sensor_params.R_noise);
                end
            end
            
            % 新landmark：初始化
            if ~isempty(zn)
                if isempty(zf)
                    % 如果没有已知观测，从运动模型采样
                    % 确保Pv正定（数值误差可能导致非正定）
                    Pv_reg = 0.5*(particles(k).Pv + particles(k).Pv') + 1e-6*eye(3);
                    particles(k).xv = multivariate_gauss(particles(k).xv, Pv_reg, 1);
                    particles(k).Pv = zeros(3);
                end
                particles(k) = add_feature(particles(k), zn, sensor_params.R_noise);
            end
            
            % 权重已在sample_proposal中更新，新landmark需额外处理
            % 简化为均匀权重（后续归一化）
        end
        
        % 归一化权重 + 重采样（含NaN/Inf清洗）
        weights = [particles.w];
        valid = ~isnan(weights) & ~isinf(weights) & (weights >= 0);
        if ~all(valid)
            if any(valid)
                min_valid = min(weights(valid));
                weights(~valid) = max(min_valid * 0.01, 1e-10);
            else
                weights = ones(1, M) / M;
            end
        end
        weights = weights / sum(weights);
        for k = 1:M, particles(k).w = weights(k); end
        Neff = 1 / sum(weights.^2);
        if Neff < M * 0.5
            particles = resample_particles(particles, M*0.5, 1);
        end
        
        %% ==============================================================
        %  [高频] Step 3: 提取信念 + 更新OG地图
        %% ==============================================================
        belief = extract_belief(particles, M);
        
        % 用最优粒子位姿更新全局OG地图
        [~, best_idx] = max([particles.w]);
        best_pose = particles(best_idx).xv;
        og_map = update_occupancy_grid(og_map, ranges, angles, best_pose, sensor_params);
        
        % 存储
        trajectory(:, t) = x_true;
        best_pose_history(:, t) = best_pose;
        entropy_history(t) = belief.current_entropy;
        state_log{t} = state.mode;
        
        if mod(t, 100) == 0
            maps{map_idx} = og_map.getOccupancyMatrix();
            map_idx = map_idx + 1;
        end
        
        %% ==============================================================
        %  [低频入口] Step 4: 状态机 + 触发条件
        %% ==============================================================
        trigger_params.time_counter = trigger_params.time_counter + 1;
        [state, trigger_params] = state_machine_update(state, belief, trigger_params);
        
        if state.need_decision
            fprintf('[%d] Decision triggered! Reason: %s | Pose uncertainty: %.4f | Unknown: %.2f%%\n', ...
                t, trigger_params.reason, det(belief.pose_cov), og_map.unknownRatio()*100);
            
            %% ==========================================================
            %  [低频] Step 5: 主动决策 — 多准则效用融合
            %% ==========================================================
            
            % 前沿检测（WFD：从当前位姿出发的双层BFS）
            frontiers = detect_frontiers_og(og_map, best_pose, 5);
            
            if isempty(frontiers)
                state.mode = 'TERMINATED';
                state.exploration_complete = true;
                fprintf('Exploration completed at step %d (no frontiers)\n', t);
                break;
            end
            
            % 过滤过近的前沿（避免机器人反复选择脚下同一个点）
            dists = sqrt(sum((frontiers - best_pose(1:2)').^2, 2));
            frontiers = frontiers(dists > 0.8, :);  % 至少距离当前位置0.8m
            
            if isempty(frontiers)
                % 无合适前沿，向前直行探索
                frontiers = best_pose(1:2)' + [cos(best_pose(3)), sin(best_pose(3))] * 2.0;
            end
            
            % 限制候选数量
            n_candidates = min(size(frontiers, 1), 15);
            % 按距当前位置距离排序，优先评估近的前沿
            dists = sqrt(sum((frontiers - best_pose(1:2)').^2, 2));
            [~, sort_idx] = sort(dists, 'ascend');
            candidate_goals = frontiers(sort_idx(1:n_candidates), :);
            
            % 关键修复：过滤直线路径被墙阻挡的候选目标。
            % 效用函数里的路径熵只算不确定栅格，不算被 occupied 墙挡住的情况。
            reachable_mask = true(size(candidate_goals, 1), 1);
            for icand = 1:size(candidate_goals, 1)
                reachable_mask(icand) = ~is_path_blocked(og_map, best_pose(1:2)', candidate_goals(icand, :));
            end
            candidate_goals = candidate_goals(reachable_mask, :);
            
            if isempty(candidate_goals)
                % 无可达前沿，向前直行探索
                candidate_goals = best_pose(1:2)' + [cos(best_pose(3)), sin(best_pose(3))] * 2.0;
            end
            
            % 多准则效用融合
            [utilities, best_idx_local, criteria] = multi_criteria_utility_fusion(...
                particles, og_map, candidate_goals, best_pose, utility_params);
            
            % 如果效用全为NaN，回退到距离最近的前沿
            if all(isnan(utilities))
                best_idx_local = 1;
            end
            
            best_goal = candidate_goals(best_idx_local, :);
            
            % === A* 路径规划 ===
            % 在OG地图上规划从当前位姿到best_goal的可行路径
            path_world = astar_og(og_map, best_pose(1:2)', best_goal, robot_radius);
            if ~isempty(path_world)
                state.current_path = path_world;
                state.current_wp_idx = 2;  % 从路径第二个点开始（第一个是起点）
            else
                % A* 失败，直接走直线（至少已在候选阶段过滤了被墙挡的）
                state.current_path = [best_pose(1:2)'; best_goal];
                state.current_wp_idx = 2;
            end
            
            % 构建目标姿态
            dx = best_goal(1) - best_pose(1);
            dy = best_goal(2) - best_pose(2);
            target_theta = atan2(dy, dx);
            
            state.current_goal = [best_goal(1); best_goal(2); target_theta];
            state.need_decision = false;
            state.mode = 'EXECUTING';
            
            % 保存准则日志
            criteria_log{criteria_idx} = criteria;
            criteria_idx = criteria_idx + 1;
            
            % 更新可视化缓存
            last_criteria = criteria;
            last_frontiers = frontiers;
            last_best_goal = best_goal;
        end
        
        %% ==============================================================
        %  Step 6: 终止条件
        %% ==============================================================
        if og_map.unknownRatio() < 0.05
            state.mode = 'TERMINATED';
            fprintf('Exploration completed at step %d (unknown < 5%%)\n', t);
            break;
        end
        
        if mod(t, 100) == 0
            fprintf('Step %d | Mode: %s | Pose err: %.3f | Unknown: %.2f%%\n', ...
                t, state.mode, norm(x_true(1:2)-best_pose(1:2)), og_map.unknownRatio()*100);
        end
        
        %% ========== 实时可视化更新 ==========
        if enable_visualization && mod(t, 5) == 0
            visualize_realtime('update', og_map, particles, x_true, best_pose, ...
                trajectory, t, state, trigger_params, belief, ...
                last_frontiers, last_best_goal, last_criteria);
            % 检查用户是否手动关闭了可视化窗口
            fig = getappdata(0, 'active_slam_fig_handle');
            if ~isempty(fig) && ~isvalid(fig)
                enable_visualization = false;
                rmappdata(0, 'active_slam_fig_handle');
                fprintf('[%d] Visualization window closed by user. Continuing without display.\n', t);
            end
        end
    end
    
    % 裁剪历史
    trajectory = trajectory(:, 1:t);
    best_pose_history = best_pose_history(:, 1:t);
    entropy_history = entropy_history(1:t);
    state_log = state_log(1:t);
    criteria_log = criteria_log(1:criteria_idx-1);
    
    %% ========== 最终可视化 ==========
    if enable_visualization
        visualize_realtime('update', og_map, particles, x_true, best_pose, ...
            trajectory, t, state, trigger_params, belief, ...
            last_frontiers, last_best_goal, last_criteria);
    end
    visualize_final(og_map, trajectory, state_log, entropy_history);
end

%% =====================================================================
%  辅助函数
%% =====================================================================

function p = init_particles(np)
% 初始化FastSLAM粒子（复用Tim Bailey的格式）
    for i = 1:np
        p(i).w = 1/np;
        p(i).xv = [0; 0; 0];
        p(i).Pv = zeros(3);
        p(i).xf = [];
        p(i).Pf = [];
        p(i).da = [];
    end
end

function belief = extract_belief(particles, M)
% 从粒子云提取信念状态（含NaN/Inf清洗）
    poses = reshape([particles.xv], 3, M);
    weights = [particles.w];
    
    % 清洗NaN/Inf粒子
    valid = ~any(isnan(poses) | isinf(poses), 1);
    if ~all(valid)
        poses = poses(:, valid);
        weights = weights(valid);
        if isempty(weights)
            % 全部异常，回退到初始状态
            belief.pose_mean = [0; 0; 0];
            belief.pose_cov = eye(3);
            belief.Neff = 1;
            belief.current_entropy = 10;
            return;
        end
        weights = weights / sum(weights);
    end
    
    belief.pose_mean = sum(poses .* weights, 2);
    belief.pose_mean(3) = atan2(sin(belief.pose_mean(3)), cos(belief.pose_mean(3)));
    
    centered = poses - belief.pose_mean;
    centered(3, :) = atan2(sin(centered(3, :)), cos(centered(3, :)));
    belief.pose_cov = (centered .* weights) * centered' + 1e-6*eye(3);
    belief.pose_cov = 0.5*(belief.pose_cov + belief.pose_cov');
    
    belief.Neff = 1 / sum(weights.^2);
    det_cov = det(belief.pose_cov);
    if isnan(det_cov) || det_cov <= 0
        belief.pose_cov = eye(3) * 0.01;
        det_cov = det(belief.pose_cov);
    end
    belief.current_entropy = 0.5 * log((2*pi*exp(1))^3 * det_cov);
end

function walls = generate_simple_environment()
% 生成简单仿真环境：矩形边界 + 内部障碍物（线段表示）
%   输出: N×4 矩阵，每行 [x1, y1, x2, y2] 表示一堵墙/线段
    walls = [
        % 外墙 (20m x 20m)
        -9, -9,  9, -9;
         9, -9,  9,  9;
         9,  9, -9,  9;
        -9,  9, -9, -9;
        
        % 内部障碍物（线段）
        -3, -3,  3, -3;   % 下横墙
        -3,  3,  3,  3;   % 上横墙
        -3,  0, -3,  3;   % 左竖墙
         3,  0,  3,  3;   % 右竖墙
         0, -1,  0, -3;   % 中竖墙
    ];
end

function [ranges, angles, odom] = sim_sensor_step(x_true, current_goal, env_walls, sensor_params, dt)
% 仿真传感器步进（实体线段障碍物 + 射线求交）
    % 纯追踪控制器
    dx = current_goal(1) - x_true(1);
    dy = current_goal(2) - x_true(2);
    target_heading = atan2(dy, dx);
    heading_error = atan2(sin(target_heading - x_true(3)), cos(target_heading - x_true(3)));
    dist_to_goal = sqrt(dx^2 + dy^2);
    
    v_max = 1.0;
    omega_max = 0.5;
    v_cmd = min(v_max, dist_to_goal * 0.5);
    omega_cmd = max(-omega_max, min(omega_max, heading_error * 2.0));
    
    if dist_to_goal < 0.5
        v_cmd = 0;
        omega_cmd = 0;
    end
    
    odom = [v_cmd; omega_cmd];
    
    % 模拟激光扫描：射线与线段求交
    N_beams = 72;  % 每5度一束
    angles = linspace(-pi, pi, N_beams)';
    ranges = sensor_params.range_max * ones(N_beams, 1);
    
    for i = 1:N_beams
        theta = x_true(3) + angles(i);
        dir = [cos(theta), sin(theta)];
        
        t = ray_segment_intersection(x_true(1:2)', dir, env_walls);
        if t < inf && t < ranges(i)
            ranges(i) = t;
        end
    end
    
    % 加噪声
    ranges = ranges + 0.05 * randn(size(ranges));
    ranges = max(sensor_params.range_min, min(ranges, sensor_params.range_max));
end

function x_new = sim_motion_step(x, odom, dt, env_walls, robot_radius)
% 仿真运动步进（差速模型 + 线段碰撞检测）
    if nargin < 5
        robot_radius = 0.3;
    end
    
    v = odom(1);
    w = odom(2);
    x_new = x;
    x_new(1) = x(1) + v * dt * cos(x(3) + w*dt/2);
    x_new(2) = x(2) + v * dt * sin(x(3) + w*dt/2);
    x_new(3) = x(3) + w * dt;
    x_new(3) = atan2(sin(x_new(3)), cos(x_new(3)));
    
    % 碰撞检测与响应（二分查找安全位置）
    if nargin >= 4 && ~isempty(env_walls)
        min_dist = point_to_segment_distance(x_new(1:2)', env_walls);
        if min_dist < robot_radius
            % 发生碰撞：从旧位置向新位置二分查找刚好不碰的位置
            x_old_pos = x(1:2);
            x_new_pos = x_new(1:2);
            safe_pos = x_old_pos;
            for step = 0.5.^(1:6)
                test_pos = safe_pos + step * (x_new_pos - x_old_pos);
                if point_to_segment_distance(test_pos', env_walls) >= robot_radius
                    safe_pos = test_pos;
                end
            end
            x_new(1:2) = safe_pos;
        end
    end
    
    % 加运动噪声
    x_new = x_new + [0.02*randn(); 0.02*randn(); 0.01*randn()];
end

function visualize_final(og_map, trajectory, state_log, entropy_history)
% 最终可视化
    figure('Name', 'Active SLAM Result');
    
    subplot(2, 2, 1);
    imagesc(og_map.Origin(1):og_map.Resolution:(og_map.Origin(1)+og_map.Width), ...
            og_map.Origin(2):og_map.Resolution:(og_map.Origin(2)+og_map.Height), ...
            og_map.getOccupancyMatrix());
    set(gca, 'YDir', 'normal');
    hold on;
    plot(trajectory(1, :), trajectory(2, :), 'b-', 'LineWidth', 1.5);
    plot(trajectory(1, 1), trajectory(2, 1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(trajectory(1, end), trajectory(2, end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title('OG Map & Trajectory');
    xlabel('x [m]'); ylabel('y [m]');
    axis equal; colorbar;
    
    subplot(2, 2, 2);
    plot(entropy_history, 'k-', 'LineWidth', 1);
    title('Pose Entropy History');
    xlabel('Step'); ylabel('Entropy');
    grid on;
    
    subplot(2, 2, 3);
    % 状态机历史
    state_numeric = zeros(size(state_log));
    modes = {'INIT', 'EXECUTING', 'REPLANNING', 'LOOP_CLOSURE', 'TERMINATED'};
    for i = 1:length(modes)
        state_numeric(strcmp(state_log, modes{i})) = i;
    end
    stairs(state_numeric, 'LineWidth', 1.5);
    set(gca, 'YTick', 1:length(modes), 'YTickLabel', modes);
    title('State Machine');
    xlabel('Step'); ylabel('State');
    grid on;
    
    subplot(2, 2, 4);
    % 轨迹误差（如果有真值的话，这里用起点到终点的累积）
    plot(trajectory(1, :), trajectory(2, :), 'b.-');
    title('Trajectory');
    xlabel('x [m]'); ylabel('y [m]');
    axis equal; grid on;
end

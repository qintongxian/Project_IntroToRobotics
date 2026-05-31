function og_map = update_occupancy_grid(og_map, ranges, angles, robot_pose, sensor_params)
%UPDATE_OCCUPANCY_GRID 使用逆传感器模型更新OG地图，带射线截断防穿墙
%   输入:
%     og_map     — OccupancyGridMap 对象
%     ranges     — N x 1 激光距离数组 [m]
%     angles     — N x 1 激光角度数组 [rad]，相对于机器人朝向
%     robot_pose — [x; y; theta]，机器人世界位姿
%     sensor_params — 结构体
%
%   关键修复：同一帧开始时取一次地图状态，若射线路径上已有高置信度
%   occupied栅格(p>0.6)，则截断射线，不更新其后方区域。这防止了定位
%   漂移时激光"穿过"地图上的墙、错误更新墙后区域。

    if nargin < 5
        sensor_params = struct();
    end
    if ~isfield(sensor_params, 'range_max'), sensor_params.range_max = 10.0; end
    if ~isfield(sensor_params, 'range_min'), sensor_params.range_min = 0.1; end
    if ~isfield(sensor_params, 'P_occ'),    sensor_params.P_occ = 0.6; end
    if ~isfield(sensor_params, 'P_free'),   sensor_params.P_free = 0.35; end
    
    % log-odds 增量
    log_occ  = log(sensor_params.P_occ / (1 - sensor_params.P_occ));
    log_free = log(sensor_params.P_free / (1 - sensor_params.P_free));
    
    x_r = robot_pose(1);
    y_r = robot_pose(2);
    theta_r = robot_pose(3);
    
    [robot_i, robot_j] = og_map.world2grid(x_r, y_r);
    
    % 预取本帧初始地图状态，用于截断判断
    occ = og_map.Grid;
    [H, W] = size(occ);
    occ_thresh = 0.6;  % 高于此值视为已建好的墙，后方截断不更新
    
    for beam = 1:length(ranges)
        r = ranges(beam);
        
        % 过滤无效测量
        if r >= sensor_params.range_max || r <= sensor_params.range_min || isnan(r) || isinf(r)
            continue;
        end
        
        phi = angles(beam);
        
        % 击中点世界坐标
        hit_x = x_r + r * cos(theta_r + phi);
        hit_y = y_r + r * sin(theta_r + phi);
        
        % 转换为网格坐标
        [hit_i, hit_j] = og_map.world2grid(hit_x, hit_y);
        
        % Bresenham 获取射线经过的所有单元格
        [cells_i, cells_j] = bresenham_line(robot_i, robot_j, hit_i, hit_j);
        
        % === 关键修复：射线截断 ===
        % 若射线路径上已有高置信度 occupied 栅格，截断到该栅格为止
        truncated = false;
        for c = 2:length(cells_i)  % 从第2个开始，避免起点边界误判
            ci = cells_i(c);
            cj = cells_j(c);
            if ci >= 1 && ci <= H && cj >= 1 && cj <= W
                if occ(ci, cj) > occ_thresh
                    cells_i = cells_i(1:c);
                    cells_j = cells_j(1:c);
                    truncated = true;
                    break;
                end
            end
        end
        
        % 更新射线上的单元格（除最后一个为自由）
        if length(cells_i) > 1
            free_pts = zeros(length(cells_i)-1, 2);
            for c = 1:length(cells_i)-1
                [fx, fy] = og_map.grid2world(cells_i(c), cells_j(c));
                free_pts(c, :) = [fx, fy];
            end
            og_map.setOccupancy(free_pts, log_free, 'logodds');
        end
        
        % 最后一个单元格（击中点）为占据（仅当未被截断时）
        if ~truncated
            [hx, hy] = og_map.grid2world(cells_i(end), cells_j(end));
            og_map.setOccupancy([hx, hy], log_occ, 'logodds');
        end
    end
end

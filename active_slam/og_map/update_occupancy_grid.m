function og_map = update_occupancy_grid(og_map, ranges, angles, robot_pose, sensor_params)
%UPDATE_OCCUPANCY_GRID 使用逆传感器模型更新OG地图
%   输入:
%     og_map     — OccupancyGridMap 对象
%     ranges     — N x 1 激光距离数组 [m]
%     angles     — N x 1 激光角度数组 [rad]，相对于机器人朝向
%     robot_pose — [x; y; theta]，机器人世界位姿
%     sensor_params — 结构体，包含:
%         .range_max    — 最大量程 [m] (默认 10)
%         .range_min    — 最小有效距离 [m] (默认 0.1)
%         .P_occ        — 击中占据概率 (默认 0.6)
%         .P_free       — 射线自由概率 (默认 0.35)
%
%   输出:
%     og_map — 更新后的 OccupancyGridMap 对象

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
    
    % 机器人网格坐标
    [robot_i, robot_j] = og_map.world2grid(x_r, y_r);
    
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
        
        % 更新射线上的单元格（除最后一个为自由）
        if length(cells_i) > 1
            free_pts = zeros(length(cells_i)-1, 2);
            for c = 1:length(cells_i)-1
                [fx, fy] = og_map.grid2world(cells_i(c), cells_j(c));
                free_pts(c, :) = [fx, fy];
            end
            og_map.setOccupancy(free_pts, log_free, 'logodds');
        end
        
        % 最后一个单元格（击中点）为占据
        [hx, hy] = og_map.grid2world(cells_i(end), cells_j(end));
        og_map.setOccupancy([hx, hy], log_occ, 'logodds');
    end
end

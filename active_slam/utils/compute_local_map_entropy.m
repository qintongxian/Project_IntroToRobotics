function H_map = compute_local_map_entropy(og_map, pose, sensor_range)
%COMPUTE_LOCAL_MAP_ENTROPY 计算传感器FOV覆盖区域内的Shannon熵
%   输入:
%     og_map      — OccupancyGridMap 对象
%     pose        — [x; y; theta]，传感器位姿
%     sensor_range— 传感器最大量程 [m]
%   输出:
%     H_map       — FOV内所有不确定单元格的Shannon熵之和

    % 圆形FOV内的候选单元格（简化：以pose为中心，sensor_range为半径的圆）
    % 实际激光雷达FOV可能是扇形，但圆形覆盖更保守且计算简单
    res = og_map.Resolution;
    n_cells = ceil(sensor_range / res);
    
    [center_i, center_j] = og_map.world2grid(pose(1), pose(2));
    
    H_map = 0;
    
    for di = -n_cells:n_cells
        for dj = -n_cells:n_cells
            i = center_i + di;
            j = center_j + dj;
            
            % 检查是否在圆形范围内
            if (di*di + dj*dj) > n_cells*n_cells
                continue;
            end
            
            % 转换为世界坐标并查询概率
            [wx, wy] = og_map.grid2world(i, j);
            p = og_map.getOccupancy([wx, wy]);
            
            % 只统计不确定单元格 (0.01 < p < 0.99)
            if p > 0.01 && p < 0.99
                % Shannon 二元熵
                H_cell = -(p*log(p) + (1-p)*log(1-p));
                H_map = H_map + H_cell;
            end
        end
    end
end

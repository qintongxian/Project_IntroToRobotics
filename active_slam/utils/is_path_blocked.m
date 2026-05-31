function blocked = is_path_blocked(og_map, start_pos, end_pos, occ_threshold)
%IS_PATH_BLOCKED 检查从起点到终点的直线路径是否穿过 occupied 栅格
%   使用 Bresenham 直线算法在 OG 地图上采样路径
%
%   输入:
%     og_map        — OccupancyGridMap 对象
%     start_pos     — [x, y] 起点世界坐标
%     end_pos       — [x, y] 终点世界坐标
%     occ_threshold — 占据阈值 (默认 0.65)
%
%   输出:
%     blocked — true/false

    if nargin < 4
        occ_threshold = 0.55;  % 单帧激光更新后墙概率≈0.6，0.65太高会漏检
    end
    
    [s_i, s_j] = og_map.world2grid(start_pos(1), start_pos(2));
    [e_i, e_j] = og_map.world2grid(end_pos(1), end_pos(2));
    
    [cells_i, cells_j] = bresenham_line(s_i, s_j, e_i, e_j);
    
    occ = og_map.getOccupancyMatrix();
    [H, W] = size(occ);
    
    % 跳过起点（避免机器人贴墙时起点本身在 occupied 边界上被误判）
    for c = 2:length(cells_i)
        ci = cells_i(c);
        cj = cells_j(c);
        if ci >= 1 && ci <= H && cj >= 1 && cj <= W
            if occ(ci, cj) > occ_threshold
                blocked = true;
                return;
            end
        end
    end
    blocked = false;
end

function H_path = compute_path_entropy(og_map, start_pos, end_pos)
%COMPUTE_PATH_ENTROPY 计算从起点到终点直线路径上的栅格熵之和
%   输入:
%     og_map   — OccupancyGridMap 对象
%     start_pos— [x, y] 起点世界坐标
%     end_pos  — [x, y] 终点世界坐标
%   输出:
%     H_path   — 路径上所有不确定单元格的熵之和（越大越危险/未知）

    [s_i, s_j] = og_map.world2grid(start_pos(1), start_pos(2));
    [e_i, e_j] = og_map.world2grid(end_pos(1), end_pos(2));
    
    [cells_i, cells_j] = bresenham_line(s_i, s_j, e_i, e_j);
    
    H_path = 0;
    for c = 1:length(cells_i)
        [wx, wy] = og_map.grid2world(cells_i(c), cells_j(c));
        p = og_map.getOccupancy([wx, wy]);
        
        if p > 0.01 && p < 0.99
            H_path = H_path + (-p*log(p) - (1-p)*log(1-p));
        end
    end
end

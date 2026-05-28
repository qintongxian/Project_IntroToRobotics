function H_renyi = compute_renyi_entropy_local(og_map, pose, sensor_range, alpha)
%COMPUTE_RENYI_ENTROPY_LOCAL 计算FOV内的Rényi熵
%   输入:
%     og_map      — OccupancyGridMap 对象
%     pose        — [x; y; theta]
%     sensor_range— 最大量程 [m]
%     alpha       — Rényi参数 (>1时折扣更激进)
%   输出:
%     H_renyi     — Rényi熵值

    if alpha <= 0 || abs(alpha - 1) < 1e-6
        % alpha=1 退化为Shannon熵，直接调用
        H_renyi = compute_local_map_entropy(og_map, pose, sensor_range);
        return;
    end
    
    res = og_map.Resolution;
    n_cells = ceil(sensor_range / res);
    [center_i, center_j] = og_map.world2grid(pose(1), pose(2));
    
    sum_p_alpha = 0;
    count = 0;
    
    for di = -n_cells:n_cells
        for dj = -n_cells:n_cells
            if (di*di + dj*dj) > n_cells*n_cells
                continue;
            end
            
            i = center_i + di;
            j = center_j + dj;
            [wx, wy] = og_map.grid2world(i, j);
            p = og_map.getOccupancy([wx, wy]);
            
            if p > 0.01 && p < 0.99
                sum_p_alpha = sum_p_alpha + p^alpha;
                count = count + 1;
            end
        end
    end
    
    if count == 0 || sum_p_alpha <= 0
        H_renyi = 0;
    else
        H_renyi = (1 / (1 - alpha)) * log(sum_p_alpha);
    end
end

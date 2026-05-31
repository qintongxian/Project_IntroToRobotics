function path_world = astar_og(og_map, start_pos, goal_pos, robot_radius)
%ASTAR_OG 在OccupancyGridMap上做A*路径规划
%   输入:
%     og_map       — OccupancyGridMap对象
%     start_pos    — [x, y] 起点世界坐标
%     goal_pos     — [x, y] 终点世界坐标
%     robot_radius — 机器人碰撞半径 [m] (默认0.3)
%   输出:
%     path_world   — N×2 路点世界坐标 [x, y]，空矩阵表示无可行路径

    if nargin < 4
        robot_radius = 0.3;
    end
    
    occ = og_map.Grid;
    [H, W] = size(occ);
    res = og_map.Resolution;
    
    %% ========== 障碍物膨胀（圆形膨胀）==========
    r_cells = ceil(robot_radius / res) + 1;  % +1格安全余量
    inflated = false(H, W);
    [obs_i, obs_j] = find(occ > 0.55);
    for k = 1:length(obs_i)
        oi = obs_i(k); oj = obs_j(k);
        di_range = max(1-oi, -r_cells):min(H-oi, r_cells);
        for di = di_range
            dj_max = floor(sqrt(r_cells^2 - di^2));
            dj_range = max(1-oj, -dj_max):min(W-oj, dj_max);
            for dj = dj_range
                inflated(oi+di, oj+dj) = true;
            end
        end
    end
    
    %% ========== 起点/终点栅格化 ==========
    [si, sj] = og_map.world2grid(start_pos(1), start_pos(2));
    [gi, gj] = og_map.world2grid(goal_pos(1), goal_pos(2));
    si = max(1, min(H, round(si))); sj = max(1, min(W, round(sj)));
    gi = max(1, min(H, round(gi))); gj = max(1, min(W, round(gj)));
    
    if inflated(si, sj) || inflated(gi, gj)
        path_world = [];
        return;
    end
    
    %% ========== A* 搜索 ==========
    g_score = inf(H, W);
    g_score(si, sj) = 0;
    closed = false(H, W);
    parent_i = zeros(H, W, 'int32');
    parent_j = zeros(H, W, 'int32');
    
    open_i = int32(si);
    open_j = int32(sj);
    
    while ~isempty(open_i)
        % 从 open set 中找 f = g + h 最小的节点
        f_vals = g_score(sub2ind([H, W], double(open_i), double(open_j))) + ...
                 sqrt((double(open_i) - gi).^2 + (double(open_j) - gj).^2) * res;
        [~, min_idx] = min(f_vals);
        ci = double(open_i(min_idx));
        cj = double(open_j(min_idx));
        open_i(min_idx) = [];
        open_j(min_idx) = [];
        
        if closed(ci, cj)
            continue;
        end
        closed(ci, cj) = true;
        
        if ci == gi && cj == gj
            % 重建路径
            path_grid = [];
            pi = gi; pj = gj;
            while ~(pi == si && pj == sj)
                path_grid = [pi, pj; path_grid]; %#ok<AGROW>
                pi_next = double(parent_i(pi, pj));
                pj_next = double(parent_j(pi, pj));
                if pi_next == 0 && pj_next == 0
                    break;
                end
                pi = pi_next; pj = pj_next;
            end
            path_grid = [si, sj; path_grid; gi, gj];
            
            % 转为世界坐标（栅格中心）
            path_world = zeros(size(path_grid, 1), 2);
            for k = 1:size(path_grid, 1)
                [path_world(k,1), path_world(k,2)] = og_map.grid2world(path_grid(k,1), path_grid(k,2));
            end
            return;
        end
        
        % 8-邻域扩展
        for di = -1:1
            for dj = -1:1
                if di == 0 && dj == 0
                    continue;
                end
                ni = ci + di;
                nj = cj + dj;
                if ni < 1 || ni > H || nj < 1 || nj > W
                    continue;
                end
                if inflated(ni, nj) || closed(ni, nj)
                    continue;
                end
                
                tentative_g = g_score(ci, cj) + sqrt(di^2 + dj^2) * res;
                if tentative_g < g_score(ni, nj)
                    g_score(ni, nj) = tentative_g;
                    parent_i(ni, nj) = int32(ci);
                    parent_j(ni, nj) = int32(cj);
                    open_i(end+1) = int32(ni); %#ok<AGROW>
                    open_j(end+1) = int32(nj); %#ok<AGROW>
                end
            end
        end
    end
    
    path_world = [];  % 无可行路径
end

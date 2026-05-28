function frontiers = detect_frontiers_og(og_map, min_size)
%DETECT_FRONTIERS_OG 在OG地图上检测前沿边界
%   输入:
%     og_map  — OccupancyGridMap 对象
%     min_size— 前沿最小连通区域大小（单元格数），默认 5
%   输出:
%     frontiers — N x 2 矩阵，每行是一个前沿中心的世界坐标 [x, y]
%                 若为空表示无前沿（探索完成）

    if nargin < 2
        min_size = 5;
    end
    
    occ = og_map.getOccupancyMatrix();
    
    % 未知区域: occ == 0.5
    % 已知自由: 0 < occ < 0.5
    % 已知占据: occ > 0.5
    unknown = abs(occ - 0.5) < 0.01;
    free = (occ < 0.5) & (occ > 0.01);
    
    % 前沿 = 与未知区域相邻的已知自由单元格
    frontier_mask = false(size(occ));
    for di = -1:1
        for dj = -1:1
            if di == 0 && dj == 0
                continue;
            end
            shifted_unknown = circshift(unknown, [di, dj]);
            frontier_mask = frontier_mask | (free & shifted_unknown);
        end
    end
    
    % 边缘裁剪（地图边界不视为前沿）
    frontier_mask(1, :) = false;
    frontier_mask(end, :) = false;
    frontier_mask(:, 1) = false;
    frontier_mask(:, end) = false;
    
    % 连通分量分析
    if exist('bwconncomp', 'file')
        CC = bwconncomp(frontier_mask, 8);
        frontiers = [];
        for k = 1:CC.NumObjects
            if length(CC.PixelIdxList{k}) < min_size
                continue;
            end
            [rows, cols] = ind2sub(size(occ), CC.PixelIdxList{k});
            center_x = mean(cols);
            center_y = mean(rows);
            [wx, wy] = og_map.grid2world(center_y, center_x);
            frontiers = [frontiers; wx, wy];
        end
    else
        % 无Image Processing Toolbox时的简易连通分量分析（4连通flood fill）
        frontiers = simple_connected_components(og_map, frontier_mask, min_size);
    end
end

function frontiers = simple_connected_components(og_map, mask, min_size)
% 简易连通分量分析（无需bwconncomp）
    [H, W] = size(mask);
    visited = false(H, W);
    frontiers = [];
    
    for i = 2:H-1
        for j = 2:W-1
            if mask(i, j) && ~visited(i, j)
                % BFS flood fill
                queue = [i, j];
                visited(i, j) = true;
                comp_i = i;
                comp_j = j;
                qhead = 1;
                
                while qhead <= size(queue, 1)
                    ci = queue(qhead, 1);
                    cj = queue(qhead, 2);
                    qhead = qhead + 1;
                    
                    for di = -1:1
                        for dj = -1:1
                            if di == 0 && dj == 0, continue; end
                            ni = ci + di;
                            nj = cj + dj;
                            if ni >= 1 && ni <= H && nj >= 1 && nj <= W
                                if mask(ni, nj) && ~visited(ni, nj)
                                    visited(ni, nj) = true;
                                    queue = [queue; ni, nj];
                                    comp_i = [comp_i; ni];
                                    comp_j = [comp_j; nj];
                                end
                            end
                        end
                    end
                end
                
                if length(comp_i) >= min_size
                    [wx, wy] = og_map.grid2world(mean(comp_i), mean(comp_j));
                    frontiers = [frontiers; wx, wy];
                end
            end
        end
    end
end

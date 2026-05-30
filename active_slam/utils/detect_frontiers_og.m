function frontiers = detect_frontiers_og(og_map, robot_pose, min_size)
%DETECT_FRONTIERS_OG 基于WFD(Wavefront Frontier Detector)算法的前沿检测
%   采用双层BFS结构，从机器人当前位置出发仅在已知free space内搜索，
%   将复杂度从全图遍历O(|M|)降至O(|S_known| + P_known)。
%
%   根据Keidar & Kaminka (2014)，frontier cell定义为：
%   占据概率为未知(p≈0.5)，且在8-邻域中至少有一个free space邻居的单元格。
%
%   输入:
%     og_map    — OccupancyGridMap 对象
%     robot_pose— [x; y; theta] 机器人当前位姿（可选）。提供位姿时WFD从
%                 该位置开始搜索；省略时自动寻找地图中第一个free cell。
%     min_size  — 前沿最小连通区域大小（单元格数），默认 5（可选）
%
%   输出:
%     frontiers — N x 2 矩阵，每行是一个前沿中心的世界坐标 [x, y]
%                 若为空表示无前沿（探索完成）

    %% ========== 参数解析（向后兼容旧调用 detect_frontiers_og(og_map, min_size)） ==========
    if nargin < 2
        robot_pose = [];
        min_size = 5;
    elseif nargin < 3
        if isnumeric(robot_pose) && isscalar(robot_pose)
            % 旧调用格式: detect_frontiers_og(og_map, min_size)
            min_size = robot_pose;
            robot_pose = [];
        else
            min_size = 5;
        end
    end
    
    occ = og_map.getOccupancyMatrix();
    [H, W] = size(occ);
    tol = 0.01;
    
    %% ========== 确定WFD外层BFS起始点（必须在free space内） ==========
    si = []; sj = [];
    if ~isempty(robot_pose)
        [si, sj] = og_map.world2grid(robot_pose(1), robot_pose(2));
        if si < 1 || si > H || sj < 1 || sj > W
            si = []; sj = [];
        elseif ~(occ(si, sj) < 0.5 - tol && occ(si, sj) > tol)
            si = []; sj = [];
        end
    end
    
    if isempty(si)
        free_mask = (occ < 0.5 - tol) & (occ > tol);
        if ~any(free_mask(:))
            frontiers = zeros(0, 2);
            return;
        end
        [si, sj] = find(free_mask, 1);
    end
    
    %% ========== WFD双层BFS ==========
    % 访问标记
    map_open = false(H, W);
    map_close = false(H, W);
    frontier_close = false(H, W);
    
    % 队列预分配（最多访问所有free cells）
    max_queue = H * W;
    queue = zeros(max_queue, 2, 'int32');
    qhead = 1;
    qtail = 2;
    queue(1, :) = int32([si, sj]);
    map_open(si, sj) = true;
    
    frontier_centers = cell(0, 1);
    
    while qhead < qtail
        p_i = double(queue(qhead, 1));
        p_j = double(queue(qhead, 2));
        qhead = qhead + 1;
        
        if map_close(p_i, p_j)
            continue;
        end
        
        % 扫描8邻域：检测frontier cell + 扩展free space
        for di = -1:1
            for dj = -1:1
                if di == 0 && dj == 0
                    continue;
                end
                ni = p_i + di;
                nj = p_j + dj;
                
                if ni < 1 || ni > H || nj < 1 || nj > W
                    continue;
                end
                
                % ---- 检查是否为未处理的frontier cell ----
                if ~frontier_close(ni, nj) && is_frontier_cell(occ, ni, nj, H, W, tol)
                    % ===== 内层BFS：提取整个连通frontier segment =====
                    seg_queue = zeros(max_queue, 2, 'int32');
                    seg_queue(1, :) = int32([ni, nj]);
                    seg_head = 1;
                    seg_tail = 2;
                    frontier_close(ni, nj) = true;
                    
                    % 记录segment单元格坐标用于计算中心
                    seg_cells = zeros(max_queue, 2, 'int32');
                    seg_cells(1, :) = int32([ni, nj]);
                    seg_count = 1;
                    
                    while seg_head < seg_tail
                        s_i = double(seg_queue(seg_head, 1));
                        s_j = double(seg_queue(seg_head, 2));
                        seg_head = seg_head + 1;
                        
                        for sdi = -1:1
                            for sdj = -1:1
                                if sdi == 0 && sdj == 0
                                    continue;
                                end
                                nni = s_i + sdi;
                                nnj = s_j + sdj;
                                
                                if nni < 1 || nni > H || nnj < 1 || nnj > W
                                    continue;
                                end
                                if frontier_close(nni, nnj)
                                    continue;
                                end
                                
                                if is_frontier_cell(occ, nni, nnj, H, W, tol)
                                    frontier_close(nni, nnj) = true;
                                    seg_queue(seg_tail, :) = int32([nni, nnj]);
                                    seg_tail = seg_tail + 1;
                                    seg_count = seg_count + 1;
                                    seg_cells(seg_count, :) = int32([nni, nnj]);
                                end
                            end
                        end
                    end
                    
                    if seg_count >= min_size
                        ci = mean(double(seg_cells(1:seg_count, 1)));
                        cj = mean(double(seg_cells(1:seg_count, 2)));
                        [wx, wy] = og_map.grid2world(ci, cj);
                        frontier_centers{end+1} = [wx, wy];
                    end
                end
                
                % ---- 将free邻居加入外层BFS队列 ----
                if ~map_open(ni, nj) && ~map_close(ni, nj)
                    if occ(ni, nj) < 0.5 - tol && occ(ni, nj) > tol
                        queue(qtail, :) = int32([ni, nj]);
                        qtail = qtail + 1;
                        map_open(ni, nj) = true;
                    end
                end
            end
        end
        
        map_close(p_i, p_j) = true;
    end
    
    %% ========== 组装输出 ==========
    if isempty(frontier_centers)
        frontiers = zeros(0, 2);
    else
        frontiers = cell2mat(frontier_centers');
    end
end

%% ========================================================================
function flag = is_frontier_cell(occ, i, j, H, W, tol)
%IS_FRONTIER_CELL 判断单元格是否为frontier cell
%   frontier cell: 未知(p≈0.5) 且 8-邻域中至少有一个free邻居
%   地图边界单元格永远不被视为frontier

    flag = false;
    if i <= 1 || i >= H || j <= 1 || j >= W
        return;  % 边界裁剪
    end
    if abs(occ(i, j) - 0.5) >= tol
        return;  % 不是未知
    end
    
    % 检查8邻域是否有free cell
    for di = -1:1
        for dj = -1:1
            if di == 0 && dj == 0
                continue;
            end
            fi = i + di;
            fj = j + dj;
            if occ(fi, fj) < 0.5 - tol && occ(fi, fj) > tol
                flag = true;
                return;
            end
        end
    end
end

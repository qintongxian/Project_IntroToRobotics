function [d_min, nearest_pt] = point_to_segment_distance(pt, walls)
%POINT_TO_SEGMENT_DISTANCE 计算点到多线段集合的最近距离
%   向量化计算点 pt 到 walls 中每条线段的最近距离，并返回最小值。
%
%   输入:
%     pt    — [x, y] 或 1×2 查询点
%     walls — N×4 矩阵，每行 [x1, y1, x2, y2]
%
%   输出:
%     d_min     — 最小距离（标量）
%     nearest_pt— (可选) 最近点坐标 [x, y]

    N = size(walls, 1);
    if N == 0
        d_min = inf;
        if nargout > 1, nearest_pt = pt; end
        return;
    end
    
    pt = pt(:)';
    P1 = walls(:, 1:2);
    P2 = walls(:, 3:4);
    S = P2 - P1;
    
    diff = pt - P1;  % N×2
    len2 = sum(S.^2, 2);  % N×1
    
    % 处理零长度线段（点）
    zero_seg = len2 < eps;
    len2(zero_seg) = 1;  % 避免除零，后面会被覆盖
    
    % 投影参数 t = dot(diff, S) / |S|^2
    t = sum(diff .* S, 2) ./ len2;
    t = max(0, min(1, t));
    
    % 最近点
    closest = P1 + t .* S;
    
    % 距离
    d_vec = pt - closest;
    d = sqrt(sum(d_vec.^2, 2));
    
    % 零长度线段：直接算到端点距离
    if any(zero_seg)
        d(zero_seg) = sqrt(sum((pt - P1(zero_seg, :)).^2, 2));
    end
    
    [d_min, idx] = min(d);
    
    if nargout > 1
        nearest_pt = closest(idx, :);
    end
end

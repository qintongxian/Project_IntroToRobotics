function t_hit = ray_segment_intersection(origin, dir, walls)
%RAY_SEGMENT_INTERSECTION 向量化射线-多线段最近交点计算
%   计算从 origin 出发沿 dir 方向的射线与 walls 中所有线段的最近有效交点。
%
%   输入:
%     origin — [x, y] 射线起点
%     dir    — [dx, dy] 射线方向向量（不必归一化）
%     walls  — N×4 矩阵，每行 [x1, y1, x2, y2] 表示一条线段
%
%   输出:
%     t_hit  — 最近有效交点的参数 t，使得交点 = origin + t * dir
%              若无有效交点，返回 inf

    N = size(walls, 1);
    if N == 0
        t_hit = inf;
        return;
    end
    
    P1 = walls(:, 1:2);
    P2 = walls(:, 3:4);
    S = P2 - P1;  % N×2, 线段方向向量
    
    % denom = cross(dir, S) = dir_x * S_y - dir_y * S_x
    denom = dir(1) * S(:, 2) - dir(2) * S(:, 1);
    
    tol = 1e-9;
    valid = abs(denom) > tol;  % 排除平行情况
    
    if ~any(valid)
        t_hit = inf;
        return;
    end
    
    P1v = P1(valid, :);
    Sv = S(valid, :);
    denom_v = denom(valid);
    
    diff = P1v - origin;  % M×2
    
    % t = cross(diff, Sv) / denom
    t = (diff(:, 1) .* Sv(:, 2) - diff(:, 2) .* Sv(:, 1)) ./ denom_v;
    
    % u = cross(diff, dir) / denom
    u = (diff(:, 1) * dir(2) - diff(:, 2) * dir(1)) ./ denom_v;
    
    % 有效交点: t >= 0 (射线前方), 0 <= u <= 1 (在线段上)
    hit = (t >= 0) & (u >= 0) & (u <= 1);
    
    if ~any(hit)
        t_hit = inf;
    else
        t_hit = min(t(hit));
    end
end

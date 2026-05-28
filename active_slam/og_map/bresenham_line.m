function [cells_i, cells_j] = bresenham_line(x0, y0, x1, y1)
%BRESENHAM_LINE 整数网格上的Bresenham直线算法
%   输入: (x0,y0) 起点, (x1,y1) 终点 (均为整数网格坐标)
%   输出: cells_i, cells_j — 射线经过的所有单元格坐标 (列向量)
%
%   注意: 输入输出均为 [row, col] 即 [i, j] 坐标系

    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    sx = sign(x1 - x0);
    sy = sign(y1 - y0);
    err = dx - dy;
    
    cells_i = x0;
    cells_j = y0;
    x = x0; 
    y = y0;
    
    while x ~= x1 || y ~= y1
        e2 = 2 * err;
        if e2 > -dy
            err = err - dy;
            x = x + sx;
        end
        if e2 < dx
            err = err + dx;
            y = y + sy;
        end
        cells_i = [cells_i; x];
        cells_j = [cells_j; y];
    end
end

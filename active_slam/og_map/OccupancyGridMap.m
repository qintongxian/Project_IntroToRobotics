classdef OccupancyGridMap < handle
%OCCUPANCYGRIDMAP 简化的占据栅格地图类 (不依赖Navigation Toolbox)
%   兼容 occupancyMap 的核心API: getOccupancy, setOccupancy, world2grid
%   grid 内部存储概率值 p ∈ [0,1]，0.5 表示未知
    
    properties
        Grid        % H x W 概率矩阵 (0=自由, 1=占据, 0.5=未知)
        Resolution  % 每个单元格的边长 [m/cell]
        Origin      % 左下角世界坐标 [x_origin, y_origin]
    end
    
    properties (Dependent)
        Width       % 地图宽度 [m]
        Height      % 地图高度 [m]
        GridSize    % [H, W]
    end
    
    methods
        function obj = OccupancyGridMap(width, height, resolution, origin)
            % 构造函数
            %   width, height: 地图尺寸 [m]
            %   resolution: 栅格分辨率 [m/cell]
            %   origin: (可选) 左下角世界坐标，默认 [0,0]
            if nargin < 4
                origin = [0, 0];
            end
            obj.Resolution = resolution;
            obj.Origin = origin;
            W = ceil(width / resolution);
            H = ceil(height / resolution);
            obj.Grid = 0.5 * ones(H, W);  % 初始化为未知
        end
        
        function val = get.Width(obj)
            val = size(obj.Grid, 2) * obj.Resolution;
        end
        
        function val = get.Height(obj)
            val = size(obj.Grid, 1) * obj.Resolution;
        end
        
        function val = get.GridSize(obj)
            val = size(obj.Grid);
        end
        
        function [i, j] = world2grid(obj, x, y)
            % 世界坐标 → 网格索引 (i=行/row=y方向, j=列/col=x方向)
            % 输入 x,y 可以是标量或同长度向量
            j = floor((x - obj.Origin(1)) / obj.Resolution) + 1;
            i = floor((y - obj.Origin(2)) / obj.Resolution) + 1;
        end
        
        function [x, y] = grid2world(obj, i, j)
            % 网格索引 → 世界坐标 (单元格中心)
            x = obj.Origin(1) + (j - 0.5) * obj.Resolution;
            y = obj.Origin(2) + (i - 0.5) * obj.Resolution;
        end
        
        function p = getOccupancy(obj, pts)
            % 查询世界坐标处的占据概率
            %   pts: N x 2 矩阵 [x, y]
            %   p:   N x 1 概率值 (越界返回 0.5)
            N = size(pts, 1);
            p = 0.5 * ones(N, 1);
            [i, j] = obj.world2grid(pts(:,1), pts(:,2));
            H = size(obj.Grid, 1);
            W = size(obj.Grid, 2);
            valid = (i >= 1) & (i <= H) & (j >= 1) & (j <= W);
            idx = sub2ind([H, W], i(valid), j(valid));
            p(valid) = obj.Grid(idx);
        end
        
        function setOccupancy(obj, pts, values, mode)
            % 设置世界坐标处的占据概率
            %   pts:    N x 2 矩阵 [x, y]
            %   values: 标量或 N x 1 (概率值或log-odds)
            %   mode:   'probability'(默认) 或 'logodds'
            if nargin < 4
                mode = 'probability';
            end
            [i, j] = obj.world2grid(pts(:,1), pts(:,2));
            H = size(obj.Grid, 1);
            W = size(obj.Grid, 2);
            valid = (i >= 1) & (i <= H) & (j >= 1) & (j <= W);
            
            if isscalar(values)
                vals = values * ones(sum(valid), 1);
            else
                vals = values(valid);
            end
            
            idx = sub2ind([H, W], i(valid), j(valid));
            
            if strcmpi(mode, 'logodds')
                % log-odds 累加模式：先转log-odds，累加，再转回概率
                p_old = obj.Grid(idx);
                % 避免边界值导致log无穷
                p_old = max(1e-4, min(1-1e-4, p_old));
                lo_old = log(p_old ./ (1 - p_old));
                lo_new = lo_old + vals;
                p_new = 1 ./ (1 + exp(-lo_new));
                obj.Grid(idx) = p_new;
            else
                % 直接设置概率
                obj.Grid(idx) = vals;
            end
        end
        
        function mat = getOccupancyMatrix(obj)
            % 返回完整的占据概率矩阵
            mat = obj.Grid;
        end
        
        function r = unknownRatio(obj)
            % 返回未知单元格的比例
            r = mean(obj.Grid(:) == 0.5);
        end
    end
end

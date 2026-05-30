function visualize_realtime(mode, varargin)
%VISUALIZE_REALTIME 主动SLAM实时可视化界面
%   模式:
%     'init'   — 首次调用，创建图形窗口
%     'update' — 更新图形（传入所有状态变量）
%     'close'  — 关闭窗口
%
%   update 用法:
%     visualize_realtime('update', og_map, particles, x_true, best_pose, ...
%         trajectory, t, state, trigger_params, belief, frontiers, best_goal, criteria)
%
%   使用 persistent 句柄避免重复创建图形对象

    persistent h fig initialized
    
    if strcmpi(mode, 'init')
        %% ========== 首次创建 ==========
        if ~isempty(fig) && isvalid(fig)
            close(fig);
        end
        fig = figure('Name', 'Active SLAM Real-time Monitor', ...
                     'Position', [50 50 1400 900], ...
                     'Color', [0.15 0.15 0.15], ...
                     'CloseRequestFcn', @(src,~) delete(src));
        setappdata(0, 'active_slam_fig_handle', fig);
        
        % 主地图 (占据左上大块)
        h.ax_map = axes('Parent', fig, 'Position', [0.05 0.35 0.55 0.60]);
        hold on; axis equal; grid on;
        h.ax_map.Color = [0.12 0.12 0.12];
        h.ax_map.XColor = [0.7 0.7 0.7];
        h.ax_map.YColor = [0.7 0.7 0.7];
        h.ax_map.GridColor = [0.3 0.3 0.3];
        xlabel('X [m]', 'Color', [0.7 0.7 0.7]);
        ylabel('Y [m]', 'Color', [0.7 0.7 0.7]);
        title('OG Map & Robot Trajectory', 'Color', [1 1 1], 'FontSize', 12);
        
        % OG 地图图像
        h.img_og = imagesc([], [], []);
        colormap(h.ax_map, gray(256));
        h.img_og.AlphaData = 0.85;
        clim(h.ax_map, [0, 1]);
        
        % 轨迹
        h.traj_true = plot(NaN, NaN, 'g-', 'LineWidth', 1.5, 'DisplayName', 'True Traj');
        h.traj_est  = plot(NaN, NaN, 'c--', 'LineWidth', 1.0, 'DisplayName', 'Est. Traj');
        
        % 粒子云
        h.particles = scatter(NaN, NaN, 8, [0.8 0.4 0.4], 'filled', 'MarkerFaceAlpha', 0.3);
        
        % 当前位姿
        h.pose_true = quiver(0, 0, 0, 0, 0.5, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        h.pose_est  = quiver(0, 0, 0, 0, 0.5, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % 协方差椭圆
        h.ellipse_cov = plot(NaN, NaN, 'y-', 'LineWidth', 1.5);
        
        % Landmark
        h.landmarks = scatter(NaN, NaN, 20, 'r', 'filled', 'MarkerFaceAlpha', 0.6);
        
        % 前沿
        h.frontiers = scatter(NaN, NaN, 30, 'MarkerEdgeColor', [0 1 1], ...
                              'Marker', 's', 'LineWidth', 1.5);
        
        % 当前目标
        h.goal = plot(NaN, NaN, 'y*', 'MarkerSize', 15, 'LineWidth', 2);
        
        legend([h.traj_true, h.traj_est, h.particles, h.landmarks, h.frontiers, h.goal], ...
               {'True Traj', 'Est. Traj', 'Particles', 'Landmarks', 'Frontiers', 'Goal'}, ...
               'TextColor', [1 1 1], 'Color', [0.2 0.2 0.2]);
        
        % 信息面板 (右上)
        h.ax_info = axes('Parent', fig, 'Position', [0.62 0.55 0.36 0.40]);
        axis off;
        h.ax_info.Color = [0.12 0.12 0.12];
        h.txt_info = text(0.05, 0.95, '', 'Units', 'normalized', ...
                          'Color', [1 1 1], 'FontName', 'Consolas', ...
                          'FontSize', 10, 'VerticalAlignment', 'top', ...
                          'Interpreter', 'none');
        
        % 位姿误差 (左下)
        h.ax_err = axes('Parent', fig, 'Position', [0.05 0.05 0.40 0.25]);
        hold on; grid on;
        h.ax_err.Color = [0.12 0.12 0.12];
        h.ax_err.XColor = [0.7 0.7 0.7];
        h.ax_err.YColor = [0.7 0.7 0.7];
        h.ax_err.GridColor = [0.3 0.3 0.3];
        xlabel('Step', 'Color', [0.7 0.7 0.7]);
        ylabel('Error', 'Color', [0.7 0.7 0.7]);
        title('Pose Error (True vs Estimated)', 'Color', [1 1 1], 'FontSize', 11);
        h.err_x = plot(NaN, NaN, 'r-', 'LineWidth', 1);
        h.err_y = plot(NaN, NaN, 'g-', 'LineWidth', 1);
        h.err_theta = plot(NaN, NaN, 'b-', 'LineWidth', 1);
        legend({'X err', 'Y err', 'Theta err'}, 'TextColor', [1 1 1], 'Color', [0.2 0.2 0.2]);
        
        % 权重分布 (右下)
        h.ax_weight = axes('Parent', fig, 'Position', [0.52 0.05 0.22 0.25]);
        h.bar_weight = bar(NaN, 'FaceColor', [0.3 0.6 0.9]);
        h.ax_weight.Color = [0.12 0.12 0.12];
        h.ax_weight.XColor = [0.7 0.7 0.7];
        h.ax_weight.YColor = [0.7 0.7 0.7];
        xlabel('Particle', 'Color', [0.7 0.7 0.7]);
        ylabel('Weight', 'Color', [0.7 0.7 0.7]);
        title('Particle Weights (Top 30)', 'Color', [1 1 1], 'FontSize', 11);
        
        % 效用值 (右下右侧)
        h.ax_util = axes('Parent', fig, 'Position', [0.78 0.05 0.20 0.25]);
        h.bar_util = bar(NaN, 'FaceColor', [0.9 0.6 0.3]);
        h.ax_util.Color = [0.12 0.12 0.12];
        h.ax_util.XColor = [0.7 0.7 0.7];
        h.ax_util.YColor = [0.7 0.7 0.7];
        xlabel('Candidate', 'Color', [0.7 0.7 0.7]);
        ylabel('Utility', 'Color', [0.7 0.7 0.7]);
        title('Utility of Candidates', 'Color', [1 1 1], 'FontSize', 11);
        
        initialized = true;
        drawnow limitrate;
        
    elseif strcmpi(mode, 'update') && initialized
        %% ========== 更新数据 ==========
        og_map      = varargin{1};
        particles   = varargin{2};
        x_true      = varargin{3};
        best_pose   = varargin{4};
        trajectory  = varargin{5};
        t           = varargin{6};
        state       = varargin{7};
        trigger_params = varargin{8};
        belief      = varargin{9};
        frontiers   = varargin{10};
        best_goal   = varargin{11};
        criteria    = varargin{12};
        
        if ~isvalid(fig), return; end
        
        % --- OG 地图 ---
        occ = og_map.getOccupancyMatrix();
        % 将概率映射到显示: 0->深色(自由), 0.5->中灰(未知), 1->亮色(占据)
        occ_display = occ;
        set(h.img_og, 'CData', occ_display, ...
                      'XData', [og_map.Origin(1), og_map.Origin(1)+og_map.Width], ...
                      'YData', [og_map.Origin(2), og_map.Origin(2)+og_map.Height]);
        
        % 调整坐标轴范围
        xlim(h.ax_map, [og_map.Origin(1), og_map.Origin(1)+og_map.Width]);
        ylim(h.ax_map, [og_map.Origin(2), og_map.Origin(2)+og_map.Height]);
        
        % --- 轨迹 ---
        set(h.traj_true, 'XData', trajectory(1, 1:t), 'YData', trajectory(2, 1:t));
        % 估计轨迹：用当前 best_pose 作为最后一个点（简化，未保存历史）
        est_x = [trajectory(1, max(1,t-1)), best_pose(1)];
        est_y = [trajectory(2, max(1,t-1)), best_pose(2)];
        set(h.traj_est, 'XData', est_x, 'YData', est_y);
        
        % --- 粒子云 ---
        poses = reshape([particles.xv], 3, []);
        valid = ~any(isnan(poses) | isinf(poses), 1);
        set(h.particles, 'XData', poses(1, valid), 'YData', poses(2, valid));
        
        % --- 当前位姿 (箭头) ---
        len = 0.8;
        set(h.pose_true, 'XData', x_true(1), 'YData', x_true(2), ...
             'UData', len*cos(x_true(3)), 'VData', len*sin(x_true(3)));
        set(h.pose_est, 'XData', best_pose(1), 'YData', best_pose(2), ...
             'UData', len*cos(best_pose(3)), 'VData', len*sin(best_pose(3)));
        
        % --- 协方差椭圆 (仅 XY 平面) ---
        if ~isempty(belief.pose_cov) && ~any(isnan(belief.pose_cov(:)))
            Sigma_xy = belief.pose_cov(1:2, 1:2);
            [V, D] = eig(Sigma_xy);
            theta_ell = atan2(V(2,1), V(1,1));
            a = sqrt(max(D(1,1), 1e-6)) * 2;  % 2-sigma
            b = sqrt(max(D(2,2), 1e-6)) * 2;
            phi = linspace(0, 2*pi, 50);
            ell_x = a*cos(phi)*cos(theta_ell) - b*sin(phi)*sin(theta_ell) + best_pose(1);
            ell_y = a*cos(phi)*sin(theta_ell) + b*sin(phi)*cos(theta_ell) + best_pose(2);
            set(h.ellipse_cov, 'XData', ell_x, 'YData', ell_y, 'Visible', 'on');
        else
            set(h.ellipse_cov, 'Visible', 'off');
        end
        
        % --- Landmark (最佳粒子) ---
        [~, best_idx] = max([particles.w]);
        lm = particles(best_idx).xf;
        if ~isempty(lm)
            set(h.landmarks, 'XData', lm(1, :), 'YData', lm(2, :));
        else
            set(h.landmarks, 'XData', [], 'YData', []);
        end
        
        % --- 前沿 ---
        if ~isempty(frontiers)
            set(h.frontiers, 'XData', frontiers(:, 1), 'YData', frontiers(:, 2));
        else
            set(h.frontiers, 'XData', [], 'YData', []);
        end
        
        % --- 当前目标 ---
        if ~isempty(best_goal)
            set(h.goal, 'XData', best_goal(1), 'YData', best_goal(2));
        else
            set(h.goal, 'XData', [], 'YData', []);
        end
        
        % --- 信息面板文本 ---
        info_str = sprintf(['Step: %d\n' ...
                            'State: %s\n' ...
                            'Trigger: %s\n' ...
                            '--------------------\n' ...
                            'Pose Entropy: %.3f\n' ...
                            'D-opt: %.4f\n' ...
                            'T-opt: %.4f\n' ...
                            'Neff: %.1f / %d\n' ...
                            'Unknown: %.2f%%\n' ...
                            '--------------------\n' ...
                            'True  Pose: [%.2f, %.2f, %.2f°]\n' ...
                            'Est.  Pose: [%.2f, %.2f, %.2f°]\n' ...
                            'Pos Error: %.3f m\n' ...
                            '--------------------\n' ...
                            'Weights: [%.2f, %.2f, %.2f, %.2f]\n' ...
                            'U_IT: %.2f | U_TOED: %.2f\n' ...
                            'U_Graph: %.2f | U_Geo: %.2f\n' ...
                            'Best Utility: %.3f'], ...
                            t, state.mode, trigger_params.reason, ...
                            belief.current_entropy, ...
                            criteria.D_opt_global, criteria.TOED(1), ...
                            belief.Neff, length(particles), ...
                            og_map.unknownRatio()*100, ...
                            x_true(1), x_true(2), rad2deg(x_true(3)), ...
                            best_pose(1), best_pose(2), rad2deg(best_pose(3)), ...
                            norm(x_true(1:2)-best_pose(1:2)), ...
                            criteria.weights(1), criteria.weights(2), criteria.weights(3), criteria.weights(4), ...
                            criteria.IT(1), criteria.TOED(1), criteria.Graph(1), criteria.Geo(1), ...
                            max(criteria.IT + criteria.TOED + criteria.Graph + criteria.Geo));
        set(h.txt_info, 'String', info_str);
        
        % --- 位姿误差历史（简化：只显示当前步的误差）---
        if t > 1
            set(h.err_x, 'XData', [get(h.err_x, 'XData'), t], ...
                'YData', [get(h.err_x, 'YData'), abs(x_true(1) - best_pose(1))]);
            set(h.err_y, 'XData', [get(h.err_y, 'XData'), t], ...
                'YData', [get(h.err_y, 'YData'), abs(x_true(2) - best_pose(2))]);
            theta_err = atan2(sin(x_true(3) - best_pose(3)), cos(x_true(3) - best_pose(3)));
            set(h.err_theta, 'XData', [get(h.err_theta, 'XData'), t], ...
                'YData', [get(h.err_theta, 'YData'), abs(theta_err)]);
        end
        
        % --- 权重分布 (Top 30) ---
        w = [particles.w];
        [w_sorted, idx] = sort(w, 'descend');
        n_show = min(30, length(w));
        set(h.bar_weight, 'XData', 1:n_show, 'YData', w_sorted(1:n_show));
        w_max = max(w_sorted(1:n_show));
        if ~isempty(w_max) && ~isnan(w_max) && w_max > 0
            ylim(h.ax_weight, [0, w_max * 1.2]);
        end
        
        % --- 效用值 ---
        if isfield(criteria, 'IT') && ~isempty(criteria.IT)
            util_vals = [criteria.IT; criteria.TOED; criteria.Graph; criteria.Geo]';
            util_total = sum(util_vals .* criteria.weights, 2);
            set(h.bar_util, 'XData', 1:length(util_total), 'YData', util_total);
            
            % 安全地设置ylim（避免空/NaN/单值导致的递减区间错误）
            u_min = min(util_total);
            u_max = max(util_total);
            if ~isempty(u_min) && ~isnan(u_min) && ~isempty(u_max) && ~isnan(u_max)
                if u_min == u_max
                    u_min = u_min - 1;
                    u_max = u_max + 1;
                end
                ylim(h.ax_util, [u_min, u_max]);
            end
        end
        
        drawnow limitrate;
        
    elseif strcmpi(mode, 'close')
        if ~isempty(fig) && isvalid(fig)
            delete(fig);
        end
        if isappdata(0, 'active_slam_fig_handle')
            rmappdata(0, 'active_slam_fig_handle');
        end
        initialized = false;
        h = [];
        fig = [];
    end
end

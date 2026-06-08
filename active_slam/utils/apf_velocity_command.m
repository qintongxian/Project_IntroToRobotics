function [v_cmd, omega_cmd] = apf_velocity_command(pose, goal, ranges, angles, robot_radius)
%APF_VELOCITY_COMMAND 人工势场法速度指令生成
%   基于当前位姿、目标点与激光扫描，计算引力+斥力合力，映射为 v/omega。
%
%   输入:
%     pose         — [x; y; theta] 当前位姿
%     goal         — [x; y] 目标点（世界坐标）
%     ranges       — N×1 激光测距值 [m]
%     angles       — N×1 激光束角度（相对机器人朝向） [rad]
%     robot_radius — 机器人碰撞半径 [m] (默认 0.3)
%
%   输出:
%     v_cmd    — 线速度 [m/s]
%     omega_cmd— 角速度 [rad/s]

    if nargin < 5
        robot_radius = 0.3;
    end

    %% ========== 默认参数 ==========
    k_att      = 2.0;   % 引力增益
    F_att_max  = 3.0;   % 引力上限
    eta        = 1.5;   % 斥力增益
    rho0       = 2.0;   % 斥力作用半径 [m]
    F_rep_max  = 5.0;   % 总斥力上限
    k_omega    = 2.5;   % 角速度增益
    v_max      = 1.0;   % 最大线速度
    omega_max  = 0.5;   % 最大角速度
    safety_margin = 0.6;% 前方减速阈值余量
    front_fov  = pi/3;  % 前方扇区 [rad]

    %% ========== 引力 ==========
    % 确保 goal 为列向量
    goal = goal(:);
    pos = pose(1:2);
    to_goal = goal - pos;
    dist_to_goal = norm(to_goal(1:2));

    if dist_to_goal < 0.5
        % 到达目标附近，仅保留斥力（停车由主循环处理）
        F_att = [0; 0];
    else
        F_att = k_att * to_goal;
        % 限幅
        f_att_norm = norm(F_att);
        if f_att_norm > F_att_max
            F_att = F_att * (F_att_max / f_att_norm);
        end
    end

    %% ========== 斥力（由激光端点构造）==========
    % 将激光扫描转为世界坐标障碍物点
    theta_beams = pose(3) + angles;
    obs_x = pose(1) + ranges .* cos(theta_beams);
    obs_y = pose(2) + ranges .* sin(theta_beams);

    % 只考虑在影响半径内的有效障碍物（排除无回波的最大量程）
    valid = (ranges < rho0) & isfinite(ranges);
    if any(valid)
        obs = [obs_x(valid), obs_y(valid)];
        % 计算障碍物到机器人的距离向量
        diff = pos' - obs;                 % M×2
        rho = sqrt(sum(diff.^2, 2));       % M×1
        % 避免零距离爆炸
        rho = max(rho, robot_radius);
        % 单位方向向量（由障碍物指向机器人）
        n = diff ./ rho;
        % 斥力大小
        mag = eta * (1./rho - 1/rho0) ./ (rho.^2);
        % 仅保留 rho < rho0 的项（理论上 valid 已保证）
        mag(rho >= rho0) = 0;
        % 总斥力
        F_rep_vec = sum(mag .* n, 1);      % 1×2
        F_rep = F_rep_vec(:);
        % 限幅
        f_rep_norm = norm(F_rep);
        if f_rep_norm > F_rep_max
            F_rep = F_rep * (F_rep_max / f_rep_norm);
        end
    else
        F_rep = [0; 0];
    end

    %% ========== 合力 ==========
    F = F_att + F_rep;
    % 若合力极小，保持原航向
    f_norm = norm(F);
    if f_norm < 1e-6
        theta_des = pose(3);
    else
        theta_des = atan2(F(2), F(1));
    end

    %% ========== 映射为速度指令 ==========
    heading_error = atan2(sin(theta_des - pose(3)), cos(theta_des - pose(3)));
    omega_cmd = max(-omega_max, min(omega_max, k_omega * heading_error));

    % 线速度：航向差越大越慢
    v_cmd = v_max * max(0, cos(heading_error));

    % 前方最近障碍物额外减速
    front_mask = abs(angles) < front_fov;
    if any(front_mask)
        front_ranges = ranges(front_mask);
        front_min = min(front_ranges(isfinite(front_ranges)));
        if front_min < robot_radius + safety_margin
            factor = max(0.1, (front_min - robot_radius) / safety_margin);
            v_cmd = v_cmd * factor;
        end
    end

    % 接近目标时停车
    if dist_to_goal < 0.5
        v_cmd = 0;
        omega_cmd = 0;
    end
end

function particle = predict_diffdrive(particle, v, w, Q, dt, add_noise)
%PREDICT_DIFFDRIVE 差速驱动模型的FastSLAM粒子预测
%   输入:
%     particle   — FastSLAM粒子结构 (.xv, .Pv)
%     v, w       — 线速度 [m/s], 角速度 [rad/s]
%     Q          — 2x2 控制噪声协方差 [v_noise^2, w_noise^2]
%     dt         — 时间步长 [s]
%     add_noise  — 若为1，从Q采样噪声加入控制量
%   输出:
%     particle   — 更新后的粒子

    if nargin < 6
        add_noise = 0;
    end
    
    % 采样控制噪声
    if add_noise == 1
        vw = multivariate_gauss([v; w], Q, 1);
        v = vw(1);
        w = vw(2);
    end
    
    xv = particle.xv;
    Pv = particle.Pv;
    theta = xv(3);
    
    % 运动模型（ velocity + angular velocity ）
    particle.xv = [xv(1) + v*dt*cos(theta + w*dt/2);
                   xv(2) + v*dt*sin(theta + w*dt/2);
                   pi_to_pi(theta + w*dt)];
    
    % 状态雅可比矩阵
    Gv = [1, 0, -v*dt*sin(theta + w*dt/2);
          0, 1,  v*dt*cos(theta + w*dt/2);
          0, 0,  1];
    
    % 控制雅可比矩阵
    Gu = [dt*cos(theta + w*dt/2), -0.5*v*dt^2*sin(theta + w*dt/2);
          dt*sin(theta + w*dt/2),  0.5*v*dt^2*cos(theta + w*dt/2);
          0,                       dt];
    
    % 协方差传播（加正则化确保严格正定，避免chol失败）
    particle.Pv = Gv*Pv*Gv' + Gu*Q*Gu' + 1e-6*eye(3);
end

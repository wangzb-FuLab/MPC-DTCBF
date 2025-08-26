function [u, feasible] = mpc_dtcbf_controller(pos, vel, angles, omega, goal_pos, obs_pos, obs_radius, params, target_history, current_time_step, current_goal_pos)
    % 优化选项
    options = optimoptions('fmincon',...
        'Algorithm','sqp',...
        'Display','off',...
        'MaxIterations',1000);
    
    % 初始猜测
    u0 = [params.m*params.g; 0; 0; 0];  % 初始化为悬停状态 [Fz, ux, uy, uz]
    
    % 约束边界
    lb = [0; -2.5; -2.5; -2.5];    % 最小推力约束 [Fz_min, ux_min, uy_min, uz_min]
    ub = [params.T_max; 2.5; 2.5; 2.5];  % 最大推力约束 [Fz_max, ux_max, uy_max, uz_max]
    
    % 求解优化问题
    try
        u = fmincon(@(u)cost_function(u, pos, vel, angles, omega, goal_pos, params.N, params.Q, params.R, params.dt, params.m, params.g, params.I),...
                   u0,...
                   [], [], [], [], lb, ub,...
                   @(u)constraints(u, pos, vel, angles, omega, obs_pos, obs_radius,...
                                  params.N, params.safety_margin, params.dt, params.m, params.g, params.max_angle, params,...
                                  target_history, current_time_step, current_goal_pos),...
                   options);
        feasible = true;
    catch
        u = u0;  % 失败时保持悬停
        feasible = false;
    end
end

function cost = cost_function(u, pos, vel, angles, omega, goal, N, Q, R, dt, m, g, I)
    cost = 0;
    x = [pos, vel, angles, omega];
    for k = 1:N
        x = dynamics_model(x, u, dt, m, g, I);
        cost = cost + (x(1:3)-goal)*Q*(x(1:3)-goal)';
    end
end

function [c, ceq] = constraints(u, pos, vel, angles, omega, obs_pos, obs_radius,...
                                N, safety_margin, dt, m, g, max_angle, params,...
                                target_history, current_time_step, current_goal_pos)
    c = [];
    ceq = [];
    x = [pos, vel, angles, omega];
    x_next = dynamics_model(x, u, dt, m, g, params.I);
    
    % 计算目标位置变化的最大值（用于G函数）
    max_position_change = calculate_max_position_change(target_history, current_time_step, params.N);
    
    % 多步预测约束
    for k = 1:N
        x_next = dynamics_model(x_next, u, dt, m, g, params.I);
        vel_all = sqrt(x_next(4)^2 + x_next(5)^2 + x_next(6)^2);

        % 最大速度约束
        c = [c; vel_all-5];
        
        % 推力约束
        Fz = u(1);
        torque_norm = norm(u(2:4));
        max_torque = params.T_max * tan(max_angle);
        c = [c; torque_norm - max_torque];

        % CBF约束
        for i = 1:size(obs_pos,1)
            % 计算当前时刻的安全函数h
            h_current = norm(x(1:3) - obs_pos(i,:)) - obs_radius(i) - safety_margin - params.UAV_radius;
            
            % 计算下一时刻的安全函数h
            h_next = norm(x_next(1:3) - obs_pos(i,:)) - obs_radius(i) - safety_margin - params.UAV_radius;
            
            % 计算Δh = h_next - h_current
            delta_h = h_next - h_current;
            
            % 计算G函数
            predicted_goal_pos = current_goal_pos + k * params.dt * params.vel_goal;
            
            % 计算目标位置变化
            position_change = norm(predicted_goal_pos - current_goal_pos);
            
            % 计算G函数
            if max_position_change > 0
                G_value = g_function(position_change / max_position_change);
            else
                G_value = 0;
            end
            
            % 计算自适应γ值
            l_kj = norm(x(1:3) - obs_pos(i,:)) - obs_radius(i) - params.UAV_radius;
            gamma_val = calculate_gamma(l_kj, params);
            
            % CBF约束
            cbf_constraint = delta_h + G_value - gamma_val * h_current;
            c = [c; -cbf_constraint]; % 转换为fmincon的≤0形式
        end
    end
end

function x_next = dynamics_model(x, u, dt, m, g, I)
    % 四旋翼动力学模型
    pos = x(1:3);
    vel = x(4:6);
    angles = x(7:9);
    omega = x(10:12);
    
    % 提取控制输入
    Fz = u(1);
    u_theta = u(2:4);
    
    % 计算旋转矩阵
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    R = rotation_matrix(phi, theta, psi);
    
    % 计算线加速度
    thrust_vector = R * [0; 0; Fz];
    gravity_vector = [0; 0; -m * g];
    acceleration = (thrust_vector + gravity_vector) / m;
    
    % 计算角加速度
    angular_acceleration = I \ u_theta;
    
    % 使用欧拉法更新状态
    pos_next = pos + vel * dt;
    vel_next = vel + acceleration' * dt;
    angles_next = angles + omega' * dt;
    omega_next = omega + angular_acceleration' * dt;
    
    x_next = [pos_next, vel_next, angles_next, omega_next];
end

function R = rotation_matrix(phi, theta, psi)
    % 计算从机体坐标系到世界坐标系的旋转矩阵
    R_x = [1, 0, 0;
           0, cos(phi), -sin(phi);
           0, sin(phi), cos(phi)];
    
    R_y = [cos(theta), 0, sin(theta);
           0, 1, 0;
           -sin(theta), 0, cos(theta)];
    
    R_z = [cos(psi), -sin(psi), 0;
           sin(psi), cos(psi), 0;
           0, 0, 1];
    
    R = R_z * R_y * R_x;
end

function max_change = calculate_max_position_change(target_history, current_time_step, N)
    % 计算目标位置变化的最大值
    max_change = 0;
    
    % 获取历史位置数据
    history_length = min(current_time_step, size(target_history, 1));
    
    if history_length > 1
        % 计算所有时间步的位置变化
        for i = 1:history_length-1
            change = norm(target_history(i+1, :) - target_history(i, :));
            if change > max_change
                max_change = change;
            end
        end
    end
    
    % 考虑预测范围内的最大可能变化
    max_change = max_change * 2;
end

function G = g_function(ratio)
    % G函数的实现
    G = 1 / (1 + exp(-10 * (ratio - 0.5)));
end

function gamma_val = calculate_gamma(l_kj, params)
    % 计算自适应γ值
    A = params.gamma_A;
    c_param = params.gamma_c;
    l_E = params.gamma_l_E;
    
    integral_part = (sqrt(pi)/2) * erf(c_param * (l_kj - l_E));
    gamma_val = A * (1 + (2/sqrt(pi)) * integral_part);
    gamma_val = max(0.01, min(0.5, gamma_val));
end
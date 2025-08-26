function [pos_new, vel_new, angles_new, omega_new] = update_dynamics(pos, vel, angles, omega, u, params)
    % 提取控制输入
    Fz = u(1);
    u_theta = u(2:4); % 扭矩输入 [ux, uy, uz]
    
    % 提取姿态角
    phi = angles(1);   % 滚转角
    theta = angles(2); % 俯仰角
    psi = angles(3);   % 偏航角
    
    % 计算旋转矩阵
    R = rotation_matrix(phi, theta, psi);
    
    % 计算线加速度
    thrust_vector = R * [0; 0; Fz];
    gravity_vector = [0; 0; -params.m * params.g];
    acceleration = (thrust_vector + gravity_vector) / params.m;
    
    % 计算角加速度
    angular_acceleration = params.I_inv * u_theta;
    
    % 使用欧拉法更新状态
    % 更新位置
    pos_new = pos + vel * params.dt;
    
    % 更新速度
    vel_new = vel + acceleration' * params.dt;
    
    % 更新姿态角
    angles_new = angles + omega' * params.dt;
    
    % 更新角速度
    omega_new = omega + angular_acceleration' * params.dt;
end

function R = rotation_matrix(phi, theta, psi)
    % 计算从机体坐标系到世界坐标系的旋转矩阵.使用ZYX欧拉角顺序 (偏航-俯仰-滚转)

    R_x = [1, 0, 0;
           0, cos(phi), -sin(phi);
           0, sin(phi), cos(phi)];
    
    R_y = [cos(theta), 0, sin(theta);
           0, 1, 0;
           -sin(theta), 0, cos(theta)];
    
    R_z = [cos(psi), -sin(psi), 0;
           sin(psi), cos(psi), 0;
           0, 0, 1];
    
    % 组合旋转矩阵
    R = R_z * R_y * R_x;
end
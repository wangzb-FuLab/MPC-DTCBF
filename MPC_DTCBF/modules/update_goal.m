function dynamic_goal = update_goal(dynamic_goal, goal_points, obs_pos_goal, obs_radius_goal, vel_goal, params)
    % 随机游走目标模型，带障碍物斥力与边界反弹
    
    % 参数
    v_max = 2;              % 最大线速度
    v_min = 0.2;            % 最小线速度
    sigma_turn = 15*pi/180; % 随机转向标准差
    k_rep = 8;              % 障碍物斥力系数
    d_safe = 2.5;           % 斥力生效距离
    
    % 方向惯性
    persistent h_dir        % 保存上一帧的单位方向向量
    if isempty(h_dir)
        rnd = randn(1,3); 
        rnd(3) = 0;        % 初始朝向只在水平面
        h_dir = rnd / norm(rnd);
    end

    % 给方向加小扰动
    [az, el, ~] = cart2sph(h_dir(1), h_dir(2), h_dir(3));
    az = az + sigma_turn*randn;    % 水平转向
    el = el + sigma_turn*randn/2;  % 垂直扰动小一点
    [h_dir(1), h_dir(2), h_dir(3)] = sph2cart(az, el, 1);  % 单位化

    % 速度大小
    speed = v_min + (v_max - v_min)*rand; 
    vel_goal = speed * h_dir;              % 纯随机速度

    % 障碍物斥力
    F_rep = zeros(1,3);
    for i = 1:size(obs_pos_goal,1)
        d_obs = norm(dynamic_goal.pos - obs_pos_goal(i,:)) - obs_radius_goal(i) - 0.2;
        if d_obs < d_safe
            dir_rep = (dynamic_goal.pos - obs_pos_goal(i,:)) / ...
                       norm(dynamic_goal.pos - obs_pos_goal(i,:));
            F_rep = F_rep + k_rep * (1/d_obs - 1/d_safe) * (1/d_obs^2) * dir_rep;
        end
    end
    vel_goal = vel_goal + F_rep * params.dt;  % 斥力矢量累加到速度

    % 速度限幅
    v_norm = norm(vel_goal);
    if v_norm > v_max
        vel_goal = vel_goal / v_norm * v_max;
    elseif v_norm < v_min
        vel_goal = vel_goal / v_norm * v_min;
    end

    % 位置更新
    dynamic_goal.pos = dynamic_goal.pos + vel_goal * params.dt;
    dynamic_goal.vel = vel_goal;

    % 边界反弹
    for dim = 1:3
        if dynamic_goal.pos(dim) < params.map_limits(dim,1)
            dynamic_goal.pos(dim) = params.map_limits(dim,1) + 0.1;
            dynamic_goal.vel(dim) = abs(dynamic_goal.vel(dim));
            h_dir(dim) = abs(h_dir(dim));
        elseif dynamic_goal.pos(dim) > params.map_limits(dim,2)
            dynamic_goal.pos(dim) = params.map_limits(dim,2) - 0.1;
            dynamic_goal.vel(dim) = -abs(dynamic_goal.vel(dim));
            h_dir(dim) = -abs(h_dir(dim));
        end
    end
end
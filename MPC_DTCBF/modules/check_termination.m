function stop = check_termination(dynamic_goal, uav_positions, uav_velocities, UAV_subTargets, k, params)
    stop = false;
    d_success = 2;
    
    % 捕获检测
    distances = zeros(1, params.M);
    for i = 1:params.M
        distances(i) = norm(uav_positions(i,:) - dynamic_goal.pos);
    end
    
    if all(distances < d_success)
        fprintf('捕获成功! 耗时 %.1f秒\n', k*params.dt);
        stop = true;
        return
    end
    
    % 最终目标逃逸检测
    if norm(dynamic_goal.pos - params.goal_points(end,:)) < 1.0
        fprintf('目标逃逸!\n');
        stop = true;
    end
end
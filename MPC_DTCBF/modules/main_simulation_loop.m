function main_simulation_loop(params)
    % 初始化无人机和目标状态
    uav_positions = zeros(params.M, 3);
    uav_velocities = zeros(params.M, 3);
    uav_angles = zeros(params.M, 3);    % 新增：姿态角 [roll, pitch, yaw]
    uav_omega = zeros(params.M, 3);     % 新增：角速度 [ωx, ωy, ωz]
    
    % 设置无人机初始位置和姿态
    for i = 1:params.M
        uav_positions(i,:) = [10+rand*5, 10+rand*5, 10+rand*5];
        uav_angles(i,:) = [0, 0, 0];    % 初始姿态角为零
        uav_omega(i,:) = [0, 0, 0];     % 初始角速度为零
    end
    
    % 初始化目标状态
    dynamic_goal.pos = params.goal_points(1,:);
    dynamic_goal.vel = [0, 0, 0];
    
    % 初始化数据记录
    data_matrix = [];
    column_names = {'Time'};
    for i = 1:params.M
        column_names = [column_names, ...
            sprintf('UAV%d_X', i), sprintf('UAV%d_Y', i), sprintf('UAV%d_Z', i), ...
            sprintf('UAV%d_Xvel', i), sprintf('UAV%d_Yvel', i), sprintf('UAV%d_Zvel', i)];
    end
    column_names = [column_names, 'Target_X', 'Target_Y', 'Target_Z'];
    target_history = []; % 添加目标历史记录
    % 主仿真循环
    for k = 1:length(params.t)
        t = params.t(k);

        % 更新目标位置
        dynamic_goal = update_goal(dynamic_goal, params.goal_points, params.obs_pos, params.obs_radius, params.vel_goal, params);

        % 记录目标历史
        target_history = [target_history; dynamic_goal.pos];
        
        % 计算子目标点（基于斐波那契球）
        sub_targets = zeros(params.M, 3);
        for i = 1:params.M
            sub_targets(i,:) = dynamic_goal.pos + params.d * [params.X(i), params.Y(i), params.Z(i)];
        end
        
        % 使用拍卖算法分配目标点
        [assignments, ~] = auction_algorithm(uav_positions, uav_velocities, sub_targets);
        
        % 为每个无人机计算控制输入
        UAV_subTargets = sub_targets(assignments,:);
        u_inputs = zeros(params.M, 4);  % 改为4维输入 [Fz, ux, uy, uz]
        feasible_flags = false(params.M, 1);

        for i = 1:params.M
            [u_inputs(i,:), feasible_flags(i)] = mpc_dtcbf_controller(...
                uav_positions(i,:), uav_velocities(i,:), uav_angles(i,:), uav_omega(i,:), ...
                UAV_subTargets(i,:), params.obs_pos, params.obs_radius, params,...
                target_history, k, dynamic_goal.pos);
        end

        % 更新无人机状态
        for i = 1:params.M
            [uav_positions(i,:), uav_velocities(i,:), uav_angles(i,:), uav_omega(i,:)] = update_dynamics(...
                uav_positions(i,:), uav_velocities(i,:), uav_angles(i,:), uav_omega(i,:), ...
                u_inputs(i,:), params);
        end
        
        % 记录数据
        data_row = [t];
        for i = 1:params.M
            data_row = [data_row, uav_positions(i,:), uav_velocities(i,:)];
        end
        data_row = [data_row, dynamic_goal.pos];
        data_matrix = [data_matrix; data_row];
        
        % 更新可视化
        update_visualization(...
            params.traj1, params.traj2, params.traj3, params.traj4, ...
            params.traj5, params.traj6, params.traj7, params.traj8, ...
            params.uav1, params.uav2, params.uav3, params.uav4, ...
            params.uav5, params.uav6, params.uav7, params.uav8, ...
            params.goal_plot, params.goal_traj, ...
            uav_positions(1,:), uav_positions(2,:), uav_positions(3,:), ...
            uav_positions(4,:), uav_positions(5,:), uav_positions(6,:), ...
            uav_positions(7,:), uav_positions(8,:), dynamic_goal.pos);
        
        % 检查终止条件
        if check_termination(dynamic_goal, uav_positions, uav_velocities, UAV_subTargets, k, params)
            break;
        end
    end
    
    % 保存数据到params结构体，供后续分析使用
    params.data_matrix = data_matrix;
    params.column_names = column_names;
end
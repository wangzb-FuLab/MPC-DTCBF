function [assignments, total_cost] = auction_algorithm(uav_positions, uav_velocities, sub_targets)
    num_uavs = size(uav_positions, 1);
    num_targets = size(sub_targets, 1);
    
    % 初始化价格和分配
    prices = zeros(num_targets, 1);
    assignments = zeros(num_uavs, 1);
    epsilon = 0.1;  % 价格增量
    
    % 迭代直到所有无人机都分配到目标点
    while any(assignments == 0)
        for uav_idx = 1:num_uavs
            if assignments(uav_idx) ~= 0
                continue;  % 已分配的无人机跳过
            end
            
            % 计算每个目标点的净成本
            net_costs = zeros(num_targets, 1);
            for target_idx = 1:num_targets
                cost = calculate_cost(uav_positions(uav_idx,:), sub_targets(target_idx,:), uav_velocities(uav_idx,:));
                net_costs(target_idx) = cost - prices(target_idx);
            end
            
            % 找到最小净成本的目标点
            [min_cost, best_target] = min(net_costs);
            
            % 更新价格
            if min_cost > 0
                prices(best_target) = prices(best_target) - min_cost + epsilon;
                assignments(uav_idx) = best_target;
            end
        end
    end
    
    % 计算总成本
    total_cost = 0;
    for uav_idx = 1:num_uavs
        target_idx = assignments(uav_idx);
        cost = calculate_cost(uav_positions(uav_idx,:), sub_targets(target_idx,:), uav_velocities(uav_idx,:));
        total_cost = total_cost + cost;
    end
end

function cost = calculate_cost(uav_pos, sub_target, current_vel)
    % 距离成本：当前位置到子目标点的距离
    distance_cost = norm(uav_pos - sub_target);
    
    % 方向成本：当前速度方向与目标方向的偏差
    target_direction = sub_target - uav_pos;
    if norm(current_vel) > 0
        velocity_cost = 1 - dot(current_vel/norm(current_vel), target_direction/norm(target_direction));
    else
        velocity_cost = 0;
    end
    
    % 总成本 = 距离成本 + 速度成本权重
    cost = 0.8*distance_cost + 0.2 * velocity_cost;
end
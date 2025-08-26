function obs_pos = generate_obstacles(num_obs, map_limits, obs_radius)
    obs_pos = zeros(num_obs,3);
    for i = 1:num_obs
        r = obs_radius(i);
        valid_range = [
            map_limits(1,1)+r, map_limits(1,2)-r;
            map_limits(2,1)+r, map_limits(2,2)-r;
            map_limits(3,1)+r, map_limits(3,2)-r;
        ];    
        obs_pos(i,:) = [
            valid_range(1,1) + diff(valid_range(1,:))*rand(),
            valid_range(2,1) + diff(valid_range(2,:))*rand(),
            valid_range(3,1) + diff(valid_range(3,:))*rand()
        ];
    end
end
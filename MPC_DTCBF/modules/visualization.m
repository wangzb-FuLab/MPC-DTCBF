function params = visualization(params)
    % 可视化初始化
    fig = figure('Color','w','Position',[1100 400 1000 800]);
    hold on; grid on; axis equal;
    xlim(params.map_limits(1,:)); 
    ylim(params.map_limits(2,:)); 
    zlim(params.map_limits(3,:));
    view(45,30);
    title('MPC-CBF无人机动态追踪系统');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % 绘制障碍物
    for i = 1:params.num_obs
        [xs,ys,zs] = sphere(10);
        surf(params.obs_radius(i)*xs + params.obs_pos(i,1),...
             params.obs_radius(i)*ys + params.obs_pos(i,2),...
             params.obs_radius(i)*zs + params.obs_pos(i,3),...
             'FaceColor',[1 0.6 0.6],'EdgeColor','none','FaceAlpha',0.6);
    end
    
    % 初始化动态元素
    params.traj1 = animatedline('Color',[0 0.5 1],'LineWidth',1.5);
    params.traj2 = animatedline('Color',[0 0.5 1],'LineWidth',1.5);
    params.traj3 = animatedline('Color',[0 0.5 1],'LineWidth',1.5);
    params.traj4 = animatedline('Color',[0 0.5 1],'LineWidth',1.5);
    params.traj5 = animatedline('Color',[0 0.5 1],'LineWidth',1.5);
    params.traj6 = animatedline('Color',[0 0.5 1],'LineWidth',1.5);
    params.traj7 = animatedline('Color',[0 0.5 1],'LineWidth',1.5);
    params.traj8 = animatedline('Color',[0 0.5 1],'LineWidth',1.5);
    
    params.uav1 = scatter3(nan,nan,nan,70,'filled','MarkerFaceColor',[0 0.3 0.9],'MarkerEdgeColor','k');
    params.uav2 = scatter3(nan,nan,nan,70,'filled','MarkerFaceColor',[0 0.3 0.9],'MarkerEdgeColor','k');
    params.uav3 = scatter3(nan,nan,nan,70,'filled','MarkerFaceColor',[0 0.3 0.9],'MarkerEdgeColor','k');
    params.uav4 = scatter3(nan,nan,nan,70,'filled','MarkerFaceColor',[0 0.3 0.9],'MarkerEdgeColor','k');
    params.uav5 = scatter3(nan,nan,nan,70,'filled','MarkerFaceColor',[0 0.3 0.9],'MarkerEdgeColor','k');
    params.uav6 = scatter3(nan,nan,nan,70,'filled','MarkerFaceColor',[0 0.3 0.9],'MarkerEdgeColor','k');
    params.uav7 = scatter3(nan,nan,nan,70,'filled','MarkerFaceColor',[0 0.3 0.9],'MarkerEdgeColor','k');
    params.uav8 = scatter3(nan,nan,nan,70,'filled','MarkerFaceColor',[0 0.3 0.9],'MarkerEdgeColor','k');
    
    params.goal_plot = scatter3(nan,nan,nan,70,'g','filled','MarkerEdgeColor','k');
    params.goal_traj = animatedline('Color',[0 0.7 0],'LineWidth',2,'LineStyle',':');
    
    params.subPlots = gobjects(params.M,1);
    for ii = 1:params.M
        params.subPlots(ii) = scatter3(nan,nan,nan,70,'filled', ...
            'MarkerFaceColor',[0.6 1 0.6],'MarkerEdgeColor','k');
    end
    
    % 保存图形句柄
    params.fig = fig;
end
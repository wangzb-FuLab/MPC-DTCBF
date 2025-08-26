function data_export_and_analysis(params, data_matrix, column_names)
    % 将矩阵转换为表格并添加列名
    data_table = array2table(data_matrix, 'VariableNames', column_names);

    % 写入Excel文件
    filename = sprintf('UAV_Positions_%s.xlsx', datestr(now,'yyyymmdd_HHMMSS'));
    writetable(data_table, filename);

    % 提示存储成功
    fprintf('无人机位置数据已保存至：%s\n', filename);

    % 轨迹可视化
    figure('Name','无人机三维运动轨迹','Color','w','Position',[100 100 1200 800]);
    xlim(params.map_limits(1,:)); 
    ylim(params.map_limits(2,:)); 
    zlim(params.map_limits(3,:));
    hold on; grid on; axis equal;

    % 绘制所有无人机轨迹
    colors = lines(params.M+1); % 生成8种区分色
    for UAV_id = 1:params.M+1
        x = data_table.(sprintf('UAV%d_X',UAV_id));
        y = data_table.(sprintf('UAV%d_Y',UAV_id));
        z = data_table.(sprintf('UAV%d_Z',UAV_id));
        if UAV_id == 1
            plot3(x, y, z, 'Color', colors(UAV_id,:), 'LineWidth', 2.5, 'LineStyle', '--');
        else
            plot3(x, y, z, 'Color', colors(UAV_id,:), 'LineWidth', 1.5);
        end
    end

    % 可视化装饰
    xlabel('X坐标(m)'); ylabel('Y坐标(m)'); zlabel('Z坐标(m)');
    title('多无人机协同围捕三维轨迹');
    legend('Target','UAV1','UAV2','UAV3','UAV4','UAV5','UAV6','UAV7','UAV8');
    view(45,30); % 设置三维视角

    % 分维度时间序列分析
    figure('Name','坐标时间序列','Color','w','Position',[100 100 1400 600]);

    % X坐标变化
    subplot(3,1,1); hold on; grid on;
    for UAV_id = 1:params.M+1
        if UAV_id == 1
            plot(data_table.Time, data_table.(sprintf('UAV%d_X',UAV_id)),...
             'Color', colors(UAV_id,:),'LineWidth',2,'LineStyle','--');
        else
            plot(data_table.Time, data_table.(sprintf('UAV%d_X',UAV_id)),...
             'Color', colors(UAV_id,:),'LineWidth',1.5);
        end
    end
    title('X坐标时间序列'); xlabel('时间(s)'); ylabel('位置(m)');

    % Y坐标变化
    subplot(3,1,2); hold on; grid on;
    for UAV_id = 1:params.M+1
        if UAV_id == 1
            plot(data_table.Time, data_table.(sprintf('UAV%d_Y',UAV_id)),...
             'Color', colors(UAV_id,:),'LineWidth',2,'LineStyle','--');
        else
            plot(data_table.Time, data_table.(sprintf('UAV%d_Y',UAV_id)),...
             'Color', colors(UAV_id,:),'LineWidth',1.5);
        end
    end
    title('Y坐标时间序列'); xlabel('时间(s)'); ylabel('位置(m)');

    % Z坐标变化
    subplot(3,1,3); hold on; grid on;
    for UAV_id = 1:params.M+1
        if UAV_id == 1
            plot(data_table.Time, data_table.(sprintf('UAV%d_Z',UAV_id)),...
             'Color', colors(UAV_id,:),'LineWidth',2,'LineStyle','--');
        else
            plot(data_table.Time, data_table.(sprintf('UAV%d_Z',UAV_id)),...
             'Color', colors(UAV_id,:),'LineWidth',1.5);
        end
    end
    title('Z坐标时间序列'); xlabel('时间(s)'); ylabel('位置(m)');
    legend('Target','UAV1','UAV2','UAV3','UAV4','UAV5','UAV6','UAV7','UAV8');
               
    % 速度时间序列分析
    figure('Name','速度时间序列','Color','w','Position',[100 100 800 600]);
    hold on; grid on;

    % 计算并绘制每架无人机的总速度
    for UAV_id = 1:params.M+1
        % 获取三个方向的速度分量
        vx = data_table.(sprintf('UAV%d_Xvel',UAV_id));
        vy = data_table.(sprintf('UAV%d_Yvel',UAV_id));
        vz = data_table.(sprintf('UAV%d_Zvel',UAV_id));
        
        % 计算总速度
        total_vel = sqrt(vx.^2 + vy.^2 + vz.^2);
        
        % 绘制总速度曲线
        if UAV_id == 1
            plot(data_table.Time, total_vel, 'Color', colors(UAV_id,:), 'LineWidth', 2, 'LineStyle', '--');
        else
            plot(data_table.Time, total_vel, 'Color', colors(UAV_id,:), 'LineWidth', 1.5);
        end
    end
    % 设置图表属性
    title('各无人机速度变化');
    xlabel('时间(s)');
    ylabel('速度(m/s)');
    legend_names = {'Target'};
    for i = 1:params.M
        legend_names{end+1} = sprintf('UAV%d', i);
    end
    legend(legend_names);
    grid on;

    % 无人机与最近障碍物距离分析
    figure('Name','无人机与最近障碍物距离','Color','w','Position',[100 100 800 600]);
    hold on; grid on;

    % 计算并绘制每架无人机与最近障碍物的距离
    for UAV_id = 2:params.M+1  % 从2开始，因为1是目标
        % 获取无人机位置
        Ux = data_table.(sprintf('UAV%d_X',UAV_id));
        Uy = data_table.(sprintf('UAV%d_Y',UAV_id));
        Uz = data_table.(sprintf('UAV%d_Z',UAV_id));
        
        % 计算与每个障碍物的距离
        min_distances = zeros(size(Ux));
        for t = 1:length(Ux)
            uav_pos = [Ux(t), Uy(t), Uz(t)];
            distances = zeros(size(params.obs_pos,1), 1);
            for i = 1:size(params.obs_pos,1)
                distances(i) = norm(uav_pos - params.obs_pos(i,:)) - params.obs_radius(i);
            end
            min_distances(t) = min(distances);
        end
        
        % 绘制距离曲线
        plot(data_table.Time, min_distances, 'Color', colors(UAV_id,:), 'LineWidth', 1.5);
    end

    % 设置图表属性
    title('各无人机与最近障碍物距离');
    xlabel('时间(s)');
    ylabel('距离(m)');
    legend_names = {};
    for i = 1:params.M
        legend_names{end+1} = sprintf('UAV%d', i);
    end
    legend(legend_names);
    grid on;

    % 添加安全距离参考线
    yline(params.safety_margin, 'r--', 'LineWidth', 1.5, 'Label', '安全距离0.3');

    % γ值时间序列分析
    figure('Name','γ值时间序列','Color','w','Position',[100 100 800 600]);
    hold on; grid on;

    for UAV_id = 2:params.M+1  % 从2开始，因为1是目标
        % 获取无人机位置
        Ux = data_table.(sprintf('UAV%d_X',UAV_id));
        Uy = data_table.(sprintf('UAV%d_Y',UAV_id));
        Uz = data_table.(sprintf('UAV%d_Z',UAV_id));
        
        % 计算与每个障碍物的距离
        min_distances = zeros(size(Ux));
        for t = 1:length(Ux)
            uav_pos = [Ux(t), Uy(t), Uz(t)];
            distances = zeros(size(params.obs_pos,1), 1);
            for i = 1:size(params.obs_pos,1)
                distances(i) = norm(uav_pos - params.obs_pos(i,:)) - params.obs_radius(i);
            end
            min_distances(t) = min(distances);
        end
        % 计算γ值
        gamma = 0.5 * (1 + erf(0.3 * (min_distances - 4)));
        % 绘制γ值曲线
        plot(data_table.Time, gamma, 'Color', colors(UAV_id,:), 'LineWidth', 1.5);
    end

    % 设置图表属性
    title('各无人机γ值随时间变化');
    xlabel('时间(s)');
    ylabel('\gamma 值');
    legend_names = {};
    for i = 1:params.M
        legend_names{end+1} = sprintf('UAV%d', i);
    end
    legend(legend_names);
    grid on;
    ylim([0 1]);

    % 各无人机与目标距离
    figure('Name','各无人机与目标距离','Color','w','Position',[100 100 800 600]);
    hold on; grid on;

    % 计算并绘制每架无人机与目标的距离
    Tx = data_table.(sprintf('UAV1_X'));
    Ty = data_table.(sprintf('UAV1_Y'));
    Tz = data_table.(sprintf('UAV1_Z'));

    for UAV_id = 2:params.M+1
        % 获取三个方向的速度分量
        Ux = data_table.(sprintf('UAV%d_X',UAV_id));
        Uy = data_table.(sprintf('UAV%d_Y',UAV_id));
        Uz = data_table.(sprintf('UAV%d_Z',UAV_id));
        % 计算总速度
        dis = sqrt((Tx-Ux).^2 + (Ty-Uy).^2 + (Tz-Uz).^2);
        % 绘制总速度曲线
        plot(data_table.Time, dis, 'Color', colors(UAV_id,:), 'LineWidth', 1.5);
    end

    % 设置图表属性
    title('各无人机与目标距离');
    xlabel('时间(s)');
    ylabel('距离(m)');
    legend('UAV1','UAV2','UAV3','UAV4','UAV5','UAV6','UAV7','UAV8');
    grid on;
end
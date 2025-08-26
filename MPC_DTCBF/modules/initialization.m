function params = initialization()
    % 系统参数初始化
    params.dt = 0.1;                % 时间步长
    params.T = 50;                  % 最大仿真时间
    params.t = 0:params.dt:params.T;
    
    % 无人机物理参数
    params.M = 8;                   % 无人机数量
    params.m = 0.5;                 % 质量
    params.g = 9.81;                % 重力加速度
    params.T_max = 1.3*params.m*params.g; % 最大推力
    params.max_angle = deg2rad(35); % 最大倾斜角
    params.u0 = [0; 0; params.m*params.g]; % 初始输入
    params.UAV_radius = 0.2;        % 无人机自身半径
    params.I = diag([0.00245, 0.00245, 0.0049]); % 惯性矩阵 [Ixx, Iyy, Izz]
    params.I_inv = inv(params.I);        % 惯性矩阵的逆
    
    % MPC-CBF参数
    params.N = 10;                   % 预测步长
    params.Q = diag([10,10,20]);    % 状态权重矩阵
    params.R = diag([0.1,0.1,0.1]); % 控制权重矩阵
    params.safety_margin = 0.3;     % 安全裕度
    params.gama = 0.3;             % CBF收敛系数
    params.gamma_A = 0.3;        % A ∈ (0, 0.5]
    params.gamma_c = 0.5;        % 可调整的参数c
    params.gamma_l_E = 2.0;      % 距离阈值
    
    % 随速度自适应的聚焦权重参数
    params.v_ref_min = 0.0;         % 逃跑者速度 ≤ v_ref_min 时 α=α_min
    params.v_ref_max = 3.0;         % 逃跑者速度 ≥ v_ref_max 时 α=α_max
    params.alpha_min = 0.05;        % 最分散：近乎均匀包围
    params.alpha_max = 0.85;        % 最聚焦：几乎集中到速度方向
    params.alpha_cluster = params.alpha_min; % 初始值
    
    % 目标参数
    params.goal_radius = 0.2;       % 目标半径
    params.goal_points = [20, 35, 35;  
                        35, 10, 10;
                        10,35,35];
    params.vel_goal = [0,0,0];
    
    % 斐波那契球生成算法
    params.phi = (1+sqrt(5))/2;     % 黄金分割比例
    params.i = 0:params.M-1;        % 索引序列
    params.x = mod(params.i/params.phi, 1); % 水平坐标
    params.y = params.i/params.M;   % 垂直坐标
    params.theta = acos(1 - 2*params.x); % 极角（仰角）
    params.phi_angle = 2*pi*params.y; % 方位角
    params.X = sin(params.theta) .* cos(params.phi_angle);
    params.Y = sin(params.theta) .* sin(params.phi_angle);
    params.Z = cos(params.theta);
    params.d = 1.5;
    params.lambda = 0.02;
    
    % 障碍物生成
    params.num_obs = 15;
    params.map_limits = [0 50; 0 50; 0 50];
    params.obs_limits = [0 40; 0 40; 0 40];
    params.obs_radius = 1.2 + rand(params.num_obs,1)*1.2;
    params.obs_pos = generate_obstacles(params.num_obs, params.obs_limits, params.obs_radius);
    
    % 可视化初始化
    params = visualization(params); 
    
    % 设置GIF保存路径
    params.gif_filename = 'simulation.gif';
    
    % 准备输出目录
    params.base_dir = fullfile(pwd,'picture');
    if ~exist(params.base_dir,'dir'); mkdir(params.base_dir); end
    
    params.ts = datestr(now,'yyyymmdd_HHMMSS');
    params.session_dir = fullfile(params.base_dir,params.ts);
    mkdir(params.session_dir);
    
    params.view_names = {'default','xy','xz'};
    params.out_dirs = cell(size(params.view_names));
    for i = 1:numel(params.view_names)
        params.out_dirs{i} = fullfile(params.session_dir,params.view_names{i});
        mkdir(params.out_dirs{i});
    end
    
    % 保存默认视角
    [params.az0, params.el0] = view;
end
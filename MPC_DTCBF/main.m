clc; clear; close all;

% 添加模块路径
addpath('modules');
addpath('utils');

% 系统参数初始化
params = initialization();

% 主仿真循环
main_simulation_loop(params);

% 数据导出和分析
data_export_and_analysis(params);
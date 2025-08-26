function update_visualization(traj1,traj2,traj3,traj4,traj5,traj6,traj7,traj8,uav1,uav2,uav3,uav4,uav5,uav6,uav7,uav8, goal_plot, goal_traj, pos1,pos2,pos3,pos4,pos5,pos6,pos7,pos8, goal_pos)
    addpoints(traj1, pos1(1), pos1(2), pos1(3));
    addpoints(traj2, pos2(1), pos2(2), pos2(3));
    addpoints(traj3, pos3(1), pos3(2), pos3(3));
    addpoints(traj4, pos4(1), pos4(2), pos4(3));
    addpoints(traj5, pos5(1), pos5(2), pos5(3));
    addpoints(traj6, pos6(1), pos6(2), pos6(3));
    addpoints(traj7, pos7(1), pos7(2), pos7(3));
    addpoints(traj8, pos8(1), pos8(2), pos8(3));
    addpoints(goal_traj, goal_pos(1), goal_pos(2), goal_pos(3));
    set(uav1, 'XData',pos1(1), 'YData',pos1(2), 'ZData',pos1(3));
    set(uav2, 'XData',pos2(1), 'YData',pos2(2), 'ZData',pos2(3));
    set(uav3, 'XData',pos3(1), 'YData',pos3(2), 'ZData',pos3(3));
    set(uav4, 'XData',pos4(1), 'YData',pos4(2), 'ZData',pos4(3));
    set(uav5, 'XData',pos5(1), 'YData',pos5(2), 'ZData',pos5(3));
    set(uav6, 'XData',pos6(1), 'YData',pos6(2), 'ZData',pos6(3));
    set(uav7, 'XData',pos7(1), 'YData',pos7(2), 'ZData',pos7(3));
    set(uav8, 'XData',pos8(1), 'YData',pos8(2), 'ZData',pos8(3));
    set(goal_plot, 'XData',goal_pos(1), 'YData',goal_pos(2), 'ZData',goal_pos(3));
    drawnow; % 强制实时更新图形以确保捕获完整帧
    pause(0.001)
end
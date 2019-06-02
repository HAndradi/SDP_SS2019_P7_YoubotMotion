roslaunch mir_bringup_sim robot.launch
roslaunch ~/kinetic/src/mas_industrial_robotics/mir_manipulation/mir_moveit_youbot/youbot-brsu-1/move_group.launch
roslaunch mir_2dnav 2dnav.launch
rviz                                             ---(visualize robotmodel and map)
roslaunch mcr_direct_base_controller direct_base_controller.launch
roslaunch mir_move_base_safe move_base.launch
 






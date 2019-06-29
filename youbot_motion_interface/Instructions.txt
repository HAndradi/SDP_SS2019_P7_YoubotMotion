First launch the following launch files and rviz
-------------------------------------------------
roslaunch mir_bringup_sim robot.launch
roslaunch ~/kinetic/src/mas_industrial_robotics/mir_manipulation/mir_moveit_youbot/youbot-brsu-1/move_group.launch
roslaunch mir_2dnav 2dnav.launch
roslaunch mcr_direct_base_controller direct_base_controller.launch
roslaunch mir_move_base_safe move_base.launch
rosrun youbot_motion_interface youbot_motion_interface_node.py

rviz                                                                   ---(visualize robotmodel and map)

Send motion commands by publishing to the following topic
----------------------------------------------------------
rostopic pub /youbot_motion_interface/youbot_motion_goal               ---(tab complete)

**The action types and their corresponding values are as follows:
  1) move base with goal pose
  2) move base with goal pose name
  3) direct base control with goal pose





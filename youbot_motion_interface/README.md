#### Sending a motion command to the youbot

A motion command can be sent by publishing to the topic, "/youbot_motion_interface/youbot_motion_goal". The following command can be used to send a motion command from the terminal,

>rostopic pub rostopic pub /youbot_motion_interface/youbot_motion_goal   ---(tab complete)

The available action types for the arm and base motion can be found in the "Goal_Arm.msg" and "Goal_Base.msg" files. The following commands can be used to display the action types,

>rosmsg show youbot_motion_interface/Goal_Arm
>rosmsg show youbot_motion_interface/Goal_Base

#### Displaying the feedback information

The acknowledgement and result of the requested motion commands as well the activity of the youbot's arm and base can be monitored using the topics "/youbot_motion_interface/youbot_motion_acknowledgement", "/youbot_motion_interface/youbot_motion_result", and "/youbot_motion_interface/youbot_monitor_feedback" respectively.

The following commands could be run to view the information on these topics from the terminal,

>rostopic echo /youbot_motion_interface/youbot_motion_acknowledgement

>rostopic echo /youbot_motion_interface/youbot_motion_result

>rostopic echo /youbot_motion_interface/youbot_motion_monitor_feedback






## Project 7: Synchronized base and arm motions
### Date: 29.04.2019

Mihir Mulye:

   * I learned basics of YouBot.
   * I will learn more about MoveIt and about ROS
   * I still have to go into more details of the implementation.

Heruka Andradi:

   * I taught Mihir about the basics of YouBot. Learnt how to use move_base and direct base control on youbot simulation.
   * Create a simple action server to give motion commands to the base.
   * Action server might not be a good solution to the problem.


Carlo Wiesse:

   * Teach Heruka and Mihir a little about MoveIt commander. 
   * I should learn about action libraries and servers. Also, investigate how to avoid singularities.
   * Learn more about ROS combined with C++


### Date: 06.05.2019

Mihir Mulye:

   * I installed youbot simulation software 
   * I will learn how to use the software in a more detailed manner
   

Heruka Andradi:

   * Setup youbot simulation to control youbot base using move_base and direct_base_control
   * Fix the youbot simulation for arm control and start writing a simple program to control the robot arm and base simultaneously. Try to understand the direct_base_control code
   * Might have difficulties fixing the simulation

Carlo Wiesse:

   * Got a better undestanding of the pipeline for the Manipulation and learned a little bit about yb_actions libraries. 
   * Extract useful information from the yb_actions libraries to share with my group 
   * Might be difficult to decide on a flexible but robust architecture for the whole body motion
   
   
### Date: 13.05.2019

Mihir Mulye:

   * Worked on basic control flow of the task process 
   * Work with Heruka to modify the task process according to his code. Modify the test code according to the repository code 
   

Heruka Andradi:

   * Wrote a simple script to simultaneously control the base and arm (using moveit and move_base) by publishing the goal location and arm position names to a rostopic. Tried to debug moveit execution failures.
   * Incorporate direct_base_control and moveit based arm control using joint angle commands to the current script.
   * Might be difficult to handle simultaneous base motion commands to move_base and direct_base_control

Carlo Wiesse:

   * Implementation of custom cartisian velocity controller package using C++
   * Add preemted functionality to such package.
   
### Date: 20.05.2019

Mihir Mulye:

   * Tried to refactor the main code according to the sample repository code 
   * Will try to refactor the main code as i had some issues with the last attempt. Construct the flowchart which Heruka created. 
  

Heruka Andradi:

   * Tested using move_base and direct_base_control to give simultaneous base motion commands to the robot.
   * Decide how to handle simultaneous base motion commands and implement using a simple python script.
   * Might not be possible to effectively hand simultaneous base motion commands without modifying the move_base and direct_base_control packages.

Carlo Wiesse:

   * mcr_moveit_client package revision and transfer knowledge about it to teammates.
   * Contribution to graphical representation of pipeline by listing topics and message types to communicate with arm.
   
### Date: 27.05.2019

Mihir Mulye:

   * Carlo and I created message files and Interface files
  

Heruka Andradi:

   * Made a ros package to provide move_base and direct_base_controller based motion commands to the youbot using a single rostopic. Identified how to obtain the node name of subscribers for monitoring and feedback.
   * Incorporate moveit and monitoring to the ros package. Figure out how to cancel move_base_safe commands
   * Might not be able to cancel move_base_safe commands without modifying the move_base_safe package

Carlo Wiesse:

   *
   *
   *
   
### Date: 03.06.2019

Mihir Mulye:

   * Carlo and i worked on the task of moving arm using custom file on the simulation. 
   * test the package 'youbot_motion_interface package' on the youbot write a script to publish commands to the youbot_motion_interface package by passing arguments to the python file. 
   
  

Heruka Andradi:

   * Incorporated preemption, request acknowledgement,and result feedback functionalities to the youbot_motion_interface package to give goal pose or goal name based commands to the youbot using move_base or direct_base_control packages.
   * Integrate arm functionality to the youbot_motion_interface package
   * Might come across unexpected issues during integration

Carlo Wiesse:

   *
   *
   *
   
   ### Date: 17.06.2019

Mihir Mulye:

   * tested the package 'youbot_motion_interface package' on the youbot 
   * write a script to publish commands to the youbot_motion_interface package by passing arguments to the python file. 
   
  

Heruka Andradi:

   * Refactored the code and seperated the functionality into different classes. Added a monitor feedback functionality.
   * Integrate the arm_motion_interface package from Carlo to the youbot_motion_interace.
   *
   
Carlo Wiesse:

   *
   *
   *
   
   ### Date: 24.06.2019

Mihir Mulye:

   * 
   * 
   
  

Heruka Andradi:

   * Refactortored the youbot_motion_interface pakcage to accomodate arm motion commands. Tested giving simultaneous base motion commands with moveit name based arm motion commands.
   * Integrate the rest of the arm motion functionality and test on the youbot.
   *
   
Carlo Wiesse:

   *
   *
   *



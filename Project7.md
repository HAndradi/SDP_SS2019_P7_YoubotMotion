
## Project 7: Synchronized base and arm motions
Date: 29.04.2019

<Mihir Mulye>:

   * I learned basics of YouBot.
   * I will learn more about MoveIt and about ROS
   * I still have to go into more details of the implementation.

<Heruka Andradi>:

   * I taught Mihir about the basics of YouBot. Learnt how to use move_base and direct base control on youbot simulation.
   * Create a simple action server to give motion commands to the base.
   * Action server might not be a good solution to the problem.


<Carlo Wiesse>:

   * Teach Heruka and Mihir a little about MoveIt commander. 
   * I should learn about action libraries and servers. Also, investigate how to avoid singularities.
   * Learn more about ROS combined with C++


Date: 06.05.2019

<Mihir Mulye>:

   * I installed youbot simulation software 
   * I will learn how to use the software in a more detailed manner
   * 

<Heruka Andradi>:

   * Setup youbot simulation to control youbot base using move_base and direct_base_control
   * Fix the youbot simulation for arm control and start writing a simple program to control the robot arm and base simultaneously. Try to understand the direct_base_control code
   * Might have difficulties fixing the simulation

<Carlo Wiesse>:

   * Got a better undestanding of the pipeline for the Manipulation and learned a little bit about yb_actions libraries. 
   * Extract useful information from the yb_actions libraries to share with my group 
   * Might be difficult to decide on a flexible but robust architecture for the whole body motion
   
   
Date: 13.05.2019

<Mihir Mulye>:

   * Worked on basic control flow of the task process 
   * Work with Heruka to modify the task process according to his code. Modify the test code according to the repository code 
   * 

<Heruka Andradi>:

   * Wrote a simple script to simultaneously control the base and arm (using moveit and move_base) by publishing the goal location and arm position names to a rostopic. Tried to debug moveit execution failures.
   * Incorporate direct_base_control and moveit based arm control using joint angle commands to the current script.
   * Might be difficult to handle simultaneous base motion commands to move_base and direct_base_control

<Carlo Wiesse>:

   *  
   *
   * 


# sdp_ss2019_P7_YoubotMotion

![Python version](https://img.shields.io/badge/PyPI-v2.7-brightgreen.svg) ![GitHub contributors](https://img.shields.io/badge/contributors-3-blue.svg) ![GitHub issues](https://img.shields.io/badge/issues-0-9cf.svg)

## Install Ubuntu
The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Indigo: Ubuntu 16.04

If you do not have a Ubuntu distribution on your computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control
### Install Git Software
Install the Git core components and some additional GUI's for the version control:

     sudo apt-get install git-core gitg gitk

### Set Up Git
Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

     git config --global user.name "Your Name Here"

Git also saves your email address into the commits you make.

     git config --global user.email "your-email@youremail.com"


### GIT Tutorial
If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/


## ROS - Robot Operating System
### Install ROS
The repository has been tested successfully with the following ROS distributions. Use the link behind a ROS distribution to get to the particular ROS installation instructions.


- ROS Indigo - http://wiki.ros.org/indigo/Installation/Ubuntu

NOTE: Do not forget to update your .bashrc!
 

### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://wiki.ros.org/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters"). 


## Set up a catkin workspace

    source /opt/ros/indigo/setup.bash
    mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws
    catkin_make
    
## Clone and compile the RoboCup@Work source code
First of all you have to clone the repository.

    cd ~/catkin_ws/src
    git clone git@github.com:mas-group/robocup-at-work.git

Then go on with installing further external dependencies:
       
    cd ~/catkin_ws/src/robocup-at-work/mas_industrial_robotics
    ./repository.debs
    
    source ~/catkin_ws/devel/setup.bash

The last command should be added to the ~/.bashrc file so that they do not need to be executed everytime you open a new terminal.


And finally compile the repository:

    cd ~/catkin_ws
    catkin_make


If no errors appear everything is ready to use. Great job!


### Setting the Environment Variables
#### ROBOT variable
With the ROBOT variable you can choose which hardware configuration should be loaded when starting the robot. The following line will add the variable to your .bashrc:

     echo "export ROBOT=youbot-brsu-1" >> ~/.bashrc
     source ~/.bashrc



#### ROBOT_ENV Variable
The ROBOT_ENV variable can be used to switch between different environments. The following line will add the variable to your .bashrc:

     echo "export ROBOT_ENV=brsu-c025-sim" >> ~/.bashrc
     source ~/.bashrc



## Bring up the robot and it's basic components
### In Simulation

     roslaunch mir_bringup_sim robot.launch
     roslaunch ~/kinetic/src/mas_industrial_robotics/mir_manipulation/mir_moveit_youbot/youbot-brsu-1/move_group.launch     

### At the Real Robot

     roslaunch mir_bringup robot.launch
     roslaunch mir_planning_bringup robot.launch     

## Use youbot motion package 

    roslaunch youbot_motion_interface youbot_motion_interface.launch 

     



# sdp_ss2019_P7_YoubotMotion

![Python version](https://img.shields.io/badge/PyPI-v3.5.2-brightgreen.svg) ![GitHub contributors](https://img.shields.io/badge/contributors-3-blue.svg) ![GitHub issues](https://img.shields.io/badge/issues-11-critical.svg)

**Milestones**

1. Establishing a common ground

what? Make sure that all team members understand how ROS works and what is the goal of our current project.

why? So that all team members can contribute towards the common goal and understand each other's arguments with respect to the ROS interface.

expected? Basic knowledge about ROS nodes, topics, subscribers, publishers, servers and clients.

This milestones is reached once the team settles on the number of subscribers/publishers or client/servers needed for this project and sketches an overview of our desired system architecture.

2. Exploring available resources

what? Since we'll be working with the KUKA arm and base, many of the desired functionalities for our project have already been implemented and can be found in the b-it-bots repositories.

why? It would be wise to locate the packages that can be useful to us and undestand how they work. This will save us time and effort.

expected? We expect to find the functionalities for the arm manipulation and the base control in these repositories.

This milestone is reached once the team has successfully identified all relevant ROS packages from b-it-bots and thought of a way to make use of them.

3. Prototype implementation and simulated testing

what? Implement the arm and base controller nodes in separate ROS packages using python and test these functionalities using rviz simulation. After the functionality of the nodes is deemed correctly, we'll proceed to merge them into one ROS package

why? Before testing the new software on the robot, it would be safe to test it on a simulated environment first.

expected? We expect to have the arm and base controller nodes implemented in different ROS packages and fix any bugs that can be found during the simulated testing. Then, proceed to merge them into one ROS package and repeat the simulated testing.

This milestone is reached once our nodes do everything we expect them to do in the simulated environment.

4. Robot testing

what? Test our ROS package on the robot in a real-world environment.

why? This ROS package running on the robot is ultimately the goal of our whole project.

expected? We expect to fix any possible bugs that may come up during testing.

This milestone is reached once the integration with the robot is running as desired.

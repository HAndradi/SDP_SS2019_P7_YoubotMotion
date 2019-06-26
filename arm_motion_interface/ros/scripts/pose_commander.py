#!/usr/bin/env python 

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':

    rospy.init_node('pose_commander')
    arm = moveit_commander.MoveGroupCommander("arm_1")

#    arm_goal_pose_pub = rospy.Publisher("/moveit_client/target_pose", PoseStamped, queue_size=1)
    
    command = PoseStamped()
    command.header.frame_id = 'base_link'
    command.pose.position.x = 0.271407354048
    command.pose.position.y = -0.000873746568434
    command.pose.position.z = 0.642999977347
    command.pose.orientation.x = 0.0
    command.pose.orientation.y = 0.0
    command.pose.orientation.z = 0.0
    command.pose.orientation.w = 1.0

#    rospy.sleep(1)
#    arm_goal_pose_pub.publish(command)
    
    if arm.has_end_effector_link(): 
        rospy.loginfo('%s is good' % arm.get_name())
    
    # while True:
    #     try:
    #         arm.set_pose_target(command, end_effector_link="arm_link_5")
    #         rospy.loginfo("Goal pose sent")
    #         break
    #     except Exception as e:
    #         rospy.logerr('unable to set target position: %s' % (str(e)))
    #         continue
    
    arm.go(joints=command.pose, wait=True)

#!/usr/bin/env python 

import rospy
from sdp_project.msg import Arm
from brics_actuator.msg import JointPositions
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class ArmInterface(object):
	def __init__(self):

	    self.arm_goal_name_pub = rospy.Publisher("/moveit_client/target_string_pose",String,queue_size=1)
	    self.arm_goal_joints_pub = rospy.Publisher("/moveit_client/target_configuration",JointPositions,queue_size=1)
	    self.arm_goal_pose_pub = rospy.Publisher("/moveit_client/target_pose",PoseStamped,queue_size=1)

	def update(self):
	    rospy.Subscriber("~goal",Arm,self.goal_cb)

	
	def goal_cb(self,msg):
	    
	    if msg.arm_action_type == msg.ARM_ACTION_TYPE_NAME: 
	    	self.arm_goal_name_pub.publish(msg.name.data)
            	rospy.loginfo("Goal name given")

	    elif msg.arm_action_type == msg.ARM_ACTION_TYPE_POSE: 
	    	self.arm_goal_pose_pub.publish(msg.pose)
            	rospy.loginfo("Goal pose given")

	    elif msg.arm_action_type == msg.ARM_ACTION_TYPE_JOINTS: 
	    	self.arm_goal_joints_pub.publish(msg.joints)
            	rospy.loginfo("Goal joints given")
 
	    elif msg.arm_action_type == msg.ARM_ACTION_TYPE_VELOCITY: 
	    	self.arm_goal_pose_pub.publish(msg.pose)
            	rospy.loginfo("Goal pose given")

if __name__ == '__main__':

    rospy.init_node("arm_node")
    rospy.loginfo("Node initiated")
    interface_object = ArmInterface()
    loop_rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
	interface_object.update()
        loop_rate.sleep()

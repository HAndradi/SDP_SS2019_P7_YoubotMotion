#!/usr/bin/env python 

import rospy 
from sdp_project.msg import Arm
from brics_actuator.msg import JointPositions
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class ArmInterface(object):
	def __init__(self):
	    rospy.Subscriber("~goal",Arm,self.goal_cb)
	    
	  
	    self.arm_goal_name_pub = rospy.Publisher("~target_string_pose",String,queue_size=1)
	    self.arm_goal_joints_pub = rospy.Publisher("~target_configuration",JointPositions,queue_size=1)
	    self.arm_goal_pose_pub = rospy.Publisher("~target_pose",PoseStamped,queue_size=1)

	
	def goal_cb(self,msg):
	    
	    if msg.arm_action_type == msg.ARM_ACTION_TYPE_NAME: 
	    	self.arm_goal_name_pub.publish(msg.name)

	    elif msg.arm_action_type == msg.ARM_ACTION_TYPE_POSE: 
	    	self.arm_goal_pose_pub.publish(msg.pose)

	    elif msg.arm_action_type == msg.ARM_ACTION_TYPE_JOINTS: 
	    	self.arm_goal_joints_pub.publish(msg.joints)
 
	    elif msg.arm_action_type == msg.ARM_ACTION_TYPE_VELOCITY: 
	    	self.arm_goal_pose_pub.publish(msg.pose)

	    #for base
	    #self.stop_current_base = msg.base.stop_current
	    #self.goal_location_base = msg.base.goal_location
	    #self.location_name_base = msg.base.location_name


	    #for arm 
	    #self.stop_current_arm = msg.arm.stop_current 
	    #self.goal_location_arm = msg.arm.goal_location
	    #self.location_name_arm = msg.arm.location_name
	    #self.cartesian_velocities = msg.arm.cartesian_velocities
	    
	


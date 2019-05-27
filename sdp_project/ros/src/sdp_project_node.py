#!/usr/bin/env python

import rospy
from sdp_project.msg import StatusMessage, GoalMessage
from brics_actuator.msg import JointPositions
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String

class Interface(object):

	def __init__(self):
	    
	    rospy.Subscriber("~goal",GoalMessage,self.goal_cb)
	    rospy.Subscriber("~e_start",String,self.event_start_cb)
	    self.status_pub = rospy.Publisher("~status",StatusMessage,queue_size=1)
	    self.move_base_pub = rospy.Publisher("~move_base",String,queue_size=1)
	    self.direct_base_pub = rospy.Publisher("~direct_base", PoseStamped,queue_size=1)
	    self.moveit_client1_pub = rospy.Publisher("~moveit_client1",String,queue_size=1)
	    self.moveit_client2_pub = rospy.Publisher("~moveit_client2",JointPositions,queue_size=1)
	    self.moveit_client3_pub = rospy.Publisher("~moveit_client3",PoseStamped,queue_size=1)
	    self.velocity_pub = rospy.Publisher("~velocity_arm",TwistStamped,queue_size=1)
	
	def goal_cb(self,msg):
	    #for base 
	    self.stop_current_base = msg.base.stop_current
	    self.goal_location_base = msg.base.goal_location
	    self.location_name_base = msg.base.location_name

	    #for arm 
	    self.stop_current_arm = msg.arm.stop_current 
	    self.goal_location_arm = msg.arm.goal_location
	    self.location_name_arm = msg.arm.location_name
	    self.cartesian_velocities = msg.arm.cartesian_velocities
	    
	def event_start_cb(self,msg):
	    self.event_in = msg	

if __name__ == '__main__':

    rospy.init_node("interface")
    rospy.loginfo("Node initiated")
    interface_object = Interface()
    loop_rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
         loop_rate.sleep()


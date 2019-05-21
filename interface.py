#not sure of the import statement as of 21/05 
#can we make a cmake file

import ./msg/goal_message.msg
import msg.status_message.msg
import brics_actuator.msg 
import geometry_msgs.msg
import std_msgs.msg

class Interface(object):

	def __init__(self):
	    
	    rospy.Subscriber("~goal",goal_message,self.goal_cb)
	    rospy.Subscriber("~event_start",bool,self.event_start_cb)
	    self.status_pub = rospy.Publisher("~status",status_message,queue_size=1)
	    self.move_base_pub = rospy.Publisher("~move_base",std_msgs.msg.String,queue_size=1)
	    self.direct_base_pub = rospy.Publisher("~direct_base",geometry_msgs.msg.PoseStamped,queue_size=1)
	    self.moveit_client1_pub = rospy.Publisher("~moveit_client1",std_msgs.msg.String,queue_size=1)
	    self.moveit_client2_pub = rospy.Publisher("~moveit_client2",brics_actuator.msg.JointPositions,queue_size=1)
	    self.moveit_client3_pub = rospy.Publisher("~moveit_client3",geometry_msgs.msg.PoseStamped,queue_size=1)
	    self.velocity_pub = rospy.Publisher("~velocity_arm",geometry_msgs.msg.TwistStamped,queue_size=1)
	
	def goal_cb(self,goal):
	    #for base 
	    
	    self.stop_current_base = goal.base.stop_current
	    self.goal_location_base = goal.base.goal_location
	    self.location_name_base = goal.base.location_name

	    #for arm 
	    self.stop_current_arm = goal.arm.stop_current 
	    self.goal_location_arm = goal.arm.goal_location
	    self.location_name_arm = goal.arm.location_name
	    self.cartesian_velocities = goal.arm.cartesian_velocities
	    
	def event_start_cb(self,message):
	    self.event_in = message	

	def update(self):
	#WRITE STATE MACHINE SOMEWHERE
	    self.status_pub.publish(#write something here)  
	    self.move_base_pub.publish(#write something here)  
	    self.direct_base_pub.publish(#write something here)  
	    self.moveit_client1_pub.publish(#write something here)  
	    self.moveit_client2_pub.publish(#write something here)  
	    self.moveit_client3_pub.publish(#write something here)  
	    self.velocity_pub.publish(#write something here)  
	    
def main():
    rospy.init_node("interface")
    interface_object = Interface()
    interface_object.update()
   
         	

    


	    
	    
	    

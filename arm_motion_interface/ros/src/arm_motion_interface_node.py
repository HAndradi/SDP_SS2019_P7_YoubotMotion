#!/usr/bin/env python

import rospy
from sdp_project.msg import Arm, Result, Acknowledgement, Monitor
from brics_actuator.msg import JointPositions
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class ArmMotionCoordinator:
	def __init__(self):

		'''
        PUBLISHERS
        '''
		self.moveit_event_in_pub = rospy.Publisher('~moveit_event_in', String, queue_size=1)
		self.moveit_string_pub = rospy.Publisher('~moveit_string_command', String, queue_size=1)
		self.moveit_joints_pub = rospy.Publisher('~moveit_joints_command', JointPositions, queue_size=1)
		self.moveit_pose_pub = rospy.Publisher('~moveit_pose_command', PoseStamped, queue_size=1)
        self.cvc_event_in_pub = rospy.Publisher('~cvc_event_in', String, queue_size=1)
		self.cvc_pose_pub = rospy.Publisher('~cvc_pose_command', PoseStamped, queue_size=1)

        '''
        SUBSCRIBERS
        '''
		rospy.Subscriber('~cvc_event_out', String, self.cvc_event_out_cb)
		rospy.Subscriber('~moveit_event_out', String, self.moveit_event_out_cb)

	def acknowledge(self, action_type = 'UNSPECIFIED', request_node = 'unknown', message = ''):
		acknowledgement_msg = Acknowledgement()

		if self.status is 'accepted':
			acknowledgement_msg.request_node = self.request_node_caller_id
			acknowledgement_msg.action_type = self.new_action
			acknowledgement_msg.status = self.status
			acknowledgement_msg.error = message
			state = self.new_action
		else:
			acknowledgement_msg.request_node = request_node
			acknowledgement_msg.action_type = action_type
			acknowledgement_msg.status = self.status
			acknowledgement_msg.error = message

		feedback_pub.publish(acknowledgement_msg)
		self.new_action = 'UNSPECIFIED'

	def send_result(self):
		result_msg = Result()
		result_msg.action_type = self.state
		result_msg.status = self.status

		if status == 'stopped':
			result_msg.result_node = self.preempted_node_caller_id
		else:
			result_msg.result_node = self.request_node_caller_id

		result_pub.publish(result_msg)

	def moveit_event_out_cb(self, msg):
		if msg.data is 'e_success':
			self.moveit_status = Result().ACTION_SUCCEEDED
		elif msg.data is 'e_stopped':
			self.moveit_status = Result().ACTION_PREEMPTED
		elif msg.data is 'e_failure':
			self.moveit_status = Result().ACTION_FAILED

	def cvc_event_out_cb(self, msg):
		if msg.data is 'e_success':
			self.cvc_status = Result().ACTION_SUCCEEDED
		elif msg.data is 'e_stopped':
			self.cvc_status = Result().ACTION_PREEMPTED
		elif msg.data is 'e_failure':
			self.cvc_status = Result().ACTION_FAILED

	def execute_moveit_motion(self, goal_msg):
		if goal_msg.arm_action_type is Goal().ARM_ACTION_TYPE_STRING:
			self.moveit_string_pub.publish(goal_msg)

		elif goal_msg.arm_action_type is Goal().ARM_ACTION_TYPE_POSE:
			self.moveit_pose_pub.publish(goal_msg)

		elif goal_msg.arm_action_type is Goal().ARM_ACTION_TYPE_JOINTS:
			self.moveit_joints_pub.publish(goal_msg)

	def execute_cvc_motion(self, goal_msg):
		self.moveit_cvc_pose_pub.publish(goal_msg)

	def preempt_moveit_motion(self):
		self.moveit_event_in_pub.publish('e_stop')

	def preempt_cvc_motion(self):
		self.cvc_event_in_pub.publish('e_stop')

	def get_moveit_status(self):
		return self.moveit_status

	def get_cvc_status(self):
		return self.cvc_status

class MotionCoordinator:
	def __init__(self):

		'''
        INITIALIZERS
        '''
        self.arm_motion_coordinator = ArmMotionCoordinator()
		self.state = 'IDLE'
        self.action_type = Goal().DEFAULT
	    self.arm_command_action_type = Goal().DEFAULT
		self.actions = ['idle', 'moveit_name', 'moveit_pose', 'moveit_joints', 'cvc_pose']
		self.errors = ['', 'Another action is active and preemption not requested!', 'Invalid goal']

		'''
		PUBLISHERS
		'''
		self.acknowledgement_pub = rospy.Publisher('~acknowledgement', Acknowledgement, queue_size=1)
		self.result_pub = rospy.Publisher('~result', Result, queue_size=1)
		self.monitor_pub = rospy.Publisher('~monitor', Monitor, queue_size=1)

		'''
		SUBSCRIBERS
		'''
		rospy.Subscriber('~youbot_motion_goal', Goal, self.motion_command_cb)

	def motion_command_cb(self, msg):
        if self.state is not 'IDLE' and not msg.preempt_current_action:
        	self.send_acknowledgement(msg._connection_header['callerid'], msg.action_type,\
									  Acknowledgement().REQUEST_REJECTED,\
									  Acknowledgement().DIFFERENT_ACTION_IS_RUNNING_ERROR)
        else:

      		if self.state is not 'IDLE':
               	self.preempted_caller_id = self.arm_command_caller_id
                self.preempted_action_type = self.arm_command_action_type
                self.arm_command_preempt_flag = msg.preempt_current_action
            else:
                self.arm_command_preempt_flag = False

  	        self.arm_command_action_type = msg.action_type
    	    self.arm_command_goal_pose = msg.arm_goal_pose
        	self.arm_command_goal_name = msg.arm_goal_pose_name
            self.arm_command_caller_id = msg._connection_header['callerid']

    def send_acknowledgement(self, caller_id, action_type, status_type=Acknowledgement().REQUEST_ACCEPTED,\
							 error_type=Acknowledgement().NO_ERROR):
        acknowledgement_msg = Acknowledgement()
        acknowledgement_msg.caller_id = caller_id
        acknowledgement_msg.action_type = action_type

        if action_type in range(5):
			acknowledgement_msg.action_name = self.actions[action_type]
        else:
            acknowledgement_msg.action_name = 'invalid action type'

        acknowledgement_msg.status_type = status_type
        acknowledgement_msg.status = ['rejected', 'accepted'][status_type]
        acknowledgement_msg.error_type = error_type
        acknowledgement_msg.error_msg = self.errors[error_type]
        self.acknowledgement_pub.publish(acknowledgement_msg)

    def send_monitor_feedback(self):
        monitor_feedback_msg = Monitor()
        monitor_feedback_msg.action_type = self.arm_command_action_type
        monitor_feedback_msg.action_name = self.actions[monitor_feedback_msg.action_type]

        if monitor_feedback_msg.action_type in range(1,5):
            monitor_feedback_msg.caller_id = self.arm_command_caller_id

        self.monitor_feedback_pub.publish(monitor_feedback_msg)

    def send_result(self, status_type):
        result_msg = Result()
        result_msg.status_type = status_type

        if status_type == Result().ACTION_PREEMPTED:
            result_msg.caller_id = self.preempted_caller_id
            result_msg.action_type = self.preempted_action_type
        else:
            result_msg.caller_id = self.arm_command_caller_id
            result_msg.action_type = self.arm_command_action_type
            self.arm_command_action_type = Goal().DEFAULT

		result_msg.action_name = self.actions[result_msg.action_type]
        result_msg.status = ['failed', 'succeeded', 'preempted'][result_msg.status_type]
        self.result_pub.publish(result_msg)

    def check_states(self):
        rospy.loginfo('READY!')
        while not rospy.is_shutdown():
            if self.state is 'IDLE':
                self.execute_idle_state()
            elif self.state is 'CVC':
                self.execute_cvc_command_state()
            else:
                self.execute_moveit_command_state()

            rospy.sleep(0.1)
            self.send_monitor_feedback()

    def execute_idle_state(self):
        if self.arm_command_action_type == Goal().ACTION_TYPE_CVC_POSE:
            self.state = 'CVC'
            self.arm_motion_coordinator.execute_cvc_motion(self.arm_command_goal_pose)
            self.send_acknowledgement(self.arm_command_caller_id, self.arm_command_action_type)
        elif self.arm_command_action_type == Goal().ACTION_TYPE_MOVEIT_POSE or\
			 self.arm_command_action_type == Goal().ACTION_TYPE_MOVEIT_NAME or\
			 self.arm_command_action_type == Goal().ACTION_TYPE_MOVEIT_JOINTS:
			 
			if self.base_command_goal_pose is None:
				self.send_acknowledgement(self.arm_command_caller_id, self.arm_command_action_type,\
										  Acknowledgement().REQUEST_REJECTED,\
                                          Acknowledgement().INVALID_GOAL_ERROR)
                self.arm_command_action_type = Goal().DEFAULT
                return 0

            self.state = 'MOVEIT'
            self.arm_motion_coordinator.execute_moveit_motion(self.arm_command_goal_pose)
            self.send_acknowledgement(self.arm_command_caller_id, self.arm_command_action_type)

    def execute_moveit_command_state(self):
        if self.arm_motion_coordinator.get_moveit_status() is not 'active':
            self.send_result(self.arm_motion_coordinator.moveit_status)
            self.state = 'IDLE'

        if self.arm_command_preempt_flag:
            self.arm_motion_coordinator.preempt_moveit_motion()
            self.arm_command_preempt_flag = False

    def execute_cvc_command_state(self):
        if self.arm_motion_coordinator.get_cvc_status() is not 'active':
        	self.send_result(self.arm_motion_coordinator.cvc_status)
        	self.state = 'IDLE'

        if self.arm_command_preempt_flag:
        	self.arm_motion_coordinator.preempt_cvc_motion()
			self.arm_command_preempt_flag = False

if __name__ == '__main__':
	rospy.init_node("arm_motion_interface_node")
	motion_coordinator = MotionCoordinator()
	motion_coordinator.check_states()


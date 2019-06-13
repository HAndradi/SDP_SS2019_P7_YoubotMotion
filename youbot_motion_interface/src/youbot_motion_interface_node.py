#! /usr/bin/env python
import rospy
import roslib
import numpy as np
import param_server_utils

import geometry_msgs.msg
from std_msgs.msg import String
from youbot_motion_interface.msg import Goal
from youbot_motion_interface.msg import Acknowledgement
from youbot_motion_interface.msg import Result

class BaseMotionCoordinator:
    def __init__(self):
        self.dbc_pose_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/input_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.dbc_event_in_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/coordinator/event_in', String, queue_size=1)
        self.move_base_pose_pub = rospy.Publisher('/move_base_wrapper/pose_in', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.move_base_event_in_pub = rospy.Publisher('/move_base_wrapper/event_in', String, queue_size=1)
        rospy.Subscriber('/mcr_navigation/direct_base_controller/coordinator/event_out', String, self.dbc_event_out_cb, queue_size=1)
        rospy.Subscriber('/move_base_wrapper/event_out', String, self.move_base_event_out_cb, queue_size=1)

    def dbc_event_out_cb(self, msg):
        if msg.data == 'e_success':
            self.dbc_status = Result().ACTION_SUCCEEDED
        elif msg.data == 'e_stopped':
            self.dbc_status = Result().ACTION_PREEMPTED
        elif msg.data == 'e_failure':
            self.dbc_status = Result().ACTION_FAILED

    def move_base_event_out_cb(self, msg):
        if msg.data == 'e_success':
            self.move_base_status = Result().ACTION_SUCCEEDED
        elif msg.data == 'e_stopped':
            self.move_base_status = Result().ACTION_PREEMPTED
        elif msg.data == 'e_failure':
            self.move_base_status = Result().ACTION_FAILED

    def execute_dbc_motion(self, goal_pose):
        self.dbc_pose_pub.publish(goal_pose)
        self.dbc_event_in_pub.publish('e_start')
        self.dbc_status = 'active'

    def preempt_dbc_motion(self):
        self.dbc_event_in_pub.publish('e_stop')

    def get_dbc_status(self):
        return self.dbc_status

    def execute_move_base_motion(self, goal_pose):
        self.move_base_pose_pub.publish(goal_pose)
        self.move_base_event_in_pub.publish('e_start')
        self.move_base_status = 'active'
    
    def preempt_move_base_motion(self):
        self.move_base_event_in_pub.publish('e_stop')

    def get_move_base_status(self):
        return self.move_base_status


##########################################################################################
##########################################################################################
##########################################################################################

class MotionCoordinator:
    def __init__(self):
        self.acknowledgement_pub = rospy.Publisher('~youbot_motion_acknowledgement', Acknowledgement, queue_size=1)
        self.result_pub = rospy.Publisher('~youbot_motion_result', Result, queue_size=1)
        rospy.Subscriber('~youbot_motion_goal', Goal, self.motion_command_cb, queue_size=1)

        self.base_motion_coordinator = BaseMotionCoordinator()
        self.base_command_action_type = Goal().DEFAULT
        self.state = 'IDLE'
        self.action_type = Goal().DEFAULT 

    def motion_command_cb(self, msg):
        if not self.state == 'IDLE' and not msg.preempt_current_action:
            self.send_acknowledgement(msg._connection_header['callerid'], msg.action_type, Acknowledgement().REQUEST_REJECTED, \
                                      Acknowledgement().DIFFERENT_ACTION_IS_RUNNING_ERROR)
        else:
            if not self.state == 'IDLE':
                self.preempted_caller_id = self.base_command_caller_id
                self.preempted_action_type = self.base_command_action_type
                self.base_command_preempt_flag = msg.preempt_current_action
            else:
                self.base_command_preempt_flag = False
            self.base_command_action_type = msg.action_type
            self.base_command_goal_pose = msg.base_goal_pose
            self.base_command_goal_name = msg.base_goal_pose_name
            self.base_command_caller_id = msg._connection_header['callerid']

    def send_acknowledgement(self, caller_id, action_type, status_type=Acknowledgement().REQUEST_ACCEPTED , error_type=Acknowledgement().NO_ERROR):
        acknowledgement_msg = Acknowledgement()
        acknowledgement_msg.caller_id = caller_id 
        acknowledgement_msg.action_type = action_type
        if action_type in range(4):
            acknowledgement_msg.action_name = ['unkown', 'move base pose', 'move base name', 'dbc'][action_type]
        else:
            acknowledgement_msg.action_name = 'invalid action type'
        acknowledgement_msg.status_type = status_type
        acknowledgement_msg.status = ['rejected', 'accepted'][status_type]
        acknowledgement_msg.error_type = error_type
        acknowledgement_msg.error_msg = ['', 'another action is active and preemption not requested!', 'Invalid goal pose name'][error_type]
        self.acknowledgement_pub.publish(acknowledgement_msg)

    def send_result(self, status_type):
        result_msg = Result()
        result_msg.status_type = status_type
        if status_type == Result().ACTION_PREEMPTED:
            result_msg.caller_id = self.preempted_caller_id
            result_msg.action_type = self.preempted_action_type
        else:
            result_msg.caller_id = self.base_command_caller_id
            result_msg.action_type = self.base_command_action_type
            self.base_command_action_type = Goal().DEFAULT
        result_msg.action_name = ['unkown', 'move base pose', 'move base name', 'dbc'][result_msg.action_type]
        result_msg.status = ['failed', 'succeeded', 'preempted'][result_msg.status_type]
        self.result_pub.publish(result_msg)
        
    def check_states(self):
        print 'READY!'
        while not rospy.is_shutdown():
            if self.state == 'IDLE':
                self.execute_idle_state()
            elif self.state == 'DBC':
                self.execute_dbc_command_state()
            else:
                self.execute_move_base_command_state()
            rospy.sleep(0.1)

    def execute_idle_state(self):
        if self.base_command_action_type == Goal().ACTION_TYPE_DBC_POSE:
            self.state = 'DBC'
            self.base_motion_coordinator.execute_dbc_motion(self.base_command_goal_pose)
            self.send_acknowledgement(self.base_command_caller_id, self.base_command_action_type)
        elif self.base_command_action_type == Goal().ACTION_TYPE_MOVE_BASE_POSE or self.base_command_action_type == Goal().ACTION_TYPE_MOVE_BASE_NAME:
            if self.base_command_action_type == Goal().ACTION_TYPE_MOVE_BASE_NAME:
                self.base_command_goal_pose = param_server_utils.get_pose_from_param_server(self.base_command_goal_name)
                if self.base_command_goal_pose is None:
                    self.send_acknowledgement(self.base_command_caller_id, self.base_command_action_type, Acknowledgement().REQUEST_REJECTED, \
                                              Acknowledgement().INVALID_POSE_NAME_ERROR)
                    self.base_command_action_type = Goal().DEFAULT
                    return
            self.state = 'MOVE_BASE'
            self.base_motion_coordinator.execute_move_base_motion(self.base_command_goal_pose)
            self.send_acknowledgement(self.base_command_caller_id, self.base_command_action_type)
            
    def execute_dbc_command_state(self):
        if not self.base_motion_coordinator.get_dbc_status() == 'active': 
            self.send_result(self.base_motion_coordinator.dbc_status)
            self.state = 'IDLE'
        if self.base_command_preempt_flag:
            self.base_motion_coordinator.preempt_dbc_motion()
            self.base_command_preempt_flag = False

    def execute_move_base_command_state(self):
        if not self.base_motion_coordinator.get_move_base_status() == 'active': 
            self.send_result(self.base_motion_coordinator.move_base_status)
            self.state = 'IDLE'
        if self.base_command_preempt_flag:
            self.base_motion_coordinator.preempt_move_base_motion()
            self.base_command_preempt_flag = False

if __name__ == '__main__':
    rospy.init_node('youbot_motion_interface')
    motion_coordinator = MotionCoordinator()
    motion_coordinator.check_states()

#! /usr/bin/env python
import rospy
import roslib
import numpy as np
import param_server_utils

import geometry_msgs.msg
from std_msgs.msg import String
from youbot_motion_interface.msg import Goal
from youbot_motion_interface.msg import Acknowledgement
from youbot_motion_interface.msg import Monitor
from youbot_motion_interface.msg import Result

import time
import brics_actuator.msg
class ArmMotionCoordinator:
    def __init__(self):
        self.moveit_event_in_pub = rospy.Publisher('/moveit_client/event_in', String, queue_size=1)
        self.moveit_configuration_pub = rospy.Publisher('/moveit_client/target_configuration', brics_actuator.msg.JointPositions, queue_size=1)
        self.moveit_pose_pub = rospy.Publisher('/moveit_client/target_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.moveit_pose_name_pub = rospy.Publisher('/moveit_client/target_string_pose', String, queue_size=1)
        rospy.Subscriber('/moveit_client/event_out', String, self.moveit_event_out_cb, queue_size=1)

    def moveit_event_out_cb(self, msg):
        if msg.data == 'e_success':
            self.moveit_status = Result().ACTION_SUCCEEDED
        elif msg.data == 'e_stopped':
            self.moveit_status = Result().ACTION_PREEMPTED
        elif msg.data == 'e_failure':
            self.moveit_status = Result().ACTION_FAILED
    
    def execute_moveit_pose_name_motion(self, goal_pose_name):
        self.moveit_pose_name_pub.publish(goal_pose_name)
        self.moveit_event_in_pub.publish('e_start') 
        self.moveit_status = 'active'

    def execute_moveit_pose_motion(self, goal_pose):
        self.moveit_pose_name_pub.publish(goal_pose)
        self.moveit_event_in_pub.publish('e_start') 
        self.moveit_status = 'active'

    def execute_moveit_joint_configuration_motion(self, joint_configuration):
        self.moveit_pose_name_pub.publish(joint_configuration)
        self.moveit_event_in_pub.publish('e_start') 
        self.moveit_status = 'active'

    def get_moveit_status(self):
        return self.moveit_status


##########################################################################################
##########################################################################################
##########################################################################################


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
        self.monitor_feedback_pub = rospy.Publisher('~youbot_motion_monitor_feedback', Monitor, queue_size=1)
        self.result_pub = rospy.Publisher('~youbot_motion_result', Result, queue_size=1)
        rospy.Subscriber('~youbot_motion_goal', Goal, self.motion_command_cb, queue_size=1)

        self.base_motion_coordinator = BaseMotionCoordinator()
        self.arm_motion_coordinator = ArmMotionCoordinator()
        self.base_command_action_type = Goal().base.DEFAULT
        self.arm_command_action_type = Goal().arm.DEFAULT
        self.base_state = 'IDLE'
        self.arm_state = 'IDLE'
        self.whole_body_motion_flag = False

    def motion_command_cb(self, msg):
        if (msg.base.action_type in range(1,4) and not self.base_state == 'IDLE' and not msg.base.preempt_current_action) or \
                      (msg.arm.action_type in range(1,5) and not self.arm_state == 'IDLE' and not msg.arm.preempt_current_action):
            self.send_acknowledgement(msg._connection_header['callerid'], msg.arm.action_type, msg.base.action_type, \
                             Acknowledgement().REQUEST_REJECTED, Acknowledgement().DIFFERENT_ACTION_IS_RUNNING_ERROR)                
        else:
            if msg.base.action_type in range(1,4):
                if not self.base_state == 'IDLE':
                    self.base_preempted_caller_id = self.base_command_caller_id
                    self.base_preempted_action_type = self.base_command_action_type
                    self.base_command_preempt_flag = msg.base.preempt_current_action
                else:
                    self.base_command_preempt_flag = False
                self.base_command_action_type = msg.base.action_type
                self.base_command_goal_pose = msg.base.goal_pose
                self.base_command_goal_name = msg.base.goal_pose_name
                self.base_command_caller_id = msg._connection_header['callerid']
            if msg.arm.action_type in range(1,5):
                if not self.arm_state == 'IDLE':
                    self.arm_preempted_caller_id = self.arm_command_caller_id
                    self.arm_preempted_action_type = self.arm_command_action_type
                    self.arm_command_preempt_flag = msg.arm.preempt_current_action
                else:
                    self.arm_command_preempt_flag = False
                self.arm_command_action_type = msg.arm.action_type
                self.arm_command_goal_pose = msg.arm.goal_pose
                self.arm_command_goal_name = msg.arm.goal_pose_name
                self.arm_command_caller_id = msg._connection_header['callerid']

    def send_acknowledgement(self, caller_id, arm_action_type, base_action_type, status_type=Acknowledgement().REQUEST_ACCEPTED, \
                                                                                           error_type=Acknowledgement().NO_ERROR):
        acknowledgement_msg = Acknowledgement()
        acknowledgement_msg.caller_id = caller_id 
        acknowledgement_msg.arm_action_type = arm_action_type
        acknowledgement_msg.base_action_type = base_action_type
        if base_action_type in range(4):
            acknowledgement_msg.base_action_name = ['idle', 'move base pose', 'move base name', 'dbc'][base_action_type]
        else:
            acknowledgement_msg.base_action_name = 'invalid action type'
        if arm_action_type in range(4):
            acknowledgement_msg.arm_action_name = ['idle', 'moveit name', 'moveit pose', 'moveit joint angles', 'cartesian pose'][arm_action_type]
        else:
            acknowledgement_msg.arm_action_name = 'invalid action type'
        acknowledgement_msg.status_type = status_type
        acknowledgement_msg.status = ['rejected', 'accepted'][status_type]
        acknowledgement_msg.error_type = error_type
        acknowledgement_msg.error_msg = ['', 'another action is active and preemption not requested!', 'Invalid goal pose name'][error_type]
        self.acknowledgement_pub.publish(acknowledgement_msg)

    def send_monitor_feedback(self):
        monitor_feedback_msg = Monitor()
        if not self.base_state == 'IDLE':
            monitor_feedback_msg.caller_id = self.base_command_caller_id
            monitor_feedback_msg.action_type = self.base_command_action_type 
            monitor_feedback_msg.action_name = ['idle', 'move base pose', 'move base name', 'dbc'][monitor_feedback_msg.action_type]
        else: 
            monitor_feedback_msg.action_type = Monitor().DEFAULT 
            monitor_feedback_msg.action_name = 'idle'
        self.monitor_feedback_pub.publish(monitor_feedback_msg)

    def send_result(self, status_type):
        result_msg = Result()
        result_msg.status_type = status_type
        if status_type == Result().ACTION_PREEMPTED:
            result_msg.caller_id = self.preempted_caller_id
            result_msg.action_type = self.preempted_action_type
        else:
            result_msg.caller_id = self.base_command_caller_id
            result_msg.action_type = self.base_command_action_type
            self.base_command_action_type = Goal().base.DEFAULT
        result_msg.action_name = ['idle', 'move base pose', 'move base name', 'dbc'][result_msg.action_type]
        result_msg.status = ['failed', 'succeeded', 'preempted'][result_msg.status_type]
        self.result_pub.publish(result_msg)
        
    def check_states(self):
        print 'READY!'
        while not rospy.is_shutdown():
            if self.base_state == 'IDLE':
                self.execute_base_idle_state()
            elif self.base_state == 'DBC':
                self.execute_dbc_command_state()
            else:
                self.execute_move_base_command_state()

            if self.arm_state == 'IDLE':
                self.execute_arm_idle_state()
            elif self.arm_state == 'MOVEIT':
                self.execute_moveit_command_state()
            else:
                self.execute_cvc_command_state() #cartesian velocity controller
            rospy.sleep(0.1)
            self.send_monitor_feedback()

    def execute_arm_idle_state(self):
        if self.arm_command_action_type == Goal().arm.ACTION_TYPE_MOVEIT_NAME:
            self.arm_state = 'MOVEIT'
            self.arm_motion_coordinator.execute_moveit_pose_name_motion(self.arm_command_goal_name)
            print 'moving'
           
    def execute_moveit_command_state(self):
        if not self.arm_motion_coordinator.get_moveit_status() == 'active': 
            print 'MOVEIT RESULT !!!: '
            print self.arm_motion_coordinator.moveit_status
#            self.send_result(self.base_motion_coordinator.dbc_status)
            self.arm_state = 'IDLE'
            self.arm_command_action_type = Goal().arm.DEFAULT
 
    def execute_base_idle_state(self):
        base_acknowledgement = 'accepted'
        if self.base_command_action_type == Goal().base.ACTION_TYPE_DBC_POSE:
            self.base_state = 'DBC'
            self.base_motion_coordinator.execute_dbc_motion(self.base_command_goal_pose)
        elif self.base_command_action_type == Goal().base.ACTION_TYPE_MOVE_BASE_POSE or self.base_command_action_type == Goal().base.ACTION_TYPE_MOVE_BASE_NAME:
            if self.base_command_action_type == Goal().base.ACTION_TYPE_MOVE_BASE_NAME:
                self.base_command_goal_pose = param_server_utils.get_pose_from_param_server(self.base_command_goal_name)
                if self.base_command_goal_pose is None:
                    base_acknowledgement = 'rejected'
                    self.base_command_action_type = Goal().base.DEFAULT
                    return
            self.base_state = 'MOVE_BASE'
            self.base_motion_coordinator.execute_move_base_motion(self.base_command_goal_pose)
        else: 
            base_acknowledgement = 'unspecified'
        if base_acknowledgement == 'accepted':
            self.send_acknowledgement(self.base_command_caller_id, self.base_command_action_type, self.arm_command_action_type)
        elif base_acknowledgement == 'rejected':
            self.send_acknowledgement(self.base_command_caller_id, self.base_command_action_type, Acknowledgement().REQUEST_REJECTED, \
                                                                                             Acknowledgement().INVALID_POSE_NAME_ERROR)
        else:
            pass
            
    def execute_dbc_command_state(self):
        if not self.base_motion_coordinator.get_dbc_status() == 'active': 
            self.send_result(self.base_motion_coordinator.dbc_status)
            self.base_state = 'IDLE'
        if self.base_command_preempt_flag:
            self.base_motion_coordinator.preempt_dbc_motion()
            self.base_command_preempt_flag = False

    def execute_move_base_command_state(self):
        if not self.base_motion_coordinator.get_move_base_status() == 'active': 
            self.send_result(self.base_motion_coordinator.move_base_status)
            self.base_state = 'IDLE'
        if self.base_command_preempt_flag:
            self.base_motion_coordinator.preempt_move_base_motion()
            self.base_command_preempt_flag = False

if __name__ == '__main__':
    rospy.init_node('youbot_motion_interface')
    motion_coordinator = MotionCoordinator()
    motion_coordinator.check_states()

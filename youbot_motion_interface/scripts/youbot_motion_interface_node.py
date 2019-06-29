#! /usr/bin/env python
import sys
import rospy
import rospkg

rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path("mir_move_base_safe")+'/ros/scripts/')
import param_server_utils

from arm_motion_coordinator.ArmMotionCoordinator
from base_motion_coordinator import BaseMotionCoordinator
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from youbot_motion_interface.msg import Goal
from youbot_motion_interface.msg import Acknowledgement
from youbot_motion_interface.msg import Monitor
from youbot_motion_interface.msg import Result
from youbot_motion_interface.msg import Action_Info

class MotionCoordinator:
    def __init__(self):
        self.acknowledgement_pub = rospy.Publisher('~youbot_motion_acknowledgement', Acknowledgement, queue_size=1)
        self.monitor_feedback_pub = rospy.Publisher('~youbot_motion_monitor_feedback', Monitor, queue_size=1)
        self.result_pub = rospy.Publisher('~youbot_motion_result', Result, queue_size=1)
        rospy.Subscriber('~youbot_motion_goal', Goal, self.motion_command_cb, queue_size=1)

        self.arm_motion_coordinator = ArmMotionCoordinator()
        self.base_motion_coordinator = BaseMotionCoordinator()

        self.arm_action_types = {Action_Info().ACTION_TYPE_MOVEIT_POSE:'Moveit pose', Action_Info().ACTION_TYPE_MOVEIT_NAME:'Moveit name',
                                 Action_Info().ACTION_TYPE_MOVEIT_JOINTS:'Moveit joints', Action_Info().ACTION_TYPE_CVC_POSE:'CVC pose'}
        self.base_action_types = {Action_Info().ACTION_TYPE_MOVE_BASE_POSE:'Move base pose', Action_Info().ACTION_TYPE_MOVE_BASE_NAME:'Move base name',
                                  Action_Info().ACTION_TYPE_DBC_POSE:'DBC pose', Action_Info().ACTION_TYPE_DBC_NAME:'DBC name'}

        self.acknowledgement_status_types = {Acknowledgement().STATUS_TYPE_REJECTED:'Rejected', Acknowledgement().STATUS_TYPE_ACCEPTED:'Accepted'}
        self.acknowledgement_error_types = {Acknowledgement().ERROR_TYPE_INVALID_ACTION_TYPE:'An invalid action type has been entered.', \
          Acknowledgement().ERROR_TYPE_CONFLICTING_ACTION_RUNNING:'A conflicting action is already running. Please request preemption.', \
         Acknowledgement().ERROR_TYPE_ARM_CANNOT_BE_PREEMPTED:'The active arm action cannot be preempted. Please try again in a while.', \
                                                                                           Acknowledgement().ERROR_TYPE_NONE:'No errors.'}

        self.result_status_types = {Result().STATUS_TYPE_FAILED:'failed', Result().STATUS_TYPE_SUCCEEDED:'succeeded', \
                                                                            Result().STATUS_TYPE_PREEMPTED:'preempted'}
        self.result_error_types = {Result().ERROR_TYPE_NONE:'No errors', Result().ERROR_TYPE_CONTROLLER_FAILURE:'The controller failed to execute \
                                                     action', Result().ERROR_TYPE_INVALID_POSE_NAME:'An invalid target pose name has been entered'}

        self.arm_command_action_type = Action_Info().ACTION_TYPE_DEFAULT
        self.base_command_action_type = Action_Info().ACTION_TYPE_DEFAULT

        self.arm_state = 'IDLE'
        self.base_state = 'IDLE'

    def motion_command_cb(self, msg):
        if (not msg.arm.action_type in self.arm_action_types.keys() and not msg.arm.action_type == Action_Info().ACTION_TYPE_DEFAULT) or \
             (not msg.base.action_type in self.base_action_types.keys() and not msg.base.action_type == Action_Info().ACTION_TYPE_DEFAULT):
            self.send_acknowledgement(msg._connection_header['callerid'], msg.arm.action_type, msg.base.action_type, \
                             Acknowledgement().STATUS_TYPE_REJECTED, Acknowledgement().ERROR_TYPE_INVALID_ACTION_TYPE)                
        elif msg.arm.action_type in self.arm_action_types.keys() and self.arm_state == 'MOVEIT':
            self.send_acknowledgement(msg._connection_header['callerid'], msg.arm.action_type, msg.base.action_type, \
                         Acknowledgement().STATUS_TYPE_REJECTED, Acknowledgement().ERROR_TYPE_ARM_CANNOT_BE_PREEMPTED)                
        elif (msg.base.action_type in self.base_action_types.keys() and not self.base_state == 'IDLE' and not msg.base.preempt_current_action) or \
                         (msg.arm.action_type in self.arm_action_types.keys() and not self.arm_state == 'IDLE' and not msg.arm.preempt_current_action):
            self.send_acknowledgement(msg._connection_header['callerid'], msg.arm.action_type, msg.base.action_type, \
                      Acknowledgement().STATUS_TYPE_REJECTED, Acknowledgement().ERROR_TYPE_CONFLICTING_ACTION_RUNNING)                
        else:
            self.send_acknowledgement(msg._connection_header['callerid'], msg.arm.action_type, msg.base.action_type)
            if msg.base.action_type in self.base_action_types.keys():
                if not self.base_state == 'IDLE':
                    self.base_preempted_caller_id = self.base_command_caller_id
                    self.base_preempted_action_type = self.base_command_action_type
                    self.base_command_preempt_flag = msg.base.preempt_current_action
                else:
                    self.base_command_preempt_flag = False
                self.base_command_action_type = msg.base.action_type
                self.base_command_target_pose = msg.base.target_pose
                self.base_command_target_name = msg.base.target_pose_name
                self.base_command_caller_id = msg._connection_header['callerid']
            if msg.arm.action_type in self.arm_action_types.keys():
                if not self.arm_state == 'IDLE':
                    self.arm_preempted_caller_id = self.arm_command_caller_id
                    self.arm_preempted_action_type = self.arm_command_action_type
                    self.arm_command_preempt_flag = msg.arm.preempt_current_action
                else:
                    self.arm_command_preempt_flag = False
                self.arm_command_action_type = msg.arm.action_type
                self.arm_command_target_pose = msg.arm.target_pose
                self.arm_command_target_name = msg.arm.target_pose_name
                self.arm_command_target_joint_config = msg.arm.target_joint_configuration
                self.arm_command_caller_id = msg._connection_header['callerid']

    def send_acknowledgement(self, caller_id, arm_action_type, base_action_type, status_type=Acknowledgement().STATUS_TYPE_ACCEPTED, \
                                                                                        error_type=Acknowledgement().ERROR_TYPE_NONE):
        acknowledgement_msg = Acknowledgement()
        acknowledgement_msg.caller_id = caller_id 
        acknowledgement_msg.arm_action.type = arm_action_type
        acknowledgement_msg.base_action.type = base_action_type
        if base_action_type in self.base_action_types.keys():
            acknowledgement_msg.base_action.name = self.base_action_types[base_action_type] 
        elif base_action_type == Action_Info().ACTION_TYPE_DEFAULT:
            acknowledgement_msg.base_action.name = 'Idle'
        else:
            acknowledgement_msg.base_action.name = 'Invalid action'
        if arm_action_type in self.arm_action_types.keys():
            acknowledgement_msg.arm_action.name = self.arm_action_types[arm_action_type]
        elif arm_action_type == Action_Info().ACTION_TYPE_DEFAULT:
            acknowledgement_msg.arm_action.name = 'Idle'
        else:
            acknowledgement_msg.arm_action.name = 'Invalid action'
        acknowledgement_msg.status_type = status_type
        acknowledgement_msg.status_msg = self.acknowledgement_status_types[status_type]
        acknowledgement_msg.error_type = error_type
        acknowledgement_msg.error_msg = self.acknowledgement_error_types[error_type]
        self.acknowledgement_pub.publish(acknowledgement_msg)

    def send_monitor_feedback(self):
        monitor_feedback_msg = Monitor()
        if not self.base_state == 'IDLE':
            monitor_feedback_msg.base_action_caller_id = self.base_command_caller_id
            monitor_feedback_msg.base_action.type = self.base_command_action_type 
            monitor_feedback_msg.base_action.name = self.base_action_types[self.base_command_action_type]
        else: 
            monitor_feedback_msg.base_action.type = Action_Info().ACTION_TYPE_DEFAULT
            monitor_feedback_msg.base_action.name = 'Idle'
        if not self.arm_state == 'IDLE':
            monitor_feedback_msg.arm_action_caller_id = self.arm_command_caller_id
            monitor_feedback_msg.arm_action.type = self.arm_command_action_type 
            monitor_feedback_msg.arm_action.name = self.arm_action_types[self.arm_command_action_type]
        else: 
            monitor_feedback_msg.arm_action.type = Action_Info().ACTION_TYPE_DEFAULT
            monitor_feedback_msg.arm_action.name = 'Idle'
        self.monitor_feedback_pub.publish(monitor_feedback_msg)

    def send_result(self, status_type, command_type, error_type = Result().ERROR_TYPE_NONE):
        result_msg = Result()
        result_msg.status_type = status_type
        result_msg.error_type = error_type
        if status_type == Result().STATUS_TYPE_FAILED and error_type == Result().ERROR_TYPE_NONE:
            error_type = Result().ERROR_TYPE_CONTROLLER_FAILURE 
        if command_type == 'base_command':
            if status_type == Result().STATUS_TYPE_PREEMPTED:
                result_msg.caller_id = self.base_preempted_caller_id
                result_msg.base_action.type = self.base_preempted_action_type
            else:
                result_msg.caller_id = self.base_command_caller_id
                result_msg.base_action.type = self.base_command_action_type
            result_msg.base_action.name = self.base_action_types[result_msg.base_action.type]
            self.base_command_action_type = Goal().base.ACTION_TYPE_DEFAULT
        elif command_type == 'arm_command':
            if status_type == Result().STATUS_TYPE_PREEMPTED:
                result_msg.caller_id = self.arm_preempted_caller_id
                result_msg.arm_action.type = self.arm_preempted_action_type
            else:
                result_msg.caller_id = self.arm_command_caller_id
                result_msg.arm_action.type = self.arm_command_action_type
                self.arm_command_action_type = Goal().arm.ACTION_TYPE_DEFAULT
            result_msg.arm_action.name = self.arm_action_types[result_msg.arm_action.type]
            self.arm_command_action_type = Goal().base.ACTION_TYPE_DEFAULT
        result_msg.status_msg = self.result_status_types[status_type]
        result_msg.error_msg = self.result_error_types[error_type]
        self.result_pub.publish(result_msg)
        
    def check_states(self):
        print 'READY!'
        while not rospy.is_shutdown():
            if self.base_state == 'IDLE':
                self.execute_base_idle_state()
            elif self.base_state == 'DBC':
                self.execute_dbc_command_state()
            elif self.base_state == 'MOVE_BASE':
                self.execute_move_base_command_state()

            if self.arm_state == 'IDLE':
                self.execute_arm_idle_state()
            elif self.arm_state == 'MOVEIT':
                self.execute_moveit_command_state()
            elif self.arm_state == 'CVC':
                self.execute_cvc_command_state() 
            rospy.sleep(0.1)
            self.send_monitor_feedback()

    def execute_arm_idle_state(self):
        if self.arm_command_action_type == Goal().arm.ACTION_TYPE_MOVEIT_POSE:
            self.arm_state = 'MOVEIT'
            self.arm_motion_coordinator.execute_moveit_pose_motion(self.arm_command_target_pose)
        elif self.arm_command_action_type == Goal().arm.ACTION_TYPE_MOVEIT_NAME:
            self.arm_state = 'MOVEIT'
            self.arm_motion_coordinator.execute_moveit_pose_name_motion(self.arm_command_target_name)
        elif self.arm_command_action_type == Goal().arm.ACTION_TYPE_MOVEIT_JOINTS:
            self.arm_state = 'MOVEIT'
            self.arm_motion_coordinator.execute_moveit_joint_config_motion(self.arm_command_target_joint_config)
        elif self.arm_command_action_type == Goal().arm.ACTION_TYPE_CVC_POSE:
            self.arm_state = 'CVC'
            self.arm_motion_coordinator.execute_cvc_pose_motion(self.arm_command_target_pose)
           
    def execute_moveit_command_state(self):
        if not self.arm_motion_coordinator.get_moveit_status() == 'active':
            self.send_result(self.arm_motion_coordinator.get_moveit_status(), 'arm_command')
            self.arm_state = 'IDLE'
 
    def execute_cvc_command_state(self):
        if not self.arm_motion_coordinator.get_cvc_status() == 'active':
            self.send_result(self.arm_motion_coordinator.get_cvc_status(), 'arm_command')
            self.arm_state = 'IDLE'
        if self.arm_command_preempt_flag:
            self.arm_motion_coordinator.preempt_cvc_motion()
            self.arm_command_preempt_flag = False
 
    def execute_base_idle_state(self):
        if self.base_command_action_type == Goal().base.ACTION_TYPE_DBC_NAME or self.base_command_action_type == Goal().base.ACTION_TYPE_MOVE_BASE_NAME:
            self.base_command_target_pose = param_server_utils.get_pose_from_param_server(self.base_command_target_name)
            if self.base_command_target_pose is None:
                self.base_command_action_type = Goal().base.DEFAULT
                self.base_state = 'IDLE'
                self.send_result(self, self.arm_motion_coordinator.get_moveit_status(), 'base_command', error_type = Result().ERROR_TYPE_INVALID_POSE_NAME)
                return
        if self.base_command_action_type == Goal().base.ACTION_TYPE_DBC_POSE or self.base_command_action_type == Goal().base.ACTION_TYPE_DBC_NAME:
            self.base_state = 'DBC'
            self.base_motion_coordinator.execute_dbc_motion(self.base_command_target_pose)
        elif self.base_command_action_type == Goal().base.ACTION_TYPE_MOVE_BASE_POSE or self.base_command_action_type == Goal().base.ACTION_TYPE_MOVE_BASE_NAME:
            self.base_state = 'MOVE_BASE'
            self.base_motion_coordinator.execute_move_base_motion(self.base_command_target_pose)
            
    def execute_dbc_command_state(self):
        if not self.base_motion_coordinator.get_dbc_status() == 'active': 
            self.send_result(self.base_motion_coordinator.get_dbc_status(), 'base_command')
            self.base_state = 'IDLE'
        if self.base_command_preempt_flag:
            self.base_motion_coordinator.preempt_dbc_motion()
            self.base_command_preempt_flag = False

    def execute_move_base_command_state(self):
        if not self.base_motion_coordinator.get_move_base_status() == 'active': 
            self.send_result(self.base_motion_coordinator.get_move_base_status(), 'base_command')
            self.base_state = 'IDLE'
        if self.base_command_preempt_flag:
            self.base_motion_coordinator.preempt_move_base_motion()
            self.base_command_preempt_flag = False

if __name__ == '__main__':
    rospy.init_node('youbot_motion_interface')
    motion_coordinator = MotionCoordinator()
    motion_coordinator.check_states()

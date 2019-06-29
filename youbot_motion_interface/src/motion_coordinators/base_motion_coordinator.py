#! /usr/bin/env python
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from youbot_motion_interface.msg import Result

class BaseMotionCoordinator:
    def __init__(self):
        self.dbc_pose_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/input_pose', PoseStamped, queue_size=1)
        self.dbc_event_in_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/coordinator/event_in', String, queue_size=1)
        self.move_base_pose_pub = rospy.Publisher('/move_base_wrapper/pose_in', PoseStamped, queue_size=1)
        self.move_base_event_in_pub = rospy.Publisher('/move_base_wrapper/event_in', String, queue_size=1)
        rospy.Subscriber('/mcr_navigation/direct_base_controller/coordinator/event_out', String, self.dbc_event_out_cb, queue_size=1)
        rospy.Subscriber('/move_base_wrapper/event_out', String, self.move_base_event_out_cb, queue_size=1)

    def dbc_event_out_cb(self, msg):
        if msg.data == 'e_success':
            self.dbc_status = Result.STATUS_TYPE_SUCCEEDED
        elif msg.data == 'e_stopped':
            self.dbc_status = Result.STATUS_TYPE_PREEMPTED
        elif msg.data == 'e_failure':
            self.dbc_status = Result.STATUS_TYPE_FAILED

    def move_base_event_out_cb(self, msg):
        if msg.data == 'e_success':
            self.move_base_status = Result.STATUS_TYPE_SUCCEEDED
        elif msg.data == 'e_stopped':
            self.move_base_status = Result.STATUS_TYPE_PREEMPTED
        elif msg.data == 'e_failure':
            self.move_base_status = Result.STATUS_TYPE_FAILED

    def execute_dbc_motion(self, target_pose):
        self.dbc_pose_pub.publish(target_pose)
        self.dbc_event_in_pub.publish('e_start')
        self.dbc_status = 'active'

    def execute_move_base_motion(self, target_pose):
        self.move_base_pose_pub.publish(target_pose)
        self.move_base_event_in_pub.publish('e_start')
        self.move_base_status = 'active'

    def preempt_dbc_motion(self):
        self.dbc_event_in_pub.publish('e_stop')

    def preempt_move_base_motion(self):
        self.move_base_event_in_pub.publish('e_stop')

    def get_dbc_status(self):
        return self.dbc_status

    def get_move_base_status(self):
        return self.move_base_status

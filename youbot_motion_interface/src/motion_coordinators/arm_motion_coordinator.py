#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from  brics_actuator.msg import JointPositions
from  brics_actuator.msg import JointValue
from youbot_motion_interace import Result

class ArmMotionCoordinator:
    def __init__(self):
        self.moveit_event_in_pub = rospy.Publisher('/moveit_client/event_in', String, queue_size=1)
        self.moveit_configuration_pub = rospy.Publisher('/moveit_client/target_configuration', JointPositions, queue_size=1)
        self.moveit_pose_pub = rospy.Publisher('/moveit_client/target_pose', PoseStamped, queue_size=1)
        self.moveit_pose_name_pub = rospy.Publisher('/moveit_client/target_string_pose', String, queue_size=1)
        self.cvc_event_in_pub = rospy.Publisher('/CVC_node/event_in', String, queue_size=1)
        self.cvc_pose_pub = rospy.Publisher('/CVC_node/target_pose', PoseStamped, queue_size=1)
        rospy.Subscriber('/moveit_client/event_out', String, self.moveit_event_out_cb, queue_size=1)
        rospy.Subscriber('/CVC_node/event_out', String, self.cvc_event_out_cb)

    def moveit_event_out_cb(self, msg):
        if msg.data == 'e_success':
            self.moveit_status = Result.STATUS_TYPE_SUCCEEDED
        elif msg.data == 'e_stopped':
            self.moveit_status = Result.STATUS_TYPE_PREEMPTED
        elif msg.data == 'e_failure':
            self.moveit_status = Result.STATUS_TYPE_FAILED

    def cvc_event_out_cb(self, msg):
        if msg.data == 'e_success':
            self.cvc_status = Result.STATUS_TYPE_SUCCEEDED
        elif msg.data == 'e_stopped':
            self.cvc_status = Result.STATUS_TYPE_PREEMPTED
        elif msg.data == 'e_failure':
            self.cvc_status = Result.STATUS_TYPE_FAILED

    def execute_moveit_pose_name_motion(self, target_pose_name):
        self.moveit_pose_name_pub.publish(target_pose_name)
        self.moveit_event_in_pub.publish('e_start')
        self.moveit_status = 'active'

    def execute_moveit_pose_motion(self, target_pose):
        self.moveit_pose_pub.publish(target_pose)
        self.moveit_event_in_pub.publish('e_start')
        self.moveit_status = 'active'

    def execute_moveit_joint_config_motion(self, target_joint_config):
        target_joint_config_msg = JointPositions()
        for i,joint_val in enumerate(target_joint_config):
            joint = JointValue()
            joint.joint_uri = 'arm_joint_'+str(i+1)
            joint.unit = 'rad'
            joint.value = joint_val
            target_joint_config_msg.positions.append(joint)
        self.moveit_configuration_pub.publish(target_joint_config_msg)
        self.moveit_event_in_pub.publish('e_start')
        self.moveit_status = 'active'

    def execute_cvc_pose_motion(self, target_pose):
        self.cvc_pose_pub.publish(target_pose)
        self.cvc_event_in_pub.publish('e_start')
        self.cvc_status = 'active'

    def preempt_cvc_motion(self):
        self.cvc_event_in_pub.publish('e_stop')

    def get_moveit_status(self):
        return self.moveit_status

    def get_cvc_status(self):
        return self.cvc_status

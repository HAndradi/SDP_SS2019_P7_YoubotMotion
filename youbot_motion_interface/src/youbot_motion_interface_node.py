#! /usr/bin/env python
import rospy
import roslib
import actionlib
import numpy as np
import std_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

from mir_yb_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from youbot_motion_interface.msg import Goal

def base_goal_cb(data):
    print "received new command"
    if data.base_action_type == data.BASE_ACTION_TYPE_DBC_POSE:
        dbc_pose_pub.publish(data.base_goal_pose)
        dbc_event_in_pub.publish('e_start')
    elif data.base_action_type == data.BASE_ACTION_TYPE_LOC_NAME:
        dbc_event_in_pub.publish('e_stop')
        client.cancel_goal()
        move_base_pub.publish(data.base_goal_pose_name)
    print 'done'

def move_base_cb(data):
    client.cancel_goal()
    move_base_goal.destination_location = data.data
    timeout = 60.0
    client.send_goal(move_base_goal)
    client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    client.cancel_goal()
    print "successfully moved the base"

if __name__ == '__main__':
    rospy.init_node('youbot_motion_interface')

    client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
    client.wait_for_server()
    move_base_goal = MoveBaseSafeGoal()
    move_base_goal.arm_safe_position = 'barrier_tape'
    move_base_goal.source_location = 'LOC_1'

    dbc_pose_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/input_pose', geometry_msgs.msg.PoseStamped, queue_size=0)
    dbc_event_in_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/coordinator/event_in', String, queue_size=0)
    move_base_pub = rospy.Publisher('~youbot_motion_coordinator/move_base', String, queue_size=0)

    rospy.Subscriber("~youbot_motion_goal", Goal, base_goal_cb)
    rospy.Subscriber("~youbot_motion_coordinator/move_base", String, move_base_cb)

    rospy.spin()

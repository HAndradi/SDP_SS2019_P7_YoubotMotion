#! /usr/bin/env python
import rospy
import roslib
import actionlib

import numpy as np
import std_msgs.msg
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg

import sys

from std_msgs.msg import String

from mir_yb_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from mir_yb_action_msgs.msg import base_arm

def callback(data):
    print "received new command"
    loc_pub.publish(data.base_loc)	
    pos_pub.publish(data.arm_pos)	
    print 'done'

def loc_callback(data):
    client.cancel_goal()
    goal.source_location = 'LOC_1'
    goal.destination_location = data.data
    timeout = 60.0
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    client.cancel_goal()
    print "successfully moved the base"

def pos_callback(data):
    arm.clear_pose_targets()
    arm.set_start_state_to_current_state()
    try:
	arm.set_named_target(data.data)
    except Exception as e:
	rospy.logerr('unable to set start configuration: %s' % (str(e)))
	return False
    success = arm.go(wait=True)
    if not success:
	rospy.logerr('Arm motion unsuccessful')
	return False
    else:
        rospy.loginfo('Arm motion successful')
	return True


if __name__ == '__main__':
    rospy.init_node('move_base_safe_client_tester')

    arm = moveit_commander.MoveGroupCommander("arm_1")
    robot = moveit_commander.RobotCommander()

    client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
    client.wait_for_server()
    goal = MoveBaseSafeGoal()
    goal.arm_safe_position = 'barrier_tape'

    loc_pub = rospy.Publisher('LOC', String, queue_size=0)
    pos_pub = rospy.Publisher('POS', String, queue_size=0)

    rospy.Subscriber("/LOC_POS", base_arm, callback)
    rospy.Subscriber("/LOC", String, loc_callback)
    rospy.Subscriber("/POS", String, pos_callback)
    rospy.spin()


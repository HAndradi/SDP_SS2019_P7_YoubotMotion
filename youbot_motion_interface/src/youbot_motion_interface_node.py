#! /usr/bin/env python
import rospy
import roslib
import numpy as np
import param_server_utils

import geometry_msgs.msg
from std_msgs.msg import String
from youbot_motion_interface.msg import Goal
from youbot_motion_interface.msg import Acknowledgement

def base_goal_cb(msg):
    global state, new_action, stop_current_action, goal_pose, goal_name, request_node_caller_id
 
    if msg.action_type == msg.ACTION_TYPE_DBC_POSE:
        action = 'DBC_POSE'        
    elif msg.action_type == msg.ACTION_TYPE_MOVE_BASE_POSE:
        action = 'MOVE_BASE_POSE'
    elif msg.action_type == msg.ACTION_TYPE_MOVE_BASE_NAME:
        action = 'MOVE_BASE_NAME'
    else:
        action = 'INVALID_ACTION TYPE'

    if (not state == 'IDLE') and msg.stop_current_action:
        stop_current_action = True   
    else:
        stop_current_action = False 

    if state == 'IDLE' or msg.stop_current_action:
        goal_pose = msg.base_goal_pose
        goal_name = msg.base_goal_pose_name
        request_node_caller_id = msg._connection_header['callerid']
        new_action = action
    else:
        acknowledge('rejected', action, msg._connection_header['callerid'], 'An action is already running and preemption is disabled')


def dbc_event_out_cb(msg):
    global dbc_status
    if msg.data == 'e_success':
        dbc_status = 'succeeded'
    elif msg.data == 'e_stopped':
        dbc_status = 'stopped'
    elif msg.data == 'e_failure':
        dbc_status = 'failed'


def move_base_event_out_cb(msg):
    global move_base_status
    if msg.data == 'e_success':
        move_base_status = 'succeeded'
    elif msg.data == 'e_stopped':
        move_base_status = 'stopped'
    elif msg.data == 'e_failure':
        move_base_status = 'failed'


def acknowledge(status, action_type = 'UNSPECIFIED', request_node = 'unknown', message = ''):
    global state, new_action, request_node_caller_id
    acknowledgement_msg = Acknowledgement()
    if status == 'accepted':
        acknowledgement_msg.requesting_node = request_node_caller_id 
        acknowledgement_msg.action_type = new_action
        acknowledgement_msg.status = status
        acknowledgement_msg.error = message
        state = new_action
    else:      
        acknowledgement_msg.requesting_node = request_node 
        acknowledgement_msg.action_type = action_type
        acknowledgement_msg.status = status
        acknowledgement_msg.error = message
    acknowledgement_pub.publish(acknowledgement_msg)
    new_action = 'UNSPECIFIED'

def states():
    global state, new_action, stop_current_action, goal_pose, goal_name, dbc_status, move_base_status, request_node_caller_id
    state = 'IDLE'
    new_action = 'UNSPECIFIED'
    print 'READY!'
    while True:
        if state == 'IDLE':
            if new_action == 'DBC_POSE':
                acknowledge('accepted')
                dbc_pose_pub.publish(goal_pose)
                dbc_event_in_pub.publish('e_start')
                dbc_status = 'waiting'
            elif new_action == 'MOVE_BASE_POSE' or new_action == 'MOVE_BASE_NAME':
                if new_action == 'MOVE_BASE_NAME':
                    goal_pose = param_server_utils.get_pose_from_param_server(goal_name)
                if goal_pose is not None:
                    acknowledge('accepted')
                    move_base_pose_pub.publish(goal_pose)
                    move_base_event_in_pub.publish('e_start')
                    move_base_status = 'waiting'
                else:
                    acknowledge('rejected', new_action, request_node_caller_id, 'invalid goal pose')

        elif state == 'MOVE_BASE_POSE' or state == 'MOVE_BASE_NAME':
            if not move_base_status == 'waiting':
                state = 'IDLE'
                print 'move base outcome: ' + move_base_status
            if stop_current_action == True:
                stop_current_action = False
                move_base_event_in_pub.publish('e_stop')
                
        elif state == 'DBC_POSE':
            if not dbc_status == 'waiting':
                state = 'IDLE'
                print 'dbc outcome: ' + dbc_status
            if stop_current_action == True:
                stop_current_action = False
                dbc_event_in_pub.publish('e_stop')
        rospy.sleep(0.1)
                

if __name__ == '__main__':
    rospy.init_node('youbot_motion_interface')

    dbc_pose_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/input_pose', geometry_msgs.msg.PoseStamped, queue_size=0)
    dbc_event_in_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/coordinator/event_in', String, queue_size=0)
    move_base_pose_pub = rospy.Publisher('/move_base_wrapper/pose_in', geometry_msgs.msg.PoseStamped, queue_size=0)
    move_base_event_in_pub = rospy.Publisher('/move_base_wrapper/event_in', String, queue_size=0)

    rospy.Subscriber('/mcr_navigation/direct_base_controller/coordinator/event_out', String, dbc_event_out_cb)
    rospy.Subscriber('/move_base_wrapper/event_out', String, move_base_event_out_cb)

    rospy.Subscriber('~youbot_motion_goal', Goal, base_goal_cb, queue_size=1)
    acknowledgement_pub = rospy.Publisher('~youbot_motion_acknowledgement', Acknowledgement, queue_size=0)

    states()

    rospy.spin()

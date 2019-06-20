#!/usr/bin/env python 

import rospy
from brics_actuator.msg import JointPositions, JointValue

if __name__ == '__main__':

    rospy.init_node('send_command_to_joints')

    # arm_goal_joints_pub = rospy.Publisher("/moveit_client/target_configuration", JointPositions, queue_size=1)
    arm_goal_joints_pub = rospy.Publisher("/arm_1/arm_controller/position_command", JointPositions, queue_size=1)

    command = JointPositions()
    # joint_values = [2.16419484686, 1.1344673702, -2.54819292522, 1.78896002508, 2.93066346796]
    # joint_values = [0.0,0.0,0.0,0.0,0.0]
    joint_values = [0.05, 1.1344673702, -2.54819292522, 1.78896002508, 2.93066346796]

    for n,i in enumerate(joint_values):
        joint = JointValue()
        joint.joint_uri = 'arm_joint_'+str(n+1)
        joint.unit = 'rad'
        joint.value = i
        command.positions.append(joint)

    rospy.sleep(1)
    arm_goal_joints_pub.publish(command)
    rospy.loginfo("Goal configuration sent")



#!/usr/bin/env python 

import numpy as np
import tf
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PoseStamped

class VelocityCommander:
    def __init__(self):

        '''
        INITIALIZERS
        '''
        self.tf_listener = tf.TransformListener()
        self.target_pose = None
        self.event_in = None

        '''
        PARAMETERS
        '''
        self.max_velocity = 0.1
        self.feedback_gain = 3
        self.goal_tolerance = 0.01
        self.loop_rate = rospy.Rate(10)

        '''
        PUBLISHERS
        '''
        self.event_out = rospy.Publisher('~event_out', String, queue_size=1)
        self.velocity_pub = rospy.Publisher('/arm_1/arm_controller/cartesian_velocity_command', TwistStamped, queue_size=1)

        '''
        SUBSCRIBERS
        '''
        rospy.Subscriber('~target_pose', PoseStamped, self.target_pose_cb)
        rospy.Subscriber('~event_in', String, self.event_in_cb)

    def reset_variables(self):
        self.event_in = None
        self.target_pose = None
        rospy.loginfo('Done')
    
    def check_goal_request(self):
        while not rospy.is_shutdown():
            if self.target_pose is not None and self.event_in == 'e_start':
                self.move_to_pose()
                self.reset_variables()
            self.loop_rate.sleep()

    def event_in_cb(self, msg):
        self.event_in = msg.data
        rospy.loginfo('event_in obtained!')
    
    def target_pose_cb(self, msg):
        if self.target_pose is None:
            self.target_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        rospy.loginfo('target_pose obtained!')

    def get_current_pose(self):
        while True:
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
                return trans,rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def move_to_pose(self):

        count = 0

        trans,rot = self.get_current_pose()
        current_pos = np.array([trans[0], trans[1], trans[2]])
        old_pos = current_pos.copy()
        
        distance_trans = np.linalg.norm(self.target_pose - current_pos)

        while True:

            if self.event_in == 'e_stop':
                self.event_out.publish('e_stopped')
                break

            if count % 1000 == 0 and count > 0:

                if np.allclose(current_pos,old_pos):
                    self.event_out.publish('e_failure')
                    break
                else:
                    old_pos = current_pos.copy()

            if distance_trans < self.goal_tolerance:
                self.event_out.publish('e_success')
                break

            vel_x = self.feedback_gain * (self.target_pose[0] - current_pos[0])
            vel_y = self.feedback_gain * (self.target_pose[1] - current_pos[1])
            vel_z = self.feedback_gain * (self.target_pose[2] - current_pos[2])

            vel_norm = (np.sqrt(vel_x**2 + vel_y**2 + vel_z**2))

            if vel_norm > self.max_velocity:
                vel_x *= self.max_velocity / vel_norm
                vel_y *= self.max_velocity / vel_norm
                vel_z *= self.max_velocity / vel_norm

            message = TwistStamped()
            message.header.seq = count
            message.header.frame_id = "/base_link"
            message.twist.linear.x = vel_x
            message.twist.linear.y = vel_y
            message.twist.linear.z = vel_z

            self.velocity_pub.publish(message)

            trans,rot = self.get_current_pose()
            current_pos = np.array([trans[0], trans[1], trans[2]])
            distance_trans = np.linalg.norm(self.target_pose - current_pos)
            count += 1

        message = TwistStamped()
        message.header.seq = count
        message.header.frame_id = "/base_link"
        message.twist.linear.x = 0.0
        message.twist.linear.y = 0.0
        message.twist.linear.z = 0.0

        self.velocity_pub.publish(message)

if __name__ == '__main__':

    rospy.init_node("CVC_node")
    rospy.loginfo("CVC node initiated")
    velocity_commander = VelocityCommander()
    velocity_commander.check_goal_request()
    

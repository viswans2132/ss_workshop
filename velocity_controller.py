#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class VelocityController:
    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.des_position = np.array([0.0, 0.0, 0.0])
        self.des_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.error = 0.0
        self.Kpos = np.array([-0.8, -0.8, -1.5])
        self.Korient = -0.3

        self.odom_sub = rospy.Subscriber("/odometry", Odometry, self.callback_odometry)
        self.setpoint_sub = rospy.Subscriber(
            "/setpoint_position", Odometry, self.callback_setpoint
        )
        self.cmd_vel_pub = rospy.Publisher(
            "/mavros/setpoint_attitude/cmd_vel", TwistStamped, queue_size=10
        )

        while not rospy.is_shutdown():
            self.vel_sp()

    def callback_odometry(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w

    def callback_setpoint(self, msg):
        self.des_position[0] = msg.pose.pose.position.x
        self.des_position[1] = msg.pose.pose.position.y
        self.des_position[2] = msg.pose.pose.position.z
        self.des_orientation[0] = msg.pose.pose.orientation.x
        self.des_orientation[1] = msg.pose.pose.orientation.y
        self.des_orientation[2] = msg.pose.pose.orientation.z
        self.des_orientation[3] = msg.pose.pose.orientation.w

    def vector2Arrays(self, vector):
        return np.array([vector.x, vector.y, vector.z])

    def vel_sp(self):
        cur_pos = self.vector2Arrays(self.position)
        des_pos = self.vector2Arrays(self.des_position)
        # cur_orient = self.vector2Arrays(self.orientation)
        # des_orient = self.vector2Arrays(self.des_orientation)
        self.error_pos = cur_pos - des_pos
        # self.error_orient = cur_orient - des_orient

        des_vel = self.Kpos * self.error
        # desYawVel = self.Korient*self.error_orient[2]

        vel_sp = TwistStamped()
        vel_sp.twist.linear.x = des_vel[0]
        vel_sp.twist.linear.y = des_vel[1]
        vel_sp.twist.linear.z = des_vel[2]
        # vel_sp.twist.angular.z = desYawVel
        vel_sp.header.stamp = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node("random_delay_node", anonymous=True)
    node = VelocityController()
    rospy.spin()

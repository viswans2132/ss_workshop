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
        self.error_pos = np.array([0.0, 0, 0])
        self.Kpos = np.array([-0.8, -0.8, -1.5])
        self.Korient = -0.3

        self.odom_sub = rospy.Subscriber(
            "/estimated_odometry", Odometry, self.callback_odometry
        )
        self.setpoint_sub = rospy.Subscriber(
            "/setpoint_position", Odometry, self.callback_setpoint
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=10)

    def callback_odometry(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w
        self.vel_sp()

    def callback_setpoint(self, msg):
        self.des_position[0] = msg.pose.pose.position.x
        self.des_position[1] = msg.pose.pose.position.y
        self.des_position[2] = msg.pose.pose.position.z
        self.des_orientation[0] = msg.pose.pose.orientation.x
        self.des_orientation[1] = msg.pose.pose.orientation.y
        self.des_orientation[2] = msg.pose.pose.orientation.z
        self.des_orientation[3] = msg.pose.pose.orientation.w

    def vel_sp(self):
        self.error_pos = self.position - self.des_position
        # self.error_orient = self.orientation - self.des_orientation

        des_vel = self.Kpos * self.error_pos
        # print(self.position, self.des_position, des_vel)
        # desYawVel = self.Korient*self.error_orient[2]

        vel_sp = TwistStamped()
        vel_sp.twist.linear.x = des_vel[0]
        vel_sp.twist.linear.y = des_vel[1]
        vel_sp.twist.linear.z = des_vel[2]
        # vel_sp.twist.angular.z = desYawVel
        vel_sp.header.stamp = rospy.Time.now()
        self.cmd_vel_pub.publish(vel_sp)


if __name__ == "__main__":
    rospy.init_node("velocity_sp_node", anonymous=True)
    node = VelocityController()
    rospy.spin()

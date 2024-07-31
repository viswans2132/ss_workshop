#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class PositionPredictor:
    def __init__(self):
        self.delay = 0.0
        self.dt = 0.0
        self.counter = 0
        self.cmd_vel = np.array([0.0, 0.0, 0.0])

        self.odom_sub = rospy.Subscriber("/odometry", Odometry, self.callback_odometry)
        self.cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel", TwistStamped, self.callback_cmd_vel
        )
        self.est_odom_pub = rospy.Publisher(
            "/estimated_odometry", Odometry, queue_size=10
        )

    def callback_odometry(self, msg):
        self.estimated_odometry = Odometry()
        recv_time = rospy.Time.now()
        pub_time = msg.header.stamp
        self.delay = (recv_time - pub_time).to_sec()
        if self.counter < 500:  # sliding window of ~5s
            self.counter = self.counter + 1
        self.dt = np.divide(self.delay + (self.counter - 1) * self.dt, self.counter)
        if self.dt > 0.05:
            self.dt = 0.05
        # self.dt = 0.0
        self.estimated_odometry = msg
        self.estimated_odometry.pose.pose.position.x = (
            msg.pose.pose.position.x + self.cmd_vel[0] * self.dt
        )
        self.estimated_odometry.pose.pose.position.y = (
            msg.pose.pose.position.y + self.cmd_vel[1] * self.dt
        )
        self.estimated_odometry.pose.pose.position.z = (
            msg.pose.pose.position.z + self.cmd_vel[2] * self.dt
        )
        # self.estimated_odometry.twist.twist.linear.x = (
        #     msg.twist.twist.linear.x + self.cmd_vel[0] * self.dt
        # )
        # self.estimated_odometry.twist.twist.linear.y = (
        #     msg.twist.twist.linear.y + self.cmd_vel[1] * self.dt
        # )
        # self.estimated_odometry.twist.twist.linear.z = (
        #     msg.twist.twist.linear.z
        #     + (self.cmd_vel[2] - 9.81) * self.dt  # what is the equilibrium cmd_vel
        # )
        # print("counter:", self.counter, "av.delay:", self.dt, "new.delay:", self.delay)
        self.estimated_odometry.header.stamp = rospy.Time.now()
        self.est_odom_pub.publish(self.estimated_odometry)

    def callback_cmd_vel(self, msg):
        self.cmd_vel[0] = msg.twist.linear.x
        self.cmd_vel[1] = msg.twist.linear.y
        self.cmd_vel[2] = msg.twist.linear.z


if __name__ == "__main__":
    rospy.init_node("random_delay_node", anonymous=True)
    node = PositionPredictor()
    rospy.spin()

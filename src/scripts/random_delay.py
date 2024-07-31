#!/usr/bin/env python

import rospy
import random
import functools
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class RandomDelayNode:
    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            "/pelican/odometry_sensor1/odometry", Odometry, self.callback_odometry
        )
        self.cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel", TwistStamped, self.callback_cmd_vel
        )
        self.odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(
            "/pelican/vel_msg", TwistStamped, queue_size=10
        )

    def callback_odometry(self, msg):
        delay = random.uniform(0.04, 0.041)  # Random delay between 0.01 and 0.05 seconds
        delay_secs = int(delay)
        delay_nsecs = int((delay - delay_secs) * 1e9)
        self.odom_pub.publish(msg)

        # msg.header.stamp.nsecs -= delay_nsecs
        # if msg.header.stamp.nsecs < 0:
        #     msg.header.stamp.nsecs += int(1e9)
        #     msg.header.stamp.secs -= 1
        # msg.header.stamp.secs -= delay_secs


        # timer = rospy.Timer(rospy.Duration(delay),
        #                 functools.partial(self.delayed_odom_callback, msg),
        #                 oneshot=True)

    def delayed_odom_callback(self, msg, event):
        self.odom_pub.publish(msg)

    def callback_cmd_vel(self, msg):
        delay = random.uniform(0.04, 0.041)  # Random delay between 0.01 and 0.05 seconds
        delay_secs = int(delay)
        delay_nsecs = int((delay - delay_secs) * 1e9)
        self.cmd_vel_pub.publish(msg)

        # msg.header.stamp.nsecs -= delay_nsecs
        # if msg.header.stamp.nsecs < 0:
        #     msg.header.stamp.nsecs += int(1e9)
        #     msg.header.stamp.secs -= 1
        # msg.header.stamp.secs -= delay_secs

        # timer = rospy.Timer(rospy.Duration(delay),
        #                 functools.partial(self.delayed_cmd_callback, msg),
        #                 oneshot=True)

    def delayed_cmd_callback(self, msg, event):
        self.cmd_vel_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("random_delay_node", anonymous=True)
    node = RandomDelayNode()
    rospy.spin()

#!/usr/bin/env python

import rospy
import random
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class RandomDelayNode:
    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            "/hummingbird/ground_truth/odometry", Odometry, self.callback_odometry
        )
        self.cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel", TwistStamped, self.callback_cmd_vel
        )
        self.odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(
            "/mavros/setpoint_attitude/cmd_vel", TwistStamped, queue_size=10
        )

    def callback_odometry(self, msg):
        delay = random.uniform(0.1, 2.0)  # Random delay between 0.1 and 2.0 seconds
        # rospy.loginfo(f"Received message: {msg.data}, delaying for {delay:.2f} seconds")
        time.sleep(delay)
        self.odom_pub.publish(msg)
        rospy.loginfo(f"Published message: {msg.data} after delay")

    def callback_cmd_vel(self, msg):
        delay = random.uniform(0.1, 2.0)  # Random delay between 0.1 and 2.0 seconds
        # rospy.loginfo(f"Received message: {msg.data}, delaying for {delay:.2f} seconds")
        time.sleep(delay)
        self.cmd_vel_pub.publish(msg)
        rospy.loginfo(f"Published message: {msg.data} after delay")


if __name__ == "__main__":
    rospy.init_node("random_delay_node", anonymous=True)
    node = RandomDelayNode()
    rospy.spin()

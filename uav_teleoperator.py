#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from pynput import keyboard
from tf.transformations import *


class DroneTeleoperator:
    def __init__(self):
        rospy.init_node("drone_teleop", anonymous=True)
        self.pub = rospy.Publisher("/setpoint_position", Odometry, queue_size=10)
        self.odom = Odometry()
        self.odom.header.frame_id = "base_link"

        # Set initial positions
        self.odom.pose.pose.position.x = 0.0
        self.odom.pose.pose.position.y = 0.0
        self.odom.pose.pose.position.z = 0.0
        self.yaw_angle = 0.0
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = 0.0
        self.odom.pose.pose.orientation.w = 1.0
        self.rate = rospy.Rate(10)  # 10 Hz

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:  # Forward
                self.odom.pose.pose.position.x += 0.1
            elif key == keyboard.Key.down:  # Backward
                self.odom.pose.pose.position.x -= 0.1
            elif key == keyboard.Key.left:  # Left
                self.odom.pose.pose.position.y += 0.1
            elif key == keyboard.Key.right:  # Right
                self.odom.pose.pose.position.y -= 0.1
            elif key.char == "w":  # Up
                self.odom.pose.pose.position.z += 0.1
            elif key.char == "s":  # Down
                self.odom.pose.pose.position.z -= 0.1
            elif key.char == "a":  # Yaw Left
                self.yaw_angle += 0.1
                if self.yaw_angle >= np.pi:
                    self.yaw_angle = self.yaw_angle - np.pi
            elif key.char == "d":  # Yaw Right
                self.yaw_angle -= 0.1
                if self.yaw_angle <= -np.pi:
                    self.yaw_angle = self.yaw_angle + np.pi
            print("alive")
            rospy.loginfo(f"yaw angle: {self.yaw_angle}")
            quat = quaternion_from_euler(0.0, 0.0, self.yaw_angle)
            self.odom.pose.pose.orientation.x = quat[0]
            self.odom.pose.pose.orientation.y = quat[1]
            self.odom.pose.pose.orientation.z = quat[2]
            self.odom.pose.pose.orientation.w = quat[3]
        except AttributeError:
            pass

    def run(self):
        while not rospy.is_shutdown():
            self.odom.header.stamp = rospy.Time.now()
            self.pub.publish(self.odom)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        controller = DroneTeleoperator()
        controller.run()
    except rospy.ROSInterruptException:
        pass

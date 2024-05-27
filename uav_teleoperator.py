#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from pynput import keyboard


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
        self.odom.pose.pose.orientation.w = 1.0
        self.rate = rospy.Rate(10)  # 10 Hz

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:  # Forward
                self.odom.pose.pose.position.x += 1.0
            elif key == keyboard.Key.down:  # Backward
                self.odom.pose.pose.position.x -= 1.0
            elif key == keyboard.Key.left:  # Left
                self.odom.pose.pose.position.y += 1.0
            elif key == keyboard.Key.right:  # Right
                self.odom.pose.pose.position.y -= 1.0
            elif key.char == "w":  # Up
                self.odom.pose.pose.position.z += 1.0
            elif key.char == "s":  # Down
                self.odom.pose.pose.position.z -= 1.0
            elif key.char == "a":  # Yaw Left
                self.odom.pose.pose.orientation.z += 0.1
            elif key.char == "d":  # Yaw Right
                self.odom.pose.pose.orientation.z -= 0.1
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

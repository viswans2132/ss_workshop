#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import argparse
import sys

class KeyboardTeleop:
    def __init__(self, namespace=''):
        rospy.init_node('keyboard_teleop', anonymous=True)
        rospy.loginfo("KeyboardTeleop node initialized with namespace: '{}'".format(namespace))

        if namespace and not namespace.startswith('/'):
            namespace = '/' + namespace
        self.namespace = namespace

        self.publisher_ = rospy.Publisher(self.namespace + '/cmd_vel', Twist, queue_size=10)
        self.linear_speed = [0.0, 0.0, 0.0]

        rospy.loginfo("Use arrow keys to move x/y, W/S for z. Press ESC to exit.")

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        self.rate = rospy.Rate(20)

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.linear_speed[2] += 0.1
            elif key.char == 's':
                self.linear_speed[2] -= 0.1
        except AttributeError:
            if key == keyboard.Key.up:
                self.linear_speed[0] += 0.1
            elif key == keyboard.Key.down:
                self.linear_speed[0] -= 0.1
            elif key == keyboard.Key.right:
                self.linear_speed[1] -= 0.1
            elif key == keyboard.Key.left:
                self.linear_speed[1] += 0.1
            elif key == keyboard.Key.esc:
                rospy.loginfo("Exiting teleop...")
                rospy.signal_shutdown("ESC pressed")

        rospy.loginfo("Velocities: x=%.2f, y=%.2f, z=%.2f" % tuple(self.linear_speed))

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.linear_speed[0]
        msg.linear.y = self.linear_speed[1]
        msg.linear.z = self.linear_speed[2]
        self.publisher_.publish(msg)

    def spin(self):
        while not rospy.is_shutdown():
            self.publish_twist()
            self.rate.sleep()

def main():
    parser = argparse.ArgumentParser(description="Keyboard teleoperation node.")
    parser.add_argument('--namespace', type=str, default='',
                        help='Robot namespace prefix (e.g., /spot)')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    teleop = KeyboardTeleop(namespace=args.namespace)
    teleop.spin()

if __name__ == '__main__':
    main()

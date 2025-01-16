#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf.transformations import *


class VelocityController:
    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, -0.707, 0.707])
        self.des_position = np.array([0.0, 0.0, 0.5])
        self.des_orientation = np.array([0.0, 0.0, 0.0, 0.0])
        self.error_pos = np.array([0.0, 0, 0])
        self.Kpos = np.array([-2.8, -2.8, -1.5])
        self.Korient = -1.3
        self.R = np.eye(3)

        self.yaw = 0.0
        self.des_yaw = -1.57

        self.odom_sub = rospy.Subscriber(
            "/pelican/odometry_sensor1/odometry", Odometry, self.callback_odometry
        )
        self.setpoint_sub = rospy.Subscriber(
            "/setpoint_position", Odometry, self.callback_setpoint
        )
        self.setpose_sub = rospy.Subscriber(
            "/new_pose", PoseStamped, self.callback_setpose)
        self.cmd_vel_pub = rospy.Publisher("/pelican/vel_msg", TwistStamped, queue_size=10)

    def callback_odometry(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w

        # q = self.orient, self.cur_pose.pose.orientation.y, self.cur_pose.pose.orientation.z, self.cur_pose.pose.orientation.w]

        euler_angles = euler_from_quaternion(self.orientation)
        self.yaw = euler_angles[2]

        self.vel_sp()

    def callback_setpoint(self, msg):
        self.des_position[0] = msg.pose.pose.position.x
        self.des_position[1] = msg.pose.pose.position.y
        self.des_position[2] = msg.pose.pose.position.z
        self.des_orientation[0] = msg.pose.pose.orientation.x
        self.des_orientation[1] = msg.pose.pose.orientation.y
        self.des_orientation[2] = msg.pose.pose.orientation.z
        self.des_orientation[3] = msg.pose.pose.orientation.w
        euler_angles = euler_from_quaternion(self.des_orientation)
        self.des_yaw = euler_angles[2]

    def callback_setpose(self, msg):
        self.des_position[0] = msg.pose.position.x
        self.des_position[1] = msg.pose.position.y
        self.des_position[2] = msg.pose.position.z
        self.des_orientation[0] = msg.pose.orientation.x
        self.des_orientation[1] = msg.pose.orientation.y
        self.des_orientation[2] = msg.pose.orientation.z
        self.des_orientation[3] = msg.pose.orientation.w
        euler_angles = euler_from_quaternion(self.des_orientation)
        self.des_yaw = euler_angles[2]

    def vel_sp(self):
        self.error_pos = self.position - self.des_position
        # self.error_orient = self.orientation - self.des_orientation

        print(["Err: ", self.error_pos])

        des_vel = self.Kpos * self.error_pos
        # print(["Vel: ", des_vel])




        if np.linalg.norm(des_vel) > 0.2:
            des_vel = 0.2*des_vel/np.linalg.norm(des_vel)
        # print(des_vel)

        errYaw = self.yaw - self.des_yaw

        if np.abs(errYaw) > np.pi:
            errYaw = np.sign(errYaw)*(np.abs(errYaw) - 2*np.pi)
        print('{:.2f}'.format(errYaw))
        
        desYawVel = -2.3*errYaw

        desYawVel = np.minimum(0.6, np.maximum(-0.6, desYawVel))
        # print(desYawVel)


        # print(self.position, self.des_position, des_vel)
        # desYawVel = self.Korient*self.error_orient[2]

        vel_sp = TwistStamped()
        vel_sp.twist.linear.x = des_vel[0]
        vel_sp.twist.linear.y = des_vel[1]
        vel_sp.twist.linear.z = des_vel[2]
        vel_sp.twist.angular.z = desYawVel
        vel_sp.header.stamp = rospy.Time.now()
        self.cmd_vel_pub.publish(vel_sp)


if __name__ == "__main__":
    rospy.init_node("velocity_sp_node", anonymous=True)
    node = VelocityController()
    rospy.spin()

#!/usr/bin/env python

import rospy
import numpy as np
import cvxpy as cp
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class VelocityController:
    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.des_position = np.array([0.0, 0.0, 0.2])
        self.des_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.error_pos = np.array([0.0, 0, 0])
        self.Kpos = np.array([-2.8, -2.8, -1.5])
        self.Korient = -0.3

        self.odom_sub = rospy.Subscriber(
            "/pelican/odometry_sensor1/odometry", Odometry, self.callback_odometry
        )
        self.setpoint_sub = rospy.Subscriber(
            "/setpoint_position", Odometry, self.callback_setpoint
        )
        self.cmd_vel_pub = rospy.Publisher("/pelican/vel_msg", TwistStamped, queue_size=10)

    def callback_odometry(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w
        self.get_vel_sp()

    def callback_setpoint(self, msg):
        self.des_position[0] = msg.pose.pose.position.x
        self.des_position[1] = msg.pose.pose.position.y
        self.des_position[2] = msg.pose.pose.position.z
        self.des_orientation[0] = msg.pose.pose.orientation.x
        self.des_orientation[1] = msg.pose.pose.orientation.y
        self.des_orientation[2] = msg.pose.pose.orientation.z
        self.des_orientation[3] = msg.pose.pose.orientation.w


    # Not used currently
    def safety_filter(self, desVel):        
        P = np.eye(3)
        u = cp.Variable(3)

        # a = 4.0

        # l = errPos[1]*errPos[1] + errPos[2]*errPos[2]
        # l = np.maximum(l, 0.001)

        h = 1.0 - self.position[0]

        dhdx = -1
        dhdy = 0
        dhdz = 0
        dhdp = np.array([[dhdx, dhdy, dhdz]])

        # constraints = [dhdp@u >= -0.4*h]
        # prob = cp.Problem(cp.Minimize(cp.quad_form(u-desVel, P)), constraints)
        # result = prob.solve()
        # val = u.value

        psi = dhdp@desVel + 0.5*h
        u_safe = np.zeros(desVel.shape)
        if psi < 0:
            u_safe = -(dhdp.T/(dhdp@dhdp.T))*(psi)
        # print(f"{u_safe}")

        val = desVel + u_safe.reshape(desVel.shape)




        if np.linalg.norm(val) > 0.5:
            val = 0.5*val/np.linalg.norm(val)

        return val



    def get_vel_sp(self):
        # print(self.position)
        self.error_pos = self.position - self.des_position
        # print(self.error_pos)
        # self.error_orient = self.orientation - self.des_orientation

        des_vel = self.Kpos * self.error_pos
        # print(des_vel)
        des_vel = self.safety_filter(des_vel)
        # print(des_vel)
        # print('')

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
    rospy.init_node("wall_cbf_node", anonymous=True)
    node = VelocityController()
    rospy.spin()

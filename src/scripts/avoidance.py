#!/usr/bin/env python

import rospy
import numpy as np
import cvxpy as cp
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class VelocityController:
    def __init__(self):
        self.position = np.array([-1.0, -1.0, 0.5])
        self.position_2 = np.array([0.0, 0.0, 0.5])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.orientation_2 = np.array([0.0, 0.0, 0.0, 1.0])
        self.des_position = np.array([-1.0, -1.0, 1.0])
        self.des_position_2 = np.array([0.0, 0.0, 1.0])
        self.des_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.error_pos = np.array([0.0, 0, 0])
        self.Kpos = np.array([-2.8, -2.8, -1.5])
        self.Korient = -0.3

        self.odom_sub = rospy.Subscriber(
            "/pelican/odometry_sensor1/odometry", Odometry, self.callback_odometry_1
        )
        self.odom_sub_2 = rospy.Subscriber(
            "/pelican_2/odometry_sensor1/odometry", Odometry, self.callback_odometry_2
        )
        self.setpoint_sub = rospy.Subscriber(
            "/setpoint_position", Odometry, self.callback_setpoint
        )
        self.cmd_vel_pub = rospy.Publisher("/pelican/vel_msg", TwistStamped, queue_size=1)
        self.cmd_vel_pub_2 = rospy.Publisher("/pelican_2/vel_msg", TwistStamped, queue_size=1)
        self.vel_pub = rospy.Publisher("/pelican/cmd_vel", TwistStamped, queue_size=1)
        self.vel_pub_2 = rospy.Publisher("/pelican_2/cmd_vel", TwistStamped, queue_size=1)

    def callback_odometry_1(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w
        self.vel_sp()


    def callback_odometry_2(self, msg):
        self.position_2[0] = msg.pose.pose.position.x
        self.position_2[1] = msg.pose.pose.position.y
        self.position_2[2] = msg.pose.pose.position.z
        self.orientation_2[0] = msg.pose.pose.orientation.x
        self.orientation_2[1] = msg.pose.pose.orientation.y
        self.orientation_2[2] = msg.pose.pose.orientation.z
        self.orientation_2[3] = msg.pose.pose.orientation.w

    def callback_setpoint(self, msg):
        self.des_position[0] = msg.pose.pose.position.x
        self.des_position[1] = msg.pose.pose.position.y
        self.des_position[2] = msg.pose.pose.position.z
        self.des_orientation[0] = msg.pose.pose.orientation.x
        self.des_orientation[1] = msg.pose.pose.orientation.y
        self.des_orientation[2] = msg.pose.pose.orientation.z
        self.des_orientation[3] = msg.pose.pose.orientation.w


    # Not used currently
    def safety_filter(self, desVel_1, desVel_2):        
        P = np.eye(6)
        u = cp.Variable(6)

        desVel = np.append(desVel_1, desVel_2)

        # a = 4.0

        # l = errPos[1]*errPos[1] + errPos[2]*errPos[2]
        # l = np.maximum(l, 0.001)

        h = ((self.position[0] - self.position_2[0])**2 + (self.position[1] - self.position_2[1])**2 +
            (self.position[2] - self.position_2[2])**2 - 1.0)

        dhdx = 2*(self.position[0] - self.position_2[0])
        dhdy = 2*(self.position[1] - self.position_2[1])
        dhdz = 2*(self.position[2] - self.position_2[2])
        dhdx_2 = -2*(self.position[0] - self.position_2[0])
        dhdy_2 = -2*(self.position[1] - self.position_2[1])
        dhdz_2 = -2*(self.position[2] - self.position_2[2])
        dhdp = np.array([dhdx, dhdy, dhdz, dhdx_2, dhdy_2, dhdz_2])

        constraints = [dhdp@u >= -1.4*h]
        prob = cp.Problem(cp.Minimize(cp.quad_form(u-desVel, P)), constraints)
        result = prob.solve()


        val = u.value

        if np.linalg.norm(val) > 0.5:
            val = 0.5*val/np.linalg.norm(val)
        return val

    def vel_sp(self):
        self.error_pos = self.position - self.des_position
        self.error_pos_2 = self.position_2 - self.des_position_2
        # self.error_orient = self.orientation - self.des_orientation

        des_vel_1 = self.Kpos * self.error_pos
        des_vel_2 = self.Kpos * self.error_pos_2
        des_vel = self.safety_filter(des_vel_1, des_vel_2)
        # des_vel
        # print(self.position, self.des_position, des_vel)
        # desYawVel = self.Korient*self.error_orient[2]

        vel_sp = TwistStamped()
        vel_sp.twist.linear.x = des_vel[0]
        vel_sp.twist.linear.y = des_vel[1]
        vel_sp.twist.linear.z = des_vel[2]
        # vel_sp.twist.angular.z = desYawVel
        vel_sp.header.stamp = rospy.Time.now()
        self.cmd_vel_pub.publish(vel_sp)

        vel_sp = TwistStamped()
        vel_sp.twist.linear.x = des_vel[3]
        vel_sp.twist.linear.y = des_vel[4]
        vel_sp.twist.linear.z = des_vel[5]
        # vel_sp.twist.angular.z = desYawVel
        vel_sp.header.stamp = rospy.Time.now()
        self.cmd_vel_pub_2.publish(vel_sp)

        vel_sp = TwistStamped()
        vel_sp.twist.linear.x = des_vel_1[0]
        vel_sp.twist.linear.y = des_vel_1[1]
        vel_sp.twist.linear.z = des_vel_1[2]
        # vel_sp.twist.angular.z = desYawVel
        vel_sp.header.stamp = rospy.Time.now()
        self.vel_pub.publish(vel_sp)

        vel_sp = TwistStamped()
        vel_sp.twist.linear.x = des_vel_2[0]
        vel_sp.twist.linear.y = des_vel_2[1]
        vel_sp.twist.linear.z = des_vel_2[2]
        # vel_sp.twist.angular.z = desYawVel
        vel_sp.header.stamp = rospy.Time.now()
        self.vel_pub_2.publish(vel_sp)


if __name__ == "__main__":
    rospy.init_node("avoidance_node", anonymous=True)
    node = VelocityController()
    rospy.spin()

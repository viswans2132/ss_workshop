#!/usr/bin/env python

import rospy
import numpy as np
import cvxpy as cp
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker, MarkerArray


class VelocityController:
    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.des_position = np.array([0.0, 0.0, 1.2])
        self.des_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.error_pos = np.array([0.0, 0, 0])
        self.Kpos = np.array([-2.8, -2.8, -1.5])
        self.Korient = -0.3

        self.ellipses = []
        self.constraintsReceived = False
        self.cons = np.array([])

        self.odom_sub = rospy.Subscriber(
            "/shafter4/odometry_sensor1/odometry", Odometry, self.callback_odometry
        )
        self.setpoint_sub = rospy.Subscriber(
            "/setpoint_position", Odometry, self.callback_setpoint
        )
        self.ellipse_sub = rospy.Subscriber(
            "/marker/ellipses", MarkerArray, self.callback_constraints
            )
        self.cmd_vel_pub = rospy.Publisher("/shafter4/vel_msg", TwistStamped, queue_size=10)

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

    def callback_constraints(self, msg):
        markers = msg.markers
        self.ellipses = []
        no_obs = len(markers)
        if no_obs > 0:
            self.ellipses = np.zeros((no_obs, 5))
            for i, marker in enumerate(markers):
                self.ellipses[i,0] = marker.pose.position.x
                self.ellipses[i,1] = marker.pose.position.y
                self.ellipses[i,2] = (marker.scale.x+0.6)*(marker.scale.x+0.6)/4
                self.ellipses[i,3] = (marker.scale.y+0.6)*(marker.scale.y+0.6)/4

                q = [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]

                self.ellipses[i,4] = euler_from_quaternion(q)[2]
                self.ellipses[i,4] = 0.0
            self.genConsMatrix()




    def genConsMatrix(self):
        if self.ellipses.size > 0:
            # self.ellipses = self.ellipses[0,:].reshape((1,-1))
            self.cons = np.zeros((len(self.ellipses), 3))
            self.constraintsReceived = True
            for i in range(len(self.ellipses)):
                errPos = self.position[:2] -  self.ellipses[i,:2]
                x_term = errPos[0]
                y_term = errPos[1]
                print(f'ErrPos: {errPos[0]}')
                x_term = errPos[0]*np.cos(self.ellipses[i,4]) + errPos[1]*np.sin(self.ellipses[i,4])
                y_term = -errPos[0]*np.sin(self.ellipses[i,4]) + errPos[1]*np.cos(self.ellipses[i,4])
                h = (x_term*x_term/self.ellipses[i,2]) + (y_term*y_term/self.ellipses[i,3]) - 1.0
                print(f'H: {h}')
                dhdx = 2*(x_term*np.cos(self.ellipses[i,4])/self.ellipses[i,2] - y_term*np.sin(self.ellipses[i,4])/self.ellipses[i,3])
                dhdy = 2*(x_term*np.sin(self.ellipses[i,4])/self.ellipses[i,2] + y_term*np.cos(self.ellipses[i,4])/self.ellipses[i,3])
                # dhdx = x_term/self.ellipses[i,2]
                # dhdy = y_term/self.ellipses[i,3]

                self.cons[i, 0] = dhdx
                self.cons[i, 1] = dhdy
                self.cons[i, 2] = -1.1*h


        else:
            self.constraintsReceived = False



    # Not used currently
    def safety_filter(self, desVel):

        P = np.eye(2)
        u = cp.Variable(2)

        # a = 4.0

        # l = errPos[1]*errPos[1] + errPos[2]*errPos[2]
        # l = np.maximum(l, 0.001)

        h = 3.0 - self.position[0]

        dhdx = -1
        dhdy = 0
        dhdz = 0
        dhdp = np.array([dhdx, dhdy, dhdz])
        A = self.cons[:,:2]
        b = self.cons[:,2]

        # constraints = [dhdp@u >= -0.4*h]
        constraints = [A@u >= b]
        prob = cp.Problem(cp.Minimize(cp.quad_form(u-desVel, P)), constraints)
        result = prob.solve()


        val = u.value


        if np.linalg.norm(val) > 0.5:
            val = 0.5*val/np.linalg.norm(val)

        return val



    def get_vel_sp(self):
        print(self.position)
        self.error_pos = self.position - self.des_position
        print(self.error_pos)
        # self.error_orient = self.orientation - self.des_orientation

        des_vel = self.Kpos * self.error_pos
        print(des_vel)
        if self.constraintsReceived:
            des_vel = self.safety_filter(des_vel)
        print(des_vel)
        print('')

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

#!/usr/bin/env python

import rospy
import numpy as np
import cvxpy as cp
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from tf.transformations import euler_from_quaternion, quaternion_matrix


class SpFilter:
	def __init__(self):
		self.position = np.array([0.0, 0.0, 1.0])
		self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
		self.yaw = 0.0
		self.des_position = np.array([-1.0, -1.0, 1.0])
		self.pos_sp = np.array([-1.0, -1.0, 1.0])
		self.des_orientation = np.array([0.0, 0.0, 0.0, 1.0])
		self.des_yaw = 0.0
		self.error_pos = np.array([0.0, 0, 0])
		self.Kpos = np.array([-1.5, -1.5, -1.5])
		self.Korient = -0.3

		self.sp_threshold = 0.5

		self.rate = rospy.Rate(20)

		self.points_array = np.array([])
		self.A = np.array([])
		self.b = np.array([])
		self.consFlag = False
		self.odomFlag = False
		self.controlFlag = False

		self.odom_sub = rospy.Subscriber(
		    "/shafter4/odometry_sensor1/odometry", Odometry, self.callback_odometry
		)
		self.odom_setpoint_sub = rospy.Subscriber(
		    "/setpoint_position", Odometry, self.sp_odom_callback
		)
		self.posestamped_setpoint_sub = rospy.Subscriber(
		    "/shafter4/command/pose", PoseStamped, self.sp_pose_sta_callback
		)
		self.laser_sub =  rospy.Subscriber("/shafter4/velodyne_points", PointCloud2, self.pointcloud_callback)

		self.cmd_vel_pub = rospy.Publisher("/shafter4/vel_msg", TwistStamped, queue_size=10)
		self.laser_pub = rospy.Publisher("/reduced_points", PointCloud2, queue_size=10)


		while not rospy.is_shutdown():
			self.get_vel_sp()
			self.rate.sleep()



	def callback_odometry(self, msg):
		self.position[0] = msg.pose.pose.position.x
		self.position[1] = msg.pose.pose.position.y
		self.position[2] = msg.pose.pose.position.z
		self.orientation[0] = msg.pose.pose.orientation.x
		self.orientation[1] = msg.pose.pose.orientation.y
		self.orientation[2] = msg.pose.pose.orientation.z
		self.orientation[3] = msg.pose.pose.orientation.w
		q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		self.yaw = np.arctan2(2.0*(q[0]*q[1] + q[3]*q[2]), 1 - 2*(q[1]*q[1] + q[2]*q[2]))
		if not self.odomFlag:
			self.odomFlag = True
	


	def sp_pose_sta_callback(self, msg):
		self.des_position[0] = msg.pose.position.x
		self.des_position[1] = msg.pose.position.y
		self.des_position[2] = msg.pose.position.z
		self.des_orientation[0] = msg.pose.orientation.x
		self.des_orientation[1] = msg.pose.orientation.y
		self.des_orientation[2] = msg.pose.orientation.z
		self.des_orientation[3] = msg.pose.orientation.w
		q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
		self.des_yaw = np.arctan2(2.0*(q[0]*q[1] + q[3]*q[2]), 1 - 2*(q[1]*q[1] + q[2]*q[2]))
		
		if not self.controlFlag:
			self.controlFlag = True



if __name__ == "__main__":
	rospy.init_node("sp_filter_node", anonymous=True)
	node = SpFilter()
	rospy.spin()

#!/usr/bin/env python

import rospy
import numpy as np
import cvxpy as cp
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from tf.transformations import euler_from_quaternion, quaternion_matrix


class VelocityController:
	def __init__(self):
		self.position = np.array([0.0, 0.0, 0.0])
		self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
		self.des_position = np.array([-1.0, -1.0, 0.8])
		self.des_orientation = np.array([0.0, 0.0, 0.0, 1.0])
		self.error_pos = np.array([0.0, 0, 0])
		self.Kpos = np.array([-2.8, -2.8, -1.5])
		self.Korient = -0.3

		self.rate = rospy.Rate(20)

		self.points_array = np.array([])
		self.A = np.array([])
		self.b = np.array([])
		self.consFlag = False

		self.odom_sub = rospy.Subscriber(
		    "/pelican/odometry_sensor1/odometry", Odometry, self.callback_odometry
		)
		self.setpoint_sub = rospy.Subscriber(
		    "/setpoint_position", Odometry, self.callback_setpoint
		)
		self.laser_sub =  rospy.Subscriber("/velodyne_points", PointCloud2, self.pointcloud_callback)

		self.cmd_vel_pub = rospy.Publisher("/pelican/vel_msg", TwistStamped, queue_size=10)
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
		# self.get_vel_sp()

	def callback_setpoint(self, msg):
		self.des_position[0] = msg.pose.pose.position.x
		self.des_position[1] = msg.pose.pose.position.y
		self.des_position[2] = msg.pose.pose.position.z
		self.des_orientation[0] = msg.pose.pose.orientation.x
		self.des_orientation[1] = msg.pose.pose.orientation.y
		self.des_orientation[2] = msg.pose.pose.orientation.z
		self.des_orientation[3] = msg.pose.pose.orientation.w

	def pointcloud_callback(self, msg):
		# Convert the PointCloud2 message to a list of points
		points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

		# Convert the list of points to a NumPy array
		points = np.array(points)

		distances = np.linalg.norm(points, axis=1)
		points = points[distances <= 4.0]

		voxel_size = 0.5
		discrete_coords = np.floor(points/voxel_size).astype(np.int32)
		_, unique_indices = np.unique(discrete_coords, axis=0, return_index=True)
		points = points[unique_indices]
		self.points_array = np.array([points[:,0], points[:,1], points[:,2]]).T

		fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1)]


		points = pc2.create_cloud(msg.header, fields, points.tolist())

		self.laser_pub.publish(points)







		# rospy.loginfo(f"Received point cloud with shape: {self.points_array.shape}")
		if len(self.points_array) < 1:
			self.consFlag =  False

		else:
			self.genConsMatrix()

	def genConsMatrix(self):
		
		R = quaternion_matrix(self.orientation)[:-1, :-1]
		rotated_points = (R@self.points_array.T).T
		rotated_points = rotated_points[rotated_points[:,2] + self.position[2] > 0.5]

		if len(rotated_points) < 1:
			self.consFlag = False

		else:
			self.A = -2*rotated_points
			self.b = -2.5*(np.sum(rotated_points**2, 1) - 1.0)

			# rospy.loginfo(f"A: {self.A.shape}")
			# rospy.loginfo(f"b: {self.b.shape}")

			self.consFlag = True




	# Not used currently
	def safety_filter(self, desVel):        
		P = np.eye(3)
		u = cp.Variable(3)

		# # a = 4.0

		# # l = errPos[1]*errPos[1] + errPos[2]*errPos[2]
		# # l = np.maximum(l, 0.001)

		# h = 2.0 - self.position[0]

		# dhdx = -1
		# dhdy = 0
		# dhdz = 0
		# dhdp = np.array([dhdx, dhdy, dhdz])

		# constraints = [dhdp@u >= -0.4*h]

		# self.genConsMatrix()
		if len(self.A) == len(self.b):
			constraints = [self.A@u >= self.b]
			prob = cp.Problem(cp.Minimize(cp.quad_form(u-desVel, P)), constraints)
			try:
				result = prob.solve()
				val = u.value

				try: 
					val = np.maximum(-np.array([0.5, 0.5, 0.5]), np.minimum(np.array([0.5, 0.5, 0.5]), val))
				except TypeError:
					print("TypeError")
					val = np.array([0.0, 0.0, 0.0])


			except cp.error.SolverError:
				print("Solver Error")
				val = np.array([0.0, 0.0, 0.0])

			return val

		else:
			return np.zeros(desVel.shape)



	def get_vel_sp(self):
		# print(self.position)
		self.error_pos = self.position - self.des_position
		# print(self.error_pos)
		# self.error_orient = self.orientation - self.des_orientation

		des_vel = self.Kpos * self.error_pos
		# print(des_vel)
		if self.consFlag:
			des_vel = self.safety_filter(des_vel)

		des_vel = np.maximum(-np.array([0.5, 0.5, 0.5]), np.minimum(np.array([0.5, 0.5, 0.5]), des_vel))


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

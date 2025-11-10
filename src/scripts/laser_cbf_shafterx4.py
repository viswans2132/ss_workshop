#!/usr/bin/env python

import rospy
import numpy as np
import cvxpy as cp
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from tf.transformations import euler_from_quaternion, quaternion_matrix
import matplotlib.cm as cm


class VelocityController:
	def __init__(self):
		self.position = np.array([0.0, 0.0, 1.0])
		self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
		self.yaw = 0.0
		self.des_position = np.array([-1.0, -1.0, 0.0])
		self.pos_sp = np.array([-1.0, -1.0, 1.0])
		self.des_orientation = np.array([0.0, 0.0, 0.0, 1.0])
		self.des_yaw = 0.0
		self.error_pos = np.array([0.0, 0, 0])
		self.Kpos = np.array([-0.8, -0.8, -1.5])
		self.Korient = -0.3

		self.sp_threshold = 0.3

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
			self.update_pos_sp()
			self.rate.sleep()

	def update_pos_sp(self):
		posError = self.position - self.des_position
		dist = np.linalg.norm(posError)
		# print(dist)

		if dist > self.sp_threshold:
			self.pos_sp = self.position - posError/self.sp_threshold
		else:
			self.pos_sp = self.des_position.copy()


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
			self.des_position[0] = msg.pose.pose.position.x
			self.des_position[1] = msg.pose.pose.position.y
			self.des_orientation[1] = msg.pose.pose.orientation.y
			self.des_orientation[2] = msg.pose.pose.orientation.z
			self.des_orientation[3] = msg.pose.pose.orientation.w
			self.des_yaw = np.arctan2(2.0*(q[0]*q[1] + q[3]*q[2]), 1 - 2*(q[1]*q[1] + q[2]*q[2]))
			self.odomFlag = True
		# self.get_vel_sp()

	def sp_odom_callback(self, msg):
		self.des_position[0] = msg.pose.pose.position.x
		self.des_position[1] = msg.pose.pose.position.y
		self.des_position[2] = msg.pose.pose.position.z
		self.des_orientation[0] = msg.pose.pose.orientation.x
		self.des_orientation[1] = msg.pose.pose.orientation.y
		self.des_orientation[2] = msg.pose.pose.orientation.z
		self.des_orientation[3] = msg.pose.pose.orientation.w
		q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		self.des_yaw = np.arctan2(2.0*(q[0]*q[1] + q[3]*q[2]), 1 - 2*(q[1]*q[1] + q[2]*q[2]))
		# self.des_yaw = np.arctan2(2.0*(q[1]*q[2] + q[3]*q[0]), q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2])
		# print(self.des_yaw)
		# print(q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2])
		# print((q[0]*q[1] + q[3]*q[2]))


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

	def pointcloud_callback(self, msg):
		# Convert the PointCloud2 message to a list of points
		points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

		# Convert the list of points to a NumPy array
		points = np.array(points)

		distances = np.linalg.norm(points, axis=1)
		points = points[distances <= 2.5]

		voxel_size = 0.5
		discrete_coords = np.floor(points/voxel_size).astype(np.int32)
		_, unique_indices = np.unique(discrete_coords, axis=0, return_index=True)
		points = points[unique_indices]
		self.points_array = np.array([points[:,0], points[:,1], points[:,2]]).T

		# self.ground_points = self.points_array[self.points_array < -0.3]
		# self.ground_points = self.ground_points[:, np.argmax(self.ground_points[2])]

		# self.points_array = self.points_array[self.points_array >= -0.3]



		# rospy.loginfo(f"Received point cloud with shape: {self.points_array.shape}")
		if len(self.points_array) < 1:
			self.consFlag =  False

		else:
			self.genConsMatrix()

	def genConsMatrix(self):		
		R = quaternion_matrix(self.orientation)[:-1, :-1]
		rotated_points = (R@self.points_array.T).T





		# print(len(rotated_points))

		if len(rotated_points) < 1:
			self.consFlag = False

		else:
			ground_points = rotated_points[rotated_points[:,2] + self.position[2] <= 0.2]
			elevated_points = rotated_points[rotated_points[:,2] + self.position[2] > 0.2]
			self.consFlag = True

			if len(ground_points) > 0:
				self.A = np.array([[0.0, 0.0, 1.0]])
				max_ground_height = ground_points[np.argmax(ground_points[:,2]), 2] - 0.04
				self.b = np.array([-0.1*(max_ground_height - 0.5)])
			else:
				self.A = np.array([[0.0, 0.0, 1.0]])
				self.b = np.array([-0.1*(0.3)])
			print(self.b)


			if len(elevated_points) > 0:
				min_ele_height = elevated_points[np.argmin(elevated_points[:,2]), 2] - 0.04
				self.b = np.array([-0.1*(- min_ele_height - 0.2)])
				self.A = np.vstack((self.A, -2*elevated_points))
				self.b = np.hstack((self.b, -0.5*(np.sum(elevated_points**2, 1) - 1.0)))


				translated_points = elevated_points + self.position
				fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1), PointField('rgb', 12, PointField.FLOAT32, 1)]

				pcl_msg = PointCloud2()
				pcl_msg.header.stamp = rospy.Time.now()
				pcl_msg.header.frame_id = "world"

				magn = 1*(np.sum(elevated_points**2, 1) - 1.0)
				magn_normalized = (magn - magn.min())/(magn.max() - magn.min() + 0.0001)
				color_map = cm.get_cmap('viridis')
				colors = (color_map(magn_normalized)[:,:3]*255).astype(np.uint8)
				rgb_uint32 = (colors[:, 0].astype(np.uint32) << 16) | \
							(colors[:, 1].astype(np.uint32) << 8) | \
							(colors[:, 2].astype(np.uint32))
				rgb_float = rgb_uint32.view(np.float32)
				colored_points = np.column_stack((translated_points, rgb_float))
				# print([rgb_float])

				points = pc2.create_cloud(pcl_msg.header, fields, colored_points.tolist())

				self.laser_pub.publish(points)



	def safety_filter(self, desVel):        
		P = np.eye(3)
		u = cp.Variable(3)
		# print(len(self.A))		
		# print(len(self.b))
		
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
			
			print(f"{val[0]:.2f}, {val[1]:.2f}")

			return val

		else:
			return np.zeros(desVel.shape)



	def get_vel_sp(self):
		if self.controlFlag:
			self.error_pos = self.position - self.pos_sp
			des_vel = self.Kpos * self.error_pos
			des_vel = np.maximum(-np.array([0.9, 0.9, 0.5]), np.minimum(np.array([0.9, 0.9, 0.5]), des_vel))
			if self.consFlag:
				des_vel = self.safety_filter(des_vel)
			errYaw = self.yaw - self.des_yaw
		
			if np.abs(errYaw) > np.pi:
				errYaw = np.sign(errYaw)*(np.abs(errYaw) - 2*np.pi)
			# print('{:.2f}'.format(errYaw))
			
			desYawVel = -2.3*errYaw
			
			desYawVel = np.minimum(0.6, np.maximum(-0.6, desYawVel))
			# des_vel = np.zeros(des_vel.shape)
			# desYawVel = 0.0

			vel_sp = TwistStamped()
			vel_sp.twist.linear.x = des_vel[0]
			vel_sp.twist.linear.y = des_vel[1]
			vel_sp.twist.linear.z = des_vel[2]
			vel_sp.twist.angular.z = desYawVel
			vel_sp.header.stamp = rospy.Time.now()

			self.cmd_vel_pub.publish(vel_sp)
			print(f"{des_vel[0]:.2f}, {des_vel[1]:.2f}, {des_vel[2]:.2f}")


if __name__ == "__main__":
	rospy.init_node("wall_cbf_node", anonymous=True)
	node = VelocityController()
	rospy.spin()

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
import argparse
import math
from visualization_msgs.msg import Marker
from ss_workshop.srv import ControlStatus, ControlStatusResponse


class VelocityController:
	def __init__(self, namespace, startFlag):
		self.namespace = namespace
		self.startFlag = startFlag

		self.position = np.array([0.0, 0.0, 1.0])
		self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
		self.yaw = 0.0
		self.des_position = np.array([-1.0, -1.0, 1.0])
		self.pos_sp = np.array([-1.0, -1.0, 1.0])
		self.des_orientation = np.array([0.0, 0.0, 0.0, 1.0])
		self.des_yaw = 0.0
		self.error_pos = np.array([0.0, 0, 0])
		self.Kpos = np.array([-0.8, -0.8, -1.5])
		self.Korient = -0.3

		self.safety_semi_major = 1.0
		self.safety_semi_minor = 0.5

		self.CBF_H_POW2 = self.safety_semi_minor**2
		self.CBF_V_POW2 = self.safety_semi_major**4

		self._k_alpha = 20.1
		self._k_gamma = 0.5
		self._k_kappa = 10.0

		self.sp_threshold = 0.3
		self.counter = 0.0

		self.rate = rospy.Rate(20)

		self.points_array = np.array([])
		self.A = np.array([])
		self.b = np.array([])
		self.consFlag = False
		self.odomFlag = False
		self.controlFlag = self.startFlag
		self._control_status = 0

		self.odom_sub = rospy.Subscriber(
		    f"/{self.namespace}/odometry_sensor1/odometry", Odometry, self.callback_odometry
		)
		self.odom_setpoint_sub = rospy.Subscriber(
		    "/setpoint_position", Odometry, self.sp_odom_callback
		)
		self.posestamped_setpoint_sub = rospy.Subscriber(
		    f"/{self.namespace}/command/pose", PoseStamped, self.sp_pose_sta_callback
		)
		self.laser_sub =  rospy.Subscriber(f"/{self.namespace}/velodyne_points", PointCloud2, self.pointcloud_callback)

		self.cmd_vel_pub = rospy.Publisher(f"/{self.namespace}/vel_msg", TwistStamped, queue_size=10)
		self.laser_pub = rospy.Publisher("/reduced_points", PointCloud2, queue_size=10)

		self._control_status_server = rospy.Service(f"{namespace}/control_status", ControlStatus, self.return_control_status)
		self.cbf_marker_pub = rospy.Publisher("cbf_safe_set", Marker, queue_size=1)


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


	def sp_pose_sta_callback(self, msg):
		new_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
		q = msg.pose.orientation
		new_yaw = math.atan2(
		    2.0 * (q.w * q.z + q.x * q.y), 
		    1.0 - 2.0 * (q.y**2 + q.z**2)
		)
		if (np.linalg.norm(self.des_position - new_position) >= 0.3) or (np.abs(self.des_yaw - new_yaw) >= 0.05):
			self.des_position = new_position
			self.des_orientation = q
			self.des_yaw = new_yaw

			self._control_status = 0
			self.counter = 0
		
		if not self.controlFlag:
			self.controlFlag = True
			print("Controller Active")

	def return_control_status(self, req):
		return ControlStatusResponse(status=self._control_status)

	def pointcloud_callback(self, msg):
		# Convert the PointCloud2 message to a list of points
		points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

		# Convert the list of points to a NumPy array
		points = np.array(points)

		distances = np.linalg.norm(points, axis=1)
		points = points[distances <= 3.5]

		voxel_size = 0.05
		discrete_coords = np.floor(points/voxel_size).astype(np.int32)
		_, unique_indices = np.unique(discrete_coords, axis=0, return_index=True)
		points = points[unique_indices]
		self.points_array = np.array([points[:,0], points[:,1], points[:,2]]).T

		if len(self.points_array) < 1:
			self.consFlag =  False

		else:
			self.genConsMatrix()

	def genConsMatrix(self):		
		R = quaternion_matrix(self.orientation)[:-1, :-1]
		rotated_points = (R@self.points_array.T).T

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
			# print(self.b)

			if len(elevated_points) > 0:
				min_ele_height = elevated_points[np.argmin(elevated_points[:,2]), 2] - 0.04
				self.b = np.array([-0.1*(- min_ele_height - 0.2)])
				# self.A = np.vstack((self.A, -3*elevated_points**2))
				# self.b = np.hstack((self.b, -0.5*(np.sum(elevated_points**4, 1) - 1.0)))
                # 
				X_r = elevated_points[:,0]
				Y_r = elevated_points[:,1]
				Z_r = elevated_points[:,2]

				h_elevated = ((X_r**2 + Y_r**2)/self.CBF_H_POW2) + (Z_r**2/self.CBF_V_POW2) - 1.0

				Ax = -2 * (X_r**1) / self.CBF_H_POW2
				Ay = -2 * (Y_r**1) / self.CBF_H_POW2
				Az = -2 * (Z_r**1) / self.CBF_V_POW2

				A_elevated = np.column_stack((Ax, Ay, Az))

				tanh_h = np.tanh(h_elevated/self._k_gamma)
				scaled_tan_h = -self._k_kappa * tanh_h
				exp_shifted = np.exp(scaled_tan_h)
				sum_exp = np.sum(exp_shifted) + 1e-12

				H_composite = -(self._k_gamma/self._k_kappa) * (np.log(sum_exp))

				weights = exp_shifted/sum_exp

				A_composite = (weights[:, None] * A_elevated).sum(axis=0)
				b_composite = -self._k_alpha * H_composite

				# A_list.append(A_composite)
				# b_list.append(b_composite)

				self.A = np.vstack((self.A, A_composite))
				self.b = np.hstack((self.b, b_composite))

				translated_points = elevated_points + self.position
				fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1), PointField('rgb', 12, PointField.FLOAT32, 1)]

				pcl_msg = PointCloud2()
				pcl_msg.header.stamp = rospy.Time.now()
				pcl_msg.header.frame_id = "world"

				magn = -1*(np.sum(elevated_points**2, 1) - 1.0)
				magn_normalized = (magn - magn.min())/(magn.max() - magn.min() + 0.0001)
				color_map = cm.get_cmap('magma')
				colors = (color_map(magn_normalized)[:,:3]*255).astype(np.uint8)
				rgb_uint32 = (colors[:, 0].astype(np.uint32) << 16) | \
							(colors[:, 1].astype(np.uint32) << 8) | \
							(colors[:, 2].astype(np.uint32))
				rgb_float = rgb_uint32.view(np.float32)
				colored_points = np.column_stack((translated_points, rgb_float))

				points = pc2.create_cloud(pcl_msg.header, fields, colored_points.tolist())

				self.laser_pub.publish(points)
				self.publish_cbf_safe_set()



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
			
			# print(f"{val[0]:.2f}, {val[1]:.2f}")

			return val

		else:
			return np.zeros(desVel.shape)



	def get_vel_sp(self):
		if self.controlFlag:
			self.error_pos = self.position - self.pos_sp
			errYaw = self.yaw - self.des_yaw
			if np.abs(errYaw) > np.pi:
				errYaw = np.sign(errYaw)*(np.abs(errYaw) - 2*np.pi)

			desYawVel = -2.3*errYaw

			if np.linalg.norm(self.error_pos) < 0.1:
				des_vel = np.array([0.0, 0.0, 0.0])

				if errYaw < 0.05:
					desYawVel = 0.0
					self._control_status = 1

			else:
				des_vel = self.Kpos * self.error_pos
				des_vel = np.maximum(-np.array([0.9, 0.9, 0.5]), np.minimum(np.array([0.9, 0.9, 0.5]), des_vel))
			
			if self.consFlag:
				des_vel_filtered = self.safety_filter(des_vel)

				if np.linalg.norm(des_vel_filtered) < 0.03:
					des_vel_filtered = np.array([0.0, 0.0, 0.0])
					if np.linalg.norm(self.error_pos) >= 0.1:
						self.counter += 1
						if self.counter > 30:
							self._control_status = 2

				else:
					self.counter = 0

				des_vel = des_vel_filtered.copy()
			
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
			# print(f"{des_vel[0]:.2f}, {des_vel[1]:.2f}, {des_vel[2]:.2f}")

	def publish_cbf_safe_set(self):
		marker = Marker()
		marker.header.frame_id = f"{self.namespace}/base_link"
		marker.header.stamp = rospy.Time.now()
		marker.ns = "cbf_safe_set"
		marker.id = 0
		marker.type = Marker.MESH_RESOURCE
		marker.action = Marker.ADD
		marker.pose.orientation.w = 1.0

		marker.mesh_resource = "file:///home/godzillapc/catkin_workspaces/control_ws/src/ss_workshop/models/meshes/superellipsoid4.stl"
		marker.mesh_use_embedded_materials = False


		marker.scale.x = 1.0 * self.safety_semi_minor
		marker.scale.y = 1.0 * self.safety_semi_minor
		marker.scale.z = 1.0 * self.safety_semi_major

		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 0.4

		self.cbf_marker_pub.publish(marker)


if __name__ == "__main__":
	rospy.init_node("wall_cbf_node", anonymous=True)
	parser = argparse.ArgumentParser(description="Namespace")
	parser.add_argument("--namespace", type=str, default="pelican", help="Specify a namespace. Defaults to <pelican>")
	parser.add_argument("--startFlag", action='store_true', default="False", help="Prevents the controller from starting automatically. Defaults to <False>")

	args = parser.parse_args()
	startFlag = args.startFlag

	node = VelocityController(args.namespace, args.startFlag)
	rospy.spin()

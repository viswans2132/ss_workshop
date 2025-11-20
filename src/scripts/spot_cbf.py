#!/usr/bin/env python

import rospy
import os
import sys
import math
import numpy as np
import numpy.linalg as la
import cvxpy as cp
import argparse 
# ROS 1 message types
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String # Updated to include String for stop command
from std_msgs.msg import Float64MultiArray
# Point Cloud Imports
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import matplotlib.cm as cm
# Utility for Quaternion to Matrix/Euler conversion (tf_transformations or equivalent is needed)
from tf.transformations import quaternion_matrix, euler_from_quaternion 

# Define a class for the CBF Velocity Controller node
class CbfVelocityController:
    def __init__(self, namespace='', autostart=False):
        # 1. ROS 1 Node Initialization
        rospy.init_node('cbf_velocity_controller', anonymous=True)
        rospy.loginfo("CbfVelocityController node initialized (ROS 1) with namespace: '{}'".format(namespace))
        
        # --- Namespace Check ---
        if namespace and not namespace.startswith('/'):
            namespace = '/' + namespace

        # 2. Define the Publishers (Topics are namespaced using f-strings)
        self._velocity_publisher = rospy.Publisher(
            f"{namespace}/cmd_vel", Twist, queue_size=1)
        
        # TOPIC CHANGE: obstacles_pcl -> reduced_points
        self.laser_pub = rospy.Publisher(
            f"{namespace}/reduced_points", PointCloud2, queue_size=1) 
        
        # 3. Define Subscribers (Topics are namespaced using f-strings)
        # TOPIC CHANGE: odom -> odometry/imu
        self._odom_subscriber = rospy.Subscriber(
            f"{namespace}/odometry/imu", Odometry, self.odom_callback, queue_size=1)
            
        # TOPIC CHANGE: pointcloud -> velodyne_points
        self._pcl_subscriber = rospy.Subscriber(
            f"{namespace}/velodyne_points", PointCloud2, self.pointcloud_callback, queue_size=1)
            
        # TOPIC CHANGE: setpoint_pose -> command/pose
        self._setpoint_subscriber = rospy.Subscriber(
            f"{namespace}/command/pose", PoseStamped, self.setpoint_callback, queue_size=1) 

        # MESSAGE & TOPIC CHANGE: stop_command -> stop, Bool -> String
        self._stop_subscriber = rospy.Subscriber(
            f"{namespace}/stop", String, self.stop_command_callback, queue_size=1)
        
        # Time period (used for rospy.Rate)
        self._control_dt = 0.2  # seconds
        self._rate = rospy.Rate(1.0 / self._control_dt) 
        self._start_time = rospy.get_time()

        # State and Target Variables
        self._current_position = np.array([0.0, 0.0, 0.0]) # 3D Position
        self._current_orientation = np.array([0.0, 0.0, 0.0, 1.0]) # Quaternion (x, y, z, w)
        self._current_yaw = 0.0
        
        # Desired State Variables (Updated by setpoint_callback)
        self._desired_position = np.array([0.0, 0.0, 0.0]) # 3D desired position
        self._desired_orientation = np.array([0.0, 0.0, 0.0, 1.0]) # 4D desired orientation (quaternion)
        self._desired_yaw = 0.0 # Desired yaw angle
        
        self.points_array = np.array([]) # Processed 3D points

        # Control Barrier Function (CBF) and QP Variables
        self.A = np.zeros((1, 2))          # Constraint matrix A (Jacobian h)
        self.b = np.array([0.0])           # Constraint vector b (-gamma * h)
        self.P = np.eye(2)                 # Weight matrix for quadratic cost (P=I minimizes ||u - u_nom||^2)
        self.u = cp.Variable(2)            # Optimization variable (Control input vector [vx, vy] in world frame)
        
        # Control Flags
        self._odometry_received = False    # Flag to ensure initial position is known
        self._setpoint_received = autostart # Flag is true if autostart arg is used
        self._constraints_active = False   # Flag to check if obstacle constraints are present
        self._stop_command_received = False # Flag: True to force zero velocity

        # Start the main control loop
        self.control_loop()

    def _publish_zero_velocity(self):
        """Helper function to publish a zero Twist message."""
        zero_msg = Twist()
        zero_msg.linear.x = 0.0
        zero_msg.linear.y = 0.0
        zero_msg.angular.z = 0.0
        self._velocity_publisher.publish(zero_msg)

    def _check_control_state_and_stop(self, position_error):
        """
        Checks all conditions that require the robot to stop or skip control logic.
        Publishes zero velocity if any stop condition is met.
        
        Returns: True if control should be skipped, False otherwise.
        """
        # 1. Immediate Stop Command (Safety Pause)
        if self._stop_command_received:
            rospy.logwarn_throttle(1.0, "STOP COMMAND RECEIVED: Publishing zero velocity.")
            self._publish_zero_velocity()
            return True

        # 2. Initialization Check (Missing Data)
        if not self._odometry_received or not self._setpoint_received:
            rospy.logwarn_throttle(1.0, "Waiting for Odometry and/or Setpoint data...")
            self._publish_zero_velocity()
            return True
        
        # 3. Goal Reached Check
        if la.norm(position_error) < 0.1: # Threshold of 10 cm
            rospy.loginfo_throttle(1.0, "Goal reached! Holding position.")
            self._publish_zero_velocity()
            return True

        return False


    def control_loop(self):
        """
        The main control loop using rospy.Rate for timing.
        """
        rospy.loginfo("Starting control loop...")
        while not rospy.is_shutdown():
            self._main_command_callback()
            self._rate.sleep()


    def _main_command_callback(self):
        """
        Timed callback function to compute and publish velocity commands.
        """
        position_error = self._desired_position[:2] - self._current_position[:2]
        
        # Check all stop/skip conditions (Emergency Stop, Missing Data, Goal Reached).
        if self._check_control_state_and_stop(position_error):
            return 
            
        # --- Control Logic (only runs if checks above pass) ---
        
        goal_msg = Twist()

        # --- 1. Angular Control (Heading) ---
        yaw_sp = np.arctan2(position_error[1], position_error[0])
        yaw_error = yaw_sp - self._current_yaw
        
        # Normalize the angular error to be between -pi and pi
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi 

        goal_msg.angular.z = np.clip(0.5 * yaw_error, -0.5, 0.5)

        # --- 2. Linear Control (Nominal Velocity) ---
        # Rotation matrix from World Frame to Robot Frame (only 2D part needed for velocity transform)
        R_world_to_robot = np.array([[np.cos(self._current_yaw), np.sin(self._current_yaw)], 
                                     [-np.sin(self._current_yaw), np.cos(self._current_yaw)]])
        
        # Nominal velocity vector (u_nom) in World Frame 
        u_nominal = np.array([0.5 * position_error[0], 0.5 * position_error[1]])
        u_nominal = u_nominal / la.norm(u_nominal) * 0.5 # Normalize and set max speed if moving away

        # --- 3. CBF Constraint Generation ---
        self._generate_constraint_matrices()

        # --- 4. CBF Filtering (Quadratic Program) ---
        if self._constraints_active:
            u_filtered_world = self._cbf_filter(u_nominal)
        else:
            u_filtered_world = u_nominal
        
        # --- 5. Transform and Publish ---
        # Project World Frame velocity (u_filtered_world) into Robot Frame
        u_filtered_robot_frame = R_world_to_robot @ u_filtered_world
        
        # Apply clamping to output velocities
        goal_msg.linear.x = np.clip(u_filtered_robot_frame[0], -0.3, 0.3)
        goal_msg.linear.y = np.clip(u_filtered_robot_frame[1], -0.3, 0.3)
            
        # self._velocity_publisher.publish(goal_msg)
        rospy.loginfo('Command: Linear X: {:.2f}, Linear Y: {:.2f}, Angular Z: {:.2f}'.format(
            goal_msg.linear.x, goal_msg.linear.y, goal_msg.angular.z))


    def _cbf_filter(self, u_nominal):
        """
        Control Barrier Function implementation via Quadratic Programming (QP).
        Finds the closest control input u to u_nominal that satisfies A*u >= b.
        """
        try:
            constraints = [self.A @ self.u >= self.b]
            prob = cp.Problem(cp.Minimize(cp.quad_form(self.u - u_nominal, self.P)), constraints)
            
            try:
                # Use OSQP solver
                prob.solve(solver=cp.OSQP)
                u = self.u.value
            except cp.error.SolverError:
                rospy.logwarn("QP Solver Error: Holding the position.")
                u = np.array([0.0, 0.0])

        except ValueError:
            rospy.logerr("Constraint matrices have incompatible dimensions.")
            u = np.array([0.0, 0.0])

        try:
            return np.array([u[0], u[1]])
        except TypeError:
            rospy.logerr('Optimizer returned invalid type (likely unfeasible). Returning zero velocity.')
            return np.array([0.0, 0.0])


    def odom_callback(self, msg):
        """
        Standard Odometry callback. Updates 3D position and orientation quaternion.
        """
        # Store full 3D position
        self._current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        
        # Store full quaternion (x, y, z, w)
        q = msg.pose.pose.orientation
        self._current_orientation = np.array([q.x, q.y, q.z, q.w])

        # Calculate yaw from quaternion for 2D control
        roll, pitch, yaw = euler_from_quaternion(self._current_orientation)
        self._current_yaw = yaw

        if not self._odometry_received:
            self._odometry_received = True
            rospy.loginfo("Odometry Initialized.")

    def stop_command_callback(self, msg):
        """
        Callback for the string stop command.
        If any message is received (meaning the string is non-empty), 
        autonomous control is paused and zero velocity is published.
        The stop condition is handled in _check_control_state_and_stop.
        """
        # Set stop flag to True if any message is received (non-empty string data)
        if msg.data:
            self._stop_command_received = True
            rospy.logwarn("Stop command received: Controller paused.")
        else:
            # If an empty string is sent, assume it's a resume command.
            self._stop_command_received = False
            rospy.loginfo("Empty string received: Resuming control.")


    def setpoint_callback(self, msg):
        """
        Callback to receive the desired 2D/3D position and orientation setpoint (from PoseStamped).
        Updates desired position and yaw.
        """
        # Update 3D position setpoint
        self._desired_position[0] = msg.pose.position.x
        self._desired_position[1] = msg.pose.position.y
        self._desired_position[2] = msg.pose.position.z

        # Store full desired orientation
        q = msg.pose.orientation
        self._desired_orientation = np.array([q.x, q.y, q.z, q.w])

        # Calculate desired yaw from quaternion directly using math.atan2 and quaternion components
        # Yaw (z-rotation) formula: atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        self._desired_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 
            1.0 - 2.0 * (q.y**2 + q.z**2)
        )
        
        if not self._setpoint_received:
            self._setpoint_received = True
            rospy.loginfo("Setpoint Initialized.")
    
    def pointcloud_callback(self, msg):
        """
        Callback to receive and process PointCloud2 data.
        Performs downsampling (Voxel Grid) and range filtering.
        """
        # Convert the PointCloud2 message to a list of points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Convert the list of points to a NumPy array
        points = np.array(points)

        # 1. Distance filter (points must be within 2.5m)
        distances = np.linalg.norm(points, axis=1)
        points = points[distances <= 2.5]

        # 2. Voxel Grid downsampling
        voxel_size = 0.5
        discrete_coords = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(discrete_coords, axis=0, return_index=True)
        points = points[unique_indices]
        
        # Store processed points (N x 3)
        self.points_array = np.array([points[:,0], points[:,1], points[:,2]]).T

        # Check constraint feasibility
        if len(self.points_array) < 1:
            self._constraints_active = False
        else:
            self._generate_constraint_matrices()


    def _generate_constraint_matrices(self):
        """
        Generates the 2D constraint matrices A (M x 2) and b (M x 1)
        based on the point cloud data.
        """
        if self.points_array.size < 1:
            self._constraints_active = False
            return
            
        # 1. Rotate points from sensor frame to World frame (assuming sensor is on robot)
        # Get the full 3x3 rotation matrix from Quaternion
        R_robot_to_world = quaternion_matrix(self._current_orientation)[:3, :3]
        
        # Rotated points are in the world frame, relative to the robot origin
        rotated_points = (R_robot_to_world @ self.points_array.T).T

        if len(rotated_points) < 1:
            self._constraints_active = False
            return

        # 2. Separate Ground and Elevated Points based on world Z position
        z_world = rotated_points[:,2] + self._current_position[2]
        
        # Ground points are ignored for obstacle avoidance in this CBF structure
        elevated_points = rotated_points[z_world > 0.2] # Z-world is > 0.2
        
        # Reset constraint matrices
        A_list = []
        b_list = []
        
        # --- Elevated Obstacle Constraints (Multiple 2D Circular Constraints) ---
        if len(elevated_points) > 0:
            
            # A_i = -2 * [x_i, y_i]
            # Jacobian (dh/d(x,y)) for the 2D point [x_i, y_i]
            # The coordinates P_i = [x_i, y_i, z_i] are in the Robot's frame (robot at origin).
            A_elevated = -2 * elevated_points[:, :2] # N x 2 matrix
            
            # h_elevated = ||P_i||_XY^2 - r^2. Safety radius r=0.5m.
            h_elevated = np.sum(elevated_points[:,:2]**2, axis=1) - 0.5**2 
            
            # b_i = -gamma * h(x). gamma = 3.1
            b_elevated = -3.1 * h_elevated 

            A_list.append(A_elevated)
            b_list.append(b_elevated)
            
            # --- Visualization ---
            translated_points = rotated_points + self._current_position
            fields = [PointField('x', 0, PointField.FLOAT32, 1), 
                      PointField('y', 4, PointField.FLOAT32, 1), 
                      PointField('z', 8, PointField.FLOAT32, 1), 
                      PointField('rgb', 12, PointField.FLOAT32, 1)]

            pcl_msg = PointCloud2()
            pcl_msg.header.stamp = rospy.Time.now()
            pcl_msg.header.frame_id = "world"

            # Color points based on magnitude (for visualization only)
            magn = 1*(np.sum(elevated_points**2, 1) - 1.0)
            magn_normalized = (magn - magn.min())/(magn.max() - magn.min() + 0.0001)
            color_map = cm.get_cmap('viridis') 
            colors = (color_map(magn_normalized)[:,:3]*255).astype(np.uint8)
            rgb_uint32 = (colors[:, 0].astype(np.uint32) << 16) | \
                         (colors[:, 1].astype(np.uint32) << 8) | \
                         (colors[:, 2].astype(np.uint32))
            rgb_float = rgb_uint32.view(np.float32)
            colored_points = np.column_stack((translated_points, rgb_float))

            points = pc2.create_cloud(pcl_msg.header, fields, colored_points.tolist())
            self.laser_pub.publish(points)
        
        # --- Final Matrix Construction ---
        if A_list:
            self.A = np.vstack(A_list)
            self.b = np.concatenate(b_list)
            self._constraints_active = True
        else:
            # If no elevated points, use a dummy 1x2 constraint with h=large positive (always safe)
            self.A = np.zeros((1, 2))
            self.b = np.array([-1000.0])
            self._constraints_active = False 


def main():
    """
    Main function for ROS 1 node initialization and argument parsing.
    """
    # 1. Initialize argument parser
    parser = argparse.ArgumentParser(description="CBF Velocity Controller ROS 1 Node.")
    parser.add_argument(
        '--namespace',
        type=str,
        default='spot',  # Default namespace: "spot"
        help='The robot namespace (e.g., husky, spot). Default: "spot"'
    )
    parser.add_argument(
        '--autostart',
        action='store_true',
        help='If set, the controller skips waiting for the initial setpoint message.'
    )
    
    # 2. Parse arguments, isolating ROS arguments
    # rospy.myargv is necessary in ROS 1 to filter out ROS-specific command line args
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    try:
        # Pass the parsed namespace and autostart flag to the controller constructor
        CbfVelocityController(namespace=args.namespace, autostart=args.autostart)
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down the cbf_velocity_controller')
        pass

if __name__ == '__main__':
    main()
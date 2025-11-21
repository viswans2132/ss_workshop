#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2 # ROS 1 import for pc2 utils

# ROS 1 message types
from sensor_msgs.msg import LaserScan, PointCloud2 
from sensor_msgs.msg import PointField # Needed for PointCloud2 fields

class LaserToPointCloudConverter:
    """
    ROS 1 node to convert 2D LaserScan data into a PointCloud2 message.
    The resulting PointCloud is published in the sensor/robot BODY FRAME
    (the frame_id of the incoming LaserScan message).
    """
    def __init__(self):
        # 1. Initialize the ROS 1 Node
        rospy.init_node('laser_to_pointcloud_converter', anonymous=True)
        rospy.loginfo("LaserToPointCloudConverter node started.")

        # 2. Define Subscribers
        # Subscribes to the 2D laser scan topic
        self._laser_sub = rospy.Subscriber('/go1_gazebo/scan', LaserScan, self.laser_callback, queue_size=10)
        
        # 3. Define Publisher
        # Publishes the generated PointCloud2 message
        self._pc_pub = rospy.Publisher('/go1_gazebo/velodyne_points', PointCloud2, queue_size=10)
        
        # NOTE: Removed Odometry subscription as the output frame is the robot's body frame.

    def laser_callback(self, msg: LaserScan):
        """
        Converts the LaserScan message into a PointCloud in the sensor frame.
        """
        
        points_list = []
        angle = msg.angle_min
        
        # Iterate through the ranges array
        for r in msg.ranges:
            # Filter out invalid ranges
            if msg.range_min < r < msg.range_max:
                # Calculate Cartesian coordinates (X, Y) relative to the sensor
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                z = 0.0 # LaserScan is 2D, so Z is typically zero
                points_list.append([x, y, z])
            
            angle += msg.angle_increment

        if not points_list:
            rospy.logwarn("Received LaserScan but found no valid points.")
            return

        # 4. Create PointCloud2 message in the sensor frame
        
        # Define fields for X, Y, Z
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        # The PointCloud's frame_id should be the frame of the sensor data source 
        # (usually 'laser', 'base_link', or similar).
        cloud_msg = pc2.create_cloud(
            header=msg.header, 
            fields=fields, 
            points=points_list
        )
        
        # The frame_id is taken directly from the LaserScan header, which is 
        # the sensor's frame (i.e., the robot's body frame or sensor link frame).
        cloud_msg.header.frame_id = msg.header.frame_id 
        
        self._pc_pub.publish(cloud_msg)
        rospy.loginfo("Published PointCloud2 message in frame: {}".format(cloud_msg.header.frame_id))


def main():
    """
    Main entry point for the ROS 1 node.
    """
    try:
        LaserToPointCloudConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
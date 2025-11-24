#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32

def pcl2_to_pcl(msg):
    cloud = PointCloud()
    cloud.header = msg.header

    # Unpack all points (x, y, z)
    points = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
    for pt in points:
        cloud.points.append(Point32(pt[0], pt[1], pt[2]))

    # If intensity or other channel data exists, add it
    # (example for intensity channel)
    if any(f.name == 'intensity' for f in msg.fields):
        intensity_values = []
        points = pc2.read_points(msg, field_names=['intensity'], skip_nans=True)
        for p in points:
            intensity_values.append(p[0])
        intensity_channel = ChannelFloat32()
        intensity_channel.name = 'intensity'
        intensity_channel.values = intensity_values
        cloud.channels.append(intensity_channel)

    return cloud

def callback(msg):
    pcl_msg = pcl2_to_pcl(msg)
    pub.publish(pcl_msg)

if __name__ == '__main__':
    rospy.init_node('pcl2_to_pcl_converter')
    pub = rospy.Publisher('/pcl', PointCloud, queue_size=1)
    sub = rospy.Subscriber('/pelican/velodyne_points', PointCloud2, callback)
    rospy.loginfo("Ready to convert PointCloud2 to PointCloud!")
    rospy.spin()

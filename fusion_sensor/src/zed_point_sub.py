#!/usr/bin/env python

from time import sleep
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import sys
import pcl
import struct
import ctypes
import time

class PrjCloud:
    def __init__(self):
        self.pcl_sub = rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, self.pcl_callback)
        self.pcl = PointCloud2()
        print ('Ready')

    def pcl_callback(self, pcl_ros):
        self.pcl = pcl_ros

        # for point in pc2.read_points(self.prj_pcl):
        #     # point[2] = 0.0
        #     rospy.logwarn("x, y, z: %.1f, %.1f, %.1f" % (point[0], point[1], point[2]))
        #     rospy.logwarn("my field 1: %f" % (point[3]))
        # self.pcl_pub.publish(self.pcl)
    
    def ros_to_pcl(self, ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message

            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            # data[2] = 0
            points_list.append([data[0], data[1], 0, data[3]])
        # print (points_list)
        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)

        return pcl_data
    
    def pcl_to_ros(self, pcl_array):
        """ Converts a pcl PointXYZRGB to a ROS PointCloud2 message

            Args:
                pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud

            Returns:
                PointCloud2: A ROS point cloud
        """
        ros_msg = PointCloud2()

        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.header.frame_id = "base_link_zed"

        ros_msg.height = 1
        ros_msg.width = pcl_array.size

        ros_msg.fields.append(PointField(
                                name="x",
                                offset=0,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="y",
                                offset=4,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="z",
                                offset=8,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="rgb",
                                offset=16,
                                datatype=PointField.FLOAT32, count=1))

        ros_msg.is_bigendian = False
        ros_msg.point_step = 32
        ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
        ros_msg.is_dense = False
        buffer = []

        for data in pcl_array:
            s = struct.pack('>f', data[3])
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value

            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)

            buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

        ros_msg.data = "".join(buffer)

        return ros_msg
    
    def prj_pcl(self):
        pcl_data = self.ros_to_pcl(self.pcl)
        # rospy.loginfo("aaaa")
        ros_msg = self.pcl_to_ros(pcl_data)
        # self.pcl_pub.publish(self.pcl)
        return ros_msg
    

if __name__ == '__main__':
    rospy.init_node('project_point_cloud', anonymous=True)

    prj_pcl = PrjCloud()

    pcl_pub = rospy.Publisher("/semantic_pcl/semantic_pcl", PointCloud2, queue_size = 1)
    # Generate point cloud and pulish ros message
    while not rospy.is_shutdown():
        since = time.time()
        cloud_ros = prj_pcl.prj_pcl()
        # cloud_ros = prj_pcl.pcl
        pcl_pub.publish(cloud_ros)
        # print("Generate and publish pcl took", time.time() - since)
    rospy.spin()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
class read_pcd:

    def callback(self, data):
        point_cloud_list = pc2.read_points_list(data)
        #print(point_cloud_list)
        point_cloud_numpy = np.array(point_cloud_list)
        print(point_cloud_numpy.shape)

    def listener(self):
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback)

        rospy.spin()

    def publisher(self):
        pub = rospy.Publisher('pointcloud_data')



if __name__ == '__main__':
    a =  read_pcd()
    a.listener()




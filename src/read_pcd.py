#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def callback(data):

    b = pc2.read_points_list(data)
    print(len(b))
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()



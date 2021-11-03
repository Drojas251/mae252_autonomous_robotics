#!/usr/bin/env python3
from numpy.core.fromnumeric import shape, size
import rospy
#import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import math

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

def callback(data):
    pc = ros_numpy.numpify(data)
    points=np.zeros((len(pc)*len(pc[0]),3))
    count = 0
    print(pc[0])
    print(pc[1])
    
    print(pc[2])
    print(pc[200])
            


rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/velodyne_points", PointCloud2, callback)
rospy.spin()
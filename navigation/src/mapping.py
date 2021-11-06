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
    #Create numpy array
    pc = ros_numpy.numpify(data)

    #attempt to find the different rgb colors for each channel
    #or the indexing of the z value for arrays
    amount = len(pc) #Check if amount = samples = 440
    print("there are", amount, "samples being taken")
    distribution = round(amount/16)
    print("Point cloud looks like \n",pc[::distribution,2:])

    #Find the cutoff point for channels to filter lidar data
    #Using either color or hieght
    cutoffColor = 0
    cutoffHieght = 0
    colorValue = np.array([255,255,255])
    hiehgtValue = .2 #Turtlebot is 192mm. This assumes data
                     #This assumes Velodyne data is in meters

    #Color Search
    for i in range(len(pc)):
        if pc[i,3] == colorValue[0]:
            print(pc[i,3:])
            break
        else:
            cutoffColor = i
    
    #Height Search
    for i in range(len(pc)):
        if pc[i,2] == hiehgtValue:
            print(pc[i,2])
            break
        else:
            cutoffHieght = i

    #This assumes that pc index increases as hieght increases
    pc_filtered_color = pc[:cutoffColor,:3]
    pc_filtered_height = pc[pc[:cutoffHieght,:3]]


            


rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/velodyne_points", PointCloud2, callback)
rospy.spin()
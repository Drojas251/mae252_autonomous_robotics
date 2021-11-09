#!/usr/bin/env python3
from numpy.core.fromnumeric import shape, size
import rospy
#import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import math

from gazebo_msgs.msg import LinkStates

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose


class Mapping(object):

    def __init__(self):

        self.get_point_cloud = rospy.Subscriber("/velodyne_points", PointCloud2, self.point_cloud_process_callback)
        self.get_turtlebot_pose = rospy.Subscriber("/gazebo/link_states", LinkStates, self.get_turtlebot_pose)
        

    def point_cloud_process_callback(self,data):
        #Create numpy array
        pc = ros_numpy.numpify(data)

        cut_off_height = 0.2
        laser_hieght = 0.25

        # dimensions of map 
        X = 10 #meters
        Y = 2 # meters

        # resolution of cells in map
        delta_x = 0.05 # meters
        delta_y = 0.05 # meters 
        map = np.zeros((int(Y/delta_y), int(X/delta_x))) # create map

        # Get turtlebot location ("SLAM")
        pose = Pose()
        pose = self.turtle_bot_pose


        for i in range(len(pc)):
            new_height = pc[i][2] + laser_hieght # pc is wrt lidar. need points in world frame

            if  new_height < cut_off_height and new_height > 0.05:
                # Transform to world points
                World_x = pc[i][0] + pose.position.x
                World_y = pc[i][1] + pose.position.y

                if World_x > 0 and World_x < 9:
                    if World_y > 0 and World_y < 1.75:

                        # get element in the map where the point would belong
                        element_x = round(World_x/delta_x)
                        element_y = round(World_y/delta_y)

                        # store in map
                        map[[int(element_y) ], [int(element_x)]] = 1


    def get_turtlebot_pose(self,msg):
        # gets turtlebot pose in gazebo
        self.turtle_bot_pose = Pose()
        self.turtle_bot_pose = msg.pose[5]
        
        
def main():
    rospy.init_node('listener', anonymous=True)
    mapping = Mapping()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut down")

if __name__ == '__main__':
    main()    
#!/usr/bin/env python3
import rospy
import numpy as np
import ros_numpy
import math
from queue import PriorityQueue

from sensor_msgs.msg import PointCloud2
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from a_star.a_star import *


#### @Diego, is this a proper place to define a function or should it be
#### defiend in the Mapping() class? If it is defined in the mapping class 
#### how do I call it?
def euler_from_quaternion(orientation_msg):
    #calculate nume and denom of components inside of atan for euler cordinates
    numerator = 2.0 * (orientation_msg.w * orientation_msg.z + orientation_msg.x * orientation_msg.y)
    denominator = 1.0 - 2.0 * (orientation_msg.y * orientation_msg.y + orientation_msg.z * orientation_msg.z)
    yaw_z = math.atan2(numerator, denominator)
    
    #yaw is rotation around z in radians (counterclockwise)
    return yaw_z # in radians

class Mapping():
    
    def __init__(self):
        self.get_pc = rospy.Subscriber("/passthrough/output", PointCloud2, self.pc_callback,queue_size = 1)
        self.get_pos = rospy.Subscriber("/gazebo/link_states", LinkStates, self.pos_callback,queue_size = 1)
        self.pub = rospy.Publisher("worldmap", OccupancyGrid, queue_size = 1)
        self.path = rospy.Publisher("path", Path, queue_size = 1)
        self.rate = rospy.Rate(1)

        #map dimensions
        self.X = 5 #meters
        self.Y = 2.5 # meters
        self.resolution = 0.01 # meters
        self.world_map = np.zeros((int(self.Y/self.resolution), int(self.X/self.resolution)))

        self.updated_w_map = OccupancyGrid()
        self.updated_w_map.info.resolution = self.resolution # resolution = 1/resolution
        self.updated_w_map.info.width = int(self.X/self.resolution)   # self.X #length of tunnel (depends on tunnel orientation)
        self.updated_w_map.info.height = int(self.Y/self.resolution)   #self.Y #width of tunnel (depends on tunnel orientation)
        self.updated_w_map.info.origin.position.x = 0 #map orgin in gazebo world
        self.updated_w_map.info.origin.position.y = 0
        self.updated_w_map.info.origin.position.z = 0
        self.updated_w_map.header.frame_id = "odom"


        self.grid = make_grid(int(self.Y/self.resolution),int(self.X/self.resolution),self.resolution)
        self.end = self.grid[120][400]
        self.PURPLE = (128, 0, 128)

        self.id = 0

        ###### @Diego, This information is related to setting up the occupancy grid
        ###### message. Not confident that this is the correct placement
        #publish world map info

    def xy_ij(self,x,y):
        x = round(x,2)
        y = round(y,2)
        j = (x/self.resolution) + 1   # col
        i = (self.Y - (y/self.resolution)) - 1  # row
        return i,j   

    

    def pos_callback(self, data):
        #get robot x and y cords in world map followed by orientation in world map
        self.pos_x = data.pose[5].position.x
        self.pos_y = data.pose[5].position.y
        self.theta = euler_from_quaternion(data.pose[5].orientation)
        
        #define 2D transform matrix
        self.h_transform = np.array([math.cos(self.theta),-math.sin(self.theta),self.pos_x, 
                                    math.sin(self.theta),math.cos(self.theta),self.pos_y,
                                    0 ,             0,              1]).reshape(3,3)


    def pc_callback(self, data):
        
        pc = ros_numpy.numpify(data)

        for i in range(0,pc.size):
            #identify cords for each filtered lidar point from robot
            object_x = pc[i][0]
            object_y = pc[i][1]
            object_z = pc[i][2] 
            robot_frame_object = np.array([object_x, object_y, 1]).reshape(3,1)
            dist_away = math.sqrt(object_x**2+object_y**2)

            #filter lidar points to remove ground points in distance
            #if dist_away<=2.5 and object_z > 0.075:
            if dist_away<=3.5:    
                #change lidar object points to world cordinate system
                worldtf = np.dot(self.h_transform,robot_frame_object)

                wx = self.pos_x + math.cos(self.theta)*object_x + (-math.sin(self.theta)*object_y)
                wy = self.pos_y + math.sin(self.theta)*object_x + math.cos(self.theta)*object_y

                if wx >=0 and wy >=0:

                    # get element in the map where the point would belong
                    #element_x = round(worldtf[0][0]/self.resolution,2)
                    #element_y = round(worldtf[1][0]/self.resolution,2)

                    #ii, jj = self.xy_ij(wx,wy)
                    #print("i",ii)
                    #print('j',jj)

                    element_x = round(wx/self.resolution,2)
                    element_y = round(wy/self.resolution,2)

                    self.world_map[[int(element_y) ], [int(element_x)]] = 100
                    barrier = self.grid[int(element_y)][int(element_x)] 
                    barrier.make_barrier()


                ##remove elements outside of worldmap indexing (AKA negatives)
                #if (element_x) >= 0 & (element_y >=0):
                #    self.world_map[[int(element_y) ], [int(element_x)]] = 100

        self.updated_w_map.header.stamp = rospy.Time.now()
        arr = Int8()
        arr.data = self.world_map.ravel().tolist()
        for i in range(0,len(arr.data)):
                arr.data[i] = int(arr.data[i])
        self.updated_w_map.data = arr.data #input data

        self.updated_w_map.header.stamp = rospy.Time.now()
        self.pub.publish(self.updated_w_map)

        """


        start_x = round(self.pos_x/self.resolution,2)
        start_y = round(self.pos_y/self.resolution,2)

        start = None
        start = self.grid[int(start_y)][int(start_x)]
        start.make_start() # make start pos = robot current pos

        for row in self.grid: # main algorithm
            for spot in row:
                spot.update_neighbors(self.grid)

        algorithm(self.grid, start, self.end)

        pathx = []
        pathy = []

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"
        path.header.seq = self.id

        r = 0
        for row in self.grid:
            c = 0
            for spot in row:
                if spot.color == self.PURPLE: 
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "odom"

                    pose.pose.position.x = c*self.resolution
                    pose.pose.position.y = r*self.resolution
                    pose.pose.position.z = 0
                    pose.pose.orientation.w = 1

                    path.poses.append(pose)

                    pathx.append(c)
                    pathy.append(r)
                    spot.reset()
                c = c +1
            r = r +1
                    
        self.path.publish(path)
        
        start.reset()
        print("new path")
        """







                #Troubleshooting purposes
                #np.set_printoptions(threshold = np.inf)
                #print("x: ",str(element_x), " y: ", str(element_y))
                # store in map
        
                #Troubleshooting purposes
                #print(self.world_map)
                #print("*****************************")

        
        

                


def main():
    rospy.init_node('listener', anonymous=True)
    #rospy.init_node('talker', anonymous = True)

    mapping = Mapping()

    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("shut down")

if __name__ == "__main__":
    main()

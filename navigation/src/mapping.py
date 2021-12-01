#!/usr/bin/env python3
import rospy
import numpy as np
import ros_numpy
import math

from sensor_msgs.msg import PointCloud2
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8


def euler_from_quaternion(orientation_msg):
    #calculate nume and denom of components inside of atan for euler cordinates
    numerator = 2.0 * (orientation_msg.w * orientation_msg.z + orientation_msg.x * orientation_msg.y)
    denominator = 1.0 - 2.0 * (orientation_msg.y * orientation_msg.y + orientation_msg.z * orientation_msg.z)
    yaw_z = math.atan2(numerator, denominator)
    
    #yaw is rotation around z in radians (counterclockwise)
    return yaw_z # in radians


class Mapping():
    
    def __init__(self):
        # Intialize as subscriber and publisher
        self.get_pc = rospy.Subscriber("/passthrough/output", PointCloud2, self.pc_callback)
        self.get_pos = rospy.Subscriber("/gazebo/link_states", LinkStates, self.pos_callback)
        self.pub = rospy.Publisher("worldmap", OccupancyGrid, queue_size = 3)
        self.rate = rospy.Rate(1)

        # map dimensions
        self.X = 10 #meters
        self.Y = 3 # meters
        self.resolution = .01 # meters
        self.world_map = np.zeros((int(self.Y/self.resolution), int(self.X/self.resolution)))
        
        # define occupancy grid message header and data info
        self.updated_w_map = OccupancyGrid()
        self.updated_w_map.info.resolution = 1/self.resolution # resolution = 1/resolution
        self.updated_w_map.info.width = int(self.X * (1/self.resolution))  # length of tunnel (depends on tunnel orientation)
        self.updated_w_map.info.height = int(self.Y * (1/self.resolution))  # width of tunnel (depends on tunnel orientation)
        self.updated_w_map.info.origin.position.x = 0 # map orgin in gazebo world
        self.updated_w_map.info.origin.position.y = 0  # Note that thisis pose msg format
        self.updated_w_map.info.origin.position.z = 0
        self.updated_w_map.header.frame_id = "odom"

        while not rospy.is_shutdown():
  
            # publish occupancy grid from numpy info with corresponding timestamp
            self.updated_w_map.header.stamp = rospy.Time.now()
            
            map_conversion = Int8()  # make instance of ros Int8 message
            map_conversion.data = self.world_map.ravel().tolist()  # flatten array into list
            for i in range(0,len(map_conversion.data)):
                    map_conversion.data[i] = int(map_conversion.data[i])  #convert float to int
            self.updated_w_map.data = map_conversion.data #input data

            self.pub.publish(self.updated_w_map)
            rospy.Rate.sleep(self.rate)

        return;
    

    def pos_callback(self, data):
        #get robot x and y cords in world map followed by orientation in world map
        pos_x = data.pose[5].position.x
        pos_y = data.pose[5].position.y
        theta = euler_from_quaternion(data.pose[5].orientation)
        
        #define 2D transform matrix
        self.h_transform = np.array([math.cos(theta),-math.sin(theta),pos_x, 
                                    math.sin(theta),math.cos(theta),pos_y,
                                    0 ,             0,              1]).reshape(3,3)


    def pc_callback(self, data):

        barrier_buffer = 9  # in cm (units determined by 1m*self.resolution)
        
        # convert to numpy for manipulation
        pc = ros_numpy.numpify(data)

        for i in range(0,pc.size):
            #identify cords for each filtered lidar point from robot
            object_x = pc[i][0]
            object_y = pc[i][1]
            robot_frame_object = np.array([object_x, object_y, 1]).reshape(3,1)
            dist_away = math.sqrt(object_x**2+object_y**2)

            #filter lidar points to remove ground points in distance
            if dist_away <= 3:  # 3 meters
                #change lidar object points to world cordinate system
                worldtf = np.dot(self.h_transform,robot_frame_object)

                # get element in the map where the point would belong
                element_x = int(round(worldtf[0][0]/self.resolution))
                element_y = int(round(worldtf[1][0]/self.resolution))

                #remove elements outside of worldmap indexing (AKA negatives)
                if (element_x >= 0) & (element_y >=0) & element_x <= (self.X*self.resolution) & element_y <= (self.Y*self.resolution):
                    for i in range(element_x - barrier_buffer, element_x + barrier_buffer):
                        for j in range(element_y - barrier_buffer, element_y + barrier_buffer):
                            if (abs(i-element_x)**2 + abs(j-element_y)**2)**.5 < barrier_buffer and i >=0 and j >= 0:

                                self.world_map[[j], [i]] = 100
                    

                
def main():
    rospy.init_node('listener', anonymous=True)
    #rospy.init_node('talker', anonymous = True)

    mapping = Mapping() # make instance of mapping class

    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("shut down")

if __name__ == "__main__":
    main()

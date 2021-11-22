#!/usr/bin/env python3
import rospy
import numpy as np
import ros_numpy
import math

from sensor_msgs.msg import PointCloud2
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import OccupancyGrid


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
        self.get_pc = rospy.Subscriber("/passthrough/output", PointCloud2, self.pc_callback)
        self.get_pos = rospy.Subscriber("/gazebo/link_states", LinkStates, self.pos_callback)
        self.pub = rospy.Publisher("worldmap", OccupancyGrid, queue_size = 3)
        self.rate = rospy.Rate(1)

        #map dimensions
        self.X = 10 #meters
        self.Y = 3 # meters
        self.resolution = .01 # meters
        self.world_map = np.zeros((int(self.Y/self.resolution), int(self.X/self.resolution)))

        ###### @Diego, This information is related to setting up the occupancy grid
        ###### message. Not confident that this is the correct placement
        #publish world map info
        

        while not rospy.is_shutdown():
  
           
            
            #print(self.updated_w_map)

            self.updated_w_map = OccupancyGrid()
            #print(self.updated_w_map)
            self.updated_w_map.info.resolution = 1/self.resolution # resolution = 1/resolution
            self.updated_w_map.info.width = self.X #length of tunnel (depends on tunnel orientation)
            self.updated_w_map.info.height = self.Y #width of tunnel (depends on tunnel orientation)
            self.updated_w_map.info.origin.position.x = 0 #map orgin in gazebo world
            self.updated_w_map.info.origin.position.y = 0
            self.updated_w_map.info.origin.position.z = 0
            self.updated_w_map.data = self.world_map.ravel() #input data
            self.updated_w_map.header.stamp = rospy.Time.now()
            self.updated_w_map.header.frame_id = "octo-map"
            #self.pub.publish(self.updated_w_map)
            #print(self.updated_w_map)

            self.updated_w_map.data = self.world_map.tolist() #input data
            #print(type(self.world_map.tolist()))
            self.updated_w_map.header.stamp = rospy.Time.now()
            self.pub.publish(self.updated_w_map.data)
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
        
        pc = ros_numpy.numpify(data)

        for i in range(0,pc.size):
            #identify cords for each filtered lidar point from robot
            object_x = pc[i][0]
            object_y = pc[i][1]
            robot_frame_object = np.array([object_x, object_y, 1]).reshape(3,1)
            dist_away = math.sqrt(object_x**2+object_y**2)

            #filter lidar points to remove ground points in distance
            if dist_away<=4:
                #change lidar object points to world cordinate system
                worldtf = np.dot(self.h_transform,robot_frame_object)

                # get element in the map where the point would belong
                element_x = round(worldtf[0][0]/self.resolution)
                element_y = round(worldtf[1][0]/self.resolution)

                #remove elements outside of worldmap indexing (AKA negatives)
                if (element_x) >= 0 & (element_y >=0):
                    self.world_map[[int(element_y) ], [int(element_x)]] = int(1)
                    
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

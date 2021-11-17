#!/usr/bin/env python3
#from numpy.core.fromnumeric import shape, size
import rospy
import numpy as np
#import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import math

from sensor_msgs.msg import PointCloud2
from gazebo_msgs.msg import LinkStates


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    # t0 = +2.0 * (w * x + y * z)
    # t1 = +1.0 - 2.0 * (x * x + y * y)
    # roll_x = math.atan2(t0, t1)
    
    # t2 = +2.0 * (w * y - z * x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < -1.0 else t2
    # pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z # in radians

class Mapping():
    
    def __init__(self):
        self.get_pc = rospy.Subscriber("/passthrough/output", PointCloud2, self.pc_callback)
        self.get_pos = rospy.Subscriber("/gazebo/link_states", LinkStates, self.pos_callback)

        #map dimensions
        X = 12 #meters
        Y = 3 # meters
        self.delta_x = 0.1 # meters
        self.delta_y = 0.1 # meters 
        print(self.delta_x, self.delta_y)
        self.world_map = np.zeros((int(Y/self.delta_y), int(X/self.delta_x)))
    


    def pos_callback(self, data):
        # self.turtle_bot_pose = Pose()
        # self.turtle_bot_pose = data.pose[5].position.x
        theta = euler_from_quaternion(data.pose[5].orientation.x, data.pose[5].orientation.y,
                                      data.pose[5].orientation.z, data.pose[5].orientation.w)
        pos_x = data.pose[5].position.x
        pos_y = data.pose[5].position.y
        #print("x: ", str(pos_x), "y : ", str(pos_y), " theta: ", str(theta))
        #print("angle: ", str(theta), " position:",str(pos_x)," ", str(pos_y))
        #print(euler_from_quaternion(theta2.x, theta2.y, theta2.z, theta2.w))
        print(theta)
        #define transform matrix
        self.h_transform = np.array([math.cos(theta),-math.sin(theta),pos_x, math.sin(theta),math.cos(theta),pos_y,0 , 0, 1]).reshape(3,3)
        #print(self.h_transform)


    def pc_callback(self, data):
        
        pc = ros_numpy.numpify(data)
         # create map

        for i in range(0,pc.size):
            object_x = pc[i][0]
            object_y = pc[i][1]
            robot_frame_object = np.array([object_x, object_y, 1]).reshape(3,1)
            dist_away = math.sqrt(object_x**2+object_y**2)
            if dist_away<=4:
                worldtf = np.dot(self.h_transform,robot_frame_object)
                #print("world:\n",str(worldtf),"\nrobot:\n",str(robot_frame_object))
                # get element in the map where the point would belong
                element_x = round(worldtf[0][0]/self.delta_x)
                element_y = round(worldtf[1][0]/self.delta_y)


                #print("x: ",str(element_x), " y: ", str(element_y))
                # store in map
                self.world_map[[int(element_y) ], [int(element_x)]] = 1
                np.set_printoptions(threshold = np.inf)
                # print("*****************************")
                # print(self.world_map)
                


def main():
    rospy.init_node('listener', anonymous=True)

    mapping = Mapping()

    try:
        rospy.spin()
        print("spinning")
    except KeyboardInterrupt:
        print("shut down")

if __name__ == "__main__":
    main()

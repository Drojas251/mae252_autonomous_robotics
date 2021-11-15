#!/usr/bin/env python3
from numpy.core.fromnumeric import shape, size
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import math

# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
# from geometry_msgs.msg import PoseArray
# from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist


class Mapping():
    
    def __init__(self):
        self.get_pc = rospy.Subscriber("/passthrough/output", PointCloud2, self.pc_callback)
        self.get_pos = rospy.Subscriber("/gazebo/link_states", Pose, self.pos_callback)
        #self.get_twist = rospy.Subscriber("/gazebo/link_states", Twist, self.twist_callback)
    
    def pos_callback(self, data):
        theta = data.orientation.z
        pos_x = data.position.x
        pos_y = data.position.y
        print("angle: ", str(theta), " position:",str(pos_x)," ", str(pos_y))

        #define transform matrix
        self.h_transform = np.array([math.cos(theta),math.sin(theta),pos_x],
                                [math.sin(theta),math.cos(theta),pos_y],
                                [0         , 0        , 1   ])
        #print(self.h_transform)




    def pc_callback(self, data):
        
        pc = ros_numpy.numpify(data)
         # create map
        print(data)
        for i in range(0,pc.size):
            object_x = pc[i][0]
            object_y = pc[i][1]
            robot_frame_object = np.array([object_x, object_y, 0]).reshape(1,3)
            dist_away = math.sqrt(object_x^2+object_y^2)
            print(dist_away)
            if dist_away<=3:
                worldtf = np.dot(self.h_transform,robot_frame_object)
                



        # print("filtered PC:")
        # for i in range(0,pc.size):
        #     print('x: ', str(pc[i][0]), ' y: ', str(pc[i][1]), ' z: ', str(pc[i][2]))

def main():
    rospy.init_node('listener', anonymous=True)


    #map dimensions
    X = 10 #meters
    Y = 2 # meters
    delta_x = 0.05 # meters
    delta_y = 0.05 # meters 
    world_map = np.zeros((int(Y/delta_y), int(X/delta_x)))
    mapping = Mapping()

    try:
        rospy.spin()
        print("spinning")
    except KeyboardInterrupt:
        print("shut down")

if __name__ == "__main__":
    main()
    print("go")

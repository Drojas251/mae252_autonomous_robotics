#!/usr/bin/env python3
import rospy
import numpy as np
import math
from queue import PriorityQueue

from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from a_star.a_star import *



class PathPlanning():
    
    def __init__(self):
        self.get_pc = rospy.Subscriber("/worldmap", OccupancyGrid, self.path_planning_callback,queue_size = 1)
        self.get_pos = rospy.Subscriber("/gazebo/link_states", LinkStates, self.pos_callback,queue_size = 1)
        self.path = rospy.Publisher("path", Path, queue_size = 1)
        self.waypoint = rospy.Publisher("/waypoint", Pose, queue_size = 1)
        self.marker = rospy.Publisher('/marker', Marker, queue_size=1)

        #map dimensions
        self.X = 4 #meters
        self.Y = 1 # meters
        self.resolution = 0.01 # meters

        self.grid = make_grid(int(self.Y/self.resolution),int(self.X/self.resolution),self.resolution)
        #self.waypoints = np.array([50,50],[150,300])
        self.PURPLE = (128, 0, 128)
        #self.end = self.grid[50][150]

        self.id = 0 


    def pos_callback(self, data):
        #get robot x and y cords in world map followed by orientation in world map
        self.pos_x = data.pose[2].position.x
        self.pos_y = data.pose[2].position.y   


    def path_planning_callback(self, data):
     
        array_data = np.asarray(data.data)
        world_map = array_data.reshape(int(self.Y/self.resolution),int(self.X/self.resolution))
        #print("1",len(world_map))
        #print("2",len(world_map[0]))


        start_x = round(self.pos_x/self.resolution,2)
        start_y = round(self.pos_y/self.resolution,2)

        if start_x < 100:
            self.end = self.grid[50][180]
            print("Moving to first_way_point")
        elif start_x > 100 and start_x < 250:
            self.end = self.grid[80][250]
            print("Moving to Second_way_point")
        else:
            self.end = self.grid[80][400]
            print("Moving to Third_way_point")
        


        start = None
        start = self.grid[int(start_y)][int(start_x)]
        start.make_start() # make start pos = robot current pos

        for r in range(len(world_map)):
            for c in range(len(world_map[0])):
                if world_map[r][c] == 100:
                    barrier = self.grid[int(r)][int(c)] 
                    barrier.make_barrier()

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

        waypoint = Pose()
        waypoint.position.x = pathx[15]*self.resolution
        waypoint.position.y = pathy[15]*self.resolution
        waypoint.position.z = 0
        waypoint.orientation.w = 1.0

        maker_object = Marker()
        maker_object.header.frame_id = 'odom'
        maker_object.header.stamp = rospy.get_rostime()
        maker_object.id = 1
        maker_object.type = Marker.SPHERE
        maker_object.action = Marker.ADD

        maker_object.pose.position = waypoint.position
        maker_object.pose.orientation = waypoint.orientation

        maker_object.scale.x = 0.05
        maker_object.scale.y = 0.05
        maker_object.scale.z = 0.05

        maker_object.color.a = 1.0

        maker_object.lifetime = rospy.Duration(0)
        self.marker.publish(maker_object)



        self.waypoint.publish(waypoint)
        
        start.reset()
        print("new path")




def main():
    rospy.init_node('pathplanning', anonymous=True)
    #rospy.init_node('talker', anonymous = True)

    pathplanning = PathPlanning()

    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("shut down")

if __name__ == "__main__":
    main()

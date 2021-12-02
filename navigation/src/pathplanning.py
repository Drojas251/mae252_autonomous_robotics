#!/usr/bin/env python3
from ctypes import pointer
from sys import path
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
from a_star.Bezier import Bezier

def h(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)


def make_grid(rows, col,width):
	grid = []
	gap = width // rows
	for i in range(rows):
		grid.append([])
		for j in range(col):
			spot = Spot(i, j, gap, rows,col)
			grid[i].append(spot)

	return grid

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
        self.resolution = 0.02 # meters

        #self.waypoints = np.array([50,50],[150,300])
        self.PURPLE = (128, 0, 128)
        #self.end = self.grid[50][150]

        self.id = 0 



    def pos_callback(self, data):
        #get robot x and y cords in world map followed by orientation in world map
        self.pos_x = data.pose[2].position.x
        self.pos_y = data.pose[2].position.y   

    def reconstruct_path(self,came_from, current):

        path_x = []
        path_y = []
        while current in came_from:
            current = came_from[current]
            current.make_path()
            path_x.append(current.col)
            path_y.append(current.row)
        self.pathx = path_x
        self.pathy = path_y


    def algorithm(self,grid, start, end):
        count = 0
        open_set = PriorityQueue()
        open_set.put((0, count, start))
        came_from = {}
        g_score = {spot: float("inf") for row in grid for spot in row}
        g_score[start] = 0
        f_score = {spot: float("inf") for row in grid for spot in row}
        f_score[start] = h(start.get_pos(), end.get_pos())
        px = []
        py = []
        open_set_hash = {start}

        while not open_set.empty():
            #for event in pygame.event.get():
            #	if event.type == pygame.QUIT:
            #		pygame.quit()

            current = open_set.get()[2]
            open_set_hash.remove(current)

            if current == end:          
                self.reconstruct_path(came_from, end)
                end.make_end()
                return True

            for neighbor in current.neighbors:
                temp_g_score = g_score[current] + 1

                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)
                        neighbor.make_open()

            #draw()

            if current != start:
                current.make_closed()

        return False



    def path_planning_callback(self, data):
     
        array_data = np.asarray(data.data)
        world_map = array_data.reshape(int(self.Y/self.resolution),int(self.X/self.resolution))

        start_x = round(self.pos_x/self.resolution)
        start_y = round(self.pos_y/self.resolution)

        grid = make_grid(int(self.Y/self.resolution),int(self.X/self.resolution),self.resolution)
        end = grid[40][180]
        start = None
        start = grid[int(start_y)][int(start_x)]
        start.make_start() # make start pos = robot current pos

        for r in range(len(world_map)):
            for c in range(len(world_map[0])):
                if int(world_map[r][c]) > 0:
                    barrier = grid[int(r)][int(c)] 
                    barrier.make_barrier()

                    #print('r',r)
                    #print(c)

        for row in grid: # main algorithm
            for spot in row:
                spot.update_neighbors(grid)

        self.algorithm(grid, start, end)

        smooth_path = False # set to False for no smooth path

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"
        path.header.seq = self.id

        if smooth_path == True:
            #** Code for getting a smooth path ** 

            path_length = len(self.pathx)
            points_on_path = np.zeros((path_length, 2))

            for k in range(path_length):
                c = self.pathx[path_length - 1 -k]
                x = c*self.resolution

                r = self.pathy[path_length - 1 -k]
                y = self.resolution*(r)

                points_on_path[k,0] = x
                points_on_path[k,1] = y

            # create smooth path using bezier curve
            t_points = np.arange(0, 1, 0.02)
            curve_set = Bezier.Curve(t_points, points_on_path)

            for i in range(len(curve_set)):
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "odom"

                pose.pose.position.x = curve_set[i,0]
                pose.pose.position.y = curve_set[i,1]
                pose.pose.position.z = 0
                pose.pose.orientation.w = 1

                path.poses.append(pose)

            self.path.publish(path)

            pl = len(curve_set)
            waypoint = Pose()
            waypoint.position.x = curve_set[round(pl/6),0]
            waypoint.position.y = curve_set[round(pl/6),1]
            waypoint.position.z = 0
            waypoint.orientation.w = 1.0

        else:
            path_length = len(self.pathx)
            pointx = []
            pointy = []

            for k in range(path_length):
                c = self.pathx[path_length - 1 -k]
                x = c*self.resolution

                r = self.pathy[path_length - 1 -k]
                y = self.resolution*(r)

                pointx.append(x)
                pointy.append(y)

                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "odom"

                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0
                pose.pose.orientation.w = 1

                path.poses.append(pose)

            self.path.publish(path)

            pl = len(pointx)
            waypoint = Pose()
            waypoint.position.x = pointx[5]
            waypoint.position.y = pointy[5]
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
        #print("new path")




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

#! /usr/bin/python3.8

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#testing moving from one point to another 


global roll,pitch,yaw 

def waypoint_callback(waypoint_msg):
    global goal_x, goal_y
    goal_x = waypoint_msg.pose.pose.position.x
    goal_y = waypoint_msg.pose.pose.position.y 

def odom_callback(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print (msg.pose.pose.position.x, msg.pose.pose.position.y)
    dx = goal_x - msg.pose.pose.position.x
    dy = goal_y - msg.pose.pose.position.y
    theta= math.atan(dy/dx)
    print (yaw,theta)
    if(dx<0):
        linear_vel = -1
    elif(dx>0):
        linear_vel = 1
    else: 
        if(dy>0):
            linear_vel = 1
        elif(dy<0):
            linear_vel = -1
    
    if(dx < 0.2 and dx > -0.2):
        if(dy < 0.02 and dy > -0.02):
            linear_vel = 0
            angular_vel = 0
            print("reached")

    move.linear.x = 0.1*linear_vel

    if(theta > 1.56 or theta < -1.56):
        theta = 1.57
        print("theta adjusted")
    
    if (yaw>theta+0.02):
        move.angular.z = -0.1 
    elif (yaw<theta-0.02):
        move.angular.z = 0.1 
    else:
       move.angular.z = 0.0

rospy.init_node('move_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber('/odom', Odometry, odom_callback)
waypoint_sub = rospy.Subscriber('/waypoint', Odometry, waypoint_callback, queue_size = 1)
rate = rospy.Rate(2)
move = Twist()


while not rospy.is_shutdown():
    pub.publish(move)
    rate.sleep()

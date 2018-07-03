#!/usr/bin/env python  
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

dest_x = 5.0
dest_y = 3.0
k = 3

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

def callback(msg):
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    
    twist = Twist()

    if(dest_x - current_x > 0):
	twist.linear.x = k
    elif(dest_x - - current_x < 0):
	twist.linear.x = -k
    else:
	twist.linear.x = 0

    if(dest_y - current_y > 0):
	twist.linear.y = k
    elif(dest_y - - current_y < 0):
	twist.linear.y = -k
    else:
	twist.linear.y = 0

    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    

rospy.init_node('move')
sub = rospy.Subscriber('odom', Odometry, callback)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("deu erro")


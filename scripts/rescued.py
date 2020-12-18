#!/usr/bin/env python3

"""Rescued robot that waits until it detecs it rescuer, then starts node to follow"""
import sys
import rospy
import numpy as np
import time
import os
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from squaternion import Quaternion

def communicate(send):
    """Send message on /comm"""
    pub = rospy.Publisher("/comm", String, queue_size=0)
    target = String()
    target.data = "S " + send
    
    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)
    print("RESCUER FOUND")
    print("SLAVE SENT: " , target)

    os.system('roslaunch maze_solver follower.launch')

def comm_callback(data):
    """Reads messages from slave"""
    
    print("SLAVE RECEIVED: " , data)

    message = str(data)
    message = message.split()
    if message[0] == "S":
        pass
    else:
    # Do something here
        pass

def laser_callback(data):
    """Updates laser scan data"""

    global ranges
    ranges = data.ranges

    global old_ranges
    if len(old_ranges) < 2:
        old_ranges = ranges

    global robot_angle

    count = 0

    global detected_status 
    for i in range(360):
        if abs(ranges[i] - old_ranges[i]) > .1:
            count = count + 1

    if count > 1:
        if detected_status == "Undetected":
            detected_status = "Detected"
            communicate("Hello")

    old_ranges = ranges

if __name__ == '__main__':

    rospy.init_node("Rescued", anonymous=False)

    try:

        global detected_status
        detected_status = "Undetected"

        global old_ranges
        old_ranges = np.empty(1) 
        
        rospy.wait_for_message("/" + str(sys.argv[1]) + "/scan", LaserScan)

        rospy.Subscriber("/comm", String, comm_callback)
        rospy.Subscriber("/" + str(sys.argv[1]) + "/scan", LaserScan, laser_callback)

        # Prevents node from ending
        while not rospy.is_shutdown():
            rospy.spin()




    except rospy.ROSInterruptException:
        pass

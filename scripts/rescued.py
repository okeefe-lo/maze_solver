#!/usr/bin/env python3

"""Controls the movement of the robot"""
import sys
import rospy
import numpy as np
import time
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from squaternion import Quaternion

def communicate(send):

    pub = rospy.Publisher("/comm", String, queue_size=0)
    target = String()
    target.data = "S " + send
    
    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)

def comm_callback(data):

    message = data.split()
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

    global detected_status 
    if detected_status == "Undetected":
        for i in range(360):
            if abs(ranges[i] - old_ranges[i]) > .1:
                detected_status = "Detected"
                robot_angle = i

    old_ranges = ranges

if __name__ == '__main__':

    rospy.init_node("Rescued", anonymous=False)

    try:

        global detected_status
        detected_status = "Undetected"

        global old_ranges
        old_ranges = np.empty(1) 
        
        rospy.Subscriber("/comm", String, comm_callback)
        rospy.Subscriber("/" + str(sys.argv[1]) + "/scan", LaserScan, laser_callback)

        pub = rospy.Publisher("/" + str(sys.argv[1]) + "/cmd_vel", Twist, queue_size=0)
        rate = rospy.Rate(20)

        data = Twist()

        while not rospy.is_shutdown():

            if detected_status == "Undetected":
                new_fwd_vel = 0
                new_ang_vel = 0
            else:
                pass

            # Set data to publish
            data.linear.x = new_fwd_vel
            data.linear.y = 0
            data.linear.z = 0
            data.angular.x = 0
            data.angular.y = 0
            data.angular.z = new_ang_vel

            # Publish data
            pub.publish(data)

            rate.sleep()


    except rospy.ROSInterruptException:
        pass
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

if __name__ == '__main__':

    rospy.init_node("Rescued", anonymous=False)

    rospy.Subscriber("/comm", String, comm_callback)


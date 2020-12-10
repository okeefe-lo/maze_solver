#!/usr/bin/env python3

"""Controls the movement of the robot"""
import sys
import rospy
import numpy as np
import time
from math import pi
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from squaternion import Quaternion


if __name__ == '__main__':

    rospy.init_node("Rescuer", anonymous=False)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)

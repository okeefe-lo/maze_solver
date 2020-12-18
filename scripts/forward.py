#!/usr/bin/env python3

"""Moves robot forward with randomness"""
import sys
import rospy
import time
import numpy as np
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from squaternion import Quaternion


if __name__ == '__main__':

    rospy.init_node("Forward", anonymous=False)
    pub = rospy.Publisher("/" + str(sys.argv[1]) + "/cmd_vel", Twist, queue_size=0)

    rospy.wait_for_message("/" + str(sys.argv[1]) + "/odom", Odometry)

    try:

        rate = rospy.Rate(1)
        data = Twist()

        while not rospy.is_shutdown():
            
            # Set data to publish
            data.linear.x = np.random.uniform(0, .22)
            data.linear.y = 0
            data.linear.z = 0
            data.angular.x = 0
            data.angular.y = 0
            data.angular.z = np.random.normal(0, 0.15)


            # Publish data
            pub.publish(data)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

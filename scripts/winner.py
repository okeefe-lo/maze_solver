#!/usr/bin/env python3

"""Chooses winner of king of the hill"""
import sys
import rospy
import time
import numpy as np
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def euclidean(current, goal):
    """Returns euclidean distance betwen two points"""
    return (((current[0]-goal[0]) ** 2) + ((current[1]-goal[1]) ** 2)) ** .5

def odom_callback1(data):
    """Updates position data"""

    global bot1_pose
    bot1_pose = [data.pose.pose.position.x, data.pose.pose.position.y]

def odom_callback2(data):
    """Updates position data"""

    global bot2_pose
    bot2_pose = [data.pose.pose.position.x, data.pose.pose.position.y]

def odom_callback3(data):
    """Updates position data"""

    global bot3_pose
    bot3_pose = [data.pose.pose.position.x, data.pose.pose.position.y]

def odom_callback4(data):
    """Updates position data"""

    global bot4_pose
    bot4_pose = [data.pose.pose.position.x, data.pose.pose.position.y]

if __name__ == '__main__':

    rospy.init_node("Winner", anonymous=False)

    rospy.Subscriber("/" + str(sys.argv[1]) + "/odom", Odometry, odom_callback1)
    rospy.Subscriber("/" + str(sys.argv[2]) + "/odom", Odometry, odom_callback2)
    rospy.Subscriber("/" + str(sys.argv[3]) + "/odom", Odometry, odom_callback3)
    rospy.Subscriber("/" + str(sys.argv[4]) + "/odom", Odometry, odom_callback4)


    rospy.wait_for_message("/" + str(sys.argv[1]) + "/odom", Odometry)
    rospy.wait_for_message("/" + str(sys.argv[2]) + "/odom", Odometry)
    rospy.wait_for_message("/" + str(sys.argv[3]) + "/odom", Odometry)
    rospy.wait_for_message("/" + str(sys.argv[4]) + "/odom", Odometry)


    try:
        rate = rospy.Rate(1)

        global bot1_pose
        global bot2_pose
        global bot3_pose
        global bot4_pose

        while not rospy.is_shutdown():


            dis1 = euclidean(bot1_pose, [0,0])
            dis2 = euclidean(bot2_pose, [0,0])
            dis3 = euclidean(bot3_pose, [0,0])
            dis4 = euclidean(bot4_pose, [0,0])

            index = np.argmin([dis1,dis2,dis3,dis4])

            print("Currently winning is: ", sys.argv[np.argmin([dis1,dis2,dis3,dis4]) + 1])

        rate.sleep()


    except rospy.ROSInterruptException:
        pass

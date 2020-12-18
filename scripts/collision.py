#!/usr/bin/env python3

"""Checks for clear sign of collision"""
import sys
import rospy
import time
import numpy as np
from math import pi
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from squaternion import Quaternion

def imu_callback(data):
    """Updates imu data"""

    global old_linear_acc_imu
    global angular_vel_imu
    global linear_vel_imu

    angular_vel_imu = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]

    linear_vel_imu[0] = np.trapz([old_linear_acc_imu[0], data.linear_acceleration.x], dx=1/200)
    linear_vel_imu[1] = np.trapz([old_linear_acc_imu[1], data.linear_acceleration.y], dx=1/200)
    linear_vel_imu[2] = np.trapz([old_linear_acc_imu[2], data.linear_acceleration.z], dx=1/200)

def odom_callback(data):
    """Updates position/velocity data"""

    global current_pose
    q = Quaternion(data.pose.pose.orientation.x, \
        data.pose.pose.orientation.y, \
        data.pose.pose.orientation.z, \
        data.pose.pose.orientation.w)
    euler = q.to_euler(degrees=False)
    current_pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler[0]]

    global angular_vel_odom
    global linear_vel_odom
    angular_vel_odom = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]
    linear_vel_odom = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]

def command_callback(data):
    """Updates command data"""
    global angular_vel_command
    global linear_vel_command
    global old_linear_vel_command
    global old_angular_vel_command
    old_linear_vel_command = linear_vel_command
    old_angular_vel_command = angular_vel_command
    angular_vel_command = [data.angular.x, data.angular.y, data.angular.z]
    linear_vel_command = [data.linear.x, data.linear.y, data.linear.z]



if __name__ == '__main__':

    rospy.init_node("Collision", anonymous=False)

    global current_pose
    global angular_vel_odom
    global linear_vel_odom
    global angular_vel_command
    global linear_vel_command
    global angular_vel_imu
    global linear_vel_imu

    angular_vel_odom = [0, 0, 0]
    linear_vel_odom = [0, 0, 0]
    angular_vel_command = [0, 0, 0]
    linear_vel_command = [0, 0, 0]
    angular_vel_imu = [0, 0, 0]
    linear_vel_imu = [0, 0, 0]

    global old_linear_acc_imu
    old_linear_acc_imu = [0, 0, 0]

    global old_linear_vel_command
    old_linear_vel_command = [0, 0, 0]

    global old_angular_vel_command
    old_angular_vel_command = [0, 0, 0]

    try:
        rospy.Subscriber("/imu", Imu, imu_callback)
        rospy.Subscriber("/odom", Odometry, odom_callback)
        rospy.Subscriber("/cmd_vel", Twist, command_callback)
    
        error = 0

        rospy.wait_for_message("/odom", Odometry)

        while not rospy.is_shutdown():
            
            # If command velocity is not followed well enough, detect collision
            # Made specifically for d_fwd value in mapping.py
            if abs(old_linear_vel_command[0] - linear_vel_odom[0]) >= .02:
                error = 1

            if error == 1:
                print("COLLISION DETECTED")


        rospy.spin()


    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

"""Controls the movement of the robot"""
import sys
import rospy
import numpy as np
from math import pi
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
from squaternion import Quaternion

def laser_callback(data):
    """Updates laser scan data"""

    global ranges
    ranges = data.ranges

def odom_callback(data):
    """Updates position data"""

    global current_pose
    q = Quaternion(data.pose.pose.orientation.x, \
        data.pose.pose.orientation.y, \
        data.pose.pose.orientation.z, \
        data.pose.pose.orientation.w)
    euler = q.to_euler(degrees=False)
    current_pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler[0]]

def euclidean(current, start):
    return (((current[0]-start[0]) ** 2) + ((current[1]-start[1]) ** 2)) ** .5

def find_nearest(array, value):
    array = np.asarray(array)
    index = (np.abs(array - value)).argmin()
    return array[index]

def controller():
    """Creates a publisher of type twist to move the robot"""
    
    laser_params = rospy.get_param('laser_params')
    # angle_min = laser_params['angle_min']
    # angle_max = laser_params['angle_max']
    # angle_increment = laser_params['angle_increment']
    # range_min = laser_params['range_min']
    # range_max = laser_params['range_max']
    # laser_num = laser_params['laser_num']

    # Create Sub and Pub
    pub = rospy.Publisher('/' + sys.argv[1] + '/cmd_vel', Twist, queue_size=1)

    rospy.wait_for_message('/' + sys.argv[1] + "/scan", LaserScan)

    try:
        # Set rate and intitilize publisher type
        rate = rospy.Rate(20)
        data = Twist()

        new_fwd_vel = 0
        new_ang_vel = 0
        old_fwd_vel = 0
        old_ang_vel = 0

        global current_pose
        global ranges

        start_pose = current_pose
        desired_pose = current_pose

        status = "Finding Wall"
        directions = np.array([0, pi/2, pi, -pi/2, -pi])

        # Angular velocity constant
        k = 1

        # Ideal forward velocity
        d_fwd = .05

        while not rospy.is_shutdown():

            # Check if turning
            diff = desired_pose[2] - current_pose[2]
            if diff >= pi:
                diff = diff - 2 * pi
            if diff < -pi:
                diff = diff + 2 * pi 
            if abs(diff) > .05:
                status = "TURNING"
            
            # Choose what directions to go
            if status == "Finding Wall":
                new_fwd_vel = d_fwd
                print("Looking for Wall")
                if ranges[0] < .18:
                    status = "TURNING"
                    new_fwd_vel = 0
                    new_fwd_vel = 0
                    desired_pose[2] = current_pose[2] - pi/2
                    desired_pose[2] = find_nearest(directions, desired_pose[2])
                    print("Attaching")
                    print("Turn RIGHT")
            elif status == "FORWARD":
                new_fwd_vel = d_fwd
                if ranges[0] < .18:
                    status = "IDENTIFY"
                    new_fwd_vel = 0
                    print("Hitting Wall")
                elif euclidean(current_pose, start_pose) > .3:
                    status = "IDENTIFY"
                    new_fwd_vel = 0
                    new_ang_vel = 0
                    print("Moved .3 meters")
                print("Moving FORWARD")
            elif status == "TURNING":
                new_fwd_vel = 0
                if abs(desired_pose[2] - current_pose[2]) < .01:
                    new_fwd_vel = d_fwd
                    new_ang_vel = 0
                    start_pose = [current_pose[0], current_pose[1]]
                    status = "FORWARD"
                print("Turning")
            else:
                if (ranges[90] > .30) & (ranges[125] > .2): #For .2 dis from wall use [120] and .2
                    status == "TURNING"
                    new_fwd_vel = 0
                    desired_pose[2] = current_pose[2] + pi/2
                    desired_pose[2] = find_nearest(directions, desired_pose[2])
                    diff = desired_pose[2] - current_pose[2]
                    print("Turn LEFT")
                elif ranges[0] < .18:
                    status == "TURNING"
                    new_fwd_vel = 0
                    desired_pose[2] = current_pose[2] - pi/2
                    desired_pose[2] = find_nearest(directions, desired_pose[2])
                    diff = desired_pose[2] - current_pose[2]
                    print("Turn RIGHT")
                else:
                    new_fwd_vel = d_fwd
                    print("Default: Forward")

            diff = desired_pose[2] - current_pose[2]
            if diff >= pi:
                diff = diff - 2 * pi
            if diff < -pi:
                diff = diff + 2 * pi 
            new_ang_vel = k * diff

            print(current_pose[2], "  ",desired_pose[2],"  ", new_ang_vel )

            # Set data to publish
            data.linear.x = new_fwd_vel
            data.linear.y = 0
            data.linear.z = 0
            data.angular.x = 0
            data.angular.y = 0
            data.angular.z = new_ang_vel

            # Publish data
            pub.publish(data)

            old_fwd_vel = new_fwd_vel
            old_ang_vel = new_ang_vel

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':

    rospy.init_node("Mapping", anonymous=False)

    rospy.Subscriber('/' + sys.argv[1] + "/scan", LaserScan, laser_callback)
    rospy.Subscriber('/' + sys.argv[1] + "/odom", Odometry, odom_callback)
        
    controller()

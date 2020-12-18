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
    target.data = "M " + send
    
    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)
    
    print("MASTER SENT: " , target)


def comm_callback(data):

    print("MASTER RECEIVED: " , data)

    message = str(data)
    message = message.split()
    if message[0] == "M":
        pass
    else:
        spawn()

def odom_callback(data):
    """Updates position data"""

    global current_pose
    q = Quaternion(data.pose.pose.orientation.x, \
        data.pose.pose.orientation.y, \
        data.pose.pose.orientation.z, \
        data.pose.pose.orientation.w)
    euler = q.to_euler(degrees=False)
    current_pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler[0]]

def laser_callback(data):
    """Updates laser scan data"""

    global ranges
    ranges = data.ranges

def euclidean(current, start):
    return (((current[0]-start[0]) ** 2) + ((current[1]-start[1]) ** 2)) ** .5

def find_nearest(array, value):
    "Find nearest value in array"

    array = np.asarray(array)
    index = (np.abs(array - value)).argmin()
    return array[index]

def spawn():

    global target_pose
    global spawn_pose

    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    target = PoseStamped()
    target.pose.position.x = spawn_pose[0]
    target.pose.position.y = spawn_pose[1]
    target.pose.orientation.x = 0
    target.pose.orientation.y = 0
    target.pose.orientation.z = 0
    target.pose.orientation.w = 1
    target.header.frame_id = "map"

    target_pose = [spawn_pose[0], spawn_pose[1]]

    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)

    rospy.signal_shutdown("Robot Found: Exiting Maze")

if __name__ == '__main__':

    rospy.init_node("Rescuer", anonymous=False)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)

    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/comm", String, comm_callback)


    rospy.wait_for_message("/scan", LaserScan)

    global current_pose
    global spawn_pose

    try:
        # Set rate and intitilize publisher type
        rate = rospy.Rate(20)
        data = Twist()

        global ranges

        new_fwd_vel = 0
        new_ang_vel = 0
        old_fwd_vel = 0
        old_ang_vel = 0

        desired_pose = current_pose

        # Prior distance to left hand wall
        last_dis = 0

        # Timer before robot considers heading back to spawn
        start_time = 0
        # Spawn location
        spawn_pose = current_pose
        # Location where robot first encountered wall
        first_encounter = current_pose
        # Location where robot finished turning, used to identify minimum move distance after turning (move_dis)
        turn_pose = current_pose
        # Location where robot passes wall on left, used to identify minumum move distance after passing wall (past_wall)
        wall_pose = current_pose

        # Possible Status: Finding Wall, FORWARD, TURNING, SPAWN, IDENTIFY (defaults forward, searches for turns)
        status = "Finding Wall"
        # Possible robot directions, assumes 
        # Location where robot passes wall on90 degree turns only
        directions = np.array([0, pi/2, pi, -pi/2, -pi])

        # Parameters to change
        # Angular velocity constant
        k = .75
        # Ideal forward velocity
        d_fwd = .02
        # Front clearence
        clearence = .17
        # Back left clearence to ensure robot will not hit wall when turning left (clearence is based off of 125 degrees left of robot center)
        left_clearence = .17
        # Distance to move after turning
        move_dis = .25
        # Distance to move after passing wall before turning
        past_wall = .075

        while not rospy.is_shutdown():

            # Check if turning
            diff = desired_pose[2] - current_pose[2]
            if diff >= pi:
                diff = diff - 2 * pi
            if diff < -pi:
                diff = diff + 2 * pi 
            if abs(diff) > .05:
                status = "TURNING"

            # Check if robot passed wall on left
            if abs(ranges[90] - last_dis) > .15:
                wall_pose = current_pose
            
            # Choose what directions to go
            if status == "Finding Wall":
                new_fwd_vel = d_fwd
                if min(ranges[0], ranges[1], ranges[359]) < clearence:
                    status = "TURNING"
                    new_fwd_vel = 0
                    new_fwd_vel = 0
                    desired_pose[2] = current_pose[2] - pi/2
                    desired_pose[2] = find_nearest(directions, desired_pose[2])
                    first_encounter = current_pose
                    start_time = time.time()
            elif status == "FORWARD":
                new_fwd_vel = d_fwd
                if min(ranges[0], ranges[1], ranges[359]) < clearence:
                    status = "IDENTIFY"
                    new_fwd_vel = 0
                elif euclidean(current_pose, turn_pose) > move_dis:
                    status = "IDENTIFY"
                    new_fwd_vel = 0
                    new_ang_vel = 0
                elif abs(ranges[90] - last_dis) > .15:
                    wall_pose = current_pose
            elif status == "TURNING":                
                new_fwd_vel = 0
                if (abs(diff) < .012):
                    new_fwd_vel = d_fwd
                    new_ang_vel = 0
                    turn_pose = [current_pose[0], current_pose[1]]
                    status = "FORWARD"
            else:
                if (min(ranges[90], ranges[89], ranges[91]) > .30) and (ranges[125] > left_clearence) and (euclidean(current_pose, wall_pose) > past_wall):
                    status == "TURNING"
                    new_fwd_vel = 0
                    desired_pose[2] = current_pose[2] + pi/2
                    if desired_pose[2] > pi:
                        desired_pose[2] = desired_pose[2] - 2 * pi
                    if desired_pose[2] < -pi:
                        desired_pose[2] = desired_pose[2] + 2 * pi 
                    desired_pose[2] = find_nearest(directions, desired_pose[2])
                    diff = desired_pose[2] - current_pose[2]
                elif min(ranges[0], ranges[1], ranges[359]) < clearence:
                    status == "TURNING"
                    new_fwd_vel = 0
                    desired_pose[2] = current_pose[2] - pi/2
                    if desired_pose[2] > pi:
                        desired_pose[2] = desired_pose[2] - 2 * pi
                    if desired_pose[2] < -pi:
                        desired_pose[2] = desired_pose[2] + 2 * pi 
                    desired_pose[2] = find_nearest(directions, desired_pose[2])
                    diff = desired_pose[2] - current_pose[2]
                else:
                    new_fwd_vel = d_fwd
                    if abs(ranges[90] - last_dis) > .15:
                        wall_pose = current_pose

            # Find angular velocity to face current direction
            diff = desired_pose[2] - current_pose[2]
            if diff >= pi:
                diff = diff - 2 * pi
            if diff < -pi:
                diff = diff + 2 * pi 
            new_ang_vel = k * diff

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
            last_dis = ranges[90]

            rate.sleep()

    except rospy.ROSInterruptException:
        pass


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
    "Find nearest value in array"

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
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)

    rospy.wait_for_message("/scan", LaserScan)

    try:
        # Set rate and intitilize publisher type
        rate = rospy.Rate(20)
        data = Twist()

        global current_pose
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
        # Possible robot directions, assumes 90 degree turns only
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
                print("Moved Past Wall")
            
            # Choose what directions to go
            if status == "Finding Wall":
                new_fwd_vel = d_fwd
                print("Looking for Wall")
                if min(ranges[0], ranges[1], ranges[359]) < clearence:
                    status = "TURNING"
                    new_fwd_vel = 0
                    new_fwd_vel = 0
                    desired_pose[2] = current_pose[2] - pi/2
                    desired_pose[2] = find_nearest(directions, desired_pose[2])
                    first_encounter = current_pose
                    start_time = time.time()
                    print("Attaching")
                    print("Turn RIGHT")
            elif status == "FORWARD":
                new_fwd_vel = d_fwd
                if min(ranges[0], ranges[1], ranges[359]) < clearence:
                    status = "IDENTIFY"
                    new_fwd_vel = 0
                    print("Hitting Wall")
                elif euclidean(current_pose, turn_pose) > move_dis:
                    status = "IDENTIFY"
                    new_fwd_vel = 0
                    new_ang_vel = 0
                    print("Moved ", move_dis, " meters")
                elif abs(ranges[90] - last_dis) > .15:
                    wall_pose = current_pose
                    print("Moved Past Wall")
                print("Moving FORWARD")
            elif status == "TURNING":                
                new_fwd_vel = 0
                if (abs(diff) < .012):
                    new_fwd_vel = d_fwd
                    new_ang_vel = 0
                    turn_pose = [current_pose[0], current_pose[1]]
                    status = "FORWARD"
                print("Turning ", diff)
            elif status == "SPAWN":
                diff = desired_pose[2] - current_pose[2]
                if diff >= pi:
                    diff = diff - 2 * pi
                if diff < -pi:
                    diff = diff + 2 * pi 
                if abs(diff) > .05:
                    new_fwd_vel = 0
                    if abs(diff) < .012:
                        new_fwd_vel = d_fwd
                        new_ang_vel = 0
                        turn_pose = [current_pose[0], current_pose[1]]
                elif (euclidean(current_pose,first_encounter) <.2): 
                    print("ARRIVED")
                    new_fwd_vel = 0
                else:
                    new_fwd_vel = d_fwd
            else:
                if (min(ranges[90], ranges[89], ranges[91]) > .30) and (ranges[125] > left_clearence) and (euclidean(current_pose, wall_pose) > past_wall):
                    print("HERES WHY  ", min(ranges[90], ranges[89], ranges[91]), "   ", ranges[125], "  ", euclidean(current_pose, wall_pose))
                    status == "TURNING"
                    new_fwd_vel = 0
                    desired_pose[2] = current_pose[2] + pi/2
                    if desired_pose[2] > pi:
                        desired_pose[2] = desired_pose[2] - 2 * pi
                    if desired_pose[2] < -pi:
                        desired_pose[2] = desired_pose[2] + 2 * pi 
                    desired_pose[2] = find_nearest(directions, desired_pose[2])
                    diff = desired_pose[2] - current_pose[2]
                    print("Turn LEFT")
                elif min(ranges[0], ranges[1], ranges[359]) < clearence:
                    print("HERES WHY  ", min(ranges[90], ranges[89], ranges[91]), "   ", ranges[125], "  ", euclidean(current_pose, wall_pose))
                    status == "TURNING"
                    new_fwd_vel = 0
                    desired_pose[2] = current_pose[2] - pi/2
                    if desired_pose[2] > pi:
                        desired_pose[2] = desired_pose[2] - 2 * pi
                    if desired_pose[2] < -pi:
                        desired_pose[2] = desired_pose[2] + 2 * pi 
                    desired_pose[2] = find_nearest(directions, desired_pose[2])
                    diff = desired_pose[2] - current_pose[2]
                    print("Turn RIGHT")
                else:
                    new_fwd_vel = d_fwd
                    if abs(ranges[90] - last_dis) > .15:
                        wall_pose = current_pose
                        print("Moved Past Wall")
                    print("Default: Forward")

            # Find angular velocity to face current direction
            diff = desired_pose[2] - current_pose[2]
            if diff >= pi:
                diff = diff - 2 * pi
            if diff < -pi:
                diff = diff + 2 * pi 
            new_ang_vel = k * diff

            # If robot returns to where it first found wall, turn and return to spawn location
            if (start_time - time.time() > 30) and (euclidean(current_pose,first_encounter) <.2):
                spawn = "TRUE"    
                desired_pose[2] = current_pose[2] - pi/2
                if desired_pose[2] > pi:
                    desired_pose[2] = desired_pose[2] - 2 * pi
                if desired_pose[2] < -pi:
                    desired_pose[2] = desired_pose[2] + 2 * pi 
                desired_pose[2] = find_nearest(directions, desired_pose[2]) 

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

if __name__ == '__main__':

    rospy.init_node("Mapping", anonymous=False)

    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
        
    controller()

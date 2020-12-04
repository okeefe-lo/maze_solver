#!/usr/bin/env python3

"""Controls the movement of the robot"""
import sys
import rospy
from math import pi
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
from squaternion import Quaternion

global laser_params
global ranges
global current_pose
global desired_pose
global status

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

def controller():
    """Creates a publisher of type twist to move the robot"""

    # Create Sub and Pub
    pub = rospy.Publisher('/' + sys.argv[1] + '/cmd_vel', Twist, queue_size=1)

    rospy.wait_for_message('/' + sys.argv[1] + "/scan", LaserScan)

    try:
        # Set rate and intitilize publisher type
        rate = rospy.Rate(20)
        data = Twist()

        current_fwd_vel = 0
        current_ang_vel = 0

        while not rospy.is_shutdown():
            
            [forward_velocity, angular_velocity] = mapping(current_fwd_vel, current_ang_vel)

            # Set data to publish
            data.linear.x = forward_velocity
            data.linear.y = 0
            data.linear.z = 0
            data.angular.x = 0
            data.angular.y = 0
            data.angular.z = angular_velocity

            # Publish data
            pub.publish(data)

            current_fwd_vel = forward_velocity
            current_ang_vel = angular_velocity

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

def mapping(old_fwd_vel, old_ang_vel):
    """Script that determines how best to move through map"""
    global status
    global start_pos
    global current_pose
    attached = status
    k = .1
    d_forward = .1
    if attached == "NONE":
        new_fwd_vel = d_forward
        new_ang_vel = 0
        print("Unattached")
        if ranges[0] < .25:
            new_fwd_vel = 0
            status = "LEFT"
            print("Attaching")
            print("Turn RIGHT")
            new_fwd_vel = 0
            desired_pose[2] = current_pose[2] - pi/2
            diff = desired_pose[2] - current_pose[2]
            if diff > pi:
                diff = diff - 2 * pi
            if diff < -pi:
                diff = diff + 2 * pi 
            new_ang_vel = k * diff
    elif attached == "FORWARD":
        new_fwd_vel = d_forward
        new_ang_vel = 0
        if ranges[0] < .25:
            new_fwd_vel = 0
            new_ang_vel = 0
            status = "LEFT"
        elif euclidean(current_pose, start_pos) > .2:
            new_fwd_vel = 0
            new_ang_vel = 0
            status = "LEFT"
            print("Moving .2 meters")
    else:
        status = "LEFT"
        if abs(old_ang_vel) > 0:
            print("Turning")
            new_fwd_vel = 0
            new_ang_vel = old_ang_vel
            if abs(desired_pose[2] - current_pose[2]) < .005:
                new_fwd_vel = d_forward
                new_ang_vel = 0
                start_pos = [current_pose[0], current_pose[1]]
                status = "FORWARD"
        elif (ranges[90] > .2) & (ranges[135] > .3):
            print("Turn LEFT")
            new_fwd_vel = 0
            desired_pose[2] = current_pose[2] + pi/2
            diff = desired_pose[2] - current_pose[2]
            if diff > pi:
                diff = diff - 2 * pi
            if diff < -pi:
                diff = diff + 2 * pi 
            new_ang_vel = k * diff
        elif ranges[0] < .25:
            print("Turn RIGHT")
            new_fwd_vel = 0
            desired_pose[2] = current_pose[2] - pi/2
            diff = desired_pose[2] - current_pose[2]
            if diff > pi:
                diff = diff - 2 * pi
            if diff < -pi:
                diff = diff + 2 * pi 
            new_ang_vel = k * diff
        else:
            print("Move FORWARD")
            new_fwd_vel = d_forward
            new_ang_vel = 0

    return [new_fwd_vel, new_ang_vel]

if __name__ == '__main__':

    rospy.init_node("Controller", anonymous=False)

    global laser_params
    laser_params = rospy.get_param('laser_params')
    # angle_min = laser_params['angle_min']
    # angle_max = laser_params['angle_max']
    # angle_increment = laser_params['angle_increment']
    # range_min = laser_params['range_min']
    # range_max = laser_params['range_max']
    # laser_num = laser_params['laser_num']

    global desired_pose
    global status
    desired_pose =[0, 0, 0]
    status = "NONE"

    rospy.Subscriber('/' + sys.argv[1] + "/scan", LaserScan, laser_callback)
    rospy.Subscriber('/' + sys.argv[1] + "/odom", Odometry, odom_callback)
    if(sys.argv[1]) == "master":
        controller()

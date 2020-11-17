"""Controls the movement of the robot"""
import sys
import time
import rospy
from math import pi
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

global laser_params
global ranges
global current_pose
global desired_pose
global wall

def laser_callback(data):
    """Updates laser scan data"""

    global ranges
    ranges = data.ranges

def odom_callback(data):
    """Updates position data"""

    global current_pose
    euler = euler_from_quaternion([\
        data.pose.pose.orientation.x, \
        data.pose.pose.orientation.y, \
        data.pose.pose.orientation.z, \
        data.pose.pose.orientation.w])
    current_pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler[2]]

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
    global wall
    attached = wall
    if attached == 'none':
        new_fwd_vel = .2
        new_ang_vel = 0
        if ranges[0] < .2:
            new_fwd_vel = 0
            attached = 'left'
    elif attached == 'left':
        if abs(old_ang_vel) > 0:
            new_fwd_vel = 0
            new_ang_vel = old_ang_vel
            if abs(desired_pose[2] - current_pose[2]) < .05:
                new_ang_vel = 0
        elif ranges[0] < .2:
            new_fwd_vel = 0
            desired_pose[2] = current_pose[2] - pi/2
            new_ang_vel = -.2
        elif ranges[90] > .2:
            new_fwd_vel = 0
            desired_pose[2] = current_pose[2] + pi/2
        else:
            new_fwd_vel = 0
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
    desired_pose =[0, 0, 0]
    global wall
    wall = 'none'

    rospy.Subscriber('/' + sys.argv[1] + "/scan", LaserScan, laser_callback)
    rospy.Subscriber('/' + sys.argv[1] + "/odom", Odometry, odom_callback)
    if(sys.argv[1]) == "master":
        controller()

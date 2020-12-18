#!/usr/bin/env python3

"""Takes /target and publishes a 2D nav goal"""
import sys
import rospy
import time
from math import pi
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from squaternion import Quaternion

def target_callback(data):
    """Takes in location of point and publishes to proper topic"""

    # To send movement command from command line
    # rostopic pub /target geometry_msgs/Point '{x: -1.25, y: 0.13}'

    print("MESSAGE RECEIVED")

    global target_pose

    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    target = PoseStamped()
    target.pose.position.x = data.x
    target.pose.position.y = data.y
    target.pose.orientation.x = 0
    target.pose.orientation.y = 0
    target.pose.orientation.z = 0
    target.pose.orientation.w = 1
    target.header.frame_id = "map"
    
    target_pose = [data.x, data.y]

    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)

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
    """Returns euclidean distance betwen two points"""
    return (((current[0]-start[0]) ** 2) + ((current[1]-start[1]) ** 2)) ** .5

def spawn(initial_pose):
    """Send the command to return to spawn location"""
    global target_pose

    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    target = PoseStamped()
    target.pose.position.x = initial_pose[0]
    target.pose.position.y = initial_pose[1]
    target.pose.orientation.x = 0
    target.pose.orientation.y = 0
    target.pose.orientation.z = 0
    target.pose.orientation.w = 1
    target.header.frame_id = "map"

    target_pose = [initial_pose[0], initial_pose[1]]

    pub.publish(target)
    time.sleep(.1)
    pub.publish(target)

if __name__ == '__main__':

    rospy.init_node("Navigation", anonymous=False)
    rospy.Subscriber("/target", Point, target_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.wait_for_message("/odom", Odometry)

    try:
        global target_pose
        global current_pose

        target_pose = [1000, 1000]

        spawn_pose = current_pose

        while not rospy.is_shutdown():
          
            # If robot has arrived at destination, wait 5 seconds before giving command to return to spawn
            if (euclidean(current_pose, target_pose) < .05):
                start_time = time.time()
                time.sleep(5)
                spawn(spawn_pose)
                time.sleep(.1)
                spawn(spawn_pose)
                time.sleep(.1)
                spawn(spawn_pose)

        rospy.spin()


    except rospy.ROSInterruptException:
        pass

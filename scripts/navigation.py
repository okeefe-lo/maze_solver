#!/usr/bin/env python3

"""Takes /target and publishes a 2D nav goal"""
import sys
import rospy
import time
from geometry_msgs.msg import Point, PoseStamped

def target_callback(data):

    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    target = Point()
    target.pose.position.x = data.x
    target.pose.position.y = data.y
    pub.publish(target)

if __name__ == '__main__':

    rospy.init_node("Navigation", anonymous=False)
    rospy.Subscriber("/target", Point, target_callback)


#!/usr/bin/env python
# coding=utf8

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int8

DETECT = False

aruco_detector_res = None
ids = None
_id_get = 0

def pub_vel(x, y, w):
    # Publish x, y, w values to the robot
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.angular.z = w
    pub.publish(twist)

def callback(data):
    global aruco_detector_res
    aruco_detector_res = data

def main():
    global aruco_detector_res
    rospy.init_node('qr_to_robot', anonymous=True)
    rate = rospy.Rate(30)

    # Publishers
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscribers
    rospy.Subscriber("/goal_data", PoseStamped, callback)

    while not rospy.is_shutdown():
        if aruco_detector_res is not None:
            # Extract x, y, and w values from aruco_detector_res
            x = aruco_detector_res.pose.position.x
            y = aruco_detector_res.pose.position.y
            w = aruco_detector_res.pose.orientation.w

            # Send x, y, and w values to the robot
            pub_vel(x, y, w)

            # Reset aruco_detector_res after processing
            aruco_detector_res = None

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

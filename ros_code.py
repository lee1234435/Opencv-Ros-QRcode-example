#!/usr/bin/env python
# coding=utf8

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
import cv2

class QRCodeFollower:
    def __init__(self):
        rospy.init_node('qr_code_follower', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(30)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            rospy.logerr(e)
            return

        # Decode QR code
        qr_data = decode(cv_image)

        if qr_data:
            # Extract x, y, and w values from QR code data
            x, y, w = map(float, qr_data[0].data.decode('utf-8').split(','))

            # Publish x, y, w values to the robot
            self.publish_twist(x, y, w)

    def publish_twist(self, x, y, w):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = w
        self.cmd_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        qr_code_follower = QRCodeFollower()
        qr_code_follower.run()
    except rospy.ROSInterruptException:
        pass

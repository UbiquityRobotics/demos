#!/usr/bin/python

import rospy
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
import math

image_width = 640
image_height = 480

rospy.init_node('follower')

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def detection_callback(data):
    x_center = data.detections[0].bbox.center.x

    lateral_error = image_width/2 - x_center
    z_r = lateral_error/400

    area = data.detections[0].bbox.size_x * data.detections[0].bbox.size_y
    x_f = (1/area) * 700
    print x_f

    twist = Twist()
    twist.linear.x = x_f
    twist.angular.z = z_r
    cmd_vel_pub.publish(twist)

rospy.Subscriber('/detectnet/detections', Detection2DArray, detection_callback)

rospy.spin()


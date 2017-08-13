#!/usr/bin/python

"""
Copyright (c) 2015, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
Simultaneous location and mapping based on fiducial poses.
Receives FiducialTransform messages and builds a map and estimates the
camera pose from them
"""


import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, \
                              TransformStamped, Twist

from fiducial_pose.msg import Fiducial, FiducialTransform, FiducialTransformArray

from tf.transformations import euler_from_quaternion, quaternion_slerp, \
                               translation_matrix, quaternion_matrix, \
                               translation_from_matrix, quaternion_from_matrix, \
                               quaternion_from_euler

import tf2_ros

from math import pi, sqrt
import sys
import os
import traceback
import math
import numpy
import time
import threading
import copy

def degrees(r):
    return 180.0 * r / math.pi

class Follow:
    def __init__(self):
       rospy.init_node('follow')
       self.tfBuffer = tf2_ros.Buffer(rospy.Time(30))
       self.lr = tf2_ros.TransformListener(self.tfBuffer)
       self.br = tf2_ros.TransformBroadcaster()
       self.cmdPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
       self.target_fiducial = rospy.get_param("target_fiducial", "fid49")
       rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)
       self.fid_x = 0.6
       self.fid_y = 0
       self.got_fid = False

    """
    Called when a FiducialTransformArray is received
    """
    def newTf(self, msg):
        imageTime = msg.header.stamp
        self.linSpeed = 0

        for m in msg.transforms:
            id = m.fiducial_id
            trans = m.transform.translation
            rot = m.transform.rotation
            print "Fid %d %lf %lf %lf %lf %lf %lf %lf\n" % \
                                 (id, trans.x, trans.y, trans.z, 
                                  rot.x, rot.y, rot.z, rot.w)
            t = TransformStamped()
            t.child_frame_id = "fid%d" % id
            t.header.frame_id = "raspicam"
            t.header.stamp = imageTime
            t.transform.translation.x = trans.x
            t.transform.translation.y = trans.y
            t.transform.translation.z = trans.z
            t.transform.rotation.x = rot.x
            t.transform.rotation.y = rot.y
            t.transform.rotation.z = rot.z
            t.transform.rotation.w = rot.w
            self.br.sendTransform(t)

            rospy.sleep(0.01) # hack
            try:
                tf = self.tfBuffer.lookup_transform("base_link", self.target_fiducial, imageTime)
                ct = tf.transform.translation
                cr = tf.transform.rotation
                print "T_fidBase %lf %lf %lf %lf %lf %lf %lf\n" % \
                                 (ct.x, ct.y, ct.z, cr.x, cr.y, cr.z, cr.w)
                self.fid_x = ct.x
                self.fid_y = ct.y
                self.got_fid = True 
            except:
                #traceback.print_exc()
                print "Could not get tf from fid%d" % id

    def run(self):
        rate = rospy.Rate(10) # 10hz
        linSpeed = 0.0
        theta = 0.0
        times_since_last_fid = 0
        while not rospy.is_shutdown():
            forward_error = self.fid_x - 0.6
            lateral_error = self.fid_y
            angular_error = math.atan2(self.fid_y, self.fid_x)
            print "Forward error", forward_error
            print "Lateral error", lateral_error
            print "Angular error", degrees(angular_error)

            if (self.got_fid):
                times_since_last_fid = 0

            if (forward_error > 2.0):
                print "Fiducial is too far away"
                linSpeed = 0 
                theta = 0
                twist = Twist()
                twist.angular.z = theta
                twist.linear.x = linSpeed 
                self.cmdPub.publish(twist)
            elif (self.got_fid == False and times_since_last_fid < 2):
                times_since_last_fid += 1
                twist = Twist()
                twist.angular.z = theta
                twist.linear.x = linSpeed 
                self.cmdPub.publish(twist)
            elif (self.got_fid == False and times_since_last_fid < 300):
                linSpeed = 0
                twist = Twist()
                twist.angular.z = theta
                twist.linear.x = linSpeed 
                self.cmdPub.publish(twist)
                print "Try keep rotating to refind fiducial: try# %d" % times_since_last_fid
                times_since_last_fid += 1
            elif (self.got_fid == False and times_since_last_fid >= 300):
                linSpeed = 0
                theta = 0
                twist = Twist()
                twist.angular.z = theta
                twist.linear.x = linSpeed 
                self.cmdPub.publish(twist)
                print "Give up try# %d" % times_since_last_fid
                times_since_last_fid += 1
            elif (abs(forward_error) > 0.1 or abs(lateral_error) > 0.1): 
                linSpeed = min(forward_error, 1.2)
                theta = angular_error
                twist = Twist()
                twist.angular.z = theta
                twist.linear.x = linSpeed 
                self.cmdPub.publish(twist)
            elif (self.got_fid):
                linSpeed = 0 
                theta = 0
                twist = Twist()
                twist.angular.z = theta
                twist.linear.x = linSpeed 
                self.cmdPub.publish(twist)
                print "We have fiducial so stop"
            else:
                linSpeed = 0
                theta = 0
            self.got_fid = False
            rate.sleep()

if __name__ == "__main__":
    node = Follow()
    node.run()

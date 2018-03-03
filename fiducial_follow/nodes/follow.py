#!/usr/bin/python

"""
Copyright (c) 2017, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of fiducial_follow nor the names of its
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
Fiducial Follow Demo.  Receives trasforms to fiducials and generates
movement commands for the robot to follow the fiducial of interest.
"""

import rospy
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time


def degrees(r):
    return 180.0 * r / math.pi

class Follow:
    """
    Constructor for our class
    """
    def __init__(self):
       rospy.init_node('follow')

       # Set up a transform listener so we can lookup transforms in the past
       self.tfBuffer = tf2_ros.Buffer(rospy.Time(30))
       self.lr = tf2_ros.TransformListener(self.tfBuffer)

       # Setup a transform broadcaster so that we can publish transforms
       # This allows to visualize the 3D position of the fiducial easily in rviz
       self.br = tf2_ros.TransformBroadcaster()

       # A publisher for robot motion commands
       self.cmdPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

       # Flag to avoid sending repeated zero speeds
       self.suppressCmd = False

       # The name of the coordinate frame of the fiducial we are interested in
       self.target_fiducial = rospy.get_param("~target_fiducial", "fid49")

       # Minimum distance we want the robot to be from the fiducial
       self.min_dist = rospy.get_param("~min_dist", 0.6)

       # Maximum distance a fiducial can be away for us to attempt to follow
       self.max_dist = rospy.get_param("~max_dist", 2.5)

       # Proportion of angular error to use as angular velocity
       self.angular_rate = rospy.get_param("~angular_rate", 2.0)

       # Maximum angular speed (radians/second)
       self.max_angular_rate = rospy.get_param("~max_angular_rate", 1.2)

       # Angular velocity when a fiducial is not in view
       self.lost_angular_rate = rospy.get_param("~lost_angular_rate", 0.6)

       # Proportion of linear error to use as linear velocity
       self.linear_rate = rospy.get_param("~linear_rate", 1.2)

       # Maximum linear speed (meters/second)
       self.max_linear_rate = rospy.get_param("~max_linear_rate", 1.5)

       # Linear speed decay (meters/second)
       self.linear_decay = rospy.get_param("~linear_decay", 0.9)

       # How many loop iterations to keep linear velocity after fiducial
       # disappears
       self.hysteresis_count = rospy.get_param("~hysteresis_count", 20)

       # How many loop iterations to keep rotating after fiducial disappears
       self.max_lost_count = rospy.get_param("~max_lost_count", 400)

       # Subscribe to incoming transforms
       rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)
       self.fid_x = self.min_dist
       self.fid_y = 0
       self.got_fid = False


    """
    Called when a FiducialTransformArray is received
    """
    def newTf(self, msg):
        imageTime = msg.header.stamp
        self.linSpeed = 0

        print imageTime, rospy.Time.now()
        print "*****"
        found = False

        # For every fiducial found by the dectector, publish a transform
        for m in msg.transforms:
            id = m.fiducial_id
            trans = m.transform.translation
            rot = m.transform.rotation
            print "Fid %d %lf %lf %lf %lf %lf %lf %lf\n" % \
                                 (id, trans.x, trans.y, trans.z,
                                  rot.x, rot.y, rot.z, rot.w)
            t = TransformStamped()
            t.child_frame_id = "fid%d" % id
            t.header.frame_id = msg.header.frame_id
            t.header.stamp = imageTime
            t.transform.translation.x = trans.x
            t.transform.translation.y = trans.y
            t.transform.translation.z = trans.z
            t.transform.rotation.x = rot.x
            t.transform.rotation.y = rot.y
            t.transform.rotation.z = rot.z
            t.transform.rotation.w = rot.w
            self.br.sendTransform(t)

            if t.child_frame_id == self.target_fiducial:
                # We found the fiducial we are looking for
                found = True

                # Add the transform of the fiducial to our buffer
                self.tfBuffer.set_transform(t, "follow")

        if not found:
            return # Exit this function now, we don't see the fiducial
        try:
            # Get the fiducial position relative to the robot center, instead of the camera
            tf = self.tfBuffer.lookup_transform("base_link", self.target_fiducial, imageTime)
            ct = tf.transform.translation
            cr = tf.transform.rotation
            print "T_fidBase %lf %lf %lf %lf %lf %lf %lf\n" % \
                             (ct.x, ct.y, ct.z, cr.x, cr.y, cr.z, cr.w)

            # Set the state varibles to the position of the fiducial
            self.fid_x = ct.x
            self.fid_y = ct.y
            self.got_fid = True
        except:
            traceback.print_exc()
            print "Could not get tf for %s" % self.target_fiducial


    """
    Main loop
    """
    def run(self):
        # setup for looping at 20hz
        rate = rospy.Rate(20)

        # Setup the variables that we will use later
        linSpeed = 0.0
        angSpeed = 0.0
        times_since_last_fid = 0

        # While our node is running
        while not rospy.is_shutdown():
            # Calculate the error in the x and y directions
            forward_error = self.fid_x - self.min_dist
            lateral_error = self.fid_y

            # Calculate the amount of turning needed towards the fiducial
            # atan2 works for any point on a circle (as opposed to atan)
            angular_error = math.atan2(self.fid_y, self.fid_x)

            print "Errors: forward %f lateral %f angular %f" % \
              (forward_error, lateral_error, degrees(angular_error))

            if self.got_fid:
                times_since_last_fid = 0
            else:
                times_since_last_fid += 1

            if forward_error > self.max_dist:
                print "Fiducial is too far away"
                linSpeed = 0
                angSpeed = 0
            # A fiducial was detected since last iteration of this loop
            elif self.got_fid:
                # Set the turning speed based on the angular error
                # Add some damping based on the previous speed to smooth the motion 
                angSpeed = angular_error * self.angular_rate - angSpeed / 2.0
                # Make sure that the angular speed is within limits
                if angSpeed < -self.max_angular_rate:
                    angSpeed = -self.max_angular_rate
                if angSpeed > self.max_angular_rate:
                    angSpeed = self.max_angular_rate

                # Set the forward speed based distance
                linSpeed = forward_error * self.linear_rate
                # Make sure that the angular speed is within limits
                if linSpeed < -self.max_linear_rate:
                    linSpeed = -self.max_linear_rate
                if linSpeed > self.max_linear_rate:
                    linSpeed = self.max_linear_rate

            # Hysteresis, don't immediately stop if the fiducial is lost
            elif not self.got_fid and times_since_last_fid < self.hysteresis_count:
                # Decrease the speed (assuming linear decay is <1)
                linSpeed *= self.linear_decay

            # Try to refind fiducial by rotating
            elif self.got_fid == False and times_since_last_fid < self.max_lost_count:
                # Stop moving forward
                linSpeed = 0
                # Keep turning in the same direction
                if angSpeed < 0:
                    angSpeed = -self.lost_angular_rate
                elif angSpeed > 0:
                    angSpeed = self.lost_angular_rate
                else:
                    angSpeed = 0
                print "Try keep rotating to refind fiducial: try# %d" % times_since_last_fid
            else:
                angSpeed = 0
                linSpeed = 0

            print "Speeds: linear %f angular %f" % (linSpeed, angSpeed)

            # Create a Twist message from the velocities and publish it
            # Avoid sending repeated zero speed commands, so teleop
            # can work
            zeroSpeed = (angSpeed == 0 and linSpeed == 0)
            if not zeroSpeed:
                self.suppressCmd = False
            print "zero", zeroSpeed, self.suppressCmd
            if not self.suppressCmd:
                twist = Twist()
                twist.angular.z = angSpeed
                twist.linear.x = linSpeed
                self.cmdPub.publish(twist)
                if zeroSpeed:
                    self.suppressCmd = True

            # We already acted on the current fiducial
            self.got_fid = False
            rate.sleep()


if __name__ == "__main__":
    # Create an instance of our follow class
    node = Follow()
    # run it
    node.run()

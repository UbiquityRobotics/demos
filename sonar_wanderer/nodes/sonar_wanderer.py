#!/usr/bin/python

"""
Copyright (c) 2021, Ubiquity Robotics
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
Sonar Wanderer Demo.  Receive sonar range data and wander around avoiding objects
"""

import rospy
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Range
from math import pi, sqrt, atan2
import traceback
import math
import time


def degrees(r):
    return 180.0 * r / math.pi

class SonarWanderer:
    """
    Constructor for our class
    """

    # Fetch new average value from current average value and new value using 
    # a fractional weight from 0 - 1.0 for the new value
    def runningAverage(self, currentAvgValue, newValue, newWeight):
        newAvgValue = (newValue * newWeight) + (currentAvgValue * (1.0 - newWeight))
        return newAvgValue

    """
    Called when a sonar message is received
    """
    def sonar_callback(self, msg):
        imageTime = msg.header.stamp
        self.linSpeed = 0

        # pick off the ranges for the sensors we are going to use for detection
        # look for msg->header.frame_id where  sonar_3=front, sonar_2=right, sonar_1=left
        sonarId = msg.header.frame_id
        sonarRange = msg.range
        
        found = False

        # we do a simple running average for the ranges
        if sonarId == "sonar_3":
            self.range_front = self.runningAverage(self.range_front, sonarRange, self.new_range_weight)
        if sonarId == "sonar_2":
            self.range_right = self.runningAverage(self.range_right, sonarRange, self.new_range_weight)
        if sonarId == "sonar_1":
            self.range_left  = self.runningAverage(self.range_left,  sonarRange, self.new_range_weight)

        print "SonarRanges: ", str(self.range_left)[:6], str(self.range_front)[:6], str(self.range_right)[:6]

    def __init__(self):
       global sonar_callback
       rospy.init_node('sonar_wanderer')

       # get a starting time so our time printouts are relative to script start
       self.startTime = rospy.Time.now()

       # A publisher for robot motion commands
       self.cmdPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

       # Subscribe to sonar messages
       self.sonarSub = rospy.Subscriber("/sonars", Range, self.sonar_callback)

       # Setup very far distances for initial sonar detection ranges so we are not blocked from start
       self.range_front = 5.0
       self.range_right = 5.0
       self.range_left  = 5.0
       self.new_range_weight = 0.25

       # Flag to avoid sending repeated zero speeds
       self.suppressCmd = False

       # allow user to set weight of the newest sonar range in running average
       self.new_range_weight = rospy.get_param("~new_range_weight", 0.25)

       # Minimum distance we want the robot to be from objects it detects
       self.min_dist = rospy.get_param("~min_dist", 0.6)

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
            # Make decisions on how to change speed
            # TODO!!!

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
            rate.sleep()


if __name__ == "__main__":
    # Create an instance of our follow class
    node = SonarWanderer()
    # run it
    node.run()

#!/usr/bin/python3

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

    Overly simple logic is used to do basic object avoidance.
    The purpose of this is to demo sonar interpritation and robot movement control.
    We are not expecting a Nobel prize for ingenious and natural AI intellegent behavior  ;-)
"""

import rospy
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from math import pi, sqrt, atan2
import traceback
import math
import time
import signal
import sys


def degrees(r):
    return 180.0 * r / math.pi

# Trap to exit the program on Control C from keyboard
def signal_handler(sig, frame):
    print(' ')
    print('Exit due to detection of Ctrl+C!')
    sys.exit(0)

# This class implements the principles of sonar topic reading and acting
# on detected objects to modify movement on the cmd_vel topic

class SonarWanderer:

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

        # we do a simple running average for the ranges that we care about
        # we have added 0.1 for the left in the callback as the sonar is off to the right about that much
        if sonarId == "sonar_1":
            self.range_left  = self.runningAverage(self.range_left,  sonarRange, self.new_range_weight)
        if sonarId == "sonar_2":
            self.range_right = self.runningAverage(self.range_right, sonarRange, self.new_range_weight)
        if sonarId == "sonar_3":
            self.range_front = self.runningAverage(self.range_front, sonarRange, self.new_range_weight)

        if self.debug_prints > 0 and self.motors_enabled == True:
            print("SonarRanges: ", str(self.range_left)[:6], str(self.range_front)[:6], str(self.range_right)[:6])

    """
    Called when a motor_power_active message is received
    """
    def motor_power_callback(self, msg):
        self.motor_power_on = msg.data

    """
    Main node control
    """
    def __init__(self):
       global sonar_callback
       rospy.init_node('sonar_wanderer')

       self.debug_prints = 1

       # allow for exit using Control-C from keyboard
       signal.signal(signal.SIGINT, signal_handler)    

       # get a starting time so our time printouts are relative to script start
       self.startTime = rospy.Time.now()

       # A publisher for robot motion commands
       self.cmdPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

       # Subscribe to sonar messages
       self.sonarSub = rospy.Subscriber("/sonars", Range, self.sonar_callback)

       # Subscribe to topic to know if motor power is on or off
       self.motorPowerSub = rospy.Subscriber("/motor_power_active", Bool, self.motor_power_callback)

       # Flag to avoid sending repeated zero speeds
       self.suppressCmd = False

       # allow user to set weight of the newest sonar range in running average
       self.new_range_weight = rospy.get_param("~new_range_weight", 0.25)

       # Setup to assume motor power is on as we start
       self.motor_power_on = True
       self.motors_enabled = True

       # Setup very far distances for initial sonar detection ranges so we are not blocked from start
       self.range_front = 2.0
       self.range_right = 2.0
       self.range_left  = 2.0
       self.new_range_weight = 0.25

       # Define the limits where we consider action is required for detected objects out front
       # The Minimum distance we want the robot to be from objects is defined for out front
       self.detect_dist = rospy.get_param("~detect_dist", 1.0)
       self.limit_front = self.detect_dist
       self.limit_right = self.detect_dist * 1.3
       self.limit_left  = self.detect_dist * 1.3

       # The rate to drive the robot when wandering in M/sec
       self.linear_rate = rospy.get_param("~linear_rate", 0.2)

       # The rate to rotate when avoiding an object
       self.angular_rate = rospy.get_param("~angular_rate", 0.7)


    """
    Main loop
    """
    def run(self):
        # setup for looping at 20hz
        loopRateSec = 0.035
        rate = rospy.Rate(1/loopRateSec)

        # Setup the variables that we will use later
        linSpeed = 0.0
        angSpeed = 0.0
        times_since_last_fid = 0

        rotation_duration = 0      # time to be rotating to avoid object

        self.motors_enabled = True

        # While our node is running
        while not rospy.is_shutdown():
            # We are keeping this very simple in this demo so up to user to make it 'smarter'
            # we will have 2 simple modes. 1 is driving forward and another is doing a rotation

            # special handling if motor power is off
            if self.motor_power_on == False:
                if self.motors_enabled == True:
                    print("Motor power was turned off. Pause till it is back on again.")
                self.motors_enabled = False
            else:
                if self.motors_enabled == False:
                    print("Motor power has been re-enabled. Continue operation.")
                self.motors_enabled = True

            if rotation_duration > 0.0:
                rotation_duration -= loopRateSec   # remain in rotation mode till correction is done
            else:
                if self.motors_enabled == False:
                    # Suspend operation when power is off
                    rate.sleep()

                elif self.range_front < self.limit_front:   # detect object straight ahead
                    linSpeed = 0.0
                    # Make decisions on rotation time based Pi/2 turn
                    rotation_duration = (6.28/8) / self.angular_rate

                    # make a decision on way to rotate by turning where side sensor has more range
                    if self.range_left > self.range_right:
                        angSpeed = self.angular_rate
                        if self.debug_prints > 0:
                            print("Avoid object in front by rotation of 90 deg to left")
                    else:
                        angSpeed = -1.0 * self.angular_rate
                        if self.debug_prints > 0:
                            print("Avoid object in front by rotation of 90 deg to right")

                elif self.range_right < self.limit_right: # detect object on the right but not close in front
                        angSpeed = self.angular_rate
                        rotation_duration = (6.28/16) / self.angular_rate
                        if self.debug_prints > 0:
                            print("Avoid object to the right by rotation of 45 deg to the left")

                elif self.range_left  < self.limit_left:
                        angSpeed = -1.0 * self.angular_rate
                        rotation_duration = (6.28/16) / self.angular_rate
                        if self.debug_prints > 0:
                            print("Avoid object to the right by rotation of 45 deg to the right")

                else:
                    # not rotating and no object within limits.  Just drive forward
                    if self.debug_prints > 0:
                        print("Drive forward")
                    rotation_duration = 0.0
                    angSpeed = 0.0
                    linSpeed = self.linear_rate
                
            if self.motors_enabled == True:
                print("Speeds: lin %f ang %f" % (linSpeed, angSpeed))

            # Create a Twist message from the velocities and publish it
            # Avoid sending repeated zero speed commands, so teleop
            # can work
            zeroSpeed = (angSpeed == 0 and linSpeed == 0)
            if not zeroSpeed:
                self.suppressCmd = False
            
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

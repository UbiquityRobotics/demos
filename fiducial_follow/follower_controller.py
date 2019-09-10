#!/usr/bin/python

"""
Copyright (c) 2019, Ubiquity Robotics
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
Example client control program for fiducial_follow app
Publish many messages to a topic for fiducial follower to use
"""

import rospy

# our custom messages for the commands we will follow
from custom_messages.msg import FollowerCommand, FollowerStatus

from math import pi, sqrt, atan2
import traceback
import math
import time


class Controller:
    """
    Constructor for our class
    """
    def __init__(self):
       rospy.init_node('controller')

       # Time per loop for the main control
       self.loop_msec = 50

       # A publisher for sending commands to the follower node
       self.followerCmdPub = rospy.Publisher("/follower_commands", FollowerCommand, queue_size=1)

       # Subscribe to incoming status 
       rospy.Subscriber("/follower_status", FollowerStatus, self.newFollowerStatus)

       # define a fiducial that when set implies we are not seeking any fiducial
       self.null_fiducial = "fid0"

       # ---------------------- start of keywords for use in commands ---------------------------

       # Defines that are of value to pass in as elements of messages in the inbound command topic

       # default action we will use once we actually find and have approached the fiducial
       # The awaitNextCommand action will continue to follow until a new command comes in
       self.actOnDoneStopFollowing = "StopFollowing"
       self.actOnDoneDoNextCommand = "DoNextCommand"

       # We need a state to indicate for the current command so we define 'InProgress' and 'Done'
       # When cmdStatus is InProgress we do not read new command except state control commands
       self.cmdStatusInProgress = "InProgress"
       self.cmdStatusDone       = "Done"

       # define names for specific commands which MUST be same as used by follow.py
       self.cmdFollowFiducial  = "FollowFiducial"
       self.cmdClearCommands   = "ClearCommands"
       self.cmdClearInProgress = "ClearInProgress"
       self.cmdStopMovement    = "StopMovement"
       self.cmdDriveForward    = "DriveForward"
       self.cmdDriveReverse    = "DriveReverse"
       self.cmdRotateLeft      = "RotateLeft"
       self.cmdRotateRight     = "RotateRight"

    def publishFollowerCommand(self, cmdType, actionOnDone, string1, numParam1, comment):
        cmdMsg = FollowerCommand()
        cmdMsg.commandType   = cmdType
        cmdMsg.actionOnDone  = actionOnDone
        cmdMsg.strParam1     = string1
        cmdMsg.numParam1     = numParam1
        cmdMsg.comment       = comment
        self.followerCmdPub.publish(cmdMsg)
        time.sleep(1.0)      # we have to wait a while between commands

    """
    Called when a Tracker Command is received
    """
    def newFollowerStatus(self, msg):
        commandType  = msg.commandType
        statusState  = msg.statusState
        string1      = msg.string1
        param1       = msg.num1
        string2      = msg.string2
        param2       = msg.num2
        print "New Follower status of Type: %s with statusState '%s' and str1 '%s' \n" % \
            (commandType, statusState, string1)


    """
    Main loop
    """
    def run(self):
        # setup for looping at 20hz
        rate = rospy.Rate(self.loop_msec)

        cmdType    = 0
        cmdParam1  = 0
        cmdParam2  = 0
        cmdString1 = "none"
 
        print "ROS publisher publishing commands to topic /follower_commands"

        # Fire off the commands we want to send.  THIS is the meat of this script's purpose
        # To make this nicer we could read from a file or get user input and so on
        # publishFollowerCommand(self, cmdType, actionOnDone, string1, numParam1, comment)
        self.publishFollowerCommand("ClearCommands", " ", " ", 0.0, "Clear all pending commands")
        #self.publishFollowerCommand("SetDriveRate", " ", " ", 0.1, "Set drive rate")
        #self.publishFollowerCommand("SetRotateRate", " ", " ", 0.4, "Set rotate rate")
        #self.publishFollowerCommand("DriveForward", " ", " ", 2.0, "Drive forward a little bit")
        #self.publishFollowerCommand("RotateLeft", " ", " ", 2.0, "Rotate left a little bit")
        #self.publishFollowerCommand("RotateRight", " ", " ", 2.0, "Rotate right back again")
        self.publishFollowerCommand("FollowFiducial", "DoNextCommand", "fid102", 0.0, "Follow fiducial 1")
        self.publishFollowerCommand("WaitInSeconds", " ", " ", 6.0, "Wait 6 sec ")
        #self.publishFollowerCommand("RotateRight", " ", " ", 1.0, "Rotate right back again")
        self.publishFollowerCommand("FollowFiducial", "DoNextCommand", "fid103", 0.0, "Follow fiducial 2")
        #self.publishFollowerCommand("RotateRight", " ", " ", 1.0, "Rotate right to next fiducial")
        self.publishFollowerCommand("FollowFiducial", "KeepFollowing", "fid104", 0.0, "Keep Follow fiducial 3")

        print "Commands sent "


if __name__ == "__main__":
    # Create an instance of our follow class
    node = Controller()
    # run it
    node.run()

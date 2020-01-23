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
Example client control program for following fiducials on the floor.
This simple tool publishes messages to a topic for fiducial follower to use.
This is more of an example than a specific demo.
This script shows how to communicate with follow.py node upgraded in late 2019
to be able to follow commands in a sequence loade to the node and report status.
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
       self.loop_hz = 50
       self.rate = rospy.Rate(10)

       # A publisher for sending commands to the follower node
       self.followerCmdPub = rospy.Publisher("/follower_commands", FollowerCommand, queue_size=1)

       # Subscribe to incoming status 
       rospy.Subscriber("/follower_status", FollowerStatus, self.newFollowerStatus)

       # define a fiducial that when set implies we are not seeking any fiducial
       self.null_fiducial = "fidnone"

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
        self.rate.sleep()
        self.rate.sleep()
        # time.sleep(0.8)      # we have to wait a while between commands

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
        # setup for delay at 10hz

        cmdType    = 0
        cmdParam1  = 0
        cmdParam2  = 0
        cmdString1 = "none"
 
        print "ROS publisher publishing commands to topic /follower_commands"

        # Fire off the commands we want to send.  THIS is the meat of this script's purpose
        # To make this nicer we could read from a file or get user input and so on
        # publishFollowerCommand(self, cmdType, actionOnDone, string1, numParam1, comment)

        # First optionally configure parameters we want to be active for commands
        #self.publishFollowerCommand("ClearCommands",   " ", " ", 0.0, "Clear all pending commands")
        self.publishFollowerCommand("SetMaxLinearRate", " ", " ", 0.3, "Set max linear approach rate")
        self.publishFollowerCommand("SetDriveRate",    " ", " ", 0.15, "Set drive rate")
        #self.publishFollowerCommand("SetRotateRate",   " ", " ", 0.3, "Set rotate rate")
        #self.publishFollowerCommand("WaitInSeconds",   " ", " ", 2.0, "Wait a few sec ")

        # Next we show how to follow a few fiducials on the floor
        self.publishFollowerCommand("DriveForward",    " ", " ", 1.0, "Drive forward 1 sec at DriveRate")
        #self.publishFollowerCommand("RotateRight",     " ", " ", 1.0, "Rotate right 1 sec at RotateRate")
        #self.publishFollowerCommand("RotateLeft",      " ", " ", 2.0, "Rotate left  2 sec at RotateRate")
        #self.publishFollowerCommand("RotateRight",     " ", " ", 1.0, "Rotate right 1 sec at RotateRate")
        #self.publishFollowerCommand("FollowFiducial",  "DriveOnTop", "fid101", 0.0, "Follow this fiducial")
        self.publishFollowerCommand("FollowFiducial",  "DoNextCommand", "fid105", 0.0, "Follow this fiducial")
        self.publishFollowerCommand("FollowFiducial",  "DriveOnTop", "fid105", 0.0, "drive over fiducial")
        #self.publishFollowerCommand("FollowFiducial",  "AssumePose", "fid103", 0.0, "Follow fiducial and match pose")
        self.publishFollowerCommand("FollowFiducial",  "DoNextCommand", "fid106", 0.0, "Follow this fiducial")
        self.publishFollowerCommand("FollowFiducial",  "DriveOnTop", "fid106", 0.0, "drive over fiducial")
        self.publishFollowerCommand("RotateRight",      " ", " ", 1.0, "Rotate right sec at RotateRate")
        self.publishFollowerCommand("FollowFiducial",  "DoNextCommand", "fid107", 0.0, "Follow this fiducial")
        #self.publishFollowerCommand("FollowFiducial", "KeepFollowing", "fid104", 0.0, "Keep Follow fiducial 3")

        print "Commands sent "


if __name__ == "__main__":
    # Create an instance of our follow class
    node = Controller()
    # run it
    node.run()

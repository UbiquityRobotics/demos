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
Fiducial Follow Demo.  Receives trasforms to fiducials and generates
movement commands for the robot to follow the fiducial of interest.
Optionally this node can be given commands to queue up to do complex 
sequences of movements and target following
"""

import rospy
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# our custom messages for the commands we will follow
from custom_messages.msg import FollowerCommand, FollowerStatus

import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time


def degrees(r):
    return 180.0 * r / math.pi

# define global names for specific commands 
cmdNameClearCommands   = "ClearCommands"     # Clear all commands (good idea on start of activities)
cmdNameFollowFiducial  = "FollowFiducial"    # Approach a fiducial to a distance of 
cmdNameClearInProgress = "ClearInProgress"   # Clear an in-progress move or wait or fiducial tracking
cmdNameStopMovement    = "StopMovement"
cmdNameWaitInSeconds   = "WaitInSeconds"     # Wait the time in seconds specified
cmdNameDriveForward    = "DriveForward"      # Drive forward for the specified time at the current drive_rate
cmdNameDriveReverse    = "DriveReverse"      # Drive reverse for the specified time at the current drive_rate
cmdNameSetDriveRate    = "SetDriveRate"      # Set drive_rate for next drive command
cmdNameRotateLeft      = "RotateLeft"        # Rotate left   for the specified time at the current rotate_rate
cmdNameRotateRight     = "RotateRight"       # Rotate right  for the specified time at the current rotate_rate
cmdNameSetRotateRate   = "SetRotateRate"     # Set rotate_rate for next rotate command
cmdNameSetMaxLinRate   = "SetMaxLinRate"     # Set maximum linear rate used to approach the target fiducial
cmdNameSetMaxAngRate   = "SetMaxAngRate"     # Set maximum angular rate used to rotate towards the target fiducial

# define global names for actions to be taken after command completes
actOnDoneStopFollowing = "StopFollowing"
actOnDoneDoNextCommand = "DoNextCommand"
actOnDoneAssumePose    = "AssumePose"        # After finding fiducial, drive over it and rotate to it's pose
actOnDoneDriveOnTop    = "DriveOnTop"        # After finding fiducial, drive over it
actOnDoneKeepFollowing = "KeepFollowing"

# define global status states
cmdStatusInProgress = "CmdInProgress"
cmdStatusDone       = "CmdDone"

class Follow:
    """
    Constructor for our class
    """
    def __init__(self):
       rospy.init_node('follow')

       self.commandQueue = []

       # Set the rate the main loop will run at
       self.loop_hz = 30

       # Set up a transform listener so we can lookup transforms in the past
       self.tfBuffer = tf2_ros.Buffer(rospy.Time(30))
       self.tfDbgBuffer = tf2_ros.Buffer(rospy.Time(self.loop_hz))
       self.lr = tf2_ros.TransformListener(self.tfBuffer)

       # Setup a transform broadcaster so that we can publish transforms
       # This allows to visualize the 3D position of the fiducial easily in rviz
       self.br = tf2_ros.TransformBroadcaster()

       # Flag to avoid sending repeated zero speeds
       self.suppressCmd = False

       # define a move time that when non-zero we continue to do rotate or drive commands
       self.moveTimeLeft = 0.0

       # define a wait time that when non-zero we continue to wait and do nothing
       self.waitTimeLeft = 0.0

       # ---------------------- start of keywords for use in commands ---------------------------

       # define a fiducial that when set implies we are not seeking any fiducial
       self.null_fiducial = "fidnone"

       # The default fiducial we will start to follow from the start. Use of null_fiducial means idle at start
       # The legacy fiducial follow used marker 49 so default to that here
       # Set this to self.null_fiducial if you want idle at start awaiting commands
       self.target_fiducial = rospy.get_param("~target_fiducial", "fid49")

       # optionally we can disable verbose debug messages or leave them on as in legacy code
       # these were on in legacy version but can be disabled using parameter below
       # 1 is less verbose and 2 more verbose with 0 almost no messages
       self.debug_follow  = rospy.get_param("~debug_follow", 1)
       self.debug_verbose = rospy.get_param("~debug_verbose", False)

       # Set to 'true' for Legacy follower where we search for the sole fiducial if it goes out of sight
       # to operate in the mode where commands are used via topic this should be false
       searchOption = rospy.get_param("~search_for_target", "false")
       if searchOption == "true":
           self.search_for_target = True 
       else:
           self.search_for_target = False 

       # ---------------------- start of keywords for use in commands ---------------------------

       # Defines that are of value to pass in as elements of messages in the inbound command topic

       # default action we will use once we actually find and have approached the fiducial
       # The awaitNextCommand action will continue to follow until a new command comes in
       self.actOnDone = actOnDoneDoNextCommand

       # We need a state to indicate for the current command so we define 'InProgress' and 'Done'
       # When cmdStatus is InProgress we do not read new command except state control commands
       self.cmdStatus = cmdStatusDone

       # ---------------------- End keywords for use in commands ---------------------------

       # a drive rate used for simple drive commands
       # we may choose to include this in the drive commands someday
       self.drive_rate = rospy.get_param("~drive_rate", 0.4)

       # a rotate rate used for simple rotate commands
       # we may choose to include this in the drive commands someday
       self.rotate_rate = rospy.get_param("~rotate_rate", 0.6)

       # Minimum distance we want the robot to be from the fiducial
       self.min_dist = rospy.get_param("~min_dist", 0.4)

       # Maximum distance a fiducial can be away for us to attempt to follow
       self.max_dist = rospy.get_param("~max_dist", 4.5)

       # Proportion of angular error to use as angular velocity
       self.angular_rate = rospy.get_param("~angular_rate", 2.0)

       # Maximum angular speed (radians/second)
       self.max_angular_rate = rospy.get_param("~max_angular_rate", 1.2)

       # Angular velocity when a fiducial is not in view
       self.lost_angular_rate = rospy.get_param("~lost_angular_rate", 0.6)

       # Proportion of linear error to use as linear velocity
       self.linear_rate = rospy.get_param("~linear_rate", 1.2)

       # Maximum linear speed (meters/second)
       self.max_linear_rate = rospy.get_param("~max_linear_rate", 1.0)

       # Linear speed decay (meters/second)
       self.linear_decay = rospy.get_param("~linear_decay", 0.9)

       # How many loop iterations to keep linear velocity after fiducial
       # disappears
       self.hysteresis_count = rospy.get_param("~hysteresis_count", 20)

       # How many loop iterations to keep rotating after fiducial disappears
       self.max_lost_count = rospy.get_param("~max_lost_count", 400)

       # ------------------------------------------------------------------
       # Setup ROS topics to be published or subscribed to for operation

       # A publisher for status of this node
       self.statusPub = rospy.Publisher("/follower_status", FollowerStatus, queue_size=1)

       # Subscribe to any optional incoming Commands we will then exectute
       rospy.Subscriber("/follower_commands", FollowerCommand, self.newFollowerCommand)

       # A publisher for robot motion commands
       self.cmdPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

       # Subscribe to incoming transforms
       rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)
       self.fid_x = self.min_dist
       self.fid_y = 0
       self.fid_r = 0
       self.fid_in_view = False
       # ------------------------------------------------------------------

    def publishStatus1Str(self, cmdType, statusState, string1):
        statusMsg = FollowerStatus()
        statusMsg.commandType = cmdType
        statusMsg.statusState = statusState
        statusMsg.string1     = string1
        self.statusPub.publish(statusMsg)

    def publishStatus2Str(self, cmdType, statusState, string1, string2):
        statusMsg = FollowerStatus()
        statusMsg.commandType = cmdType
        statusMsg.statusState = statusState
        statusMsg.string1     = string1
        self.statusPub.publish(statusMsg)

    def publishStatus1StrAndValue(self, cmdType, statusState, string1, num1):
        statusMsg = FollowerStatus()
        statusMsg.commandType = cmdType
        statusMsg.statusState = statusState
        statusMsg.string1     = string1
        statusMsg.num1        = num1
        self.statusPub.publish(statusMsg)

    """
    Called when a FiducialTransformArray is received
    """
    def newTf(self, msg):
        imageTime = msg.header.stamp
        self.linSpeed = 0

        print imageTime, rospy.Time.now()
        print "*****"
        found = False
        self.tfDbgBuffer = self.tfBuffer

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

            # Get the fiducial position relative to the robot center, instead of the camera
            roll = 0.0
            pitch = 0.0
            yaw = 0.0
            if self.debug_verbose == True:
                self.tfDbgBuffer.set_transform(t, "follow")
                tfd = self.tfDbgBuffer.lookup_transform("base_link", t.child_frame_id, imageTime)
                ctd = tfd.transform.translation
                crd = tfd.transform.rotation
                print "T_fidBase Quat for fid %d %lf %lf %lf %lf %lf %lf %lf\n" % \
                             (id, ctd.x, ctd.y, ctd.z, crd.x, crd.y, crd.z, crd.w)
                quat = (crd.x, crd.y, crd.z, crd.w)
                (roll, pitch, yaw) = euler_from_quaternion(quat)
                print "T_fidBase euler for fid %d roll %lf  pitch %lf  yaw (z rot) %lf \n" % (id, roll, pitch, yaw)


            if t.child_frame_id == self.target_fiducial:
                if self.debug_follow > 1:
                    print "Fiducial %s found in transforms." % (self.target_fiducial)
                # We found the fiducial we are looking for
                found = True

                # Add the transform of the fiducial to our buffer
                self.tfBuffer.set_transform(t, "follow")

        if not found:
            if self.debug_follow > 1:
                print "Fiducial %s NOT found." % (self.target_fiducial)
            return # Exit this function now, we don't see the fiducial
        try:
            # Get the fiducial position relative to the robot center, instead of the camera
            tf = self.tfBuffer.lookup_transform("base_link", self.target_fiducial, imageTime)
            ct = tf.transform.translation
            cr = tf.transform.rotation
            quat = (cr.x, cr.y, cr.z, cr.w)
            (roll, pitch, yaw) = euler_from_quaternion(quat)
            print "T_fidBase %lf %lf %lf %lf %lf %lf %lf\n" % \
                             (ct.x, ct.y, ct.z, cr.x, cr.y, cr.z, cr.w)
            if self.debug_verbose == True
                print "T_fidBase euler for fid %d roll %lf  pitch %lf  Z rot (yaw) %lf \n" % (id, roll, pitch, yaw)

            # Set the state varibles to the position and rotation of the fiducial
            self.fid_x = ct.x
            self.fid_y = ct.y
            self.fid_r = yaw
            self.fid_in_view = True

        except:
            traceback.print_exc()
            print "Could not get tf for %s" % self.target_fiducial


    # Here we have a routine to process the newly ready to run command
    # if the command was recognized we return 1 else 0
    def processNewCommand(self, cmdType, actOnDone, strParam1, numParam1, cmdComment):
        # default is when done fetch another command
        self.currentCommand  = cmdType
        self.cmdStatus       = cmdStatusInProgress
        self.cmd_comment     = cmdComment

        retCode = 1

        # set the action on command done so we know what to do after this command 
        self.actOnDone = actOnDoneDoNextCommand

        if cmdType == cmdNameFollowFiducial:
            # set new fiducial to follow with required end action once we have tracked it
            self.target_fiducial = strParam1
            self.actOnDone       = actOnDone      # required parameter for a follow
            self.fid_in_view     = False          # force at least one target acquisition
            self.publishStatus1Str(cmdType, "SetNewFiducial", self.target_fiducial)

        elif cmdType == cmdNameWaitInSeconds:
            # Wait before going on to next command
            self.publishStatus1StrAndValue(cmdType, "Waiting", "Wait for seconds of ", numParam1)
            # this wait would be best doing full loop so we could break out of it. 
            self.waitTimeLeft    = numParam1                  # set the duration of this movement
            self.cmdStatus       = cmdStatusDone

        elif cmdType == cmdNameDriveForward:
            # Drive straight for the specified time in seconds of numParam1 at the current drive_rate
            self.target_fiducial = self.null_fiducial
            self.moveTimeLeft    = numParam1                  # set the duration of this movement
            driveRate            = self.drive_rate            # set the rate for this movement
            self.publishStatus1StrAndValue(cmdType, "Driving", "Drive time in seconds of ", numParam1)

        elif cmdType == cmdNameDriveReverse:
            # Drive in reverse for the specified time in seconds of numParam1 at the current drive_rate
            self.target_fiducial = self.null_fiducial
            self.moveTimeLeft    = numParam1                  # set the duration of this movement
            driveRate            = self.drive_rate * (-1.0)   # set the rate for this movement
            self.publishStatus1StrAndValue(cmdType, "Driving", "Drive time in seconds of ", numParam1)

        elif cmdType == cmdNameRotateLeft:
            # Rotate left for the specified time in seconds of numParam1 at the current rotate_rate
            self.target_fiducial = self.null_fiducial
            self.moveTimeLeft    = numParam1                  # set the duration of this movement
            rotateRate           = self.rotate_rate           # set the rate for this movement
            self.publishStatus1StrAndValue(cmdType, "RotatingLeft", "Rotate time in seconds of ", numParam1)

        elif cmdType == cmdNameRotateRight:
            # Rotate right for the specified time in seconds of numParam1 at the current rotate_rate
            self.target_fiducial = self.null_fiducial
            self.moveTimeLeft    = numParam1                  # set the duration of this movement
            rotateRate           = self.rotate_rate * (-1.0)  # set the rate for this movement
            self.publishStatus1StrAndValue(cmdType, "RotatingRight", "Rotate time in seconds of ", numParam1)

        elif cmdType == cmdNameClearCommands:
            # The queue has been cleared for all but this one but we may need to
            # cleanup some state in this background loop so do that now
            self.cmdStatus       = cmdStatusDone
        else:
            print "Invalid command of '%s'. No action taken" % (cmdType)
            retCode = 0
        return retCode

    """
    Called when a Follower Command is received
    """
    def newFollowerCommand(self, msg):
        cmdType    = msg.commandType
        actOnDone  = msg.actionOnDone
        strParam1  = msg.strParam1
        strParam2  = msg.strParam2
        numParam1  = msg.numParam1
        numParam2  = msg.numParam2
        cmdComment = msg.comment

        print "Received New Command: '%s' actionOnDone: '%s' str1: '%s' num1 %lf comment: '%s'\n" % \
                  (cmdType, actOnDone, strParam1, numParam1, cmdComment)

        # special control commands handled here prior to commands that go into the queue
        if cmdType == cmdNameClearInProgress: 
            self.cmdStatus       = cmdStatusDone
            if self.waitTimeLeft > 0.0:
                self.publishStatus1Str(cmdType, "Waiting", "Canceled due to ClearInProgress")
            if self.moveTimeLeft > 0.0:
                self.publishStatus1Str(cmdType, "Moving", "Canceled due to ClearInProgress")
            self.waitTimeLeft    = 0.0    # clear wait time active or not
            self.moveTimeLeft    = 0.0    # clear move time active or not
        elif cmdType == cmdNameStopMovement: 
            # Set the null fiducial which in effect leads to limbo or the next command if there is one
            self.actOnDone       = actOnDoneDoNextCommand
            self.target_fiducial = self.null_fiducial
            self.cmd_comment     = "clear any current movement command"
            self.cmdStatus       = cmdStatusDone
        elif cmdType == cmdNameSetDriveRate:
            # set a new drive rate in meters per second
            if (numParam1 > 0.02) and (numParam1 < 1.2):
                self.drive_rate = numParam1
                self.publishStatus1StrAndValue(cmdType, "Set drive rate", "Rate in meters per sec of ", numParam1)
            else:
                self.publishStatus1StrAndValue(cmdType, "Set drive rate", "Illegal rate in meters per sec of ", numParam1)
        elif cmdType == cmdNameSetRotateRate:
            # set a new rotation rate in radians per second
            if (numParam1 > 0.02) and (numParam1 < 3.14):
                self.rotate_rate = numParam1
                self.publishStatus1StrAndValue(cmdType, "Set rotate rate", "Rate in radians per sec of ", numParam1)
            else:
                self.publishStatus1StrAndValue(cmdType, "Set rotate rate", "Illegal rate in radians per sec of ", numParam1)
        elif cmdType == cmdNameSetMaxLinRate:
            # set a new maximum follow linear rate in meters per sec
            if (numParam1 > 0.02) and (numParam1 < 1.20):
                self.max_linear_rate = numParam1
                self.publishStatus1StrAndValue(cmdType, "Set max linear approach rate", "Rate in meters per sec of ", numParam1)
            else:
                self.publishStatus1StrAndValue(cmdType, "Set max linear rate", "Illegal rate in meters per sec of ", numParam1)
        elif cmdType == cmdNameSetMaxAngRate:
            # set a new maximum follow rotation rate in radians per sec
            if (numParam1 > 0.02) and (numParam1 < 2.00):
                self.max_angular_rate = numParam1
                self.publishStatus1StrAndValue(cmdType, "Set max angular rate", "Rate in radians per sec of ", numParam1)
            else:
                self.publishStatus1StrAndValue(cmdType, "Set max angular rate", "Illegal rate in radians per sec of ", numParam1)
        else:
            if cmdType == cmdNameClearCommands:
                # clear command queue clears queue so it is first and also picked up for any cleanup in loop
                self.commandQueue    = []

            # place this command and its parameters onto the queue
            # This packing must be consistent with commandQueue unpacking in main routine 
            self.commandQueue.append((cmdType, actOnDone, strParam1, strParam2, numParam1, numParam2, cmdComment))


    """
    Main loop
    """
    def run(self):
        # setup for looping at 20hz
        rate = rospy.Rate(self.loop_hz)
        secPerLoop = 1.0 / self.loop_hz

        # Setup the variables that we will use later
        linSpeed = 0.0
        angSpeed = 0.0
        times_since_last_fid = 0

        cmdType    = 0
        cmdParam1  = 0
        cmdParam2  = 0
        cmdString1 = "none"

        # these values control drive and rotate speeds while a movement command is active
        driveRate  = 0.0
        rotateRate = 0.0

        self.publishStatus1Str("ProgramStatus", "Starting", " ")

        # While our node is running
        while not rospy.is_shutdown():

            # If we are not held off due to a command still in progres then search for new commands
            # The cmdStatus of InProgress can be cleared but only as a special command
            # of clearInProgress received right off of the inbound command topic
            inboundCmdCount = len(self.commandQueue)
            if (inboundCmdCount > 0) and (self.cmdStatus != cmdStatusInProgress):
                newCommand  = self.commandQueue.pop(0)

                # This unpacking must be consistent with commandQueue append on message reception
                # The command message is generic where the context of the parameters are cmdType specific
                cmdType, actOnDone, strParam1, strParam2, numParam1, numParam2, cmdComment = newCommand

                print "Process next queued Command: %s actionOnDone %s str1 %s num1 %lf comment %s\n" % \
                    (cmdType, actOnDone, strParam1, numParam1, cmdComment)

                # Interprit the command and set state based on the command
                cmdValid = self.processNewCommand(cmdType, actOnDone, strParam1, numParam1, cmdComment)

                if cmdValid == 1:
                    # some commands need more work here
                    if cmdType == cmdNameClearCommands:
                        # The queue has been cleared for all but this one but we may need to
                        # cleanup some state in this background loop so do that now
                        self.cmdStatus       = cmdStatusDone
                        driveRate            = 0.0
                        rotateRate           = 0.0
                else:
                    print "Invalid command of '%s'. No action taken" % (cmdType)

            # Handle wait command and still allow it to be aborted by new message
            # We could have a special abort the wait using a button in future if desired
            if (self.waitTimeLeft > 0.0):
                self.waitTimeLeft = self.waitTimeLeft - secPerLoop 
                if (self.waitTimeLeft <= 0.0):
                    self.publishStatus1Str(cmdType, "Waiting", "Done with waiting")
                    self.cmdStatus       = cmdStatusDone   # allows next command to be read

            # Handle drive and rotate commands until they time out
            if (self.moveTimeLeft > 0.0):
                self.moveTimeLeft = self.moveTimeLeft - secPerLoop 
                twist = Twist()
                twist.angular.z = 0.0
                twist.linear.x  = 0.0
                if (self.cmdStatus == cmdStatusInProgress) and (self.moveTimeLeft > 0.0):
                    if (cmdType == cmdNameDriveForward) or (cmdType == cmdNameDriveReverse): 
                        twist.linear.x   = driveRate
                    if (cmdType == cmdNameRotateLeft) or (cmdType == cmdNameRotateRight):
                        twist.angular.z  = rotateRate
                else:
                    if (cmdType == cmdNameDriveForward) or (cmdType == cmdNameDriveReverse): 
                        self.publishStatus1Str(cmdType, "Done", "Driving completed")
                    elif (cmdType == cmdNameRotateLeft) or (cmdType == cmdNameRotateRight):
                        self.publishStatus1Str(cmdType, "Done", "Rotation completed")
                    self.cmdStatus       = cmdStatusDone
                    driveRate            = 0.0
                    rotateRate           = 0.0

                self.cmdPub.publish(twist)

            # if we do not have a fiducial to track we do the pause and skip follow logic
            if self.target_fiducial == self.null_fiducial:
                print "IDLE: No fiducial to follow. " 
                self.fid_in_view = False
                rate.sleep()
                continue

            # ------------------------------------------------------------------------------------
            # At this time we have a fiducial to follow and we see it or not based on self.fid_in_view

            # Calculate the error in the x and y directions
            forward_error = self.fid_x - self.min_dist
            lateral_error = self.fid_y

            # Calculate the amount of turning needed towards the fiducial
            # atan2 works for any point on a circle (as opposed to atan)
            angular_error = math.atan2(self.fid_y, self.fid_x)

            if self.debug_follow > 0:
                print "DistToFid: %f Errors: forward %f lateral %f angular %f" % \
                    (self.fid_x, forward_error, lateral_error, degrees(angular_error))

            if self.fid_in_view:
                times_since_last_fid = 0
            else:
                times_since_last_fid += 1

            if forward_error > self.max_dist:
                print "Fiducial is too far away"
                linSpeed = 0
                angSpeed = 0
            # A fiducial was detected since last iteration of this loop
            elif self.fid_in_view:
                # Set the turning speed based on the angular error
                # Add some damping based on the previous speed to smooth the motion 
                #angSpeed = (angular_error * self.angular_rate) - (angSpeed / 2.0)
                angSpeed = angular_error * self.angular_rate
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

                if self.debug_follow > 0:
                    print "DistMovement: linSpeed %f angSpeed %f"  % (linSpeed, angSpeed)

            # Hysteresis, don't immediately stop if the fiducial is lost
            elif not self.fid_in_view and times_since_last_fid < self.hysteresis_count:
                # Decrease the speed (assuming linear decay is <1)
                linSpeed *= self.linear_decay

            # Try to refind fiducial continuously if we see it or rotate to search.
            # We will only search for the fiducial if the search_for_target option is True
            elif (self.search_for_target == True) and (self.fid_in_view == False) \
                and (times_since_last_fid < self.max_lost_count):
                # Stop moving forward
                linSpeed = 0
                # Keep turning in the same direction
                if angSpeed < 0:
                    angSpeed = -self.lost_angular_rate
                elif angSpeed > 0:
                    angSpeed = self.lost_angular_rate
                else:
                    angSpeed = 0
                print "Try keep rotating to refind fiducial %s try# %d" % (self.target_fiducial, times_since_last_fid)
            else:
                # go to zero movement 
                angSpeed = 0
                linSpeed = 0

            if self.debug_follow > 1:
                print "Speeds: linear %f angular %f" % (linSpeed, angSpeed)

            # Create a Twist message from the velocities and publish it
            # Avoid sending repeated zero speed commands, so teleop can work
            zeroSpeed = (angSpeed == 0 and linSpeed == 0)
            if not zeroSpeed:
                self.suppressCmd = False

            if (self.fid_in_view == True) and (forward_error <= (self.min_dist * 1.005)) \
                and (abs(linSpeed) < 0.01) and (abs(angSpeed) < 0.01):
                # Here is logic to stop following if we think we are at the target
                # and we see the target and action once found is set to stop following
                fidDistanceX = self.fid_x 
                fidRotationZ = self.fid_r 
                if  (self.cmdStatus == cmdStatusInProgress) and \
                    (self.currentCommand == cmdNameFollowFiducial):
                   print "Arrived at following fiducial %s  actionOnDone %s comment %s\n" % \
                       (self.target_fiducial, self.actOnDone, self.cmd_comment)
                   if (self.actOnDone == actOnDoneDriveOnTop) or (self.actOnDone == actOnDoneAssumePose):
                       # Roughly Drive over the fiducial 
                       print "fiducial %s  is %f meters away at angle of %s radians " % \
                           (self.target_fiducial, fidDistanceX, fidRotationZ)
                       self.publishStatus2Str(cmdType, "AtFiducial", self.target_fiducial, "Drive to over the Fiducial")
                       twist = Twist()
                       twist.angular.z = 0.0
                       twist.linear.x  = 0.1   # a rather slow speed for accuracy of simple movement
                       # using slow speed drive over the fiducial
                       self.moveTimeLeft = fidDistanceX / twist.linear.x
                       print "Drive over fiducial for %f sec" % (self.moveTimeLeft)
                       while self.moveTimeLeft > 0.0:
                           self.moveTimeLeft = self.moveTimeLeft - secPerLoop 
                           self.cmdPub.publish(twist)
                           rate.sleep()

                       self.publishStatus2Str(cmdType, "AtFiducial", self.target_fiducial, "Drove to location of fiducial") 

                       if self.actOnDone == actOnDoneAssumePose:
                           # If assuming the pose then using slow speed rotate over the fiducial
                           twist.linear.x  = 0.0
                           twist.angular.z = 0.2     # a rather slow rotation
                           if (fidRotationZ < 0.0):
                               twist.angular.z = twist.angular.z * -1.0
                           self.moveTimeLeft = abs(fidRotationZ / twist.angular.z)
                           print "Rotate over fiducial for %f sec" % (self.moveTimeLeft)
                           self.publishStatus2Str(cmdType, "AtFiducial", self.target_fiducial, "Rotate to pose of fiducial")
                           while self.moveTimeLeft > 0.0:
                               self.moveTimeLeft = self.moveTimeLeft - secPerLoop 
                               self.cmdPub.publish(twist)
                               rate.sleep()
                           self.target_fiducial = self.null_fiducial
                           self.fid_in_view = False
                           self.publishStatus2Str(cmdType, "AtFiducial", self.target_fiducial, "Assumed pose of fiducial")
                           self.cmdStatus = cmdStatusDone
                       else:
                           # This is the just drive on top and don't rotate situation
                           self.cmdStatus = cmdStatusDone
                           self.target_fiducial = self.null_fiducial
                           self.fid_in_view = False

                   elif (self.actOnDone != actOnDoneKeepFollowing):
                       self.cmdStatus = cmdStatusDone
                       self.target_fiducial = self.null_fiducial
                       self.fid_in_view = False
                       self.publishStatus2Str(cmdType, "AtFiducial", self.target_fiducial, "Proceeding to next command")

            if self.debug_follow > 1:
                print "zero", zeroSpeed, self.suppressCmd
            if not self.suppressCmd:
                twist = Twist()
                twist.angular.z = angSpeed
                twist.linear.x = linSpeed
                self.cmdPub.publish(twist)
                if zeroSpeed:
                    self.suppressCmd = True

            # We already acted on the current fiducial
            self.fid_in_view = False
            rate.sleep()


if __name__ == "__main__":
    # Create an instance of our follow class
    node = Follow()
    # run it
    node.run()

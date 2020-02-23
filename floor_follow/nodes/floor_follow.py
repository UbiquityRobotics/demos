#!/usr/bin/python

"""
Copyright (c) 2020, Ubiquity Robotics
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
Floor Follow is meant to follow fiducials on the floor to carry out instructions from a client

This node accepts actionlib goals which tell this module to do things like go to a fiducial
and many more operations of a more advanced nature in dealing with fiducials on the floor.

The camera is meant to be elevated to follow floor fiducials well.

This node is not a complient actionlib server until the states are properly handled.
For info on actionlib server states see http://wiki.ros.org/actionlib/DetailedDescription
"""

import roslib
roslib.load_manifest('floor_follow')
import rospy
import actionlib
import actionlib_msgs
import follow_actions.msg
from follow_actions.msg import DoFollowCmdAction

# our custom messages for the commands we will follow
from custom_messages.msg import FollowerCommand, FollowerStatus

from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Range

import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time


def degrees(r):
    return 180.0 * r / math.pi


"""
    An actionlib server to perform goals from a client to move the robot and seek fiducials
"""
class DoFloorFollowServer:
    # create messages that are used to publish feedback/result
    _goal = follow_actions.msg.DoFollowCmdGoal()
    _feedback = follow_actions.msg.DoFollowCmdFeedback()
    _result = follow_actions.msg.DoFollowCmdResult()


    """
    Constructor for our class
    """
    def __init__(self):

       self.commandQueue = []

       # Set the rate the main loop will run at
       self.loop_hz = 25

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

       # The current fiducial we will be following 
       self.target_fiducial = self.null_fiducial

       # optionally we can disable verbose debug messages or leave them on as in legacy code
       # these were on in legacy version but can be disabled using parameter below
       # 1 is less verbose and 2 more verbose with 0 almost no messages
       self.debug_follow  = rospy.get_param("~debug_follow", 1)
       self.debug_verbose = rospy.get_param("~debug_verbose", 1)
       self.debug_cmdvel  = 0    # set to 0 for sending to /cmd_vel else we do not publish to /cmd_vel topic

       # ---------------------- start of keywords for use in commands ---------------------------

       # Defines that are of value to pass in as elements of messages in the inbound command topic
       print "DEBUG: INIT Commands and actions \n"

       # default action we will use once we actually find and have approached the fiducial
       # The awaitNextCommand action will continue to follow until a new command comes in
       self.actOnDone = 0

       # We need a state to indicate for the current command so we define 'InProgress' and 'Done'
       # When cmdStatus is InProgress we do not read new command except state control commands
       self.cmdStatus = 0

       # the results for the prior command
       self.cmdResult = 0
       self.cmdErrorCode  = 0

       # ---------------------- End keywords for use in commands ---------------------------

       # a drive rate used for simple drive commands
       # we may choose to include this in the drive commands someday
       self.drive_rate = rospy.get_param("~drive_rate", 0.4)
       self.drive_speed = 0.0

       # a rotate rate used for simple rotate commands
       # we may choose to include this in the drive commands someday
       self.rotate_rate = rospy.get_param("~rotate_rate", 0.6)
       self.rotate_speed = 0.0

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

       # The gain used to approach target. We still cap speed at max_linear_rate
       self.approach_gain = rospy.get_param("~approach_gain", 1.0)

       # Maximum linear speed (meters/second)
       self.max_linear_rate = rospy.get_param("~max_linear_rate", 1.5)

       # The percent of target distance we get to before we consider we have arrived
       self.arrival_window = rospy.get_param("~arrival_window", 1.1)

       # Linear speed decay (meters/second)
       self.linear_decay = rospy.get_param("~linear_decay", 0.9)

       # How many loop iterations to keep linear velocity after fiducial
       # disappears
       self.hysteresis_count = rospy.get_param("~hysteresis_count", 20)

       # How many loop iterations to keep rotating after fiducial disappears
       self.max_lost_count = rospy.get_param("~max_lost_count", 400)

       # ------------------------------------------------------------------
       # Obstacle avoidance related. Setup an array that will hold the sonar ranges
       self.num_sonars   = 5
       self.sonar_ranges = [None] * self.num_sonars

       # Set a threshold for when we must be stopped when at or closer than this distance
       self.obstacle_stop_range = rospy.get_param("~obstacle_stop_range", 0.3)

       # ------------------------------------------------------------------
       # Setup ROS topics to be published or subscribed to for operation

       # A publisher for status of this node
       self.statusPub = rospy.Publisher("/follower_status", FollowerStatus, queue_size=1)

       # A publisher for robot motion commands
       self.cmdvelPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

       # Subscribe to incoming transforms
       rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newFiducialTfs)

       # Subscribe to sonar range sensors for object detection
       rospy.Subscriber("/sonars", Range, self.rangeCallback)

       self.fid_x = self.min_dist
       self.fid_y = 0
       self.fid_r = 0
       self.fid_in_view = 0
       # ------------------------------------------------------------------

       self.server = actionlib.SimpleActionServer('do_floor_follow', DoFollowCmdAction, self.executeAction, False)

       print "INIT START actionlib server \n"
       self.server.start()

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
    def rangeCallback(self, msg):

        #if msg.header.frame_id == "sonar_1" or msg.header.frame_id == "sonar_2" or msg.header.frame_id == "sonar_3":
        words = msg.header.frame_id.split('_')    # we expect frame_id like  sonar_3 for number 3
        idx = int(words[1])

        # save the most recent sonar range
        self.sonar_ranges[idx] = msg.range

        #if msg.header.frame_id == "sonar_3":
        #    print msg.header.frame_id, words[1], idx, self.sonar_ranges[3], msg.range, 


    """
    Called when a FiducialTransformArray is received
    """
    def newFiducialTfs(self, msg):
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
            if self.debug_verbose > 0 :
                self.tfDbgBuffer.set_transform(t, "follow")
                tfd = self.tfDbgBuffer.lookup_transform("base_link", t.child_frame_id, imageTime)
                ctd = tfd.transform.translation
                crd = tfd.transform.rotation
                print "T_fidBase Quat for fid %d %lf %lf %lf %lf %lf %lf %lf\n" % \
                             (id, ctd.x, ctd.y, ctd.z, crd.x, crd.y, crd.z, crd.w)
                if self.debug_verbose > 1 :
                    quat = (crd.x, crd.y, crd.z, crd.w)
                    (roll, pitch, yaw) = euler_from_quaternion(quat)
                    print "T_fidBase euler for fid %d roll %lf  pitch %lf  yaw (z rot) %lf \n" % (id, roll, pitch, yaw)


            if t.child_frame_id == self.target_fiducial:
                if self.debug_follow > 1:
                    print "Fiducial %s found in transforms." % (self.target_fiducial)
                # We found the fiducial we are looking for
                found = True
                # self.fid_in_view = True

                # Add the transform of the fiducial to our buffer
                self.tfBuffer.set_transform(t, "follow")

        if not found:
            if self.debug_follow > 0 and self.target_fiducial != self.null_fiducial:
                print "Target Fiducial %s NOT found." % (self.target_fiducial)
            self.fid_in_view = 0
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
            if self.debug_verbose > 0 :
                print "T_fidBase euler for fid %d roll %lf  pitch %lf  Z rot (yaw) %lf \n" % (id, roll, pitch, yaw)

            # Set the state varibles to the position and rotation of the fiducial
            self.fid_x = ct.x
            self.fid_y = ct.y
            self.fid_r = yaw
            self.fid_in_view = 1

        except:
            traceback.print_exc()
            print "Could not get tf for %s" % self.target_fiducial


    # Here we have a routine to process the newly ready to run command
    # if the command was recognized we return 1 else 0
    def processNewCommand(self, goal):

        # default is when done fetch another command
        cmdType          = goal.commandType
        self.cmdStatus   = goal.FF_STATUS_CMD_IN_PROGRESS
        self.cmd_comment = goal.comment

        retCode = goal.FF_STATUS_CMD_DONE_OK

        # set the action on command done so we know what to do after this command 
        # by default set that a command is in progress now
        self.actOnDone = goal.FF_ONDONE_DO_NEXT_COMMAND
        self.cmdStatus = goal.FF_STATUS_CMD_IN_PROGRESS

        if cmdType == goal.FF_CMD_IS_FIDUCIAL_SEEN:
            # set new fiducial to seek if we are seeing it in view now
            self.target_fiducial = goal.strParam1
            self.actOnDone       = goal.FF_ONDONE_DO_NEXT_COMMAND  
            self.fid_in_view     = 0              # force at least one target acquisition
            self.publishStatus1Str(cmdType, "QueryIfFiducialIsSeen", self.target_fiducial)

        elif cmdType == goal.FF_CMD_FOLLOW_FIDUCIAL or cmdType == goal.FF_CMD_GOTO_FIDUCIAL_ON_PATH:
            # set new fiducial to follow or approach on a pathwith required end action once we have tracked it
            self.target_fiducial = goal.strParam1
            self.actOnDone       = goal.actionOnDone      # required parameter for a follow
            self.fid_in_view     = 0              # force at least one target acquisition
            self.publishStatus1Str(cmdType, "SetNewFiducial", self.target_fiducial)
            print "Set new follow fiducial to %s " % (self.target_fiducial)

        elif cmdType == goal.FF_CMD_WAIT_IN_SECONDS:
            # Wait before going on to next command
            self.publishStatus1StrAndValue(cmdType, "Waiting", "Wait for seconds of ", goal.numParam1)
            # this wait would be best doing full loop so we could break out of it. 
            self.waitTimeLeft    = goal.numParam1                  # set the duration of this movement
            self.cmdResult       = goal.FF_RESULT_CMD_DONE_OK

        elif cmdType == goal.FF_CMD_DRIVE_FORWARD:
            # Drive straight for the specified time in seconds of numParam1 at the current drive_rate
            self.target_fiducial = self.null_fiducial
            self.moveTimeLeft    = goal.numParam1                  # set the duration of this movement
            self.drive_speed     = self.drive_rate            # set the speed for this movement
            self.publishStatus1StrAndValue(cmdType, "Driving", "Drive time in seconds of ", goal.numParam1)

        elif cmdType == goal.FF_CMD_DRIVE_REVERSE:
            # Drive in reverse for the specified time in seconds of numParam1 at the current drive_rate
            self.target_fiducial = self.null_fiducial
            self.moveTimeLeft    = goal.numParam1                  # set the duration of this movement
            self.drive_speed     = self.drive_rate * (-1.0)   # set the speed for this movement
            self.publishStatus1StrAndValue(cmdType, "Driving", "Drive time in seconds of ", goal.numParam1)

        elif cmdType == goal.FF_CMD_ROTATE_LEFT:
            # Rotate left for the specified time in seconds of numParam1 at the current rotate_rate
            self.target_fiducial = self.null_fiducial
            self.moveTimeLeft    = goal.numParam1                  # set the duration of this movement
            self.rotate_speed    = self.rotate_rate           # set the rate for this movement
            self.publishStatus1StrAndValue(cmdType, "RotatingLeft", "Rotate time in seconds of ", goal.numParam1)

        elif cmdType == goal.FF_CMD_ROTATE_RIGHT:
            # Rotate right for the specified time in seconds of numParam1 at the current rotate_rate
            self.target_fiducial = self.null_fiducial
            self.moveTimeLeft    = goal.numParam1                  # set the duration of this movement
            self.rotate_speed    = self.rotate_rate * (-1.0)  # set the rate for this movement
            self.publishStatus1StrAndValue(cmdType, "RotatingRight", "Rotate time in seconds of ", goal.numParam1)

            # 
            # Administrative or parameter setup commands
            #
        elif cmdType == goal.FF_CMD_CLEAR_COMMANDS:
            # The queue has been cleared for all but this one but we may need to
            # cleanup some state in this background loop so do that now
            self.cmdStatus       = goal.FF_STATUS_CMD_IDLE

        elif cmdType == goal.FF_CMD_SET_MAX_LIN_RATE:
            if (goal.numParam1 < 0.0 or goal.numParam1 > 1.5):
                retCode = goal.FF_STATUS_CMD_PARAM_RANGE 
                self.publishStatus1StrAndValue(cmdType, "SetMaxLinierRate", "Invalid max linear rate!", goal.numParam1)
            else:
                self.max_linear_rate = goal.numParam1
                self.publishStatus1StrAndValue(cmdType, "SetMaxLinierRate", "max linear rate set ok", goal.numParam1)

        elif cmdType == goal.FF_CMD_SET_APPROACH_GAIN:
            if (goal.numParam1 < 0.0 or goal.numParam1 > 6.0):
                retCode = goal.FF_STATUS_CMD_PARAM_RANGE 
                self.publishStatus1StrAndValue(cmdType, "SetApproachGain", "Invalid approach gain!", goal.numParam1)
            else:
                self.approach_gain = goal.numParam1
                self.publishStatus1StrAndValue(cmdType, "SetApproachGain", "approach gain set ok", goal.numParam1)

        elif cmdType == goal.FF_CMD_SET_ARRIVAL_WINDOW:
            if (goal.numParam1 < 0.5 or goal.numParam1 > 1.5):
                retCode = goal.FF_STATUS_CMD_PARAM_RANGE 
                self.publishStatus1StrAndValue(cmdType, "SetArrivalWindow", "Invalid arrival window!", goal.numParam1)
            else:
                self.arrival_window = goal.numParam1
                self.publishStatus1StrAndValue(cmdType, "SetArrivalWindow", "arrival window set ok", goal.numParam1)

        elif cmdType == goal.FF_CMD_SET_DRIVE_RATE:
            if (goal.numParam1 < 0.0 or goal.numParam1 > 1.0):
                retCode = goal.FF_STATUS_CMD_PARAM_RANGE 
                self.publishStatus1StrAndValue(cmdType, "SetDriveRate", "Invalid drive rate!", goal.numParam1)
            else:
                self.drive_speed = goal.numParam1
                self.publishStatus1StrAndValue(cmdType, "SetDriveRate", "Drive rate set ok ", goal.numParam1)

        elif cmdType == goal.FF_CMD_SET_ROTATE_RATE:
            if (goal.numParam1 < 0.0 or goal.numParam1 > 12.6):
                retCode = goal.FF_STATUS_CMD_PARAM_RANGE 
                self.publishStatus1StrAndValue(cmdType, "SetRotateRate", "Invalid rotate rate!", goal.numParam1)
            else:
                self.rotate_speed = goal.numParam1
                self.publishStatus1StrAndValue(cmdType, "SetRotateRate", "Rotate rate set ok ", goal.numParam1)
        else:
            print "Invalid command of %d. No action taken" % (cmdType)
            self.cmdStatus       = goal.FF_STATUS_CMD_IDLE
            retCode = goal.FF_STATUS_CMD_INVALID 
        return retCode

    # Do object range detection in this routine which uses ongoing sonar distances
    # This will return nearest obstical distance in meters
    # We can apply weights to different sensors to adjust for their angle for example
    def getNearestObstacleRange(self):
        nearestDetection = 100.0    # set FAR past possible detection range
        for i in range(self.num_sonars):
            if self.sonar_ranges[i] < nearestDetection:
                nearestDetection = self.sonar_ranges[i]
        return nearestDetection
         

    # -----------------------------------------------------------------------------------------
    # The handler routines execute specific goals if in the mode their function is required..
    # They return with a status code and a string 
    #. 
    # The return code is an integer which is as follows where errors are negative values
    #   FF_STATUS_CMD_NO_ACTION  No action done (not an error, means no action required by handler)
    #   FF_STATUS_CMD_DONE_OK    The command was executed with success
    #   FF_STATUS_CMD_ERROR      An error speific to the command happened
    #   FF_STATUS_CMD_TIMEOUT    The command took longer than the given timeout
    #   Other negative values    All negative values mean action was taken but an error occurred
    # The string is informational only
    # -----------------------------------------------------------------------------------------

    #
    # handler for wait command.  
    # CMD handlers Return 0 on no action required and 1 for 'done' and -1 for done with error
    def handle_waitCommand(self, goal, loopRate):
        if goal.commandType != goal.FF_CMD_WAIT_IN_SECONDS or self.waitTimeLeft <= 0.0:
            return goal.FF_STATUS_CMD_NO_ACTION, "No wait required"

        secPerLoop = 1.0 / loopRate 
        r = rospy.Rate(loopRate)
        while (self.waitTimeLeft > 0.0):
            self.waitTimeLeft = self.waitTimeLeft - secPerLoop 
            r.sleep()

        return goal.FF_STATUS_CMD_DONE_OK, "Wait completed"

    #
    # handler for drive commands
    #
    # TODO  Enhancement is to Implement odom based driving and rotations
    def handle_driveCommand(self,  goal, driveSpeed, rotateSpeed, loopRate, timeout):
        cmdType = goal.commandType

        if  cmdType != goal.FF_CMD_DRIVE_FORWARD and cmdType != goal.FF_CMD_DRIVE_REVERSE and \
            cmdType != goal.FF_CMD_ROTATE_LEFT   and cmdType != goal.FF_CMD_ROTATE_RIGHT:
            return goal.FF_STATUS_CMD_NO_ACTION, "No drive required"

        # If command is correct but no time left it is 'done' by our handler standards
        if (self.moveTimeLeft <= 0.0):
            self.moveTimeLeft = 0.0
            return goal.FF_STATUS_CMD_DONE_OK, "No action. Drive command already complete"

        # In order to avoid objects or slow to not hit them we detect things in front
        nearestObject = self.getNearestObstacleRange()

        secInProgress = 0.0
        secPerLoop = 1.0 / loopRate 
        r = rospy.Rate(loopRate)

        print "Do Move for cmd %d. Status %d driveSpeed %f rotateSpeed %f with %f sec left. NearObj at %f" % \
            (cmdType, self.cmdStatus, driveSpeed, rotateSpeed, self.moveTimeLeft, nearestObject)

        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x  = 0.0

        while self.moveTimeLeft > 0.0:
            if (cmdType == goal.FF_CMD_DRIVE_FORWARD) or (cmdType == goal.FF_CMD_DRIVE_REVERSE): 
                twist.linear.x   = driveSpeed
            if (cmdType == goal.FF_CMD_ROTATE_LEFT)   or (cmdType == goal.FF_CMD_ROTATE_RIGHT):
                twist.angular.z  = rotateSpeed

            # The simple drive forward command will stop in error if an object is too close
            nearestObject = self.getNearestObstacleRange()
            if cmdType == goal.FF_CMD_DRIVE_FORWARD and nearestObject <= self.obstacle_stop_range:
                return goal.FF_STATUS_CMD_OBSTACLE_CLOSE, "STOP for object too close for forward drive command!"

            # publish the required /cmd_vel message to motor controller
            if self.debug_cmdvel == 0: 
                self.cmdvelPub.publish(twist)

            self.moveTimeLeft = self.moveTimeLeft - secPerLoop 

            secInProgress += secPerLoop
            if secInProgress > timeout:
                return goal.FF_STATUS_CMD_TIMEOUT, "Timeout in the drive command!"
            r.sleep()

        # After the drive is done wrap thing up
        if (cmdType == goal.FF_CMD_DRIVE_FORWARD) or (cmdType == goal.FF_CMD_DRIVE_REVERSE): 
            self.publishStatus1Str(cmdType, "Done", "Driving completed")
        elif (self.cmdType == goal.FF_CMD_ROTATE_LEFT) or (cmdType == goal.FF_CMD_ROTATE_RIGHT):
            self.publishStatus1Str(cmdType, "Done", "Rotation completed")
        self.cmdStatus       = goal.FF_STATUS_CMD_IDLE
        self.cmdResult       = goal.FF_RESULT_CMD_DONE_OK

        return goal.FF_STATUS_CMD_DONE_OK, "No action. Drive command already complete"


    # -----------------------------------------------------------------------------------------
    # handler for query if a specific fiducial is in view
    #
    def handle_isFiducialSeen(self, goal, target_fiducial):
        if goal.commandType != goal.FF_CMD_IS_FIDUCIAL_SEEN:
            return goal.FF_STATUS_CMD_NO_ACTION, "No action required"

        # set target fiducial as the one we are doing this query upon
        loops_of_searching = 60
        fiducial_is_seen = 0
        r = rospy.Rate(20)
        while fiducial_is_seen == 0 and loops_of_searching > 0:
            if self.fid_in_view == 1:
                fiducial_is_seen = 1;
            r.sleep()
            loops_of_searching -= 1

        if fiducial_is_seen:
            return goal.FF_STATUS_CMD_DONE_OK, "The fiducial is seen now"
        else:
            return goal.FF_STATUS_CMD_FID_NOT_SEEN, "The fiducial is not seen"
        

    # -----------------------------------------------------------------------------------------
    # handler for approaching a fiducial
    #
    # This handler relies on many node global variables and is mostly here to limit clutter in main code
    # This should be called only if we have the fiducial to follow already in view per  self.fid_in_view
    #
    def handle_approachFiducial(self, goal, target_fiducial, approach_gain, arrival_window):
        if goal.commandType != goal.FF_CMD_FOLLOW_FIDUCIAL and goal.commandType != goal.FF_CMD_GOTO_FIDUCIAL_ON_PATH:
            return goal.FF_STATUS_CMD_NO_ACTION, "No action required"

        # Calculate the error in the x and y directions
        forward_error = self.fid_x - self.min_dist
        lateral_error = self.fid_y
        linSpeed = 0.0
        angSpeed = 0.0
        loops_since_fid_last_seen = 0

        # Calculate the amount of turning needed towards the fiducial
        # atan2 works for any point on a circle (as opposed to atan)
        angular_error = math.atan2(self.fid_y, self.fid_x)

        if self.fid_in_view == 1:
            loops_since_fid_last_seen = 0
        else:
            loops_since_fid_last_seen += 1

        if self.debug_follow > 0 and (loops_since_fid_last_seen % 10) == 0:
            print "Fid: %s Dist %f in_view %d  missing count %d Errors: forward %f lateral %f angular %f" % \
                (target_fiducial, self.fid_x, self.fid_in_view, loops_since_fid_last_seen, \
                forward_error, lateral_error, degrees(angular_error))

        # Decide on movement required from observations with or without fiducial being seen
        if forward_error > self.max_dist:
            return goal.FF_STATUS_CMD_FID_TOO_FAR,"Fiducial is too far away to approach"

        elif self.fid_in_view == 1:
            # A fiducial was detected since last iteration of this loop

            # Set the turning speed based on the angular error
            # Add some damping based on the previous speed to smooth the motion 
            #angSpeed = (angular_error * self.angular_rate) - (angSpeed / 2.0)
            angSpeed = angular_error * self.angular_rate
            # Make sure that the angular speed is within limits
            if angSpeed < -self.max_angular_rate:
                angSpeed = -self.max_angular_rate
            if angSpeed > self.max_angular_rate:
                angSpeed = self.max_angular_rate

            # Set the forward speed which backs down as we get closer to the fiducial
            # Normally approach_gain is 1.0 but to do faster end approach we use higher value
            linSpeed = forward_error * self.linear_rate * self.approach_gain

            # Cap the linear speed within limits
            if linSpeed < -self.max_linear_rate:
                linSpeed = -self.max_linear_rate
            if linSpeed > self.max_linear_rate:
                linSpeed = self.max_linear_rate

            if self.debug_follow > 0:
                print "DistMovement: linSpeed %f angSpeed %f"  % (linSpeed, angSpeed)
            twist = Twist()
            twist.linear.x  = linSpeed
            twist.angular.z = angSpeed
            if self.debug_cmdvel == 0: 
                self.cmdvelPub.publish(twist)

        else:
            # A fiducial was NOT detected since last iteration of this loop

            if loops_since_fid_last_seen < self.hysteresis_count:
                # Hysteresis, don't immediately stop if the fiducial is lost
                # Decrease the speed (assuming linear decay is <1)
                linSpeed *= self.linear_decay

            # Indicate to caller that the fiducial is no longer in view 
            # The complexity of finding it will be left to the caller
            else:
                return goal.FF_STATUS_CMD_FID_WAS_LOST,"Fiducial is no longer in view"

        if self.debug_follow > 1:
            print "Speeds: linear %f angular %f" % (linSpeed, angSpeed)

        # Create a Twist message from the velocities and publish it

        # Avoid sending repeated zero speed commands, so teleop can work
        zeroSpeed = (angSpeed == 0 and linSpeed == 0)
        if not zeroSpeed:
            self.suppressCmd = False

        # Determine if we have reached the target close enough to move on to next goal
        # for FOLLOW we really zero in on the fiducial but for PATH mode we move on to next one when close
        if (self.fid_in_view == 1 and abs(forward_error) <= (self.min_dist * self.arrival_window)) and \
            ((goal.commandType == goal.FF_CMD_FOLLOW_FIDUCIAL and abs(linSpeed) < 0.01 and abs(angSpeed) < 0.01) or \
            goal.commandType == goal.FF_CMD_GOTO_FIDUCIAL_ON_PATH):
            # Here is logic to stop following if we think we are at the target
            # and we see the target and action once found is set to stop following
            fidDistanceX = self.fid_x 
            fidRotationZ = self.fid_r 

            # for FOLLOW_FIDUCIAL we have option to drive on top slowly and also assume fiducial pose
            if  (self.cmdStatus == goal.FF_STATUS_CMD_IN_PROGRESS) and \
                (self.currentCommand == goal.FF_CMD_FOLLOW_FIDUCIAL):
                print "Arrived at following fiducial %s  actionOnDone %d comment %s\n" % \
                    (target_fiducial, self.actOnDone, self.cmd_comment)

                if self.actOnDone == goal.FF_ONDONE_DRIVE_ON_TOP or self.actOnDone == goal.FF_ONDONE_ASSUME_POSE:
                    # Roughly Drive over the fiducial 
                    print "fiducial %s  is %f meters away at angle of %s radians " % \
                        (target_fiducial, fidDistanceX, fidRotationZ)
                    self.publishStatus2Str(goal.commandType, "AtFiducial", target_fiducial, "Drive to over the Fiducial")
                    twist = Twist()
                    twist.angular.z = 0.0
                    twist.linear.x  = 0.1   # a rather slow speed for accuracy of simple movement
                    # using slow speed drive over the fiducial
                    self.moveTimeLeft = fidDistanceX / twist.linear.x
                    print "Drive over fiducial for %f sec" % (self.moveTimeLeft)
                    loopRate = 20.0
                    secPerLoop = 1.0 / loopRate 
                    r = rospy.Rate(loopRate)
                    while self.moveTimeLeft > 0.0:
                        self.moveTimeLeft = self.moveTimeLeft - secPerLoop 
                        if self.debug_cmdvel == 0: 
                            self.cmdvelPub.publish(twist)
                        r.sleep()

                    self.publishStatus2Str(goal.commandType, "AtFiducial", target_fiducial, "Drove to location of fiducial") 

                    if self.actOnDone == goal.FF_ONDONE_ASSUME_POSE:
                        # If assuming the pose then using slow speed rotate over the fiducial
                        twist.linear.x  = 0.0
                        twist.angular.z = 0.2     # a rather slow rotation
                        if (fidRotationZ < 0.0):
                            twist.angular.z = twist.angular.z * -1.0
                        self.moveTimeLeft = abs(fidRotationZ / twist.angular.z)
                        print "Rotate over fiducial for %f sec" % (self.moveTimeLeft)
                        self.publishStatus2Str(goal.commandType, "AtFiducial", target_fiducial, "Rotate to pose of fiducial")
                        while self.moveTimeLeft > 0.0:
                            self.moveTimeLeft = self.moveTimeLeft - secPerLoop 
                            if self.debug_cmdvel == 0: 
                                self.cmdvelPub.publish(twist)
                            rate.sleep()
                        self.publishStatus2Str(goal.commandType, "AtFiducial", target_fiducial, "Assumed pose of fiducial")
                        return goal.FF_STATUS_CMD_DONE_OK, "At the fiducial and have assumed the pose"
                    else:
                        # This is the just drive on top and don't rotate situation
                        return goal.FF_STATUS_CMD_DONE_OK, "At the fiducial and on top of it now"

            if self.debug_follow > 1:
                print "zero", zeroSpeed, self.suppressCmd

            if not self.suppressCmd:
                twist = Twist()
                twist.angular.z = angSpeed
                twist.linear.x = linSpeed
                if self.debug_cmdvel == 0: 
                    self.cmdvelPub.publish(twist)
                if zeroSpeed:
                    self.suppressCmd = True
                return goal.FF_STATUS_CMD_DONE_OK, "Have approached the fiducial"

        # This handler relies on being called over and over till done
        return goal.FF_STATUS_CMD_IN_PROGRESS, "Still approaching the fiducial"

    """
    DEPRICATED Method:  Receive ActionLib goals and pack them into command queue
    """
    def executeActionToQueue(self, goal):
 
        print "Got goal: Cmd " + str(goal.commandType) + " actOnDone " + str(goal.actionOnDone) + " state is " + str(self.cmdStatus)

        # To keep this module simple we may restrict the queue to only be loaded if we are idle
        # unless we receive some form of abort
        if self.cmdStatus == goal.FF_STATUS_CMD_IDLE:
            # place this command and its parameters onto the queue
            # This packing must be consistent with commandQueue unpacking in main routine 
            
            self.commandQueue.append((goal.commandType, goal.actionOnDone, goal.strParam1, goal.numParam1, goal.numParam2, goal.comment))

            #print "DEBUG: Mark goal as succeeded"
            #self.server.set_succeeded()
        else:
            # we reject this goal unless it is to clear all activity or pending operation.
            # TODO: Lots of logic is required to clear or reject one goal in progress. Need to do this next
            print "DEBUG: Mark goal as rejected due to being busy"
            # self.server.set_rejected()



    """
    Receive ActionLib goals and do processing for this one action
    This is a simplier model than the queued action model
    """
    def executeAction(self, goal):
        print "Got goal: Cmd " + str(goal.commandType) + " actOnDone " + str(goal.actionOnDone) + " state is " + str(self.cmdStatus)

        # This unpacking must be consistent with commandQueue append on message reception
        self.cmdType    = goal.commandType 
        self.actOnDone  = goal.actionOnDone 
        self.strParam1  = goal.strParam1
        self.numParam1  = goal.numParam1
        self.numParam2  = goal.numParam2
        self.cmdComment = goal.comment

        print "Process Newly Received Goal:  cmdType %d actionOnDone %d str1 %s num1 %lf comment %s\n" % \
            (self.cmdType, self.actOnDone, self.strParam1, self.numParam1, self.cmdComment)

        # Interprit the command and set state based on the command
        cmdValid = self.processNewCommand(goal) 

        if cmdValid == goal.FF_STATUS_CMD_DONE_OK:
            self.currentCommand  = goal.commandType
            # TODO: some commands for abort or clearing overrides need more work here
            if self.cmdType == goal.FF_CMD_CLEAR_COMMANDS:
                # The queue has been cleared for all but this one but we may need to
                # cleanup some state in this background loop so do that now
                self.cmdStatus       = goal.FF_STATUS_CMD_IDLE
                self.drive_speed     = 0.0
                self.rotate_speed    = 0.0
        else:
            print "Invalid command of %d. No action taken" % (cmdType)
            self.server.set_aborted(None, "Invalid command")

        # -------------------
        # Handle wait command till we have waited long enough
        retCode,retString = self.handle_waitCommand(goal, self.loop_hz,)
        if (retCode == goal.FF_STATUS_CMD_DONE_OK):
            print "Wait command of %f seconds done. %s" % (self.numParam1, retString)
            self.server.set_succeeded(self._result)
            return 
        elif (retCode < 0):
            self.server.set_aborted(self._result)
            return 
        # FF_STATUS_CMD_NO_ACTION is normal and indicates this handler ignored the goal
            
        # -------------------
        # Handle drive and rotate commands until they time out
        retCode,retString = self.handle_driveCommand(goal, self.drive_speed, self.rotate_speed, self.loop_hz, 20.0)
        if (retCode == goal.FF_STATUS_CMD_DONE_OK):
            print "Drive command %d at speed %f and rotate of %f done. %s" % (self.cmdType, self.drive_speed, self.rotate_speed, retString)
            self.server.set_succeeded(self._result)
            return 
        elif (retCode == goal.FF_STATUS_CMD_TIMEOUT):
            print "Drive command %d timeout. %s" % (self.cmdType,retString)
            self.server.set_aborted(self._result)
            return 
        elif (retCode < 0):
            print "Drive command %d error of %d. %s" % (self.cmdType,retCode,retString)
            self.server.set_aborted(self._result)
            return 
        # FF_STATUS_CMD_NO_ACTION is normal and indicates this handler ignored the goal
            
        # if we do not have a fiducial to track we do the pause and skip follow logic
        if self.target_fiducial == self.null_fiducial:
            # print "IDLE: No fiducial to follow. " 
            self.fid_in_view = 0
            self.server.set_aborted(self._result)
            return 

        # -------------------
        # Handle looking if a fiducial is being seen now
        retCode,retString = self.handle_isFiducialSeen(goal, self.target_fiducial)
        if (retCode == goal.FF_STATUS_CMD_DONE_OK):
            print "Fiducial %s is seen now" % (self.target_fiducial)
            self.server.set_succeeded(self._result)
            return 
        elif (retCode == goal.FF_STATUS_CMD_FID_NOT_SEEN):
            print "Fiducial %s is not seen" % (self.target_fiducial)
            self.server.set_aborted(self._result)
            return 
        # FF_STATUS_CMD_NO_ACTION is normal and indicates this handler ignored the goal

        # -------------------
        # Handle approaching a fiducial
        retCode = goal.FF_STATUS_CMD_IN_PROGRESS
        secInProgress = 0.0
        timeout = 30.0
        secPerLoop = 1.0 / self.loop_hz 
        r = rospy.Rate(self.loop_hz)
        print "Approach fiducial %s " % (self.target_fiducial)
        while retCode == goal.FF_STATUS_CMD_IN_PROGRESS:
            retCode,retString  = self.handle_approachFiducial(goal, self.target_fiducial, self.approach_gain, self.arrival_window)
            # print "DEBUG: Approach fiducial %s gave retCode %d and %s" % (self.target_fiducial, retCode, retString)
            if (retCode == goal.FF_STATUS_CMD_DONE_OK):
                self.cmdStatus = goal.FF_STATUS_CMD_IDLE
                self.cmdResult = goal.FF_RESULT_CMD_DONE_OK
                self.target_fiducial = self.null_fiducial
                self.fid_in_view = 0
                self.server.set_succeeded(self._result)
                return 

            # Here are some cases the client may wish to consider 
            # We let the client program deal with logic for these cases of failure
            elif retCode == goal.FF_STATUS_CMD_FID_TOO_FAR:
                print "fiducial %s is too far away to approach" % (self.target_fiducial)
                self.server.set_aborted(self._result)
                return 
            elif retCode == goal.FF_STATUS_CMD_FID_WAS_LOST:
                print "fiducial %s is no longer in view" % (self.target_fiducial)
                self.server.set_aborted(self._result)
                return 
            elif (retCode < 0):
                print "Approach of fiducial %s error of %d. %s" % (self.target_fiducial,retCode,retString)
                self.server.set_aborted(self._result)
                return 
            secInProgress += secPerLoop
            if secInProgress > timeout:
                print "Approach of fiducial %s timeout." % (self.target_fiducial)
                self.server.set_aborted(self._result)
                return 
            r.sleep()

        # Other return codes indicate this handler was not required

        # 
        # If we get to here we did not know how to process this goal
        print "Unknown goal with command type %d" % (self.cmdType)
        self.server.set_aborted(self._result)
        return
            

    """
    Main loop
    """
    def run(self):
        print "INIT: Start the run main thread"
        # setup for looping at 25hz
        rate = rospy.Rate(self.loop_hz)
        secPerLoop = 1.0 / self.loop_hz

        # Setup the variables that we will use later
        self.cmdType    = 0
        self.actOnDone  = 0
        self.strParam1  = ""
        self.numParam1  = 0
        self.numParam2  = 0
        self.cmdComment = ""

        # these values control drive and rotate speeds while a movement command is active
        self.drive_speed  = 0.0
        self.rotate_speed = 0.0

        print "Fiducial follow starting with fid %s and looprate %d debug %d" % \
            (self.target_fiducial, self.loop_hz, self.debug_follow)

        # While our node is running
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('floor_follow')

    # Create an instance of our follow class
    server = DoFloorFollowServer()

    server.run()
    rospy.spin()

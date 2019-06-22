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
Rotate to find a fiducial, and then navigate to a series of points

It provides a service /dock, which can be called from the command line:

$ rosservice call /dock

"""

import rospy
import actionlib
from actionlib_msgs.msg import *
import docking.srv as docking
from geometry_msgs.msg import Quaternion, Point, PoseStamped, Pose
from fiducial_msgs.msg import FiducialArray, FiducialMapEntryArray
from move_basic.msg import FollowMode
from std_msgs.msg import String
import move_base_msgs.msg
import tf
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import math
import traceback
from geometry_msgs.msg import Quaternion, Point
from fiducial_slam.srv import AddFiducial
import time


# Utility function to convert radians to degrees
def degrees(r):
    return 180.0 * r / math.pi

def radians(d):
    return d * math.pi / 180.0


# Class to encapsulate our node
class Dock:
    """
    Constructor for our class
    """
    def __init__(self):
        self.ok = False
        rospy.init_node("dock")

        # Set up transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # How many times to rotate during search
	self.rotation_limit = rospy.get_param("~rotation_limit", 8)

        # How much to rotate during search (degrees)
        self.angle_increment = radians(rospy.get_param("~angle_increment", 40))

        # Setup the service we serve
        self.srv = rospy.Service("dock", docking.Dock, self.service_callback)

        # Subscribe to fiducial transform messages
        self.target_fiducial = None
        self.fiducial_sub = rospy.Subscriber("/fiducial_vertices",
                                             FiducialArray, self.fiducial_callback)
        # Subscribe to fiducial map messages
        self.broadcaster = tf.TransformBroadcaster()
        self.map_sub = rospy.Subscriber("/fiducial_map",
                                        FiducialMapEntryArray, self.map_callback)

        # Publish follow mode messages to control the speed
        self.follow_pub = rospy.Publisher("/follow_mode", FollowMode, queue_size=1)

        # Publish messages to control ignoring of fiducials
        self.ignore_pub = rospy.Publisher("/ignore_fiducials", String, queue_size=1)

        # Create a proxy object for the move action server
        self.move = actionlib.SimpleActionClient('/move_base',
                                                 move_base_msgs.msg.MoveBaseAction)
        rospy.loginfo("Waiting for move service to be available")
        if not self.move.wait_for_server(rospy.Duration(10.0)):
           rospy.logerr("Move service not available")
           self.move = None
           return

        self.add_fiducial = rospy.ServiceProxy('/fiducial_slam/add_fiducial',
                                               AddFiducial)

        rospy.loginfo("Ready")
        self.seen_fiducial = False

        self.ok = True

    def map_callback(self, msg):
        # Publish a tf to create a fiducial frame, on the floor
	# behind the fiducial with x pointing backwards
        for fiducial in msg.fiducials:
            if fiducial.fiducial_id == self.target_fiducial:
               t = (fiducial.x, fiducial.y, 0.0)
               q = quaternion_from_euler(0.0, fiducial.ry,
                                         math.pi/2.0 + fiducial.rz)
               self.broadcaster.sendTransform(t, q, rospy.Time.now(),
                                              "fiducial_%d" % self.target_fiducial,
                                              "map")
               self.fiducial_in_map = True

    # Called when map messages are received
    def fiducial_callback(self, msg):
        for fiducial in msg.fiducials:
            if fiducial.fiducial_id == self.target_fiducial:
                self.seen_fiducial = True

    # Publish a message to ignore fiducials
    def ignore_fiducials(self, fid=None):
        if not fid is None:
            msg = "%d-%d,%d-%d" % (0, fid-1, fid+1, 10000)
        else:
            msg = ""
        self.ignore_pub.publish(msg)

    # This is called when we receive a rotate service call
    def service_callback(self, req):
        rospy.loginfo("Dock service call: fiducial %d, waypoints %s" % \
                      (req.fiducial_id, req.waypoints))

        # Create a response to our service which we return later
        response = docking.DockResponse()

        num_rotations = 0
        self.target_fiducial = None
        self.seen_fiducial = False
        self.fiducial_in_map = False
        self.ignore_fiducials(req.fiducial_id)

        self.target_fiducial = req.fiducial_id

        # Rotate until we find the target
        while not self.seen_fiducial and num_rotations < self.rotation_limit:
            t = 0
            while t < 3:
                time.sleep(0.5)
                if self.seen_fiducial:
                   break
                t += 0.5
            if not self.seen_fiducial:
                rospy.loginfo("Rotating to search for fiducial")
                q = quaternion_from_euler(0, 0,
                        self.angle_increment)
                if self.goto_goal(Quaternion(*q), frame="base_link",
                                  targetFrame="odom"):
                    num_rotations += 1
                else:
                    self.ignore_fiducials()
                    response.message = "Error rotating"
                    response.success = False
                    return response
            else:
                rospy.loginfo("Not rotating because fiducial seen")

        # Create a negative response to our rotate service call
        if not self.seen_fiducial:
            self.ignore_fiducials()
            response.message = "Not fiducial seen after rotating"
            response.success = False
            return response

        # Be sure robot has stopped moving
        time.sleep(1.5)

        self.add_fiducial(self.target_fiducial)
        count = 0
        while not self.fiducial_in_map and count < 5:
            time.sleep(1.0)
            count += 1

        if not self.fiducial_in_map:
            self.ignore_fiducials()
            response.message = "Fiducial was not added to map"
            response.success = False
            return response

        # Look up our current position in the fiducial's frame
        trans = self.getPose()
        if trans is None:
            self.ignore_fiducials()
            response.message = "Could not get current position to determine evacuation point"
            rospy.logerr(response.message)
            response.success = False
            return response

        # Waypoints
        waypoints = []
        for wp_str in req.waypoints.split(","):
            elems = wp_str.strip(" ").split()
            if not len(elems) == 4:
                self.ignore_fiducials()
                response.message = "Invalid waypoint %s: expect x, y, theta, v" % elems
                response.success = False
                return response
            x, y, theta, speed = elems
            if x == "X":
                x = trans.transform.translation.x
            if y == "Y":
                y = trans.transform.translation.y
            waypoints.append((float(x), float(y), float(theta), float(speed)))

        # Go to each waypoint in succession
        response.message = ""
        for x, y, theta, speed in waypoints:
            rospy.loginfo("Going to goal %f %f %f", x, y, theta)
            msg = FollowMode()
            msg.speed = speed
            self.follow_pub.publish(msg)
            q = quaternion_from_euler(0, 0, radians(theta))
            p = Point(x, y, 0)
            if not self.goto_goal(Quaternion(*q), position=Point(x, y, 0),
                                  frame="fiducial_%d" % self.target_fiducial):
                 self.ignore_fiducials()
                 response.message += "Error going to goal %s %s %s" % (x, y, theta)
                 response.success = False
                 return response
            else:
                 trans = self.getPose()
                 response.message += "Goal %f %f, actual %f %f; " % (x, y,
                    trans.transform.translation.x, trans.transform.translation.y)

        self.ignore_fiducials()
        response.message += "Completed"
        response.success = True
        return response

    # Look up our current position in the fiducial's frame
    def getPose(self):
        try:
            trans = self.tf_buffer.lookup_transform("fiducial_%d" % self.target_fiducial,
                                                    "base_link",
                                                    rospy.Time(), rospy.Duration(5))
            rospy.loginfo("Current position relative to fiducial %f %f" % \
                          (trans.transform.translation.x,
                           trans.transform.translation.y))
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    # Go to a goal
    def goto_goal(self, orientation, position=Point(), frame="base_link",
                  targetFrame="map"):
        goal = move_base_msgs.msg.MoveBaseGoal()
        pose_base = PoseStamped()
        pose_base.pose.orientation = orientation
        pose_base.pose.position = position
        pose_base.header.frame_id = frame
        try:
            pose_odom = self.tf_buffer.transform(pose_base, targetFrame,
                                                 rospy.Duration(1.0))
        except:
            rospy.logerr("Unable to transform goal into target frame")
            return False
        goal.target_pose = pose_odom
        self.move.send_goal(goal)
        self.move.wait_for_result(rospy.Duration(50.0))
        return self.move.get_state() == GoalStatus.SUCCEEDED

    # Just sleep while the node is running
    def run(self):
        rate = rospy.Rate(1)
        while (not rospy.is_shutdown() and self.ok):
            rate.sleep()
        rospy.loginfo("Dock node exiting")

if __name__ == "__main__":
    # Create an instance of our Dock class
    node = Dock()
    # run it
    node.run()

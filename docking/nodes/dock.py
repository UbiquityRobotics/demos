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

It provides a service /rotate, which can be called from the command line:

$ rosservice call /dock

"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Quaternion, Point, PoseStamped, Pose
from fiducial_msgs.msg import FiducialArray
import move_base_msgs.msg
import tf.transformations
import tf2_ros
import tf2_geometry_msgs
import math
import traceback
from geometry_msgs.msg import Quaternion, Point
import time


EVACUTIONX = 1.5

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
 
        # Waypoints
        self.waypoints = []
        waypoint_str = rospy.get_param("/docking/waypoints", "-1 0 0, 0 0 0")
        for wp_str in waypoint_str.split(","):
            elems = wp_str.strip(" ").split()
            self.waypoints.append(map(float, elems))

        # Setup the service we serve
        self.srv = rospy.Service("dock", Trigger, self.service_callback)
 
        # Subscribe to fiducial messages 
        self.fiducial_sub = rospy.Subscriber("/fiducial_vertices", 
                                             FiducialArray, self.fiducial_callback)

        # Create a proxy object for the move action server
        self.move = actionlib.SimpleActionClient('/move_base',
                                                 move_base_msgs.msg.MoveBaseAction)
        rospy.loginfo("Waiting for move service to be available")
        if not self.move.wait_for_server(rospy.Duration(10.0)):
           rospy.logerr("Move service not available")
           self.move = None
           return
        rospy.loginfo("Ready")
        self.seen_fiducial = False

        self.ok = True


    def fiducial_callback(self, msg):
        if len(msg.fiducials) > 0:
            self.seen_fiducial = True

    # This is called when we receive a rotate service call
    def service_callback(self, req):
        print("Received dock service call")

        # Create a response to our service which we return later
        response = TriggerResponse()

        num_rotations = 0
        self.seen_fiducial = False

        # Rotate until we find the target
        while not self.seen_fiducial and num_rotations < self.rotation_limit:
            time.sleep(2)
            if not self.seen_fiducial:
                rospy.loginfo("Rotating to search for fiducial")
                q = tf.transformations.quaternion_from_euler(0, 0,
                        self.angle_increment)
                if self.goto_goal(Quaternion(*q)):
                    num_rotations += 1
                else:
                    response.message = "Error rotating"
                    response.success = False
                    return response
            else:
                rospy.loginfo("Not rotating because fiducial seen")

        # Create a negative response to our rotate service call
        if not self.seen_fiducial: 
            response.message = "Not fiducial seen after rotating"
            response.success = False
            return response

        # Go to evacuation point. This requires looking up our current position
        try:
            trans = tfBuffer.lookup_transform("map", 'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            response.message = "Could not get current position to determine evacuation point"
            response.success = False
            return response
        evacutationPoint = (EVACUATIONX, trans.transform.translation.y, 0)
        if not self.goto_goal(Quaternion(0, 0, 0, 1), position=evacuationPoint, frame="map"):
            response.message = "Error going to evacuation point"
            response.success = False
            return response

        # Go to each waypoint in succession
        for x, y, theta in self.waypoints:
            rospy.loginfo("Going to goal %f %f %f", x, y, theta)
            q = tf.transformations.quaternion_from_euler(0, 0, radians(theta))
            p = Point(x, y, 0)
            if not self.goto_goal(Quaternion(*q), position=Point(x, y, 0), frame="map"):
                 response.message = "Error going to goal %s %s %s" % (x, y, theta) 
                 response.success = False
                 return response

        response.message = "Did it"
        response.success = True
        return response

    # Go to a goal
    def goto_goal(self, orientation, position=Point(), frame=None):
        goal = move_base_msgs.msg.MoveBaseGoal()
        if frame is None:
            pose_base = PoseStamped()
            pose_base.header.frame_id = "base_footprint"
            pose_base.pose.orientation = orientation
            pose_base.pose.position = position

            try:
                pose_odom = self.tf_buffer.transform(pose_base, "odom", rospy.Duration(1.0))
            except:
                rospy.logerr("Unable to transform goal into odom frame")
                return False
            goal.target_pose = pose_odom
        else:
            goal.target_pose = PoseStamped()
            goal.target_pose.pose.orientation = orientation
            goal.target_pose.pose.position = position
            goal.target_pose.header.frame_id = frame
        # Create goal and actionlib call to rotate
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

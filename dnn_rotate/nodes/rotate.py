#!/usr/bin/python

"""
Copyright (c) 2018, Ubiquity Robotics
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
Rotate towards an object detected with dnn_detect.

It provides a service /rotate, which can be called from the command line:

$ rosservice call /rotate "object: 'bottle'"

As well as providing a service, it is a client to two others: one to trigger
the dnn_detect node to detect objects, and another to move the robot.

"""

import rospy
import dnn_rotate.srv
import dnn_detect.srv
import actionlib
from actionlib_msgs.msg import *
import move_base_msgs.msg
import tf.transformations
import math
import traceback
from geometry_msgs.msg import Quaternion, Point

# Utility function to convert radians to degrees
def degrees(r):
    return 180.0 * r / math.pi


# Class to encapsulate our node
class Rotate:
    """
    Constructor for our class
    """
    def __init__(self):
        self.ok = False
        rospy.init_node("rotate")

        # These paramaters allow us to caluclate the angle per pixel in the image
        # It could alternatively be done with camera_info topic
        self.image_width = rospy.get_param("~image_width", 410)
        self.field_of_view = rospy.get_param("~field_of_view", 1.05)

        # How many times to rotate during search
	self.rotation_limit = rospy.get_param("~rotation_limit", 8)

        # How much to rotate during search (radians)
        self.angle_increment = rospy.get_param("~angle_increment", 0.7)

        # How far to go towards target (meters)
        self.forward_dist = rospy.get_param("~forward_dist", 0.2)

        # Setup the rotate service we serve
        self.srv = rospy.Service("rotate", dnn_rotate.srv.StringTrigger,
                                 self.service_callback)
 
        # Create a proxy object for the move action server
        self.move = actionlib.SimpleActionClient('move_base',
                                                 move_base_msgs.msg.MoveBaseAction)
        rospy.loginfo("Waiting for move service to be available")
        if not self.move.wait_for_server(rospy.Duration(10.0)):
           rospy.logerr("Move service not available")
           return

        # Create a proxy object fot the trigger service we call
        rospy.loginfo("Waiting for detect service to be available")
        try:
            rospy.wait_for_service("/dnn_detect/detect", 10)
            self.trigger = rospy.ServiceProxy("/dnn_detect/detect",
                                              dnn_detect.srv.Detect)
            rospy.loginfo("Rotate node ready")
            self.ok = True
        except:
            rospy.logerr("detect service is not available")

    # This is called when we receive a rotate service call
    def service_callback(self, rotate_req):
        target = rotate_req.object
        print("Received service call: rotate %s" % target)

        # Create a response to our service which we return later
        response = dnn_rotate.srv.StringTriggerResponse()

        found_target = False
        num_rotations = 0

        # Rotate until we find the target
        while not found_target and num_rotations < self.rotation_limit:
            # Make a detection trigger service call
            detect_req = dnn_detect.srv.DetectRequest()
            detect_resp = self.trigger(detect_req)
 
            # Look for matching objects
            best_target = None
            best_confidence = 0.0
            for object in detect_resp.result.objects:
                print object
                if object.class_name == target:
                    found_target = True
                    if object.confidence > best_confidence:
                        best_target = object
                        best_confidence = object.confidence

            if not found_target:
                rospy.loginfo("Rotating to search for %s" % target)
                q = tf.transformations.quaternion_from_euler(0, 0,
                        self.angle_increment)
                if self.goto_goal(Quaternion(*q)):
                    num_rotations += 1
                else:
                    response.response = "Error rotating"
                    return response
            else:
                rospy.loginfo("Not rotating because found object")

        # Create a negative response to our rotate service call
        if best_target is None: 
            response.response = "No %s object found after rotating" % target
            return response

        # Calculate how far we need to rotate
        x_center = (best_target.x_min + best_target.x_max) / 2.0
        angle_per_pixel = self.field_of_view / self.image_width
        angle_to_rotate = (self.image_width/2.0 - x_center) * angle_per_pixel
        rospy.loginfo("Center of target %f, rotation required %f degrees" % \
                      (x_center, degrees(angle_to_rotate)))

        # The angle needs to be in quaternion form (a 3D angle)
        # XXX need to transform
        q = tf.transformations.quaternion_from_euler(0, 0, angle_to_rotate)
        if self.goto_goal(Quaternion(*q)):
            # go forward
            self.goto_goal(Quaternion(0, 0, 0, 1),
                           Point(self.forward_dist,  0, 0))
            response.response = "Rotated to %s" % target
        else:
            response.response = "Error rotating to %s" % target
        return response

    # Go to a goal
    def goto_goal(self, orientation, position=Point()):
        # Create goal and actionlib call to rotate
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.pose.orientation = orientation
        goal.target_pose.pose.position = position
        self.move.send_goal(goal)
        self.move.wait_for_result(rospy.Duration(60.0))

        return self.move.get_state() == GoalStatus.SUCCEEDED

    # Just sleep while the node is running
    def run(self):
        rate = rospy.Rate(1)
        while (not rospy.is_shutdown() and self.ok):
            rate.sleep()
        rospy.loginfo("Rotate node exiting")

if __name__ == "__main__":
    # Create an instance of our Rotate class
    node = Rotate()
    # run it
    node.run()

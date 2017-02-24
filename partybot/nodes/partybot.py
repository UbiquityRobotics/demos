#!/usr/bin/env python
#
# State machine:
#  Search
#  Drive
#  Pause/Offer
#  Drive Away
#
# Sub states:
#  Search
#   Turn left (60 deg)
#   Pause
#   Repeat
#
#  Drive
#   Drive towards face (/roi)
#    limit on size of face, angle to face
#
#  Pause/Offer
#   Play sounds
#   Wait 30 seconds
#
#  Drive Away
#   Turn 120 deg
#   Drive N seconds
#
# Input topics:
#  /roi - face position in image
#       - convert to angle and use for visual servoing
#       - what is our output like if the detector isn't picking up a face?
#  /odom - robot odometry
#
# Parameters:
#  ?camera FOV: field-of-fiew of camera, used for servoing?
#
#  search angle: angle to turn during search phase
#  search time: time to pause while searching
#
#  drive timeout: timeout when chasing a person
#  angle_p: porportion for angular motion
#  target_size: face target size (width in pixels)
#  target_p: porportion for size tracking
#
#  pause time: time to pause while offering a drink
#  offer_files: a list of sound files to play while offering a drink
#
#  return angle: turn angle serving a drink
#  return distance: distance to drive after serving a drink


# States
SEARCH=0
DRIVE=1
PAUSE=2
AWAY=3

import roslib ; roslib.load_manifest('partybot')
import rospy
import threading
import random
import os
from dynamic_reconfigure.server import Server

from partybot.cfg import PartybotConfig
from sensor_msgs.msg import *
from geometry_msgs.msg import Twist

class Partybot:
    def __init__(self):
        rospy.init_node("partybot")
        self.config = None
        self.cfg_srv = Server(PartybotConfig, self.cfg_callback)
        self.roi_sub = rospy.Subscriber("roi", RegionOfInterest,
                self.roi_callback)

        self.cmd_pub = rospy.Publisher("cmd_vel", Twist)

        sound_dir = rospy.get_param("~sound_dir", '/home/turtlebot/groovy/rosbuild/raspberry/partybot/vivian')
        sounds = [
        "A01_wouldyoulikecoke.wav", "B08_havecokeandsmile.wav",
        "D08_pleasetakeacoke.wav", "D09_justgrabacoke.wav",
        "D10_takeacoketheyrefree.wav", "D13_goaheadgrabacoke.wav",
        "D14_goaheadtakeone.wav", "D16_takeacokenow.wav",
        "D17_takecokenow.wav"
        ]
        #sounds = os.listdir(sound_dir)
        sounds = [ os.path.join(sound_dir, d) for d in sounds ]
        self.sounds = rospy.get_param("~sounds", sounds)
        rospy.set_param("~sounds", sounds)

        self.state = SEARCH
        self.search_start = rospy.Time.now()
        self.drive_start = rospy.Time(0)
        self.last_face = rospy.Time(0)
        self.pause_start = rospy.Time(0)
        self.sound_playing = False

        self.roi_cmd = Twist()

    class Sound(threading.Thread):
        def __init__(self, parent, path):
            threading.Thread.__init__(self)
            self.path = path
            self.parent = parent

        def run(self):
            self.parent.sound_playing = True
            os.system("aplay %s"%(self.path))
            rospy.sleep(3)
            self.parent.sound_playing = False
    
    def play_sound(self, path):
        s = self.Sound(self, path)
        s.start()

    def cfg_callback(self, config, level):
        self.config = config
        return config

    def roi_callback(self, roi):
        now = rospy.Time.now()
        self.last_face = now
        if self.state == SEARCH:
            rospy.loginfo("Got ROI")
            self.state = DRIVE
            self.drive_start = now

        if self.state == DRIVE:
            # Target tracking
            target_offset_x = roi.x_offset + roi.width / 2 - self.config.image_width / 2
            target_offset_y = roi.y_offset + roi.height / 2 - self.config.image_height / 2
            # size_offset > 0 -> forward
            size_offset = self.config.target_size - (roi.width * roi.height)

            try:
                percent_offset_x = float(target_offset_x) / float(self.config.image_width)
                percent_offset_y = float(target_offset_y) / float(self.config.image_height)
                percent_offset_size = float(size_offset) / float(self.config.target_size)
            except:
                percent_offset_x = 0
                percent_offset_y = 0
                percent_offset_size = 0

            cmd = Twist()
            got_person = True

            if abs(percent_offset_x) > self.config.image_x_thresh:
                speed = self.config.angle_gain * percent_offset_x
                if speed < 0:
                    direction = -1
                else:
                    direction = 1
                cmd.angular.z = -direction * max(
                        self.config.min_angular_speed, min(
                            self.config.max_angular_speed,
                            abs(speed)))
                got_person = False
            else:
                cmd.angular.z = 0

            if abs(percent_offset_size) > self.config.image_size_thresh:
                speed = self.config.target_gain * percent_offset_size
                if speed < 0:
                    direction = -1
                else:
                    direction = 1

                cmd.linear.x = direction * max(
                        self.config.min_linear_speed, min(
                            self.config.max_linear_speed,
                            abs(speed)))
                got_person = False
            else:
                cmd.linear.x = 0

            self.roi_cmd = cmd
            if got_person:
                rospy.loginfo("Got Person")
                self.state = PAUSE
                self.pause_start = now



    def main(self):
        while self.config == None:
            rospy.loginfo("Waiting for initial config")
            rospy.sleep(1)

        r = rospy.Rate(30)
        i = 0

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.state == SEARCH:
                if i == 0:
                    rospy.loginfo("Searching...")
                search_time = (now - self.search_start).to_sec()
                turn_time = self.config.search_angle / self.config.search_speed
                cmd = Twist()
                if search_time < turn_time:
                    cmd.angular.z = self.config.search_speed
                elif search_time < (turn_time + self.config.search_time):
                    pass
                else:
                    self.search_start = now
                    
                self.cmd_pub.publish(cmd)
            elif self.state == DRIVE:
                if i == 0:
                    rospy.loginfo("Driving...")
                if (now - self.last_face).to_sec() > self.config.face_timeout:
                    rospy.loginfo("Lost Face.")
                    self.state = SEARCH
                if (now - self.drive_start).to_sec() > self.config.drive_timeout:
                    rospy.loginfo("Person Following Timed out")
                    self.state = AWAY
                    self.away_start = now

                self.cmd_pub.publish(self.roi_cmd)
            elif self.state == PAUSE:
                # play sound
                if i == 0:
                    rospy.loginfo("Pausing...")

                if not self.sound_playing:
                    self.play_sound(random.choice(self.sounds))

                # wait
                if (now - self.pause_start).to_sec() > self.config.pause_time:
                    self.state = AWAY
                    self.away_start = now
                    self.sound_playing = False
                self.cmd_pub.publish(Twist())
            elif self.state == AWAY:   
                if i == 0:
                    rospy.loginfo("Away...")
                away_time = (now - self.away_start).to_sec()
                turn_time = self.config.return_angle / self.config.search_speed
                drive_time = self.config.return_dist / self.config.return_speed
                cmd = Twist()
                if away_time < turn_time:
                    cmd.angular.z = self.config.search_speed
                elif away_time < (turn_time + drive_time):
                    cmd.linear.x = self.config.return_speed
                else:
                    self.state = SEARCH
                    self.search_start = now
                
                self.cmd_pub.publish(cmd)
            else:
                rospy.logerr("ERROR: bad state: %d"%(self.state))
                self.state = SEARCH
                self.search_start = now
                self.cmd_pub.publish(Twist())
            r.sleep()
            i += 1
            if i >= 30:
                i = 0
    
        rospy.loginfo("Partybot Exiting")

if __name__ == '__main__':
    bot = Partybot()
    bot.main()



# The Floor Follow Actionlib Server

## The floor_follow.py Actionlib Server

The floor follow server is meant to be controled by a client script or program that uses the ROS actionlib interface.  The server is sent actionlib goals and processes one at a time.  The client issues a goal and awaits a result with status.

The floor_follow server demo uses [aruco_detect](http://wiki.ros.org/aruco_detect)
to detect fiducials in the image feed from a camera.  Default Magni configuration will require a downward facing camera that can see floor fiducials in front of the robot.

The ROS actionlib SimpleActionServer defines the interfaces and for reference please see explanations on the ROS doc for [actionlib_tutorials](http://wiki.ros.org/actionlib_tutorials)

The floor follow server allows for programable operations to be requested by a client that interact with pre placed fiducials on the floor as well as allow for simple movement commands that may be required to keep the next fiducial in view if turns are required.
This new mode is an alternative to our full room navigation methods which are in general more complex but also far more exact in navigation of a work space that uses ceiling fiducial map. 

There is an actionlib interface defined in the follow_actions package in the DoFollowCmd.action file that allows control using an actionlib client.  A list of messages can include new setup parameters as well as fiducials for the robot to follow or act upon. 

The floor_follow.py program implementes a ROS node for this actionlib server.
An example program using many of the commands this node will process can be found in the scripts folder in the floor_follow scripts folder where you will find the client program follower_client.py


### Paramaters

* `target_fiducial`: the fiducial we are following. Use `none` to start idle. Default `fid_49`.
* `min_dist`: the minimum distance in meters to consider it followed. Default `0.4`.
* `max_dist`: the maximum distance in meters where we will try to follow. Default `4.5`.
* `max_linear_rate`: the maximum approach speed used when following in meters per sec. Default `1.0`.
* `max_angular_rate`: the maximum rotation speed used when following in radians per sec. Default `1.2`.
* `search_for_target`: rotate to find the fiducial if it is not in view. Default `True`.
* `lost_angular_rate`: angular rotation rate in radians per sec used in searching. Default `0.6`.
* `drive_rate`: rate in meters per second for the simple drive commands.  Default `0.4`.
* `rotate_rate`: rate in radians per second for the simple rotate commands.  Default `1.5`.

### Publications

* `cmd_vel` (geometry_msgs/Twist): commands to move the robot.
* `follower_status` (custom_msgs/FollowerStatus): shows status as commands are received, started, and completed

### Subscriptions

* `fiducial_transforms` The node listens for fiducial poses created by the aruco detect node

## Following fiducials On The Floor Using Commands

The floor_follow.py script can act on commands sent to it using actionlib interface DoFollowCmdAction  goals
These commands arrive as an actionlib goal and start being processed in the executeAction callback in the node main script, floor_follow.py

The script can be issued a list of commands so the robot can follow a trail of fiducials on the floor.  For optimal floor fiducial following there are some configuration changes needed where the config files will be present starting from a path of /opt/ros/kinetic/share/raspicam_node for the case of ROS Kinetic.  Floor fiducials of 110mm size are a better size for this mode of operation because they end up very close to the camera and we want the robot to drive over the fiducials generally. 

The ROS launch file that does some of the things needed can be run as follows.

    roslaunch magni_demos fiducial_floor_follow.launch

### The following of fiducials on the floor is intended after Config Changes
The camera should be mounted in what we will call and release as the 'downward' mounting.
We hope to support this on an image as soon as early 2020. Contact us on the forum if you need this earlier.

At this time in early 2020 the default raspicam pose of 'downward' is for a camera located 55cm above the floor on a tall mast on the front left side of the Magni which will be pointing downward at 45 degrees.    This pose can be changed but it is a complex matter not presented at this time.

An updated version of the magni.urdf.xacro file and modification of /etc/ubiquity/robot.yaml to specify raspicam pose as `downward` would be required and ideally higher resolution raspicam config and launch files would be best.  We hope to support all of these things for floor fiducial following in our next raspberry Pi image perhaps early in 2020.

The camera resolution set in our launch for this mode will be high such as 1280x920 so detection of fiducials at large angles seen from the robot from over a meter away are possible.  

* Camera config file must be formed and present, for this case it would be in camera_info/camerav2_1280x960.yaml  
* A raspicam launch file must be present where we will initially use launch/camerav2_1280x960_10fps.launch
* The launch file should specify argument 'search_for_target' as  'false'
* The launch file can specify other argument suitable for other drive and rotate needs.


### A very actionlib example client that issues and listens to status

As an example we have a very simple example called follower_client.py in the scripts folder under the floor_follow folder.
Take a look at this to see how to send commands and receive status.   This is simple starting point for your own node if you desire.
Modify the script perhaps first to just drive a little then print and place your own fiducials and have it follow them in some order you define.
Keep in mind that the basic rotate and perhaps the basic drive commands may have to be used after ariving at one fiducial in order to point the robot in the general direction where it can see the next fiducial.
You would first run the fiducial_follow node as already shown above on this page and then the command below would start this sample client node

    python ./follower_client.py 

#### The Actionlib Goal Message

The goal that this node responts to is define in follow_actions folder under subfolder action in file DoFollowCmd.action.  Any changes to this .action file require a catkin_make.   The goal has these fields

* `commandType`  String such as `RotateRight` or `FollowFiducial` to set next target
* `actionOnDone`  String that Tells what will be done after the command.  This is specific to the command used
* `numParam1`  Floating point numeric parameter used if required by the command
* `numParam2`  Second floating point numeric parameter used if required by the command
* `strParam1`  String parameter for commands that need a string such as the next fiducial to follow
* `comment`  This string will show up in the status for the command and could be used for feedback to the sender

The above command is in a general form.  Below are the command types to the left and we indicate which parameter(s) are used for the command.  Any user inputs can be in `comment` and a user may use this for triggering his app or just readable status message.

All commands are processed one at a time by the actionlib server implemented in floor_follow.py

#### The commandTypes To The floor follow server

To simplify this table I will leave off the prefix of FF_CMD_ used in each command type  seen in full in the DoFollowCmd.action file

* `DRIVE_REVERSE`  The numParam1 parameter is the time in seconds to drive straight backwards at the rate set using the `drive_rate` which was set as a parameter or modified most recently using the SetDriveRate command

* `ROTATE_RIGHT`  The numParam1 parameter is the time in seconds to rotate to the right at the rate set using the `rotate_rate` which was set as a parameter or modified most recently using the SetRotateRate command

* `ROTATE_LEFT`  The numParam1 parameter is the time in seconds to rotate to the left at the rate set using the `rotate_rate` which was set as a parameter or modified most recently using the SetRotateRate command

* `FOLLOW_FIDUCIAL` The `strParam1` string will be the next fiducial to be followed such as `fid101`.  The `actionOnDone` is generally assumed to be just to move on to do the next command. You may specify 'DriveOnTop' for actionOnDone so the robot drives itself over the fiducial.  You may specify 'AssumePose' for actionOnDone so the robot drives over then rotates to the pose of the fiducial.  You may specify for actionOnDone to `KeepFollowing` so the robot continues to follow this fiducial just like the legacy fiducial follow mode.    

* `DRIVE_FORWARD`  The numParam1 parameter is the time in seconds to drive straight forward at the rate set using the `drive_rate` which was set as a parameter or modified most recently using the SetDriveRate command

* `SET_MAX_LIN_RATE`  The numParam1 parameter is used as the new `max_linear_rate` for finding fiducials. We limit this between very slow to about 1.2 meters per second.

* `SET_MAX_ANG_RATE`  The numParam1 parameter is used as the new `max_angular_rate` for following fiducials. We limit this between very slow to about 2.0 meters per second.

* `SET_DRIVE_RATE`  The numParam1 parameter is used as the new `drive_rate` for drive commands. We limit this between very slow to about 1.2 meters per second.

* `SET_ROTATE_RATE`  The numParam1 parameter is used as the new `rotate_rate` for rotate commands. We limit this between very slow to about 3.2 meters per second.

* `CLEAR_COMMANDS`  This is a depreciated command and only to be used if we ever re-implement a command queue.  It was used in an earlier architecture

* `STOP_MOVEMENT`  RESERVED command to stop the robot from any current movement command.   FUTURE PLAN

#### The Status Result  

The node will result in a status result which may be valuable for a client node.
Generally we send one status as it is the start of that command to be acted upon and then send a status on completion.
This is to be documented in the future


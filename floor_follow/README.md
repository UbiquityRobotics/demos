

# Nodes

## follow.py

The fiducial_follow demo uses [aruco_detect](http://wiki.ros.org/aruco_detect)
to detect fiducials in the image feed from a camera.  Default usage will follow the default #49 `target_fiducial`. When the `target_fiducial` is in view or search for it if not found which was the only mode of the original version of fiducial follow demo. 

An optional yet flexible interface has been added to greatly increase the possible usage of follow.py to respond to commands on an inbound ros topic and issue status on a second topic.  The new mode allows for programable on the fly following of pre placed fiducials on the floor or walls as well as very simple movement commands that may be required to keep the next fiducial in view prior to going to that fiducial.
This new mode is an alternative to our full room navigation methods which are in general more complex but also far more exact in navigation of a work space that uses ceiling fiducial map. 

There is an interface to follow.py that allows control using messages on topic `/follower_commands`.  A list of messages can include new setup parameters as well as fiducials for the robot to follow or act upon. These commands can be queued in the follow.py app or can be received as the robot progresses and the user app detects status that then requires a new command to be issued to the robot. 
The status for completed commands comes out on topic /follower_status.  Included in the commands are simple movement commands that allow the robot to drive or turn which could be useful to find the next fiducial if it is out of view after the prior fiducial has been found.  This also allows use of different paths through floor based fiducials based on the directions in the commands to turn towards the next fiducial target.


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

* `fiducial_transforms` (fiducial_msgs/FiducialTransformArray): Listen for locations of observed fiducials
* `follower_commands` (custom_msgs/FollowerCommand): Accept commands to process in order of reception

### Example usage for simple single fiducial following
For the simple legacy fiducial follow mode a single aruco fiducial of size 140mm on edge of the black pattern that is number 49 is used.
Prepare such a fiducial and place it where the robot can see it with the camera then start the program.

    roslaunch magni_demos fiducial_follow.launch

## Following fiducials On The Floor Using Commands

The follow.py script can act on commands sent to it using ros topic /follower_commands.
These commands arrive in a general message, FollowerCommand, upon a well known topic of /follower_commands that is monitored by the follower script. 

The script can be issued a list of commands so the robot can follow a trail of fiducials on the floor.  For optimal floor fiducial following there are some configuration changes suggested where the config files will be present starting from a path of /opt/ros/kinetic/share/raspicam_node for the case of ROS Kinetic.  Floor fiducials of 100mm size are a better size for this mode of operation because they end up very close to the camera and we want the robot to drive over the fiducials generally. 

An example launch file that does some of the things needed can be run as follows.

    roslaunch magni_demos fiducial_floor_follow.launch

### The following of fiducials on the floor is intended after Config Changes
The camera should be mounted in what we will call and release as the 'downward' mounting.
We hope to support this on an image as soon as early 2020. Contact us on the forum if you need this earlier.

An updated version of the magni.urdf.xacro file and modification of /etc/ubiquity/robot.yaml to specify raspicam pose as `downward` would be required and ideally higher resolution raspicam config and launch files would be best.  We hope to support all of these things for floor fiducial following in our next raspberry Pi image perhaps early in 2020.

The camera resolution set in our launch for this mode will be high such as 1280x920 so detection of fiducials at large angles seen from the robot from over a meter away are possible.  

* Camera config file must be formed and present, for this case it would be in camera_info/camerav2_1280x960.yaml  
* A raspicam launch file must be present where we will initially use launch/camerav2_1280x960_10fps.launch
* The launch file should specify argument 'search_for_target' as  'false'
* The launch file can specify other argument suitable for other drive and rotate needs.


### A very simple example client that issues and listens to status

As an example we have a very simple example called follower_client.py in the scripts folder.
Take a look at this to see how to send commands and receive status.   This is simple starting point for your own node if you desire.
Modify the script perhaps first to just drive a little then print and place your own fiducials and have it follow them in some order you define.
Keep in mind that the basic rotate and perhaps the basic drive commands may have to be used after ariving at one fiducial in order to point the robot in the general direction where it can see the next fiducial.
You would first run the fiducial_follow node and then the command below would start this sample client node

    python ./follower_client.py 

#### The general purpose command message

The general message has these fields which are standard ROS format types, mostly just strings.
The messages are in demos/custom_messages so that they can stand alone for other apps or nodes

* `commandType`  String such as `RotateRight` or `FollowFiducial` to set next target
* `actionOnDone`  String that Tells what will be done after the command.  This is specific to the command used
* `strParam1`  String parameter for commands that need a string such as the next fiducial to follow
* `strParam2`  Second string parameter if required by the command
* `numParam1`  Floating point numeric parameter used if required by the command
* `numParam2`  Second floating point numeric parameter used if required by the command
* `comment`  This string will show up in the status for the command and could be used for feedback to the sender

The above command is in a general form.  Below are the command types to the left and we indicate which parameter(s) are used for the command.  Any user inputs can be in `comment` and a user may use this for triggering his app or just readable status message.

It is important to keep in mind that sending these commands adds them to a queue.  So if 3 commands are sent right away for the FiducialFollow commandType the 3 will be sought out and driven to one at a time.  All commands are processed in first in first executed order.  

#### The commandTypes To The Fiducial Follow node

* `FollowFiducial` The `strParam1` string will be the next fiducial to be followed such as `fid101`.  The `actionOnDone` is generally assumed to be just to move on to do the next command. You may specify 'DriveOnTop' for actionOnDone so the robot drives itself over the fiducial.  You may specify 'AssumePose' for actionOnDone so the robot drives over then rotates to the pose of the fiducial.  You may specify for actionOnDone to `KeepFollowing` so the robot continues to follow this fiducial just like the legacy fiducial follow mode.    
* `SetMaxLinRate`  The numParam1 parameter is used as the new `max_linear_rate` for finding fiducials. We limit this between very slow to about 1.2 meters per second.
* `SetMaxAngRate`  The numParam1 parameter is used as the new `max_angular_rate` for following fiducials. We limit this between very slow to about 2.0 meters per second.
* `DriveForward`  The numParam1 parameter is the time in seconds to drive straight forward at the rate set using the `drive_rate` which was set as a parameter or modified most recently using the SetDriveRate command

* `DriveReverse`  The numParam1 parameter is the time in seconds to drive straight backwards at the rate set using the `drive_rate` which was set as a parameter or modified most recently using the SetDriveRate command

* `SetDriveRate`  The numParam1 parameter is used as the new `drive_rate` for drive commands. We limit this between very slow to about 1.2 meters per second.

* `RotateRight`  The numParam1 parameter is the time in seconds to rotate to the right at the rate set using the `rotate_rate` which was set as a parameter or modified most recently using the SetRotateRate command

* `RotateLeft`  The numParam1 parameter is the time in seconds to rotate to the left at the rate set using the `rotate_rate` which was set as a parameter or modified most recently using the SetRotateRate command

* `SetRotateRate`  The numParam1 parameter is used as the new `rotate_rate` for rotate commands. We limit this between very slow to about 3.2 meters per second.

* `ClearCommands`  Typically used to flush out any commands not yet executed but queued from prior commands

* `ClearInProgress`  RESERVED command type to stop the robot from any current movement command.  FUTURE PLAN

* `StopMovement`  RESERVED command to stop the robot from any current movement command.   FUTURE PLAN

#### The Status Messages published by Fiducial Follow node

The node will publish status messages which may be valuable for a client node.
At this time these messages are not greatly thought out for some 'grand scheme' as that depends on a users specific needs.
Generally we send one status as it is the start of that command to be acted upon and then send a status on completion.

* `commandType`  String that normally shows the command for which this status applies.
* `statusState`  String that indicates something about the state such as a new command starting to be run or is Done.
* `string1`  String that often states the fiducial involved or just more about the action or more about units something is being set
* `string2`  Second string is for more optional status some commands may fill in as needed.
* `num1`  Floating point numeric value. Often this can be a value for a set type of operation for verification
* `num2`  Second floating point value, not used yet.
* `statusBits`  An unused integer perhaps of value for future or user extensions of this script

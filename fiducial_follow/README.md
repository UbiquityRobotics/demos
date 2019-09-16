

# Nodes

## follow.py

The fiducial_follow demo uses [aruco_detect](http://wiki.ros.org/aruco_detect)
to detect fiducials in the image feed from a camera.  Default usage will follow the`target_fiducial`
when in view or search for it if not found like legacy version.  Movement commands are optionally issued to the script to allow it to follow a path of fiducials or do simple driving operations to re-position to next fiducial.

An optional yet flexible interface has been added to greatly increase the possible usage of follow.py to do useful and programable on the fly following of pre placed fiducials on the floor or walls.
There is an interface to follow.py that allows control using messages on topic /follower_commands which follow.py listens upon.  A list of fiducials for the robot to follow can be received up front or as the robot progresses.
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

### Usage for following a sequence of fiducials and/or performing simple movements

The follow.py script can act on commands sent to it using ros topic /follower_commands.
These commands arrive in a general message, FollowerCommand, upon a well known topic of /follower_commands that is monitored by the follower script. 

The script can be issued a list of commands so the robot can follow a trail of fiducials on the floor.  For optimal floor fiducial following there are some configuration changes suggested where the config files will be present starting from a path of /opt/ros/kinetic/share/raspicam_node for the case of ROS Kinetic.  Floor fiducials of 140mm size up to almost a 2 meters away have been recognized with success. 

#### Optimal following of fiducials on the floor requires Config Changes
The camera resolution should be increased to perhaps 1640x1232 so detection of fiducials from over a meter away is possible.  This allows greater spacing of fiducials on the floor.   Also it is best to turn off searching.
* Camera config file must be formed and present, for this case it would be in camera_info/camerav2_1640x1232.yaml  The config file is best formed from a camera calibration done at the resolution, in this case 1640x1232.
* A raspicam launch file must be present, for this case in launch/camerav2_1640x1232_10fps.launch
* The launch file should specify argument 'search_for_target' as  'false'

#### A very simple example client that issues and listens to status

As an example we have a very simple example called follower_controller.py in the demos repository.
Take a look at this to see how to send commands and receive status.   This is also a nice starting point for your own simple node if you desire.
Modify the script perhaps first to just drive a little then print and place your own fiducials and have it follow them in some order you define.
Keep in mind that the basic rotate may have to be used to point the robot in the general direction of the next fiducial.
You would first run the fiducial_follow node and then the command below would start this sample client node

    python ./follower_controller.py 

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

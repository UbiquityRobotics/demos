

# Nodes

## follow.py

The fiducial_follow demo uses [aruco_detect](http://wiki.ros.org/aruco_detect)
to detect fiducials in the image feed from a camera.  If the `target_fiducial`
is detected, movement commands are issued to the robot to make it move towards
the fiducial.

In late 2019 a simple yet flexible interface has been added to greatly increase the possible usage of follow.py to do useful and programable on the fly following of pre placed fiducials on the floor or walls.
There is an interface to follow.py that allows control using messages on topic /follower_commands which follow.py listens upon.  A list of fiducials for the robot to follow can be received up front or as the robot progresses.
The status for completed commands comes out on topic /follower_status.  Included in the commands are simple movement commands that allow the robot to drive or turn which could be useful to find the next fiducial if it is out of view after the prior fiducial has been found.  This also allows use of different paths through floor based fiducials based on the directions in the commands to turn towards the next fiducial target.


### Paramaters

* `target_fiducial`: the fiducial we are following. Use `none` to start idle. Default `fid_49`.
* `min_dist`: the minimum distance in meters to consider it followed. Default `0.4`.
* `max_dist`: the maximum distance in meters where we will try to follow. Default `4.5`.
* `search_for_target`: rotate to find the fiducial if it is not in view. Default `True`.
* `lost_angular_rate`: angular rotation rate in radians per sec used in searching. Default `0.6`.
* `drive_rate`: rate in meters per second for the simple drive commands.  Default `0.4`.
* `rotate_rate`: rate in radians per second for the simple rotate commands.  Default `1.5`.

### Publications

`cmd_vel`(geometry_msgs/Twist): commands to move the robot.
`follower_status`:(custom_msgs/FollowerStatus)

### Subscriptions

`fiducial_transforms`:(fiducial_msgs/FiducialTransformArray)
`follower_Command`:(custom_msgs/FollowerCommand)

### Example usage for simple single fiducial following

    rosrun fiducial_follow follow.py

### Usage for following a sequence of fiducials and/or performing simple movements

The follow.py script can act on commands sent to it using ros topic /follower_commands.
These commands arrive in a general message, FollowerCommand, upon a well known topic of /follower_commands that is monitored by the follower script. 

The general message has these fields which are standard ROS format types, mostly just strings

* `commandType`  String such as `RotateRight` or `FollowFiducial` to set next target
* `actionOnDone`  String that Tells what will be done after the command.  This is specific to the command used
* `strParam1`  String parameter for commands that need a string such as the next fiducial to follow
* `strParam2`  Second string parameter if required by the command
* `numParam1`  Floating point numeric parameter used if required by the command
* `numParam2`  Second floating point numeric parameter used if required by the command
* `comment`  This string will show up in the status for the command and could be used for feedback to the sender

The above command is in a general form.  Below are the command types to the left and we indicate which parameter(s) are used for the command.  Any user inputs can be in `comment` and a user may use this for triggering his app or just readable status message.

It is important to keep in mind that sending these commands adds them to a queue.  So if 3 commands are sent right away for the FiducialFollow commandType the 3 will be sought out and driven to one at a time.  All commands are processed in first in first executed order.  

* `FollowFiducial` The `strParam1` string will be the next fiducial to be followed such as `fid101`.  The `actionOnDone` is generally assumed to be just to move on to do the next command but in this case you may set `KeepFollowing` so the robot continues to follow this fiducial.    
* `DriveForward`  The numParam1 parameter is the time in seconds to drive straight forward at the rate set using the `drive_rate` which was set as a parameter or modified most recently using the SetDriveRate command

* `DriveReverse`  The numParam1 parameter is the time in seconds to drive straight backwards at the rate set using the `drive_rate` which was set as a parameter or modified most recently using the SetDriveRate command

* `SetDriveRate`  The numParam1 parameter is used as the new `drive_rate` for drive commands. We limit this between very slow to about 1.2 meters per second.

* `RotateRight`  The numParam1 parameter is the time in seconds to rotate to the right at the rate set using the `rotate_rate` which was set as a parameter or modified most recently using the SetRotateRate command

* `RotateLeft`  The numParam1 parameter is the time in seconds to rotate to the left at the rate set using the `rotate_rate` which was set as a parameter or modified most recently using the SetRotateRate command

* `SetRotateRate`  The numParam1 parameter is used as the new `rotate_rate` for rotate commands. We limit this between very slow to about 3.2 meters per second.

* `ClearCommands`  Typically used to flush out any commands not yet executed but queued from prior commands

* `ClearInProgress`  RESERVED command type to stop the robot from any current movement command.  FUTURE PLAN

* `StopMovement`  RESERVED command to stop the robot from any current movement command.   FUTURE PLAN

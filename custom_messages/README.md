

# Nodes

## follow.py

The fiducial_tracker demo uses [aruco_detect](http://wiki.ros.org/aruco_detect)
to detect fiducials in the image feed from a camera.  The demo receives directions
from an input topic for things like 'goto fiducial 4' or turn right or stop.
For a goto the current `target_fiducial` is located and then the robot moves to the target.
A list of fiducials to follow and possible rotatation commands to find next target
allows waypoints to be followed on the floor and final pose to be done with rotate.


### Paramaters

`target_fiducial`: the fiducial we are following. Default `fid_49`.

### Publications

`cmd_vel`(geometry_msgs/Twist): commands to move the robot.

### Subscriptions

`fiducial_transforms`:(fiducial_msgs/FiducialTransformArray)

### Example usage

    rosrun fiducial_tracker fiducial_tracker.py



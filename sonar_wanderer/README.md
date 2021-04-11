

# Nodes

## follow.py

The fiducial_follow demo uses [aruco_detect](http://wiki.ros.org/aruco_detect)
to detect fiducials in the image feed from a camera.  If the `target_fiducial`
is detected, movement commands are issued to the robot to make it move towards
the fiducial.


### Paramaters

`target_fiducial`: the fiducial we are following. Default `fid_49`.

### Publications

`cmd_vel`(geometry_msgs/Twist): commands to move the robot.

### Subscriptions

`fiducial_transforms`:(fiducial_msgs/FiducialTransformArray)

### Example usage

    rosrun fiducial_follow follow.py



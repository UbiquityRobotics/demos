
# Fiducial Follow

The fiducial_follow demo uses [aruco_detect](http://wiki.ros.org/aruco_detect)
to detect fiducials in the image feed from a camera.  If the `target_fiducial`
is detected, movement commands are issued to the robot to make it move towards
the fiducial.


## Paramaters

`target_fiducial` the fiducial we are following. Default `fid49`

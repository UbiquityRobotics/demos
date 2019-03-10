

# Nodes

## docking.py

The docking service uses [fiducials](http://wiki.ros.org/fiducials)
to detect fiducials markers in an image feed from a camera. When the sevice
 is invoked, the robot rotates to find a fiducial in order to determine is
 location and Then navigates to a designated point along a specified path.

### Services Provided

`/dock` (`std_srvs/Trigger`): invokes the docking service


### Publications

`/move_base/goal` (`move_base_msgs/MoveBaseActionGoal`): sends a movement goal

### Parameters

`~waypoints` a string containg a list of poses in the form `x0 y0 theta, x1 y1 theta1`, where
 x and y are distances in meters and theta is an angle in degrees, relative
 to the map frame. Default: "-1 0 0, 0 0 0"

`~angle_increment`: Angle to rotate in degrees if object is not found. Default `40`.

`~rotation_limit`: How many times to rotate `angle_increment` radians if the object is not found,
 before giving up. Default `8`.



### Example usage

    roslaunch docking docking.launch

    rosservice call /dock

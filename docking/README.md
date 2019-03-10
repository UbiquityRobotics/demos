

# Nodes

## docking.py

The docking service uses [fiducials](http://wiki.ros.org/fiducials)
to detect fiducials markers in an image feed from a camera. When the sevice
 is invoked, the robot rotates to find a fiducial in order to determine is
 location and Then navigates to a designated point along a specified path.

### Services Provided

`/dock` (`std_srvs/Empty`): invokes the docking service


### Publications

`/move_base/goal` (`move_base_msgs/MoveBaseActionGoal`): sends a movement goal

### Parameters

`path` a string containg a list of poses in the form (x, y, theta), where
 x and y are distances in meters and theta is an angle in degrees, relative
 to the map frame.

### Example usage

    rosrun docking docking.py


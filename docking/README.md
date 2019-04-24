

# Nodes

## docking.py

The docking service uses [fiducials](http://wiki.ros.org/fiducials)
to detect fiducials markers in an image feed from a camera. When the sevice
 is invoked, the robot rotates to find a fiducial in order to determine is
 location and Then navigates to a set of waypoints in sequence.

### Services Provided

`/dock` (`docking/Dock`): invokes the docking service
The robot will rotate until it either sees the target fiducial or completes one revolution. If a fiducial is seen, it will attempt to navigate to the waypoints in sequence. The format of a waypoint is three elements representing x, y, and theta. x and y are co-ordinates relative to the fiducial in meters and theta is a heading angle in degrees. The strings XandYare replaced by the currentxandy` co-ordinates, respectively.


### Publications

`/move_base/goal` (`move_base_msgs/MoveBaseActionGoal`): sends a movement goal

### Parameters

`~angle_increment`: Angle to rotate in degrees if object is not found. Default `40`.

`~rotation_limit`: How many times to rotate `angle_increment` radians if the object is not found,
 before giving up. Default `8`.



### Example usage

    roslaunch docking docking.launch

    rosservice call /dock "{'fiducial_id': 42, 'waypoints': '-1 Y 0, -1 0 0'}"

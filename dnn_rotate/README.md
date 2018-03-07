

# Nodes

## rotate.py

The dnn_rotate demo uses [dnn_detect](http://wiki.ros.org/dnn_detect)
to detect objects in the image feed from a camera.

### Services Provided

`/rotate` (`dnn_rotate/StringTrigger`): invokes `dnn_detect` to find the specified object, rotating as necessary.

### Services Called

`/dnn_detect/detect` (`dnn_detect/Detect`): requests `dnn_detect` to run on the next image.

### Publications

`/move_base/goal` (`move_base_msgs/MoveBaseActionGoal`): sends a movement goal

### Parameters

`~image_width`: Image width in pixels. Default `419`.

`~field_of_view`: Horizontal field of view in radians. Default `1.05`.

`~angle_increment`: Angle to rotate in radians if object is not found. Default `0.7`.

`~rotation_limit`: How many times to rotate `angle_increment` radians if the object is not found, before giving up. Default `8`.

`~forward_dist`: How far to move forward in meters when object is found. Default `0.2`.
`

### Example usage

    rosrun dnn_rotate rotate.py


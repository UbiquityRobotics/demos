

# Nodes

## sonar_wanderer.py

The sonar_wanderer uses 3 sonar detectors to drive around in a room.

It uses very simple logic so is likely to find ways to get confused but the purpose of this demo is to show how you can read the sonars and act on them to make decisions of how to move the robot with movement commands

The sonar id numbers are 0-5 where 1 is to the left 45 deg, 2 is to the right 45 deg and 3 is forward.
The measurements of Range are in meters from the sonar location itself.

If you are going to experiment with this I recommend setting up our suggested RF kill switch that plugs into our switch board.   Refer to `Making A Remote RF Estop Switch` on our page at  https://learn.ubiquityrobotics.com/mcb_pinouts_leds_userpower

### Paramaters

There are a few other parameters but the most obvious ones to play with are as follows

`detect_dist`: The distance out in front where we detect an object. A good default is about 0.8M.  The shorter this is the more likely it is the robot may get in trouble rotating in certain locations. (We adjust this for the right and left in the node)

`linear_rate`: The speed we drive forward. This is set to 0.2 M/sec to keep things safe.  (We adjust this for the right and left in the node)

`angular_rate`: The rotational speed in Radians/Sec for when we are turning away from an object.


### Publications

`cmd_vel`(geometry_msgs/Twist): commands to move the robot.

### Subscriptions

`sonars`:(sensor_msgs:Range)

### Example usage

    rosrun sonar_wanderer sonar_wanderer.py



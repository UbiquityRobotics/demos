
# Overview

Because navigation using a Lidar is a very popular and relatively easy mode for robots we are going to supply here some basic starter launch files and a little bit of directions for how to get started in robot navigation using a Lidar.

Our goal here is to put in one place the key elements of robot navigation using a very popular device, a LIDAR, to allow the robot to see walls all around and both map and then later navigate within a known mapped area.

The full system here could be studied and investigated by learning more about such things as ROS /tf topic and the lidar itself as well as how ROS understands the orientation of sensors in 3D space.   Plenty to learn but this is a known starting point as an example.

# The RPLidar Mechanical And USB Connectivity

We will use a relatively low cost and very popular Slamtec  RPLidar A1 that is connected to the Magni raspberry pi USB and will by default in most cases show up as serial device /dev/ttyUSB0.  

For these launch files the RPLidar is screwed to the top plate using 5mm spacers so the ribbon cable can bend around and allow their little USB board to be connected via USB cable to a port of the Raspberry Pi.

The center of the lidar is centered in Y and is thus half way between each wheel but of course on top of the Magni top plate.  The motor and pully is on the rear side of the lidar.

To modify this example for use of other Lidars is best done after you understand this demo but even reading this demo set of instructions will offer you many things to investigate and research to go as far as you like with your own robot hardware.   Below is the line that defines the location as we have mounted it in the picture called  RPLidar_MagniLidarDemoMounting.jpg

    arg name="lidar_translation" default="-0.03 0 0.20 0 3.14 3.14"

There are 6 floating point values.   First 3 are X,Y,Z translation in meters from our 'base_link' or a spot in space located directly between the centers of our two large main drive wheels. Z is not at all critical because the lidar only generates data in the 2D  plane of X and Y so don't worry about exact height of lidar.   The next 3 values will make your head spin to think much about them but in short they are the rotations in radians about the X,Y,Z axis.  You need to do your own research if you have the lidar in any other rotation than the picture. 

# ROS Configuration Required To Run these sets of demo launch files

Unless we later install these on our images at this time, late 2020, these installs are required.

    sudo apt update
    sudo apt install ros-kinetic-navigation
    sudo apt install ros-kinetic-slam-gmapping

After the above installs to be prepaired to run navigation code you will also need the driver for the SlamTec RPLidar.
There is a bug in how they setup the /scan topic ROS publisher so the ROS parameter of ```scan_topic_name``` does not work at the time of this writing, Oct 2020.  So I have edited the hard coded string they use as seen below before the make.

    cd ~/catkin_ws/src
    git clone https://github.com/sharp-rmf/rplidar_ros
    cd ~/catkin_ws
    vi ~/catkin_ws/src/rp_lidar_ros/src/node.cpp
    Edit to replace ```scan_topic_name``` with ```scan``` where ros::Publisher_scan_pub is setup 
    catkin_make
    

After the above steps you will need to decide on a location for the lidar and this location will have to be placed into the launch files below.   We are trying to keep things simple but there are ways in ros to have parameter files and so on and perhaps those may be added.   

For each launch file below open it up in an editor and be sure to make proper modifications as stated in the start of the launch file comments


## Setup a PC or Laptop That Is a Slave To Your Robot ROS 

To know that is going on you really need to have a laptop or other PC on the same network that is setup with ROS and configured to have your robot as the ROS_MASTER  for the laptop.

First you need to setup a laptop or workstation to have ROS Kinetic on it and then setup that machine to recognize the Magni robot as the ROS master.   This page will be of use if you have not already done that.

    https://learn.ubiquityrobotics.com/workstation_setup

Also involved is you have to connect to the Magni using ssh.  This means you have to be on the same network.  You have a couple choices.  Have the magni plugged into the same network with a cable till you get everything setup and can connect with ssh session  and then LATER connect to the Magni WiFi hotspot.   Both are explained below

    https://learn.ubiquityrobotics.com/connecting

Once the laptop is fully configured (a big job) OR you use our virtual image (sort of easier) we can move on to running the software. 


# Making Your Map

Place the magni where you would like to be the origin of the map which will be X,Y as 0,0.   Now you should totally restart the robot where one way is to do a  ```sudo shutdown -h now```    and then  a full power off and power back on.   The reason for this is that this is a certain way to set the MCB odom counters at 0,0 so when the magni-base service starts odometry will be reset.

A launch file example to start a lidar, the RPLIDAR A1, and make the system ready for gmapping is in this repository.

    roslaunch magni_lidar magni_lidar_mapmaker.launch

## Publications
Once the lidar is started, the /scan ROS topic will publish the lidar scan data.
You can verify it is generating data (although it is a great deal of data) using this test command

    rostopic echo /scan

Use Control-C to stop this onslaught of text!   we just wanted to see if the Lidar is running

    
## Running gmapping once the mapmaker launch file is running
    rosrun gmapping slam_gmapping scan:=scan

## Run RViz Back on The LapTop To Watch As Things Progress

On the laptop copy over lidar_mapmaker.rviz from this repository to your home directory so it can configure rviz easily.
Then on the laptop you can run this from home folder.

    rosrun rviz rviz -d lidar_mapmaker.rviz


##  Drive Around To Create A Suitable Map

Might be best to have a Joystick but if not you have to ssh to the robot and use 'twist' to drive around.
You would then drive around the area with the optional Logitech F710 joystick or 'twist' command below but perhaps start 

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py 


As the robot drives. you will be building more date for the map and you would see it on RVIZ

## SAVE THE MAP!
After your map looks ok you MUST SAVE THE MAP or your map is lost on stopping gmapping!    Here is the command to save the map

    rosrun map_server map_saver -f mynewmap-ils

We suggest you move both the .pgm and the .yaml files into magni_lidar/maps so they can be found and used easily



# Driving Around Within A Pre-Created Map

Once a map is available you can then navigate within that map or set of rooms.   This is that you have been waiting for frankly!     The idea here is the launch file publishes onto ROS the previously made map and then you either drive around using only robot odometry OR you use some very advanced software called AMCL or Adaptive Monte Carlo Locationization figures out where the robot is at any given time.  Both methods will be shown in this section.

A key piece of software used in this simple example is the move_basic package that will accept commands to go places and talk to the robot navigation stack to drive the robot to the destination.  The move_basic package is unique to Ubiquity Robotics and is a simplified version for just point to point movements based on the more advanced move_base concepts.  The ROS move_base package does path planning as well as object avoidance if the system has been setup for detection of things like a person or object getting in the way of the robot.

You then can use  RViz on your laptop (described in mapping example) and can define a pose that you want the robot to move to.   A 'Pose' means a specific location in terms of X and Y on the floor as well as the rotation of the robot.  
Lets GO!

## Start Up Most Of The Navigation Stack Using The Map You have Saved

Here we need to start the launch file and specify a map that will be used for navigation in whatever room or area you are in that has previously been mapped using gmapping and saved as a map.
Edit magni_lidar_maprunner.launch to set the desired map.  We supply a tinyroom.map as an example but this is just a small square area and unless you duplicate it exactly this will not work for you.  It was about 1.9M x 1.5M if you have a bunch of cardboard you could start doing navigation without the making of the map part of this demo  

    roslaunch magni_lidar magni_lidar_maprunner.launch  

This launch file will strictly respect the odom information the robot keeps track of to determine robot position and rotation (called robot ```pose```).   The problem with this method is all robot odom only determination of pose drifts over time the more movements that take place.  So this method is ok for a short demo but not very usable in general real world situations.

## Start Up move_basic which will allow Path Planning and autonomous movement

So far we have setup things so the robot knows where it is within a map.   We need to start some software that we can tell where we want to move to so that that piece of software can control the robot to approach a desired destination X,Y and rotation (both of these things together are called a desired ```pose```.    We will use rviz but we must start this piece of software called move_base now.   Ubiquity Robotics ```move_basic``` is a simplified version of move_base where move_base can do complex plans to get around objects or corners.   The move_basic package can only do line of sight straight paths and if something gets in the way it stops and does not plan around the object.

    roslaunch magni_nav move_basic.launch

We will now be ready to accept 'goals' and then move to those goals.

The more general solution to navigation uses object avoidance and the move_base package combined with a map that holds obstacles seen by sensors such as the sonar.  The objects that move in fron are in what is called the ```costmap```.  Perhaps that will be added to this example in the future.

## Run RViz Back on The LapTop To Watch As Things Progress

On the laptop copy over lidar_mapmaker.rviz from this repository to your home directory so it can configure rviz easily.
Then on the laptop you can run this from home folder.

    rosrun rviz rviz -d lidar_mapmaker.rviz


## Tell The Robot Where It Is Initially Located In The Map

The AMCL package is now told just where the robot is within the map and which direction it is pointing.  This step allows the AMCL package to have a very good initial pose for the robot so it can estimate how to get to other locations right away. 

Set the 2D Pose Estimate using the RViz interface as follows:

    - Left click the ```2D Pose Estimate` button in RViz 
    - Find the place the robot is in the map and left click on that spot but HOLD MOUSE BUTTON
    - A large green arrow will appear and you have to drag mouse so arrow points same angle as the robot
    - Release the mouse button and then the robot will have a very good estimate of it's pose
    
## Define a Target Pose And Let The robot Go there

Here is the really fun part, assuming all the other things are working.  This is where you tell the robot to move to places in the map. Because we are using the simple move_basic planner be sure the path is clear to the destination.

The 'pose' of a floor based robot is both it's X and Y location as well as the rotation on the floor.  That is what you will define.

I will attempt to explain in words how to define a goal for the robot.  Basically we want to form a 2D Goal for the robot that is at some location and indicates the direction we want the robot to be facing when the goal is reached

    - Identify a place you want the robot to drive to
    - Click on the left mouse button on that spot and HOLD MOUSE KEY DOWN because an arrow will show up
    - Move around the mouse so the arrow points in the direction the robot will face when done
    - Release the left mouse button

If the gods are with you the robot will approach that spot and rotate to the direction you specified.


## Running AMCL to dynamically correct the robot location

The most common package that figures out where the robot it in the room (relative to the map made before) is called AMCL.  It uses an adaptive Monte Carlo method to find the location of the robot at any given time.

Run the launch file below to use AMCL.  To use AMCL in any real world situation which may have a complex map you first have to use RViz to tell the robot where it is in the map.  

    roslaunch magni_lidar magni_lidar_maprunner_amcl.launch  

More things have to be all working nicely for reasonable results using AMCL so we suggest you get the odom only running first and only after that works move on to this example using AMCL.

The AMCL package figures out the map to odom transform and publishes that which in effect corrects the errors that build up on all robot self posting of odom frame.  The earlier launch file had a static transform that published that the map to odom transform was 'zero' from map to odom frames. This launch file removes that static transform and lets the AMCL package publish the map to odom transform or ```tf```. 


## A Word About move_base avoids Dynamic Objects That May Move

We are not using the more powerful move_base in this example (so far) to keep things as simple as possible.   If you choose to use move_base the planing is much more advanced but of course more things can then go wrong.
ROS navigation stack can use detection of things that get in the way of the robot to then alter the path the robot was taking to the destination.   This is only mentioned here at this time as so far this simple demo does not use this feature.     Sensors such as sonar sensors can be used to detect things and place them in something called a ```costmap```.  The costmap can change as things move into and out of the path of the robot like your dog or cat or even yourself.

The move_base code can dynamically re-think the path to be taken to avoid objects in the costmap.

    
## A word about the cruelty of the Real World

There are a great many setting so make robot navigation work well AND your robot is required to have excellent odometry to make less errors in navigation.  So what you will find is after moving a couple times there may be buildup of errors and the robot is NOT where it should be.   We will work on parameters to tweek the accuracy.

# Conclusion - The Real World Is not Ideal in the land of robotics

The purpose of this entire demos is to present you with the concepts of map making and navigation.
Do not expect excellent performance, that takes a great deal of tweeking and effort not discussed here.

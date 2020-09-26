
# Overview

Because navigation using a Lidar is a very popular and relatively easy mode for robots we are going to supply here some basic starter launch files and a little bit of directions for how to get started in robot navigation using a Lidar.

We will use a relatively low cost and very popular Slamtec  RPLidar A1 that is connected to the Magni raspberry pi USB and will by default in most cases show up as serial device /dev/ttyUSB0

For these files the RPLidar is screwed to the top plate using 5mm spacers so the ribbon cable can bend around and allow their little USB board to be connected via USB cable to a port of the Raspberry Pi.

The center of the lidar is centered in Y and is thus half way between each wheel but of course on top of the Magni top plate.  the motor and pully is on the rear side of the lidar.

# ROS Configuration Required To Run these sets of demo launch files

Unless we later install these on our images at this time, late 2020, these installs are required.

    sudo apt update
    sudo apt install ros-kinetic-navigation
    sudo apt install ros-kinetic-slam-gmapping

After the above installs to be prepaired to run navigation code you will also need the driver for the SlamTec RPLidar

    cd ~/catkin_ws/src
    git clone https://github.com/sharp-rmf/rplidar_ros
    cd ~/catkin_ws
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

    roslaunch magni_rplidar_nav magni_lidar_mapmaker.launch

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

THIS SECTION IS YET TO BE SUPPORTED FOR LACK OF LAUNCH FILES TO BE COMPLETED but it involves use of  AMCL  software to find robot position as well as setting goals to be driven to in RVIZ or other ways. 


Once a map is available you can then navigate within that map or set of rooms.   This is that you have been waiting for frankly!     The idea here is the launch file publishes onto ROS the previously made map and then some very advanced software called   AMCL or Adaptive Monte Carlo Locationization figures out where the robot is at any given time.  

You then can use  RViz on your laptop (described in mapping example) and can define a pose that you want the robot to move to.   A 'Pose' means a specific location in terms of X and Y on the floor as well as the rotation of the robot.  

Lets GO!

## Start Up Most Of The Navigation Stack Using The Map You have Saved

Here we need to start the launch file and specify a map that will be used for navigation in whatever room or area you are in that has previously been mapped using gmapping and saved as a map.
Edit magni_lidar_maprunner.launch to set the desired map.  We supply a tinyroom.map as an example but this is just a small square area and unless you duplicate it exactly this will not work for you.  It was about 1.9M x 1.5M if you have a bunch of cardboard you could start doing navigation without the making of the map part of this demo

    roslaunch magni_lidar magni_lidar_maprunnier.launch  

## Start Up move_base which will allow Path Planning and autonomous movement

So far we have setup things so the robot knows where it is within a map.   We need to start some software that we can tell where we want to move to so that that piece of software can control the robot to approach a desired destination X,Y and rotation (both of these things together are called a desired ```pose```.    We will use rviz but we must start this piece of software called move_base now.   Ubiquity Robotics has a similar package called  ```move_basic```.

    roslaunch magni_nav move_base.launch

We will now be ready to accept 'goals' and then move to those goals.

## Run RViz Back on The LapTop To Watch As Things Progress

On the laptop copy over lidar_mapmaker.rviz from this repository to your home directory so it can configure rviz easily.
Then on the laptop you can run this from home folder.

    rosrun rviz rviz -d lidar_mapmaker.rviz


## Define a Target Pose And Let The robot Go there

Here is the really fun part, assuming all the other things are working.  This is where you tell the robot to move to places in the map.

The 'pose' of a floor based robot is both it's X and Y location as well as the rotation on the floor.  That is what you will define.

I will attempt to explain in words how to define a goal for the robot.  Basically we want to form a 2D Goal for the robot that is at some location and indicates the direction we want the robot to be facing when the goal is reached

    Identify a place you want the robot to drive to
    Click on the left mouse button on that spot and HOLD MOUSE KEY DOWN because an arrow will show up
    Move around the mouse so the arrow points in the direction the robot will face when done
    Release the left mouse button

If the gods are with you the robot will approach that spot and rotate to the direction you specified.

    
## A word about the cruelty of the Real World

There are a great many setting so make robot navigation work well AND your robot is required to have excellent odometry to make less errors in navigation.  So what you will find is after moving a couple times there may be buildup of errors and the robot is NOT where it should be.   We will work on parameters to tweek the accuracy.

# Conclusion - The Real World Is not Ideal in the land of robotics

The purpose of this entire demos is to present you with the concepts of map making and navigation.
Do not expect excellent performance, that takes a great deal of tweeking and effort not discussed here.

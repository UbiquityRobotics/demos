
# Overview

Because navigation using a Lidar is a very popular and relatively easy mode for robots we are going to supply here some basic starter launch files and a little bit of directions for how to get started in robot navigation using a Lidar.

We will use a relatively low cost and very popular Slamtec  RPLidar A1 that is connected to the Magni raspberry pi USB and will by default in most cases show up as serial device /dev/ttyUSB0

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

A launch file example to start a lidar, the RPLIDAR A1, and make the system ready for gmapping is in this repository.

    roslaunch magni_rplidar_nav magni_lidar_mapmaker.launch

## Publications
Once the lidar is started, the /scan ROS topic will publish the lidar scan data.
You can verify it is generating data (although it is a great deal of data) using this test command

    rostopic echo /scan

Use Control-C to stop this onslaught of text!   we just wanted to see if the Lidar is running

    
## Running gmapping once the mapmaker launch file is running
    rosrun gmapping slam_gmapping scan:=scan

## Back on The LapTop you can run RViz To Watch Map Building

This will require RViz to have a configureation (we hope to supply later in this repository) that will allow you to see the map and laser scan.

You would then drive around the area with the optional Logitech F710 joystick or 'twist' command below

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py 



## Drive Around To Complete A Map
  
Use the robot joystick if you have it or use this twist command to drive around and as you do so the map will be building more date and you would see it on RVIZ

## SAVE THE MAP!
After your map looks ok you MUST SAVE THE MAP or your map is lost on stopping gmapping!    Here is the command to save the map

    rosrun map_server map_saver -f mynewmap-ils



# Driving Around Within A Pre-Created Map

THIS SECTION IS YET TO BE COMPLETED but it involves use of  AMCL  software to find robot position as well as setting goals to be driven to in RVIZ or other ways. 

    

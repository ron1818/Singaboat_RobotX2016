Singaboat RobotX2016
====================

Introduction
------------
this repository hosts all the files for [robotx challenge 2016](http://www.robotx.org)
it is a ROS based projects

Usage
-----
### Installation ###
+ supports **ros-indigo** with **ubuntu 14.04LTS**
+ must install `ros-indigo-desktop-full`
+ dependencies are `robot-localization` `nmea-navsat-driver` `viso2` `gazebo`
+ 
#### script to install additional packages ####
```bash
sudo apt-get install ros-indigo-gazebo*
sudo apt-get install ros-indigo-amcl ros-indigo-gmapping ros-indigo-move-base ros-indigo-map*
sudo apt-get install ros-indigo-robot-localization ros-indigo-nmea-navsat-driver
sudo apt-get install ros-indigo-stereo-image* ros-indigo-viso2*
```

### Team work ###
+ please create pull request for each task, such as GPS, IMU, Video, etc.
+ do not create any files to the master branch, the maintainer will do the merge.


1. fork this respository to your github account
2. clone your forked respository to your catkin workspace
3. create branch and do your work
4. create pull request if you want to contribute to the main branch

### Directories ###

naming convention: **robotx_**

suggested directories: `bringup`, `gazebo`, `rviz`, `control`, `nav`, `msg`, `vision`, etc.

non-ROS based projects, you can create them in `arduino`, `document`, `misc` directory.



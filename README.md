Singaboat RobotX2016
====================
###changelog###
+ (2017-01-17): leave ERI@N, tidy up the readme markdown file
+ (2016-12-15): competition in progress
+ (2016-09-30): initialize the repository

Introduction
------------
this repository hosts all the files for [robotx challenge 2016](http://www.robotx.org)
it is a ROS based project

### overview ###
![topdown list](Documents/Screenshot%20from%202016-09-22%2011:38:49.png)

### Installation ###
supports **ros-indigo** with **ubuntu 14.04LTS**

#### fork repository ####
[create](https://help.github.com/categories/collaborating-with-issues-and-pull-requests/)
[pull request](https://help.github.com/articles/creating-a-pull-request-from-a-fork/)
[remote](https://help.github.com/articles/creating-a-pull-request-from-a-fork/)
[sync](https://help.github.com/articles/syncing-a-fork/)

useful commands:
`git clone`, `git fetch`, `git remote -v`, `git remote add upstream`
`git checkout xxx`, `git fetch upstream`, `git merge upstream/xxx`
#### clone repository ####
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.com/ron1818/Singaboat_RobotX2016
```

#### script to install packages ####
```bash
#### core modules ####
sudo apt-get install ros-indigo-desktop-full
sudo apt-get install ros-indigo-gazebo* \
ros-indigo-amcl ros-indigo-gmapping ros-indigo-move-base ros-indigo-map* \
ros-indigo-robot-localization ros-indigo-nmea-navsat-driver \
ros-indigo-stereo-image*
#### install python and opencv ####
sudo apt-get install python-numpy python-scipy python-serial \
libopencv-dev python-opencv ros-indigo-vision-opencv
#### other ros modules ####
sudo apt-get install ros-indigo-turtlebot-bringup \
ros-indigo-turtlebot-create-desktop ros-indigo-openni-* \
ros-indigo-openni2-* ros-indigo-freenect-* \
ros-indigo-laser-* ros-indigo-hokuyo-node \
ros-indigo-audio-common gstreamer0.10-pocketsphinx \
ros-indigo-pocketsphinx ros-indigo-slam-gmapping \
ros-indigo-joystick-drivers python-rosinstall \
ros-indigo-orocos-kdl ros-indigo-python-orocos-kdl \
python-setuptools ros-indigo-dynamixel-motor-* \
ros-indigo-depthimage-to-laserscan ros-indigo-arbotix-* \
ros-indigo-turtlebot-teleop ros-indigo-move-base \
ros-indigo-map-server ros-indigo-fake-localization ros-indigo-hector* \
ros-indigo-gazebo-ros* ros-indigo-serial
sudo apt-get install ros-indigo-myahrs-driver
#### version control ####
sudo apt-get install git subversion mercurial
```

#### install other packages from source ####
```bash
cd ~/catkin_ws/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git
git clone https://github.com/srv/viso2
git clone https://github.com/KristofRobot/razor_imu_9dof
git clone https://github.com/ron1818/nmea_navsat_driver
git clone https://github.com/ron1818/ROS-OSM-map-integration
cd ~/catkin_ws
catkin_make
rospack profile
```

#### update openCV 2.4.13.1 ####
``` bash
cd
sudo apt-get install v4l2ucp v4l-utils libv4l-dev
git clone https://github.com/opencv/opencv
cd opencv
git checkout 2.4
mkdir release
cd release
sudo cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
sudo make
sudo make install
```

### Team work ###
+ please create pull request for each task, such as GPS, IMU, Video, etc.
+ do not create any files to the master branch, the maintainer will do the merge.

1. fork this respository to your github account
2. clone your forked respository to your catkin workspace
3. create branch and do your work
4. create pull request if you want to contribute to the main branch

Usage
-----

### PVC prototype ###

### kayak prototype ###

### wamv competition boat ###


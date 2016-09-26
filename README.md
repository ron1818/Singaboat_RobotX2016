Singaboat RobotX2016
====================

TASKS
-----
### overview ###
![topdown list](Documents/Screenshot%20from%202016-09-22%2011:38:49.png)

### 2016-09-19 ~~ 2016-09-23 ###
1. yellow computer repartition and install **finished**
2. usb devices [udev rules](http://askubuntu.com/questions/670197/udev-rules-targeting-every-usb-device)
or follow [linorobot](https://linorobot.org/getting-started/)
3. integrate GPS with `nmea_navsat_driver` and `navsat_transform`, publish `/navsat/fix` as `sensor_msgs/NavSatFix` and `navsat/vel`
4. integrate IMU with customized code (suggest to buy newer version so that we can use `razor-imu-9dof`)
5. `cmd_vel` must be written and can control the motors

### 2016-09-24 ~~ 2016-09-30 ###
1. opencv on color detection and object detection, use `cv2.Canny` for edge detection and `cv2.threshold` on HSV color detection
2. train cascade classifiers for circle, triangle and cross and apply cascade classifier
3. calibrate camera with checkboard
4. dry run and prepare for water test

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

#### clone repository ####
```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
catkin_init_workspace
git clone https://github.com/xxx/Singaboat_RobotX2016
```

#### script to install additional packages ####
```bash
sudo apt-get install ros-indigo-gazebo* \
ros-indigo-amcl ros-indigo-gmapping ros-indigo-move-base ros-indigo-map* \
ros-indigo-robot-localization ros-indigo-nmea-navsat-driver \
ros-indigo-stereo-image* ros-indigo-viso2*
```

#### install python and opencv ####
```bash
sudo apt-get install python-numpy python-scipy python-serial
sudo apt-get install ros-indigo-turtlebot-bringup \
ros-indigo-turtlebot-create-desktop ros-indigo-openni-* \
ros-indigo-openni2-* ros-indigo-freenect-* ros-indigo-usb-cam \
ros-indigo-laser-* ros-indigo-hokuyo-node \
ros-indigo-audio-common gstreamer0.10-pocketsphinx \
ros-indigo-pocketsphinx ros-indigo-slam-gmapping \
ros-indigo-joystick-drivers python-rosinstall \
ros-indigo-orocos-kdl ros-indigo-python-orocos-kdl \
python-setuptools ros-indigo-dynamixel-motor-* \
libopencv-dev python-opencv ros-indigo-vision-opencv \
ros-indigo-depthimage-to-laserscan ros-indigo-arbotix-* \
ros-indigo-turtlebot-teleop ros-indigo-move-base \
ros-indigo-map-server ros-indigo-fake-localization ros-indigo-hector* \
ros-indigo-amcl git subversion mercurial

cd ~/catkin_ws/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git
git clone https://github.com/srv/viso2
cd ~/catkin_ws
catkin_make
rospack profile
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

## Launch the program ##
### gazebo with robot localization ###
```bash
roslaunch robotx_gazebo robotx_sim.launch # launch gazebo with robot_localization
roslaunch robotx_bringup vo_stereo.launch # camera visual odometry
roslaunch robotx_nav move_base_with_blank_map.launch # map server
```

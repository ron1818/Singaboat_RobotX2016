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

Directory
---------
+ Documents: contain documentations for the work such as how to, scripts for VIM, etc.
+ bringup: toplevel launch files to bringup the robot's topics and tf
+ control: mainly `robot localization` and `rosserial` for arduino to drive the thrusters
+ description: urdf for the `robot state publisher` and gazebo for `gazebo`
+ gazebo: gazebo simulation files, incluiding launch, world, and test objects
+ gui: for robotx competition tasks 4 and 5, report result to judge
+ nav: `move base` related files, including toplevel behaviors
+ rviz: `rviz` configuration files
+ sensor: drivers for all sensors: GPS, IMU, camera, water speed sensor, wind sensor, lidar, etc.
+ vision: `opencv` based scripts for image recognition
+ sketch: arduino source codes

Branches
--------
+ master: upto date codes
+ pvc-dev: PVC prototype development, freezed by end of Jan 2017
+ kayak-dev: kayak prototype development
+ wamv-dev: competition development, freezed by end of Dec 2016

Usage
-----

### PVC prototype ###

#### PVC in gazebo ###
```bash
roslaunch robotx_gazebo pvc_empty_world.launch
# launch rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/move_base.rviz
```

for movebase behaviors, use wamv's code

#### real PVC ####
```bash
# on the boat computer, raspberry pi
roslaunch robotx_bringup pvc_minimal.launch
roslaunch robotx_bringup pvc_move_base.launch mapname:=nanyanglake

# on the base computer
roslaunch robotx_bringup wamv_teleop.launch
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/move_base.rviz

```

### kayak prototype ###

### wamv competition boat ###

#### fake wamv ####
bring up fake wamv to do pure tf simulation in rviz:
```bash
roslaunch robotx_bringup fake_wamv.launch
# arbotix driver is to drive the fake model in rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/fake_move_base.rviz
```

bring up movebase to test the function
```bash
# run your behavior file, makesure *.py is executable< chmod +x *.py
rosrun robotx_nav move_base_loiter.py _radius:=5 _polygon:=6 _ccw:=true
rosrun robotx_nav move_base_forward.py 
rosrun robotx_nav move_base_scout.py
rosrun robotx_nav move_base_zigzag.py
# OR
roslaunch robotx_nav loiter_behavior.launch isfake:=true
roslaunch robotx_nav constant_heading_behavior.launch isfake:=true
roslaunch robotx_nav scout_behavior.launch isfake:=true
roslaunch robotx_nav zigzag_behavior.launch isfake:=true
roslaunch robotx_nav reverse_behavior.launch isfake:=true
roslaunch robotx_nav rotation_behavior.launch isfake:=true
roslaunch robotx_nav station_keep_behavior.launch isfake:=true
```

#### wamv in gazebo ####
test whether the model is correct:
```bash
# convert urdf.xacro to urdf
rosrun xacro xacro.py xxx.xacro > xxx.urdf
# open gazebo with an empty world
roslaunch gazebo_ros empty_world.launch
# spawn a model into the world, e.g. dock
rosrun gazebo_ros spawn_model -file `rospack find robotx_description`/urdf/dock.urdf \
-urdf -x 0 -y 0 -z 1 -model dock
```

with launch file
```bash
roslaunch robotx_gazebo robotx_test.launch test1:=true
# in total eight tests, to enable, set them to true, e.g. to use test5
roslaunch robotx_gazebo robotx_test.launch test5:=true
# or to have a blank map:
roslaunch robotx_gazebo robotx_test.launch
# launch rviz
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/move_base.rviz
```

move base for control
```bash
# launch movebase, you can change the mapname by: hawaii or blank_map
roslaunch robotx_nav move_base_map.launch isgazebo:=true mapname:=pandan
# launch individual behavior, you can check one by one
roslaunch robotx_nav loiter_behavior.launch isgazebo:=true is_relative:=true
roslaunch robotx_nav constant_heading_behavior.launch isgazebo:=true is_relative:=true
roslaunch robotx_nav zigzag_behavior.launch isgazebo:=true
roslaunch robotx_nav reverse_behavior.launch isgazebo:=true
roslaunch robotx_nav rotation_behavior.launch isgazebo:=true
roslaunch robotx_nav station_keep_behavior.launch isgazebo:=true
# launch toplevel behavior
roslaunch robotx_nav test1_behavior.launch isgazebo:=true
```

#### real wamv ####
```bash
# not run
# # check gps and imu
# roslaunch robotx_sensor gps_serial.launch
# rostopic echo /navsat/fix
# roslaunch robotx_sensor imu_razor_pub.launch
# roslaunch razor-imu-9dof razor-display.launch

# on the boat computer, intel NUC
roslaunch robotx_bringup wamv_minimal.launch
roslaunch robotx_bringup wamv_move_base.launch mapname:=blank_map

# on the base computer
roslaunch robotx_bringup wamv_teleop.launch
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/move_base.rviz
# # optionally record bag
# rosbag record -a
```

movebase on the base computer
```bash
# launch individual behavior, you can check one by one
roslaunch robotx_nav loiter_behavior.launch iswamv:=true is_relative:=false
roslaunch robotx_nav constant_heading_behavior.launch iswamv:=true
roslaunch robotx_nav scout_behavior.launch iswamv:=true
roslaunch robotx_nav zigzag_behavior.launch iswamv:=true
roslaunch robotx_nav reverse_behavior.launch iswamv:=true
roslaunch robotx_nav rotation_behavior.launch iswamv:=true
roslaunch robotx_nav station_keep_behavior.launch iswamv:=true
```

#### replay bag files ####
```bash
# start roscore
roscore
# play back data, ignore tf and tfstatic
rosbag play xxx.bag tf:=fakeft tfstatic:=faketfstatic
# call robot localization
roslaunch robotx_control bag_r_l_control.launch
# view in rviz, specially configured
rosrun rviz rviz -d `rospack find robotx_rviz`/rviz/bag.rviz
# optionally can view gps tracks on a OSM map
roslaunch robotx_sensor gps_osm_viewer.launch
# open a browser that target to: file:///home/xxx/catkin_ws/src/ROS-OSM-map-integration/index.html
```



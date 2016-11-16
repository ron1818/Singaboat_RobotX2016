INSTALL OPENCV
--------------
```bash
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

COMMANDS
--------

### Camshift ###
#### launch gazebo camera and camshift ####
```bash
# launch gazebo with test 1
roslaunch robotx_gazebo robotx_test.launch
roslaunch robotx_vision camshift.launch namespace:=/bow_stereo/left input_rgb_image:=image_raw
# track left camera on red
roslaunch robotx_vision camshift.launch namespace:=/bow_stereo/right input_rgb_image:=image_raw
# track right camera on green
```
#### launch gazebo camera and camshift with auto color detection ####
```bash
# launch gazebo with test 1
roslaunch robotx_gazebo robotx_test.launch
roslaunch robotx_vision camshift_color.launch namespace:=/bow_stereo/left input_rgb_image:=image_raw color_under_detect:=red
# track left camera on red
roslaunch robotx_vision camshift_color.launch namespace:=/bow_stereo/right input_rgb_image:=image_raw color_under_detect:=green
# track right camera on green
```

#### logitech camera and camshift ####
```bash
# launch webcam
roslaunch robotx_vision usb_cam.launch
# do rectification
ROS_NAMESPACE=/camera/rgb rosrun image_proc image_proc
# launch camshift
roslaunch robotx_vision camshift.launch namespace:=/camera/rgb input_rgb_image:=image_rect_color
# or 
roslaunch robotx_vision camshift_color.launch namespace:=/camera/rgb input_rgb_image:=image_rect_color color_under_detect:=red
```

### feature matching ###
must first install opencv 2.4.13.1 from source (github),
follow opencv website for more info

OBJECT DETECTION
--------
### 13/11/2016 ###
1. Add `underwater` node for Coral Survey task.
2. Scan the code sequence will be updated through parameter server `/gui/color1`, `/gui/color2`, `/gui/color3`.
3. Add function to detect white totem marker.
4. Tweak update.

### 03/11/2016 ###
1. Update algoritm of `object_detector` & `symbol_identifier`
2. Add node `sequence_scanner` and launch file

### 30/10/2016 ###
object detected will be published as a custom message type (msg/object_detection.msg)
#### naming convetion of custom message ####
1. `frame_id`  camera, using convetional naming
2. `type`      object's type: `marker_totem`, `tiki_totem`, `obstacle`, `triangle`, `circle`, `cruciform`
3. `color`     object's color: `red`, `green`, `blue`, `black`, `white`
4. `angle_t`   top angle of detected object on image
5. `angle_b`   bottom angle of detected object on image
6. `angle_l`   left angle of detected object on image
7. `angle_r`   right angle of detected object on image

# launch node
paramater need to be adjusted depends on the subscribed camera's topic
```bash
catkin_make
roslaunch robotx_vision [node_of_interest].launch
``` 
